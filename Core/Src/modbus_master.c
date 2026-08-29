/*
 * modbus_master.c
 *
 *  Created on: Jul 11, 2026
 *      Author: tomas
 */

/**
 * @file modbus_master.c
 *
 * @brief Hardware-specific Modbus RTU master implementation.
 *
 * Connects the protocol layer (modbus_protocol.c, which knows only about
 * raw frames) with the STM32 peripherals: USART1 for Modbus/RS-485,
 * DE_RE_Output for transceiver direction control. Porting to another UART
 * or transceiver only touches this file; the protocol layer stays the same.
 */

#include "modbus_master.h"
#include "usart.h"
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"

#define MODBUS_TC_TIMEOUT_MS 20U

/*
 * Internal receive state of the Modbus master, private to this module.
 * Higher layers only see modbus_master_poll()'s status code and copied
 * response data, never these buffers directly.
 *
 * modbus_rx_size is volatile because it's written from the UART callback
 * (ISR context) and read from task code.
 */
static volatile uint16_t modbus_rx_size = 0;

/*
 * DMA keeps writing into modbus_rx_buffer for as long as USART1 is
 * receiving, including the next frame - the moment modbus_rx_event() below
 * re-arms it, this buffer is fair game for DMA again. modbus_rx_snapshot is
 * the copy the task actually reads: modbus_rx_event() fills it before
 * re-arming DMA, so the task always has a frame that nothing else is
 * touching, even if a second frame starts arriving while it's still being
 * validated.
 */
static uint8_t modbus_rx_buffer[256];
static uint8_t modbus_rx_snapshot[256];

/*
 * Signals "a response frame just arrived" from modbus_rx_event() (ISR) to
 * modbus_master_poll() (task). A semaphore is used here instead of polling
 * a flag so the task can actually block while it waits, instead of sitting
 * in a busy loop burning CPU.
 */
static SemaphoreHandle_t modbus_rx_semaphore;
static StaticSemaphore_t modbus_rx_semaphore_buffer;

/**
 * @brief Start asynchronous DMA reception for Modbus responses.
 *
 * @return
 *         MODBUS_OK   DMA reception was started successfully.
 *         MODBUS_ERR_HAL HAL failed to start reception.
 */
ModbusStatus_t modbus_master_init(void) {

	modbus_rx_semaphore = xSemaphoreCreateBinaryStatic(
			&modbus_rx_semaphore_buffer);

	HAL_StatusTypeDef hal_status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
			modbus_rx_buffer, sizeof(modbus_rx_buffer));

	if (hal_status != HAL_OK) {
		return MODBUS_ERR_HAL;
	}

	return MODBUS_OK;
}

/**
 * @brief Send one Modbus request frame over RS-485.
 *
 * Half-duplex RS-485 requires the transceiver direction to switch back to
 * receive only after transmission has *physically* finished. DMA-complete
 * only means DMA finished feeding bytes to the UART peripheral, not that
 * the last bit has left the shift register - switching DE/RE on DMA
 * completion would risk cutting off the end of the frame. That is why this
 * function waits on the UART TC flag instead.
 *
 * @param modbus_tx_buffer Buffer containing the complete Modbus request.
 * @param modbus_tx_buffer_size Number of bytes to transmit.
 *
 * @return
 *         MODBUS_OK      Transmission completed successfully.
 *         MODBUS_ERR_HAL HAL reported an error while starting DMA.
 */
static ModbusStatus_t modbus_send_request(uint8_t *modbus_tx_buffer,
		uint8_t modbus_tx_buffer_size) {

	HAL_StatusTypeDef hal_status;

	/*
	 * TC may still be set from a previous transmission; clear it first or
	 * the wait loop below could see a stale TC=1 and release the bus early.
	 */
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);

	HAL_GPIO_WritePin(
	DE_RE_Output_GPIO_Port,
	DE_RE_Output_Pin, GPIO_PIN_SET);

	hal_status = HAL_UART_Transmit_DMA(&huart1, modbus_tx_buffer,
			modbus_tx_buffer_size);

	if (hal_status != HAL_OK) {

		/*
		 * Nothing to wait for - return the bus to receive mode now,
		 * otherwise it would remain stuck in transmit mode.
		 */
		HAL_GPIO_WritePin(
		DE_RE_Output_GPIO_Port,
		DE_RE_Output_Pin, GPIO_PIN_RESET);

		return MODBUS_ERR_HAL;
	}

	uint32_t tc_wait_start = HAL_GetTick();

	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {

		if (HAL_GetTick() - tc_wait_start >= MODBUS_TC_TIMEOUT_MS) {

			/*
			 * Transmission never physically finished - return the bus to
			 * receive mode so it doesn't stay stuck in transmit mode, and
			 * report the failure instead of hanging forever.
			 */
			HAL_GPIO_WritePin(
			DE_RE_Output_GPIO_Port,
			DE_RE_Output_Pin, GPIO_PIN_RESET);

			return MODBUS_ERR_HAL;

		}
	}

	HAL_GPIO_WritePin(
	DE_RE_Output_GPIO_Port,
	DE_RE_Output_Pin, GPIO_PIN_RESET);

	return MODBUS_OK;
}

/**
 * @brief Send a Modbus request and wait for a valid response.
 *
 * One complete transaction: transmit -> wait for response -> validate ->
 * retry if the error is retryable. A Modbus exception response is not
 * retried, since it is a logical answer from the slave rather than a
 * transmission problem, and resending would normally not change it.
 *
 * @param modbus_tx_buffer Request frame.
 * @param modbus_tx_buffer_size Request frame length.
 * @param modbus_rx_data Destination buffer for a valid response.
 * @param modbus_target Expected slave/function/register configuration.
 * @param modbus_retry_policy Timeout and retry configuration.
 *
 * @return Final status of the transaction.
 */
ModbusStatus_t modbus_master_poll(uint8_t *modbus_tx_buffer,
		uint8_t modbus_tx_buffer_size, uint8_t *modbus_rx_data,
		const ModbusTarget_t *modbus_target,
		const ModbusRetryPolicy_t *modbus_retry_policy) {

	ModbusStatus_t status;
	ModbusStatus_t final_status = MODBUS_ERR_TIMEOUT;

	/*
	 * Take with a zero timeout just to clear out any stale "give" left over
	 * from before this transaction - e.g. a frame that arrived just after
	 * the previous poll gave up waiting. Without this, the very first
	 * xSemaphoreTake() below could immediately return with old data instead
	 * of actually waiting for the response to the request we're about to
	 * send.
	 */
	xSemaphoreTake(modbus_rx_semaphore, 0);

	for (uint8_t attempt = 0; attempt < modbus_retry_policy->max_attempts;
			attempt++) {

		ModbusStatus_t attempt_status = MODBUS_ERR_TIMEOUT;

		status = modbus_send_request(modbus_tx_buffer, modbus_tx_buffer_size);

		/*
		 * A HAL-level send failure is a local hardware/driver problem,
		 * not a bad Modbus response - nothing to retry here.
		 */
		if (status != MODBUS_OK) {
			return status;
		}

		/*
		 * The task actually sleeps here instead of spinning - it stays
		 * Blocked until either modbus_rx_event() gives the semaphore from
		 * the UART callback, or response_timeout_ms runs out.
		 */
		if (xSemaphoreTake(modbus_rx_semaphore,
				pdMS_TO_TICKS(modbus_retry_policy->response_timeout_ms))
				== pdTRUE) {

			attempt_status = modbus_validate_frame(modbus_rx_snapshot,
					modbus_rx_size, modbus_target->slave_id,
					modbus_target->function_code,
					modbus_target->register_count_lo);
		}

		/*
		 * Must be stored on every attempt: the final result has to reflect
		 * the LAST attempt, including a timeout that follows an earlier
		 * CRC error.
		 */
		final_status = attempt_status;

		if (attempt_status == MODBUS_OK) {

			/*
			 * modbus_rx_buffer keeps getting re-armed by the receive
			 * callback, so callers work on this stable copy instead of the
			 * DMA buffer directly.
			 */
			memcpy(modbus_rx_data, modbus_rx_snapshot, modbus_rx_size);

			break;
		}

		if (attempt_status == MODBUS_ERR_EXCEPTION) {
			break;
		}

		/*
		 * Same reasoning as the take at the top of this function: clear out
		 * the semaphore before the next attempt, in case a late or
		 * unrelated frame gives it right after this attempt's timeout.
		 */
		xSemaphoreTake(modbus_rx_semaphore, 0);
	}

	return final_status;
}

/**
 * @brief Handle a completed UART reception detected by ReceiveToIdle_DMA.
 *
 * HAL_UART_RXEVENT_IDLE marks the end of the current Modbus RTU response
 * frame (no more bytes for at least one character time). Kept short, as
 * ISR/callback code should be: snapshot the frame, mark it ready, re-arm
 * DMA. Frame validation happens later from task context.
 *
 * The snapshot copy has to happen before DMA is re-armed at the end of this
 * function, not after: the instant HAL_UARTEx_ReceiveToIdle_DMA() runs
 * again, modbus_rx_buffer becomes fair game for the next incoming frame. On
 * a noisy or shared RS-485 bus that next frame could start arriving almost
 * immediately, so without this copy the task could end up validating a
 * buffer that's being overwritten out from under it.
 *
 * @param event UART reception event type reported by HAL.
 * @param Size Number of bytes received before the event.
 */
void modbus_rx_event(HAL_UART_RxEventTypeTypeDef event, uint16_t Size) {

	if (event == HAL_UART_RXEVENT_IDLE) {

		memcpy(modbus_rx_snapshot, modbus_rx_buffer, Size);
		modbus_rx_size = Size;

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(modbus_rx_semaphore, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, modbus_rx_buffer,
			sizeof(modbus_rx_buffer));
}

/**
 * @brief Recover DMA reception after a UART hardware error.
 *
 * Framing/noise/overrun errors can terminate the current reception. Without
 * re-arming here, the Modbus master would stop receiving permanently after
 * the first communication error.
 *
 * @param huart UART instance that reported the error.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {

	if (huart == &huart1) {

		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, modbus_rx_buffer,
				sizeof(modbus_rx_buffer));
	}
}
