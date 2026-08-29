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
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"

/*
 * Internal receive state of the Modbus master, private to this module.
 * Higher layers only see modbus_master_poll()'s status code and copied
 * response data, never this buffer directly.
 *
 * volatile is required: written from the UART callback (ISR context),
 * read from task code.
 */

#define MODBUS_TC_TIMEOUT_MS 20U

static volatile uint16_t modbus_rx_size = 0;
static volatile bool modbus_rx_ready = false;

static uint8_t modbus_rx_buffer[256];     // stale nadpisywany przez DMA
static uint8_t modbus_rx_snapshot[256];   // stabilna kopia dla taska

static SemaphoreHandle_t modbus_rx_semaphore;
static StaticSemaphore_t modbus_rx_sempahore_buffer;

/**
 * @brief Start asynchronous DMA reception for Modbus responses.
 *
 * @return
 *         MODBUS_OK   DMA reception was started successfully.
 *         MODBUS_ERR_HAL HAL failed to start reception.
 */
ModbusStatus_t modbus_master_init(void) {

	modbus_rx_semaphore = xSemaphoreCreateBinaryStatic(
			&modbus_rx_sempahore_buffer);

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
	 * Drain any stale "give" left over from before this transaction
	 * (odpowiednik dzisiejszego modbus_rx_ready = false;).
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
		 * Task śpi tutaj (Blocked), zamiast zajmować CPU w pętli, aż
		 * modbus_rx_event() da semafor albo minie timeout.
		 */
		if (xQueueSemaphoreTake(modbus_rx_semaphore,
				pdMS_TO_TICKS(
						modbus_retry_policy->response_timeout_ms))== pdTRUE) {

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
		 * Drain przed kolejną próbą, na wypadek spóźnionej/przypadkowej
		 * ramki z poprzedniej próby.
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
 * ISR/callback code should be: store the size, mark the buffer ready,
 * re-arm DMA. Frame validation happens later from task context.
 *
 * TODO: DMA is re-armed immediately below, before modbus_master_poll() has
 * copied this data out. On a noisy or shared RS-485 bus, a second IDLE
 * event could in theory overwrite modbus_rx_buffer while it is still being
 * read. Consider copying into a snapshot buffer here, or double-buffering.
 *
 * @param event UART reception event type reported by HAL.
 * @param Size Number of bytes received before the event.
 */
void modbus_rx_event(HAL_UART_RxEventTypeTypeDef event, uint16_t Size) {

	if (event == HAL_UART_RXEVENT_IDLE) {

		/*
		 * Copy out before re-arming below - DMA could start overwriting
		 * modbus_rx_buffer with a new frame the moment
		 * HAL_UARTEx_ReceiveToIdle_DMA() is called again.
		 */
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
