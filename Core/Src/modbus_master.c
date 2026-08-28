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
 * This module connects the protocol layer with the STM32 peripherals.
 *
 * The protocol layer (modbus_protocol.c) knows only about raw frames.
 * This module is responsible for:
 *
 *   - sending Modbus requests through USART1,
 *   - controlling the RS-485 transceiver direction,
 *   - receiving responses using DMA + IDLE line detection,
 *   - handling response timeout and retries,
 *   - passing the received frame to the protocol validator.
 *
 * The implementation is intentionally tied to this project's hardware:
 *
 *   USART1       -> Modbus / RS-485
 *   DE_RE_Output -> transceiver direction control
 *
 * Porting this module to another UART or another transceiver would require
 * changes here, while the protocol module could remain unchanged.
 */

#include "modbus_master.h"
#include "usart.h"
#include <string.h>
#include <stdbool.h>

/*
 * These variables form the internal receive state of the Modbus master.
 *
 * They are intentionally kept private to this module. Higher layers do not
 * access the DMA buffer directly. Instead, they call modbus_master_poll()
 * and receive a status code plus copied response data.
 *
 * volatile is required because these variables are written from the UART
 * callback context and read from normal task code.
 */
static volatile uint16_t modbus_rx_size = 0;
static volatile bool modbus_rx_ready = false;

/*
 * DMA writes received bytes into this buffer.
 *
 * The application does not use this buffer directly. Once a valid response
 * is received, modbus_master_poll() copies it to the caller-provided
 * modbus_rx_data buffer.
 */
static uint8_t modbus_rx_buffer[256];

/**
 * @brief Start asynchronous DMA reception for Modbus responses.
 *
 * ReceiveToIdle_DMA allows the UART to receive an unknown-length frame and
 * generate a callback when the UART detects an IDLE condition on the line.
 *
 * In this project an IDLE event is used as the indication that the Modbus
 * response frame has finished arriving.
 *
 * @return
 *         MODBUS_OK   DMA reception was started successfully.
 *         MODBUS_ERR_HAL HAL failed to start reception.
 */
ModbusStatus_t modbus_master_init(void) {

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
 * Sending through a half-duplex RS-485 bus requires control of the
 * transceiver direction:
 *
 *   1. Ensure the UART TC flag starts in a known state.
 *   2. Set DE/RE -> transmitter enabled.
 *   3. Start UART transmission using DMA.
 *   4. Wait until the UART reports TC (Transmission Complete).
 *   5. Set DE/RE low -> return the bus to receive mode.
 *
 * The distinction between DMA completion and UART TC is important.
 *
 * DMA completion means that DMA has finished moving bytes into the UART
 * peripheral. It does NOT necessarily mean that the UART has physically
 * transmitted the last bit onto the RS-485 bus.
 *
 * If DE/RE were switched to receive mode too early, the last part of the
 * Modbus frame could be cut off.
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
	 * TC indicates that the UART shift register and transmit register are
	 * empty. Clearing the previous state is important because TC may still
	 * contain the result of an earlier transmission.
	 *
	 * Without clearing it first, the code below could see an old TC=1 state
	 * and release the RS-485 bus too early.
	 */
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);

	/*
	 * Enable transmission on the RS-485 transceiver.
	 */
	HAL_GPIO_WritePin(
	DE_RE_Output_GPIO_Port,
	DE_RE_Output_Pin, GPIO_PIN_SET);

	/*
	 * Let DMA feed the request bytes to USART1.
	 */
	hal_status = HAL_UART_Transmit_DMA(&huart1, modbus_tx_buffer,
			modbus_tx_buffer_size);

	/*
	 * If DMA could not be started, there is no valid transmission to wait
	 * for. Report the hardware abstraction layer error immediately.
	 */
	if (hal_status != HAL_OK) {

		/*
		 * Return the transceiver to receive mode before leaving the
		 * function. Otherwise the bus could remain stuck in transmit mode.
		 */
		HAL_GPIO_WritePin(
		DE_RE_Output_GPIO_Port,
		DE_RE_Output_Pin, GPIO_PIN_RESET);

		return MODBUS_ERR_HAL;
	}

	/*
	 * Wait for the actual end of the UART transmission.
	 *
	 * This is intentionally checking TC rather than DMA completion.
	 * See the function documentation above for the reason.
	 */
	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {
	}

	/*
	 * Transmission has physically finished, so the RS-485 transceiver can
	 * return to receive mode.
	 */
	HAL_GPIO_WritePin(
	DE_RE_Output_GPIO_Port,
	DE_RE_Output_Pin, GPIO_PIN_RESET);

	return MODBUS_OK;
}

/**
 * @brief Send a Modbus request and wait for a valid response.
 *
 * This function performs one complete Modbus transaction:
 *
 *   request
 *      ↓
 *   transmit
 *      ↓
 *   wait for response
 *      ↓
 *   validate response
 *      ↓
 *   retry if the error is retryable
 *
 * A single request may be attempted several times according to
 * modbus_retry_policy.
 *
 * Retry is used for transmission-level problems such as:
 *   - timeout,
 *   - CRC error,
 *   - invalid frame length.
 *
 * A Modbus exception response is not retried because it is a logical
 * response from the slave. Sending the same request again would normally
 * not change the result.
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

	/*
	 * status contains the result of the lower-level send operation.
	 */
	ModbusStatus_t status;

	/*
	 * final_status stores the result of the latest attempt.
	 *
	 * Initialising it to timeout is intentional: if no response is received
	 * during the entire retry sequence, timeout is the correct final result.
	 */
	ModbusStatus_t final_status = MODBUS_ERR_TIMEOUT;

	/*
	 * A previous response must never be mistaken for the response to the
	 * new transaction.
	 */
	modbus_rx_ready = false;

	for (uint8_t attempt = 0; attempt < modbus_retry_policy->max_attempts;
			attempt++) {

		/*
		 * Each attempt starts with the assumption that no valid response
		 * will arrive. If a frame is received, this value will be replaced
		 * by the validation result.
		 */
		ModbusStatus_t attempt_status = MODBUS_ERR_TIMEOUT;

		/*
		 * Transmit the Modbus request.
		 */
		status = modbus_send_request(modbus_tx_buffer, modbus_tx_buffer_size);

		/*
		 * A HAL-level transmission failure is different from a bad Modbus
		 * response. It is a local hardware/driver failure, so there is no
		 * useful response to validate.
		 */
		if (status != MODBUS_OK) {
			return status;
		}

		/*
		 * Wait for a response frame.
		 *
		 * At the moment this uses polling of modbus_rx_ready.
		 * The variable is set by modbus_rx_event() when the UART detects
		 * the end of the incoming frame.
		 *
		 * The loop is intentionally bounded by response_timeout_ms so that
		 * a missing slave cannot block the task forever.
		 */
		uint32_t wait_start = HAL_GetTick();

		while (HAL_GetTick() - wait_start
				< modbus_retry_policy->response_timeout_ms) {

			if (modbus_rx_ready) {
				break;
			}
		}

		/*
		 * If a frame arrived, validate it before using any of its data.
		 *
		 * The protocol module checks address, CRC, function code and expected
		 * length. Only a MODBUS_OK result means the received data is valid.
		 */
		if (modbus_rx_ready) {

			attempt_status = modbus_validate_frame(modbus_rx_buffer,
					modbus_rx_size, modbus_target->slave_id,
					modbus_target->function_code,
					modbus_target->register_count_lo);
		}

		/*
		 * Every attempt produces exactly one result.
		 *
		 * Storing that result here is important because the final result
		 * must represent the LAST attempt, including a timeout after an
		 * earlier CRC error.
		 */
		final_status = attempt_status;

		/*
		 * Valid response -> copy it to the caller's buffer and stop retrying.
		 */
		if (attempt_status == MODBUS_OK) {

			/*
			 * Copy the data out of the internal DMA buffer.
			 *
			 * The DMA buffer is continuously re-armed by the receive
			 * callback, therefore higher layers should work on their own
			 * stable copy rather than directly on the DMA memory.
			 */
			memcpy(modbus_rx_data, modbus_rx_buffer, modbus_rx_size);

			break;
		}

		/*
		 * A valid Modbus exception response means that communication with
		 * the slave worked, but the slave rejected the request logically.
		 *
		 * Retrying the exact same request is therefore not useful here.
		 */
		if (attempt_status == MODBUS_ERR_EXCEPTION) {
			break;
		}

		/*
		 * The received response was invalid or no response arrived.
		 * Prepare for the next retry by clearing the "response ready" state.
		 */
		modbus_rx_ready = false;
	}

	return final_status;
}

/**
 * @brief Handle a completed UART reception detected by ReceiveToIdle_DMA.
 *
 * The HAL calls this function when a UART reception event occurs.
 *
 * For this project we are interested in HAL_UART_RXEVENT_IDLE because the
 * IDLE condition indicates that no more bytes have arrived for at least one
 * character time. In practice this marks the end of the current Modbus RTU
 * response frame.
 *
 * The callback does not parse the frame itself. ISR/callback code should be
 * kept short. Instead it only:
 *
 *   1. stores the number of received bytes,
 *   2. marks the buffer as ready,
 *   3. immediately re-arms DMA reception.
 *
 * The actual frame validation is performed later from task context.
 *
 * @param event UART reception event type reported by HAL.
 * @param Size Number of bytes received before the event.
 */
void modbus_rx_event(HAL_UART_RxEventTypeTypeDef event, uint16_t Size) {

	if (event == HAL_UART_RXEVENT_IDLE) {

		/*
		 * Store the size before notifying the main application code.
		 */
		modbus_rx_size = Size;

		/*
		 * This flag is the simple hand-off mechanism between the UART
		 * callback and modbus_master_poll().
		 */
		modbus_rx_ready = true;
	}

	/*
	 * ReceiveToIdle_DMA uses a finite DMA transfer. After the reception
	 * event, DMA must be armed again to receive the next Modbus frame.
	 *
	 * This re-arm operation is also necessary after an IDLE event because
	 * the previous transfer is no longer active.
	 */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, modbus_rx_buffer,
			sizeof(modbus_rx_buffer));
}

/**
 * @brief Recover DMA reception after a UART hardware error.
 *
 * UART errors such as framing, noise or overrun can terminate the current
 * reception. If DMA were not re-armed here, the Modbus master could stop
 * receiving permanently after the first communication error.
 *
 * The actual error classification and escalation are handled at a higher
 * level. This callback only restores the receive mechanism.
 *
 * @param huart UART instance that reported the error.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {

	if (huart == &huart1) {

		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, modbus_rx_buffer,
				sizeof(modbus_rx_buffer));
	}
}
