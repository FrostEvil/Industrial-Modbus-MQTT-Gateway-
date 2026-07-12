/*
 * modbus_master.c
 *
 *  Created on: Jul 11, 2026
 *      Author: tomas
 */

/**
 * @file modbus_master.c
 * @brief Implementation of the Modbus RTU master role. Hardware-specific:
 *        hardcoded to USART1 and the DE_RE_Output GPIO pin (this project's
 *        transceiver has a single combined direction pin, not separate
 *        DE/RE - see modbus_send_request()). Porting to a different UART
 *        or a transceiver with separate DE/RE pins requires changes here.
 */

#include "modbus_master.h"
#include "usart.h"
#include <string.h>

// Internal receive state. Not exposed via modbus_master.h - callers only
// ever see the result of modbus_master_poll(), never this buffer directly.
static volatile uint16_t modbus_rx_size = 0;
static volatile uint8_t modbus_rx_flag = 0;
static uint8_t modbus_rx_buffer[256];

void modbus_master_init(void) {
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, modbus_rx_buffer,
			sizeof(modbus_rx_buffer));
}

/**
 * @brief Sends one Modbus request frame over the bus.
 *
 * Sequence: clear TC flag -> drive direction pin high (transmit enable) ->
 * start DMA transmit -> wait for the REAL end of transmission (TC flag),
 * not just DMA handing bytes to the UART -> release the bus (direction pin
 * low). Waiting for TC specifically matters: switching back to receive
 * before the last bit has physically left the shift register would clip
 * the end of the frame on the wire.
 */
static void modbus_send_request(uint8_t *modbus_tx_buffer,
		uint8_t modbus_tx_buffer_size) {

	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);

	HAL_GPIO_WritePin(DE_RE_Output_GPIO_Port, DE_RE_Output_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit_DMA(&huart1, modbus_tx_buffer, modbus_tx_buffer_size);

	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {
	}

	HAL_GPIO_WritePin(DE_RE_Output_GPIO_Port, DE_RE_Output_Pin, GPIO_PIN_RESET);
}

ModbusStatus_t modbus_master_poll(uint8_t *modbus_tx_buffer,
		uint8_t modbus_tx_buffer_size, uint8_t *modbus_rx_data,
		const ModbusTarget_t *modbus_target,
		const ModbusRetryPolicy_t *modbus_retry_policy, uint8_t *exception_code) {

	// Default result if every single attempt is pure silence (flag never
	// gets set at all, so the block below never runs to overwrite this).
	ModbusStatus_t status = MODBUS_ERR_TIMEOUT;
	modbus_rx_flag = 0;

	for (uint8_t i = 0; i < modbus_retry_policy->max_attempts; i++) {
		modbus_send_request(modbus_tx_buffer, modbus_tx_buffer_size);

		uint32_t wait_start = HAL_GetTick();
		while (HAL_GetTick() - wait_start
				< modbus_retry_policy->response_timeout_ms) {
			if (modbus_rx_flag == 1) {
				break;
			}
		}

		if (modbus_rx_flag == 1) {
			status = modbus_validate_frame(modbus_rx_buffer,
					(uint8_t) modbus_rx_size, modbus_target->slave_id,
					modbus_target->function_code,
					modbus_target->register_count_lo, exception_code);

			memcpy(modbus_rx_data, modbus_rx_buffer, modbus_rx_size);

			if (status == MODBUS_OK || status == MODBUS_ERR_EXCEPTION) {
				break;
			}

			// Not OK, not exception -> transmission-level fault, worth
			// retrying. Reset the flag so the next attempt's wait loop
			// does not immediately see this already-processed response.
			modbus_rx_flag = 0;
		}
		// If the flag never became 1 within the timeout, status simply
		// keeps whatever it was (either the MODBUS_ERR_TIMEOUT default,
		// or a leftover status from an earlier, different failure) and
		// the loop tries again.
	}

	return status;
}

/**
 * @brief Called by HAL after every RX DMA event. RX DMA runs in Normal
 *        mode, not Circular (Circular + repeated re-arming of
 *        HAL_UARTEx_ReceiveToIdle_DMA had undocumented reliability issues
 *        observed during development - HAL_BUSY on re-arm, inconsistent
 *        buffer position across calls). In Normal mode, this fires either
 *        on a genuine Idle Line (slave finished a frame) or on Transfer
 *        Complete (the 256-byte buffer filled up without an idle gap -
 *        unexpected noise, not a valid Modbus response). Only IDLE is
 *        treated as a real frame; reception is re-armed unconditionally in
 *        both cases, since a Normal-mode transfer does not restart itself.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == &huart1) {
		if (huart->RxEventType == HAL_UART_RXEVENT_IDLE) {
			modbus_rx_size = Size;
			modbus_rx_flag = 1;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, modbus_rx_buffer,
				sizeof(modbus_rx_buffer));
	}
}

/**
 * @brief Called by HAL on a UART error (framing, noise, overrun).
 *        Occasional errors are expected on a real RS-485 bus. Without
 *        re-arming reception here, the peripheral would stop listening
 *        permanently after the first such error.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, modbus_rx_buffer,
				sizeof(modbus_rx_buffer));
	}
}
