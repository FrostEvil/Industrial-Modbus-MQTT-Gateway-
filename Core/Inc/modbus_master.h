/*
 * modbus_master.h
 *
 *  Created on: Jul 11, 2026
 *      Author: tomas
 */

/**
 * @file modbus_master.h
 * @brief Modbus RTU master role over RS-485, specialized for this project's
 *        hardware (USART1, GPIO-controlled transceiver direction, single
 *        slave). Unlike modbus_protocol.c/h, this module IS coupled to
 *        specific hardware (HAL, huart1) - see modbus_master.c for details.
 *
 * Public interface is deliberately small: modbus_master_init() once at
 * startup, modbus_master_poll() once per poll cycle. Callers never see the
 * internal receive buffer or DMA/interrupt mechanics - only a status and a
 * copy of the validated frame data.
 */

#ifndef INC_MODBUS_MASTER_H_
#define INC_MODBUS_MASTER_H_

#include "modbus_protocol.h"

/**
 * @brief Identifies which slave and which registers a poll targets.
 *
 * register_start/register_count are split into hi/lo bytes because that is
 * how they are transmitted on the wire (Modbus is big-endian for 16-bit
 * fields) - storing them pre-split avoids repeating the same bit-shifting
 * in multiple places when building the request frame.
 */
typedef struct {
	uint8_t slave_id;
	uint8_t function_code;
	uint8_t register_start_hi;
	uint8_t register_start_lo;
	uint8_t register_count_hi;
	uint8_t register_count_lo;
} ModbusTarget_t;

/**
 * @brief Controls how the master behaves across a poll cycle, independent
 *        of which slave/registers are being queried.
 */
typedef struct {
	uint16_t poll_period_ms;      /**< Time between the start of one poll cycle and the next. */
	uint16_t response_timeout_ms; /**< Max time to wait for a response on a single attempt. */
	uint8_t max_attempts;         /**< Max number of send/wait attempts before giving up. */
} ModbusRetryPolicy_t;

/**
 * @brief Arms UART reception. Must be called exactly once, at startup,
 *        before the first call to modbus_master_poll().
 */
void modbus_master_init(void);

/**
 * @brief Sends a request, retries on transmission-level failures, and
 *        returns a validated result.
 *
 * Retry policy: on MODBUS_OK or MODBUS_ERR_EXCEPTION, returns immediately
 * (exception is a logical error on the request itself - retrying an
 * identical request will not help). On any other status (timeout, bad CRC,
 * wrong length/address), retries up to max_attempts times. The status
 * returned after all attempts are exhausted is whatever happened on the
 * LAST attempt specifically - not an aggregate of all attempts.
 *
 * @param modbus_tx_buffer       Complete request frame, including CRC.
 * @param modbus_tx_buffer_size  Length of modbus_tx_buffer in bytes.
 * @param modbus_rx_data         Output buffer for the received frame, valid
 *                                only when the return value is MODBUS_OK.
 *                                Must be large enough for the expected
 *                                response length.
 * @param modbus_target          Which slave/registers this poll targets.
 * @param modbus_retry_policy    Timeout and retry behaviour for this poll.
 * @param exception_code         Output parameter, filled in only when the
 *                                return value is MODBUS_ERR_EXCEPTION.
 * @return                       MODBUS_OK on success, otherwise the status
 *                                of the last attempt (see retry policy above).
 */
ModbusStatus_t modbus_master_poll(uint8_t *modbus_tx_buffer,
		uint8_t modbus_tx_buffer_size, uint8_t *modbus_rx_data,
		const ModbusTarget_t *modbus_target,
		const ModbusRetryPolicy_t *modbus_retry_policy, uint8_t *exception_code);

#endif /* INC_MODBUS_MASTER_H_ */
