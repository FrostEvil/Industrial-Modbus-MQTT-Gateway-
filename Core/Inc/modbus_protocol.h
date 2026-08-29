/*
 * modbus_protocol.h
 *
 *  Created on: Jul 5, 2026
 *      Author: tomas
 */

#ifndef INC_MODBUS_PROTOCOL_H_
#define INC_MODBUS_PROTOCOL_H_

#include <stdint.h>

/**
 * @brief Result of Modbus communication or response validation.
 *
 * MODBUS_ERR_EXCEPTION covers both a real Modbus exception response and an
 * unexpected function code. Both are treated as non-retryable protocol
 * errors here, so the two cases are not distinguished further.
 */
typedef enum {
	MODBUS_OK = 0x00U,
	MODBUS_ERR_LENGTH = 0x01U,
	MODBUS_ERR_ADDRESS = 0x02U,
	MODBUS_ERR_EXCEPTION = 0x03U,
	MODBUS_ERR_CRC = 0x04U,
	MODBUS_ERR_TIMEOUT = 0x05U,
	MODBUS_ERR_HAL = 0x06U
} ModbusStatus_t;

/**
 * @brief Calculate Modbus RTU CRC16 (init 0xFFFF, polynomial 0xA001, LSB first).
 *
 * Does not modify the input buffer or append the CRC to it.
 */
uint16_t modbus_crc16(const uint8_t *data, uint16_t len);

/**
 * @brief Validate a received Modbus response.
 *
 * Checked in this order: length -> slave address -> CRC -> exception /
 * function code -> length. CRC is checked before anything else in the
 * frame is trusted, since none of those fields mean anything until frame
 * integrity is confirmed.
 *
 * The expected-length calculation assumes function code 0x03 (Read Holding
 * Registers); other function codes would need a different formula.
 */
ModbusStatus_t modbus_validate_frame(
		const uint8_t *modbus_rx_buffer,
		uint16_t size,
		uint8_t slave_id,
		uint8_t function_code,
		uint8_t register_count);

#endif /* INC_MODBUS_PROTOCOL_H_ */
