/*
 * modbus_protocol.c
 *
 *  Created on: Jul 5, 2026
 *      Author: tomas
 */

/**
 * @file modbus_protocol.c
 * @brief Implementation of Modbus RTU CRC16 calculation and frame validation.
 *        See modbus_protocol.h for the full interface documentation.
 */

#include <stdint.h>
#include <string.h>
#include "modbus_protocol.h"

uint16_t modbus_crc16(const uint8_t *data, uint16_t len) {
	uint16_t crc = 0xFFFF;

	for (uint16_t byte_index = 0; byte_index < len; byte_index++) {
		crc ^= (uint16_t) data[byte_index];

		for (uint8_t i = 0; i < 8; i++) {
			if (crc & 0x0001) {
				crc >>= 1;
				crc ^= 0xA001;
			} else {
				crc >>= 1;
			}
		}
	}

	return crc;
}

ModbusStatus_t modbus_validate_frame(uint8_t *modbus_rx_buffer, uint8_t size,
		uint8_t slave_id, uint8_t function_code, uint8_t register_count,
		uint8_t *exception_code) {

	if (size < 4) {
		return MODBUS_ERR_LENGTH;
	}

	if (slave_id != modbus_rx_buffer[0]) {
		return MODBUS_ERR_ADDRESS;
	}

	uint16_t crc = modbus_crc16(modbus_rx_buffer, size - 2);
	uint16_t received_crc = modbus_rx_buffer[size - 2]
			| (modbus_rx_buffer[size - 1] << 8);

	if (crc != received_crc) {
		return MODBUS_ERR_CRC;
	}

	if (modbus_rx_buffer[1] >= 0x80) {
		if (exception_code != NULL) {
			*exception_code = modbus_rx_buffer[2];
		}
		return MODBUS_ERR_EXCEPTION;
	}

	if (modbus_rx_buffer[1] != function_code) {
		return MODBUS_ERR_EXCEPTION;
	}

	uint8_t expected_length = 5 + (2 * register_count);
	if (size != expected_length) {
		return MODBUS_ERR_LENGTH;
	}

	return MODBUS_OK;
}
