/*
 * modbus_protocol.c
 *
 *  Created on: Jul 5, 2026
 *      Author: tomas
 */

/**
 * @file modbus_protocol.c
 *
 * @brief Modbus RTU protocol logic independent of the STM32 hardware.
 *
 * This module operates only on raw Modbus frames. It knows nothing about
 * UART, DMA, GPIO, FreeRTOS or the RS-485 transceiver, so it stays easy to
 * test and could be reused with a different UART driver or MCU.
 */

#include <stdint.h>
#include "modbus_protocol.h"

/**
 * @brief Calculate CRC16 according to the Modbus RTU specification.
 *
 * "modbus" here names the specific CRC16 variant used (init 0xFFFF,
 * polynomial 0xA001, LSB first - formally "CRC-16/MODBUS" in CRC
 * catalogues), not "only usable for Modbus frames". This function is also
 * reused in flash_logger.c / alarm_manager_task.c to protect Flash log
 * records - any 16-bit checksum would do there, this one was simply
 * already available.
 *
 * @param data Pointer to the bytes for which the CRC should be calculated.
 * @param len Number of bytes to process.
 *
 * @return Calculated Modbus CRC16 value.
 */
uint16_t modbus_crc16(const uint8_t *data, uint16_t len) {
	uint16_t crc = 0xFFFF;

	for (uint16_t byte_index = 0; byte_index < len; byte_index++) {
		crc ^= (uint16_t) data[byte_index];

		/*
		 * Modbus CRC processes each byte LSB-first.
		 */
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

/**
 * @brief Validate a received Modbus response frame.
 *
 * @param modbus_rx_buffer Received Modbus frame.
 * @param size Number of bytes received.
 * @param slave_id Expected slave address.
 * @param function_code Expected Modbus function code.
 * @param register_count Number of registers expected in the response.
 *
 * @return
 *         MODBUS_OK               Frame is valid.
 *         MODBUS_ERR_LENGTH       Frame length is invalid.
 *         MODBUS_ERR_ADDRESS      Response came from an unexpected slave.
 *         MODBUS_ERR_CRC          CRC verification failed.
 *         MODBUS_ERR_EXCEPTION    Slave returned an exception or an
 *                                 unexpected function code.
 */
ModbusStatus_t modbus_validate_frame(const uint8_t *modbus_rx_buffer,
		uint16_t size, uint8_t slave_id, uint8_t function_code,
		uint8_t register_count) {

	/*
	 * Shortest possible RTU response: address + function + CRC(2) = 4 bytes.
	 */
	if (size < 4) {
		return MODBUS_ERR_LENGTH;
	}

	/*
	 * Reject frames from other devices on the shared RS-485 bus.
	 */
	if (slave_id != modbus_rx_buffer[0]) {
		return MODBUS_ERR_ADDRESS;
	}

	uint16_t crc = modbus_crc16(modbus_rx_buffer, size - 2);

	uint16_t received_crc = modbus_rx_buffer[size - 2]
			| (modbus_rx_buffer[size - 1] << 8);

	if (crc != received_crc) {
		return MODBUS_ERR_CRC;
	}

	/*
	 * Bit 7 of the function code set (e.g. 0x03 -> 0x83) marks a Modbus
	 * exception response. Treated as a logical protocol error, not a
	 * transmission error, so the caller will not retry the same request.
	 */
	if (modbus_rx_buffer[1] >= 0x80) {
		return MODBUS_ERR_EXCEPTION;
	}

	if (modbus_rx_buffer[1] != function_code) {
		return MODBUS_ERR_EXCEPTION;
	}

	/*
	 * Expected length for a Read Holding Registers (0x03) response:
	 * address(1) + function(1) + byte_count(1) + 2*register_count + CRC(2).
	 */
	uint8_t expected_length = 5 + (2 * register_count);

	if (size != expected_length) {
		return MODBUS_ERR_LENGTH;
	}

	return MODBUS_OK;
}
