/*
 * modbus_protocol.h
 *
 *  Created on: Jul 5, 2026
 *      Author: tomas
 */

/**
 * @file modbus_protocol.h
 * @brief Protocol-level logic for Modbus RTU: CRC16 calculation and frame
 *        validation. Contains no hardware/HAL dependencies - this module
 *        operates purely on byte buffers, independent of UART, DMA, or GPIO.
 *
 * This layer mirrors the driver/application separation used in previous
 * projects (e.g. bme280.c): it knows nothing about how a frame physically
 * arrives (UART, DMA, RS-485 direction control), only how to interpret and
 * verify the bytes once they exist in memory.
 */

#ifndef INC_MODBUS_PROTOCOL_H_
#define INC_MODBUS_PROTOCOL_H_

#include <stdint.h>

/**
 * @brief Result of validating a received Modbus RTU frame.
 *
 * Note: MODBUS_ERR_EXCEPTION covers both a genuine slave exception response
 * (function code with bit 0x80 set) and a mismatched function code in an
 * otherwise well-formed, CRC-valid frame. Both cases mean "the frame is
 * intact, but not the normal response we expected" - retrying an identical
 * request will not help in either case.
 *
 * MODBUS_ERR_TIMEOUT is returned by modbus_master_poll() (not by
 * modbus_validate_frame() itself) when no frame at all arrived within the
 * response window on the LAST retry attempt. If earlier attempts received
 * an invalid frame (wrong length/address/CRC) but the final attempt was
 * pure silence, MODBUS_ERR_TIMEOUT is what gets reported - see
 * modbus_master.c for the exact retry/reporting policy.
 */
typedef enum {
	MODBUS_OK = 0x00U, /**< Frame is valid and matches the expected response. */
	MODBUS_ERR_LENGTH = 0x01U, /**< Frame is too short, or does not match the expected length for this function code. */
	MODBUS_ERR_ADDRESS = 0x02U, /**< Slave address in the frame does not match the address we queried. */
	MODBUS_ERR_EXCEPTION = 0x03U, /**< Slave returned an exception response, or an unexpected function code. */
	MODBUS_ERR_CRC = 0x04U, /**< CRC16 mismatch - frame content cannot be trusted (transmission error). */
	MODBUS_ERR_TIMEOUT = 0x05U /**< No response at all within the timeout window (see note above). */
} ModbusStatus_t;

/**
 * @brief Modbus exception codes relevant to this project.
 *
 * This project only ever sends function code 0x03 (Read Holding Registers),
 * so this is NOT an exhaustive list of all exception codes defined by the
 * Modbus specification - only the ones that can realistically occur here.
 */
typedef enum {
	MODBUS_EXC_ILLEGAL_FUNCTION = 0x01U, /**< Slave does not support this function code (should not happen - we always send 0x03). */
	MODBUS_EXC_ILLEGAL_ADDRESS = 0x02U, /**< Requested register does not exist on the slave (e.g. wrong start address). */
	MODBUS_EXC_ILLEGAL_VALUE = 0x03U, /**< Request itself is malformed (e.g. invalid register count). */
	MODBUS_EXC_SLAVE_FAILURE = 0x04U /**< Internal error on the slave side (Arduino). */
} ModbusExceptionCode_t;

/**
 * @brief Calculates the Modbus RTU CRC16 checksum over a byte buffer.
 *
 * Standard Modbus CRC16 algorithm: initial value 0xFFFF, polynomial 0xA001
 * (reflected form of the CRC-16/MODBUS polynomial), processed LSB-first.
 * Verified against the official test vector: CRC16({0x01,0x03,0x00,0x00,0x00,0x01}) == 0x0A84.
 *
 * @param data  Pointer to the byte buffer to calculate the CRC over.
 * @param len   Number of bytes to include in the calculation.
 * @return      16-bit CRC value. When appending to a frame, send the low
 *              byte first, then the high byte (little-endian on the wire).
 */
uint16_t modbus_crc16(const uint8_t *data, uint16_t len);

/**
 * @brief Validates a received Modbus RTU frame (request or response).
 *
 * Check order is intentional:
 * 1. Minimum length  - avoid reading bytes that were never received.
 * 2. Slave address   - cheapest check, reject frames from other devices.
 * 3. CRC16           - must happen BEFORE interpreting any other byte,
 *                       because until CRC is verified we cannot trust
 *                       the content of the function code / exception byte.
 * 4. Function code / exception - only trusted once CRC confirms the
 *                       frame content is intact.
 * 5. Length vs expected for a normal (non-exception) response.
 *
 * @param modbus_rx_buffer  Pointer to the received frame.
 * @param size              Number of bytes actually received.
 * @param slave_id          Expected slave address (the one we queried).
 * @param function_code     Expected function code (e.g. 0x03).
 * @param register_count    Number of registers requested - needed to
 *                           compute the expected length of a normal response.
 *                           Limited to 8 bits (max 255 registers) - not a
 *                           real constraint for this project (always 3),
 *                           but noted here as a known limitation.
 * @param exception_code    Output parameter, filled in only when the
 *                           function returns MODBUS_ERR_EXCEPTION due to a
 *                           genuine slave exception (bit 0x80 set). Pass
 *                           NULL if the caller does not need this detail.
 * @return                  MODBUS_OK if the frame is valid and matches the
 *                           expected response; an error status otherwise.
 */
ModbusStatus_t modbus_validate_frame(uint8_t *modbus_rx_buffer, uint8_t size,
		uint8_t slave_id, uint8_t function_code, uint8_t register_count,
		uint8_t *exception_code);

#endif /* INC_MODBUS_PROTOCOL_H_ */
