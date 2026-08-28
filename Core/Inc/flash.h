/*
 * flash.h
 *
 *  Created on: Jul 22, 2026
 *      Author: tomas
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include <stdint.h>

/**
 * @brief Result of an external Flash memory operation.
 *
 * The status is returned by the low-level Flash driver and allows higher
 * layers to distinguish between invalid input, communication problems and
 * Flash-specific conditions.
 */
typedef enum {
	FLASH_STATUS_OK = 0,
	FLASH_STATUS_INVALID_ADDRESS,
	FLASH_STATUS_PAGE_OVERFLOW,
	FLASH_STATUS_TIMEOUT,
	FLASH_STATUS_SPI_ERROR,
	FLASH_STATUS_INVALID_ARGUMENT,
	FLASH_STATUS_WRITE_NOT_ENABLED,
	FLASH_STATUS_LOG_CORRUPTED,
	FLASH_STATUS_FULL
} FlashStatus_t;

/**
 * @brief JEDEC identification returned by the Flash device.
 *
 * The three fields are the standard bytes returned by the JEDEC ID command:
 * manufacturer ID, memory type and device capacity.
 */
typedef struct {
	uint8_t manufacturer_id;
	uint8_t memory_type;
	uint8_t capacity;
} FlashJedecId_t;

/**
 * @brief Read the JEDEC identification of the Flash device.
 */
FlashStatus_t flash_read_jedec_id(FlashJedecId_t *jedec_id);

/**
 * @brief Read raw bytes from the Flash memory.
 *
 * The complete requested range must fit inside the physical Flash address
 * space.
 */
FlashStatus_t flash_read_data(uint32_t address, uint8_t *data, uint16_t length);

/**
 * @brief Program raw data into the Flash memory.
 *
 * The write must fit inside one 256-byte Flash page. Page boundary handling
 * is deliberately left to the caller instead of splitting the operation
 * automatically.
 */
FlashStatus_t flash_write_data(uint32_t address, const uint8_t *data,
		uint16_t length);

/**
 * @brief Erase one 4 KB Flash sector.
 *
 * The address must point to the beginning of a sector.
 */
FlashStatus_t flash_erase_sector(uint32_t address);

/**
 * @brief Check whether the Flash is currently busy.
 *
 * The returned value is non-zero while an internal program or erase
 * operation is still in progress.
 */
FlashStatus_t flash_is_busy(uint8_t *busy);

/**
 * @brief Check whether the Flash Write Enable Latch is set.
 *
 * The returned value is non-zero when the device has accepted the
 * Write Enable command and is ready for a program or erase operation.
 */
FlashStatus_t flash_is_write_enabled(uint8_t *write_enabled);

/**
 * @brief Erase the entire Flash chip.
 *
 * This is a long-running operation. The driver waits until the chip
 * finishes the erase or the configured timeout is reached.
 */
FlashStatus_t flash_erase_chip(void);

#endif /* INC_FLASH_H_ */
