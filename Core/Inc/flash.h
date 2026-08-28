/*
 * flash.h
 *
 *  Created on: Jul 22, 2026
 *      Author: tomas
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include <stdint.h>

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

typedef struct {
	uint8_t manufacturer_id;
	uint8_t memory_type;
	uint8_t capacity;
} FlashJedecId_t;

FlashStatus_t flash_read_jedec_id(FlashJedecId_t *jedec_id);
FlashStatus_t flash_read_data(uint32_t address, uint8_t *data, uint16_t length);
FlashStatus_t flash_write_data(uint32_t address, const uint8_t *data,
		uint16_t length);
FlashStatus_t flash_erase_sector(uint32_t address);
FlashStatus_t flash_is_busy(uint8_t *busy);
FlashStatus_t flash_is_write_enabled(uint8_t *write_enabled);
FlashStatus_t flash_erase_chip();
#endif /* INC_FLASH_H_ */
