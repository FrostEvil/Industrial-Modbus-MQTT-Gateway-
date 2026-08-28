/*
 * flash.c
 *
 *  Created on: Jul 22, 2026
 *      Author: tomas
 */

#include "flash.h"
#include <stdint.h>
#include "gpio.h"
#include "spi.h"

#define FLASH_SIZE_BYTES 0x01000000
#define FLASH_PAGE_SIZE 0x0100
#define FLASH_CHIP_ERASE_TIMEOUT_MS 200000
#define FLASH_SECTOR_ERASE_TIMEOUT_MS 450
#define FLASH_PAGE_PROGRAM_TIMEOUT_MS 3
#define FLASH_SPI_TIMEOUT_MS 500

#define FLASH_CMD_WRITE_ENABLE      0x06
#define FLASH_CMD_READ_STATUS_REG1  0x05
#define FLASH_CMD_READ_JEDEC_ID     0x9F
#define FLASH_CMD_READ_DATA         0x03
#define FLASH_CMD_PAGE_PROGRAM      0x02
#define FLASH_CMD_SECTOR_ERASE      0x20
#define FLASH_CMD_CHIP_ERASE		0xC7

#define FLASH_STATUS_REG1_BUSY_BIT  0x01
#define FLASH_STATUS_REG1_WEL_BIT 0x02

static HAL_StatusTypeDef flash_spi_transmit(const uint8_t *tx_buffer,
		uint16_t size) {
	HAL_StatusTypeDef spi_transmit_status;

	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
	spi_transmit_status = HAL_SPI_Transmit(&hspi2, tx_buffer, size,
	FLASH_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

	return spi_transmit_status;
}

static HAL_StatusTypeDef flash_spi_transmit_command_and_data(
		const uint8_t *tx_buffer_first, uint16_t size_first,
		const uint8_t *tx_buffer_second, uint16_t size_second) {
	HAL_StatusTypeDef spi_transmit_status;

	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
	spi_transmit_status = HAL_SPI_Transmit(&hspi2, tx_buffer_first, size_first,
	FLASH_SPI_TIMEOUT_MS);
	if (spi_transmit_status != HAL_OK) {
		HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
		return spi_transmit_status;
	}
	spi_transmit_status = HAL_SPI_Transmit(&hspi2, tx_buffer_second,
			size_second,
			FLASH_SPI_TIMEOUT_MS);

	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

	return spi_transmit_status;
}

static HAL_StatusTypeDef flash_spi_transfer(const uint8_t *tx_buffer,
		uint16_t tx_size, uint8_t *rx_buffer, uint16_t rx_size) {
	HAL_StatusTypeDef spi_transmit_status;
	HAL_StatusTypeDef spi_receive_status;

	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
	spi_transmit_status = HAL_SPI_Transmit(&hspi2, tx_buffer, tx_size,
	FLASH_SPI_TIMEOUT_MS);

	if (spi_transmit_status != HAL_OK) {
		HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
		return spi_transmit_status;
	}

	spi_receive_status = HAL_SPI_Receive(&hspi2, rx_buffer, rx_size,
	FLASH_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

	return spi_receive_status;
}

static FlashStatus_t flash_write_enable(void) {
	uint8_t write_enable_tx_buffer[] = { FLASH_CMD_WRITE_ENABLE };

	if (flash_spi_transmit(write_enable_tx_buffer,
			sizeof(write_enable_tx_buffer)) != HAL_OK) {
		return FLASH_STATUS_SPI_ERROR;
	}

	return FLASH_STATUS_OK;

}

FlashStatus_t flash_read_jedec_id(FlashJedecId_t *jedec_id) {
	if (jedec_id == NULL) {
		return FLASH_STATUS_INVALID_ARGUMENT;
	}

	uint8_t tx_buffer[] = { FLASH_CMD_READ_JEDEC_ID };
	uint8_t rx_buffer[3];

	if (flash_spi_transfer(tx_buffer, sizeof(tx_buffer), rx_buffer,
			sizeof(rx_buffer)) != HAL_OK) {
		return FLASH_STATUS_SPI_ERROR;
	}
	jedec_id->manufacturer_id = rx_buffer[0];
	jedec_id->memory_type = rx_buffer[1];
	jedec_id->capacity = rx_buffer[2];
	return FLASH_STATUS_OK;

}

static FlashStatus_t flash_read_status_register(uint8_t *status_register) {

	uint8_t tx_buffer[] = { FLASH_CMD_READ_STATUS_REG1, 0x00 };
	uint8_t rx_buffer[2];

	if (status_register == NULL) {
		return FLASH_STATUS_INVALID_ARGUMENT;
	}

	if (flash_spi_transfer(tx_buffer, sizeof(tx_buffer), rx_buffer,
			sizeof(rx_buffer)) != HAL_OK) {
		return FLASH_STATUS_SPI_ERROR;
	}

	*status_register = rx_buffer[1];
	return FLASH_STATUS_OK;
}

FlashStatus_t flash_is_write_enabled(uint8_t *write_enabled) {
	if (write_enabled == NULL) {
		return FLASH_STATUS_INVALID_ARGUMENT;
	}

	uint8_t status_register;
	FlashStatus_t flash_status;

	flash_status = flash_read_status_register(&status_register);

	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	*write_enabled = status_register & FLASH_STATUS_REG1_WEL_BIT;
	return FLASH_STATUS_OK;
}

static FlashStatus_t flash_write_enable_and_verify(void) {
	FlashStatus_t flash_status;
	uint8_t write_enable;

	flash_status = flash_write_enable();
	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	flash_status = flash_is_write_enabled(&write_enable);
	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	if (!write_enable) {
		return FLASH_STATUS_WRITE_NOT_ENABLED;
	}

	return FLASH_STATUS_OK;
}

FlashStatus_t flash_is_busy(uint8_t *busy) {
	if (busy == NULL) {
		return FLASH_STATUS_INVALID_ARGUMENT;
	}

	uint8_t status_register;
	FlashStatus_t flash_status;

	flash_status = flash_read_status_register(&status_register);

	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	*busy = status_register & FLASH_STATUS_REG1_BUSY_BIT;
	return FLASH_STATUS_OK;
}

static FlashStatus_t flash_wait_while_busy(uint32_t timeout) {
	FlashStatus_t flash_status;
	uint8_t status_register;
	uint8_t status_busy = 1;

	uint32_t start_time = HAL_GetTick();
	while (status_busy) {

		flash_status = flash_read_status_register(&status_register);
		if (flash_status != FLASH_STATUS_OK) {
			return flash_status;
		}

		status_busy = status_register & FLASH_STATUS_REG1_BUSY_BIT;

		if (HAL_GetTick() - start_time > timeout) {
			return FLASH_STATUS_TIMEOUT;
		}
	}
	return FLASH_STATUS_OK;
}

FlashStatus_t flash_read_data(uint32_t address, uint8_t *data, uint16_t length) {
	if (data == NULL || length == 0) {
		return FLASH_STATUS_INVALID_ARGUMENT;
	}

	const uint8_t tx_buffer[] = { FLASH_CMD_READ_DATA, (address >> 16) & 0xFF,
			(address >> 8) & 0xFF, address & 0xFF };

	if (flash_spi_transfer(tx_buffer, sizeof(tx_buffer), data, length)
			!= HAL_OK) {
		return FLASH_STATUS_SPI_ERROR;
	} else {
		return FLASH_STATUS_OK;
	}

}

FlashStatus_t flash_write_data(uint32_t address, const uint8_t *data,
		uint16_t length) {

	FlashStatus_t flash_status;

	if (data == NULL || length == 0) {
		return FLASH_STATUS_INVALID_ARGUMENT;
	}

	if (address >= FLASH_SIZE_BYTES || address + length > FLASH_SIZE_BYTES) {
		return FLASH_STATUS_INVALID_ADDRESS;
	}

	if ((address & 0xFF) + length > FLASH_PAGE_SIZE) {
		return FLASH_STATUS_PAGE_OVERFLOW;
	}

	uint8_t tx_buffer[] = { FLASH_CMD_PAGE_PROGRAM, (address >> 16) & 0xFF,
			(address >> 8) & 0xFF, address & 0xFF };

	flash_status = flash_write_enable_and_verify();
	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	if (flash_spi_transmit_command_and_data(tx_buffer, sizeof(tx_buffer), data,
			length) != HAL_OK) {
		return FLASH_STATUS_SPI_ERROR;
	}

	return flash_wait_while_busy(FLASH_PAGE_PROGRAM_TIMEOUT_MS);
}

FlashStatus_t flash_erase_sector(uint32_t address) {

	if (address >= FLASH_SIZE_BYTES) {
		return FLASH_STATUS_INVALID_ADDRESS;
	}

	const uint8_t tx_buffer[] = { FLASH_CMD_SECTOR_ERASE, (address >> 16)
			& 0xFF, (address >> 8) & 0xFF, address & 0xFF };
	FlashStatus_t flash_status;

	flash_status = flash_write_enable_and_verify();
	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	if (flash_spi_transmit(tx_buffer, sizeof(tx_buffer)) != HAL_OK) {
		return FLASH_STATUS_SPI_ERROR;
	}
	return flash_wait_while_busy(FLASH_SECTOR_ERASE_TIMEOUT_MS);
}

FlashStatus_t flash_erase_chip(void) {
	FlashStatus_t flash_status;

	const uint8_t tx_buffer[] = { FLASH_CMD_CHIP_ERASE };

	flash_status = flash_write_enable_and_verify();
	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	if (flash_spi_transmit(tx_buffer, sizeof(tx_buffer)) != HAL_OK) {
		return FLASH_STATUS_SPI_ERROR;
	}

	return flash_wait_while_busy(FLASH_CHIP_ERASE_TIMEOUT_MS);

}
