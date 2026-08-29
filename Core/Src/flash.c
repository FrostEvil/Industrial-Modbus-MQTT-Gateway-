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
#include "FreeRTOS.h"
#include "task.h"

#define FLASH_SIZE_BYTES                0x01000000UL
#define FLASH_PAGE_SIZE                0x0100U
#define FLASH_SECTOR_SIZE              0x1000U

#define FLASH_CHIP_ERASE_TIMEOUT_MS    200000U
#define FLASH_SECTOR_ERASE_TIMEOUT_MS  450U
#define FLASH_PAGE_PROGRAM_TIMEOUT_MS  3U
#define FLASH_SPI_TIMEOUT_MS           500U

#define FLASH_CMD_WRITE_ENABLE         0x06U
#define FLASH_CMD_READ_STATUS_REG1     0x05U
#define FLASH_CMD_READ_JEDEC_ID        0x9FU
#define FLASH_CMD_READ_DATA            0x03U
#define FLASH_CMD_PAGE_PROGRAM         0x02U
#define FLASH_CMD_SECTOR_ERASE         0x20U
#define FLASH_CMD_CHIP_ERASE           0xC7U

#define FLASH_STATUS_REG1_BUSY_BIT     0x01U
#define FLASH_STATUS_REG1_WEL_BIT      0x02U

/*
 * One continuous SPI transaction (single CS assertion), for commands that
 * only transmit bytes, e.g. Write Enable or Sector Erase.
 */
static HAL_StatusTypeDef flash_spi_transmit(const uint8_t *tx_buffer,
		uint16_t size) {
	HAL_StatusTypeDef spi_transmit_status;

	HAL_GPIO_WritePin(
	FLASH_CS_GPIO_Port,
	FLASH_CS_Pin, GPIO_PIN_RESET);

	spi_transmit_status = HAL_SPI_Transmit(&hspi2, tx_buffer, size,
	FLASH_SPI_TIMEOUT_MS);

	HAL_GPIO_WritePin(
	FLASH_CS_GPIO_Port,
	FLASH_CS_Pin, GPIO_PIN_SET);

	return spi_transmit_status;
}

/*
 * Two consecutive transmits with CS held low the whole time - needed for
 * commands like Page Program, where command + address + data must belong
 * to one SPI transaction.
 */
static HAL_StatusTypeDef flash_spi_transmit_command_and_data(
		const uint8_t *tx_buffer_first, uint16_t size_first,
		const uint8_t *tx_buffer_second, uint16_t size_second) {
	HAL_StatusTypeDef spi_transmit_status;

	HAL_GPIO_WritePin(
	FLASH_CS_GPIO_Port,
	FLASH_CS_Pin, GPIO_PIN_RESET);

	spi_transmit_status = HAL_SPI_Transmit(&hspi2, tx_buffer_first, size_first,
	FLASH_SPI_TIMEOUT_MS);

	if (spi_transmit_status != HAL_OK) {

		HAL_GPIO_WritePin(
		FLASH_CS_GPIO_Port,
		FLASH_CS_Pin, GPIO_PIN_SET);

		return spi_transmit_status;
	}

	spi_transmit_status = HAL_SPI_Transmit(&hspi2, tx_buffer_second,
			size_second,
			FLASH_SPI_TIMEOUT_MS);

	HAL_GPIO_WritePin(
	FLASH_CS_GPIO_Port,
	FLASH_CS_Pin, GPIO_PIN_SET);

	return spi_transmit_status;
}

/*
 * Command followed by a read within one continuous transaction (CS stays
 * low between the write and the read) - used for JEDEC ID / Status Register.
 */
static HAL_StatusTypeDef flash_spi_transfer(const uint8_t *tx_buffer,
		uint16_t tx_size, uint8_t *rx_buffer, uint16_t rx_size) {
	HAL_StatusTypeDef spi_transmit_status;
	HAL_StatusTypeDef spi_receive_status;

	HAL_GPIO_WritePin(
	FLASH_CS_GPIO_Port,
	FLASH_CS_Pin, GPIO_PIN_RESET);

	spi_transmit_status = HAL_SPI_Transmit(&hspi2, tx_buffer, tx_size,
	FLASH_SPI_TIMEOUT_MS);

	if (spi_transmit_status != HAL_OK) {

		HAL_GPIO_WritePin(
		FLASH_CS_GPIO_Port,
		FLASH_CS_Pin, GPIO_PIN_SET);

		return spi_transmit_status;
	}

	spi_receive_status = HAL_SPI_Receive(&hspi2, rx_buffer, rx_size,
	FLASH_SPI_TIMEOUT_MS);

	HAL_GPIO_WritePin(
	FLASH_CS_GPIO_Port,
	FLASH_CS_Pin, GPIO_PIN_SET);

	return spi_receive_status;
}

/*
 * NOR Flash requires the WEL (Write Enable Latch) bit set before it accepts
 * a program or erase command.
 */
static FlashStatus_t flash_write_enable(void) {
	const uint8_t write_enable_tx_buffer[] = {
	FLASH_CMD_WRITE_ENABLE };

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

	const uint8_t tx_buffer[] = {
	FLASH_CMD_READ_JEDEC_ID };

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

/*
 * Status Register 1: BUSY = internal program/erase still running,
 * WEL = Write Enable accepted.
 */
static FlashStatus_t flash_read_status_register(uint8_t *status_register) {
	if (status_register == NULL) {
		return FLASH_STATUS_INVALID_ARGUMENT;
	}

	const uint8_t tx_buffer[] = {
	FLASH_CMD_READ_STATUS_REG1, 0x00U };

	uint8_t rx_buffer[2];

	if (flash_spi_transfer(tx_buffer, sizeof(tx_buffer), rx_buffer,
			sizeof(rx_buffer)) != HAL_OK) {

		return FLASH_STATUS_SPI_ERROR;
	}

	/*
	 * rx_buffer[0] corresponds to the command phase; the register is in
	 * rx_buffer[1].
	 */
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

	*write_enabled = (status_register & FLASH_STATUS_REG1_WEL_BIT) != 0U;

	return FLASH_STATUS_OK;
}

/*
 * Send Write Enable and verify the Flash actually accepted it, instead of
 * blindly starting a program/erase after just sending the command.
 */
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

	*busy = (status_register & FLASH_STATUS_REG1_BUSY_BIT) != 0U;

	return FLASH_STATUS_OK;
}

/*
 * Page Program / Erase return before the Flash has actually finished
 * modifying cells; BUSY stays set until then. timeout guards against the
 * device never clearing BUSY.
 */

static FlashStatus_t flash_wait_while_busy(uint32_t timeout) {
	FlashStatus_t flash_status;
	uint8_t status_register;
	uint32_t start_time = HAL_GetTick();

	while (1) {

		flash_status = flash_read_status_register(&status_register);

		if (flash_status != FLASH_STATUS_OK) {
			return flash_status;
		}

		if ((status_register & FLASH_STATUS_REG1_BUSY_BIT) == 0U) {
			return FLASH_STATUS_OK;
		}

		if (HAL_GetTick() - start_time >= timeout) {
			return FLASH_STATUS_TIMEOUT;
		}

		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

FlashStatus_t flash_read_data(uint32_t address, uint8_t *data, uint16_t length) {
	if (data == NULL || length == 0U) {
		return FLASH_STATUS_INVALID_ARGUMENT;
	}

	/*
	 * Subtraction form avoids overflow in address + length for large
	 * uint32_t values.
	 */
	if (address >= FLASH_SIZE_BYTES || length > FLASH_SIZE_BYTES - address) {

		return FLASH_STATUS_INVALID_ADDRESS;
	}

	const uint8_t tx_buffer[] = {
	FLASH_CMD_READ_DATA, (uint8_t) (address >> 16), (uint8_t) (address >> 8),
			(uint8_t) address };

	if (flash_spi_transfer(tx_buffer, sizeof(tx_buffer), data, length)
			!= HAL_OK) {

		return FLASH_STATUS_SPI_ERROR;
	}

	return FLASH_STATUS_OK;
}

FlashStatus_t flash_write_data(uint32_t address, const uint8_t *data,
		uint16_t length) {
	FlashStatus_t flash_status;

	if (data == NULL || length == 0U) {
		return FLASH_STATUS_INVALID_ARGUMENT;
	}

	if (address >= FLASH_SIZE_BYTES || length > FLASH_SIZE_BYTES - address) {

		return FLASH_STATUS_INVALID_ADDRESS;
	}

	/*
	 * Page Program cannot cross a 256-byte page boundary - the logger
	 * guarantees every record fits in one page.
	 */
	if ((address & 0xFFU) + length > FLASH_PAGE_SIZE) {
		return FLASH_STATUS_PAGE_OVERFLOW;
	}

	const uint8_t tx_buffer[] = {
	FLASH_CMD_PAGE_PROGRAM, (uint8_t) (address >> 16), (uint8_t) (address >> 8),
			(uint8_t) address };

	flash_status = flash_write_enable_and_verify();

	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	if (flash_spi_transmit_command_and_data(tx_buffer, sizeof(tx_buffer), data,
			length) != HAL_OK) {

		return FLASH_STATUS_SPI_ERROR;
	}

	return flash_wait_while_busy(
	FLASH_PAGE_PROGRAM_TIMEOUT_MS);
}

FlashStatus_t flash_erase_sector(uint32_t address) {
	if (address >= FLASH_SIZE_BYTES || (address % FLASH_SECTOR_SIZE) != 0U) {

		return FLASH_STATUS_INVALID_ADDRESS;
	}

	const uint8_t tx_buffer[] = {
	FLASH_CMD_SECTOR_ERASE, (uint8_t) (address >> 16), (uint8_t) (address >> 8),
			(uint8_t) address };

	FlashStatus_t flash_status;

	flash_status = flash_write_enable_and_verify();

	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	if (flash_spi_transmit(tx_buffer, sizeof(tx_buffer)) != HAL_OK) {

		return FLASH_STATUS_SPI_ERROR;
	}

	return flash_wait_while_busy(
	FLASH_SECTOR_ERASE_TIMEOUT_MS);
}

FlashStatus_t flash_erase_chip(void) {
	FlashStatus_t flash_status;

	const uint8_t tx_buffer[] = {
	FLASH_CMD_CHIP_ERASE };

	flash_status = flash_write_enable_and_verify();

	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	if (flash_spi_transmit(tx_buffer, sizeof(tx_buffer)) != HAL_OK) {

		return FLASH_STATUS_SPI_ERROR;
	}

	/*
	 * Intentionally blocking here - a rare service operation, and
	 * FlashLoggerTask's priority (1, lowest of the app tasks) means it
	 * won't hold up Modbus/Alarm/MQTT while it runs.
	 */
	return flash_wait_while_busy(
	FLASH_CHIP_ERASE_TIMEOUT_MS);
}
