/*
 * flash_logger.c
 *
 *  Created on: Jul 25, 2026
 *      Author: tomas
 */

#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>

#include "flash.h"
#include "flash_logger.h"
#include "app_queues.h"
#include "usart.h"
#include "flash_logger_task.h"
#include "modbus_protocol.h"
#include <stdbool.h>

#define SECTOR_SIZE                 4096U
#define SECTOR_COUNT                4096U
#define PAGE_SIZE                   256U
#define PAGE_COUNT                  16U
#define RECORD_SIZE                 20U
#define RECORD_PER_PAGE             (PAGE_SIZE / RECORD_SIZE)
#define LAST_RECORD_PAGE_OFFSET     16U
#define MAX_READ_RECORDS            20U

#define STARTING_ADDRESS            0x00000000UL
#define FLASH_END_ADDRESS           (STARTING_ADDRESS + (SECTOR_SIZE * SECTOR_COUNT))

static uint8_t sector_buffer[SECTOR_SIZE];
static uint8_t uart2_rx_buffer[64];
static char uart2_tx_buffer[192];

static FlashRecord_t history_buffer[MAX_READ_RECORDS];

/*
 * Shared "is this range still blank?" check, used for a whole sector, the
 * last record slot in a sector, and a single record - all three are really
 * the same question, just over a different range.
 */
static bool is_region_erased(const uint8_t *buf, uint32_t offset, uint32_t len) {
	for (uint32_t i = 0; i < len; i++) {
		if (buf[offset + i] != 0xFF) {
			return false;
		}
	}
	return true;
}

static FlashStatus_t update_data_sector_status(const uint32_t address,
		SectorStatus_t *data_sector) {
	FlashStatus_t flash_status;

	flash_status = flash_read_data(address, sector_buffer,
	SECTOR_SIZE);

	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	if (is_region_erased(sector_buffer, 0, SECTOR_SIZE)) {

		*data_sector = SECTOR_EMPTY;
		return FLASH_STATUS_OK;
	}

	const uint32_t last_record_offset = SECTOR_SIZE - LAST_RECORD_PAGE_OFFSET
			- RECORD_SIZE;

	*data_sector =
			is_region_erased(sector_buffer, last_record_offset, RECORD_SIZE) ?
					SECTOR_ACTIVE : SECTOR_FULL;

	return FLASH_STATUS_OK;
}

static void decode_data_write_position(
		EntryRecordAddress_t *measurement_record_address) {
	measurement_record_address->record_index_in_page =
			(measurement_record_address->address & 0xFFU) / RECORD_SIZE;

	measurement_record_address->page_index =
			(measurement_record_address->address >> 8) & 0x0FU;
}

/*
 * TODO: worst case (log nearly full at restart) this reads every sector
 * (up to SECTOR_COUNT * SECTOR_SIZE = 16 MB over SPI) once at startup to
 * find the write position. One-time cost at init, not per-record, so it's
 * an acceptable tradeoff here - just worth writing down in the README so
 * "startup gets slower as the log fills up" isn't a surprise later.
 */
static FlashStatus_t resolve_data_write_slot(
		EntryRecordAddress_t *measurement_address) {
	FlashStatus_t flash_status;
	SectorStatus_t sector_status = SECTOR_FULL;
	uint32_t sector_address = 0;

	for (uint16_t i = 0; i < SECTOR_COUNT; i++) {

		sector_address = (uint32_t) i * SECTOR_SIZE;

		flash_status = update_data_sector_status(sector_address,
				&sector_status);

		if (flash_status != FLASH_STATUS_OK) {
			return flash_status;
		}

		if (sector_status == SECTOR_ACTIVE || sector_status == SECTOR_EMPTY) {
			break;
		}
	}

	if (sector_status == SECTOR_FULL) {

		/*
		 * measurement_address always means "next write position";
		 * FLASH_END_ADDRESS means there isn't one because the log is full.
		 */
		measurement_address->address = FLASH_END_ADDRESS;
		measurement_address->page_index = PAGE_COUNT;
		measurement_address->record_index_in_page = RECORD_PER_PAGE;

		return FLASH_STATUS_FULL;
	}

	/*
	 * sector_buffer already holds the full sector read by
	 * update_data_sector_status() above - scan it for the first fully
	 * erased record slot.
	 */
	for (uint8_t page = 0; page < PAGE_COUNT; page++) {

		for (uint8_t record = 0; record < RECORD_PER_PAGE; record++) {

			uint32_t slot_offset = (page * PAGE_SIZE) + (record * RECORD_SIZE);

			if (is_region_erased(sector_buffer, slot_offset, RECORD_SIZE)) {
				measurement_address->address = sector_address + slot_offset;

				return FLASH_STATUS_OK;
			}
		}
	}

	return FLASH_STATUS_LOG_CORRUPTED;
}

static void uart_command_terminate(UartCommandFrame_t *uart_command_flash) {
	if (uart_command_flash == NULL) {
		return;
	}

	while (uart_command_flash->length > 0U) {

		uint8_t last = uart_command_flash->data[uart_command_flash->length - 1U];

		if (last == '\r' || last == '\n') {
			uart_command_flash->length--;
		} else {
			break;
		}
	}

	if (uart_command_flash->length < sizeof(uart_command_flash->data)) {

		uart_command_flash->data[uart_command_flash->length] = '\0';
	}
}

FlashCommand_t parse_command(UartCommandFrame_t *uart_command_flash,
		uint8_t *records_to_read) {
	if (uart_command_flash == NULL || records_to_read == NULL) {
		return FLASH_CMD_UNKNOWN;
	}

	uart_command_terminate(uart_command_flash);

	char *command = (char*) uart_command_flash->data;

	for (uint16_t i = 0; command[i] != '\0'; i++) {
		command[i] = (char) tolower((unsigned char ) command[i]);
	}

	if (strcmp(command, "erase history") == 0) {
		return FLASH_CMD_ERASE;
	}

	if (strncmp(command, "read", 4) == 0 && command[4] == ' ') {

		/*
		 * Index 5 = right after the space checked above - if the "read "
		 * prefix format ever changes, this offset needs to change with it.
		 */
		char *number_start = &command[5];
		char *end;

		if (*number_start == '-') {
			return FLASH_CMD_UNKNOWN;
		}

		unsigned long records_count = strtoul(number_start, &end, 10);

		/*
		 * strtoul() stops at the first non-numeric character - only
		 * trailing spaces/tabs are accepted after the number.
		 */
		if (end == number_start) {
			return FLASH_CMD_UNKNOWN;
		}

		while (*end == ' ' || *end == '\t') {
			end++;
		}

		if (*end != '\0') {
			return FLASH_CMD_UNKNOWN;
		}

		if (records_count == 0UL) {
			return FLASH_CMD_UNKNOWN;
		}

		if (records_count <= MAX_READ_RECORDS) {
			*records_to_read = (uint8_t) records_count;
		} else {
			*records_to_read = MAX_READ_RECORDS;
		}

		return FLASH_CMD_READ;
	}

	return FLASH_CMD_UNKNOWN;
}

FlashStatus_t flash_logger_init(
		EntryRecordAddress_t *measurement_record_address) {
	FlashStatus_t flash_status;

	HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_rx_buffer,
			sizeof(uart2_rx_buffer));

	flash_status = resolve_data_write_slot(measurement_record_address);

	/*
	 * FULL is a valid end-of-capacity state, not an init error.
	 */
	if (flash_status == FLASH_STATUS_FULL) {
		return FLASH_STATUS_FULL;
	}

	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	decode_data_write_position(measurement_record_address);

	return FLASH_STATUS_OK;
}

FlashStatus_t flash_logger_write_record(const FlashRecord_t *flash_record,
		EntryRecordAddress_t *measurement_record_address) {
	FlashStatus_t flash_status;

	if (flash_record == NULL || measurement_record_address == NULL) {
		return FLASH_STATUS_INVALID_ARGUMENT;
	}

	if (measurement_record_address->address >= FLASH_END_ADDRESS) {
		return FLASH_STATUS_FULL;
	}

	flash_status = flash_write_data(measurement_record_address->address,
			(const uint8_t*) flash_record, sizeof(FlashRecord_t));

	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	/*
	 * Last record of the last page -> the whole chip is now consumed.
	 */
	if ((measurement_record_address->page_index == PAGE_COUNT - 1U)
			&& (measurement_record_address->record_index_in_page
					== RECORD_PER_PAGE - 1U)) {

		measurement_record_address->address = FLASH_END_ADDRESS;
		measurement_record_address->page_index = PAGE_COUNT;
		measurement_record_address->record_index_in_page =
		RECORD_PER_PAGE;

		return FLASH_STATUS_FULL;
	}

	/*
	 * Last record in the current page - skip the unused padding before
	 * the next page.
	 */
	if ((measurement_record_address->page_index < PAGE_COUNT - 1U)
			&& (measurement_record_address->record_index_in_page
					== RECORD_PER_PAGE - 1U)) {

		measurement_record_address->address +=
		RECORD_SIZE + LAST_RECORD_PAGE_OFFSET;

		measurement_record_address->page_index += 1U;
		measurement_record_address->record_index_in_page = 0U;

		return FLASH_STATUS_OK;
	}

	/*
	 * Normal case: next record in the same page.
	 */
	measurement_record_address->address += RECORD_SIZE;
	measurement_record_address->record_index_in_page += 1U;

	return FLASH_STATUS_OK;
}

void flash_logger_rx_event(HAL_UART_RxEventTypeTypeDef event, uint16_t Size) {
	UartCommandFrame_t uart_command_frame;

	if (event == HAL_UART_RXEVENT_IDLE) {

		if (Size >= sizeof(uart_command_frame.data)) {
			Size = sizeof(uart_command_frame.data) - 1U;
		}

		uart_command_frame.length = Size;

		memcpy(uart_command_frame.data, uart2_rx_buffer, Size);

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		xQueueSendFromISR(flashCommandQueue, &uart_command_frame,
				&xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_rx_buffer,
			sizeof(uart2_rx_buffer));
}

FlashStatus_t flash_logger_erase_history(void) {
	return flash_erase_chip();
}

static FlashStatus_t flash_logger_read_history(
		const EntryRecordAddress_t *measurement_record_address,
		uint8_t records_to_read, uint8_t *read_records) {
	EntryRecordAddress_t current_record_read;
	FlashStatus_t flash_status;
	FlashRecord_t flash_record;

	*read_records = 0U;

	/*
	 * No record has ever been written.
	 */
	if (measurement_record_address->address == STARTING_ADDRESS) {
		return FLASH_STATUS_OK;
	}

	/*
	 * When the log is full, measurement_record_address points to
	 * FLASH_END_ADDRESS rather than the last record - the latest record is
	 * the last slot of the last sector instead.
	 */
	if (measurement_record_address->address == FLASH_END_ADDRESS) {

		current_record_read.address =
		FLASH_END_ADDRESS - (RECORD_SIZE + LAST_RECORD_PAGE_OFFSET);

		current_record_read.page_index = PAGE_COUNT - 1U;
		current_record_read.record_index_in_page =
		RECORD_PER_PAGE - 1U;

	} else {

		/*
		 * Normally this points to the next empty slot - step back one slot
		 * before the first read.
		 */
		current_record_read = *measurement_record_address;

		if (current_record_read.address == STARTING_ADDRESS
				&& current_record_read.page_index == 0U
				&& current_record_read.record_index_in_page == 0U) {
			return FLASH_STATUS_OK;
		}

		if (current_record_read.page_index == 0U
				&& current_record_read.record_index_in_page == 0U) {

			/*
			 * Cross into the previous sector's last record.
			 */
			current_record_read.address -=
			RECORD_SIZE + LAST_RECORD_PAGE_OFFSET;

			current_record_read.page_index = PAGE_COUNT - 1U;
			current_record_read.record_index_in_page =
			RECORD_PER_PAGE - 1U;

		} else if (current_record_read.record_index_in_page == 0U) {

			/*
			 * Cross into the previous page's last record.
			 */
			current_record_read.address -=
			RECORD_SIZE + LAST_RECORD_PAGE_OFFSET;

			current_record_read.page_index -= 1U;
			current_record_read.record_index_in_page =
			RECORD_PER_PAGE - 1U;

		} else {

			/*
			 * Previous record in the same page.
			 */
			current_record_read.address -= RECORD_SIZE;
			current_record_read.record_index_in_page -= 1U;
		}
	}

	for (uint8_t i = 0; i < records_to_read; i++) {

		flash_status = flash_read_data(current_record_read.address,
				sector_buffer,
				RECORD_SIZE);

		if (flash_status != FLASH_STATUS_OK) {
			return flash_status;
		}

		if (is_region_erased(sector_buffer, 0, RECORD_SIZE)) {
			break;
		}

		/*
		 * sector_buffer holds exactly what flash_write_data() wrote in
		 * flash_logger_write_record() - a raw dump of FlashRecord_t. Reading
		 * it back the same way keeps this in sync automatically if the
		 * struct changes, no manual byte offsets to maintain here. The
		 * _Static_assert next to the struct definition in flash_logger.h
		 * still catches it at compile time if FlashRecord_t's size ever
		 * drifts from RECORD_SIZE.
		 */
		memcpy(&flash_record, sector_buffer, sizeof(FlashRecord_t));

		/*
		 * CRC covers everything before the crc field, including
		 * trigger_channel and the padding byte.
		 */
		uint16_t crc = modbus_crc16(sector_buffer,
				offsetof(FlashRecord_t, crc));

		if (crc != flash_record.crc) {

			/*
			 * Bit 3 marks "invalid CRC" when history is returned to the
			 * user.
			 */
			flash_record.trigger_channel |= 0x08U;
		}

		history_buffer[i] = flash_record;
		*read_records += 1U;

		if (current_record_read.address == STARTING_ADDRESS) {
			break;
		}

		if (current_record_read.page_index == 0U
				&& current_record_read.record_index_in_page == 0U) {

			break;

		} else if (current_record_read.record_index_in_page == 0U) {

			current_record_read.address -=
			RECORD_SIZE + LAST_RECORD_PAGE_OFFSET;

			current_record_read.page_index -= 1U;
			current_record_read.record_index_in_page =
			RECORD_PER_PAGE - 1U;

		} else {

			current_record_read.address -= RECORD_SIZE;
			current_record_read.record_index_in_page -= 1U;
		}
	}

	return FLASH_STATUS_OK;
}

FlashStatus_t flash_logger_send_history(
		const EntryRecordAddress_t *measurement_record_address,
		uint8_t records_to_read) {
	uint8_t read_records = 0U;
	FlashStatus_t flash_status;

	if (measurement_record_address == NULL || records_to_read == 0U) {
		return FLASH_STATUS_INVALID_ARGUMENT;
	}

	if (records_to_read > MAX_READ_RECORDS) {
		records_to_read = MAX_READ_RECORDS;
	}

	flash_status = flash_logger_read_history(measurement_record_address,
			records_to_read, &read_records);

	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	if (read_records == 0U) {

		snprintf(uart2_tx_buffer, sizeof(uart2_tx_buffer),
				"No data to read.\r\n");

		HAL_UART_Transmit(&huart2, (uint8_t*) uart2_tx_buffer,
				strlen(uart2_tx_buffer),
				HAL_MAX_DELAY);

		return FLASH_STATUS_OK;
	}

	/*
	 * NOTE: snprintf() returns how many characters it WOULD have written if
	 * the buffer had been unlimited - not how many actually fit. At this
	 * buffer size (192 B) and with these short, fixed messages, truncation
	 * never happens in practice, so `len` is always accurate here. If this
	 * buffer ever shrinks or the messages grow, `len` could end up larger
	 * than what's really in the buffer, and `uart2_tx_buffer + len` /
	 * `sizeof(uart2_tx_buffer) - len` (unsigned!) would then point/compute
	 * out of bounds. Worth clamping `len` to `sizeof(uart2_tx_buffer) - 1`
	 * after each snprintf() if that ever becomes a real risk.
	 */
	for (uint8_t i = 0; i < read_records; i++) {

		size_t len = 0;

		if (history_buffer[i].trigger_channel & 0x08U) {

			len +=
					snprintf(uart2_tx_buffer, sizeof(uart2_tx_buffer),
							"Event detected, but data unreadable (interrupted write)\r\n");

		} else {

			len += snprintf(uart2_tx_buffer, sizeof(uart2_tx_buffer),
					"U:%.1fV, I: %.1fA, T: %.1f\xE2\x84\x83, time:%lums",
					history_buffer[i].voltage, history_buffer[i].current,
					history_buffer[i].temperature,
					history_buffer[i].timestamp_ms);

			if (history_buffer[i].trigger_channel & 0x01U) {

				len += snprintf(uart2_tx_buffer + len,
						sizeof(uart2_tx_buffer) - len,
						", voltage changed state");
			}

			if (history_buffer[i].trigger_channel & 0x02U) {

				len += snprintf(uart2_tx_buffer + len,
						sizeof(uart2_tx_buffer) - len,
						", current changed state");
			}

			if (history_buffer[i].trigger_channel & 0x04U) {

				len += snprintf(uart2_tx_buffer + len,
						sizeof(uart2_tx_buffer) - len,
						", temperature changed state");
			}

			len += snprintf(uart2_tx_buffer + len,
					sizeof(uart2_tx_buffer) - len, "\r\n");
		}

		HAL_UART_Transmit(&huart2, (uint8_t*) uart2_tx_buffer,
				strlen(uart2_tx_buffer),
				HAL_MAX_DELAY);
	}

	if (read_records != records_to_read) {

		snprintf(uart2_tx_buffer, sizeof(uart2_tx_buffer),
				"Only %u record(s) available.\r\n", read_records);

		HAL_UART_Transmit(&huart2, (uint8_t*) uart2_tx_buffer,
				strlen(uart2_tx_buffer),
				HAL_MAX_DELAY);

	} else if (read_records == MAX_READ_RECORDS) {

		snprintf(uart2_tx_buffer, sizeof(uart2_tx_buffer),
				"Maximum number of records read (%u).\r\n",
				(unsigned int) MAX_READ_RECORDS);

		HAL_UART_Transmit(&huart2, (uint8_t*) uart2_tx_buffer,
				strlen(uart2_tx_buffer),
				HAL_MAX_DELAY);
	}

	return FLASH_STATUS_OK;
}
