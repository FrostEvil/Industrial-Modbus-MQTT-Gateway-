/*
 * flash_logger.c
 *
 * Linear event log persistence for measurement records.
 *
 * Data sectors are used exactly once each, in strictly increasing order
 * (0, 1, 2, 3, ...) -- never reused, never rotated. This relies entirely
 * on the whole chip being fully erased before logging starts (either a
 * genuinely fresh chip, or after an explicit "erase history" command) --
 * every sector a write ever moves into is therefore guaranteed to already
 * be blank (0xFF), so no erase-ahead is needed during normal operation.
 *
 * Records are always aligned to fixed, page-relative slots -- a record
 * never spans a page boundary, and each page has a few leftover padding
 * bytes at the end that never hold a record (PAGE_SIZE isn't evenly
 * divisible by RECORD_SIZE).
 *
 * Recovery after any reset is a straightforward linear scan from sector 0:
 * find the first sector that isn't completely FULL (either genuinely
 * EMPTY, or ACTIVE with some records already in it) and resume writing
 * there. No index, no rotation, no wraparound -- once every sector is
 * FULL, the log is full; the only way to reclaim space is an explicit
 * "erase history" command.
 *
 * flash_logger_init() reconstructs where writing left off after a reset.
 * flash_logger_write_record() is the runtime entry point, called once per
 * logged event.
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

#define SECTOR_SIZE 4096
#define SECTOR_COUNT 4096
#define RECORD_SIZE 20
#define LAST_RECORD_PAGE_OFFSET 16
#define LAST_RECORD_INDEX_START (SECTOR_SIZE - RECORD_SIZE - LAST_RECORD_PAGE_OFFSET)
#define STARTING_ADDRESS 0x000000
#define PAGE_SIZE 256
#define PAGE_COUNT 16
#define RECORD_PER_PAGE (PAGE_SIZE / RECORD_SIZE)
#define MAX_READ_RECORDS 20

/* Shared scratch buffer for reading a whole sector at once. Every function
 * below reads fresh into it immediately before using it -- none of them
 * rely on what a *previous* call left behind. Keep it that way; several
 * bugs in earlier versions of this file came from breaking that rule. */
static uint8_t sector_buffer[SECTOR_SIZE];

static uint8_t uart2_rx_buffer[64];
static char uart2_tx_buffer[192];

static FlashRecord_t history_buffer[MAX_READ_RECORDS];

/**
 * True if every byte currently in sector_buffer is 0xFF -- i.e. the sector
 * it was just read from is genuinely, completely erased with nothing
 * written to it at all.
 */
static uint8_t is_sector_fully_empty(void) {
	uint32_t free_slot_counter = 0;
	for (uint32_t i = 0; i < SECTOR_SIZE; i++) {
		if (sector_buffer[i] == 0xFF) {
			free_slot_counter++;
		}
	}
	return free_slot_counter == SECTOR_SIZE;
}

/**
 * True if the LAST possible record slot in sector_buffer (at
 * LAST_RECORD_INDEX_START) is entirely 0xFF -- i.e. there's still room for
 * at least one more record in this sector.
 *
 * This is deliberately the only slot checked (not a full sector scan): a
 * data sector can only ever be EMPTY, ACTIVE, or FULL (records are always
 * written contiguously from the start, so "is the very last slot free" is
 * enough to tell ACTIVE apart from FULL).
 *
 * Known accepted risk: this checks whether every byte in the slot equals
 * 0xFF, not whether a real record living there is well-formed. A record
 * whose raw bytes happen to be legitimately all-0xFF (unlikely, but not
 * impossible for arbitrary float data) could in principle be misread here.
 */
static uint8_t is_last_record_slot_empty(void) {
	uint8_t free_slot_counter = 0;
	for (uint8_t i = 0; i < RECORD_SIZE; i++) {
		if (sector_buffer[LAST_RECORD_INDEX_START + i] == 0xFF) {
			free_slot_counter++;
		}
	}
	return free_slot_counter == RECORD_SIZE;
}

/**
 * True if the RECORD_SIZE bytes at the start of sector_buffer are all
 * 0xFF -- used when reading history backwards to recognize "this slot was
 * never written", i.e. the end of real history has been reached.
 */
static uint8_t is_record_empty(void) {
	uint8_t free_slot_counter = 0;
	for (uint8_t i = 0; i < RECORD_SIZE; i++) {
		if (sector_buffer[i] == 0xFF) {
			free_slot_counter++;
		}
	}

	return (free_slot_counter == RECORD_SIZE) ? 1 : 0;
}

/**
 * Classifies the DATA sector at `address` into EMPTY, ACTIVE, or FULL.
 *
 * SECTOR_DAMAGED is part of the enum for symmetry with earlier project
 * history, but this function never produces it: a data sector is written
 * strictly front-to-back and never erased mid-use, so there's no
 * "interrupted erase" state to detect.
 */
static FlashStatus_t update_data_sector_status(const uint32_t address,
		SectorStatus_t *data_sector) {
	FlashStatus_t flash_status;

	flash_status = flash_read_data(address, sector_buffer,
	SECTOR_SIZE);

	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	if (is_sector_fully_empty()) {
		*data_sector = SECTOR_EMPTY;
		return FLASH_STATUS_OK;
	}

	*data_sector = is_last_record_slot_empty() ? SECTOR_ACTIVE : SECTOR_FULL;

	return FLASH_STATUS_OK;
}

/**
 * Given a data-sector write address already known (measurement_record_
 * address->address), derives page_index and record_index_in_page directly
 * via bit masking instead of scanning -- page/record position always live
 * entirely within the low 12 bits of any data-sector address (since a
 * sector base address always has its low 12 bits clear), so this is safe
 * to apply to a full, absolute Flash address, not just a within-sector
 * offset.
 */
static void decode_data_write_position(
		EntryRecordAddress_t *measurement_record_address) {
	measurement_record_address->record_index_in_page =
			(measurement_record_address->address & 0xFF) / RECORD_SIZE;
	measurement_record_address->page_index =
			(measurement_record_address->address >> 8) & 0x0F;
}

/**
 * Boot-time recovery: finds where writing should resume by scanning
 * sectors 0, 1, 2, ... in order and stopping at the first one that isn't
 * completely FULL.
 *
 *   - EMPTY sector found: nothing has ever been written to it. Falls
 *     through to the same scan used for ACTIVE below, which correctly
 *     resolves this to that sector's very first slot (offset 0) -- no
 *     special-casing needed, EMPTY is just "ACTIVE with zero records".
 *   - ACTIVE sector found: scans page by page, record slot by record slot
 *     (skipping each page's trailing padding bytes, since a record never
 *     spans a page boundary) for the first slot that's entirely 0xFF.
 *   - Every sector is FULL: the log has no remaining capacity anywhere.
 *     Returns FLASH_STATUS_FULL -- the caller (FlashLoggerTask) decides
 *     how to react (block further writes, prompt for "erase history").
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
		/* Loop ran through every sector without finding one that wasn't
		 * FULL -- genuinely out of capacity, not a logic error.
		 *
		 * TODO: decide the exact UX here -- auto-erase on reaching FULL,
		 * or warn well before it (e.g. at ~90% capacity) and let the user
		 * erase manually after reading out the history first. */
		return FLASH_STATUS_FULL;
	}

	/* sector_status is ACTIVE (or EMPTY, handled identically): scan for
	 * the first entirely-0xFF record slot. free_slot_counter reaching
	 * RECORD_SIZE exactly when k hits the last byte of a slot means every
	 * byte of that slot was 0xFF -- return immediately on the first such
	 * slot found. */
	uint32_t free_slot_counter = 0;

	for (uint8_t i = 0; i < PAGE_COUNT; i++) {
		for (uint8_t j = 0; j < RECORD_PER_PAGE; j++) {
			for (uint8_t k = 0; k < RECORD_SIZE; k++) {
				if (sector_buffer[i * PAGE_SIZE + j * RECORD_SIZE + k]
						== 0xFF) {
					free_slot_counter++;
				}
				if (k == RECORD_SIZE - 1 && free_slot_counter == RECORD_SIZE) {
					measurement_address->address = sector_address
							+ (i * PAGE_SIZE + j * RECORD_SIZE);
					return FLASH_STATUS_OK;
				}
			}
			free_slot_counter = 0;
		}
	}

	/* Every fixed slot was occupied despite the sector being classified
	 * ACTIVE -- shouldn't happen given how ACTIVE is determined (the last
	 * slot being free), but kept as a defensive catch-all. */
	return FLASH_STATUS_LOG_CORRUPTED;
}

static void uart_command_terminate(UartCommandFrame_t *uart_command_flash) {
	if (uart_command_flash == NULL) {
		return;
	}

	while (uart_command_flash->length > 0) {
		uint8_t last = uart_command_flash->data[uart_command_flash->length - 1];

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
	if (uart_command_flash == NULL) {
		return FLASH_CMD_UNKNOWN;
	}

	uart_command_terminate(uart_command_flash);
	char *command = (char*) uart_command_flash->data;

	for (uint16_t i = 0; command[i] != '\0'; i++) {
		command[i] = (char) tolower((unsigned char ) command[i]);
	}

	if ((strcmp(command, "erase history") == 0)) {
		return FLASH_CMD_ERASE;
	}

	else if (((strncmp(command, "read", 4) == 0)) && command[4] == ' ') {

		char *end;
		unsigned long records_count = strtoul(&command[5], &end, 10);

		if (command[5] == '-') {
			return FLASH_CMD_UNKNOWN;
		}

		if (end == &command[5]) {
			return FLASH_CMD_UNKNOWN;
		}

		if (records_count == 0) {
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

/**
 * Public entry point: reconstructs, after any reset, exactly where writing
 * should resume in the linear data log.
 */
FlashStatus_t flash_logger_init(
		EntryRecordAddress_t *measurement_record_address) {
	FlashStatus_t flash_status;

	HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_rx_buffer,
			sizeof(uart2_rx_buffer));

	flash_status = resolve_data_write_slot(measurement_record_address);

	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	decode_data_write_position(measurement_record_address);

	return FLASH_STATUS_OK;
}

/**
 * Public entry point: writes one measurement record to its already-known
 * position, then advances that position for next time -- across record,
 * page, and sector boundaries. Sectors are never reused or erased here --
 * they're guaranteed blank from the last full-chip erase, so a sector
 * transition is just moving the write pointer forward, nothing more.
 *
 * Relies entirely on flash_logger_init() having already recovered a
 * correct starting position: this function only ever needs to execute its
 * own straightforward, uninterrupted sequence once per call, trusting that
 * any interruption will be corrected by the next boot's recovery pass.
 */
FlashStatus_t flash_logger_write_record(const FlashRecord_t *flash_record,
		EntryRecordAddress_t *measurement_record_address) {
	FlashStatus_t flash_status;

	flash_status = flash_write_data(measurement_record_address->address,
			(const uint8_t*) flash_record, sizeof(FlashRecord_t));

	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	/* Last record of the last page: the sector is now full -- move to the
	 * next data sector. */
	if ((measurement_record_address->page_index == PAGE_COUNT - 1)
			&& (measurement_record_address->record_index_in_page
					== RECORD_PER_PAGE - 1)) {
		if ((measurement_record_address->address >> 12) == SECTOR_COUNT - 1) {
			/* That was the last record of the very last sector -- no
			 * further sector to move to. The position is deliberately left
			 * unchanged (still pointing at this last, already-written
			 * slot); the caller must not attempt another write here. */
			return FLASH_STATUS_FULL;
		}
		measurement_record_address->address =
				((measurement_record_address->address >> 12) + 1) << 12;
		measurement_record_address->page_index = 0;
		measurement_record_address->record_index_in_page = 0;

		return FLASH_STATUS_OK;
	}

	/* Last record of a non-final page: skip the page's trailing padding
	 * bytes and move to the next page. */
	if ((measurement_record_address->page_index < PAGE_COUNT - 1)
			&& measurement_record_address->record_index_in_page
					== RECORD_PER_PAGE - 1) {

		measurement_record_address->address += RECORD_SIZE
				+ LAST_RECORD_PAGE_OFFSET;
		measurement_record_address->page_index += 1;
		measurement_record_address->record_index_in_page = 0;

		return FLASH_STATUS_OK;
	}

	/* Normal case: next record slot, same page. */
	measurement_record_address->address += RECORD_SIZE;
	measurement_record_address->record_index_in_page += 1;

	return FLASH_STATUS_OK;
}

void flash_logger_rx_event(HAL_UART_RxEventTypeTypeDef event, uint16_t Size) {
	UartCommandFrame_t uart_command_frame;

	if (event == HAL_UART_RXEVENT_IDLE) {
		if (Size >= sizeof(uart_command_frame.data)) {
			Size = sizeof(uart_command_frame.data) - 1;
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

FlashStatus_t flash_logger_erase_history() {
	return flash_erase_chip();
}

/**
 * Reads up to records_to_read history entries, walking backwards from the
 * current write position. No wraparound: sector 0 is always genuinely the
 * oldest possible data, so hitting it (or an empty slot) simply ends the
 * walk early with however many real records were found.
 */
static FlashStatus_t flash_logger_read_history(
		const EntryRecordAddress_t *measurement_record_address,
		uint8_t records_to_read, uint8_t *read_records) {
	EntryRecordAddress_t current_record_read;
	FlashStatus_t flash_status;
	FlashRecord_t flash_record;
	*read_records = 0;

	/* Nothing has ever been written anywhere. */
	if (measurement_record_address->address == STARTING_ADDRESS) {
		return FLASH_STATUS_OK;
	}

	current_record_read.address = measurement_record_address->address;
	current_record_read.page_index = measurement_record_address->page_index;
	current_record_read.record_index_in_page =
			measurement_record_address->record_index_in_page;

	for (uint8_t i = 0; i < records_to_read; i++) {

		/* Already at the very first record ever written (sector 0, slot
		 * 0) -- there is nowhere further back to go. Stop here, same as
		 * running into a genuinely empty slot. */
		if (current_record_read.address == STARTING_ADDRESS
				&& current_record_read.page_index == 0
				&& current_record_read.record_index_in_page == 0) {
			break;
		}

		if (current_record_read.page_index == 0
				&& current_record_read.record_index_in_page == 0) {
			/* Crossing back into the previous sector's last slot. */
			current_record_read.address -= (RECORD_SIZE
					+ LAST_RECORD_PAGE_OFFSET);
			current_record_read.page_index = PAGE_COUNT - 1;
			current_record_read.record_index_in_page = RECORD_PER_PAGE - 1;

		} else if (current_record_read.record_index_in_page == 0
				&& current_record_read.page_index != 0) {
			/* Crossing back into the previous page's last slot. */
			current_record_read.address -= (RECORD_SIZE
					+ LAST_RECORD_PAGE_OFFSET);
			current_record_read.page_index -= 1;
			current_record_read.record_index_in_page = RECORD_PER_PAGE - 1;
		} else {
			/* Normal case: previous slot, same page. */
			current_record_read.address -= RECORD_SIZE;
			current_record_read.record_index_in_page -= 1;
		}

		flash_status = flash_read_data(current_record_read.address,
				sector_buffer, RECORD_SIZE);
		if (flash_status != FLASH_STATUS_OK) {
			return flash_status;
		}

		if (!is_record_empty()) {
			memcpy(&flash_record.voltage, &sector_buffer[0],
					sizeof(flash_record.voltage));
			memcpy(&flash_record.current, &sector_buffer[4],
					sizeof(flash_record.current));
			memcpy(&flash_record.temperature, &sector_buffer[8],
					sizeof(flash_record.temperature));
			memcpy(&flash_record.timestamp_ms, &sector_buffer[12],
					sizeof(flash_record.timestamp_ms));
			memcpy(&flash_record.crc, &sector_buffer[16],
					sizeof(flash_record.crc));
			memcpy(&flash_record.trigger_channel, &sector_buffer[18],
					sizeof(flash_record.trigger_channel));

			uint16_t crc = modbus_crc16(sector_buffer,
					offsetof(FlashRecord_t, crc));

			if (crc != flash_record.crc) {
				flash_record.trigger_channel |= 0x08;
			}

			*read_records += 1;

			history_buffer[i] = flash_record;
		} else {
			break;
		}
	}
	return FLASH_STATUS_OK;
}

FlashStatus_t flash_logger_send_history(
		const EntryRecordAddress_t *measurement_record_address,
		uint8_t records_to_read) {
	uint8_t read_records = 0;
	FlashStatus_t flash_status;

	flash_status = flash_logger_read_history(measurement_record_address,
			records_to_read, &read_records);

	if (flash_status != FLASH_STATUS_OK) {
		return flash_status;
	}

	if (read_records == 0) {
		snprintf(uart2_tx_buffer, sizeof(uart2_tx_buffer),
				"No data to read.\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart2_tx_buffer,
				strlen(uart2_tx_buffer), HAL_MAX_DELAY);
		return flash_status;
	}

	for (uint8_t i = 0; i < read_records; i++) {
		if (history_buffer[i].trigger_channel & 0x08) {
			snprintf(uart2_tx_buffer, sizeof(uart2_tx_buffer),
					"Event detected, but data unreadable (interrupted write)\r\n");
		} else {
			snprintf(uart2_tx_buffer, sizeof(uart2_tx_buffer),
					"U:%.1fV, I: %.1fA, T: %.1f\xE2\x84\x83, time:%lums",
					history_buffer[i].voltage, history_buffer[i].current,
					history_buffer[i].temperature,
					history_buffer[i].timestamp_ms);

			if (history_buffer[i].trigger_channel & 0x01) {
				snprintf(uart2_tx_buffer + strlen(uart2_tx_buffer),
						sizeof(uart2_tx_buffer) - strlen(uart2_tx_buffer),
						", voltage changed state");
			}

			if (history_buffer[i].trigger_channel & 0x02) {
				snprintf(uart2_tx_buffer + strlen(uart2_tx_buffer),
						sizeof(uart2_tx_buffer) - strlen(uart2_tx_buffer),
						", current changed state");
			}

			if (history_buffer[i].trigger_channel & 0x04) {
				snprintf(uart2_tx_buffer + strlen(uart2_tx_buffer),
						sizeof(uart2_tx_buffer) - strlen(uart2_tx_buffer),
						", temperature changed state");
			}
			snprintf(uart2_tx_buffer + strlen(uart2_tx_buffer),
					sizeof(uart2_tx_buffer) - strlen(uart2_tx_buffer), "\r\n");
		}

		HAL_UART_Transmit(&huart2, (uint8_t*) uart2_tx_buffer,
				strlen(uart2_tx_buffer), HAL_MAX_DELAY);
	}

	if (read_records != records_to_read) {
		snprintf(uart2_tx_buffer, sizeof(uart2_tx_buffer),
				"Only %u record(s) available.\r\n", read_records);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart2_tx_buffer,
				strlen(uart2_tx_buffer), HAL_MAX_DELAY);
	} else if (read_records == MAX_READ_RECORDS) {
		snprintf(uart2_tx_buffer, sizeof(uart2_tx_buffer),
				"Maximum number of records read (%u).\r\n",
				(unsigned int) MAX_READ_RECORDS);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart2_tx_buffer,
				strlen(uart2_tx_buffer), HAL_MAX_DELAY);
	}
	return FLASH_STATUS_OK;
}
