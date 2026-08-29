/*
 * flash_logger.h
 *
 *  Created on: Jul 25, 2026
 *      Author: tomas
 */

#ifndef FLASH_LOGGER_H_
#define FLASH_LOGGER_H_

#include <stdint.h>
#include "flash.h"
#include "flash_logger_task.h"
#include "usart.h"

/**
 * @brief One event record stored in the external SPI Flash.
 *
 * trigger_channel bitmask:
 *   bit 0 -> voltage state changed
 *   bit 1 -> current state changed
 *   bit 2 -> temperature state changed
 *   bit 3 -> invalid CRC detected while reading history
 *   bits 4-7 unused
 *
 * crc is calculated over all preceding bytes, including trigger_channel and
 * the structure padding before crc - see flash_logger.c for the exact byte
 * offsets used when reading a record back.
 */
typedef struct {
	float voltage;
	float current;
	float temperature;
	uint32_t timestamp_ms;
	uint8_t trigger_channel;
	uint16_t crc;
} FlashRecord_t;

_Static_assert(sizeof(FlashRecord_t) == 20U,
		"FlashRecord_t layout changed - update RECORD_SIZE and the manual "
		"byte offsets in flash_logger_read_history()");

/**
 * @brief Current state of one data sector.
 *
 * EMPTY: only erased bytes. ACTIVE: has records and at least one free slot.
 * FULL: all record slots occupied.
 */
typedef enum {
	SECTOR_EMPTY = 0, SECTOR_ACTIVE, SECTOR_FULL
} SectorStatus_t;

/**
 * @brief Current write position in the linear Flash log.
 *
 * address is the next record slot to write. page_index/record_index_in_page
 * are cached positional info for moving between records/pages/sectors.
 * address == FLASH_END_ADDRESS means the log is full.
 */
typedef struct {
	uint32_t address;
	uint8_t page_index;
	uint8_t record_index_in_page;
} EntryRecordAddress_t;

/**
 * @brief Raw command frame received through the service UART.
 *
 * data is not assumed null-terminated until uart_command_terminate() runs.
 */
typedef struct {
	uint16_t length;
	uint8_t data[64];
} UartCommandFrame_t;

/**
 * @brief Command understood by the Flash logger.
 */
typedef enum {
	FLASH_CMD_UNKNOWN = 0, FLASH_CMD_READ, FLASH_CMD_ERASE
} FlashCommand_t;

/**
 * @brief Recover the current log position after startup or erase by
 * scanning the linear log for the next available record slot.
 *
 * FLASH_STATUS_FULL is a valid result: no further record can be stored
 * until the history is erased.
 */
FlashStatus_t flash_logger_init(
		EntryRecordAddress_t *measurement_record_address);

/**
 * @brief Write one event record to the current log position and advance it.
 *
 * FLASH_STATUS_FULL is returned once the final available slot has been
 * written.
 */
FlashStatus_t flash_logger_write_record(const FlashRecord_t *flash_record,
		EntryRecordAddress_t *measurement_record_address);

/**
 * @brief Handle a UART2 reception event (HAL ReceiveToIdle) and forward
 * completed command frames to the Flash Logger task via a queue.
 */
void flash_logger_rx_event(HAL_UART_RxEventTypeTypeDef event, uint16_t Size);

/**
 * @brief Parse a command received from the service UART.
 *
 * Supported: "erase history", "read <number>" (case-insensitive).
 * The requested record count is capped at MAX_READ_RECORDS.
 */
FlashCommand_t parse_command(UartCommandFrame_t *uart_command_flash,
		uint8_t *records_to_read);

/**
 * @brief Erase the complete Flash history.
 */
FlashStatus_t flash_logger_erase_history(void);

/**
 * @brief Read up to records_to_read history entries (newest first,
 * moving backwards through the linear log) and send them through UART2.
 */
FlashStatus_t flash_logger_send_history(
		const EntryRecordAddress_t *measurement_record_address,
		uint8_t records_to_read);

#endif /* FLASH_LOGGER_H_ */
