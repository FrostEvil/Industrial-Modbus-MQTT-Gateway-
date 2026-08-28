/*
 * flash_logger_task.c
 *
 *  Created on: Jul 17, 2026
 *      Author: tomas
 */

#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "app_queues.h"
#include "queue.h"
#include "string.h"
#include "flash_logger.h"
#include "flash_logger_task.h"

#define STARTING_ADDRESS 0x000000

TaskHandle_t flashLoggerTaskHandle;

#define FLASH_LOGGER_TASK_STACK_SIZE 512
static StackType_t flashLoggerTaskStack[FLASH_LOGGER_TASK_STACK_SIZE];
static StaticTask_t flashLoggerTaskTCB;

static char uart2_tx_buffer[64];

static void send_uart2_message(const char *message) {

	snprintf((char*) uart2_tx_buffer, sizeof(uart2_tx_buffer), "%s", message);
	HAL_UART_Transmit(&huart2, (uint8_t*) uart2_tx_buffer,
			strlen(uart2_tx_buffer), HAL_MAX_DELAY);
}

void FlashLoggerTask(void *argument) {
	FlashRecord_t flash_record;
	EntryRecordAddress_t measurement_record_address = { 0 };
	FlashStatus_t flash_status;
	uint8_t alarm_counter = 0;
	FlashLoggerAlarmFault_t alarm_fault = ALARM_OK;
	UartCommandFrame_t uart_command_frame;
	QueueSetMemberHandle_t activeQueue;
	FlashCommand_t flash_command;
	uint8_t records_to_read = 0;

	flash_status = flash_logger_init(&measurement_record_address);

	if (flash_status != FLASH_STATUS_OK) {
		Error_Handler();
	}

	for (;;) {

		activeQueue = xQueueSelectFromSet(flashQueueSet, portMAX_DELAY);

		if (activeQueue == alarmToFlashQueue) {

			xQueueReceive(alarmToFlashQueue, &flash_record, 0);

			if (alarm_fault != STORAGE_FAULT) {

				flash_status = flash_logger_write_record(&flash_record,
						&measurement_record_address);

				if (flash_status == FLASH_STATUS_TIMEOUT
						|| flash_status == FLASH_STATUS_SPI_ERROR
						|| flash_status == FLASH_STATUS_WRITE_NOT_ENABLED) {

					alarm_counter++;

					if (alarm_counter == 5) {
						alarm_fault = STORAGE_FAULT;
						xQueueOverwrite(flashToAlarmQueue, &alarm_fault);
					} else {
						alarm_fault = COMMUNICATION_FAULT;
						xQueueOverwrite(flashToAlarmQueue, &alarm_fault);
					}
				}

				if (flash_status == FLASH_STATUS_INVALID_ADDRESS
						|| flash_status == FLASH_STATUS_PAGE_OVERFLOW
						|| flash_status == FLASH_STATUS_INVALID_ARGUMENT
						|| flash_status == FLASH_STATUS_LOG_CORRUPTED) {
					alarm_fault = STORAGE_FAULT;
					xQueueOverwrite(flashToAlarmQueue, &alarm_fault);
				}

				if (flash_status == FLASH_STATUS_FULL) {
					/* Capacity genuinely exhausted, not a hardware/logic
					 * fault -- but the position is frozen on the last,
					 * already-written slot, so any further write attempt
					 * here would corrupt it (NOR flash ANDs new data into
					 * existing bytes rather than overwriting). Reusing
					 * STORAGE_FAULT is the simplest way to guarantee no
					 * further flash_logger_write_record() calls happen
					 * until the user erases history -- if a distinct LED/
					 * message ("full, needs erase" vs "hardware fault")
					 * turns out to matter, split this into its own
					 * FlashLoggerAlarmFault_t value later.
					 *
					 * TODO: decide the exact UX -- auto-erase on reaching
					 * FULL, or warn well before it (e.g. ~90% capacity)
					 * and let the user erase manually after reading out
					 * the history first. */
					alarm_fault = STORAGE_FAULT;
					xQueueOverwrite(flashToAlarmQueue, &alarm_fault);
					send_uart2_message(
							"History FULL, you need to take action and erase flash history.\r\n");
				}

				if (flash_status == FLASH_STATUS_OK
						&& alarm_fault == COMMUNICATION_FAULT) {
					alarm_counter = 0;
					alarm_fault = ALARM_OK;
					xQueueOverwrite(flashToAlarmQueue, &alarm_fault);
				}
			}
		} else if (activeQueue == flashCommandQueue) {

			xQueueReceive(flashCommandQueue, &uart_command_frame, 0);

			flash_command = parse_command(&uart_command_frame,
					&records_to_read);

			if (flash_command == FLASH_CMD_ERASE) {

				send_uart2_message(
						"Erasing history... This may take up to 200 seconds.\r\n");

				flash_status = flash_logger_erase_history();

				if (flash_status != FLASH_STATUS_OK) {

					send_uart2_message(
							"Something went wrong. Your history couldn't be erased.\r\n");

				}

				flash_status = flash_logger_init(&measurement_record_address);

				if (flash_status != FLASH_STATUS_OK) {
					Error_Handler();
				}

				/* A successful erase always clears whatever fault the
				 * logger was previously in -- the chip is now fully
				 * blank and flash_logger_init() just proved it reads
				 * back cleanly. */
				alarm_counter = 0;
				alarm_fault = ALARM_OK;
				xQueueOverwrite(flashToAlarmQueue, &alarm_fault);

				send_uart2_message("History erased!\r\n");
			}

			else if (flash_command == FLASH_CMD_READ) {
				flash_status = flash_logger_send_history(
						&measurement_record_address, records_to_read);

				if (flash_status != FLASH_STATUS_OK) {

					send_uart2_message(
							"Something went wrong. Your history couldn't be sent back.\r\n");

				}
			} else if (flash_command == FLASH_CMD_UNKNOWN) {
				send_uart2_message("Unknown command.\r\n");
			}

		}

	}
}

void FlashLoggerTask_Init(void) {
	flashLoggerTaskHandle = xTaskCreateStatic(FlashLoggerTask, "FlashLogger",
	FLASH_LOGGER_TASK_STACK_SIZE, NULL, 1, flashLoggerTaskStack,
			&flashLoggerTaskTCB);
}
