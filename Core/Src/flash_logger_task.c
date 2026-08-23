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
	EntryRecordAddress_t flash_log_address = { 0 };
	EntryRecordAddress_t measurement_record_address = { 0 };
	FlashStatus_t flash_status;
	uint8_t alarm_counter = 0;
	FlashLoggerAlarmFault_t alarm_fault = ALARM_OK;
	UartCommandFrame_t uart_command_frame;
	QueueSetMemberHandle_t activeQueue;
	FlashCommand_t flash_command;
	uint8_t records_to_read = 0;

	flash_status = flash_logger_init(&flash_log_address,
			&measurement_record_address);

	if (flash_status != FLASH_STATUS_OK) {
		Error_Handler();
	}

	for (;;) {

		activeQueue = xQueueSelectFromSet(flashQueueSet, portMAX_DELAY);

		if (activeQueue == alarmToFlashQueue) {
			send_uart2_message("Alarm to FLASH!\r\n");

			xQueueReceive(alarmToFlashQueue, &flash_record, 0);

			if (alarm_fault != STORAGE_FAULT) {
				flash_status = flash_logger_write_record(&flash_record,
						&flash_log_address, &measurement_record_address);

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

				if (flash_status == FLASH_STATUS_OK
						&& alarm_fault == COMMUNICATION_FAULT) {
					alarm_counter = 0;
					alarm_fault = ALARM_OK;
					xQueueOverwrite(flashToAlarmQueue, &alarm_fault);
				}
			}
		} else if (activeQueue == flashCommandQueue) {
//			send_uart2_message("Command to FLASH!\r\n");

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

				} else {
					flash_status = flash_logger_init(&flash_log_address,
							&measurement_record_address);

					if (flash_status != FLASH_STATUS_OK) {
						Error_Handler();
					}

					send_uart2_message("History erased!\r\n");
				}

			} else if (flash_command == FLASH_CMD_READ) {
//				send_uart2_message("Reading...\r\n");

				flash_status = flash_logger_send_history(
						&measurement_record_address, records_to_read);
//				send_uart2_message("DONE!\r\n");
				if (flash_status != FLASH_STATUS_OK) {

					send_uart2_message(
							"Something went wrong. Your history couldn't be sent back.\r\n");

				}
			}else if( flash_command == FLASH_CMD_UNKNOWN){
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
