/*
 * flash_logger_task.c
 *
 *  Created on: Jul 17, 2026
 *      Author: tomas
 */

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

void FlashLoggerTask(void *argument) {
	FlashRecord_t flash_record;
	EntryRecordAddress_t flash_log_address;
	EntryRecordAddress_t measurement_record_address;
	FlashStatus_t flash_status;
	uint8_t alarm_counter = 0;
	FlashLoggerAlarmFault_t alarm_fault = ALARM_OK;

	flash_status = flash_logger_init(&flash_log_address,
			&measurement_record_address);

	if (flash_status != FLASH_STATUS_OK) {
		Error_Handler();
	}

	for (;;) {
		xQueueReceive(alarmToFlashQueue, &flash_record,
		portMAX_DELAY);

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

	}
}

void FlashLoggerTask_Init(void) {
	flashLoggerTaskHandle = xTaskCreateStatic(FlashLoggerTask, "FlashLogger",
	FLASH_LOGGER_TASK_STACK_SIZE, NULL, 1, flashLoggerTaskStack,
			&flashLoggerTaskTCB);
}
