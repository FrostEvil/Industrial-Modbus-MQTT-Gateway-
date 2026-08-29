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

/*
 * UART2 carries service/debug messages only - not part of the normal
 * application data path.
 */
static void send_uart2_message(const char *message) {

	snprintf(uart2_tx_buffer, sizeof(uart2_tx_buffer), "%s", message);

	HAL_UART_Transmit(&huart2, (uint8_t*) uart2_tx_buffer,
			strlen(uart2_tx_buffer),
			HAL_MAX_DELAY);
}

/*
 * Handle one incoming measurement record: write it to Flash and update the
 * fault/alarm state based on the result. Does nothing if writes are
 * currently blocked by STORAGE_FAULT - clearing that requires an explicit
 * ERASE HISTORY command (see handle_uart_command()).
 */
static void handle_new_record(EntryRecordAddress_t *measurement_record_address,
		FlashLoggerHealth_t *flash_logger_health) {
	FlashRecord_t flash_record;
	FlashStatus_t flash_status;

	xQueueReceive(alarmToFlashQueue, &flash_record, 0);

	if (flash_logger_health->alarm_fault == STORAGE_FAULT) {
		return;
	}

	flash_status = flash_logger_write_record(&flash_record,
			measurement_record_address);

	/*
	 * Communication-level problems are treated as temporary at first.
	 */
	if (flash_status == FLASH_STATUS_TIMEOUT
			|| flash_status == FLASH_STATUS_SPI_ERROR
			|| flash_status == FLASH_STATUS_WRITE_NOT_ENABLED) {

		flash_logger_health->alarm_counter++;

		flash_logger_health->alarm_fault =
				(flash_logger_health->alarm_counter >= 5) ?
						STORAGE_FAULT : COMMUNICATION_FAULT;

		xQueueOverwrite(flashToAlarmQueue, &flash_logger_health->alarm_fault);
	}

	/*
	 * Not temporary - escalate immediately.
	 */
	if (flash_status == FLASH_STATUS_INVALID_ADDRESS
			|| flash_status == FLASH_STATUS_PAGE_OVERFLOW
			|| flash_status == FLASH_STATUS_INVALID_ARGUMENT
			|| flash_status == FLASH_STATUS_LOG_CORRUPTED) {

		flash_logger_health->alarm_fault = STORAGE_FAULT;

		xQueueOverwrite(flashToAlarmQueue, &flash_logger_health->alarm_fault);
	}

	if (flash_status == FLASH_STATUS_FULL) {

		flash_logger_health->alarm_fault = STORAGE_FAULT;

		xQueueOverwrite(flashToAlarmQueue, &flash_logger_health->alarm_fault);

		send_uart2_message(
				"History FULL, you need to take action and erase flash history.\r\n");
	}

	/*
	 * A successful write after COMMUNICATION_FAULT confirms Flash is
	 * working again. STORAGE_FAULT is NOT cleared here - that needs an
	 * explicit erase.
	 */
	if (flash_status == FLASH_STATUS_OK
			&& flash_logger_health->alarm_fault == COMMUNICATION_FAULT) {

		flash_logger_health->alarm_counter = 0;
		flash_logger_health->alarm_fault = ALARM_OK;

		xQueueOverwrite(flashToAlarmQueue, &flash_logger_health->alarm_fault);
	}
}

/*
 * Handle one command received over the service UART: ERASE HISTORY,
 * READ <n>, or anything unrecognised.
 */
static void handle_uart_command(
		EntryRecordAddress_t *measurement_record_address,
		FlashLoggerHealth_t *flash_logger_health) {
	UartCommandFrame_t uart_command_frame;
	FlashCommand_t flash_command;
	uint8_t records_to_read = 0;
	FlashStatus_t flash_status;

	xQueueReceive(flashCommandQueue, &uart_command_frame, 0);

	flash_command = parse_command(&uart_command_frame, &records_to_read);

	if (flash_command == FLASH_CMD_ERASE) {

		send_uart2_message(
				"Erasing history... This may take up to 200 seconds.\r\n");

		/*
		 * Synchronous on purpose: a rare service operation, and keeping it
		 * inside this task means Flash always has a single owner.
		 */
		flash_status = flash_logger_erase_history();

		if (flash_status != FLASH_STATUS_OK) {

			send_uart2_message(
					"Something went wrong. Your history couldn't be erased.\r\n");
			return;
		}

		flash_status = flash_logger_init(measurement_record_address);

		if (flash_status != FLASH_STATUS_OK) {
			Error_Handler();
		}

		/*
		 * A clean erase clears both temporary and persistent Flash faults.
		 */
		flash_logger_health->alarm_counter = 0;
		flash_logger_health->alarm_fault = ALARM_OK;

		xQueueOverwrite(flashToAlarmQueue, &flash_logger_health->alarm_fault);

		send_uart2_message("History erased!\r\n");

	} else if (flash_command == FLASH_CMD_READ) {

		flash_status = flash_logger_send_history(measurement_record_address,
				records_to_read);

		if (flash_status != FLASH_STATUS_OK) {

			send_uart2_message(
					"Something went wrong. Your history couldn't be sent back.\r\n");
		}

	} else if (flash_command == FLASH_CMD_UNKNOWN) {

		send_uart2_message("Unknown command.\r\n");
	}
}

void FlashLoggerTask(void *argument) {

	/*
	 * Address of the next Flash record to write. Recovered from existing
	 * Flash contents on every start by flash_logger_init(), since the
	 * write position itself is not stored anywhere non-volatile.
	 *
	 * FLASH_STATUS_FULL from flash_logger_init() sets this to
	 * FLASH_END_ADDRESS.
	 */
	EntryRecordAddress_t measurement_record_address = { 0 };

	FlashStatus_t flash_status;
	FlashLoggerHealth_t flash_logger_health;
	flash_logger_health.alarm_counter = 0;
	flash_logger_health.alarm_fault = ALARM_OK;

	/*
	 * Lets this task block on either source of work: alarmToFlashQueue
	 * (new record) or flashCommandQueue (READ/ERASE command).
	 */
	QueueSetMemberHandle_t activeQueue;

	flash_status = flash_logger_init(&measurement_record_address);

	if (flash_status == FLASH_STATUS_FULL) {

		/*
		 * A full log is a valid operating state, not an init failure.
		 */
		flash_logger_health.alarm_fault = STORAGE_FAULT;

		xQueueOverwrite(flashToAlarmQueue, &flash_logger_health.alarm_fault);

	} else if (flash_status != FLASH_STATUS_OK) {

		/*
		 * Anything else means the logger could not reconstruct a usable
		 * state - unrecoverable at this layer.
		 */
		Error_Handler();
	}

	for (;;) {

		/*
		 * Blocks here without consuming CPU until either queue in the set
		 * has something.
		 */
		activeQueue = xQueueSelectFromSet(flashQueueSet,
		portMAX_DELAY);

		if (activeQueue == alarmToFlashQueue) {

			handle_new_record(&measurement_record_address,
					&flash_logger_health);

		} else if (activeQueue == flashCommandQueue) {

			handle_uart_command(&measurement_record_address,
					&flash_logger_health);
		}
	}
}

void FlashLoggerTask_Init(void) {

	flashLoggerTaskHandle = xTaskCreateStatic(FlashLoggerTask, "FlashLogger",
	FLASH_LOGGER_TASK_STACK_SIZE,
	NULL, 1, flashLoggerTaskStack, &flashLoggerTaskTCB);
}
