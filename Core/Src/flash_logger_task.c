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

	/*
	 * UART2 is used only for service/debug messages from the Flash logger.
	 * These messages are not part of the normal application data path.
	 */
	snprintf(uart2_tx_buffer, sizeof(uart2_tx_buffer), "%s", message);

	HAL_UART_Transmit(&huart2, (uint8_t*) uart2_tx_buffer,
			strlen(uart2_tx_buffer),
			HAL_MAX_DELAY);
}

void FlashLoggerTask(void *argument) {

	FlashRecord_t flash_record;

	/*
	 * Contains the address at which the next Flash record should be stored.
	 * flash_logger_init() determines the correct position after startup
	 * or after history is erased.
	 *
	 * When the log is full, flash_logger_init() sets the address to
	 * FLASH_END_ADDRESS and returns FLASH_STATUS_FULL.
	 */
	EntryRecordAddress_t measurement_record_address = { 0 };

	FlashStatus_t flash_status;

	/*
	 * Number of consecutive communication-level Flash errors.
	 *
	 * Short-lived errors produce COMMUNICATION_FAULT. Five consecutive
	 * errors escalate to STORAGE_FAULT and stop further automatic writes.
	 */
	uint8_t alarm_counter = 0;

	FlashLoggerAlarmFault_t alarm_fault = ALARM_OK;

	UartCommandFrame_t uart_command_frame;

	/*
	 * Queue set allows this task to wait for two different sources of work:
	 *
	 * alarmToFlashQueue -> new record to store
	 * flashCommandQueue -> READ / ERASE command
	 */
	QueueSetMemberHandle_t activeQueue;

	FlashCommand_t flash_command;
	uint8_t records_to_read = 0;

	/*
	 * Recover the current write position from the existing Flash contents.
	 * This is needed after every restart because the write position itself
	 * is kept in RAM rather than stored separately in non-volatile memory.
	 */
	flash_status = flash_logger_init(&measurement_record_address);

	if (flash_status == FLASH_STATUS_FULL) {

		/*
		 * A full log is a valid operating state, not an initialisation
		 * failure. Stop automatic writes and report the condition to
		 * AlarmManagerTask.
		 */
		alarm_fault = STORAGE_FAULT;

		xQueueOverwrite(flashToAlarmQueue, &alarm_fault);

	} else if (flash_status != FLASH_STATUS_OK) {

		/*
		 * Any other initialisation error means that the Flash logger
		 * could not reconstruct a usable state.
		 */
		Error_Handler();
	}

	for (;;) {

		/*
		 * Wait until either a new record or a user command is available.
		 * The task remains blocked without consuming CPU until something
		 * appears in one of the queues belonging to the queue set.
		 */
		activeQueue = xQueueSelectFromSet(flashQueueSet,
		portMAX_DELAY);

		if (activeQueue == alarmToFlashQueue) {

			/*
			 * xQueueSelectFromSet() already confirmed that the queue
			 * contains an item, so the receive itself does not need to wait.
			 */
			xQueueReceive(alarmToFlashQueue, &flash_record, 0);

			/*
			 * Once STORAGE_FAULT is reached, automatic writes are blocked.
			 * The user must erase the history before logging can resume.
			 */
			if (alarm_fault != STORAGE_FAULT) {

				flash_status = flash_logger_write_record(&flash_record,
						&measurement_record_address);

				/*
				 * These errors indicate communication-level problems with
				 * the Flash device. A small number of consecutive errors
				 * is treated as temporary.
				 */
				if (flash_status == FLASH_STATUS_TIMEOUT
						|| flash_status == FLASH_STATUS_SPI_ERROR
						|| flash_status == FLASH_STATUS_WRITE_NOT_ENABLED) {

					alarm_counter++;

					if (alarm_counter >= 5) {

						/*
						 * Five consecutive failures indicate that the
						 * problem is persistent. Further writes are blocked.
						 */
						alarm_fault = STORAGE_FAULT;

						xQueueOverwrite(flashToAlarmQueue, &alarm_fault);

					} else {

						alarm_fault = COMMUNICATION_FAULT;

						xQueueOverwrite(flashToAlarmQueue, &alarm_fault);
					}
				}

				/*
				 * These statuses indicate an invalid argument, address,
				 * page transition or corrupted log state. Unlike temporary
				 * communication failures, they immediately escalate to
				 * STORAGE_FAULT.
				 */
				if (flash_status == FLASH_STATUS_INVALID_ADDRESS
						|| flash_status == FLASH_STATUS_PAGE_OVERFLOW
						|| flash_status == FLASH_STATUS_INVALID_ARGUMENT
						|| flash_status == FLASH_STATUS_LOG_CORRUPTED) {

					alarm_fault = STORAGE_FAULT;

					xQueueOverwrite(flashToAlarmQueue, &alarm_fault);
				}

				if (flash_status == FLASH_STATUS_FULL) {

					/*
					 * The log has consumed all available storage.
					 * This is not a hardware failure, but automatic writes
					 * must stop until the user erases the history.
					 */
					alarm_fault = STORAGE_FAULT;

					xQueueOverwrite(flashToAlarmQueue, &alarm_fault);

					send_uart2_message(
							"History FULL, you need to take action and erase flash history.\r\n");
				}

				/*
				 * A successful write after a temporary communication fault
				 * confirms that the Flash is working again.
				 *
				 * STORAGE_FAULT is intentionally not cleared here. That
				 * state requires an explicit ERASE HISTORY operation.
				 */
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

				/*
				 * Erasing the entire chip is intentionally synchronous.
				 * It is a rare service operation and keeping it inside
				 * this task ensures that Flash has only one owner.
				 */
				flash_status = flash_logger_erase_history();

				if (flash_status != FLASH_STATUS_OK) {

					send_uart2_message(
							"Something went wrong. Your history couldn't be erased.\r\n");

				} else {

					/*
					 * Reinitialise the logger after erase so that the write
					 * position points to the first record slot again.
					 */
					flash_status = flash_logger_init(
							&measurement_record_address);

					if (flash_status != FLASH_STATUS_OK) {
						Error_Handler();
					}

					/*
					 * A successful erase restores a known clean storage
					 * state, so temporary and persistent Flash faults can
					 * be cleared.
					 */
					alarm_counter = 0;
					alarm_fault = ALARM_OK;

					xQueueOverwrite(flashToAlarmQueue, &alarm_fault);

					send_uart2_message("History erased!\r\n");
				}

			} else if (flash_command == FLASH_CMD_READ) {

				/*
				 * Read history without changing the stored data.
				 */
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
	FLASH_LOGGER_TASK_STACK_SIZE,
	NULL, 1, flashLoggerTaskStack, &flashLoggerTaskTCB);
}
