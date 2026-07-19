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

TaskHandle_t flashLoggerTaskHandle;

#define FLASH_LOGGER_TASK_STACK_SIZE 128
static StackType_t flashLoggerTaskStack[FLASH_LOGGER_TASK_STACK_SIZE];
static StaticTask_t flashLoggerTaskTCB;

void FlashLoggerTask(void *argument) {
	MeasurementMessage_t measurement_message;
	const char message[] = "Received data\r\n";
	for (;;) {
		xQueueReceive(modbusToFlashLoggerQueue, &measurement_message,
		portMAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message),
		HAL_MAX_DELAY);
	}
}

void FlashLoggerTask_Init(void) {
	flashLoggerTaskHandle = xTaskCreateStatic(FlashLoggerTask, "FlashLogger",
	FLASH_LOGGER_TASK_STACK_SIZE, NULL, 1, flashLoggerTaskStack,
			&flashLoggerTaskTCB);
}
