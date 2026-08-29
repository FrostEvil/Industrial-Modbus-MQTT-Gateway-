/*
 * app_tasks.c
 *
 *  Created on: Jul 16, 2026
 *      Author: tomas
 */

#include "modbus_poller_task.h"
#include "alarm_manager_task.h"
#include "flash_logger_task.h"
#include "mqtt_publisher_task.h"
#include "cmsis_gcc.h"
#include "gpio.h"
#include "usart.h"
#include <string.h>

/*
 * FreeRTOS calls this on stack overflow. Fatal-error diagnostics only:
 * signal on LED2 + UART2 with the offending task name, then stop - a stack
 * overflow can leave the system in an undefined state, so continuing is
 * not an option.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
	HAL_GPIO_WritePin(
	LD2_GPIO_Port,
	LD2_Pin, GPIO_PIN_SET);

	HAL_UART_Transmit(&huart2, (uint8_t*) pcTaskName, strlen(pcTaskName),
	HAL_MAX_DELAY);

	__disable_irq();

	for (;;) {
	}
}

/*
 * Create all application tasks, statically allocated, before the scheduler
 * starts.
 */
void AppTasksInit(void) {
	ModbusPollerTask_Init();
	AlarmManagerTask_Init();
	FlashLoggerTask_Init();
	MqttPublisherTask_Init();
}
