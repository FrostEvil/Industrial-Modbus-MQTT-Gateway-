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
 * Called by FreeRTOS when a task stack overflow is detected.
 *
 * This hook is intended only for fatal error diagnostics. The task name is
 * sent through UART2 and the LED is turned on before interrupts are disabled
 * and the system is stopped.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
	HAL_GPIO_WritePin(
	LD2_GPIO_Port,
	LD2_Pin, GPIO_PIN_SET);

	HAL_UART_Transmit(&huart2, (uint8_t*) pcTaskName, strlen(pcTaskName),
	HAL_MAX_DELAY);

	/*
	 * A stack overflow can leave the system in an undefined state.
	 * Stop normal execution instead of allowing the application to continue.
	 */
	__disable_irq();

	for (;;) {
	}
}

/*
 * Create all application tasks.
 *
 * The tasks use static allocation and are created before the FreeRTOS
 * scheduler is started.
 */
void AppTasksInit(void) {
	ModbusPollerTask_Init();
	AlarmManagerTask_Init();
	FlashLoggerTask_Init();
	MqttPublisherTask_Init();
}
