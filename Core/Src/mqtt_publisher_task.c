/*
 * mqtt_publisher_task.c
 *
 *  Created on: Jul 19, 2026
 *      Author: tomas
 */

#include "FreeRTOS.h"
#include "task.h"
#include "app_queues.h"
#include "alarm_manager_task.h"
#include "usart.h"
#include <string.h>

TaskHandle_t mqttPublisherTaskHandle;

#define MQTT_PUBLISHER_TASK_STACK_SIZE 128
static StackType_t mqttPublisherStack[MQTT_PUBLISHER_TASK_STACK_SIZE];
static StaticTask_t mqttPublisherTaskTCB;

static void SendAlarmMessage(AlarmState_t alarm_state) {
	const char *alarm_message = NULL;

	if (alarm_state == ALARM_MEASUREMENT) {
		alarm_message = "Measurement alarm!\r\n";
	} else if (alarm_state == ALARM_COMMUNICATION) {
		alarm_message = "Communication alarm!\r\n";
	}
	if (alarm_message != NULL){
		HAL_UART_Transmit(&huart2, (uint8_t*) alarm_message, strlen(alarm_message),
		HAL_MAX_DELAY);
	}

}

void MqttPublisherTask(void *argument) {
	MeasurementRecord_t measurement_record;
	AlarmState_t alarm_state;
//	const char message[] = "Publication of data\r\n";

	for (;;) {

		if (xQueueReceive(alarmToMqttQueue, &alarm_state,
				pdMS_TO_TICKS(5000)) == pdTRUE) {
//			SendAlarmMessage(alarm_state);

		}
		xQueueReceive(modbusToMqttQueue, &measurement_record,
		portMAX_DELAY);

//		HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message),
//		HAL_MAX_DELAY);

	}
}

void MqttPublisherTask_Init(void) {
	mqttPublisherTaskHandle = xTaskCreateStatic(MqttPublisherTask,
			"MQTT Publisher", MQTT_PUBLISHER_TASK_STACK_SIZE, NULL, 2,
			mqttPublisherStack, &mqttPublisherTaskTCB);
}
