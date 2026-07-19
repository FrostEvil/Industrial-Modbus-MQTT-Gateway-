/*
 * app_queues.c
 *
 *  Created on: Jul 15, 2026
 *      Author: tomas
 */

#include "app_queues.h"
#include "alarm_manager_task.h"

// Handle definitions - declared in the header as extern, defined here once.
QueueHandle_t modbusToAlarmQueue;
QueueHandle_t modbusToFlashLoggerQueue;
QueueHandle_t modbusToMqttQueue;
QueueHandle_t alarmToMqttQueue;
// Depth 1: Alarm Manager only ever cares about the MOST RECENT reading,
// not a history of past ones - see xQueueOverwrite() usage in
// AlarmManagerTask (Etap 3.5), which requires exactly this depth.
#define MODBUS_TO_ALARM_QUEUE_LENGTH 1
#define MODBUS_TO_FLASH_LOGGER_QUEUE_LENGTH 5
#define MODBUS_TO_MQTT_QUEUE_LENGTH 1
#define ALARM_TO_MQTT_QUEUE_LENGTH 1

// Static allocation, consistent with the tasks (xTaskCreateStatic):
// no heap usage, no risk of a failed runtime allocation.
static uint8_t modbusToAlarmQueueStorage[MODBUS_TO_ALARM_QUEUE_LENGTH
		* sizeof(MeasurementMessage_t)];
static StaticQueue_t modbusToAlarmQueueControlBlock;

static uint8_t modbusToFlashLoggerQueueStorage[MODBUS_TO_FLASH_LOGGER_QUEUE_LENGTH
		* sizeof(MeasurementMessage_t)];
static StaticQueue_t modbusToFlashLoggerQueueControlBlock;

static uint8_t modbusToMqttQueueStorage[MODBUS_TO_MQTT_QUEUE_LENGTH
		* sizeof(MeasurementMessage_t)];
static StaticQueue_t modbusToMqTTQueueControlBlock;

static uint8_t alarmToMqttQueueStorage[ALARM_TO_MQTT_QUEUE_LENGTH
		* sizeof(AlarmState_t)];
static StaticQueue_t alarmToMqttQueueControlBlock;

void AppQueuesInit(void) {
	modbusToAlarmQueue = xQueueCreateStatic(MODBUS_TO_ALARM_QUEUE_LENGTH, // liczba elementow (glebokosc)
			sizeof(MeasurementMessage_t),// rozmiar POJEDYNCZEGO elementu
			modbusToAlarmQueueStorage,// Twoj bufor na dane
			&modbusToAlarmQueueControlBlock// Twoja struktura kontrolna kolejki
			);
	modbusToFlashLoggerQueue = xQueueCreateStatic(
			MODBUS_TO_FLASH_LOGGER_QUEUE_LENGTH, sizeof(MeasurementMessage_t),
			modbusToFlashLoggerQueueStorage,
			&modbusToFlashLoggerQueueControlBlock);

	modbusToMqttQueue = xQueueCreateStatic(MODBUS_TO_MQTT_QUEUE_LENGTH,
			sizeof(MeasurementMessage_t), modbusToMqttQueueStorage,
			&modbusToMqTTQueueControlBlock);

	alarmToMqttQueue = xQueueCreateStatic(ALARM_TO_MQTT_QUEUE_LENGTH,
			sizeof(AlarmState_t), alarmToMqttQueueStorage,
			&alarmToMqttQueueControlBlock);

}
