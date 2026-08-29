/*
 * app_queues.c
 *
 *  Created on: Jul 15, 2026
 *      Author: tomas
 */

#include "app_queues.h"
#include "queue.h"
#include "alarm_manager_task.h"
#include "flash_logger_task.h"
#include "modbus_poller_task.h"
#include "flash_logger.h"

QueueHandle_t modbusToAlarmQueue;
QueueHandle_t modbusToMqttQueue;
QueueHandle_t alarmToMqttQueue;
QueueHandle_t alarmToFlashQueue;
QueueHandle_t flashToAlarmQueue;
QueueHandle_t flashCommandQueue;
QueueSetHandle_t flashQueueSet;

/*
 * Length-1 queues hold "latest state" (producer uses xQueueOverwrite()).
 * alarmToFlashQueue is length 5 because FlashLoggerTask may be temporarily
 * unable to consume events, e.g. during a long erase.
 */
#define MODBUS_TO_ALARM_QUEUE_LENGTH    1
#define MODBUS_TO_MQTT_QUEUE_LENGTH     1
#define ALARM_TO_MQTT_QUEUE_LENGTH      1
#define ALARM_TO_FLASH_QUEUE_LENGTH     5
#define FLASH_TO_ALARM_QUEUE_LENGTH     1
#define FLASH_COMMAND_QUEUE_LENGTH      1

/*
 * A queue set's length must be at least the sum of the capacities of the
 * queues that belong to it.
 */
#define FLASH_QUEUE_SET_LENGTH \
		(ALARM_TO_FLASH_QUEUE_LENGTH + FLASH_COMMAND_QUEUE_LENGTH)

/*
 * All queues use static allocation: storage buffer for the items plus a
 * StaticQueue_t for FreeRTOS's internal control block.
 */
static uint8_t modbusToAlarmQueueStorage[MODBUS_TO_ALARM_QUEUE_LENGTH
		* sizeof(MeasurementRecord_t)];

static StaticQueue_t modbusToAlarmQueueControlBlock;

static uint8_t modbusToMqttQueueStorage[MODBUS_TO_MQTT_QUEUE_LENGTH
		* sizeof(MeasurementRecord_t)];

static StaticQueue_t modbusToMqttQueueControlBlock;

static uint8_t alarmToMqttQueueStorage[ALARM_TO_MQTT_QUEUE_LENGTH
		* sizeof(MqttAlarmState_t)];

static StaticQueue_t alarmToMqttQueueControlBlock;

static uint8_t alarmToFlashQueueStorage[ALARM_TO_FLASH_QUEUE_LENGTH
		* sizeof(FlashRecord_t)];

static StaticQueue_t alarmToFlashQueueControlBlock;

static uint8_t flashToAlarmQueueStorage[FLASH_TO_ALARM_QUEUE_LENGTH
		* sizeof(FlashLoggerAlarmFault_t)];

static StaticQueue_t flashToAlarmQueueControlBlock;

static uint8_t flashCommandQueueStorage[FLASH_COMMAND_QUEUE_LENGTH
		* sizeof(UartCommandFrame_t)];

static StaticQueue_t flashCommandQueueControlBlock;

void AppQueuesInit(void) {

	modbusToAlarmQueue = xQueueCreateStatic(MODBUS_TO_ALARM_QUEUE_LENGTH,
			sizeof(MeasurementRecord_t), modbusToAlarmQueueStorage,
			&modbusToAlarmQueueControlBlock);

	modbusToMqttQueue = xQueueCreateStatic(MODBUS_TO_MQTT_QUEUE_LENGTH,
			sizeof(MeasurementRecord_t), modbusToMqttQueueStorage,
			&modbusToMqttQueueControlBlock);

	alarmToMqttQueue = xQueueCreateStatic(ALARM_TO_MQTT_QUEUE_LENGTH,
			sizeof(MqttAlarmState_t), alarmToMqttQueueStorage,
			&alarmToMqttQueueControlBlock);

	alarmToFlashQueue = xQueueCreateStatic(ALARM_TO_FLASH_QUEUE_LENGTH,
			sizeof(FlashRecord_t), alarmToFlashQueueStorage,
			&alarmToFlashQueueControlBlock);

	flashToAlarmQueue = xQueueCreateStatic(FLASH_TO_ALARM_QUEUE_LENGTH,
			sizeof(FlashLoggerAlarmFault_t), flashToAlarmQueueStorage,
			&flashToAlarmQueueControlBlock);

	flashCommandQueue = xQueueCreateStatic(FLASH_COMMAND_QUEUE_LENGTH,
			sizeof(UartCommandFrame_t), flashCommandQueueStorage,
			&flashCommandQueueControlBlock);

	/*
	 * FlashLoggerTask needs to react to two sources of work (a record to
	 * write, or a READ/ERASE command) - the queue set lets it block until
	 * either becomes ready instead of polling both.
	 */
	flashQueueSet = xQueueCreateSet(FLASH_QUEUE_SET_LENGTH);

	xQueueAddToSet(alarmToFlashQueue, flashQueueSet);

	xQueueAddToSet(flashCommandQueue, flashQueueSet);
}
