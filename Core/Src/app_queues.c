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

/*
 * Queue handles are declared as extern in app_queues.h and are defined
 * here. Each handle identifies one inter-task communication channel.
 */
QueueHandle_t modbusToAlarmQueue;
QueueHandle_t modbusToMqttQueue;
QueueHandle_t alarmToMqttQueue;
QueueHandle_t alarmToFlashQueue;
QueueHandle_t flashToAlarmQueue;
QueueHandle_t flashCommandQueue;
QueueSetHandle_t flashQueueSet;

/*
 * Queue length is chosen according to the type of information being passed.
 *
 * Queues with length 1 represent the latest state/value. The producer uses
 * xQueueOverwrite(), so an older value is replaced when a new one arrives.
 *
 * alarmToFlashQueue has length 5 because FlashLoggerTask may temporarily
 * be unable to consume events, for example during a long erase operation.
 */
#define MODBUS_TO_ALARM_QUEUE_LENGTH    1
#define MODBUS_TO_MQTT_QUEUE_LENGTH     1
#define ALARM_TO_MQTT_QUEUE_LENGTH      1
#define ALARM_TO_FLASH_QUEUE_LENGTH     5
#define FLASH_TO_ALARM_QUEUE_LENGTH     1
#define FLASH_COMMAND_QUEUE_LENGTH      1

/*
 * The queue set contains alarmToFlashQueue and flashCommandQueue.
 * Its length must be at least the sum of the capacities of all queues
 * belonging to the set.
 */
#define FLASH_QUEUE_SET_LENGTH \
		(ALARM_TO_FLASH_QUEUE_LENGTH + FLASH_COMMAND_QUEUE_LENGTH)

/*
 * All queues use static allocation.
 *
 * The storage buffer contains the actual queue items, while StaticQueue_t
 * stores the internal control information maintained by FreeRTOS.
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
	/*
	 * Each queue stores complete structures rather than pointers.
	 * FreeRTOS copies the structure into the queue when an item is sent.
	 */
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
	 * FlashLoggerTask needs to react to two different sources of work:
	 *
	 * alarmToFlashQueue -> record to write
	 * flashCommandQueue -> READ / ERASE command
	 *
	 * Queue Set allows the task to block until either queue becomes ready.
	 */
	flashQueueSet = xQueueCreateSet(FLASH_QUEUE_SET_LENGTH);

	xQueueAddToSet(alarmToFlashQueue, flashQueueSet);

	xQueueAddToSet(flashCommandQueue, flashQueueSet);
}
