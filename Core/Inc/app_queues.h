/*
 * app_queues.h
 *
 *  Created on: Jul 15, 2026
 *      Author: tomas
 */

#ifndef INC_APP_QUEUES_H_
#define INC_APP_QUEUES_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "modbus_protocol.h"

/**
 * @brief Measurement result passed between application tasks.
 *
 * Filled by ModbusPollerTask after each polling cycle. When status is not
 * MODBUS_OK, voltage/current/temperature are zero and must not be treated
 * as valid readings - consumers check status first.
 */
typedef struct {
	float voltage;
	float current;
	float temperature;
	ModbusStatus_t status;
} MeasurementRecord_t;

/*
 * Queue handles shared by the tasks. Names describe the direction:
 *
 * modbusToAlarmQueue   ModbusPollerTask -> AlarmManagerTask
 * modbusToMqttQueue    ModbusPollerTask -> MqttPublisherTask
 * alarmToMqttQueue     AlarmManagerTask -> MqttPublisherTask
 * alarmToFlashQueue    AlarmManagerTask -> FlashLoggerTask
 * flashToAlarmQueue    FlashLoggerTask  -> AlarmManagerTask
 * flashCommandQueue    UART command reception -> FlashLoggerTask
 * flashQueueSet        Set containing alarmToFlashQueue and flashCommandQueue
 */
extern QueueHandle_t modbusToAlarmQueue;
extern QueueHandle_t modbusToMqttQueue;
extern QueueHandle_t alarmToMqttQueue;
extern QueueHandle_t alarmToFlashQueue;
extern QueueHandle_t flashToAlarmQueue;
extern QueueHandle_t flashCommandQueue;
extern QueueSetHandle_t flashQueueSet;

/**
 * @brief Create all application queues and the Flash queue set.
 *
 * Must be called once during application initialisation before the
 * FreeRTOS scheduler is started.
 */
void AppQueuesInit(void);

#endif /* INC_APP_QUEUES_H_ */
