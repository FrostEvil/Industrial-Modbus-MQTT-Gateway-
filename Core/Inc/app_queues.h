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
 * @brief One measurement cycle's result, passed from Modbus Poller to
 *        any task that needs to react to it (currently: Alarm Manager).
 *
 * When status != MODBUS_OK, voltage/current/temperature are zeroed -
 * receivers MUST check status before trusting the measurement fields.
 */

typedef struct {
	float voltage;
	float current;
	float temperature;
	ModbusStatus_t status;
} MeasurementRecord_t;

// Handle used by both the sender (Modbus Poller) and the receiver
// (Alarm Manager) to refer to this specific queue.
extern QueueHandle_t modbusToAlarmQueue;
extern QueueHandle_t modbusToMqttQueue;
extern QueueHandle_t alarmToMqttQueue;
extern QueueHandle_t alarmToFlashQueue;
extern QueueHandle_t flashToAlarmQueue;
extern QueueHandle_t flashCommandQueue;
extern QueueHandle_t flashQueueSet;
/**
 * @brief Creates all inter-task queues. Must be called once, from main(),
 *        before osKernelStart() - same timing rule as AppTaskInit().
 */
void AppQueuesInit(void);

#endif /* INC_APP_QUEUES_H_ */
