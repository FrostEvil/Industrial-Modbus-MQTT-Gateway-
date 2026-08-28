/*
 * alarm_manager_task.h
 *
 *  Created on: Jul 16, 2026
 *      Author: tomas
 */

#ifndef INC_ALARM_MANAGER_TASK_H_
#define INC_ALARM_MANAGER_TASK_H_

#include <stdint.h>
#include "flash_logger_task.h"
typedef enum {
	ALARM_NORMAL = 0x00U, ALARM_MEASUREMENT = 0x01U, ALARM_COMMUNICATION = 0x02U
} AlarmState_t;

typedef struct {
	AlarmState_t alarm_state;              // domena Modbus/pomiary - bez zmian
	uint8_t measurement_bitmask; // IN/OUT per kanal, sensowne gdy alarm_state==ALARM_MEASUREMENT
	FlashLoggerAlarmFault_t storage_fault; // domena SPI/Flash - w pełni niezależna
} MqttAlarmState_t;

typedef enum {
	MEASUREMENT_IN_RANGE = 0, MEASUREMENT_OUT_OF_RANGE = 1
} MeasurementRangeStatus_t;

typedef struct {
	MeasurementRangeStatus_t voltage;
	MeasurementRangeStatus_t current;
	MeasurementRangeStatus_t temperature;
} MeasurementStatus_t;

void AlarmManagerTask_Init(void);

#endif /* INC_ALARM_MANAGER_TASK_H_ */
