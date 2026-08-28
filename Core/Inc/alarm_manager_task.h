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

/**
 * @brief Overall alarm state reported by AlarmManagerTask.
 *
 * The state describes the source of the current alarm:
 * normal operation, measurement out of range or communication failure.
 *
 * Flash-related faults are kept separately in FlashLoggerAlarmFault_t.
 */
typedef enum {
	ALARM_NORMAL = 0x00U,
	ALARM_MEASUREMENT = 0x01U,
	ALARM_COMMUNICATION = 0x02U
} AlarmState_t;

/**
 * @brief Alarm state sent from AlarmManagerTask to the MQTT task.
 *
 * alarm_state:
 *     Overall state of the measurement/Modbus part of the system.
 *
 * measurement_bitmask:
 *     Indicates which measurement channels are currently out of range.
 *     Bit 0 = voltage, bit 1 = current, bit 2 = temperature.
 *
 * storage_fault:
 *     Current state of the Flash logging subsystem.
 */
typedef struct {
	AlarmState_t alarm_state;
	uint8_t measurement_bitmask;
	FlashLoggerAlarmFault_t storage_fault;
} MqttAlarmState_t;

/**
 * @brief Current range state of one measurement channel.
 */
typedef enum {
	MEASUREMENT_IN_RANGE = 0,
	MEASUREMENT_OUT_OF_RANGE = 1
} MeasurementRangeStatus_t;

/**
 * @brief Range state of all monitored measurements.
 *
 * The states are stored separately so that each channel can have its own
 * hysteresis and state transition history.
 */
typedef struct {
	MeasurementRangeStatus_t voltage;
	MeasurementRangeStatus_t current;
	MeasurementRangeStatus_t temperature;
} MeasurementStatus_t;

/**
 * @brief Create the Alarm Manager FreeRTOS task.
 */
void AlarmManagerTask_Init(void);

#endif /* INC_ALARM_MANAGER_TASK_H_ */
