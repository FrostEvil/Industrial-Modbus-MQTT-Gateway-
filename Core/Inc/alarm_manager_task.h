/*
 * alarm_manager_task.h
 *
 *  Created on: Jul 16, 2026
 *      Author: tomas
 */

#ifndef INC_ALARM_MANAGER_TASK_H_
#define INC_ALARM_MANAGER_TASK_H_

typedef enum {
	ALARM_NORMAL = 0x00U, ALARM_MEASUREMENT = 0x01U, ALARM_COMMUNICATION = 0x02U
} AlarmState_t;

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
