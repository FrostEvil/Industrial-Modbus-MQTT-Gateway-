/*
 * alarm_manager_task.c
 *
 *  Created on: Jul 16, 2026
 *      Author: tomas
 */

#include "alarm_manager_task.h"
#include "app_queues.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define VOLTAGE_MIN 207
#define VOLTAGE_MAX 253
#define VOLTAGE_HYSTERESIS 1

#define CURRENT_MIN 0
#define CURRENT_MAX 10
#define CURRENT_HYSTERESIS 0.5

#define TEMPERATURE_MIN 20
#define TEMPERATURE_MAX 35
#define TEMPERATURE_HYSTERESIS 1

TaskHandle_t alarmManagerTaskHandle;

#define ALARM_MANAGER_TASK_STACK_SIZE 128
static StackType_t alarmManagerTaskStack[ALARM_MANAGER_TASK_STACK_SIZE];
static StaticTask_t alarmManagerTaskTCB;

static uint8_t is_measurement_out_of_range(float value, float min, float max) {
	if (value <= max && value >= min) {
		return 0;
	} else {
		return 1;
	}

}

static uint8_t should_clear_alarm(float value, float min, float max,
		float hysteresis) {
	if ((value >= min + hysteresis) && (value <= max - hysteresis)) {
		return 1;
	} else {
		return 0;
	}
}

static MeasurementRangeStatus_t update_measurement_status(
		MeasurementRangeStatus_t current_status, float value, float min,
		float max, float hysteresis) {
	if (current_status == MEASUREMENT_IN_RANGE) {
		if (is_measurement_out_of_range(value, min, max)) {
			return MEASUREMENT_OUT_OF_RANGE;
		}
	} else {
		if (should_clear_alarm(value, min, max, hysteresis)) {
			return MEASUREMENT_IN_RANGE;
		}
	}

	return current_status;
}

static void validate_measurements(MeasurementMessage_t *measurement_message,
		AlarmState_t *alarm_state, MeasurementStatus_t *measurement_status) {

	if (measurement_message->status == MODBUS_OK) {

		measurement_status->voltage = update_measurement_status(
				measurement_status->voltage, measurement_message->voltage,
				VOLTAGE_MIN, VOLTAGE_MAX, VOLTAGE_HYSTERESIS);
		measurement_status->current = update_measurement_status(
				measurement_status->current, measurement_message->current,
				CURRENT_MIN, CURRENT_MAX, CURRENT_HYSTERESIS);
		measurement_status->temperature = update_measurement_status(
				measurement_status->temperature,
				measurement_message->temperature,
				TEMPERATURE_MIN, TEMPERATURE_MAX, TEMPERATURE_HYSTERESIS);

		if (measurement_status->voltage == MEASUREMENT_IN_RANGE
				&& measurement_status->current == MEASUREMENT_IN_RANGE
				&& measurement_status->temperature == MEASUREMENT_IN_RANGE) {
			*alarm_state = ALARM_NORMAL;
		} else {
			*alarm_state = ALARM_MEASUREMENT;
		}

	} else {
		*alarm_state = ALARM_COMMUNICATION;
	}
}

static void update_alarm_led(AlarmState_t alarm_state) {
	switch (alarm_state) {
	case ALARM_NORMAL:
		HAL_GPIO_WritePin(ALARM_NORMAL_GPIO_Port, ALARM_NORMAL_Pin,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(ALARM_MEASUREMENT_GPIO_Port, ALARM_MEASUREMENT_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ALARM_COMMUNICATION_GPIO_Port,
		ALARM_COMMUNICATION_Pin, GPIO_PIN_RESET);
		break;

	case ALARM_MEASUREMENT:
		HAL_GPIO_WritePin(ALARM_NORMAL_GPIO_Port, ALARM_NORMAL_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ALARM_MEASUREMENT_GPIO_Port, ALARM_MEASUREMENT_Pin,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(ALARM_COMMUNICATION_GPIO_Port,
		ALARM_COMMUNICATION_Pin, GPIO_PIN_RESET);
		break;
	case ALARM_COMMUNICATION:
		HAL_GPIO_WritePin(ALARM_NORMAL_GPIO_Port, ALARM_NORMAL_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ALARM_MEASUREMENT_GPIO_Port, ALARM_MEASUREMENT_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ALARM_COMMUNICATION_GPIO_Port,
		ALARM_COMMUNICATION_Pin, GPIO_PIN_SET);
		break;

	default:
		HAL_GPIO_WritePin(ALARM_NORMAL_GPIO_Port, ALARM_NORMAL_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ALARM_MEASUREMENT_GPIO_Port, ALARM_MEASUREMENT_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ALARM_COMMUNICATION_GPIO_Port,
		ALARM_COMMUNICATION_Pin, GPIO_PIN_RESET);
	}
}

void AlarmManagerTask(void *argument) {
	MeasurementMessage_t measurement_message;
	MeasurementStatus_t measurement_status =
			{ .voltage = MEASUREMENT_IN_RANGE, .current = MEASUREMENT_IN_RANGE,
					.temperature = MEASUREMENT_IN_RANGE };
	AlarmState_t alarm_state = ALARM_NORMAL;

	for (;;) {
		xQueueReceive(modbusToAlarmQueue, &measurement_message, portMAX_DELAY);
		validate_measurements(&measurement_message, &alarm_state,
				&measurement_status);

		if (alarm_state != ALARM_NORMAL) {
			xQueueOverwrite(alarmToMqttQueue, &alarm_state);
		}

		update_alarm_led(alarm_state);
	}
}

void AlarmManagerTask_Init(void) {
	alarmManagerTaskHandle = xTaskCreateStatic(AlarmManagerTask, "AlarmManager",
	ALARM_MANAGER_TASK_STACK_SIZE, NULL, 5, alarmManagerTaskStack,
			&alarmManagerTaskTCB);
}
