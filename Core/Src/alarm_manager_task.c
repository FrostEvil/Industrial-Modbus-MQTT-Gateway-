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
#include "flash_logger_task.h"
#include <string.h>

#define VOLTAGE_MIN 207
#define VOLTAGE_MAX 253
#define VOLTAGE_HYSTERESIS 1

#define CURRENT_MIN 0
#define CURRENT_MAX 25
#define CURRENT_HYSTERESIS 0.5

#define TEMPERATURE_MIN 10
#define TEMPERATURE_MAX 45
#define TEMPERATURE_HYSTERESIS 1

#define MEASUREMENT_BIT_VOLTAGE   0x01
#define MEASUREMENT_BIT_CURRENT   0x02
#define MEASUREMENT_BIT_TEMP      0x04

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

static void validate_measurements(MeasurementRecord_t *measurement_message,
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

static void validate_measurement_status(
		const MeasurementStatus_t *measurement_status,
		MeasurementStatus_t *prev_measurement_status,
		uint8_t *is_status_changed, uint8_t *trigger_channel) {

	if (measurement_status->voltage != prev_measurement_status->voltage) {
		*trigger_channel |= MEASUREMENT_BIT_VOLTAGE;
		*is_status_changed = 1;
	}

	if (measurement_status->current != prev_measurement_status->current) {
		*trigger_channel |= MEASUREMENT_BIT_CURRENT;
		*is_status_changed = 1;
	}

	if (measurement_status->temperature
			!= prev_measurement_status->temperature) {
		*trigger_channel |= MEASUREMENT_BIT_TEMP;
		*is_status_changed = 1;
	}

	prev_measurement_status->voltage = measurement_status->voltage;
	prev_measurement_status->current = measurement_status->current;
	prev_measurement_status->temperature = measurement_status->temperature;
}

static void validate_measurement_bitmask(uint8_t *measurement_bitmask,
		const MeasurementStatus_t *measurement_status) {
	if (measurement_status->voltage == MEASUREMENT_OUT_OF_RANGE) {
		*measurement_bitmask |= MEASUREMENT_BIT_VOLTAGE;
	} else {
		*measurement_bitmask &= ~MEASUREMENT_BIT_VOLTAGE;
	}

	if (measurement_status->current == MEASUREMENT_OUT_OF_RANGE) {
		*measurement_bitmask |= MEASUREMENT_BIT_CURRENT;
	} else {
		*measurement_bitmask &= ~MEASUREMENT_BIT_CURRENT;
	}

	if (measurement_status->temperature == MEASUREMENT_OUT_OF_RANGE) {
		*measurement_bitmask |= MEASUREMENT_BIT_TEMP;
	} else {
		*measurement_bitmask &= ~MEASUREMENT_BIT_TEMP;
	}
}

static void update_alarm_led(AlarmState_t alarm_state,
		FlashLoggerAlarmFault_t flash_logger_alarm_fault) {

	uint8_t normal_led = (alarm_state == ALARM_NORMAL)
			&& (flash_logger_alarm_fault == ALARM_OK);
	uint8_t measurement_led = (alarm_state == ALARM_MEASUREMENT);
	uint8_t communication_led = (alarm_state == ALARM_COMMUNICATION)
			|| (flash_logger_alarm_fault == COMMUNICATION_FAULT);
	uint8_t storage_led = (flash_logger_alarm_fault == STORAGE_FAULT);

	HAL_GPIO_WritePin(ALARM_NORMAL_GPIO_Port, ALARM_NORMAL_Pin,
			normal_led ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ALARM_MEASUREMENT_GPIO_Port, ALARM_MEASUREMENT_Pin,
			measurement_led ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ALARM_COMMUNICATION_GPIO_Port, ALARM_COMMUNICATION_Pin,
			communication_led ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ALARM_STORAGE_FAULT_GPIO_Port, ALARM_STORAGE_FAULT_Pin,
			storage_led ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void AlarmManagerTask(void *argument) {
	MeasurementRecord_t measurement_record;
	MeasurementStatus_t prev_measurement_status = { .voltage =
			MEASUREMENT_IN_RANGE, .current = MEASUREMENT_IN_RANGE,
			.temperature = MEASUREMENT_IN_RANGE };
	MeasurementStatus_t measurement_status =
			{ .voltage = MEASUREMENT_IN_RANGE, .current = MEASUREMENT_IN_RANGE,
					.temperature = MEASUREMENT_IN_RANGE };
	uint8_t is_measurement_status_changed = 0;

	AlarmState_t measurement_alarm_state = ALARM_NORMAL;
	AlarmState_t prev_measurement_alarm_state = ALARM_NORMAL;
	FlashLoggerAlarmFault_t prev_flash_logger_alarm_fault = ALARM_OK;
	FlashLoggerAlarmFault_t flash_logger_alarm_fault = ALARM_OK;

	MqttAlarmState_t mqtt_alarm_state;

	uint8_t dropped_measurements = 0;

	for (;;) {

		FlashRecord_t flash_record;

		if (xQueueReceive(flashToAlarmQueue, &flash_logger_alarm_fault,
				pdMS_TO_TICKS(100)) == pdTRUE) {

		}

		xQueueReceive(modbusToAlarmQueue, &measurement_record,
		portMAX_DELAY);

		//wypełnienie 0x00 - kontrolowany padding.
		memset(&flash_record, 0, sizeof(FlashRecord_t));

		flash_record.timestamp_ms = xTaskGetTickCount();
		flash_record.voltage = measurement_record.voltage;
		flash_record.current = measurement_record.current;
		flash_record.temperature = measurement_record.temperature;
		flash_record.crc = modbus_crc16((uint8_t*) &flash_record,
				offsetof(FlashRecord_t, crc));
		flash_record.trigger_channel = 0x00;

		validate_measurements(&measurement_record, &measurement_alarm_state,
				&measurement_status);

		validate_measurement_status(&measurement_status,
				&prev_measurement_status, &is_measurement_status_changed,
				&flash_record.trigger_channel);

		if (is_measurement_status_changed) {

			if (xQueueSend(alarmToFlashQueue, &flash_record,
					pdMS_TO_TICKS(1000)) != pdPASS) {
				dropped_measurements++;

			}

		}

		if (measurement_alarm_state != prev_measurement_alarm_state
				|| flash_logger_alarm_fault != prev_flash_logger_alarm_fault
				|| is_measurement_status_changed) {

			mqtt_alarm_state.alarm_state = measurement_alarm_state;
			mqtt_alarm_state.storage_fault = flash_logger_alarm_fault;
			validate_measurement_bitmask(&mqtt_alarm_state.measurement_bitmask,
					&measurement_status);
			xQueueOverwrite(alarmToMqttQueue, &mqtt_alarm_state);
		}

		prev_measurement_alarm_state = measurement_alarm_state;
		prev_flash_logger_alarm_fault = flash_logger_alarm_fault;

		update_alarm_led(measurement_alarm_state, flash_logger_alarm_fault);
		is_measurement_status_changed = 0;
	}
}

void AlarmManagerTask_Init(void) {
	alarmManagerTaskHandle = xTaskCreateStatic(AlarmManagerTask, "AlarmManager",
	ALARM_MANAGER_TASK_STACK_SIZE, NULL, 5, alarmManagerTaskStack,
			&alarmManagerTaskTCB);
}
