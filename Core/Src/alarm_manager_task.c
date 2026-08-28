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
#include "flash_logger.h"
#include <string.h>
#include <stdbool.h>

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

/*
 * Returns true when the measurement is outside the normal operating range.
 *
 * This function does not apply hysteresis. Hysteresis is handled separately
 * by update_measurement_status() because the allowed transition depends on
 * the previous state of the channel.
 */
static bool is_measurement_out_of_range(float value, float min, float max) {
	if (value <= max && value >= min) {
		return false;
	}

	return true;
}

/*
 * A channel that is already in alarm should not return to the normal state
 * immediately when the value crosses the original limit. The value must
 * first enter the narrower range created by the hysteresis.
 *
 * Example for voltage:
 * normal range: 207...253 V
 * hysteresis:   1 V
 *
 * After an alarm, the value must return to 208...252 V to clear it.
 */
static bool should_clear_alarm(float value, float min, float max,
		float hysteresis) {

	if ((value >= min + hysteresis)
			&& (value <= max - hysteresis)) {
		return true;
	}

	return false;
}

/*
 * Update one channel's state using its previous state.
 *
 * When the channel is currently normal, the normal limits determine whether
 * an alarm should start.
 *
 * When the channel is already in alarm, the hysteresis limits determine
 * whether the alarm can be cleared.
 */
static MeasurementRangeStatus_t update_measurement_status(
		MeasurementRangeStatus_t current_status,
		float value,
		float min,
		float max,
		float hysteresis) {

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

/*
 * Update the measurement-related alarm state.
 *
 * A communication error is handled separately from measurement alarms.
 * In that case the previous V/I/T states are deliberately preserved so that
 * a temporary Modbus failure does not make the system "forget" an active
 * measurement alarm.
 */
static void update_measurement_alarm_state(
		MeasurementRecord_t *measurement_message,
		AlarmState_t *alarm_state,
		MeasurementStatus_t *measurement_status) {

	if (measurement_message->status == MODBUS_OK) {

		measurement_status->voltage = update_measurement_status(
				measurement_status->voltage,
				measurement_message->voltage,
				VOLTAGE_MIN,
				VOLTAGE_MAX,
				VOLTAGE_HYSTERESIS);

		measurement_status->current = update_measurement_status(
				measurement_status->current,
				measurement_message->current,
				CURRENT_MIN,
				CURRENT_MAX,
				CURRENT_HYSTERESIS);

		measurement_status->temperature = update_measurement_status(
				measurement_status->temperature,
				measurement_message->temperature,
				TEMPERATURE_MIN,
				TEMPERATURE_MAX,
				TEMPERATURE_HYSTERESIS);

		if (measurement_status->voltage == MEASUREMENT_IN_RANGE
				&& measurement_status->current == MEASUREMENT_IN_RANGE
				&& measurement_status->temperature == MEASUREMENT_IN_RANGE) {

			*alarm_state = ALARM_NORMAL;

		} else {

			*alarm_state = ALARM_MEASUREMENT;
		}

	} else {

		/*
		 * Communication failure has its own alarm state. The individual
		 * measurement states are intentionally not modified here.
		 */
		*alarm_state = ALARM_COMMUNICATION;
	}
}

/*
 * Compare the current V/I/T states with the states from the previous
 * successful measurement.
 *
 * trigger_channel is a bitmask. Several channels may change during the
 * same measurement cycle, so the function uses |= instead of assigning
 * a single value.
 *
 * The previous state is updated at the end so that the next cycle can
 * detect another transition.
 */
static void detect_measurement_state_changes(
		const MeasurementStatus_t *measurement_status,
		MeasurementStatus_t *prev_measurement_status,
		bool *is_status_changed,
		uint8_t *trigger_channel) {

	if (measurement_status->voltage != prev_measurement_status->voltage) {
		*trigger_channel |= MEASUREMENT_BIT_VOLTAGE;
		*is_status_changed = true;
	}

	if (measurement_status->current != prev_measurement_status->current) {
		*trigger_channel |= MEASUREMENT_BIT_CURRENT;
		*is_status_changed = true;
	}

	if (measurement_status->temperature
			!= prev_measurement_status->temperature) {
		*trigger_channel |= MEASUREMENT_BIT_TEMP;
		*is_status_changed = true;
	}

	prev_measurement_status->voltage = measurement_status->voltage;
	prev_measurement_status->current = measurement_status->current;
	prev_measurement_status->temperature = measurement_status->temperature;
}

/*
 * Build the bitmask describing which measurement channels are currently
 * outside their allowed range.
 *
 * This is different from trigger_channel:
 *
 * trigger_channel = which channel changed state in this event
 * measurement_bitmask = which channels are currently in alarm
 */
static void build_measurement_bitmask(uint8_t *measurement_bitmask,
		const MeasurementStatus_t *measurement_status) {

	/*
	 * Rebuild the mask from scratch so that unused bits never contain
	 * stale or uninitialised data.
	 */
	*measurement_bitmask = 0x00;

	if (measurement_status->voltage == MEASUREMENT_OUT_OF_RANGE) {
		*measurement_bitmask |= MEASUREMENT_BIT_VOLTAGE;
	}

	if (measurement_status->current == MEASUREMENT_OUT_OF_RANGE) {
		*measurement_bitmask |= MEASUREMENT_BIT_CURRENT;
	}

	if (measurement_status->temperature == MEASUREMENT_OUT_OF_RANGE) {
		*measurement_bitmask |= MEASUREMENT_BIT_TEMP;
	}
}

/*
 * Update all alarm LEDs from the current application state.
 *
 * Measurement, communication and Flash faults are kept as separate
 * concepts. This allows, for example, a measurement alarm and a Flash
 * communication fault to be visible at the same time.
 */
static void update_alarm_led(
		AlarmState_t alarm_state,
		FlashLoggerAlarmFault_t flash_logger_alarm_fault) {

	uint8_t normal_led = (alarm_state == ALARM_NORMAL)
			&& (flash_logger_alarm_fault == ALARM_OK);

	uint8_t measurement_led =
			(alarm_state == ALARM_MEASUREMENT);

	uint8_t communication_led =
			(alarm_state == ALARM_COMMUNICATION)
			|| (flash_logger_alarm_fault == COMMUNICATION_FAULT);

	uint8_t storage_led =
			(flash_logger_alarm_fault == STORAGE_FAULT);

	HAL_GPIO_WritePin(
			ALARM_NORMAL_GPIO_Port,
			ALARM_NORMAL_Pin,
			normal_led ? GPIO_PIN_SET : GPIO_PIN_RESET);

	HAL_GPIO_WritePin(
			ALARM_MEASUREMENT_GPIO_Port,
			ALARM_MEASUREMENT_Pin,
			measurement_led ? GPIO_PIN_SET : GPIO_PIN_RESET);

	HAL_GPIO_WritePin(
			ALARM_COMMUNICATION_GPIO_Port,
			ALARM_COMMUNICATION_Pin,
			communication_led ? GPIO_PIN_SET : GPIO_PIN_RESET);

	HAL_GPIO_WritePin(
			ALARM_STORAGE_FAULT_GPIO_Port,
			ALARM_STORAGE_FAULT_Pin,
			storage_led ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void AlarmManagerTask(void *argument) {

	/*
	 * This structure is reused on every cycle. FreeRTOS queues copy the
	 * structure when xQueueOverwrite()/xQueueSend() is called, so the same
	 * object can safely be reused afterwards.
	 */
	MeasurementRecord_t measurement_record;

	/*
	 * Store the previous state separately from the current state.
	 *
	 * The previous state is needed to detect transitions such as:
	 *
	 *     IN -> OUT
	 *     OUT -> IN
	 */
	MeasurementStatus_t prev_measurement_status = {
			.voltage = MEASUREMENT_IN_RANGE,
			.current = MEASUREMENT_IN_RANGE,
			.temperature = MEASUREMENT_IN_RANGE
	};

	MeasurementStatus_t measurement_status = {
			.voltage = MEASUREMENT_IN_RANGE,
			.current = MEASUREMENT_IN_RANGE,
			.temperature = MEASUREMENT_IN_RANGE
	};

	bool is_measurement_status_changed = false;

	AlarmState_t measurement_alarm_state = ALARM_NORMAL;
	AlarmState_t prev_measurement_alarm_state = ALARM_NORMAL;

	FlashLoggerAlarmFault_t prev_flash_logger_alarm_fault = ALARM_OK;
	FlashLoggerAlarmFault_t flash_logger_alarm_fault = ALARM_OK;

	MqttAlarmState_t mqtt_alarm_state;

	for (;;) {

		FlashRecord_t flash_record;

		/*
		 * FlashLoggerTask reports changes of the Flash subsystem through
		 * flashToAlarmQueue.
		 *
		 * A timeout is used here because the task must not wait forever for
		 * a Flash status update. If there is no new Flash status, the
		 * current state is simply kept.
		 */
		xQueueReceive(
				flashToAlarmQueue,
				&flash_logger_alarm_fault,
				pdMS_TO_TICKS(100));

		/*
		 * The next measurement is the main trigger for this task.
		 */
		xQueueReceive(
				modbusToAlarmQueue,
				&measurement_record,
				portMAX_DELAY);

		/*
		 * FlashRecord_t may contain compiler padding between fields.
		 * Clearing the whole structure gives those padding bytes a
		 * deterministic value before the record is included in the CRC.
		 */
		memset(&flash_record, 0, sizeof(flash_record));

		flash_record.timestamp_ms = xTaskGetTickCount();
		flash_record.voltage = measurement_record.voltage;
		flash_record.current = measurement_record.current;
		flash_record.temperature = measurement_record.temperature;

		/*
		 * trigger_channel starts empty and is filled later by
		 * detect_measurement_state_changes().
		 */
		flash_record.trigger_channel = 0x00;

		update_measurement_alarm_state(
				&measurement_record,
				&measurement_alarm_state,
				&measurement_status);

		/*
		 * Determine which measurement channels changed state during this
		 * cycle. This may set more than one bit.
		 */
		detect_measurement_state_changes(
				&measurement_status,
				&prev_measurement_status,
				&is_measurement_status_changed,
				&flash_record.trigger_channel);

		/*
		 * trigger_channel must already contain its final value before CRC
		 * calculation because it is part of the protected record data.
		 */
		flash_record.crc = modbus_crc16(
				(uint8_t *) &flash_record,
				offsetof(FlashRecord_t, crc));

		if (is_measurement_status_changed) {

			/*
			 * Do not block AlarmManagerTask waiting for FlashLoggerTask.
			 *
			 * If the queue is full, the event is dropped. This is preferable
			 * here to blocking alarm handling, LED updates and MQTT updates
			 * while Flash is busy.
			 */
			if (xQueueSend(
					alarmToFlashQueue,
					&flash_record,
					0) != pdPASS) {

				/*
				 * The event is intentionally dropped if the Flash queue
				 * is full. The current alarm state continues to exist
				 * independently of the historical Flash log.
				 */
			}
		}

		/*
		 * MQTT is updated when something relevant changes:
		 *
		 *   - overall measurement alarm state,
		 *   - Flash fault state,
		 *   - individual measurement channel state.
		 *
		 * xQueueOverwrite() is used because MQTT needs the latest state,
		 * not every intermediate state.
		 */
		if (measurement_alarm_state != prev_measurement_alarm_state
				|| flash_logger_alarm_fault != prev_flash_logger_alarm_fault
				|| is_measurement_status_changed) {

			mqtt_alarm_state.alarm_state = measurement_alarm_state;
			mqtt_alarm_state.storage_fault = flash_logger_alarm_fault;

			build_measurement_bitmask(
					&mqtt_alarm_state.measurement_bitmask,
					&measurement_status);

			xQueueOverwrite(
					alarmToMqttQueue,
					&mqtt_alarm_state);
		}

		prev_measurement_alarm_state = measurement_alarm_state;
		prev_flash_logger_alarm_fault = flash_logger_alarm_fault;

		update_alarm_led(
				measurement_alarm_state,
				flash_logger_alarm_fault);

		/*
		 * This flag describes only the current measurement cycle.
		 * Clear it before waiting for the next measurement.
		 */
		is_measurement_status_changed = false;
	}
}

void AlarmManagerTask_Init(void) {

	alarmManagerTaskHandle = xTaskCreateStatic(
			AlarmManagerTask,
			"AlarmManager",
			ALARM_MANAGER_TASK_STACK_SIZE,
			NULL,
			5,
			alarmManagerTaskStack,
			&alarmManagerTaskTCB);
}
