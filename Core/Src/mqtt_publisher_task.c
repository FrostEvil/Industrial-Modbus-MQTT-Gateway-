/*
 * mqtt_publisher_task.c
 *
 *  Created on: Jul 19, 2026
 *      Author: tomas
 */

#include "FreeRTOS.h"
#include "task.h"
#include "app_queues.h"
#include "alarm_manager_task.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

TaskHandle_t mqttPublisherTaskHandle;

#define MQTT_PUBLISHER_TASK_STACK_SIZE 512
#define TX_BUFFER_SIZE 64

#define MEASUREMENT_BIT_VOLTAGE   0x01
#define MEASUREMENT_BIT_CURRENT   0x02
#define MEASUREMENT_BIT_TEMP      0x04

static StackType_t mqttPublisherStack[MQTT_PUBLISHER_TASK_STACK_SIZE];
static StaticTask_t mqttPublisherTaskTCB;

char uart6_tx_buffer[TX_BUFFER_SIZE];

static void send_uart6_measurement_record(
		const MeasurementRecord_t *measurement_record) {

	snprintf(uart6_tx_buffer, TX_BUFFER_SIZE, "M,V=%.1f,I=%.1f,T=%.1f\n",
			measurement_record->voltage, measurement_record->current,
			measurement_record->temperature);

	HAL_UART_Transmit(&huart6, (uint8_t*) uart6_tx_buffer,
			strlen(uart6_tx_buffer), HAL_MAX_DELAY);
}

static void check_measurement_range(uint8_t measurement_bitmask,
		uint8_t prev_measurement_bitmask, uint8_t measurement_bit,
		char measurement_symbol, uint8_t len) {
	if (((measurement_bitmask & measurement_bit) != 0)
			&& ((prev_measurement_bitmask & measurement_bit) == 0)) {

		snprintf(uart6_tx_buffer + len, TX_BUFFER_SIZE - len, ",%c=OUT",
				measurement_symbol);
	} else if (((measurement_bitmask & measurement_bit) == 0)
			&& ((prev_measurement_bitmask & measurement_bit) != 0)) {

		snprintf(uart6_tx_buffer + len, TX_BUFFER_SIZE - len, ",%c=IN",
				measurement_symbol);
	}
}

// prev_measurement_bitmask is now a POINTER, updated by this function itself --
// and only in the branch that actually just compared the channels against it.
// Previously, MqttPublisherTask updated this value unconditionally after every
// received event, even when alarm_state == ALARM_COMMUNICATION and the channel
// comparison below never ran at all. That let a real, already-happened channel
// recovery get silently absorbed into the "last known" value without ever
// being announced as "<channel>=IN" -- the next real S=NORMAL event would then
// compare against a bitmask that had quietly already "caught up", see no
// difference, and never report the recovery. Moving the update to right here,
// in the same place that does the comparison, means it can only ever move
// forward in lockstep with something that was actually communicated.
static void send_uart_alarm_state(const MqttAlarmState_t *mqtt_alarm_state,
		uint8_t *prev_measurement_bitmask) {

	snprintf(uart6_tx_buffer, TX_BUFFER_SIZE, "E");

	uint8_t len = strlen(uart6_tx_buffer);

	if (mqtt_alarm_state->storage_fault == ALARM_OK) {
		snprintf(uart6_tx_buffer + len, TX_BUFFER_SIZE - len, ",SPI=OK");
	} else if (mqtt_alarm_state->storage_fault == COMMUNICATION_FAULT) {
		snprintf(uart6_tx_buffer + len, TX_BUFFER_SIZE - len,
				",SPI=COMMUNICATION");
	} else if (mqtt_alarm_state->storage_fault == STORAGE_FAULT) {
		snprintf(uart6_tx_buffer + len, TX_BUFFER_SIZE - len, ",SPI=STORAGE");
	}

	len = strlen(uart6_tx_buffer);

	if (mqtt_alarm_state->alarm_state == ALARM_COMMUNICATION) {

		snprintf(uart6_tx_buffer + len, TX_BUFFER_SIZE - len,
				",S=COMMUNICATION");
		// Deliberately untouched: we don't trust per-channel data while
		// communication itself is down, so *prev_measurement_bitmask must
		// stay exactly as it was -- nothing about V/I/T was announced here.
	} else {

		mqtt_alarm_state->alarm_state == ALARM_MEASUREMENT ?
				snprintf(uart6_tx_buffer + len, TX_BUFFER_SIZE - len,
						",S=MEASUREMENT") :
				snprintf(uart6_tx_buffer + len, TX_BUFFER_SIZE - len,
						",S=NORMAL");

		len = strlen(uart6_tx_buffer);
		check_measurement_range(mqtt_alarm_state->measurement_bitmask,
				*prev_measurement_bitmask, MEASUREMENT_BIT_VOLTAGE, 'V', len);

		len = strlen(uart6_tx_buffer);
		check_measurement_range(mqtt_alarm_state->measurement_bitmask,
				*prev_measurement_bitmask, MEASUREMENT_BIT_CURRENT, 'I', len);

		len = strlen(uart6_tx_buffer);
		check_measurement_range(mqtt_alarm_state->measurement_bitmask,
				*prev_measurement_bitmask, MEASUREMENT_BIT_TEMP, 'T', len);

		// Only update the bookkeeping value once we've actually just compared
		// against it and reported whatever differed -- see the comment above
		// the function for why this can no longer happen unconditionally.
		*prev_measurement_bitmask = mqtt_alarm_state->measurement_bitmask;
	}

	len = strlen(uart6_tx_buffer);
	snprintf(uart6_tx_buffer + len, TX_BUFFER_SIZE - len, "\n");

	HAL_UART_Transmit(&huart6, (uint8_t*) uart6_tx_buffer,
			strlen(uart6_tx_buffer), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*) uart6_tx_buffer,
				strlen(uart6_tx_buffer), HAL_MAX_DELAY);

}

void MqttPublisherTask(void *argument) {
	MeasurementRecord_t measurement_record;
	MqttAlarmState_t mqtt_alarm_state;
	uint8_t prev_measurement_bitmask = 0x00;
	for (;;) {

		if (xQueueReceive(alarmToMqttQueue, &mqtt_alarm_state,
				pdMS_TO_TICKS(100)) == pdTRUE) {

//			mqtt_alarm_state.alarm_state = ALARM_MEASUREMENT;
//			mqtt_alarm_state.storage_fault = ALARM_OK;
//			prev_measurement_bitmask = 0x00;
//			mqtt_alarm_state.measurement_bitmask = 0x00;
			send_uart_alarm_state(&mqtt_alarm_state, &prev_measurement_bitmask);
		}
		xQueueReceive(modbusToMqttQueue, &measurement_record,
		portMAX_DELAY);

		send_uart6_measurement_record(&measurement_record);

	}
}

void MqttPublisherTask_Init(void) {
	mqttPublisherTaskHandle = xTaskCreateStatic(MqttPublisherTask,
			"MQTT Publisher", MQTT_PUBLISHER_TASK_STACK_SIZE, NULL, 2,
			mqttPublisherStack, &mqttPublisherTaskTCB);
}
