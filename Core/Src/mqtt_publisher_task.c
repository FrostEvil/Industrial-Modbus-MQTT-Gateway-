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

#define MEASUREMENT_BIT_VOLTAGE   0x01U
#define MEASUREMENT_BIT_CURRENT   0x02U
#define MEASUREMENT_BIT_TEMP      0x04U

static StackType_t mqttPublisherStack[MQTT_PUBLISHER_TASK_STACK_SIZE];
static StaticTask_t mqttPublisherTaskTCB;

static char uart6_tx_buffer[TX_BUFFER_SIZE];

/*
 * Convert the latest measurement record into the text protocol used
 * between STM32 and ESP8266.
 *
 * Example:
 * M,V=230.0,I=12.5,T=32.0
 */
static void send_uart6_measurement_record(
		const MeasurementRecord_t *measurement_record) {

	snprintf(
			uart6_tx_buffer,
			TX_BUFFER_SIZE,
			"M,V=%.1f,I=%.1f,T=%.1f\n",
			measurement_record->voltage,
			measurement_record->current,
			measurement_record->temperature);

	HAL_UART_Transmit(
			&huart6,
			(uint8_t *) uart6_tx_buffer,
			strlen(uart6_tx_buffer),
			HAL_MAX_DELAY);
}

/*
 * Compare one measurement channel between the current and previous
 * bitmasks.
 *
 * A 0 -> 1 transition means that the channel entered the alarm state.
 * A 1 -> 0 transition means that the channel returned to the normal state.
 *
 * The function only appends text to the existing UART buffer. It does not
 * modify the previous bitmask; that is handled by send_uart_alarm_state()
 * after all three channels have been processed.
 */
static void append_measurement_state_change(
		uint8_t measurement_bitmask,
		uint8_t prev_measurement_bitmask,
		uint8_t measurement_bit,
		char measurement_symbol,
		uint8_t len) {

	if (((measurement_bitmask & measurement_bit) != 0U)
			&& ((prev_measurement_bitmask & measurement_bit) == 0U)) {

		snprintf(
				uart6_tx_buffer + len,
				TX_BUFFER_SIZE - len,
				",%c=OUT",
				measurement_symbol);

	} else if (((measurement_bitmask & measurement_bit) == 0U)
			&& ((prev_measurement_bitmask & measurement_bit) != 0U)) {

		snprintf(
				uart6_tx_buffer + len,
				TX_BUFFER_SIZE - len,
				",%c=IN",
				measurement_symbol);
	}
}

/*
 * Build and send an alarm event message.
 *
 * The message contains:
 *
 *   SPI=...          Flash logger state
 *   S=...            overall measurement/communication state
 *   V/I/T=...        channel state changes when communication is valid
 *
 * Per-channel state tracking is intentionally not updated while
 * ALARM_COMMUNICATION is active. During a communication failure the values
 * received earlier are preserved and are only compared again after a valid
 * measurement is available.
 */
static void send_uart_alarm_state(
		const MqttAlarmState_t *mqtt_alarm_state,
		uint8_t *prev_measurement_bitmask) {

	snprintf(
			uart6_tx_buffer,
			TX_BUFFER_SIZE,
			"E");

	uint8_t len = strlen(uart6_tx_buffer);

	/*
	 * Report the state of the Flash logging subsystem independently from
	 * the Modbus/measurement state.
	 */
	if (mqtt_alarm_state->storage_fault == ALARM_OK) {

		snprintf(
				uart6_tx_buffer + len,
				TX_BUFFER_SIZE - len,
				",SPI=OK");

	} else if (mqtt_alarm_state->storage_fault == COMMUNICATION_FAULT) {

		snprintf(
				uart6_tx_buffer + len,
				TX_BUFFER_SIZE - len,
				",SPI=COMMUNICATION");

	} else if (mqtt_alarm_state->storage_fault == STORAGE_FAULT) {

		snprintf(
				uart6_tx_buffer + len,
				TX_BUFFER_SIZE - len,
				",SPI=STORAGE");
	}

	len = strlen(uart6_tx_buffer);

	/*
	 * A communication failure means that the measurement values and their
	 * per-channel state cannot currently be trusted, so no V/I/T transition
	 * is generated here.
	 */
	if (mqtt_alarm_state->alarm_state == ALARM_COMMUNICATION) {

		snprintf(
				uart6_tx_buffer + len,
				TX_BUFFER_SIZE - len,
				",S=COMMUNICATION");

	} else {

		if (mqtt_alarm_state->alarm_state == ALARM_MEASUREMENT) {

			snprintf(
					uart6_tx_buffer + len,
					TX_BUFFER_SIZE - len,
					",S=MEASUREMENT");

		} else {

			snprintf(
					uart6_tx_buffer + len,
					TX_BUFFER_SIZE - len,
					",S=NORMAL");
		}

		/*
		 * Compare the current channel states with the last channel states
		 * that were actually communicated.
		 */
		len = strlen(uart6_tx_buffer);

		append_measurement_state_change(
				mqtt_alarm_state->measurement_bitmask,
				*prev_measurement_bitmask,
				MEASUREMENT_BIT_VOLTAGE,
				'V',
				len);

		len = strlen(uart6_tx_buffer);

		append_measurement_state_change(
				mqtt_alarm_state->measurement_bitmask,
				*prev_measurement_bitmask,
				MEASUREMENT_BIT_CURRENT,
				'I',
				len);

		len = strlen(uart6_tx_buffer);

		append_measurement_state_change(
				mqtt_alarm_state->measurement_bitmask,
				*prev_measurement_bitmask,
				MEASUREMENT_BIT_TEMP,
				'T',
				len);

		/*
		 * Update the reference only after all channel changes have been
		 * compared and appended to the message.
		 */
		*prev_measurement_bitmask =
				mqtt_alarm_state->measurement_bitmask;
	}

	len = strlen(uart6_tx_buffer);

	snprintf(
			uart6_tx_buffer + len,
			TX_BUFFER_SIZE - len,
			"\n");

	/*
	 * UART6 carries the application protocol towards the ESP8266.
	 */
	HAL_UART_Transmit(
			&huart6,
			(uint8_t *) uart6_tx_buffer,
			strlen(uart6_tx_buffer),
			HAL_MAX_DELAY);
}

void MqttPublisherTask(void *argument) {

	MeasurementRecord_t measurement_record;
	MqttAlarmState_t mqtt_alarm_state;

	/*
	 * Stores the last measurement bitmask that was successfully processed
	 * while communication was available.
	 *
	 * It is deliberately not updated during ALARM_COMMUNICATION.
	 */
	uint8_t prev_measurement_bitmask = 0x00U;

	for (;;) {

		/*
		 * Check for a new alarm state before waiting for the next
		 * measurement.
		 *
		 * The timeout allows the task to continue to its normal measurement
		 * path even when no alarm event is waiting.
		 */
		if (xQueueReceive(
				alarmToMqttQueue,
				&mqtt_alarm_state,
				pdMS_TO_TICKS(100)) == pdTRUE) {

			send_uart_alarm_state(
					&mqtt_alarm_state,
					&prev_measurement_bitmask);
		}

		/*
		 * Wait for the next measurement record.
		 *
		 * modbusToMqttQueue contains the latest measurement state produced
		 * by ModbusPollerTask.
		 */
		xQueueReceive(
				modbusToMqttQueue,
				&measurement_record,
				portMAX_DELAY);

		send_uart6_measurement_record(&measurement_record);
	}
}

void MqttPublisherTask_Init(void) {

	/*
	 * The task uses static allocation so its stack and TCB are provided
	 * by application-owned memory rather than the FreeRTOS heap.
	 */
	mqttPublisherTaskHandle = xTaskCreateStatic(
			MqttPublisherTask,
			"MQTT Publisher",
			MQTT_PUBLISHER_TASK_STACK_SIZE,
			NULL,
			2,
			mqttPublisherStack,
			&mqttPublisherTaskTCB);
}
