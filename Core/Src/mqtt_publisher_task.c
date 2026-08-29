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
 * Text protocol between STM32 and ESP8266, e.g. "M,V=230.0,I=12.5,T=32.0".
 */
static void send_uart6_measurement_record(
		const MeasurementRecord_t *measurement_record) {

	snprintf(uart6_tx_buffer,
	TX_BUFFER_SIZE, "M,V=%.1f,I=%.1f,T=%.1f\n", measurement_record->voltage,
			measurement_record->current, measurement_record->temperature);

	HAL_UART_Transmit(&huart6, (uint8_t*) uart6_tx_buffer,
			strlen(uart6_tx_buffer),
			HAL_MAX_DELAY);
}

/*
 * Appends to the existing UART buffer only - does not touch
 * prev_measurement_bitmask, that's updated once by the caller after all
 * three channels are processed.
 *
 * 0 -> 1 transition: channel entered alarm. 1 -> 0: channel returned to
 * normal.
 */
static uint8_t append_measurement_state_change(uint8_t measurement_bitmask,
		uint8_t prev_measurement_bitmask, uint8_t measurement_bit,
		char measurement_symbol, uint8_t len) {

	if (((measurement_bitmask & measurement_bit) != 0U)
			&& ((prev_measurement_bitmask & measurement_bit) == 0U)) {

		return snprintf(uart6_tx_buffer + len,
		TX_BUFFER_SIZE - len, ",%c=OUT", measurement_symbol);

	} else if (((measurement_bitmask & measurement_bit) == 0U)
			&& ((prev_measurement_bitmask & measurement_bit) != 0U)) {

		return snprintf(uart6_tx_buffer + len,
		TX_BUFFER_SIZE - len, ",%c=IN", measurement_symbol);
	}

	return 0;
}

/*
 * Builds and sends one alarm event message: SPI=... (Flash logger state),
 * S=... (measurement/communication state), V/I/T=... (channel transitions,
 * only while communication is valid).
 *
 * Per-channel tracking is intentionally NOT updated during
 * ALARM_COMMUNICATION - values from before the failure are preserved and
 * compared again only once a valid measurement is available.
 */

/*
 * NOTE: snprintf() returns how many characters it WOULD have written if
 * the buffer had been unlimited - not how many actually fit. At this
 * buffer size (192 B) and with these short, fixed messages, truncation
 * never happens in practice, so `len` is always accurate here. If this
 * buffer ever shrinks or the messages grow, `len` could end up larger
 * than what's really in the buffer, and `uart6_tx_buffer + len` /
 * `TX_BUFFER_SIZE - len` (unsigned!) would then point/compute
 * out of bounds. Worth clamping `len` to `TX_BUFFER_SIZE - 1`
 * after each snprintf() if that ever becomes a real risk.
 */
static void send_uart_alarm_state(const MqttAlarmState_t *mqtt_alarm_state,
		uint8_t *prev_measurement_bitmask) {
	/*
	 * NOTE: snprintf() returns how many characters it WOULD have written...
	 * (patrz komentarz wyżej)
	 */
	uint8_t len = snprintf(uart6_tx_buffer,
	TX_BUFFER_SIZE, "E");

	if (mqtt_alarm_state->storage_fault == ALARM_OK) {

		len += snprintf(uart6_tx_buffer + len,
		TX_BUFFER_SIZE - len, ",SPI=OK");

	} else if (mqtt_alarm_state->storage_fault == COMMUNICATION_FAULT) {

		len += snprintf(uart6_tx_buffer + len,
		TX_BUFFER_SIZE - len, ",SPI=COMMUNICATION");

	} else if (mqtt_alarm_state->storage_fault == STORAGE_FAULT) {

		len += snprintf(uart6_tx_buffer + len,
		TX_BUFFER_SIZE - len, ",SPI=STORAGE");
	}

	if (mqtt_alarm_state->alarm_state == ALARM_COMMUNICATION) {

		len += snprintf(uart6_tx_buffer + len,
		TX_BUFFER_SIZE - len, ",S=COMMUNICATION");

	} else {

		if (mqtt_alarm_state->alarm_state == ALARM_MEASUREMENT) {

			len += snprintf(uart6_tx_buffer + len,
			TX_BUFFER_SIZE - len, ",S=MEASUREMENT");

		} else {

			len += snprintf(uart6_tx_buffer + len,
			TX_BUFFER_SIZE - len, ",S=NORMAL");
		}

		len += append_measurement_state_change(
				mqtt_alarm_state->measurement_bitmask,
				*prev_measurement_bitmask,
				MEASUREMENT_BIT_VOLTAGE, 'V', len);

		len += append_measurement_state_change(
				mqtt_alarm_state->measurement_bitmask,
				*prev_measurement_bitmask,
				MEASUREMENT_BIT_CURRENT, 'I', len);

		len += append_measurement_state_change(
				mqtt_alarm_state->measurement_bitmask,
				*prev_measurement_bitmask,
				MEASUREMENT_BIT_TEMP, 'T', len);

		*prev_measurement_bitmask = mqtt_alarm_state->measurement_bitmask;
	}

	len += snprintf(uart6_tx_buffer + len,
	TX_BUFFER_SIZE - len, "\n");

	HAL_UART_Transmit(&huart6, (uint8_t*) uart6_tx_buffer,
			strlen(uart6_tx_buffer),
			HAL_MAX_DELAY);
}

void MqttPublisherTask(void *argument) {

	MeasurementRecord_t measurement_record;
	MqttAlarmState_t mqtt_alarm_state;

	/*
	 * Last bitmask actually processed while communication was available -
	 * deliberately not updated during ALARM_COMMUNICATION.
	 */
	uint8_t prev_measurement_bitmask = 0x00U;

	for (;;) {

		/*
		 * Checked first, with a timeout, so the task still falls through
		 * to the normal measurement path when no alarm is waiting.
		 */
		if (xQueueReceive(alarmToMqttQueue, &mqtt_alarm_state,
				pdMS_TO_TICKS(100)) == pdTRUE) {

			send_uart_alarm_state(&mqtt_alarm_state, &prev_measurement_bitmask);
		}

		xQueueReceive(modbusToMqttQueue, &measurement_record,
		portMAX_DELAY);

		send_uart6_measurement_record(&measurement_record);
	}
}

void MqttPublisherTask_Init(void) {

	mqttPublisherTaskHandle = xTaskCreateStatic(MqttPublisherTask,
			"MQTT Publisher",
			MQTT_PUBLISHER_TASK_STACK_SIZE,
			NULL, 2, mqttPublisherStack, &mqttPublisherTaskTCB);
}
