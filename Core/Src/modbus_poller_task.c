/*
 * modbus_poller_task.c
 *
 *  Created on: Jul 15, 2026
 *      Author: tomas
 */

#include "modbus_poller_task.h"
#include "modbus_master.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include "app_queues.h"
#include "task.h"

TaskHandle_t modbusPollerTaskHandle;

#define MODBUS_POLLER_TASK_STACK_SIZE 512
static StackType_t modbusPollerTaskStack[MODBUS_POLLER_TASK_STACK_SIZE];
static StaticTask_t modbusPollerTaskTCB;

static ModbusTarget_t modbus_target = { .slave_id = 0x01, .function_code = 0x03,
		.register_start_hi = 0x00, .register_start_lo = 0x00,
		.register_count_hi = 0x00, .register_count_lo = 0x03 };

static ModbusRetryPolicy_t modbus_retry_policy = { .poll_period_ms = 5000,
		.response_timeout_ms = 200, .max_attempts = 3 };

static uint8_t modbus_request_payload[6];
static uint8_t modbus_tx_buffer_size = 8;
static uint8_t modbus_tx_buffer[8];
static uint8_t modbus_rx_data[256];
static char pc_tx_buffer[64];
static uint8_t dropped_measurements = 0;

MeasurementRecord_t measurement_record;

static void modbus_parse_measurements(uint8_t *frame) {

	measurement_record.voltage = (float) ((frame[3] << 8) | frame[4]) / 10.0f;
	measurement_record.current = (float) ((frame[5] << 8) | frame[6]) / 10.0f;
	measurement_record.temperature = (float) ((frame[7] << 8) | frame[8])
			/ 10.0f;

	snprintf(pc_tx_buffer, sizeof(pc_tx_buffer),
			"Voltage:%.1fV, Current:%.1fA, Temperature:%.1fC\r\n",
			measurement_record.voltage, measurement_record.current,
			measurement_record.temperature);
	HAL_UART_Transmit(&huart2, (uint8_t*) pc_tx_buffer, strlen(pc_tx_buffer),
	HAL_MAX_DELAY);
}

static void modbus_result(ModbusStatus_t modbus_master_poll_status,
		uint8_t exception_code) {

	switch (modbus_master_poll_status) {
	case MODBUS_OK:
		modbus_parse_measurements(modbus_rx_data);
//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // heartbeat on each successful cycle
		break;

	case MODBUS_ERR_EXCEPTION:
		snprintf(pc_tx_buffer, sizeof(pc_tx_buffer),
				"Exception 0x%02X - not retrying\r\n", exception_code);
		break;

	case MODBUS_ERR_LENGTH:
		snprintf(pc_tx_buffer, sizeof(pc_tx_buffer),
				"Frame length error on last attempt (after %u attempts)\r\n",
				modbus_retry_policy.max_attempts);
		break;

	case MODBUS_ERR_ADDRESS:
		snprintf(pc_tx_buffer, sizeof(pc_tx_buffer),
				"Slave address mismatch on last attempt (after %u attempts)\r\n",
				modbus_retry_policy.max_attempts);
		break;

	case MODBUS_ERR_CRC:
		snprintf(pc_tx_buffer, sizeof(pc_tx_buffer),
				"CRC16 mismatch on last attempt (after %u attempts)\r\n",
				modbus_retry_policy.max_attempts);
		break;

	case MODBUS_ERR_TIMEOUT:
		snprintf(pc_tx_buffer, sizeof(pc_tx_buffer),
				"No response from slave (silence after %u attempts)\r\n",
				modbus_retry_policy.max_attempts);
		break;

	default:
		snprintf(pc_tx_buffer, sizeof(pc_tx_buffer), "Unknown error\r\n");
		break;
	}

	if (modbus_master_poll_status != MODBUS_OK) {
		measurement_record.voltage = 0;
		measurement_record.current = 0;
		measurement_record.temperature = 0;
		HAL_UART_Transmit(&huart2, (uint8_t*) pc_tx_buffer,
				strlen(pc_tx_buffer), HAL_MAX_DELAY);
	}
}

void ModbusPollerTask(void *argument) {
	modbus_request_payload[0] = modbus_target.slave_id;
	modbus_request_payload[1] = modbus_target.function_code;
	modbus_request_payload[2] = modbus_target.register_start_hi;
	modbus_request_payload[3] = modbus_target.register_start_lo;
	modbus_request_payload[4] = modbus_target.register_count_hi;
	modbus_request_payload[5] = modbus_target.register_count_lo;

	uint16_t crc = modbus_crc16(modbus_request_payload,
			sizeof(modbus_request_payload));
	uint8_t crc_lsb = crc & 0x00FF;
	uint8_t crc_msb = (crc >> 8) & 0x00FF;

	memcpy(modbus_tx_buffer, modbus_request_payload,
			sizeof(modbus_request_payload));
	modbus_tx_buffer[6] = crc_lsb;
	modbus_tx_buffer[7] = crc_msb;

	TickType_t lastWakeTime = xTaskGetTickCount();

	modbus_master_init();

	for (;;) {

		uint8_t exception_code = 0;

		ModbusStatus_t modbus_master_poll_status = modbus_master_poll(
				modbus_tx_buffer, modbus_tx_buffer_size, modbus_rx_data,
				&modbus_target, &modbus_retry_policy, &exception_code);
		modbus_result(modbus_master_poll_status, exception_code);

		measurement_record.status = modbus_master_poll_status;
		xQueueOverwrite(modbusToAlarmQueue, &measurement_record);
		xQueueOverwrite(modbusToMqttQueue, &measurement_record);
		if (xQueueSend(modbusToFlashLoggerQueue, &measurement_record,
				pdMS_TO_TICKS(100)) != pdPASS) {
			dropped_measurements++;

			snprintf(pc_tx_buffer, sizeof(pc_tx_buffer),
					"Dropped measurements: %d\r\n", dropped_measurements);
			HAL_UART_Transmit(&huart2, (uint8_t*) pc_tx_buffer,
					strlen(pc_tx_buffer),
					HAL_MAX_DELAY);
		}

		vTaskDelayUntil(&lastWakeTime,
				pdMS_TO_TICKS(modbus_retry_policy.poll_period_ms));

	}
}

void ModbusPollerTask_Init(void) {
	modbusPollerTaskHandle = xTaskCreateStatic(ModbusPollerTask, "ModbusPoller",
	MODBUS_POLLER_TASK_STACK_SIZE, NULL, 5, modbusPollerTaskStack,
			&modbusPollerTaskTCB);

}
