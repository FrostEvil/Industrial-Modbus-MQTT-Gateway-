/*
 * modbus_poller_task.c
 *
 *  Created on: Jul 15, 2026
 *      Author: tomas
 */

/**
 * @file modbus_poller_task.c
 *
 * @brief FreeRTOS task responsible for cyclic Modbus polling.
 *
 * Owns the polling cycle: builds the request, polls the slave, converts
 * registers into voltage/current/temperature, attaches the communication
 * status, and publishes the result to AlarmManagerTask and
 * MqttPublisherTask via queues. UART/DMA/RS-485 details stay in
 * modbus_master.c.
 */

#include "modbus_poller_task.h"
#include "modbus_master.h"
#include "app_queues.h"
#include <string.h>
#include "task.h"

TaskHandle_t modbusPollerTaskHandle;

#define MODBUS_POLLER_TASK_STACK_SIZE 512

static StackType_t modbusPollerTaskStack[MODBUS_POLLER_TASK_STACK_SIZE];
static StaticTask_t modbusPollerTaskTCB;

/*
 * Modbus slave and registers being read:
 *
 *   slave address  = 0x01
 *   function       = 0x03 (Read Holding Registers)
 *   start address  = 0x0000
 *   register count = 3   (register 0 = voltage, 1 = current, 2 = temperature)
 */
static ModbusTarget_t modbus_target = { .slave_id = 0x01, .function_code = 0x03,
		.register_start_hi = 0x00, .register_start_lo = 0x00,
		.register_count_hi = 0x00, .register_count_lo = 0x03 };

static ModbusRetryPolicy_t modbus_retry_policy = { .poll_period_ms = 1000,
		.response_timeout_ms = 200, .max_attempts = 3 };

/*
 * Request payload without CRC:
 * [0] slave address, [1] function code, [2:3] start register, [4:5] count.
 */
static uint8_t modbus_request_payload[6];

/*
 * Complete request: 6 bytes payload + 2 bytes CRC.
 */
static uint8_t modbus_tx_buffer[8];

/*
 * Local copy of the validated response, populated by modbus_master_poll().
 */
static uint8_t modbus_rx_data[256];

/**
 * @brief Convert a valid Modbus response into application measurements.
 *
 * The slave sends the three measurements as 16-bit unsigned registers
 * scaled by 10 (e.g. register value 2350 -> 235.0 V). Assumes the frame was
 * already validated by modbus_validate_frame(), so the indexes below can be
 * used directly.
 *
 * Response layout: [0] address, [1] function, [2] byte count,
 * [3:4] voltage, [5:6] current, [7:8] temperature, [9:10] CRC.
 *
 * @param frame Validated Modbus response frame.
 * @param measurement_record Destination for the parsed values.
 */
static void modbus_parse_measurement_response(const uint8_t *frame,
		MeasurementRecord_t *measurement_record) {

	measurement_record->voltage = (float) ((frame[3] << 8) | frame[4]) / 10.0f;

	measurement_record->current = (float) ((frame[5] << 8) | frame[6]) / 10.0f;

	measurement_record->temperature = (float) ((frame[7] << 8) | frame[8])
			/ 10.0f;
}

/**
 * @brief Convert the Modbus transaction result into a measurement record.
 *
 * On MODBUS_OK the parsed values are used. On any other status the values
 * are zeroed so a comms failure can never be mistaken for a real reading -
 * downstream tasks rely on the separate 'status' field to tell the two
 * cases apart instead of inferring it from the numbers.
 *
 * @param modbus_master_poll_status Result returned by modbus_master_poll().
 * @param measurement_record Record to populate.
 */
static void modbus_result(const ModbusStatus_t modbus_master_poll_status,
		MeasurementRecord_t *measurement_record) {

	if (modbus_master_poll_status == MODBUS_OK) {

		modbus_parse_measurement_response(modbus_rx_data, measurement_record);

	} else {

		measurement_record->voltage = 0;
		measurement_record->current = 0;
		measurement_record->temperature = 0;
	}
}

/**
 * @brief Main FreeRTOS task responsible for cyclic Modbus polling.
 *
 * Builds the request once before the loop, since its contents never
 * change between cycles. Each iteration polls, parses, publishes the
 * latest record to both consumers (xQueueOverwrite - consumers only care
 * about the newest state) and waits for the next period.
 *
 * @param argument FreeRTOS task argument. Not used by this task.
 */
void ModbusPollerTask(void *argument) {

	MeasurementRecord_t measurement_record;

	modbus_request_payload[0] = modbus_target.slave_id;
	modbus_request_payload[1] = modbus_target.function_code;
	modbus_request_payload[2] = modbus_target.register_start_hi;
	modbus_request_payload[3] = modbus_target.register_start_lo;
	modbus_request_payload[4] = modbus_target.register_count_hi;
	modbus_request_payload[5] = modbus_target.register_count_lo;

	uint16_t crc = modbus_crc16(modbus_request_payload,
			sizeof(modbus_request_payload));

	/*
	 * Modbus RTU transmits the CRC low byte first.
	 */
	uint8_t crc_lsb = crc & 0x00FF;
	uint8_t crc_msb = (crc >> 8) & 0x00FF;

	memcpy(modbus_tx_buffer, modbus_request_payload,
			sizeof(modbus_request_payload));

	modbus_tx_buffer[6] = crc_lsb;
	modbus_tx_buffer[7] = crc_msb;

	/*
	 * vTaskDelayUntil() keeps the polling period stable even when one
	 * cycle takes longer than usual due to retries or comms errors, unlike
	 * a plain vTaskDelay() which would drift.
	 */
	TickType_t lastWakeTime = xTaskGetTickCount();

	if (modbus_master_init() != MODBUS_OK) {
		Error_Handler();
	}

	for (;;) {

		ModbusStatus_t modbus_master_poll_status = modbus_master_poll(
				modbus_tx_buffer, sizeof(modbus_tx_buffer), modbus_rx_data,
				&modbus_target, &modbus_retry_policy);

		modbus_result(modbus_master_poll_status, &measurement_record);

		measurement_record.status = modbus_master_poll_status;

		xQueueOverwrite(modbusToAlarmQueue, &measurement_record);

		xQueueOverwrite(modbusToMqttQueue, &measurement_record);

		vTaskDelayUntil(&lastWakeTime,
				pdMS_TO_TICKS(modbus_retry_policy.poll_period_ms));
	}
}

/**
 * @brief Create the Modbus Poller task using static allocation, priority 5.
 */
void ModbusPollerTask_Init(void) {

	modbusPollerTaskHandle = xTaskCreateStatic(ModbusPollerTask, "ModbusPoller",
	MODBUS_POLLER_TASK_STACK_SIZE,
	NULL, 4, modbusPollerTaskStack, &modbusPollerTaskTCB);
}
