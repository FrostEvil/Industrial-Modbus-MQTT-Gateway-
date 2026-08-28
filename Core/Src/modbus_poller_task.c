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
 * This task is the application-level owner of the Modbus polling cycle.
 *
 * Its responsibilities are:
 *
 *   1. Build the Modbus request frame.
 *   2. Periodically ask the slave for the current measurements.
 *   3. Convert the received register values into application-level
 *      measurements.
 *   4. Attach the communication status to the measurement record.
 *   5. Publish the latest measurement record to the consumers through
 *      FreeRTOS queues.
 *
 * The task does not deal directly with UART registers, DMA or RS-485
 * direction control. Those details are hidden inside modbus_master.c.
 */

#include "modbus_poller_task.h"
#include "modbus_master.h"
#include "app_queues.h"
#include <string.h>
#include "task.h"

/*
 * Task handle is kept globally because other parts of the application may
 * need to refer to the task later, for example for diagnostics.
 */
TaskHandle_t modbusPollerTaskHandle;

/*
 * Static allocation is used instead of dynamic allocation.
 *
 * This means the task's stack and TCB are allocated at compile time and
 * their memory addresses remain fixed for the lifetime of the application.
 */
#define MODBUS_POLLER_TASK_STACK_SIZE 512

static StackType_t modbusPollerTaskStack[MODBUS_POLLER_TASK_STACK_SIZE];
static StaticTask_t modbusPollerTaskTCB;

/*
 * Description of the Modbus slave and the registers we want to read.
 *
 * Current configuration:
 *
 *   slave address  = 0x01
 *   function       = 0x03 (Read Holding Registers)
 *   start address  = 0x0000
 *   register count = 3
 *
 * The three registers represent:
 *
 *   register 0 -> voltage
 *   register 1 -> current
 *   register 2 -> temperature
 */
static ModbusTarget_t modbus_target = {
		.slave_id = 0x01,
		.function_code = 0x03,
		.register_start_hi = 0x00,
		.register_start_lo = 0x00,
		.register_count_hi = 0x00,
		.register_count_lo = 0x03
};

/*
 * Timing and retry parameters for the polling cycle.
 *
 * poll_period_ms:
 *      Desired time between consecutive polling cycles.
 *
 * response_timeout_ms:
 *      Maximum time to wait for one slave response.
 *
 * max_attempts:
 *      Number of times the same request may be attempted before the
 *      communication is reported as failed.
 */
static ModbusRetryPolicy_t modbus_retry_policy = {
		.poll_period_ms = 1000,
		.response_timeout_ms = 200,
		.max_attempts = 3
};

/*
 * Request payload without CRC.
 *
 * The payload contains:
 *
 *   [0] slave address
 *   [1] function code
 *   [2] starting register high byte
 *   [3] starting register low byte
 *   [4] register count high byte
 *   [5] register count low byte
 */
static uint8_t modbus_request_payload[6];

/*
 * Complete Modbus request including the two CRC bytes.
 *
 * 6 bytes payload + 2 bytes CRC = 8 bytes total.
 */
static uint8_t modbus_tx_buffer[8];

/*
 * Local copy of the received response.
 *
 * modbus_master_poll() copies the validated frame here so that the rest of
 * the application does not need direct access to the DMA receive buffer.
 */
static uint8_t modbus_rx_data[256];

/**
 * @brief Convert a valid Modbus response into application measurements.
 *
 * The Modbus slave sends the three measurements as 16-bit unsigned register
 * values with a scaling factor of 10.
 *
 * Example:
 *
 *   register value 2350 -> 235.0 V
 *
 * The parser assumes that the frame has already been validated by
 * modbus_validate_frame(). Therefore the indexes below can be used directly.
 *
 * Expected response format:
 *
 *   [0]      slave address
 *   [1]      function code
 *   [2]      byte count
 *   [3:4]    voltage
 *   [5:6]    current
 *   [7:8]    temperature
 *   [9:10]   CRC
 *
 * @param frame Validated Modbus response frame.
 * @param measurement_record Destination for the parsed values.
 */
static void modbus_parse_measurement_response(
		const uint8_t *frame,
		MeasurementRecord_t *measurement_record) {

	/*
	 * Registers are transmitted as two bytes in big-endian order.
	 *
	 * Reconstruct the 16-bit register value and then divide by 10 because
	 * the Arduino Modbus slave sends values scaled by 10.
	 */
	measurement_record->voltage =
			(float) ((frame[3] << 8) | frame[4]) / 10.0f;

	measurement_record->current =
			(float) ((frame[5] << 8) | frame[6]) / 10.0f;

	measurement_record->temperature =
			(float) ((frame[7] << 8) | frame[8]) / 10.0f;
}

/**
 * @brief Convert the Modbus transaction result into a measurement record.
 *
 * A measurement is considered valid only if modbus_master_poll() reports
 * MODBUS_OK.
 *
 * When communication fails, the task still publishes a record to the queues,
 * but the measurement values are explicitly set to zero. The separate
 * 'status' field is then used by downstream tasks to distinguish:
 *
 *   valid measurement + MODBUS_OK
 *
 * from:
 *
 *   communication failure + zeroed measurement values
 *
 * This allows AlarmManagerTask and MqttPublisherTask to handle the fault
 * without having to infer communication state from the numeric values.
 *
 * @param modbus_master_poll_status Result returned by modbus_master_poll().
 * @param measurement_record Record to populate.
 */
static void modbus_result(
		const ModbusStatus_t modbus_master_poll_status,
		MeasurementRecord_t *measurement_record) {

	if (modbus_master_poll_status == MODBUS_OK) {

		/*
		 * The response has already passed address, CRC, function code and
		 * length validation, so it is safe to decode the register values.
		 */
		modbus_parse_measurement_response(
				modbus_rx_data,
				measurement_record
		);

	} else {

		/*
		 * Communication failed, therefore the numerical values must not be
		 * treated as valid measurements.
		 *
		 * The actual reason for the failure is preserved separately in
		 * measurement_record->status.
		 */
		measurement_record->voltage = 0;
		measurement_record->current = 0;
		measurement_record->temperature = 0;
	}
}

/**
 * @brief Main FreeRTOS task responsible for cyclic Modbus polling.
 *
 * The task performs all one-time request preparation before entering the
 * infinite loop. This avoids rebuilding a request whose contents do not
 * change on every cycle.
 *
 * Inside the loop:
 *
 *   1. Send the request and obtain a validated response.
 *   2. Convert the response into voltage/current/temperature.
 *   3. Store the communication status.
 *   4. Publish the record to AlarmManagerTask.
 *   5. Publish the same record to MqttPublisherTask.
 *   6. Wait until the next scheduled polling period.
 *
 * xQueueOverwrite() is used because the consumers are interested in the
 * latest measurement state, not in processing every historical sample.
 *
 * @param argument FreeRTOS task argument. Not used by this task.
 */
void ModbusPollerTask(void *argument) {

	/*
	 * The measurement record belongs to this task.
	 *
	 * FreeRTOS queues copy the structure when xQueueOverwrite() is called,
	 * so the same local object can safely be reused in the next polling
	 * cycle.
	 */
	MeasurementRecord_t measurement_record;

	/*
	 * Build the request payload from the configured Modbus target.
	 *
	 * Keeping the target configuration in a structure makes the task easier
	 * to modify than hard-coding every byte directly into the frame.
	 */
	modbus_request_payload[0] = modbus_target.slave_id;
	modbus_request_payload[1] = modbus_target.function_code;
	modbus_request_payload[2] = modbus_target.register_start_hi;
	modbus_request_payload[3] = modbus_target.register_start_lo;
	modbus_request_payload[4] = modbus_target.register_count_hi;
	modbus_request_payload[5] = modbus_target.register_count_lo;

	/*
	 * Calculate CRC over the six-byte payload before appending it to the
	 * final transmission buffer.
	 */
	uint16_t crc = modbus_crc16(
			modbus_request_payload,
			sizeof(modbus_request_payload)
	);

	/*
	 * Modbus RTU transmits the CRC low byte first.
	 */
	uint8_t crc_lsb = crc & 0x00FF;
	uint8_t crc_msb = (crc >> 8) & 0x00FF;

	/*
	 * Build the complete request:
	 *
	 *   bytes 0..5 -> payload
	 *   bytes 6..7 -> CRC
	 */
	memcpy(
			modbus_tx_buffer,
			modbus_request_payload,
			sizeof(modbus_request_payload)
	);

	modbus_tx_buffer[6] = crc_lsb;
	modbus_tx_buffer[7] = crc_msb;

	/*
	 * vTaskDelayUntil() uses this value as the reference point for the
	 * periodic schedule.
	 *
	 * Compared with a simple vTaskDelay(), this helps keep the polling
	 * period stable even if one polling cycle takes a different amount of
	 * time due to retries or communication errors.
	 */
	TickType_t lastWakeTime = xTaskGetTickCount();

	/*
	 * Start the asynchronous UART reception mechanism before entering the
	 * polling loop.
	 *
	 * At the moment there is no final error-handling policy here. This will
	 * be addressed together with the project's overall fault-handling
	 * strategy.
	 */
	if (modbus_master_init() != MODBUS_OK) {

		/*
		 * TODO:
		 * Decide how a permanent initialisation failure should be handled
		 * by the application (for example: retry, enter a fault state, or
		 * stop the task).
		 */
	}

	for (;;) {

		/*
		 * Execute one complete Modbus transaction.
		 *
		 * On success, modbus_rx_data contains a validated response.
		 * On failure, only the returned status should be trusted.
		 */
		ModbusStatus_t modbus_master_poll_status =
				modbus_master_poll(
						modbus_tx_buffer,
						sizeof(modbus_tx_buffer),
						modbus_rx_data,
						&modbus_target,
						&modbus_retry_policy
				);

		/*
		 * Convert the raw Modbus response into application-level values.
		 */
		modbus_result(
				modbus_master_poll_status,
				&measurement_record
		);

		/*
		 * Always attach the communication result to the record.
		 *
		 * Downstream tasks use this field to distinguish valid measurements
		 * from communication failures.
		 */
		measurement_record.status = modbus_master_poll_status;

		/*
		 * Publish the latest record to both consumers.
		 *
		 * xQueueOverwrite() copies the entire structure into the queue.
		 * Because these queues represent "latest state", an older unread
		 * measurement may intentionally be replaced by a newer one.
		 */
		xQueueOverwrite(
				modbusToAlarmQueue,
				&measurement_record
		);

		xQueueOverwrite(
				modbusToMqttQueue,
				&measurement_record
		);

		/*
		 * Wait until the next scheduled polling instant.
		 *
		 * Using vTaskDelayUntil() prevents the polling period from drifting
		 * over time as much as a simple vTaskDelay() would.
		 */
		vTaskDelayUntil(
				&lastWakeTime,
				pdMS_TO_TICKS(modbus_retry_policy.poll_period_ms)
		);
	}
}

/**
 * @brief Create the Modbus Poller task using static allocation.
 *
 * xTaskCreateStatic() is used so that both the task control block and
 * the task stack come from memory allocated by the application rather than
 * from the FreeRTOS heap.
 *
 * The task is assigned priority 5.
 */
void ModbusPollerTask_Init(void) {

	modbusPollerTaskHandle = xTaskCreateStatic(
			ModbusPollerTask,
			"ModbusPoller",
			MODBUS_POLLER_TASK_STACK_SIZE,
			NULL,
			5,
			modbusPollerTaskStack,
			&modbusPollerTaskTCB
	);
}
