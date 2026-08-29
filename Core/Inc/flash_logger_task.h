/*
 * flash_logger_task.h
 *
 *  Created on: Jul 17, 2026
 *      Author: tomas
 */

#ifndef INC_FLASH_LOGGER_TASK_H_
#define INC_FLASH_LOGGER_TASK_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief Fault state of the Flash logging subsystem.
 *
 * ALARM_OK: normal operation.
 * COMMUNICATION_FAULT: temporary problem, writes still allowed, clears on
 * the next successful write.
 * STORAGE_FAULT: persistent problem or exhausted capacity - automatic
 * writes are blocked until the history is erased.
 */
typedef enum {
	ALARM_OK = 0x00U, COMMUNICATION_FAULT = 0x01U, STORAGE_FAULT = 0x02U
} FlashLoggerAlarmFault_t;

typedef struct {
	uint8_t alarm_counter;
	FlashLoggerAlarmFault_t alarm_fault;
} FlashLoggerHealth_t;

/**
 * @brief Create the Flash Logger FreeRTOS task.
 */
void FlashLoggerTask_Init(void);

/**
 * @brief Handle of the Flash Logger task, exposed for diagnostics.
 */
extern TaskHandle_t flashLoggerTaskHandle;

#endif /* INC_FLASH_LOGGER_TASK_H_ */
