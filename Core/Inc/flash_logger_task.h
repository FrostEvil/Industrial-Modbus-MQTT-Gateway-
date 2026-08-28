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
 * ALARM_OK:
 *     Flash logger is operating normally.
 *
 * COMMUNICATION_FAULT:
 *     A temporary Flash communication problem occurred. Further writes
 *     are still allowed and another successful write clears this state.
 *
 * STORAGE_FAULT:
 *     A persistent storage problem or exhausted capacity was detected.
 *     Automatic writes are blocked until the history is erased.
 */
typedef enum {
	ALARM_OK = 0x00U,
	COMMUNICATION_FAULT = 0x01U,
	STORAGE_FAULT = 0x02U
} FlashLoggerAlarmFault_t;

/**
 * @brief Record stored in external SPI Flash.
 *
 * trigger_channel is a bitmask describing which measurement channel changed
 * its alarm state in the current record:
 *
 *     bit 0 (0x01) = voltage
 *     bit 1 (0x02) = current
 *     bit 2 (0x04) = temperature
 *
 * The mask describes which channel changed state, not the direction of the
 * change. Whether the channel entered or left the alarm state is determined
 * from the corresponding measurement value and configured limits.
 *
 * crc is stored at the end of the structure and is calculated over all
 * preceding record fields.
 */


/**
 * @brief Create the Flash Logger FreeRTOS task.
 */
void FlashLoggerTask_Init(void);

/**
 * @brief Handle of the Flash Logger task.
 *
 * The handle is exposed for diagnostics or future task-level control.
 */
extern TaskHandle_t flashLoggerTaskHandle;

#endif /* INC_FLASH_LOGGER_TASK_H_ */
