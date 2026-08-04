/*
 * flash_logger_task.h
 *
 *  Created on: Jul 17, 2026
 *      Author: tomas
 */

#ifndef INC_FLASH_LOGGER_TASK_H_
#define INC_FLASH_LOGGER_TASK_H_

typedef enum {
	ALARM_OK = 0x00U, COMMUNICATION_FAULT = 0x01U, STORAGE_FAULT = 0x02U
} FlashLoggerAlarmFault_t;

typedef struct {
	float voltage;
	float current;
	float temperature;
	uint32_t timestamp_ms;
	uint16_t crc;
} FlashRecord_t;

void FlashLoggerTask_Init(void);

#endif /* INC_FLASH_LOGGER_TASK_H_ */
