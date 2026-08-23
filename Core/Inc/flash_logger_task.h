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

// Bitmask indicating which channel(s) caused this record to be logged
// (their status changed since the previous measurement -- either
// crossing OUT of range, or recovering back IN range).
// Bit 0 (0x01) = voltage changed state
// Bit 1 (0x02) = current changed state
// Bit 2 (0x04) = temperature changed state
// Bit 3 (0x08) = unvalid CRC
// Bit 4 (0x16) = empty record
// A set bit means "this channel's status flipped in this record" --
// whether that flip was into or out of range must be read from the
// channel's actual value against the known thresholds, not from this
// bitmask alone.

//20bajtów
typedef struct {
	float voltage;
	float current;
	float temperature;
	uint32_t timestamp_ms;
	uint16_t crc;
	uint8_t trigger_channel;
} FlashRecord_t;

void FlashLoggerTask_Init(void);
extern TaskHandle_t flashLoggerTaskHandle;

#endif /* INC_FLASH_LOGGER_TASK_H_ */
