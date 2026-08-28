/*
 * modbus_poller_task.h
 *
 *  Created on: Jul 15, 2026
 *      Author: tomas
 */

#ifndef INC_MODBUS_POLLER_TASK_H_
#define INC_MODBUS_POLLER_TASK_H_

/**
 * @brief Create the Modbus Poller FreeRTOS task.
 *
 * The task is statically allocated and periodically polls the Modbus slave.
 */
void ModbusPollerTask_Init(void);

#endif /* INC_MODBUS_POLLER_TASK_H_ */
