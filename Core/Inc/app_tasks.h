/*
 * app_tasks.h
 *
 *  Created on: Jul 15, 2026
 *      Author: tomas
 */

#ifndef INC_APP_TASKS_H_
#define INC_APP_TASKS_H_

/**
 * @brief Create all application FreeRTOS tasks.
 *
 * Must be called during application initialisation before the FreeRTOS
 * scheduler is started.
 */
void AppTasksInit(void);

#endif /* INC_APP_TASKS_H_ */
