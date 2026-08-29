#include <stdint.h>
#include "usart.h"
#include "modbus_master.h"
#include "flash_logger.h"
#include "gpio.h"

/*
 * Common HAL UART reception callback, fired on IDLE detection. Only routes
 * the event to the module owning that UART - parsing and anything longer
 * happens outside this callback.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

	if (huart->RxEventType != HAL_UART_RXEVENT_IDLE) {
		return;
	}

	if (huart == &huart1) {

		/*
		 * USART1: Modbus RTU.
		 */
		modbus_rx_event(huart->RxEventType, Size);

	} else if (huart == &huart2) {

		/*
		 * USART2: Flash logger service interface.
		 */
		flash_logger_rx_event(huart->RxEventType, Size);

	} else if (huart == &huart6) {

		/*
		 * USART6: link to the ESP8266. The STM32 only ever transmits on
		 * this link today (measurement/alarm text from MqttPublisherTask) -
		 * nothing needs to be read back yet, so there's nothing for this
		 * branch to do.
		 */
	}
}
