#include <stdint.h>
#include "usart.h"
#include "modbus_master.h"
#include "flash_logger.h"
#include "gpio.h"

/*
 * Common UART reception callback used by the STM32 HAL.
 *
 * The callback is executed when ReceiveToIdle detects the end of a received
 * frame. Its job is only to route the event to the module responsible for
 * that UART. Parsing and other longer operations are performed outside
 * this callback.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	/*
	 * This application uses IDLE detection as the frame boundary for the
	 * asynchronous UART receivers. Other reception events are ignored.
	 */
	if (huart->RxEventType != HAL_UART_RXEVENT_IDLE) {
		return;
	}

	if (huart == &huart1) {

		/*
		 * USART1 is used for Modbus RTU communication.
		 */
		modbus_rx_event(huart->RxEventType, Size);

	} else if (huart == &huart2) {

		/*
		 * USART2 is the service interface used by the Flash logger.
		 */
		flash_logger_rx_event(huart->RxEventType, Size);

	} else if (huart == &huart6) {

		/*
		 * UART6 is connected to the ESP8266.
		 *
		 * At the moment the receive event is used only as a simple
		 * communication indicator during development.
		 */
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}
