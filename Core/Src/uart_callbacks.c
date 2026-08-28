#include <stdint.h>
#include "usart.h"
#include "modbus_master.h"
#include "flash_logger.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

	if (huart->RxEventType != HAL_UART_RXEVENT_IDLE) {
		return;
	}
	if (huart == &huart1) {

		modbus_rx_event(huart->RxEventType, Size);
	}

	if (huart == &huart2) {

		flash_logger_rx_event(huart->RxEventType, Size);
	}

	if (huart == &huart6) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//		esp_rx_event(huart->RxEventType, Size);
	}
}
