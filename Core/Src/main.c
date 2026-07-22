/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include "modbus_protocol.h"
#include "modbus_master.h"
#include "cmsis_os2.h"
#include "app_tasks.h"
#include "spi.h"
#include <stddef.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void flash_spi_transfer(uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t size) {
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, tx_buffer, rx_buffer, size,
	HAL_MAX_DELAY);
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

void write_enable_spi(void) {
	uint8_t write_enable_tx_buffer = 0x06;
	uint8_t write_enable_rx_buffer;

	flash_spi_transfer(&write_enable_tx_buffer, &write_enable_rx_buffer, 1);
}

void flash_wait_while_busy(void) {

	uint8_t status_busy = 1;
	uint8_t read_status_register_tx_buffer[] = { 0x05, 0x00 };
	uint8_t read_status_register_rx_buffer[2];

	while (status_busy) {
		flash_spi_transfer(read_status_register_tx_buffer,
				read_status_register_rx_buffer, 2);
		status_busy = read_status_register_rx_buffer[1] & 0x01;
	}
}

void read_jedec_id(void) {
	uint8_t tx_buffer[] = { 0x9F, 0x00, 0x00, 0x00 };
	uint8_t rx_buffer[4];

	flash_spi_transfer(tx_buffer, rx_buffer, 4);

	uint8_t manufacturer_id = rx_buffer[1];
	uint16_t memory_type_and_capacity = rx_buffer[2] << 8 | rx_buffer[3];

	char uart_message[64];
	snprintf(uart_message, sizeof(uart_message),
			"Manufacturer ID: 0x%02X, Memory type and capacity: 0x%04X\r\n",
			manufacturer_id, memory_type_and_capacity);

	HAL_UART_Transmit(&huart2, (uint8_t*) uart_message, strlen(uart_message),
	HAL_MAX_DELAY);
}

void read_single_page_spi(void) {
	uint8_t read_data_tx_buffer[] = { 0x03, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00 };
	uint8_t read_data_rx_buffer[7];

	char uart_message[64];

	flash_spi_transfer(read_data_tx_buffer, read_data_rx_buffer, 7);

	snprintf(uart_message, sizeof(uart_message), "0x%02X, 0x%02X, 0x%02X \r\n",
			read_data_rx_buffer[4], read_data_rx_buffer[5],
			read_data_rx_buffer[6]);

	HAL_UART_Transmit(&huart2, (uint8_t*) uart_message, strlen(uart_message),
	HAL_MAX_DELAY);
}

void read_flash_record(uint8_t *rx_buffer, uint8_t size) {

	uint8_t read_data_tx_buffer[size + 4];
	uint8_t read_data_rx_buffer[size + 4];

	read_data_tx_buffer[0] = 0x03;
	read_data_tx_buffer[1] = 0x00;
	read_data_tx_buffer[2] = 0x10;
	read_data_tx_buffer[3] = 0x00;
	memset(&read_data_tx_buffer[4], 0x00, size);

	char uart_message[128];
	uint16_t offset = 0;

	flash_spi_transfer(read_data_tx_buffer, read_data_rx_buffer, size + 4);

	memcpy(rx_buffer, &read_data_rx_buffer[4], size);

	for (uint8_t i = 0; i < size; i++) {
		offset += snprintf(&uart_message[offset], sizeof(uart_message) - offset,
				"0x%02X%s", rx_buffer[i], (i == (size - 1)) ? "\r\n" : ", ");
	}

	HAL_UART_Transmit(&huart2, (uint8_t*) uart_message, strlen(uart_message),
	HAL_MAX_DELAY);

}

void write_single_page_SPI(void) {

	uint8_t page_program_tx_buffer[] = { 0x02, 0x00, 0x10, 0x00, 0x48, 0xAF,
			0xF1 };
	uint8_t page_program_rx_buffer[7];

	write_enable_spi();
	flash_spi_transfer(page_program_tx_buffer, page_program_rx_buffer, 7);
	flash_wait_while_busy();

}

void write_flash_record(uint8_t *tx_buffer, uint8_t *rx_buffer, uint8_t size) {

	uint8_t page_program_tx_buffer[size + 4];
	uint8_t page_program_rx_buffer[size + 4];

	page_program_tx_buffer[0] = 0x02;
	page_program_tx_buffer[1] = 0x00;
	page_program_tx_buffer[2] = 0x10;
	page_program_tx_buffer[3] = 0x00;
	memcpy(&page_program_tx_buffer[4], tx_buffer, size);

	write_enable_spi();
	flash_spi_transfer(page_program_tx_buffer, page_program_rx_buffer,
			size + 4);
	flash_wait_while_busy();
}

void sector_erase(void) {

	uint8_t sector_erase_tx_buffer[] = { 0x20, 0x00, 0x10, 0x00 };
	uint8_t sector_erase_rx_buffer[4];

	write_enable_spi();
	flash_spi_transfer(sector_erase_tx_buffer, sector_erase_rx_buffer, 4);
	flash_wait_while_busy();
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */

	char uart_message[32];

	FlashRecord_t flash_record_read;
	FlashRecord_t flash_record;
	flash_record.voltage = 250.7;
	flash_record.current = 9.8;
	flash_record.temperature = 28.1;
	flash_record.timestamp_ms = 500;

	flash_record.crc = modbus_crc16((uint8_t*) &flash_record,
			offsetof(FlashRecord_t, crc));

	uint8_t tx_write_record_buffer[18];
	uint8_t rx_write_record_buffer[18];

	uint8_t rx_read_record_buffer[18];

	memcpy(&tx_write_record_buffer[0], &flash_record.voltage, sizeof(float));
	memcpy(&tx_write_record_buffer[4], &flash_record.current, sizeof(float));
	memcpy(&tx_write_record_buffer[8], &flash_record.temperature,
			sizeof(float));
	memcpy(&tx_write_record_buffer[12], &flash_record.timestamp_ms,
			sizeof(uint32_t));
	memcpy(&tx_write_record_buffer[16], &flash_record.crc, sizeof(uint16_t));

//	AppQueuesInit();
//	AppTasksInit();

	read_flash_record(rx_read_record_buffer, 18);
	memcpy(&flash_record_read.voltage, &rx_read_record_buffer[0],
			sizeof(float));
	memcpy(&flash_record_read.current, &rx_read_record_buffer[4],
			sizeof(float));
	memcpy(&flash_record_read.temperature, &rx_read_record_buffer[8],
			sizeof(float));
	memcpy(&flash_record_read.timestamp_ms, &rx_read_record_buffer[12],
			sizeof(uint32_t));
	memcpy(&flash_record_read.crc, &rx_read_record_buffer[16],
			sizeof(uint16_t));

	uint16_t computed_crc16 = modbus_crc16((uint8_t*) &flash_record_read,
			offsetof(FlashRecord_t, crc));

	if (computed_crc16 == flash_record_read.crc) {
		snprintf(uart_message, sizeof(uart_message), "OK\r\n");
	} else {
		snprintf(uart_message, sizeof(uart_message), "MISMATCH\r\n");
	}

	HAL_UART_Transmit(&huart2, (uint8_t*) uart_message, strlen(uart_message),
	HAL_MAX_DELAY);

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
