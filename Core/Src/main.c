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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Gateway-specific knowledge: which slave/registers to poll, and how to
// behave around retries. modbus_master.c knows nothing about V/I/T or
// about there being exactly one slave on the bus - that knowledge lives
// entirely here, in the application layer.
ModbusTarget_t modbus_target = { .slave_id = 0x01, .function_code = 0x03,
		.register_start_hi = 0x00, .register_start_lo = 0x00,
		.register_count_hi = 0x00, .register_count_lo = 0x03 };

ModbusRetryPolicy_t modbus_retry_policy = { .poll_period_ms = 5000,
		.response_timeout_ms = 200, .max_attempts = 3 };

// modbus_request_payload is built from modbus_target at startup (single
// source of truth - see USER CODE 2), then CRC is appended into
// modbus_tx_buffer. Both stay constant for the lifetime of the program,
// since modbus_target never changes after initialization.
uint8_t modbus_request_payload[6];
uint8_t modbus_tx_buffer_size = 8;
uint8_t modbus_tx_buffer[8];
uint8_t modbus_rx_data[256];
char pc_tx_buffer[64];

// measurements[0] = voltage (V), [1] = current (A), [2] = temperature (C)
float measurements[3];

/**
 * @brief Extracts voltage/current/temperature from a validated response
 *        frame and prints them over USART2 (debug/PC link).
 *
 * @param frame  Pointer to a validated Modbus response of the expected
 *               length for modbus_target.register_count_lo registers.
 */
static void modbus_parse_measurements(uint8_t *frame) {
	measurements[0] = (float) ((frame[3] << 8) | frame[4]) / 10.0f;
	measurements[1] = (float) ((frame[5] << 8) | frame[6]) / 10.0f;
	measurements[2] = (float) ((frame[7] << 8) | frame[8]) / 10.0f;

	snprintf(pc_tx_buffer, sizeof(pc_tx_buffer),
			"Voltage:%.1fV, Current:%.1fA, Temperature:%.1fC\r\n",
			measurements[0], measurements[1], measurements[2]);
	HAL_UART_Transmit(&huart2, (uint8_t*) pc_tx_buffer, strlen(pc_tx_buffer),
	HAL_MAX_DELAY);
}

/**
 * @brief Reports the outcome of one modbus_master_poll() call over USART2.
 *
 * Note on the error messages: since modbus_master_poll() only returns
 * after all retry attempts are exhausted (except for MODBUS_OK/EXCEPTION,
 * which return immediately), every error case here is by definition the
 * status of the LAST attempt, reached after max_attempts tries - not an
 * aggregate of what happened across all of them.
 */
static void modbus_result(ModbusStatus_t modbus_master_poll_status,
		uint8_t exception_code) {

	switch (modbus_master_poll_status) {
	case MODBUS_OK:
		modbus_parse_measurements(modbus_rx_data);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // heartbeat on each successful cycle
		break;

	case MODBUS_ERR_EXCEPTION:
		snprintf(pc_tx_buffer, sizeof(pc_tx_buffer),
				"Exception 0x%02X - not retrying\r\n", exception_code);
		break;

	case MODBUS_ERR_LENGTH:
		snprintf(pc_tx_buffer, sizeof(pc_tx_buffer),
				"Frame length error on last attempt (after %u attempts)\r\n",
				modbus_retry_policy.max_attempts);
		break;

	case MODBUS_ERR_ADDRESS:
		snprintf(pc_tx_buffer, sizeof(pc_tx_buffer),
				"Slave address mismatch on last attempt (after %u attempts)\r\n",
				modbus_retry_policy.max_attempts);
		break;

	case MODBUS_ERR_CRC:
		snprintf(pc_tx_buffer, sizeof(pc_tx_buffer),
				"CRC16 mismatch on last attempt (after %u attempts)\r\n",
				modbus_retry_policy.max_attempts);
		break;

	case MODBUS_ERR_TIMEOUT:
		snprintf(pc_tx_buffer, sizeof(pc_tx_buffer),
				"No response from slave (silence after %u attempts)\r\n",
				modbus_retry_policy.max_attempts);
		break;

	default:
		// Defensive: unreachable with the current ModbusStatus_t values,
		// kept in case the enum grows in the future.
		snprintf(pc_tx_buffer, sizeof(pc_tx_buffer), "Unknown error\r\n");
		break;
	}

	if (modbus_master_poll_status != MODBUS_OK) {
		HAL_UART_Transmit(&huart2, (uint8_t*) pc_tx_buffer,
				strlen(pc_tx_buffer), HAL_MAX_DELAY);
	}
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

	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	// Single source of truth: request payload built from modbus_target,
	// not duplicated as a separate literal array.
	modbus_request_payload[0] = modbus_target.slave_id;
	modbus_request_payload[1] = modbus_target.function_code;
	modbus_request_payload[2] = modbus_target.register_start_hi;
	modbus_request_payload[3] = modbus_target.register_start_lo;
	modbus_request_payload[4] = modbus_target.register_count_hi;
	modbus_request_payload[5] = modbus_target.register_count_lo;

	uint16_t crc = modbus_crc16(modbus_request_payload,
			sizeof(modbus_request_payload));
	uint8_t crc_lsb = crc & 0x00FF;
	uint8_t crc_msb = (crc >> 8) & 0x00FF;

	memcpy(modbus_tx_buffer, modbus_request_payload,
			sizeof(modbus_request_payload));
	modbus_tx_buffer[6] = crc_lsb;
	modbus_tx_buffer[7] = crc_msb;

	modbus_master_init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		uint8_t exception_code = 0;
		uint32_t cycle_start = HAL_GetTick();

		ModbusStatus_t modbus_master_poll_status = modbus_master_poll(
				modbus_tx_buffer, modbus_tx_buffer_size, modbus_rx_data,
				&modbus_target, &modbus_retry_policy, &exception_code);
		modbus_result(modbus_master_poll_status, exception_code);

		// Wait out whatever remains of the fixed poll period, regardless
		// of how long the request/retry sequence above actually took.
		uint32_t elapsed = HAL_GetTick() - cycle_start;
		if (elapsed < modbus_retry_policy.poll_period_ms) {
			HAL_Delay(modbus_retry_policy.poll_period_ms - elapsed);
		}
	}
}
/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
