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
#include "spi.h"
#include "cmsis_os2.h"
#include "app_tasks.h"
#include "app_queues.h"

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

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_SPI2_Init();
	MX_USART6_UART_Init();

	osKernelInitialize();

	/*
	 * Queues must exist before AppTasksInit(), since tasks may touch them
	 * as soon as the scheduler starts.
	 */
	AppQueuesInit();

	AppTasksInit();

	/*
	 * Kept from the generated project - contents should be checked
	 * separately before deciding whether it's still needed.
	 */
	MX_FREERTOS_Init();

	osKernelStart();

	/*
	 * Should never return; kept as a final guard in case it does.
	 */
	while (1) {
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/**
	 * Initialise the RCC Oscillators according to the specified parameters
	 * in RCC_OscInitTypeDef.
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

	/**
	 * Initialise the CPU, AHB and APB bus clocks.
	 */
	RCC_ClkInitStruct.ClockType =
	RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;

	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
	FLASH_LATENCY_2) != HAL_OK) {

		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode.
 *
 * TIM1 is used as the HAL time base; each update interrupt increments the
 * HAL tick used by HAL_GetTick()/HAL_Delay().
 *
 * @param  htim Timer handle.
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 *
 * Sends one message over UART2, then blinks LD2 at roughly 1 Hz forever.
 * LD2 is not used anywhere else in the firmware, so this pattern always
 * means "Error_Handler was called" - distinct from
 * vApplicationStackOverflowHook()'s solid-on LD2 (stack overflow).
 *
 * The delay is a manually calibrated busy loop, not HAL_GetTick() or
 * vTaskDelay(): interrupts are disabled below, so the TIM1 interrupt
 * driving HAL_GetTick() would never fire again, and this function can also
 * be reached before the scheduler starts (a SystemClock_Config() failure
 * in main()), where vTaskDelay() would be invalid.
 *
 * Note: if Error_Handler() is reached from SystemClock_Config() itself,
 * neither GPIO nor UART2 are initialised yet, so neither signal will be
 * visible in that specific case - a debugger is the only option there.
 */
void Error_Handler(void) {

	static const char error_message[] = "Error_Handler occurred\r\n";

	HAL_UART_Transmit(&huart2, (uint8_t*) error_message,
			sizeof(error_message) - 1, HAL_MAX_DELAY);

	__disable_irq();

	while (1) {

		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		for (volatile uint32_t i = 0; i < 8000000UL; i++) {
		}
	}
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
