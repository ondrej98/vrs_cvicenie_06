/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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
uint8_t ledState = 0;
extern RX_UART_DATA RXDataIndexer;
extern uint8_t USART2_TX_IsEmpty;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	/* USER CODE BEGIN 1 */
	RXDataIndexer = RX_UART_DATA_None;
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* System interrupt init*/

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	USART2_RegisterCallback(USART_ProcessRxData);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
		if (ledState == 1) {
			USART2_TransmitData(USART2, (uint8_t*) LED_ON_STR, LED_ON_STR_LEN);
		} else {
			USART2_TransmitData(USART2, (uint8_t*) LED_OFF_STR,
					LED_OFF_STR_LEN);
		}
		LL_mDelay(5000);
	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0) {
	}
	LL_RCC_HSI_Enable();

	/* Wait till HSI is ready */
	while (LL_RCC_HSI_IsReady() != 1) {

	}
	LL_RCC_HSI_SetCalibTrimming(16);
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_5);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1) {

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {

	}
	LL_Init1msTick(20000000);
	LL_SetSystemCoreClock(20000000);
}

/* USER CODE BEGIN 4 */
void USART_ProcessRxData(uint8_t chr) {
	switch (RXDataIndexer) {
	default:
		if (chr == 'l')
			RXDataIndexer = RX_UART_DATA_l;
		break;
	case RX_UART_DATA_l:
		if (chr == 'e')
			RXDataIndexer = RX_UART_DATA_le;
		else if (chr != ENTER)
			RXDataIndexer = RX_UART_DATA_None;
		break;
	case RX_UART_DATA_le:
		if (chr == 'd')
			RXDataIndexer = RX_UART_DATA_led;
		else if (chr != ENTER)
			RXDataIndexer = RX_UART_DATA_None;
		break;
	case RX_UART_DATA_led:
		if (chr == 'O')
			RXDataIndexer = RX_UART_DATA_ledO;
		else if (chr != ENTER)
			RXDataIndexer = RX_UART_DATA_None;
		break;
	case RX_UART_DATA_ledO:
		if (chr == 'N')
			RXDataIndexer = RX_UART_DATA_ledON;
		else if (chr == 'F')
			RXDataIndexer = RX_UART_DATA_ledOF;
		else if (chr != ENTER)
			RXDataIndexer = RX_UART_DATA_None;
		break;
	case RX_UART_DATA_ledON:
		LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
		ledState = 1;
		RXDataIndexer = RX_UART_DATA_None;
		break;
	case RX_UART_DATA_ledOF:
		if (chr == 'F')
			RXDataIndexer = RX_UART_DATA_ledOFF;
		else if (chr != ENTER)
			RXDataIndexer = RX_UART_DATA_None;
		break;
	case RX_UART_DATA_ledOFF:
		LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
		RXDataIndexer = RX_UART_DATA_None;
		ledState = 0;
		break;

	}
}
void USART2_TransmitData(USART_TypeDef *USARTx, uint8_t ptr[], uint16_t lenght) {
	if (ptr != NULL) {
		for (uint16_t i = 0; i < lenght; i++) {
			LL_USART_TransmitData8(USARTx, ptr[i]);
			LL_mDelay(5);
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
