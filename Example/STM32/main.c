/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus.h"
#include "modbus_rtu.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define buadrate	19200
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t buff[250];
uint8_t response[250];
int timeout_3_5C = 40; // ms ((1000 * 4 * 11) / buadrate);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
ModbusStatus_t modbus_uart_receive_Handler(uint8_t *Data) {
	ModbusStatus_t res;
	res = (ModbusStatus_t) HAL_UART_Receive(&huart1, Data, 1, timeout_3_5C);
	return res;
}

void modbus_uart_transmit_Handler(uint8_t *Data, uint16_t length) {
	HAL_Delay(5);
	HAL_UART_Transmit(&huart1, Data, length, 100);
}

void modbus_uart_init_Handler(Serial_t *Serial) {
	huart1.Instance = (USART_TypeDef*) Serial->UART;
	huart1.Init.BaudRate = Serial->BaudRate;

	if (Serial->StopBit == StopBit_1)
		huart1.Init.StopBits = UART_STOPBITS_1;
	else
		huart1.Init.StopBits = UART_STOPBITS_2;

	if (Serial->Parity == NONE_PARITY)
		huart1.Init.Parity = UART_PARITY_NONE;
	else if (Serial->Parity == ODD_PARITY)
		huart1.Init.Parity = UART_PARITY_ODD;
	else
		huart1.Init.Parity = UART_PARITY_EVEN;

	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(&huart1);
}
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
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	/* Get DIP switch */
	uint8_t dip_switch = HAL_GPIO_ReadPin(DS_1_GPIO_Port, DS_1_Pin)
			| (HAL_GPIO_ReadPin(DS_1_GPIO_Port, DS_2_Pin) << 1)
			| (HAL_GPIO_ReadPin(DS_1_GPIO_Port, DS_3_Pin) << 2)
			| (HAL_GPIO_ReadPin(DS_1_GPIO_Port, DS_4_Pin) << 3);

	/* Get Address (That made with setting on RS-485)*/
	if (dip_switch == 0x0f) {

	}
	/* Setting Mode (change device setting: serial and address)*/
	else if (dip_switch == 0x00) {

	}
	/* Default setting */
	else {
		set_slave_ID(dip_switch);

		Serial_t default_serial;
		default_serial.UART = (uint32_t*) USART1;
		default_serial.BaudRate = 9600;
		default_serial.Parity = NONE_PARITY;
		default_serial.StopBit = StopBit_1;

		modbus_serial_init(&default_serial);
	}

	unsigned char previous_coils_val[8];

	/* All RELAY PRE SET/RESET */
	HAL_GPIO_WritePin(COIL_1_GPIO_Port, COIL_1_Pin, !Get_coil_status(1));
	HAL_GPIO_WritePin(COIL_2_GPIO_Port, COIL_2_Pin, !Get_coil_status(2));

	while (1) {

		/* Get the current coil status from memory */
		previous_coils_val[0] = Get_coil_status(1)
				| (Get_coil_status(2) << 1) /* | 2 ...*/;

		/* Modbus network monitor to receive frames from the master device */
		ModbusStatus_t res = MODBUS_RTU_MONITOR(buff, 3000, &uwTick, Normal);

		/* Busy led: Changing the status of the busy LED (according to 
		 the status of the Modbus monitor function) */
		if (res == MODBUS_OK)
			HAL_GPIO_WritePin(busy_led_GPIO_Port, busy_led_Pin, GPIO_PIN_SET);
		else if (res == MODBUS_MONITOR_TIMEOUT)
			HAL_GPIO_WritePin(busy_led_GPIO_Port, busy_led_Pin, GPIO_PIN_RESET);

		/* SET/RESET RELAY if Changed coil's status */
		/* REL 1 */
		if (Get_coil_status(1) != (previous_coils_val[0] & 1)) {

			HAL_GPIO_WritePin(COIL_1_GPIO_Port, COIL_1_Pin,
					!Get_coil_status(1));
		}
		/* REL 2 */
		if (Get_coil_status(2) != (previous_coils_val[0] >> 1 & 1)) {

			HAL_GPIO_WritePin(COIL_2_GPIO_Port, COIL_2_Pin,
					!Get_coil_status(2));
		}
		/* REL n ...  */

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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */
	huart1.Init.BaudRate = buadrate;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(busy_led_GPIO_Port, busy_led_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, COIL_1_Pin | COIL_2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : busy_led_Pin */
	GPIO_InitStruct.Pin = busy_led_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(busy_led_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DS_1_Pin DS_2_Pin DS_3_Pin DS_4_Pin */
	GPIO_InitStruct.Pin = DS_1_Pin | DS_2_Pin | DS_3_Pin | DS_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : COIL_1_Pin COIL_2_Pin */
	GPIO_InitStruct.Pin = COIL_1_Pin | COIL_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
