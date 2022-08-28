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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern __IO uint32_t uwTick;
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
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, Data, length, 100);
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
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
int main(void)
{
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

	/* Get address from DIP switch */
	uint8_t dip_switch_address = (!HAL_GPIO_ReadPin(DS_1_GPIO_Port, DS_1_Pin))
			| (!HAL_GPIO_ReadPin(DS_2_GPIO_Port, DS_2_Pin) << 1)
			| (!HAL_GPIO_ReadPin(DS_3_GPIO_Port, DS_3_Pin) << 2)
			| (!HAL_GPIO_ReadPin(DS_4_GPIO_Port, DS_4_Pin) << 3)
			| (!HAL_GPIO_ReadPin(DS_5_GPIO_Port, DS_5_Pin) << 4);

	/* Get Baud rate from DIP switch */
	uint8_t dip_switch_baudrate = (!HAL_GPIO_ReadPin(DS_6_GPIO_Port, DS_6_Pin))
			| (!HAL_GPIO_ReadPin(DS_7_GPIO_Port, DS_7_Pin) << 1)
			| (!HAL_GPIO_ReadPin(DS_8_GPIO_Port, DS_8_Pin) << 2);

	/* Supported baud rates are 2400, 4800, 9600, 14400, 19200, 28800, 38400 and 115200 baud. */
	uint32_t baudrate;
	switch (dip_switch_baudrate) {
	case 1:
		baudrate = 2400;
		break;
	case 2:
		baudrate = 4800;
		break;
	case 3:
		baudrate = 14400;
		break;
	case 4:
		baudrate = 19200;
		break;
	case 5:
		baudrate = 28800;
		break;
	case 6:
		baudrate = 38400;
		break;
	case 7:
		baudrate = 115200;
		break;
	default:
		baudrate = 9600;
		break;
	}

	/* Default setting */
	set_slave_ID(dip_switch_address);

	Serial_t default_serial;
	default_serial.UART = (uint32_t*) USART1;
	default_serial.BaudRate = baudrate;
	default_serial.Parity = NONE_PARITY;
	default_serial.StopBit = StopBit_1;

	modbus_serial_init(&default_serial);


	unsigned char previous_coils_val[1];
	uint16_t previous_holding_register_val[8];

	/* All RELAY PRESET */
	HAL_GPIO_WritePin(COIL_1_GPIO_Port, COIL_1_Pin, !Get_coil_status(1) | !Get_holding_register(1));
	HAL_GPIO_WritePin(COIL_2_GPIO_Port, COIL_2_Pin, !Get_coil_status(2) | !Get_holding_register(2));
	HAL_GPIO_WritePin(COIL_3_GPIO_Port, COIL_3_Pin, !Get_coil_status(3) | !Get_holding_register(3));
	HAL_GPIO_WritePin(COIL_4_GPIO_Port, COIL_4_Pin, !Get_coil_status(4) | !Get_holding_register(4));
	HAL_GPIO_WritePin(COIL_5_GPIO_Port, COIL_5_Pin, !Get_coil_status(5) | !Get_holding_register(5));
	HAL_GPIO_WritePin(COIL_6_GPIO_Port, COIL_6_Pin, !Get_coil_status(6) | !Get_holding_register(6));
	HAL_GPIO_WritePin(COIL_7_GPIO_Port, COIL_7_Pin, !Get_coil_status(7) | !Get_holding_register(7));
	HAL_GPIO_WritePin(COIL_8_GPIO_Port, COIL_8_Pin, !Get_coil_status(8) | !Get_holding_register(8));



	while (1) {

		/* Get the current coil status from memory */
		previous_coils_val[0] = Get_coil_status(1) | (Get_coil_status(2) << 1)
				| (Get_coil_status(3) << 2) | (Get_coil_status(4) << 3)
				| (Get_coil_status(5) << 4) | (Get_coil_status(6) << 5)
				| (Get_coil_status(7) << 6) | (Get_coil_status(8) << 7);

		/* Get the current holding register from memory */
		for (uint8_t add = 0; add < 8; add++) {
			previous_holding_register_val[add] = Get_holding_register(add + 1);
		}


		/* Modbus network monitor to receive frames from the master device */
		ModbusStatus_t res = MODBUS_RTU_MONITOR(buff, 3000, &uwTick, Normal);

		/* Busy led: Changing the status of the busy LED (according to 
		 the status of the Modbus monitor function) */
		if (res == MODBUS_OK)
			HAL_GPIO_WritePin(busy_led_GPIO_Port, busy_led_Pin, GPIO_PIN_RESET);
		else if (res == MODBUS_MONITOR_TIMEOUT)
			HAL_GPIO_WritePin(busy_led_GPIO_Port, busy_led_Pin, GPIO_PIN_SET);


		/* SET/RESET RELAY if Changed coil's status */

		/* REL 1 */
		if (Get_coil_status(1) != (previous_coils_val[0] & 1)) {
			HAL_GPIO_WritePin(COIL_1_GPIO_Port, COIL_1_Pin,
					!Get_coil_status(1));
		}
		if (Get_holding_register(1) != previous_holding_register_val[0]) {
			HAL_GPIO_WritePin(COIL_1_GPIO_Port, COIL_1_Pin,
					!Get_holding_register(1));
		}

		/* REL 2 */
		if (Get_coil_status(2) != (previous_coils_val[0] >> 1 & 1)) {
			HAL_GPIO_WritePin(COIL_2_GPIO_Port, COIL_2_Pin,
					!Get_coil_status(2));
		}
		if (Get_holding_register(2) != previous_holding_register_val[1]) {
			HAL_GPIO_WritePin(COIL_2_GPIO_Port, COIL_2_Pin,
					!Get_holding_register(2));
		}

		/* REL 3 */
		if (Get_coil_status(3) != (previous_coils_val[0] >> 2 & 1)) {
			HAL_GPIO_WritePin(COIL_3_GPIO_Port, COIL_3_Pin,
					!Get_coil_status(3));
		}
		if (Get_holding_register(3) != previous_holding_register_val[2]) {
			HAL_GPIO_WritePin(COIL_3_GPIO_Port, COIL_3_Pin,
					!Get_holding_register(3));
		}

		/* REL 4 */
		if (Get_coil_status(4) != (previous_coils_val[0] >> 3 & 1)) {
			HAL_GPIO_WritePin(COIL_4_GPIO_Port, COIL_4_Pin,
					!Get_coil_status(4));
		}
		if (Get_holding_register(4) != previous_holding_register_val[3]) {
			HAL_GPIO_WritePin(COIL_4_GPIO_Port, COIL_4_Pin,
					!Get_holding_register(4));
		}

		/* REL 5 */
		if (Get_coil_status(5) != (previous_coils_val[0] >> 4 & 1)) {
			HAL_GPIO_WritePin(COIL_5_GPIO_Port, COIL_5_Pin,
					!Get_coil_status(5));
		}
		if (Get_holding_register(5) != previous_holding_register_val[4]) {
			HAL_GPIO_WritePin(COIL_5_GPIO_Port, COIL_5_Pin,
					!Get_holding_register(5));
		}

		/* REL 6 */
		if (Get_coil_status(6) != (previous_coils_val[0] >> 5 & 1)) {
			HAL_GPIO_WritePin(COIL_6_GPIO_Port, COIL_6_Pin,
					!Get_coil_status(6));
		}
		if (Get_holding_register(6) != previous_holding_register_val[5]) {
			HAL_GPIO_WritePin(COIL_6_GPIO_Port, COIL_6_Pin,
					!Get_holding_register(6));
		}

		/* REL 7 */
		if (Get_coil_status(7) != (previous_coils_val[0] >> 6 & 1)) {
			HAL_GPIO_WritePin(COIL_7_GPIO_Port, COIL_7_Pin,
					!Get_coil_status(7));
		}
		if (Get_holding_register(7) != previous_holding_register_val[6]) {
			HAL_GPIO_WritePin(COIL_7_GPIO_Port, COIL_7_Pin,
					!Get_holding_register(7));
		}

		/* REL 8 */
		if (Get_coil_status(8) != (previous_coils_val[0] >> 7 & 1)) {
			HAL_GPIO_WritePin(COIL_8_GPIO_Port, COIL_8_Pin,
					!Get_coil_status(8));
		}
		if (Get_holding_register(8) != previous_holding_register_val[7]) {
			HAL_GPIO_WritePin(COIL_8_GPIO_Port, COIL_8_Pin,
					!Get_holding_register(8));
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	huart1.Init.BaudRate = 9600;
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(busy_led_GPIO_Port, busy_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, COIL_1_Pin|COIL_2_Pin|COIL_3_Pin|COIL_4_Pin
                          |COIL_5_Pin|COIL_6_Pin|COIL_7_Pin|COIL_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : busy_led_Pin */
  GPIO_InitStruct.Pin = busy_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(busy_led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DS_1_Pin DS_2_Pin DS_3_Pin DS_4_Pin
                           DS_5_Pin */
  GPIO_InitStruct.Pin = DS_1_Pin|DS_2_Pin|DS_3_Pin|DS_4_Pin
                          |DS_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : COIL_1_Pin COIL_2_Pin COIL_3_Pin COIL_4_Pin
                           COIL_5_Pin COIL_6_Pin COIL_7_Pin COIL_8_Pin */
  GPIO_InitStruct.Pin = COIL_1_Pin|COIL_2_Pin|COIL_3_Pin|COIL_4_Pin
                          |COIL_5_Pin|COIL_6_Pin|COIL_7_Pin|COIL_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DS_6_Pin DS_7_Pin DS_8_Pin */
  GPIO_InitStruct.Pin = DS_6_Pin|DS_7_Pin|DS_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_Pin */
  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
