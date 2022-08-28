/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define busy_led_Pin GPIO_PIN_14
#define busy_led_GPIO_Port GPIOC
#define DS_1_Pin GPIO_PIN_0
#define DS_1_GPIO_Port GPIOA
#define DS_2_Pin GPIO_PIN_1
#define DS_2_GPIO_Port GPIOA
#define DS_3_Pin GPIO_PIN_2
#define DS_3_GPIO_Port GPIOA
#define DS_4_Pin GPIO_PIN_3
#define DS_4_GPIO_Port GPIOA
#define DS_5_Pin GPIO_PIN_4
#define DS_5_GPIO_Port GPIOA
#define COIL_1_Pin GPIO_PIN_0
#define COIL_1_GPIO_Port GPIOB
#define COIL_2_Pin GPIO_PIN_1
#define COIL_2_GPIO_Port GPIOB
#define COIL_3_Pin GPIO_PIN_2
#define COIL_3_GPIO_Port GPIOB
#define DS_6_Pin GPIO_PIN_12
#define DS_6_GPIO_Port GPIOB
#define DS_7_Pin GPIO_PIN_13
#define DS_7_GPIO_Port GPIOB
#define DS_8_Pin GPIO_PIN_14
#define DS_8_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_11
#define DIR_GPIO_Port GPIOA
#define COIL_4_Pin GPIO_PIN_3
#define COIL_4_GPIO_Port GPIOB
#define COIL_5_Pin GPIO_PIN_4
#define COIL_5_GPIO_Port GPIOB
#define COIL_6_Pin GPIO_PIN_5
#define COIL_6_GPIO_Port GPIOB
#define COIL_7_Pin GPIO_PIN_6
#define COIL_7_GPIO_Port GPIOB
#define COIL_8_Pin GPIO_PIN_7
#define COIL_8_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
