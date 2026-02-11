/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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
#define TEC_ENABLE_Pin GPIO_PIN_5
#define TEC_ENABLE_GPIO_Port GPIOF
#define SPI5_EN_Pin GPIO_PIN_6
#define SPI5_EN_GPIO_Port GPIOF
#define PHOTODIODE_Pin GPIO_PIN_0
#define PHOTODIODE_GPIO_Port GPIOA
#define MASTER_START_Pin GPIO_PIN_2
#define MASTER_START_GPIO_Port GPIOA
#define MASTER_START_EXTI_IRQn EXTI2_IRQn
#define FLEX2_Pin GPIO_PIN_3
#define FLEX2_GPIO_Port GPIOA
#define T_SET_Pin GPIO_PIN_4
#define T_SET_GPIO_Port GPIOA
#define Program_Indicator_Pin GPIO_PIN_8
#define Program_Indicator_GPIO_Port GPIOA
#define Error_Indicator_Pin GPIO_PIN_9
#define Error_Indicator_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
