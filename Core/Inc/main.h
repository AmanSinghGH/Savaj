/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIR8_Pin GPIO_PIN_3
#define DIR8_GPIO_Port GPIOF
#define E1A_Pin GPIO_PIN_5
#define E1A_GPIO_Port GPIOC
#define E1A_EXTI_IRQn EXTI9_5_IRQn
#define E2A_Pin GPIO_PIN_2
#define E2A_GPIO_Port GPIOB
#define E2A_EXTI_IRQn EXTI2_IRQn
#define DIR3_Pin GPIO_PIN_11
#define DIR3_GPIO_Port GPIOF
#define DIR4_Pin GPIO_PIN_12
#define DIR4_GPIO_Port GPIOF
#define DIR5_Pin GPIO_PIN_13
#define DIR5_GPIO_Port GPIOF
#define DIR6_Pin GPIO_PIN_14
#define DIR6_GPIO_Port GPIOF
#define DIR7_Pin GPIO_PIN_15
#define DIR7_GPIO_Port GPIOF
#define E1B_Pin GPIO_PIN_12
#define E1B_GPIO_Port GPIOB
#define E1B_EXTI_IRQn EXTI15_10_IRQn
#define E3B_Pin GPIO_PIN_13
#define E3B_GPIO_Port GPIOB
#define E3B_EXTI_IRQn EXTI15_10_IRQn
#define E3A_Pin GPIO_PIN_14
#define E3A_GPIO_Port GPIOB
#define E3A_EXTI_IRQn EXTI15_10_IRQn
#define E2B_Pin GPIO_PIN_15
#define E2B_GPIO_Port GPIOB
#define E2B_EXTI_IRQn EXTI15_10_IRQn
#define P1_Pin GPIO_PIN_4
#define P1_GPIO_Port GPIOG
#define DIR1_Pin GPIO_PIN_5
#define DIR1_GPIO_Port GPIOG
#define P2_Pin GPIO_PIN_6
#define P2_GPIO_Port GPIOG
#define P3_Pin GPIO_PIN_7
#define P3_GPIO_Port GPIOG
#define DIR2_Pin GPIO_PIN_8
#define DIR2_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
