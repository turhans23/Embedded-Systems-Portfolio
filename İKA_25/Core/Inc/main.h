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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define E_STOP_TRIGGER_Pin GPIO_PIN_1
#define E_STOP_TRIGGER_GPIO_Port GPIOC
#define E_STOP_IN_Pin GPIO_PIN_3
#define E_STOP_IN_GPIO_Port GPIOC
#define Lazer_X_Pin GPIO_PIN_6
#define Lazer_X_GPIO_Port GPIOA
#define LED_OUTPUT_Pin GPIO_PIN_5
#define LED_OUTPUT_GPIO_Port GPIOC
#define LazerX_DIR_Pin GPIO_PIN_1
#define LazerX_DIR_GPIO_Port GPIOB
#define LazerY_DIR_Pin GPIO_PIN_2
#define LazerY_DIR_GPIO_Port GPIOB
#define LASER_OUT_Pin GPIO_PIN_10
#define LASER_OUT_GPIO_Port GPIOD
#define Ye_il_LED_Pin GPIO_PIN_12
#define Ye_il_LED_GPIO_Port GPIOD
#define Turuncu_LED_Pin GPIO_PIN_13
#define Turuncu_LED_GPIO_Port GPIOD
#define K_rm_z__LED_Pin GPIO_PIN_14
#define K_rm_z__LED_GPIO_Port GPIOD
#define Mavi_LED_Pin GPIO_PIN_15
#define Mavi_LED_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
