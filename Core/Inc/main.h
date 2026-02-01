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
#define E_stop_read_Pin GPIO_PIN_3
#define E_stop_read_GPIO_Port GPIOC
#define Jetson_TX_Pin GPIO_PIN_2
#define Jetson_TX_GPIO_Port GPIOA
#define Jetson_RX_Pin GPIO_PIN_3
#define Jetson_RX_GPIO_Port GPIOA
#define Relay_Trigger_Pin GPIO_PIN_5
#define Relay_Trigger_GPIO_Port GPIOA
#define LED_Trigger_Pin GPIO_PIN_6
#define LED_Trigger_GPIO_Port GPIOA
#define Ye_il_LED_Pin GPIO_PIN_12
#define Ye_il_LED_GPIO_Port GPIOD
#define Turuncu_LED_Pin GPIO_PIN_13
#define Turuncu_LED_GPIO_Port GPIOD
#define K_rm_z__LED_Pin GPIO_PIN_14
#define K_rm_z__LED_GPIO_Port GPIOD
#define Mavi_LED_Pin GPIO_PIN_15
#define Mavi_LED_GPIO_Port GPIOD
#define UNUSED_Pin GPIO_PIN_9
#define UNUSED_GPIO_Port GPIOA
#define RF_RX_Pin GPIO_PIN_10
#define RF_RX_GPIO_Port GPIOA
#define CAN_RX_Pin GPIO_PIN_8
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
