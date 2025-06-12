/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define IMU_ACC_CS_Pin GPIO_PIN_0
#define IMU_ACC_CS_GPIO_Port GPIOC
#define IMU_MOSI_Pin GPIO_PIN_1
#define IMU_MOSI_GPIO_Port GPIOC
#define IMU_MISO_Pin GPIO_PIN_2
#define IMU_MISO_GPIO_Port GPIOC
#define IMU_GYRO_CS_Pin GPIO_PIN_3
#define IMU_GYRO_CS_GPIO_Port GPIOC
#define PS2_DI_Pin GPIO_PIN_0
#define PS2_DI_GPIO_Port GPIOA
#define PS2_DO_Pin GPIO_PIN_2
#define PS2_DO_GPIO_Port GPIOA
#define WS2812_DIN_Pin GPIO_PIN_7
#define WS2812_DIN_GPIO_Port GPIOA
#define IMU_HEAT_Pin GPIO_PIN_1
#define IMU_HEAT_GPIO_Port GPIOB
#define PS2_CS_Pin GPIO_PIN_9
#define PS2_CS_GPIO_Port GPIOE
#define IMU_ACC_INT_Pin GPIO_PIN_10
#define IMU_ACC_INT_GPIO_Port GPIOE
#define IMU_GYRO_INT_Pin GPIO_PIN_12
#define IMU_GYRO_INT_GPIO_Port GPIOE
#define PS2_CLK_Pin GPIO_PIN_13
#define PS2_CLK_GPIO_Port GPIOE
#define IMU_SCK_Pin GPIO_PIN_13
#define IMU_SCK_GPIO_Port GPIOB
#define BUZ_Pin GPIO_PIN_15
#define BUZ_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
