/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define BLUE_LED_Pin GPIO_PIN_13
#define BLUE_LED_GPIO_Port GPIOC
#define THR8_Pin GPIO_PIN_0
#define THR8_GPIO_Port GPIOA
#define THR7_Pin GPIO_PIN_1
#define THR7_GPIO_Port GPIOA
#define THR6_Pin GPIO_PIN_2
#define THR6_GPIO_Port GPIOA
#define THR5_Pin GPIO_PIN_3
#define THR5_GPIO_Port GPIOA
#define THR4_Pin GPIO_PIN_6
#define THR4_GPIO_Port GPIOA
#define THR3_Pin GPIO_PIN_7
#define THR3_GPIO_Port GPIOA
#define THR2_Pin GPIO_PIN_0
#define THR2_GPIO_Port GPIOB
#define THR1_Pin GPIO_PIN_1
#define THR1_GPIO_Port GPIOB
#define RGBLED_R_Pin GPIO_PIN_8
#define RGBLED_R_GPIO_Port GPIOA
#define RGBLED_G_Pin GPIO_PIN_6
#define RGBLED_G_GPIO_Port GPIOB
#define RGBLED_B_Pin GPIO_PIN_7
#define RGBLED_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
