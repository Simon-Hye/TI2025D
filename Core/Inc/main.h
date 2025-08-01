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
#include "stm32g4xx_hal.h"

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
#define CS0_Pin GPIO_PIN_5
#define CS0_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_6
#define CS1_GPIO_Port GPIOA
#define AS2_Pin GPIO_PIN_7
#define AS2_GPIO_Port GPIOA
#define AS1_Pin GPIO_PIN_5
#define AS1_GPIO_Port GPIOC
#define BS0_Pin GPIO_PIN_2
#define BS0_GPIO_Port GPIOB
#define BS2_Pin GPIO_PIN_10
#define BS2_GPIO_Port GPIOB
#define AS0_Pin GPIO_PIN_6
#define AS0_GPIO_Port GPIOC
#define A_E_Pin GPIO_PIN_7
#define A_E_GPIO_Port GPIOC
#define A_G_Pin GPIO_PIN_8
#define A_G_GPIO_Port GPIOC
#define B_G_Pin GPIO_PIN_9
#define B_G_GPIO_Port GPIOC
#define C_E_Pin GPIO_PIN_9
#define C_E_GPIO_Port GPIOA
#define Z_A_Pin GPIO_PIN_11
#define Z_A_GPIO_Port GPIOA
#define Z_B_Pin GPIO_PIN_12
#define Z_B_GPIO_Port GPIOA
#define B_E_Pin GPIO_PIN_4
#define B_E_GPIO_Port GPIOB
#define BS1_Pin GPIO_PIN_5
#define BS1_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_6
#define CS2_GPIO_Port GPIOB
#define Z_C_Pin GPIO_PIN_9
#define Z_C_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
# define RX_BUF_LEN 256
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
