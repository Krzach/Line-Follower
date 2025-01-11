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
#define Battery_Pin GPIO_PIN_0
#define Battery_GPIO_Port GPIOC
#define Sn8_Pin GPIO_PIN_5
#define Sn8_GPIO_Port GPIOA
#define Sn7_Pin GPIO_PIN_6
#define Sn7_GPIO_Port GPIOA
#define Sn6_Pin GPIO_PIN_7
#define Sn6_GPIO_Port GPIOA
#define IN4_Pin GPIO_PIN_5
#define IN4_GPIO_Port GPIOC
#define Sn1_Pin GPIO_PIN_10
#define Sn1_GPIO_Port GPIOB
#define IN3_Pin GPIO_PIN_6
#define IN3_GPIO_Port GPIOC
#define Sn4_Pin GPIO_PIN_7
#define Sn4_GPIO_Port GPIOC
#define IN2_Pin GPIO_PIN_8
#define IN2_GPIO_Port GPIOC
#define IN1_Pin GPIO_PIN_9
#define IN1_GPIO_Port GPIOC
#define Sn2_Pin GPIO_PIN_8
#define Sn2_GPIO_Port GPIOA
#define Sn3_Pin GPIO_PIN_9
#define Sn3_GPIO_Port GPIOA
#define Sn5_Pin GPIO_PIN_11
#define Sn5_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ENA_T3_CH1_Pin GPIO_PIN_4
#define ENA_T3_CH1_GPIO_Port GPIOB
#define ENB_T3_CH2_Pin GPIO_PIN_5
#define ENB_T3_CH2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef struct {
	int16_t left;
	int16_t right;
} mSpeed;

// Enum definition
typedef uint8_t driveState;
#define STOP ((driveState)0)
#define REG_MPC ((driveState)1)
#define REG_PID ((driveState)2)

extern driveState carState;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
