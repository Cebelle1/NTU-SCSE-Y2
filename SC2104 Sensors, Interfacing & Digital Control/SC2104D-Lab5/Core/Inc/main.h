/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define IN2_CO_Pin GPIO_PIN_9
#define IN2_CO_GPIO_Port GPIOE
#define IN1_CO_Pin GPIO_PIN_11
#define IN1_CO_GPIO_Port GPIOE
#define IN2_DO_Pin GPIO_PIN_13
#define IN2_DO_GPIO_Port GPIOE
#define IN1_DO_Pin GPIO_PIN_14
#define IN1_DO_GPIO_Port GPIOE
#define IMU_INT_Pin GPIO_PIN_12
#define IMU_INT_GPIO_Port GPIOB
#define DC_Pin GPIO_PIN_11
#define DC_GPIO_Port GPIOD
#define RESET__Pin GPIO_PIN_12
#define RESET__GPIO_Port GPIOD
#define SDIN_Pin GPIO_PIN_13
#define SDIN_GPIO_Port GPIOD
#define SCLK_Pin GPIO_PIN_14
#define SCLK_GPIO_Port GPIOD
#define Buzzer_Pin GPIO_PIN_8
#define Buzzer_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOA
#define IN2_A_PWM_Pin GPIO_PIN_8
#define IN2_A_PWM_GPIO_Port GPIOB
#define IN1_A_PWM_Pin GPIO_PIN_9
#define IN1_A_PWM_GPIO_Port GPIOB
#define USER_PB_Pin GPIO_PIN_0
#define USER_PB_GPIO_Port GPIOE
#define USER_PB_EXTI_IRQn EXTI0_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
