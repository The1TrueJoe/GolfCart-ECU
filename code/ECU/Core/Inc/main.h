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
#define AUX_RELAY_1_Pin GPIO_PIN_2
#define AUX_RELAY_1_GPIO_Port GPIOE
#define AUX_RELAY_2_Pin GPIO_PIN_3
#define AUX_RELAY_2_GPIO_Port GPIOE
#define AUX_RELAY_3_Pin GPIO_PIN_4
#define AUX_RELAY_3_GPIO_Port GPIOE
#define AUX_RELAY_4_Pin GPIO_PIN_5
#define AUX_RELAY_4_GPIO_Port GPIOE
#define AUX_RELAY_5_Pin GPIO_PIN_6
#define AUX_RELAY_5_GPIO_Port GPIOE
#define AUX_RELAY_6_Pin GPIO_PIN_13
#define AUX_RELAY_6_GPIO_Port GPIOC
#define MPU_SDA_Pin GPIO_PIN_0
#define MPU_SDA_GPIO_Port GPIOF
#define MPU_SCL_Pin GPIO_PIN_1
#define MPU_SCL_GPIO_Port GPIOF
#define Steering_L_PWM_Pin GPIO_PIN_3
#define Steering_L_PWM_GPIO_Port GPIOA
#define Steering_L_EN_Pin GPIO_PIN_4
#define Steering_L_EN_GPIO_Port GPIOA
#define Steering_R_PWM_Pin GPIO_PIN_5
#define Steering_R_PWM_GPIO_Port GPIOA
#define Steering_R_EN_Pin GPIO_PIN_6
#define Steering_R_EN_GPIO_Port GPIOA
#define Brake_R_PWM_Pin GPIO_PIN_0
#define Brake_R_PWM_GPIO_Port GPIOB
#define Brake_L_PWM_Pin GPIO_PIN_1
#define Brake_L_PWM_GPIO_Port GPIOB
#define Brake_R_EN_Pin GPIO_PIN_2
#define Brake_R_EN_GPIO_Port GPIOB
#define Brake_L_EN_Pin GPIO_PIN_15
#define Brake_L_EN_GPIO_Port GPIOF
#define Steering_ENC_1_Pin GPIO_PIN_7
#define Steering_ENC_1_GPIO_Port GPIOE
#define Steering_ENC_2_Pin GPIO_PIN_8
#define Steering_ENC_2_GPIO_Port GPIOE
#define Enable_Pin GPIO_PIN_10
#define Enable_GPIO_Port GPIOE
#define Direction_Pin GPIO_PIN_11
#define Direction_GPIO_Port GPIOE
#define Speed_SCK_Pin GPIO_PIN_12
#define Speed_SCK_GPIO_Port GPIOE
#define Speed_CS_Pin GPIO_PIN_13
#define Speed_CS_GPIO_Port GPIOE
#define Speed_SI_Pin GPIO_PIN_14
#define Speed_SI_GPIO_Port GPIOE
#define Accelerator_Detect_LED_Pin GPIO_PIN_15
#define Accelerator_Detect_LED_GPIO_Port GPIOE
#define Brake_Detect_LED_Pin GPIO_PIN_10
#define Brake_Detect_LED_GPIO_Port GPIOB
#define Steering_L_EN2_Pin GPIO_PIN_10
#define Steering_L_EN2_GPIO_Port GPIOD
#define Steering_R_EN2_Pin GPIO_PIN_11
#define Steering_R_EN2_GPIO_Port GPIOD
#define Steering_L_PWM2_Pin GPIO_PIN_12
#define Steering_L_PWM2_GPIO_Port GPIOD
#define Steering_R_PWM2_Pin GPIO_PIN_13
#define Steering_R_PWM2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
