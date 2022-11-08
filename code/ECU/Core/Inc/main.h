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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>

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
#define TEMP_1_CS_Pin GPIO_PIN_6
#define TEMP_1_CS_GPIO_Port GPIOF
#define TEMP_SCK_Pin GPIO_PIN_7
#define TEMP_SCK_GPIO_Port GPIOF
#define TEMP_SO_Pin GPIO_PIN_8
#define TEMP_SO_GPIO_Port GPIOF
#define TEMP_2_CS_Pin GPIO_PIN_9
#define TEMP_2_CS_GPIO_Port GPIOF
#define Steering_Pot_Pin GPIO_PIN_3
#define Steering_Pot_GPIO_Port GPIOC
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
#define Break_L_PWM_Pin GPIO_PIN_1
#define Break_L_PWM_GPIO_Port GPIOB
#define Brake_R_EN_Pin GPIO_PIN_2
#define Brake_R_EN_GPIO_Port GPIOB
#define Accelerator_SW_Pin GPIO_PIN_11
#define Accelerator_SW_GPIO_Port GPIOF
#define Accelerator_IN_Pin GPIO_PIN_12
#define Accelerator_IN_GPIO_Port GPIOF
#define Brake_IN_Pin GPIO_PIN_13
#define Brake_IN_GPIO_Port GPIOF
#define Brake_Pot_Pin GPIO_PIN_14
#define Brake_Pot_GPIO_Port GPIOF
#define Brake_L_EN_Pin GPIO_PIN_15
#define Brake_L_EN_GPIO_Port GPIOF
#define GPS_RX_Pin GPIO_PIN_0
#define GPS_RX_GPIO_Port GPIOG
#define GPS_TX_Pin GPIO_PIN_1
#define GPS_TX_GPIO_Port GPIOG
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
#define RS232_2_TX_Pin GPIO_PIN_14
#define RS232_2_TX_GPIO_Port GPIOB
#define RS232_2_RX_Pin GPIO_PIN_15
#define RS232_2_RX_GPIO_Port GPIOB
#define RS232_1_TX_Pin GPIO_PIN_8
#define RS232_1_TX_GPIO_Port GPIOD
#define RS232_1_RX_Pin GPIO_PIN_9
#define RS232_1_RX_GPIO_Port GPIOD
#define Steer_L_EN2_Pin GPIO_PIN_10
#define Steer_L_EN2_GPIO_Port GPIOD
#define Steer_R_EN2_Pin GPIO_PIN_11
#define Steer_R_EN2_GPIO_Port GPIOD
#define Steer_L_PWM2_Pin GPIO_PIN_12
#define Steer_L_PWM2_GPIO_Port GPIOD
#define Steer_R_PWM2_Pin GPIO_PIN_13
#define Steer_R_PWM2_GPIO_Port GPIOD
#define DEBUG_U_TX_Pin GPIO_PIN_6
#define DEBUG_U_TX_GPIO_Port GPIOC
#define DEBUG_U_RX_Pin GPIO_PIN_7
#define DEBUG_U_RX_GPIO_Port GPIOC
#define CAN_1_RX_Pin GPIO_PIN_0
#define CAN_1_RX_GPIO_Port GPIOD
#define CAN_1_TX_Pin GPIO_PIN_1
#define CAN_1_TX_GPIO_Port GPIOD
#define CAN_2_TX_Pin GPIO_PIN_9
#define CAN_2_TX_GPIO_Port GPIOG
#define CAN_2_RX_Pin GPIO_PIN_10
#define CAN_2_RX_GPIO_Port GPIOG
#define SPI6_CS_Pin GPIO_PIN_11
#define SPI6_CS_GPIO_Port GPIOG
#define CAN_3_RX_Pin GPIO_PIN_5
#define CAN_3_RX_GPIO_Port GPIOB
#define CAN_3_TX_Pin GPIO_PIN_6
#define CAN_3_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
