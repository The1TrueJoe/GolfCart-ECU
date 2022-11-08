/**
 * @file max6675.h
 * @author Joseph Telaak
 * @brief This is a driver for the MAX6675 thermocouple to digital converter
 * @version 0.1
 * @date 2022-11-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Header Protection
#ifndef MAX6675_H
#define MAX6675_H

// C Includes
#include <stdint.h>

// Local Includes
#include "stm32h7xx_hal.h"
#include "spi.h"

// SPI Handle
#define MAX6675_SPI_HANDLE hspi5

// SPI Settings
#define MAX6675_DATA_SIZE 2
#define MAX6675_TIMEOUT 100
#define MAX6675_Conversion_Factor 0.25

// TEMP1 CS Functions
#define TEMP_1_SELECT() HAL_GPIO_WritePin(TEMP_1_CS_GPIO_Port, TEMP_1_CS_Pin, GPIO_PIN_RESET)
#define TEMP_1_DESELECT() HAL_GPIO_WritePin(TEMP_1_CS_GPIO_Port, TEMP_1_CS_Pin, GPIO_PIN_SET)

// TEMP2 CS Functions
#define TEMP_2_SELECT() HAL_GPIO_WritePin(TEMP_2_CS_GPIO_Port, TEMP_2_CS_Pin, GPIO_PIN_RESET)
#define TEMP_2_DESELECT() HAL_GPIO_WritePin(TEMP_2_CS_GPIO_Port, TEMP_2_CS_Pin, GPIO_PIN_SET)

// Temperature Sensor IDs
#define TEMP_1 0
#define TEMP_2 1

// Calibration offset for TEMP1 and TEMP2
#define TEMP_1_CAL_OFFSET 0
#define TEMP_2_CAL_OFFSET 0

// Read temperature 1
float get_temp_1(void);

// Read temperature 2
float get_temp_2(void);

#endif
