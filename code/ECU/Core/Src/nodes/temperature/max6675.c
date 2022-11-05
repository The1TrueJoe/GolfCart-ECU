/**
 * @file max6675.c
 * @author Joseph Telaak
 * @brief This is a driver for the MAX6675 thermocouple to digital converter
 * @version 0.1
 * @date 2022-11-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Header includes
#include "max6675.h"

// Read temperature 1
float get_temp_1(void) {
    // Select the chip
    TEMP_1_SELECT();

    // Read the data
    uint8_t data[MAX6675_DATA_SIZE];
    HAL_SPI_Receive(&MAX6675_SPI_HANDLE, data, MAX6675_DATA_SIZE, MAX6675_TIMEOUT);

    // Deselect the chip
    TEMP_1_DESELECT();

    // Convert the data to a temperature
    float temp = (data[0] << 8 | data[1]) >> 3;
    temp *= MAX6675_Conversion_Factor;

    // Return the temperature
    return temp + TEMP_1_CAL_OFFSET;

}

// Read temperature 2
float get_temp_2(void) {
    // Select the chip
    TEMP_2_SELECT();

    // Read the data
    uint8_t data[MAX6675_DATA_SIZE];
    HAL_SPI_Receive(&MAX6675_SPI_HANDLE, data, MAX6675_DATA_SIZE, MAX6675_TIMEOUT);

    // Deselect the chip
    TEMP_2_DESELECT();

    // Convert the data to a temperature
    float temp = (data[0] << 8 | data[1]) >> 3;
    temp *= MAX6675_Conversion_Factor;

    // Return the temperature
    return temp + TEMP_2_CAL_OFFSET;

}
