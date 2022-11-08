/**
 * @file temp_node.h
 * @author Joseph Telaak
 * @brief This node publishes two temperature values
 * @version 0.1
 * @date 2022-11-04
 * 
 * @copyright Copyright (c) 2022
 * 
 * @note The temperature sensor is a MAX6675 thermocouple to digital converter attached to SPI5
 * 
 */

// Header guard
#ifndef TEMP_NODE_H
#define TEMP_NODE_H

// Sensor thread
#include "control_common.h"

// MAX6675 Driver
#include "max6675.h"

// Node Properties
#define TEMP_NODE_NAME "temp_node"
#define TEMP_1_TOPIC "temp_1"
#define TEMP_2_TOPIC "temp_2"

// Update frequency
#define TEMP_NODE_UPDATE_FREQ 1000

// Executor Handle
rclc_executor_t temp_executor;

// Node Handle
rcl_node_t temp_node;

// Timer Handles
rcl_timer_t temp_timer;

// Publishers
rcl_publisher_t temp_1_pub;
rcl_publisher_t temp_2_pub;

// Initialize the node
void temp_node_init(void);

// Timer Callbacks
void temp_callback(rcl_timer_t * timer, int64_t last_call_time);

// Spin the node
void temp_node_spin(void);

#endif /* TEMP_NODE_H */
