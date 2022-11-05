/**
 * @file gyro_node.h
 * @author Joseph Telaak
 * @brief This node reads the MPU6050 gyroscope and accelerometer anb publishes the data.
 * @version 0.1
 * @date 2022-11-05
 * 
 * @copyright Copyright (c) 2022
 * 
 * @note Uses I2C5
 * @note Topic structure: (gyro_a_x, gyro_a_y, gyro_a_z, gyro_g_x, gyro_g_y, gyro_g_z, gyro_temp)
 * 
 */

// Header guard
#ifndef GYRO_NODE_H
#define GYRO_NODE_H

// Sensor thread common
#include "sensor_common.h"

// MPU6050 Driver
#include "mpu6050.h"

// Executor handle
rclc_executor_t gyro_executor;

// Node handle
rcl_node_t gyro_node;

// Timer handle
rcl_timer_t gyro_timer;

// Publisher handle
rcl_publisher_t gyro_ax_publisher;
rcl_publisher_t gyro_ay_publisher;
rcl_publisher_t gyro_az_publisher;
rcl_publisher_t gyro_gx_publisher;
rcl_publisher_t gyro_gy_publisher;
rcl_publisher_t gyro_gz_publisher;
rcl_publisher_t gyro_temp_publisher;

// Initialize the gyro node
void gyro_node_init(void);

// Timer callback
void gyro_timer_callback(rcl_timer_t * timer, int64_t last_call_time);

// Spin the gyro node
void gyro_node_spin(void);

#endif /* GYRO_NODE_H */