/**
 * @file gyro_node.c
 * @author Joseph Telaak
 * @brief This node reads the MPU6050 gyroscope and accelerometer anb publishes the data.
 * @version 0.1
 * @date 2022-11-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Header include
#include "gyro_node.h"

// Initialize the gyro node
void gyro_node_init(void) {
    // Create the node
    rcl_node_init(&gyro_node, "gyro_node", "", &support);

    // Create the publisher
    rcl_publisher_init(&gyro_ax_publisher, &gyro_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Float32), "gyro_ax");
    rcl_publisher_init(&gyro_ay_publisher, &gyro_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Float32), "gyro_ay");
    rcl_publisher_init(&gyro_az_publisher, &gyro_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Float32), "gyro_az");
    rcl_publisher_init(&gyro_gx_publisher, &gyro_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Float32), "gyro_gx");
    rcl_publisher_init(&gyro_gy_publisher, &gyro_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Float32), "gyro_gy");
    rcl_publisher_init(&gyro_gz_publisher, &gyro_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Float32), "gyro_gz");
    rcl_publisher_init(&gyro_temp_publisher, &gyro_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Float32), "gyro_temp");

    // Create the timer
    rcl_timer_init(&gyro_timer, RCL_MS_TO_NS(GYRO_NODE_UPDATE_FREQ), gyro_timer_callback, &support.context);

    // Initialize the executor
    rclc_executor_init(&gyro_executor, &support.context, 8, &support.allocator);

    // Add the timer to the executor
    rclc_executor_add_timer(&gyro_executor, &gyro_timer);

}

// Timer callback
void gyro_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    // Create the ROS message
    sensor_msgs__msg__Float32 gyro_ax_msg;
    sensor_msgs__msg__Float32 gyro_ay_msg;
    sensor_msgs__msg__Float32 gyro_az_msg;
    sensor_msgs__msg__Float32 gyro_gx_msg;
    sensor_msgs__msg__Float32 gyro_gy_msg;
    sensor_msgs__msg__Float32 gyro_gz_msg;
    sensor_msgs__msg__Float32 gyro_temp_msg;

    // MPU6050 data
    MPU6050_t MPU6050_data;

    // Read the sensor
    MPU6050_Read_All(&hi2c5, &MPU6050_data);

    // Set the message data
    gyro_ax_msg.data = MPU6050_data.Ax;
    gyro_ay_msg.data = MPU6050_data.Ay;
    gyro_az_msg.data = MPU6050_data.Az;
    gyro_gx_msg.data = MPU6050_data.Gx;
    gyro_gy_msg.data = MPU6050_data.Gy;
    gyro_gz_msg.data = MPU6050_data.Gz;
    gyro_temp_msg.data = MPU6050_data.Temp;

    // Publish the message
    rcl_publish(&gyro_ax_publisher, &gyro_ax_msg, NULL);
    rcl_publish(&gyro_ay_publisher, &gyro_ay_msg, NULL);
    rcl_publish(&gyro_az_publisher, &gyro_az_msg, NULL);
    rcl_publish(&gyro_gx_publisher, &gyro_gx_msg, NULL);
    rcl_publish(&gyro_gy_publisher, &gyro_gy_msg, NULL);
    rcl_publish(&gyro_gz_publisher, &gyro_gz_msg, NULL);
    rcl_publish(&gyro_temp_publisher, &gyro_temp_msg, NULL);

}

// Spin the gyro node
void gyro_node_spin(void) {
    rclc_executor_spin(&gyro_executor);
}
