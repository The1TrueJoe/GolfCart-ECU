/**
 * @file temp_node.c
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

// Node Includes
#include "temp_node.h"

// Initialize the nodes
void temp_node_init(void) {
    // Create node
    rclc_node_init_default(&temp_node, TEMP_NODE_NAME, "", &support);

    // Create publishers
    rclc_publisher_init_default(&temp_1_pub, &temp_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), TEMP_1_TOPIC);
    rclc_publisher_init_default(&temp_2_pub, &temp_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), TEMP_2_TOPIC);

    // Create timers
    rclc_timer_init_default(&temp_timer, &support, RCL_MS_TO_NS(TEMP_NODE_UPDATE_FREQ), temp_callback);

    // Init executor
    rclc_executor_init(&temp_executor, &support.context, 3, &allocator);

    // Add timers to executor
    rclc_executor_add_timer(&temp_executor, &temp_timer);

}

// Timer Callbacks
void temp_callback(rcl_timer_t * timer, int64_t last_call_time) {
    // Create message
    std_msgs__msg__Float32 msg;

    // Get temperature 1
    msg.data = get_temp_1();

    // Publish temperature 1
    rcl_publish(&temp_1_pub, (const void *) &msg, NULL);

    // Get temperature 2
    msg.data = get_temp_2();

    // Publish temperature 2
    rcl_publish(&temp_2_pub, (const void *) &msg, NULL);

}

// Spin the node
void temp_node_spin(void) {
    rclc_executor_spin(&temp_executor);
}
