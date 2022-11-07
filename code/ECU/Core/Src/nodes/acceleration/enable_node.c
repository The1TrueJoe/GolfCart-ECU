/**
 * @file enable_node.c
 * @author This node controls the enable relay by subscribing to the enable topic (bool)
 * @brief 
 * @version 0.1
 * @date 2022-11-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Includes
#include "enable_node.h"

// Initialize the node
void enable_node_init(void) {
    // Create node
    rclc_node_init_default(&enable_node, "enable_node", "", &support);

    // Message type
    std_msgs__msg__Bool enable_msg;

    // Create subscriber
    rclc_subscription_init_default(&enable_sub, &enable_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "enable");

    // Init executor
    rclc_executor_init(&enable_executor, &support.context, 1, &allocator);

    // Add subscription to executor
    rclc_executor_add_subscription(&enable_executor, &enable_sub, &enable_msg, &enable_callback, ON_NEW_DATA);

}

// Subscription callback
void enable_callback(const void * msgin) {
    // Cast message
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

    // Set enable relay
    if (msg->data) {
        HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);
    }

}

// Spin the node
void enable_node_spin(void) {
    rclc_executor_spin(&enable_executor);
}
