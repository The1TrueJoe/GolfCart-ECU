/**
 * @file direction_node.c
 * @author Joseph Telaak
 * @brief This node sets the direction selection relay.
 * @version 0.1
 * @date 2022-11-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Header includes
#include "direction_node.h"

// Initialize the node
void direction_node_init(void) {
    // Create node
    rclc_node_init_default(&direction_node, "direction_node", "", &support);

    // Message types
    std_msgs__msg__Bool direction_msg;

    // Create subscriber
    rclc_subscription_init_default(&direction_sub, &direction_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "direction");

    // Initialize executor
    rclc_executor_init(&direction_executor, &support.context, 1, &support.allocator);

    // Add subscription to executor
    rclc_executor_add_subscription(&direction_executor, &direction_sub, &direction_msg, &direction_callback, ON_NEW_DATA);

}

// Callback function
void direction_callback(const void * msgin) {
    // Cast the message
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

    // Set the direction
    if (msg->data) {
        HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, GPIO_PIN_RESET);
    }
}
