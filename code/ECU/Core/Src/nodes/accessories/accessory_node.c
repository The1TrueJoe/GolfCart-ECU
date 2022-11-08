/**
 * @file accessory_node.c
 * @author Joseph Telaak
 * @brief Controls the lights and other accessories
 * @version 0.1
 * @date 2022-11-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Header imports
#include "accessory_node.h"

// Node init function
void accessory_node_init(void) {
    // Init the node
    rclc_node_init_default(&accsry_node, ACCESSORY_NODE_NAME, "", &support);

    // Init the executor
    rclc_executor_init(&accsry_executor, &support.context, 7, &allocator); 

    // Message types
    std_msgs__msg__Bool head_light_act_msg;
    std_msgs__msg__Bool tail_light_act_msg;
    std_msgs__msg__Bool turn_light_l_act_msg;
    std_msgs__msg__Bool turn_light_r_act_msg;
    std_msgs__msg__Bool horn_act_msg;
    std_msgs__msg__Bool buzz_act_msg;

    // Init the subscribers
    rclc_subscription_init_default(&head_light_act_sub, &accsry_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), HEAD_LIGHT_ACT_TOPIC);
    rclc_subscription_init_default(&tail_light_act_sub, &accsry_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TAIL_LIGHT_ACT_TOPIC);
    rclc_subscription_init_default(&turn_light_l_act_sub, &accsry_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TURN_LIGHT_L_ACT_TOPIC);
    rclc_subscription_init_default(&turn_light_r_act_sub, &accsry_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TURN_LIGHT_R_ACT_TOPIC);
    rclc_subscription_init_default(&horn_act_sub, &accsry_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), HORN_ACT_TOPIC);
    rclc_subscription_init_default(&buzz_act_sub, &accsry_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), BUZZ_ACT_TOPIC);

    // Add the subscriptions to the executor
    rclc_executor_add_subscription(&accsry_executor, &head_light_act_sub, &head_light_act_msg, &head_light_act_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&accsry_executor, &tail_light_act_sub, &tail_light_act_msg, &tail_light_act_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&accsry_executor, &turn_light_l_act_sub, &turn_light_l_act_msg, &turn_light_l_act_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&accsry_executor, &turn_light_r_act_sub, &turn_light_r_act_msg, &turn_light_r_act_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&accsry_executor, &horn_act_sub, &horn_act_msg, &horn_act_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&accsry_executor, &buzz_act_sub, &buzz_act_msg, &buzz_act_callback, ON_NEW_DATA);

}

// Subscription callbacks
void head_light_act_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    if (msg->data) {
        HAL_GPIO_WritePin(AUX_RELAY_1_GPIO_Port, AUX_RELAY_1_Pin, GPIO_PIN_SET);
        
    } else {
        HAL_GPIO_WritePin(AUX_RELAY_1_GPIO_Port, AUX_RELAY_1_Pin, GPIO_PIN_RESET);

    }
}

void tail_light_act_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    if (msg->data) {
        HAL_GPIO_WritePin(AUX_RELAY_2_GPIO_Port, AUX_RELAY_2_Pin, GPIO_PIN_SET);

    } else {
        HAL_GPIO_WritePin(AUX_RELAY_2_GPIO_Port, AUX_RELAY_2_Pin, GPIO_PIN_RESET);

    }
}

void turn_light_l_act_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    if (msg->data) {
        HAL_GPIO_WritePin(AUX_RELAY_3_GPIO_Port, AUX_RELAY_3_Pin, GPIO_PIN_SET);

    } else {
        HAL_GPIO_WritePin(AUX_RELAY_3_GPIO_Port, AUX_RELAY_3_Pin, GPIO_PIN_RESET);

    }
}

void turn_light_r_act_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    if (msg->data) {
        HAL_GPIO_WritePin(AUX_RELAY_4_GPIO_Port, AUX_RELAY_4_Pin, GPIO_PIN_SET);

    } else {
        HAL_GPIO_WritePin(AUX_RELAY_4_GPIO_Port, AUX_RELAY_4_Pin, GPIO_PIN_RESET);

    }
}

void horn_act_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    if (msg->data) {
        HAL_GPIO_WritePin(AUX_RELAY_5_GPIO_Port, AUX_RELAY_5_Pin, GPIO_PIN_SET);

    } else {
        HAL_GPIO_WritePin(AUX_RELAY_5_GPIO_Port, AUX_RELAY_5_Pin, GPIO_PIN_RESET);

    }
}

void buzz_act_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    if (msg->data) {
        HAL_GPIO_WritePin(AUX_RELAY_6_GPIO_Port, AUX_RELAY_6_Pin, GPIO_PIN_SET);

    } else {
        HAL_GPIO_WritePin(AUX_RELAY_6_GPIO_Port, AUX_RELAY_6_Pin, GPIO_PIN_RESET);

    }
}

// Spin the accessory node
void accsry_node_spin() {
    rclc_executor_spin(&accsry_executor);

}
