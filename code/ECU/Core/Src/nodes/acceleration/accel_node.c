/**
 * @file drive_node.c
 * @author Joseph Telaak
 * @brief Controls the vehicle's drive system
 * @version 0.1
 * @date 2022-11-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Header includes
#include "accel_node.h"

// Init function
void accel_node_init(void) {
    // Init node
    rclc_node_init_default(&accel_node, ACCEL_NODE_NAME, "", &support);

    // Message types
    std_msgs__msg__Bool enable_msg;
    std_msgs__msg__Bool direction_msg;
    std_msgs__msg__Float32 accel_wiper_pos_msg;

    // Init executor
    rclc_executor_init(&accel_executor, &support.context, 4, &allocator);

    // Timer
    rclc_timer_init_default(&accel_timer, &support, RCL_MS_TO_NS(ACCEL_UPDATE_FREQ), accel_timer_callback);

    // Create subscriptions
    rclc_subscription_init_default(&enable_sub, &accel_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), ACCEL_ENABLE_TOPIC);
    rclc_subscription_init_default(&direction_sub, &accel_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), ACCEL_DIRECTION_TOPIC);
    rclc_subscription_init_default(&accel_wiper_pos_sub, &accel_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), ACCEL_WIPER_POS_TOPIC);

    // Create publishers
    rclc_publisher_init_default(&accel_pedal_pressed_pub, &accel_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), ACCEL_PEDAL_PRESSED_TOPIC);
    rclc_publisher_init_default(&accel_pedal_pos_pub, &accel_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), ACCEL_PEDAL_POS_TOPIC);

    // Add subscriptions to executor
    rclc_executor_add_subscription(&accel_executor, &enable_sub, &enable_msg, &accel_enable_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&accel_executor, &direction_sub, &direction_msg, &accel_direction_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&accel_executor, &accel_wiper_pos_sub, &accel_wiper_pos_msg, &accel_wiper_pos_callback, ON_NEW_DATA);

    // Add timer to executor
    rclc_executor_add_timer(&accel_executor, &accel_timer);

}

// Subscription callbacks
void accel_enable_callback(const std_msgs__msg__Bool * msg) {
    // Enable/disable the accelerator
    accel_enable(msg->data);

}

void accel_direction_callback(const std_msgs__msg__Bool * msg) {
    // Set the accelerator direction
    accel_direction(msg->data);

}

void accel_wiper_pos_callback(const std_msgs__msg__Float32 * msg) {
    // If message is not 0-255, set the accelerator to 255
    if (msg->data < 0 || msg->data > 255) {
        accel_set(255);

    } else {
        // Set the accelerator to the message
        accel_set(msg->data);

    }
}

// Timer callback
void accel_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    // Get the accelerator pedal position
    float accel_pedal_pos = accel_get();

    // Create messages
    std_msgs__msg__Bool accel_pedal_pressed_msg;
    std_msgs__msg__Float32 accel_pedal_pos_msg;

    // Set messages
    accel_pedal_pressed_msg.data = accel_is_pressed();
    accel_pedal_pos_msg.data = accel_get_pedal_pos();

    // Publish messages
    rcl_publish(&accel_pedal_pressed_pub, &accel_pedal_pressed_msg, NULL);
    rcl_publish(&accel_pedal_pos_pub, &accel_pedal_pos_msg, NULL);

}

// Spin function
void accel_node_spin(void) {
    rclc_executor_spin(&accel_executor);

}

// Enable function
void accel_enable(bool enable) {
    if (enable) {
        HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_SET);

    } else {
        HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);

    }
}

// Direction function
void accel_direction(bool direction) {
    if (direction) {
        HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, GPIO_PIN_SET);

    } else {
        HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, GPIO_PIN_RESET);

    }
}

// Accelerator set function
void accel_set(float pos) {
    // Set the MCP4151 wiper position
    MCP4151_set_wiper_pos(pos);

}

// Accelerator get function
float accel_get(void) {
    // Get the MCP4151 wiper position
    return MCP4151_get_wiper_pos();

}

// Is accelerator actived function
bool accel_is_active(void) {
    // Get the accelerator actived state
    return accel_get() > 0.0;

}

// Is accelerator pressed function (ADC)
bool accel_is_pressed(void) {
    // Read the ADC
    SELECT_ACCEL_PEDAL_SW_CHANNEL();
    HAL_ADC_Start(ACCEL_PEDAL_SW_ADC);
    HAL_ADC_PollForConversion(ACCEL_PEDAL_SW_ADC, 100);
    uint32_t adc_val = HAL_ADC_GetValue(ACCEL_PEDAL_SW_ADC);
    HAL_ADC_Stop(ACCEL_PEDAL_SW_ADC);

    // Return the pressed state
    return adc_val > ACCEL_PEDAL_SW_THRESHOLD;
    
}

// Get accelerator pedal position function (ADC)
float accel_get_pedal_pos(void) {
    // Read the ADC
    SELECT_ACCEL_PEDAL_IN_CHANNEL();
    HAL_ADC_Start(ACCEL_PEDAL_IN_ADC);
    HAL_ADC_PollForConversion(ACCEL_PEDAL_IN_ADC, 100);
    uint32_t adc_val = HAL_ADC_GetValue(ACCEL_PEDAL_IN_ADC);
    HAL_ADC_Stop(ACCEL_PEDAL_IN_ADC);

    // Return the pedal position
    return adc_val;

}
