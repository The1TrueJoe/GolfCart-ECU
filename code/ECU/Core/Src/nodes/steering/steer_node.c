/**
 * @file steer_node.c
 * @author Joseph Telaak
 * @brief Controls the steering system
 * @version 0.1
 * @date 2022-11-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Header includes
#include "steer_node.h"

// Node init
void steer_node_init(void)
{
    // Init node
    rclc_node_init_default(&steer_node, STEER_NODE_NAME, "", &support);

    // Message types
    std_msgs__msg__Float32 steer_act_l_power_msg;
    std_msgs__msg__Float32 steer_act_r_power_msg;

    // Init publishers
    rclc_publisher_init_default(&steer_wheel_ticks_pub, &steer_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), STEER_WHEEL_TICKS_TOPIC);
    rclc_publisher_init_default(&steer_act_active_pub, &steer_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), STEER_ACT_ACTIVE_TOPIC);
    rclc_publisher_init_default(&steer_act_pos_pub, &steer_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), STEER_ACT_L_POSITION_TOPIC);

    // Init subscribers
    rclc_subscription_init_default(&steer_act_l_power_sub, &steer_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), STEER_ACT_L_POWER_TOPIC);
    rclc_subscription_init_default(&steer_act_r_power_sub, &steer_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), STEER_ACT_R_POWER_TOPIC);

    // Init timer
    rclc_timer_init_default(&steer_timer, &support, RCL_MS_TO_NS(STEER_UPDATE_FREQUENCY), steer_timer_callback);

    // Init executor
    rclc_executor_init(&steer_executor, &support.context, 5, &allocator);

    // Add timer to executor
    rclc_executor_add_timer(&steer_executor, &steer_timer);

    // Add subscriptions to executor
    rclc_executor_add_subscription(&steer_executor, &steer_act_l_power_sub, &steer_act_l_power_msg, &steer_set_act_l_power_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&steer_executor, &steer_act_r_power_sub, &steer_act_r_power_msg, &steer_set_act_r_power_callback, ON_NEW_DATA);

}

// Timer callback
void steer_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    // Get steering wheel ticks
    int32_t steer_wheel_ticks = steer_wheel_get_ticks();

    // Get steering actuator position
    int32_t steer_act_pos = steer_act_get_pos();

    // Get steering actuator active
    bool steer_act_active = steer_act_get_active();

    // Create messages
    std_msgs__msg__Float32 steer_wheel_ticks_msg;
    std_msgs__msg__Bool steer_act_active_msg;
    std_msgs__msg__Float32 steer_act_pos_msg;

    // Set messages
    steer_wheel_ticks_msg.data = steer_wheel_ticks;
    steer_act_active_msg.data = steer_act_active;
    steer_act_pos_msg.data = steer_act_pos;

    // Publish messages
    rcl_publish(&steer_wheel_ticks_pub, (const void*)&steer_wheel_ticks_msg, NULL);
    rcl_publish(&steer_act_active_pub, (const void*)&steer_act_active_msg, NULL);
    rcl_publish(&steer_act_pos_pub, (const void*)&steer_act_pos_msg, NULL);

}

// Spin node
void steer_node_spin(void) {
    rclc_executor_spin(&steer_executor);

}

// Get steering wheel ticks
int32_t steer_wheel_get_ticks(void) {
    return steer_wheel_ticks;

}

// Is the steering actuator active
bool steer_is_actuator_active(void) {
    // Check if the steer actuator is active
    return HAL_GPIO_ReadPin(Steering_L_EN_GPIO_Port, Steering_L_EN_Pin) == GPIO_PIN_SET || 
            HAL_GPIO_ReadPin(Steering_R_EN_GPIO_Port, Steering_R_EN_Pin) == GPIO_PIN_SET;
    
}

// Get steering actuator position
float steer_get_act_pos(void) {
    // Read the actuator position from the correct ADC channel
    SELECT_STEER_WHEEL_CHANNEL();
    HAL_ADC_Start(STEER_WHEEL_ADC);
    HAL_ADC_PollForConversion(STEER_WHEEL_ADC, 100);
    float steer_act_pos = HAL_ADC_GetValue(STEER_WHEEL_ADC);
    HAL_ADC_Stop(STEER_WHEEL_ADC);

    return steer_act_pos;

}

// Brake motor control
void steer_motor_control(float left_pwm, bool left_enable, float right_pwm, bool right_enable) {
    // The steer motor uses the htim3 timer
    // The left motor uses channel 4
    // The right motor uses channel 3

    // Set the left motor
    if (left_enable) {
        // Set the pwm
        __HAL_TIM_SET_COMPARE(BRAKE_ACTUATOR_MOTOR_CONTROLLER_TIMER, BRAKE_ACTUATOR_MOTOR_CONTROLLER_L, left_pwm);
        // Enable the motor
        HAL_GPIO_WritePin(Steering_L_EN_GPIO_Port, Steering_L_EN_Pin, GPIO_PIN_SET);

        #ifdef STEER_INVERTED
            // Send the steer actuator active message
            send_steer_actuator_active(true);
        #else
            // Send the steer actuator active message
            send_steer_actuator_active(false);
        #endif

    } else {
        // Disable the motor
        HAL_GPIO_WritePin(Steering_L_EN_GPIO_Port, Steering_L_EN_Pin, GPIO_PIN_RESET);

    }

    // Set the right motor
    if (right_enable) {
        // Set the pwm
        __HAL_TIM_SET_COMPARE(BRAKE_ACTUATOR_MOTOR_CONTROLLER_TIMER, BRAKE_ACTUATOR_MOTOR_CONTROLLER_R, right_pwm);
        // Enable the motor
        HAL_GPIO_WritePin(Steering_R_EN_GPIO_Port, Steering_R_EN_Pin, GPIO_PIN_SET);

        #ifdef STEER_INVERTED
            // Send the steer actuator active message
            send_steer_actuator_active(false);
        #else
            // Send the steer actuator active message
            send_steer_actuator_active(true);
        #endif

    } else {
        // Disable the motor
        HAL_GPIO_WritePin(Steering_R_EN_GPIO_Port, Steering_R_EN_Pin, GPIO_PIN_RESET);

    }

}

// Brake actuator left callback
void steer_set_act_l_power_callback(const void * msgin) {
    // Get the message
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

    // Check if the value is greater than 0
    if (msg->data > 0) {
        steer_motor_control(msg->data, true, 0, false);
        
    } else {
        steer_motor_control(0, false, 0, false);

    }
}

// Brake actuator right callback
void steer_set_act_r_power_callback(const void * msgin) {
    // Get the message
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

    // Check if the value is greater than 0
    if (msg->data > 0) {
        steer_motor_control(0, false, msg->data, true);
        
    } else {
        steer_motor_control(0, false, 0, false);

    }
}

// Send steer actuator active message
void send_steer_actuator_active_message(bool state) {
    // Set the steer actuator active message
    std_msgs__msg__Bool steer_actuator_active_msg;
    steer_actuator_active_msg.data = state;

    // Publish the steer actuator active message
    rclc_publish(&steer_act_active_pub, (const void*)&steer_actuator_active_msg);

}
