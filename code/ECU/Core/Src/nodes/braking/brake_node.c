/**
 * @file brake.c
 * @author Joseph Telaak
 * @brief Monitors the brake pedal position, controls the brake lights, and sets the brake pressure.
 * @version 0.1
 * @date 2022-11-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Includes
#include "brake_node.h"

// Inirialize the brake node
void brake_node_init(void) {
    // Init the node
    rclc_node_init_default(&brake_node, "brake_node", "", &support);

    // Messages
    std_msgs__msg__Bool emergency_msg;

    // Create subscribers
    rclc_subscription_init_default(&emergency_brake_sub, &brake_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "emergency_brake");
    rclc_subscription_init_default(&brake_actuator_l_sub, &brake_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "brake_l_pressure");
    rclc_subscription_init_default(&brake_actuator_r_sub, &brake_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "brake_r_pressure");

    // Create publishers
    rclc_publisher_init_default(&brake_pedal_pressed_pub, &brake_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "brake_pedal_pressed");
    rclc_publisher_init_default(&brake_pedal_position_pub, &brake_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "brake_pedal_position");
    rclc_publisher_init_default(&brake_act_position_pub, &brake_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "brake_act_position");

    // Create timers
    rclc_timer_init_default(&brake_timer, &support, RCL_MS_TO_NS(10), brake_timer_callback);

    // Initialize executor
    rclc_executor_init(&brake_executor, &support.context, 5, &support.allocator);

    // Add subscriptions to executor
    rclc_executor_add_subscription(&brake_executor, &emergency_brake_sub, &emergency_msg, &emergency_brake_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&brake_executor, &brake_actuator_l_sub, &emergency_msg, &brake_actuator_l_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&brake_executor, &brake_actuator_r_sub, &emergency_msg, &brake_actuator_r_callback, ON_NEW_DATA);

    // Add timers to executor
    rclc_executor_add_timer(&brake_executor, &brake_timer);

}

// Brake timer callback
void brake_timer_callback(void) {
    // Create messages
    std_msgs__msg__Bool brake_pedal_pressed_msg;
    std_msgs__msg__Float32 brake_pedal_position_msg;
    std_msgs__msg__Float32 brake_act_position_msg;
    // Get the brake actuator position
    brake_act_position_msg.data = get_brake_actuator_position();

    // Check if the brake pedal is pressed
    if (is_brake_pressed()) {
        brake_pedal_pressed_msg.data = true;
        brake_pedal_position_msg.data = get_brake_pedal_position();

    } else {
        brake_pedal_pressed_msg.data = false;
        brake_pedal_position_msg.data = 0.0;

    }

    // Publish the messages
    rclc_publish(&brake_pedal_pressed_pub, (const void*)&brake_pedal_pressed_msg);
    rclc_publish(&brake_pedal_position_pub, (const void*)&brake_pedal_position_msg);
    rclc_publish(&brake_act_position_pub, (const void*)&brake_act_position_msg);

}

// Spin function
void brake_node_spin(void) {
    rclc_executor_spin(&brake_executor);
}

// Is brake pressed
bool is_brake_pressed(void) {
    // Check if the brake pedal is pressed
    if (read_brake_pedal_position() > BRAKE_PEDAL_THRESHOLD) {
        return true;

    } else {
        return false;

    }
}

// Read brake pedal position
float read_brake_pedal_position(void) {
    // Read the brake pedal position from the correct ADC channel
    SELECT_BRAKE_PEDAL_CHANNEL();
    HAL_ADC_Start(BRAKE_PEDAL_ADC);
    HAL_ADC_PollForConversion(BRAKE_PEDAL_ADC, 100);
    uint32_t brake_pedal_position = HAL_ADC_GetValue(BRAKE_PEDAL_ADC);
    HAL_ADC_Stop(BRAKE_PEDAL_ADC);

    // Return the brake actuator position
    return brake_pedal_position;

}

// Brake motor control
void brake_motor_control(float left_pwm, bool left_enable, float right_pwm, bool right_enable) {
    // The brake motor uses the htim3 timer
    // The left motor uses channel 4
    // The right motor uses channel 3

    // Set the left motor
    if (left_enable) {
        // Set the pwm
        __HAL_TIM_SET_COMPARE(BRAKE_ACTUATOR_MOTOR_CONTROLLER_TIMER, BRAKE_ACTUATOR_MOTOR_CONTROLLER_L, left_pwm);
        // Enable the motor
        HAL_GPIO_WritePin(Brake_L_EN_GPIO_Port, Brake_L_EN_Pin, GPIO_PIN_SET);

    } else {
        // Disable the motor
        HAL_GPIO_WritePin(Brake_L_EN_GPIO_Port, Brake_L_EN_Pin, GPIO_PIN_RESET);

    }

    // Set the right motor
    if (right_enable) {
        // Set the pwm
        __HAL_TIM_SET_COMPARE(BRAKE_ACTUATOR_MOTOR_CONTROLLER_TIMER, BRAKE_ACTUATOR_MOTOR_CONTROLLER_R, right_pwm);
        // Enable the motor
        HAL_GPIO_WritePin(Brake_R_EN_GPIO_Port, Brake_R_EN_Pin, GPIO_PIN_SET);

    } else {
        // Disable the motor
        HAL_GPIO_WritePin(Brake_R_EN_GPIO_Port, Brake_R_EN_Pin, GPIO_PIN_RESET);

    }
}

// Brake actuator left callback
void brake_actuator_l_callback(const void * msgin) {
    // Get the message
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

    // Check if the value is greater than 0
    if (msg->data > 0) {
        brake_motor_control(msg->data, true, 0, false);
        
    } else {
        brake_motor_control(0, false, 0, false);

    }
}

// Brake actuator right callback
void brake_actuator_r_callback(const void * msgin) {
    // Get the message
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

    // Check if the value is greater than 0
    if (msg->data > 0) {
        brake_motor_control(0, false, msg->data, true);
        
    } else {
        brake_motor_control(0, false, 0, false);

    }
}

// Read brake actuator position
float read_brake_actuator_position(void) {
    // Read the brake actuator position from the correct ADC channel
    SELECT_BRAKE_ACTUATOR_CHANNEL();
    HAL_ADC_Start(BRAKE_ACTUATOR_ADC);
    HAL_ADC_PollForConversion(BRAKE_ACTUATOR_ADC, 100);
    uint32_t brake_actuator_position = HAL_ADC_GetValue(BRAKE_ACTUATOR_ADC);
    HAL_ADC_Stop(BRAKE_ACTUATOR_ADC);

    // Return the brake actuator position
    return brake_actuator_position;

}

// Emergency brake callback
void emergency_brake_callback(const void * msgin) {
    // Get the message
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

    // Set the emergency brake
    #ifdef BRAKE_L_FORWARD
        brake_motor_control(255, true, 0, false);
    #else
        brake_motor_control(0, false, 255, true);
    #endif

}
