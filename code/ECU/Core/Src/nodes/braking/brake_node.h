/**
 * @file brake.h
 * @author Joseph Telaak
 * @brief Monitors the brake pedal position, controls the brake lights, and sets the brake pressure.
 * @version 0.1
 * @date 2022-11-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Header Guard
#ifndef BRAKE_H
#define BRAKE_H

// Hardware Includes
#include "gpio.h"
#include "adc.h"
#include "tim.h"

// Includes
#include "control_common.h"

// Node Settings
#define BRAKE_NODE_NAME "brake_node"
#define EMERGEBCY_BRAKE_TOPIC "emergency_brake"
#define BRAKE_ACTUATOR_L_TOPIC "brake_l_pressure"
#define BRAKE_ACTUATOR_R_TOPIC "brake_r_pressure"
#define BRAKE_PEDAL_PRESSED_TOPIC "brake_pedal_pressed"
#define BRAKE_PEDAL_POSITION_TOPIC "brake_pedal_position"
#define BRAKE_ACTUATOR_ACTIVE_TOPIC "brake_actuator_active"
#define BRAKE_ACTUATOR_POSITION_TOPIC "brake_actuator_position"

// Brake Light Relay
#define BRAKE_LIGHT_RELAY_PORT AUX_RELAY_3_GPIO_Port
#define BRAKE_LIGHT_RELAY_PIN AUX_RELAY_3_Pin

// Break Pedal ADC
#define BRAKE_PEDAL_ADC &hadc2
#define BRAKE_PEDAL_ADC_CHANNEL 2
#define SELECT_BRAKE_PEDAL_CHANNEL ADC2_SELECT_CH2
#define BRAKE_PEDAL_THRESHOLD 100

// Brake Actuator ADC
#define BRAKE_ACTUATOR_ADC &hadc2
#define BRAKE_ACTUATOR_ADC_CHANNEL 6
#define SELECT_BRAKE_ACTUATOR_CHANNEL ADC2_SELECT_CH6

// Motor Controller
#define BRAKE_ACTUATOR_MOTOR_CONTROLLER_TIMER &htim3
#define BRAKE_ACTUATOR_MOTOR_CONTROLLER_L TIM_CHANNEL_4
#define BRAKE_ACTUATOR_MOTOR_CONTROLLER_R TIM_CHANNEL_3

// Executor handle
rclc_executor_t brake_executor;

// Node handle
rcl_node_t brake_node;

// Timer handle
rcl_timer_t brake_timer;

// Publisher handle
rcl_publisher_t brake_pedal_pressed_pub;
rcl_publisher_t brake_pedal_position_pub;

rcl_publisher_t brake_actuator_active_pub;
rcl_publisher_t brake_act_position_pub;

// Subscriber handle
rcl_subscription_t brake_actuator_l_sub;
rcl_subscription_t brake_actuator_r_sub;

rcl_subscription_t emergency_brake_sub;

// Initalizes the brake node
void brake_node_init();

// Timer callback
void brake_timer_callback();

// Spin function
void brake_spin();

//------- Brake Pedal -------//

// Is Brake Pressed
bool is_brake_pressed(void);

// Read brake pedal position
float read_brake_pedal_position(void);

// Brake detect led
void brake_detect_led(bool state);

//------- Brake Actuator -------//

// Brake Motor Control (Left PWM, Left Enable, Right PWM, Right Enable)
void brake_motor_control(float left_pwm, bool left_enable, float right_pwm, bool right_enable);

// Brake motor callback
void brake_actuator_l_callback(const void * msgin);

// Brake motor callback
void brake_actuator_r_callback(const void * msgin);

// Read brake actuator position
float read_brake_actuator_position(void);

// Send brake actuator active
void send_brake_actuator_active(bool state);

//------- Emergency Brake --------//

// Adjust depending on wiring
#define BRAKE_INVERTED

// Emergency Brake Callback
void emergency_brake_callback(const void * msgin);

#endif /* BRAKE_H */