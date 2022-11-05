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

// Executor handle
rclc_executor_t brake_executor;

// Node handle
rcl_node_t brake_node;

// Timer handle
rcl_timer_t brake_timer;

// Publisher handle
rcl_publisher_t brake_pedal_pressed_pub;
rcl_publisher_t brake_pedal_position_pub;

rcl_publisher_t brake_act_position_pub;

// Subscriber handle
rcl_subscription_t brake_actuator_l_sub;
rcl_subscription_t brake_actuator_r_sub;

rcl_subscription_t brake_light_sub;

rcl_subscription_t emergency_brake_sub;

// Initalizes the brake node
void brake_node_init();

// Timer callback
void brake_timer_callback();

// Spin function
void brake_spin();

//------- Relay Control -------//

// Set Brake Light
void set_brake_light(bool state);

// Brake Light Callback
void brake_light_callback(const void * msg);

//------- Brake Pedal -------//

// Is Brake Pressed
bool is_brake_pressed(void);

// Read brake pedal position
float read_brake_pedal_position(void);

//------- Brake Actuator -------//

// Brake Motor Control (Left PWM, Left Enable, Right PWM, Right Enable)
void brake_motor_control(float left_pwm, bool left_enable, float right_pwm, bool right_enable);

// Brake motor callback
void brake_actuator_l_callback(const void * msgin);

// Brake motor callback
void brake_actuator_r_callback(const void * msgin);

// Read brake actuator position
float read_brake_position(void);

//------- Emergency Brake --------//

// Adjust depending on wiring
#define BRAKE_L_FORWARD
//#define BRAKE_L_REVERSE

// Emergency Brake Callback
void emergency_brake_callback(const void * msgin);

#endif /* BRAKE_H */