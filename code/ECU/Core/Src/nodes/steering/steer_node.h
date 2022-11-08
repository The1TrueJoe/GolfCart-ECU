/**
 * @file steer_node.h
 * @author Joseph Telaak
 * @brief Controls the steering system
 * @version 0.1
 * @date 2022-11-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Header guard
#ifndef STEER_NODE_H
#define STEER_NODE_H

// Hardware includes
#include "gpio.h"
#include "adc.h"
#include "tim.h"

// Includes
#include "control_common.h"

// Node namr
#define STEER_NODE_NAME "steer_node"

// Node topics
#define STEER_WHEEL_TICKS_TOPIC "steering_wheel_ticks"
#define STEER_ACT_ACTIVE_TOPIC "steering_act_active"
#define STEER_ACT_L_POWER_TOPIC "steering_act_l_power"
#define STEER_ACT_R_POWER_TOPIC "steering_act_r_power"
#define STEER_ACT_L_POSITION_TOPIC "steering_act_pos"

// Update frequency
#define STEER_UPDATE_FREQUENCY 10

// Steering wheel ADC
#define STEER_WHEEL_ADC &hadc3
#define STEER_WHEEL_ADC_CHANNEL 1
#define SELECT_STEER_WHEEL_CHANNEL ADC3_SELECT_CH1

// Motor Controller
#define STEER_ACTUATOR_MOTOR_CONTROLLER_TIMER &htim3
#define STEER_ACTUATOR_MOTOR_CONTROLLER_L TIM_CHANNEL_4
#define STEER_ACTUATOR_MOTOR_CONTROLLER_R TIM_CHANNEL_3

// Adjust depending on wiring
#define STEER_INVERTED

// Executor handle
rclc_executor_t steer_executor;

// Node handle
rcl_node_t steer_node;

// Timer handle
rcl_timer_t steer_timer;

// Publisher handles
rcl_publisher_t steer_wheel_ticks_pub;
rcl_publisher_t steer_act_active_pub;
rcl_publisher_t steer_act_pos_pub;

// Subscriber handles
rcl_subscription_t steer_act_l_power_sub;
rcl_subscription_t steer_act_r_power_sub;

// Steering wheel encoder tick count
int32_t steer_wheel_ticks = 0;

// Node init
void steer_node_init();

// Timer callback
void steer_timer_callback(rcl_timer_t * timer, int64_t last_call_time);

// Spin
void steer_spin();

// Get steering wheel ticks
int32_t steer_get_wheel_ticks();

// Is the steering actuator active
bool steer_is_actuator_active();

// Get steering actuator position
float steer_get_act_pos();

// Set steering actuator left power callback
void steer_set_act_l_power_callback(const void * msg);

// Set steering actuator right power callback
void steer_set_act_r_power_callback(const void * msg);

// Steering motor controller function
void steer_motor_control(float left_pwm, bool left_enable, float right_pwm, bool right_enable);

// Send steer actuator active
void send_steer_actuator_active(bool state);

#endif /* STEER_NODE_H */