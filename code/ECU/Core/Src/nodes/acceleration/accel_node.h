/**
 * @file accel_node.h
 * @author Joseph Telaak
 * @brief Controls the vehicle's drive system
 * @version 0.1
 * @date 2022-11-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Header guard
#ifndef ACCEL_NODE_H
#define ACCEL_NODE_H

// Hardware includes
#include "gpio.h"
#include "spi.h"
#include "adc.h"

// Control thread
#include "control_common.h"

// Library includes
#include "mcp4151.h"

// Executor handles
rclc_executor_t accel_executor;

// Node handle
rcl_node_t accel_node;

// Timer
rcl_timer_t accel_timer;

// Node name
#define ACCEL_NODE_NAME "accel_node"

// Subscription handles
rcl_subscription_t enable_sub;
rcl_subscription_t direction_sub;
rcl_subscription_t accel_wiper_pos_sub;

// Subscription topic names
#define ACCEL_ENABLE_TOPIC "accel_enable"
#define ACCEL_DIRECTION_TOPIC "accel_direction"
#define ACCEL_WIPER_POS_TOPIC "accel_wiper_pos"

// Publisher handles
rcl_publisher_t accel_pedal_pressed_pub;
rcl_publisher_t accel_pedal_pos_pub;

// Publisher topic names
#define ACCEL_PEDAL_PRESSED_TOPIC "accel_pedal_pressed"
#define ACCEL_PEDAL_POS_TOPIC "accel_pedal_pos"

// Accelerator Pedal SW ADC
#define ACCEL_PEDAL_SW_ADC &hadc1
#define ACCEL_PEDAL_SW_ADC_CHANNEL 2
#define SELECT_ACCEL_PEDAL_SW_CHANNEL ADC1_CHANNEL_2
#define ACCEL_PEDAL_SW_THRESHOLD 100

// Accelerator Pedal IN ADC
#define ACCEL_PEDAL_IN_ADC &hadc1
#define ACCEL_PEDAL_IN_ADC_CHANNEL 6
#define SELECT_ACCEL_PEDAL_IN_CHANNEL ADC1_CHANNEL_6

// Update rate
#define ACCEL_UPDATE_FREQ 100

// Node initialization
void accel_node_init();

// Subscription callbacks
void accel_enable_callback(const std_msgs__msg__Bool * msg);
void accel_direction_callback(const std_msgs__msg__Bool * msg);
void accel_wiper_pos_callback(const std_msgs__msg__Float32 * msg);

// Timer callback
void accel_timer_callback(rcl_timer_t * timer, int64_t last_call_time);

// Spin function
void accel_spin();

// Enable function
void accel_enable(bool enable);

// Direction function
void accel_direction(bool direction);

// Accelerator set function
void accel_set(float position);

// Accelerator read function
float accel_get();

// Is accelerator active function
bool accel_is_active();

// Is accelerator pressed function
bool accel_is_pressed();

// Get accelerator pedal position function
float accel_get_pedal_pos();

#endif /* ACCEL_NODE_H */