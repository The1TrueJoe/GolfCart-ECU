/**
 * @file accessory_node.h
 * @author Joseph Telaak
 * @brief Controls the lights and other accessories
 * @version 0.1
 * @date 2022-11-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Header guard
#ifndef ACCESSORY_NODE_H
#define ACCESSORY_NODE_H

// Includes
#include "gpio.h"

// Control thread
#include "control_common.h"

// Executor handle
rclc_executor_t accsry_executor;

// Node handle
rcl_node_t accsry_node;

// Subscription handles
rcl_subscription_t head_light_act_sub;
rcl_subscription_t tail_light_act_sub;
rcl_subscription_t turn_light_l_act_sub;
rcl_subscription_t turn_light_r_act_sub;
rcl_subscription_t horn_act_sub;
rcl_subscription_t buzz_act_sub;

// Subsctiption topic names
#define HEAD_LIGHT_ACT_TOPIC "head_light_act"
#define TAIL_LIGHT_ACT_TOPIC "tail_light_act"
#define TURN_LIGHT_L_ACT_TOPIC "turn_light_l_act"
#define TURN_LIGHT_R_ACT_TOPIC "turn_light_r_act"
#define HORN_ACT_TOPIC "horn_act"
#define BUZZ_ACT_TOPIC "buzz_act"

// Node name
#define ACCESSORY_NODE_NAME "accssry_node"

// Node init function
void accsry_node_init(void);

// Subscription callbacks
void head_light_act_callback(const void * msgin);
void tail_light_act_callback(const void * msgin);
void turn_light_l_act_callback(const void * msgin);
void turn_light_r_act_callback(const void * msgin);
void horn_act_callback(const void * msgin);
void buzz_act_callback(const void * msgin);

// Spin function
void accsry_node_spin(void);

#endif /* ACCESSORY_NODE_H */