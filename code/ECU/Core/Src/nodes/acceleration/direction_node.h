/**
 * @file direction_node.h
 * @author Joseph Telaak
 * @brief This node sets the direction selection relay.
 * @version 0.1
 * @date 2022-11-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Header Guard
#ifndef DIRECTION_NODE_H
#define DIRECTION_NODE_H

// GPIO
#include "gpio.h"

// Control thread
#include "control_common.h"

// Executor handle
rclc_executor_t direction_executor;

// Node handle
rcl_node_t direction_node;

// Subscription
rcl_subscription_t direction_sub;

// Node init
void direction_node_init(void);

// Subscription callback
void direction_callback(const void * msgin);

// Spin
void direction_spin(void);

#endif /* DIRECTION_NODE_H */