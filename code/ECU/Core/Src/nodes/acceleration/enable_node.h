/**
 * @file enable_node.h
 * @author This node controls the enable relay by subscribing to the enable topic (bool)
 * @brief 
 * @version 0.1
 * @date 2022-11-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Header guard
#ifndef ENABLE_NODE_H
#define ENABLE_NODE_H

// GPIO
#include "gpio.h"

// Control thread
#include "control_common.h"

// Executor Handle
rclc_executor_t enable_executor;

// Node handle
rcl_node_t enable_node;

// Subscription
rcl_subscription_t enable_sub;

// Node intialization
void enable_node_init(void);

// Subscription callback
void enable_callback(const void * msgin);

// Spin the node
void enable_node_spin(void);

#endif /* ENABLE_NODE_H */