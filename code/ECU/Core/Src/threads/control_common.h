/**
 * @file sensors.h
 * @author Joseph Telaak
 * @brief 
 * @version 0.1
 * @date 2022-11-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Header guard
#ifndef SENSORS_H
#define SENSORS_H

// Includes
#include "main.h"
#include "stm32h7xx_hal.h"

// C Includes
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// Micro-ROS Includes
#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"

#include "std_msgs/msg/bool.h"
#include "std_msgs/msg/float32.h"

// Support Handle
rclc_support_t support;

// Allocator
rcl_allocator_t allocator;

#endif /* SENSORS_H */