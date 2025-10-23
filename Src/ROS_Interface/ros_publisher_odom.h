/**
 * @file ros_publisher_odom.h
 * @brief Publishes odometry feedback over the ROS interface.
 * @details
 *  - Registers a periodic feedback callback with ROS_Interface (50 ms).
 *  - Fills an OdometryMessage_t using ChassisOdometry and returns its buffer.
 *  - Used by ROS_Interface to transmit ROS_FEEDBACK_ODOMETRY frames.
 * @author young <com.wang@hotmail.com>
 * @date 2025-08-25
 * @ingroup ros_interface
 */

#pragma once

#include <stdbool.h>

/**
 * @brief Initialize the Odometry publisher
 * This function registers the callback for preparing odometry messages.
 */
bool ROS_PublisherOdom_Init(void);