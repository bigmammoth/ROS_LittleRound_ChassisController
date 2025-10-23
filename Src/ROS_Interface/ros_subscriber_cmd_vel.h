#pragma once

#include <stdbool.h>

/**
 * @brief Initialize the CmdVel subscriber
 * This function registers the callback for handling velocity command messages.
 * @return true if initialization was successful, false otherwise
 */
bool ROS_SubscriberCmdVel_Init(void);

/**
 * @brief Read the latest commanded velocities
 * This function provides access to the most recent linear and angular velocity commands.
 * @param vel pointer to store the linear velocity (can be NULL)
 * @param angVel pointer to store the angular velocity (can be NULL)
 */
void ROS_SubscriberCmdVel_ReadVelocity(float* vel, float* angVel);
