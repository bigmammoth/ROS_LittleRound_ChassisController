#pragma once

#include <stdbool.h>

/** @brief Callback type for reset requests */
typedef void (*ROS_Heartbeat_ResetRequestCallback_t)(void);

/**
 * @brief Initialize the Heartbeat mechanism
 * This function initializes the heartbeat mechanism by setting up a timer to
 * periodically check for heartbeat timeouts and registering a callback for
 * incoming heartbeat messages.
 * @return true if initialization was successful, false otherwise
 */
bool ROS_Heartbeat_Init(void);

/**
 * @brief Register a callback function for reset requests
 * This function allows registration of a callback that will be invoked
 * when a reset request is received via the heartbeat mechanism.
 * @param callback Function pointer to the reset request callback
 * @return true if registration was successful, false otherwise
 */
bool ROS_Heartbeat_RegisterResetRequestCallback(ROS_Heartbeat_ResetRequestCallback_t callback);