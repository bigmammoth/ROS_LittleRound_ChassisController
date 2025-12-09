#pragma once

#include <stdbool.h>

/**
 * @brief Initialize the Chassis State Publisher
 * This function initializes the chassis state publisher by registering the
 * callback function for sending chassis state messages.
 */
bool ROS_PublisherChassisState_Init(void);
