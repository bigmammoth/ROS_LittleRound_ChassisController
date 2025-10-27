#pragma once

#include <stdbool.h>

/**
 * @brief Initialize the IO service
 * This function registers the callbacks for handling SetIo and ReadIo messages.
 */
bool ROS_ServiceIO_Init(void);
