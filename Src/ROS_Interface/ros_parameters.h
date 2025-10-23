/** 
 * @file ros_parameters.h
 * @brief ROS interface handler for parameter set commands.
 * @details This file contains the handler functions for setting
 *          parameters in the ROS interface. It defines the request and response
 *          message structures for the parameter services.
 */
#pragma once

/**
 * @brief Initialize the Parameters service
 * This function registers the callback for handling parameter messages.
 */
void ROS_ServiceParameters_Init(void);
