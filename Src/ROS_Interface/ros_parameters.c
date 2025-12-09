/**
 * @file ros_parameters.c
 * @brief ROS interface handler for parameter set commands.
 * @details This file contains the handler functions for setting parameters 
 *          in the ROS interface.
 * @author Young.W <com.wang@hotmail.com>
 * @date 2025-09-02
 * @copyright (c) 2025 HintonBot. All rights reserved.
 */

#include "ros_parameters.h"

#include "ros_interface.h"
#include "ros_messages.h"
#include "data_store.h"

#include <stdint.h>
#include <string.h>

/* ----------------------------------- Static Functions ---------------------------------------- */
static void SetParametersCallback(const uint8_t *data, uint32_t size);

/**
 * @brief Initialize the Parameters service
 * This function registers the callback for handling parameter messages.
 */
bool ROS_ServiceParameters_Init(void)
{
    return ROS_Interface_RegisterIncomingCallback(ROS_CMD_PARAMETERS, SetParametersCallback);
}

/**
 * @brief Callback for setting parameters
 * This function processes incoming parameter set messages.
 * @param data pointer to the received data
 * @param size size of the received data
 * @note This function should be fast and non-blocking.
 */
void SetParametersCallback(const uint8_t *data, uint32_t size)
{
    if (data == NULL || size < sizeof(ParametersMessage_t))
        return;

    const ParametersMessage_t *msg = (const ParametersMessage_t *)data;
    if (msg->messageType != ROS_CMD_PARAMETERS)
        return;

    // Process the parameters command
    DataStore_SetStateFeedbackFrequency(msg->stateFeedbackFrequency);
    DataStore_SetOdometryFeedbackFrequency(msg->odometryFeedbackFrequency);
    DataStore_SetWheelRadius(msg->wheelDiameter/2.0f);
    DataStore_SetTrackWidth(msg->trackWidth);
    DataStore_SetMaxVelocity(msg->maxLinearVelocity);
    DataStore_SetMaxOmega(msg->maxAngularVelocity);
    DataStore_SetMaxLinearAcceleration(msg->maxLinearAcceleration);
    DataStore_SetMaxAngularAcceleration(msg->maxAngularAcceleration);

    // Send acknowledgment
    FeedbackParametersMessage_t feedback;
    feedback.messageType = ROS_FEEDBACK_PARAMETERS;
    feedback.messageID = msg->messageID;
    feedback.success = 1; // Indicate success
    ROS_Interface_SendBackMessage((const uint8_t *)&feedback, sizeof(feedback));
    // Save the modified parameters to persistent storage
    DataStore_SaveDataIfModified();
}
