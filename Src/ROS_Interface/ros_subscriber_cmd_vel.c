/**
 * @file ros_subscriber_cmd_vel.c
 * @ingroup ros_interface
 * @brief Subscriber for velocity commands (linear velocity and yaw rate) via ROS UDP.
 *
 * @details Registers a callback for CMD_VELOCITY messages, validates payload size and
 * message type, then updates the latest commanded linear velocity (velocity) and
 * angular velocity (omega). A helper function exposes the most recent values to
 * other modules.
 *
 * @note Callback is intended to run in the ROS interface incoming task context and
 * should remain fast and non-blocking.
 *
 * @dependencies ros_interface.h, ros_messages.h
 * @date 2025-09-02
 * @author Young.W <com.wang@hotmail.com>
 */

#include <stdlib.h>
#include "ros_subscriber_cmd_vel.h"
#include "ros_interface.h"
#include "ros_messages.h"

/* ------------------------- Static Variables --------------------------- */
static float velocity, omega;

/* ------------------------- Static Functions --------------------------- */
static void CmdVelCallback(const uint8_t *data, uint32_t size);

/**
 * @brief Initialize the CmdVel subscriber
 * This function registers the callback for handling velocity command messages.
 * @return true if initialization was successful, false otherwise
 */
bool ROS_SubscriberCmdVel_Init(void)
{
    return ROS_Interface_RegisterIncomingCallback(ROS_CMD_VELOCITY, CmdVelCallback);
}

/**
 * @brief Callback for velocity command messages
 * This function processes incoming velocity command messages.
 * @param data pointer to the received data
 * @param size size of the received data
 * @note This function should be fast and non-blocking.
 */
void CmdVelCallback(const uint8_t *data, uint32_t size)
{
    if (data == NULL || size != sizeof(VelocityMessage_t))
        return;

    const VelocityMessage_t *msg = (const VelocityMessage_t *)data;
    if (msg->messageType != ROS_CMD_VELOCITY)
        return;

    // Process the velocity command
    velocity = msg->velocity;
    omega = msg->omega;
}

/**
 * @brief Read the latest commanded velocities
 * This function provides access to the most recent linear and angular velocity commands.
 * @param vel pointer to store the linear velocity (can be NULL)
 * @param angVel pointer to store the angular velocity (can be NULL)
 */
void ROS_SubscriberCmdVel_ReadVelocity(float* vel, float* angVel)
{
    if (vel == NULL || angVel == NULL) return;
    *vel = velocity;
    *angVel = omega;
}
