/**
 * @file ros_publisher_odom.c
 * @brief Publishes odometry feedback over the ROS interface.
 * @details
 *  - Registers a periodic feedback callback with ROS_Interface (50 ms).
 *  - Fills an OdometryMessage_t using ChassisOdometry and returns its buffer.
 *  - Used by ROS_Interface to transmit ROS_FEEDBACK_ODOMETRY frames.
 * @author young <com.wang@hotmail.com>
 * @date 2025-08-25
 * @ingroup ros_interface
 * @copyright (c) 2025 HintonBot. All rights reserved.
 */

#include "ros_publisher_odom.h"
#include "ros_interface.h"
#include "ros_messages.h"
#include "chassis_odometry.h"

/* ---------------- Static Variables -------------------- */
static uint8_t odomBuffer[sizeof(OdometryMessage_t)] = {0};

/* ---------------- Static Functions -------------------- */
void PrepareOdomMessage(const void **data, uint32_t *size);

/**
 * @brief Initialize the Odometry publisher
 * This function registers the callback for preparing odometry messages.
 */
bool ROS_PublisherOdom_Init(void)
{
    // Register callback for odometry messages every 50ms
    const uint32_t PUBLISH_INTERVAL = 50;
    return ROS_Interface_RegisterFeedbackCallback(PUBLISH_INTERVAL, PrepareOdomMessage);
}

/**
 * @brief Prepare Odometry Message
 * This function prepares the odometry message for sending.
 * @param data pointer to the data buffer
 * @param size pointer to the size of the data
 */
void PrepareOdomMessage(const void **data, uint32_t *size)
{
    OdometryMessage_t *msg = (OdometryMessage_t *)odomBuffer;
    msg->messageType = ROS_FEEDBACK_ODOMETRY;
    if (!ChassisOdometry_GetOdometry(&msg->posX, &msg->posY, &msg->theta, &msg->velocity, &msg->omega))
    {
        msg->posX = 0.0f;
        msg->posY = 0.0f;
        msg->theta = 0.0f;
        msg->velocity = 0.0f;
        msg->omega = 0.0f;
    }
    *data = odomBuffer;
    *size = sizeof(OdometryMessage_t);
}
