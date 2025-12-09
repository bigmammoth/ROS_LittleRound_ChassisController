/**
 * @file ros_publisher_odom.c
 * @brief Publishes odometry feedback over the ROS interface.
 * @details
 *  - Registers a periodic feedback callback with ROS_Interface (50 ms).
 *  - Fills an OdometryMessage_t using ChassisOdometry and returns its buffer.
 *  - Used by ROS_Interface to transmit ROS_FEEDBACK_ODOMETRY frames.
 * @author Young <com.wang@hotmail.com>
 * @date 2025-08-25
 *      Modified on 2025-12-9 to use MotionControl_GetOdometry()
 * @version 1.0
 * @ingroup ros_interface
 * @copyright Young
 */
#include "ros_publisher_odom.h"
#include "ros_interface.h"
#include "ros_messages.h"
#include "motion_control.h"

#include <stdlib.h>

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
    // Register callback for odometry messages every 20ms
    const uint32_t PUBLISH_INTERVAL = 20;
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
    if (data == NULL || size == NULL) return;
    OdometryMessage_t *msg = (OdometryMessage_t *)odomBuffer;
    msg->messageType = ROS_FEEDBACK_ODOMETRY;
    if (!MotionControl_GetOdometry(&msg->posX, &msg->posY, &msg->theta, &msg->velocity, &msg->omega))
    {
        *data = NULL;
        *size = 0;
        return;
    }
    *data = odomBuffer;
    *size = sizeof(OdometryMessage_t);
}
