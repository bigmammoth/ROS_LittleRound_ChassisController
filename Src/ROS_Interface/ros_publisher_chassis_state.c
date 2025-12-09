/**
 * @file ros_publisher_chassis_state.c
 * @ingroup ros_interface
 * @brief Periodic publisher for chassis state feedback frames.
 *
 * @details Provides a feedback producer that packs a \ref ChassisStateMessage_t
 * into a static buffer and exposes it via a callback used by the ROS UDP
 * interface. The initialization function registers this callback so the
 * ROS interface can transmit the payload at a fixed period.
 *
 * The message contains high-level chassis information such as battery and light
 * status. Serialization is done directly into an internal buffer to avoid
 * dynamic allocation in the real-time context.
 *
 * @note Designed to be called from the ROS interface feedback task context.
 *       Callbacks should remain fast and non-blocking.
 *
 * @dependencies ros_interface.h, ros_messages.h
 * @date 2025-09-02
 * @author Young.W <com.wang@hotmail.com>
 * @copyright (c) 2025 HintonBot. All rights reserved.
 */

#include "ros_publisher_chassis_state.h"

#include "ros_interface.h"
#include "ros_messages.h"
#include "battery.h"
#include "io.h"
#include "motion_control.h"
#include "data_store.h"

#include <string.h>

#define DEFAULT_PUBLISH_INTERVAL_MS 100 // Default publish interval in milliseconds (10 Hz)

static uint8_t sendBuffer[sizeof(ChassisStateMessage_t)] = {0};

/* --------------------- Static Functions ------------------------- */
void PrepareChassisStateMessage(const void **data, uint32_t *size);

/**
 * @brief Initialize the Chassis State Publisher
 * This function initializes the chassis state publisher by registering the
 * callback function for sending chassis state messages.
 */
bool ROS_PublisherChassisState_Init(void)
{
    float frequency = DataStore_GetStateFeedbackFrequency();
    const uint32_t feedbackPeriod = (frequency > 0.0f) ? (uint32_t)(1000.0f / frequency) : DEFAULT_PUBLISH_INTERVAL_MS; // Default to 10 Hz if frequency is zero
    // Initialization code for the chassis state publisher
    return ROS_Interface_RegisterFeedbackCallback(feedbackPeriod, PrepareChassisStateMessage); // Register callback for chassis state messages
}

/**
 * @brief Prepare the Chassis State Message
 * This function prepares the chassis state message by populating the static
 * buffer with current chassis state information.
 * @param data pointer to the data buffer to be sent
 * @param size pointer to the size of the data buffer
 */
void PrepareChassisStateMessage(const void **data, uint32_t *size)
{
    if (data == NULL || size == NULL) return;
    ChassisStateMessage_t *msg = (ChassisStateMessage_t *)sendBuffer;
    msg->messageType = ROS_FEEDBACK_STATE;

    // Fill in motion information
    MotionMessage_t *motion = &msg->motion;
    motion->messageType = ROS_CMD_MOTION;
    motion->gearMode = MotionControl_GetGearMode();
    motion->autoMode = MotionControl_IsAutoPilotMode();
    
    // Fill in IO information
    ReadIoMessage_t *io = &msg->io;
    io->messageType = ROS_CMD_READ_IO;
        
    // Fill in battery information
    BatteryMessage_t *battery = &msg->battery;
    battery->messageType = ROS_FEEDBACK_STATE;

    *data = sendBuffer;
    *size = sizeof(ChassisStateMessage_t);
}
