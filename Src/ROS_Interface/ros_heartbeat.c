/**
 * @file ros_heartbeat.c
 * @brief Implements ROS heartbeat receive/monitor logic.
 * @details
 *  - Registers an incoming callback for ROS_HEART_BEAT frames.
 *  - Maintains last heartbeat tick; a periodic RTOS timer checks for timeout.
 *  - On timeout (no frame within HEARTBEAT_TIMEOUT_PERIOD) marks interface inactive.
 *  - Echoes received heartbeat back (ack) and updates ROS interface status.
 *  - Uses CMSIS-RTOS2 timer (osTimerNew / osTimerStart) for lightweight periodic checks.
 * @ingroup ros_interface
 * @author Young.W <com.wang@hotmail.com>
 * @date 2025-09-25
 * @note Callbacks must stay non-blocking; avoid long operations inside HeartBeatCallback or timeout handler.
 */

#include "main.h"
#include "ros_heartbeat.h"
#include "ros_interface.h"
#include "ros_messages.h"

#include <string.h>

#define HEARTBEAT_TIMEOUT_PERIOD 200    // ms, if no heartbeat received in this period, consider the connection lost
#define HEARTBEAT_CHECK_PERIOD 10       // ms, check heartbeat status every 10 ms

/* --------------------- Static Variables ------------------------------ */
static uint32_t lastHeartbeatTime;
static osTimerId_t heartbeatTimerId;
static ROS_Heartbeat_ResetRequestCallback_t resetRequestCallback;   // Callback for reset requests

/* --------------------- Static Functions ------------------------------ */
static void HeartBeatCallback(const uint8_t *data, uint32_t size);
static void HeartBeatTimeoutCallback(void *arg);

/**
 * @brief Initialize the Heartbeat mechanism
 * This function initializes the heartbeat mechanism by setting up a timer to
 * periodically check for heartbeat timeouts and registering a callback for
 * incoming heartbeat messages.
 * @return true if initialization was successful, false otherwise
 */
bool ROS_Heartbeat_Init(void)
{
    lastHeartbeatTime = osKernelGetTickCount();
    heartbeatTimerId = osTimerNew(HeartBeatTimeoutCallback, osTimerPeriodic, NULL, NULL);
    assert_param(heartbeatTimerId != NULL);
    osTimerStart(heartbeatTimerId, HEARTBEAT_CHECK_PERIOD);
    return ROS_Interface_RegisterIncomingCallback(ROS_HEART_BEAT, HeartBeatCallback);
}

/**
 * @brief Heartbeat message callback
 * This function is called when a heartbeat message is received. It updates the
 * lastHeartbeatTime and sets the isHeartbeatAlive flag.
 * @param data pointer to the received heartbeat message
 * @param size size of the received heartbeat message
 */
void HeartBeatCallback(const uint8_t *data, uint32_t size)
{
    if (data == NULL || size != sizeof(HeartBeatMessage_t))
        return;

    // const HeartBeatMessage_t *msg = (const HeartBeatMessage_t *)data;
    HeartBeatMessage_t msg;
    memcpy(&msg, data, sizeof(HeartBeatMessage_t));
    if (msg.messageType != ROS_HEART_BEAT) return;
    if (msg.reset && resetRequestCallback != NULL)
    {
        resetRequestCallback();
        msg.reset = 1; // Acknowledge reset request
        msg.success = 1;
        msg.messageID = 0; // Reset message ID
    }
    else 
    {
        msg.reset = 0; // Clear reset flag
        msg.success = 1;
    }

    lastHeartbeatTime = osKernelGetTickCount();
    ROS_Interface_UpdateHeartbeatStatus(true);
    ROS_Interface_SendBackMessage((const uint8_t *)&msg, sizeof(HeartBeatMessage_t));
}

/**
 * @brief Heartbeat timeout callback
 * This function is called periodically to check if the heartbeat has timed out.
 * If the time since the last heartbeat exceeds HEARTBEAT_TIMEOUT_PERIOD, it updates
 * the heartbeat status to false.
 * @param arg pointer to argument (not used)
 */
void HeartBeatTimeoutCallback(void *arg)
{
    (void)arg;
    uint32_t timeInterval = osKernelGetTickCount() - lastHeartbeatTime;
    if (timeInterval >= HEARTBEAT_TIMEOUT_PERIOD)
        ROS_Interface_UpdateHeartbeatStatus(false);
}

/**
 * @brief Register a callback function for reset requests
 * This function allows registration of a callback that will be invoked
 * when a reset request is received via the heartbeat mechanism.
 * @param callback Function pointer to the reset request callback
 * @return true if registration was successful, false otherwise
 */
bool ROS_Heartbeat_RegisterResetRequestCallback(ROS_Heartbeat_ResetRequestCallback_t callback)
{
    if (callback == NULL || resetRequestCallback != NULL) return false;
    resetRequestCallback = callback;
	return true;
}
