/**
 * @file ros_interface.c
 * @defgroup ros_interface ROS Interface
 * @brief ROS UDP interface layer for STM32 firmware.
 *
 * @details Initializes a UDP listener and spawns two CMSIS-RTOS v2 threads:
 * - IncomingTask: Receives UDP frames, parses message type, and dispatches to
 *   registered incoming callbacks via a message queue.
 * - FeedbackTask: Periodically triggers registered feedback producers and sends
 *   their payloads over UDP.
 * Also tracks upper-machine heartbeat to detect timeouts.
 *
 * @note Concurrency: Uses an osMessageQueue for ingress; callback implementations
 *       should protect shared resources if needed.
 *
 * @dependencies cmsis_os2, UDP driver, ros_messages, ros publishers/subscribers,
 *               system_config
 *
 * @author Young.W <com.wang@hotmail.com>
 * @date 2025-08-01
 */

#include <string.h>
#include "main.h"
#include "udp.h"
#include "cmsis_os2.h"
#include "ros_interface.h"
#include "ros_messages.h"
#include "system_config.h"
#include "ros_publisher_chassis_state.h"
#include "ros_publisher_odom.h"
#include "ros_subscriber_cmd_vel.h"
#include "ros_heartbeat.h"
#include "ros_service_motion_state.h"
#include "ros_service_io.h"
#include "ros_service_light.h"

/* -------------- Definitions ----------------------- */
#define ROS_INTERFACE_Q_LEN 16

/* -------------- Data type definitions ------------- */
#define MAX_INCOMING_CALLBACKS 8
#define MAX_FEEDBACK_CALLBACKS 8
#define CHECK_FEEDBACK_PERIOD  10 // ms, check if some feedbacks should be sent every 10ms

typedef struct {
    uint32_t msgType;
    ROS_Interface_IncomingCallback_t callback;
} ROS_Interface_CallbackEntry_t;

typedef struct {
    uint32_t feedbackPeriod;    // in ms, should be a multiple of INFO_REPORT_PERIOD
    int32_t remainTime;         // in ms, time remaining to send the next feedback
    ROS_Interface_FeedbackCallback_t callback;
} ROS_Interface_FeedbackEntry_t;

typedef struct {
    uint32_t size;
    uint8_t data[ROS_MAX_CMD_MESSAGE_SIZE];
} ROS_Interface_CommandMessage_t;

/* --------------- Static variables ---------------- */
static osMessageQueueId_t appRosInterfaceMsgQueueId;
static osThreadId_t incomingThreadID;
static osThreadId_t feedbackThreadID;

static osThreadAttr_t incomingThreadAttr = {
    .priority = osPriorityNormal,
    .stack_size = 1024
};

static osThreadAttr_t feedbackThreadAttr = {
    .priority = osPriorityNormal,
    .stack_size = 1024
};

static bool isUpperMachineAlive; // Flag to indicate if the upper machine is alive
static ROS_Interface_CallbackEntry_t incomingCallbackEntrys[MAX_INCOMING_CALLBACKS]; // Array of incoming message callbacks
static ROS_Interface_FeedbackEntry_t feedbackCallbackEntrys[MAX_FEEDBACK_CALLBACKS]; // Array of feedback message callbacks
static int rosInterfaceUdpSocket = -1; // UDP socket for ROS interface

/* -------------- Static functions ------------------ */
static void IncomingTask(void *);
static void FeedbackTask(void *);
static void UDP_Callback(const uint8_t *data, uint32_t size);

/** 
 * @brief Initialize the ROS Interface
 * This function initializes the ROS interface by creating a message queue and
 * starting the ROS interface processing thread.
 * It sets up the message queue to handle incoming UDP messages and starts the
 * ROS interface process thread that will handle these messages.
 */
void ROS_Interface_Init(void)
{
    appRosInterfaceMsgQueueId = osMessageQueueNew(ROS_INTERFACE_Q_LEN, sizeof(ROS_Interface_CommandMessage_t), NULL);
    assert_param(appRosInterfaceMsgQueueId != NULL);
    incomingThreadID = osThreadNew(IncomingTask, NULL, &incomingThreadAttr);
    assert_param(incomingThreadID != NULL);
    feedbackThreadID = osThreadNew(FeedbackTask, NULL, &feedbackThreadAttr);
    assert_param(feedbackThreadID != NULL);
    rosInterfaceUdpSocket = UDP_RegisterListener(DEFAULT_LOCAL_UDP_PORT, UDP_Callback); // Register the UDP listener for ROS interface messages
	assert_param(rosInterfaceUdpSocket >= 0);
    bool result = ROS_Heartbeat_Init(); // Initialize the heartbeat monitor
    assert_param(result);
    result = ROS_PublisherChassisState_Init(); // Initialize the chassis state publisher
    assert_param(result);
    result = ROS_PublisherOdom_Init(); // Initialize the odometry publisher
    assert_param(result);
    result = ROS_SubscriberCmdVel_Init(); // Initialize the velocity command subscriber
    assert_param(result);
    result = ROS_ServiceMotionState_Init(); // Initialize the motion state service
    assert_param(result);
    result = ROS_ServiceIO_Init(); // Initialize the IO service
    assert_param(result);
    result = ROS_ServiceLight_Init(); // Initialize the light service
    assert_param(result);
}

/**
 * @brief Incoming Task
 * This function is the main process for the incoming task, it waits for incoming
 * messages and processes them accordingly.
 * @param arg pointer to argument (not used)
 * @return none
 */
void IncomingTask(void *arg)
{
	(void)arg;
    osStatus_t status;
    ROS_Interface_CommandMessage_t msg;

    while (true)
    {
        status = osMessageQueueGet(appRosInterfaceMsgQueueId, &msg, NULL, osWaitForever);
        if (status != osOK) continue;
        uint32_t msgType = *(uint32_t *)msg.data;
        for (int i = 0; i < MAX_INCOMING_CALLBACKS; i++)
        {
            if (incomingCallbackEntrys[i].callback != NULL && incomingCallbackEntrys[i].msgType == msgType)
            {
                incomingCallbackEntrys[i].callback(msg.data, msg.size);
                break;
            }
        }
    }
}

/**
 * @brief Feedback Task
 * This function is the main process for the feedback task, it periodically checks
 * for feedback messages and sends them if necessary.
 * @param arg pointer to argument (not used)
 * @return none
 */
void FeedbackTask(void *arg)
{
    (void)arg;
    while (true)
    {
        osDelay(CHECK_FEEDBACK_PERIOD);
        for (int i = 0; i < MAX_FEEDBACK_CALLBACKS; i++)
        {
            // Decrease the remaining time for each feedback entry
            if (feedbackCallbackEntrys[i].remainTime > 0) feedbackCallbackEntrys[i].remainTime -= CHECK_FEEDBACK_PERIOD;
            if (feedbackCallbackEntrys[i].callback != NULL && feedbackCallbackEntrys[i].remainTime <= 0)
            {
                feedbackCallbackEntrys[i].remainTime = feedbackCallbackEntrys[i].feedbackPeriod; // Reset the remaining time
                // Call the feedback callback function
                const void *data = NULL;
                uint32_t size = 0;
                feedbackCallbackEntrys[i].callback(&data, &size);
                if (data != NULL && size > 0 && rosInterfaceUdpSocket >= 0)
                {
                    // Send the feedback data via UDP
                    ROS_Interface_SendBackMessage((const uint8_t *)data, size);
                }
            }
        }
    }
}

/**
 * @brief Callback function for UDP messages
 * This function is called when a UDP message is received. It puts the received data into the message queue
 * for further processing in the ROS interface process.
 * @param data pointer to the received data
 * @param size size of the received data
 */
void UDP_Callback(const uint8_t *data, uint32_t size)
{
    // Check if the data size is valid
    if (size < sizeof(uint32_t)) return; // Minimum size for a message type
    if (size > ROS_MAX_CMD_MESSAGE_SIZE) return; // Exceeds maximum message size
    ROS_Interface_CommandMessage_t msg;
    // Copy the incoming data to the message queue
    msg.size = size;
    memcpy(msg.data, data, size);
    osMessageQueuePut(appRosInterfaceMsgQueueId, (void *)&msg, 0, 0);
}

/**
 * @brief Register an incoming callback
 * @param messageType the type of the incoming message
 * @param callback the incoming callback function
 * @return true if the callback was registered successfully, false otherwise
 */
bool ROS_Interface_RegisterIncomingCallback(uint32_t messageType, ROS_Interface_IncomingCallback_t callback)
{
    assert_param(callback != NULL);
    for (int i = 0; i < MAX_INCOMING_CALLBACKS; i++)
    {
        if (incomingCallbackEntrys[i].callback == NULL)
        {
            incomingCallbackEntrys[i].msgType = messageType;
            incomingCallbackEntrys[i].callback = callback;
            return true;
        }
    }
    return false;
}

/**
 * @brief Register a feedback callback
 * @param period the period for the feedback callback
 * @param callback the feedback callback function
 * @return true if the callback was registered successfully, false otherwise
 */
bool ROS_Interface_RegisterFeedbackCallback(uint32_t period, ROS_Interface_FeedbackCallback_t callback)
{
    assert_param(callback != NULL);
    for (int i = 0; i < MAX_FEEDBACK_CALLBACKS; i++)
    {
        if (feedbackCallbackEntrys[i].callback == NULL)
        {
            feedbackCallbackEntrys[i].feedbackPeriod = period;
            feedbackCallbackEntrys[i].remainTime = period;
            feedbackCallbackEntrys[i].callback = callback;
            return true;
        }
    }
    return false;
}

/**
 * @brief Update the heartbeat status
 * This function updates the status of the upper machine's heartbeat.
 * @param isAlive boolean indicating if the upper machine is alive
 */
void ROS_Interface_UpdateHeartbeatStatus(bool isAlive)
{
    isUpperMachineAlive = isAlive;
}

/**
 * @brief Send a message back to the upper machine
 * This function sends a message back to the upper machine via UDP.
 * @param data pointer to the data to be sent
 * @param size size of the data to be sent
 */
void ROS_Interface_SendBackMessage(const uint8_t *data, uint32_t size)
{
    if (data != NULL && size > 0 && rosInterfaceUdpSocket >= 0)
    {
        NET_ADDR addr;
        if (!UDP_GetReceivedAddress(rosInterfaceUdpSocket, &addr))
            return; // No valid address to send data
        addr.port = DEFAULT_REMOTE_UDP_PORT; // Use the configured remote UDP port        
        UDP_SendDataTo(rosInterfaceUdpSocket, &addr, data, size);
    }
}
