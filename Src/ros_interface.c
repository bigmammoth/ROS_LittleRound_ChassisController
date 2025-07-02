#include <string.h>
#include "udp.h"
#include "cmsis_os2.h"
#include "motion_control.h"
#include "ros_interface.h"
#include "hardware_config.h"

/* -------------- Definitions ----------------------- */
#define ROS_INTERFACE_Q_LEN 16
#define ROS_INTERFACE_MSG_SIZE (sizeof(UdpSetRobotMotion_t) > sizeof(UdpMotorInfo_t) ? sizeof(UdpSetRobotMotion_t) : sizeof(UdpMotorInfo_t))

#define UPPER_MACHINE_TIMEOUT_PERIOD 2000 // ms, if the upper machine does not send heartbeat in this time, the system will report error

/* -------------- Static functions ------------------ */
static void AnalysisIncomingUdpData(uint8_t *pUdpMsg);
static void ROS_Interface_Process(void *);
static void CheckUpperMachineTimeOut(uint32_t period);
static void ReportMotorInfo(void);
static void ReportSystemStatus(void);
static void UDP_Callback(const uint8_t *data, uint32_t size);

/* --------------- Static variables ---------------- */
static osMessageQueueId_t appRosInterfaceMsgQueueId;
static osThreadId_t threadID;
static osThreadAttr_t threadAttr = {
    .priority = osPriorityNormal,
    .stack_size = 1024
};
static osTimerId_t periodicTimerReportMotorInofo; // Timer for reporting motor information
static osTimerId_t periodicTimerReportSystemStatus; // Timer for reporting system status
static uint32_t upperMachineTimeOut; // Upper machine timeout, used to check if the upper machine is alive
static bool isUpperMachineAlive; // Flag to indicate if the upper machine is alive
static int udpSocket = -1; // UDP socket for ros communication

/** 
 * @brief Initialize the ROS Interface
 * This function initializes the ROS interface by creating a message queue and
 * starting the ROS interface processing thread.
 * It sets up the message queue to handle incoming UDP messages and starts the
 * ROS interface process thread that will handle these messages.
 */
void ROS_Interface_Init(void)
{
    appRosInterfaceMsgQueueId = osMessageQueueNew(ROS_INTERFACE_Q_LEN, ROS_INTERFACE_MSG_SIZE, NULL);
    threadID = osThreadNew(ROS_Interface_Process, NULL, &threadAttr);
    udpSocket = UDP_RegisterListener(UDP_ROS_LISTEN_PORT, UDP_Callback); // Register the UDP listener for ROS interface messages
}

/**
 * @brief ROS Interface Process
 * This function is the main process for the ROS interface, it waits for incoming UDP messages
 * and processes them accordingly.
 * @param arg pointer to argument (not used)
 * @return none
 */
void ROS_Interface_Process(void *arg)
{
    osStatus_t status;
    uint8_t msg[ROS_INTERFACE_MSG_SIZE];
    static const uint32_t INFO_REPORT_PERIOD = 50; // ms, report motor info status every 50ms
    static const uint32_t REPORT_SYSTEM_STATUS_PERIOD = 500; // ms, report system status every 500ms
    uint32_t reportCount = 0;

    while (true)
    {
        status = osMessageQueueGet(appRosInterfaceMsgQueueId, &msg, NULL, INFO_REPORT_PERIOD);
        if (status == osErrorTimeout)
        {
            // If timeout, report system status and motor info
            if (isUpperMachineAlive)
            {
                ReportMotorInfo();
                if ((reportCount += INFO_REPORT_PERIOD) >= REPORT_SYSTEM_STATUS_PERIOD) // If the report count reaches the system status report period
                {
                    reportCount = 0;
                    ReportSystemStatus();
                }
            }
            CheckUpperMachineTimeOut(INFO_REPORT_PERIOD); // Check if the upper machine has timed out
            continue;
        }
        else if (status != osOK)
            continue;

        // If a message is received, process it
        // Analyze the incoming UDP message
        AnalysisIncomingUdpData(msg);
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

    // Copy the incoming data to the message queue
    osMessageQueuePut(appRosInterfaceMsgQueueId, (void *)data, 0, 0);
}

/**
 * @brief Check upper machine timeout
 * This function checks if the upper machine has timed out. If it has, it sets the
 * isUpperMachineAlive flag to false.
 * @param period the period to check for timeout
 */
void CheckUpperMachineTimeOut(uint32_t period)
{
    upperMachineTimeOut += period; // Increment the timeout counter by the period
    // Check if the upper machine has timed out
    if (upperMachineTimeOut >= UPPER_MACHINE_TIMEOUT_PERIOD)
    {
        isUpperMachineAlive = false; // Set the flag to indicate the upper machine is not alive
        upperMachineTimeOut = 0; // Reset timeout after handling
    }
}

/**
 * @brief Analysis incoming UDP data
 * Analysis incoming UDP data, do actions as the incoming UDP command.
 * @param pMsg pointer to incoming UDP message
 * @return none
 */
void AnalysisIncomingUdpData(uint8_t *pUdpMsg)
{
    uint32_t msgType = *(uint32_t *)pUdpMsg;

    switch (msgType)
    {
    case UDP_MSG_TYP_HEARTBEAT:
    {
        // Heartbeat message received, reset the upper machine timeout
        isUpperMachineAlive = true; // Set the flag to indicate the upper machine is alive
        upperMachineTimeOut = 0;
    }
    break;

    case UDP_MSG_TYPE_SET_ROBOT_MOTION:
    {
        if (MotionControl_IsManualMode()) break; // If the robot is in manual mode, ignore the command
        UdpSetRobotMotion_t *pSetRobotMotion = (UdpSetRobotMotion_t *)pUdpMsg;
        // Process the robot motion command
        MotionControl_Move(pSetRobotMotion->speed, pSetRobotMotion->omega);
    }
    break;

    default:
        break;
    }
}

/**
 * @brief Report motor information to the ROS interface
 * This function collects the current status of the motors and sends it to the ROS interface.
 * It creates a UdpMotorInfo_t structure, fills it with the current motor data, and puts it into the message queue.
 */
void ReportMotorInfo(void)
{
    UdpMotorInfo_t motorInfo;
    motorInfo.msgTyp = UDP_MSG_TYPE_MOTOR_INFO;

    // Collect motor information
    for (int i = 0; i < MOTOR_TOTAL_NUM; i++)
    {
        motorInfo.runningStatus[i] = 0;
        motorInfo.speed[i] = MotionControl_GetWheelSpeed(i);
        motorInfo.position[i] = MotionControl_GetWheelPosition(i);
    }

    // Send motor information to the ROS interface
    if (isUpperMachineAlive && udpSocket >= 0) UDP_SendData(udpSocket, (uint8_t *)&motorInfo, sizeof(motorInfo)); 
}

/**
 * @brief Report system status to the ROS interface
 * This function collects the current system status and sends it to the ROS interface.
 * It creates a UdpSystemStatus_t structure, fills it with the current system data, and puts it into the message queue.
 */
void ReportSystemStatus(void)
{
    UdpSystemStatus_t systemStatus;
    systemStatus.msgTyp = UDP_MSG_TYPE_SYSTEM_STATUS;

    // Collect system status
    systemStatus.upTime = osKernelGetTickCount();
    systemStatus.errorCode = 0;
    systemStatus.voltage = 0;
    systemStatus.current = 0;
    systemStatus.capacity = 0;
    systemStatus.inCharge = false;
    systemStatus.manualMode = MotionControl_IsManualMode();

    // Send system status to the ROS interface
    if (isUpperMachineAlive && udpSocket >= 0) UDP_SendData(udpSocket, (uint8_t *)&systemStatus, sizeof(systemStatus)); 
}
