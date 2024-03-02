#include <string.h>
#include "cmsis_os2.h"
#include "ros_interface.h"

/* -------------- Definitions ----------------------- */
#define EVENT_FLAG_REPLY_SYSTEM_STATUS		0x00000001
#define EVENT_FLAG_REPLY_MOTOR_INFO			0x00000002

/* -------------- Static functions ------------------ */
static void AnalysisIncomingUdpData(uint8_t *pUdpMsg);
static void ReplyUdpSystemStatus(void);

/* --------------- Static variables ---------------- */
static osMessageQueueId_t appRosInterfaceMsgQueueId;
static osEventFlagsId_t responseFlagsId;
static osThreadId_t osThIdAppRosInt;
static osThreadId_t osThIdAppRosResponse;

void app_ros_interface(void *arg)
{
	appRosInterfaceMsgQueueId = osMessageQueueNew(ROS_INTERFACE_Q_LEN, ROS_INTERFACE_MSG_SIZE, NULL);
	while (true)
	{
		uint8_t msg[ROS_INTERFACE_MSG_SIZE];
		osStatus_t status = osMessageQueueGet(appRosInterfaceMsgQueueId, &msg, NULL, osWaitForever);
		AnalysisIncomingUdpData(msg);
	}
}

void AppRosResponse(void* arg)
{
	while (true)
	{
		uint32_t flags = osEventFlagsWait(responseFlagsId, EVENT_FLAG_REPLY_SYSTEM_STATUS, osFlagsWaitAny, osWaitForever);
		if (flags & EVENT_FLAG_REPLY_SYSTEM_STATUS)
		{
			ReplyUdpSystemStatus();
			ReportUdpMotorInfo();
		}
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
	uint32_t msgType = *(uint32_t*)pUdpMsg;

	switch (msgType)
	{
	case UDP_MSG_TYP_HEARTBEAT:
	osEventFlagsSet(responseFlagsId, EVENT_FLAG_REPLY_SYSTEM_STATUS);
		break;

    case UDP_MSG_TYPE_SET_ROBOT_MOTION:
    {
    }
    break;

	default:
		break;
	}
}

void PutUdpMessageToRosInterface(const uint8_t *msg, uint32_t size)
{
	uint8_t message[ROS_INTERFACE_MSG_SIZE];
	memcpy(message, msg, size);
	osMessageQueuePut(appRosInterfaceMsgQueueId, message, NULL, 0);
}

void ReplyUdpSystemStatus(void)
{
}

void ReportUdpMotorInfo(void)
{
}

void ROS_Interface_Init(void)
{
    osThIdAppRosInt = osThreadNew(app_ros_interface, NULL, NULL);
	osThIdAppRosResponse = osThreadNew(AppRosResponse, NULL, NULL);
    // osThreadSetPriority(osThIdAppRosInt, osPriorityNormal7);
	osThreadSetPriority(osThIdAppRosResponse, osPriorityBelowNormal);
	responseFlagsId = osEventFlagsNew(NULL);
}
