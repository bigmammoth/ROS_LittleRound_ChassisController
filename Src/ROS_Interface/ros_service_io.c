/**
 * @file ros_service_io.c
 * @brief ROS interface service handlers for Set IO and Read IO commands.
 * @details
 *  - Registers incoming callbacks for ROS_CMD_SET_IO and ROS_CMD_READ_IO.
 *  - Validates message size and type before processing.
 *  - Intended to dispatch hardware GPIO operations (implementation TBD).
 *  - Keeps callbacks non-blocking to avoid delaying the ROS incoming dispatcher.
 * @author young <com.wang@hotmail.com>
 * @date 2025-08-25
 * @ingroup ros_interface
 * @note Add hardware abstraction integration in SetIoCallback / ReadIoCallback where marked.
 */

#include <string.h>

#include "ros_service_io.h"
#include "ros_interface.h"
#include "ros_messages.h"
#include "io.h"

/* -------------------- Static Functions --------------------- */
static void SetIoCallback(const uint8_t *data, uint32_t size);
static void ReadIoCallback(const uint8_t *data, uint32_t size);

/**
 * @brief Initialize the IO service
 * This function registers the callbacks for handling SetIo and ReadIo messages.
 */
bool ROS_ServiceIO_Init(void)
{
    bool result = ROS_Interface_RegisterIncomingCallback(ROS_CMD_SET_IO, SetIoCallback);
    if (!result) return false;
    result = ROS_Interface_RegisterIncomingCallback(ROS_CMD_READ_IO, ReadIoCallback);
    return result;
}

/**
 * @brief Callback for SetIo messages
 * This function processes incoming SetIo messages.
 * @param data pointer to the received data
 * @param size size of the received data
 * @note This function should be fast and non-blocking.
 */
void SetIoCallback(const uint8_t *data, uint32_t size)
{
    if (data == NULL || size != sizeof(SetIoMessage_t)) return;

    SetIoMessage_t msg;
    memcpy(&msg, data, sizeof(SetIoMessage_t));
    if (msg.messageType != ROS_CMD_SET_IO) return;

    IO_Write((uint16_t)(msg.ioPinNo), (bool)(msg.ioValue));
    msg.success = true;

    ROS_Interface_SendBackMessage((uint8_t *)&msg, sizeof(SetIoMessage_t));
}

/**
 * @brief Callback for ReadIo messages
 * This function processes incoming ReadIo messages.
 * @param data pointer to the received data
 * @param size size of the received data
 * @note This function should be fast and non-blocking.
 */
void ReadIoCallback(const uint8_t *data, uint32_t size)
{
    if (data == NULL || size != sizeof(ReadIoMessage_t)) return;

    ReadIoMessage_t msg;
    memcpy(&msg, data, sizeof(ReadIoMessage_t));
    if (msg.messageType != ROS_CMD_READ_IO) return;

    msg.ioValue = (uint32_t)(IO_Read((uint16_t)(msg.ioPinNo)));
    msg.success = true;

    ROS_Interface_SendBackMessage((uint8_t *)&msg, sizeof(ReadIoMessage_t));
}
