/** @file ros_messages.h
 * @brief Definitions of message types and structures for ROS communication.
 *
 * This header file defines the various message types and their corresponding
 * data structures used for communication between the chassis controller and
 * the ROS (Robot Operating System) environment. It includes command messages
 * for velocity, motion state, light control, IO operations, and parameter
 * settings, as well as feedback messages for chassis state, odometry, and
 * battery status. Additionally, it defines a heartbeat message for connection
 * monitoring.
 *
 * The message structures are designed to be compact and efficient for UDP
 * transmission, with fixed-size fields to avoid alignment issues. Each message
 * includes a message type identifier to facilitate parsing and handling on
 * both ends of the communication link.
 *
 * @note Ensure that any changes to these structures are reflected in both the
 *       chassis controller firmware and the corresponding ROS node to maintain
 *       compatibility.
 *
 * @dependencies None
 * @ingroup ros_interface
 * @see ros_interface, ros_publisher_chassis_state, ros_publisher_odom,
 *      ros_subscriber_cmd_vel, ros_heartbeat, ros_service_motion_state,
 *      ros_service_io, ros_service_light, ros_parameters,
 *      motion_state, chassis_odometry
 * @date 2025-08-25
 *       Modified on 2025-12-09 to add ParametersMessage_t and FeedbackParametersMessage_t
 * @author Young.W <com.wang@hotmail.com>
 * @copyright Young
 * @version 1.0
 */
#pragma once

#include <stdint.h>

/** @brief Enumeration of ROS message types */
typedef enum MessageType : uint32_t
{
    ROS_MESSAGE_UNKNOWN = 0,
    ROS_CMD_VELOCITY = 1001,
    ROS_CMD_MOTION,
    ROS_CMD_LIGHT,
    ROS_CMD_SET_IO,
    ROS_CMD_READ_IO,
    ROS_CMD_PARAMETERS,
    ROS_FEEDBACK_PARAMETERS,
    ROS_FEEDBACK_STATE,
    ROS_FEEDBACK_ODOMETRY,
    ROS_FEEDBACK_BATTERY,
    ROS_HEART_BEAT
} MessageType_t;

/** @brief Enumeration of gear modes */
typedef enum GearMode : uint32_t
{
    GEAR_MODE_NEUTRAL = 0,
    GEAR_MODE_PARKING = 1,
    GEAR_MODE_ESTOP = 2,
    GEAR_MODE_DRIVE = 3,
} GearMode_t;

/** @brief Heartbeat message structure */
typedef struct HeartBeatMessage
{
    MessageType_t messageType;
    uint32_t messageID;
    uint32_t success;

    uint32_t reset; // 1 to request a reset, 0 otherwise
} HeartBeatMessage_t;

/** @brief Motion message structure */
typedef struct MotionMessage
{
    MessageType_t messageType;
    uint32_t messageID;
    uint32_t success;
    
    GearMode_t gearMode;
    uint32_t autoMode;
} MotionMessage_t;

/** @brief Velocity message structure */
typedef struct VelocityMessage_t
{
    MessageType_t messageType;
    uint32_t messageID;
    uint32_t success;
    float velocity;
    float omega;
} VelocityMessage_t;

/** @brief Odometry message structure */
typedef struct OdometryMessage
{
    enum MessageType messageType;

    float posX;
    float posY;
    float theta;
    float velocity;
    float omega;
} OdometryMessage_t;

/** @brief Battery message structure */
typedef struct BatteryMessage
{
    MessageType_t messageType;

    float voltage;
    float current;
    float temperature;
    float capacity;
    float design_capacity;
    float charge_percentage;
    uint32_t batteryIsCharging;
} BatteryMessage_t;

/** @brief Light message structure */
typedef struct LightMessage
{
    MessageType_t messageType;
    uint32_t messageID;
    uint32_t success;

    uint32_t color;
    uint32_t mode;
    uint32_t brightness;
    uint32_t frequency;
} LightMessage_t;

#define MAX_IO_PINS 16

/** @brief Set IO message structure */
typedef struct SetIoMessage
{
    MessageType_t messageType;
    uint32_t messageID;
    uint32_t success;

    uint32_t pinCount;
    uint8_t pins[MAX_IO_PINS];
} SetIoMessage_t;

/** @brief Read IO message structure */
typedef struct ReadIoMessage
{
    MessageType_t messageType;
    uint32_t messageID;
    uint32_t success;

    uint32_t pinCount;
    uint8_t pins[MAX_IO_PINS];
} ReadIoMessage_t;

/** @brief Chassis State message structure */
typedef struct ChassisStateMessage
{
    MessageType_t messageType;

    MotionMessage_t motion;
    ReadIoMessage_t io;
    BatteryMessage_t battery;
    uint32_t error_code;
} ChassisStateMessage_t;

/** @brief Parameters message structure */
typedef struct ParametersMessage
{
    MessageType_t messageType;
    uint32_t messageID;
    uint32_t success;

    uint32_t stateFeedbackFrequency;
    uint32_t odometryFeedbackFrequency;
    uint32_t batteryFeedbackFrequency;
    float wheelDiameter;
    float trackWidth;
    float maxLinearAcceleration;
    float maxAngularAcceleration;
    float maxLinearVelocity;
    float maxAngularVelocity;
} ParametersMessage_t;

/** @brief Feedback Parameters message structure */
typedef struct FeedbackParametersMessage {
    MessageType_t messageType;
    uint32_t messageID;
    uint32_t success;
} FeedbackParametersMessage_t;

/** @brief Unknown message structure for unrecognized messages */
typedef struct UnknownMessage
{
    MessageType_t messageType;
    uint32_t messageID;
    uint32_t success;
} UnknownMessage_t;

/**
 * @brief Maximum size among MotionMessage_t, VelocityMessage_t, LightMessage_t,
 *        SetIoMessage_t, and ReadIoMessage_t.
 */
#define _MAX(a,b) ((a) > (b) ? (a) : (b))
#define ROS_MAX_CMD_MESSAGE_SIZE                                      \
    _MAX(sizeof(MotionMessage_t),                                     \
    _MAX(sizeof(VelocityMessage_t),                                   \
    _MAX(sizeof(LightMessage_t),                                      \
    _MAX(sizeof(ParametersMessage_t),                                 \
    _MAX(sizeof(SetIoMessage_t), sizeof(ReadIoMessage_t))))))
#define ROS_MAX_FEEDBACK_MESSAGE_SIZE                                 \
    _MAX(sizeof(OdometryMessage_t),                                   \
    _MAX(sizeof(BatteryMessage_t),                                    \
    _MAX(sizeof(FeedbackParametersMessage_t),                         \
    _MAX(sizeof(LightMessage_t), sizeof(ChassisStateMessage_t)))))
