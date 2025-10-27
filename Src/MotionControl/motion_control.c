/** 
 * @brief Motion Control Module
 *  This module is responsible for controlling the motion of the robot
 *  It handles the communication with the DC motors, processes messages from the
 *  RC receiver, and interfaces with the ROS system.
 *  It uses a message queue to receive motion commands and a periodic timer to read
 *  receiver values.
 * @file motion_control.c
 * @date 2023-10-01
 * @author Young.R com.wang@hotmail.com
 * @version 1.0
 * @note This module is implemented using CMSIS-OS2.
 */

#include "motion_control.h"

#include "rl_net.h" // Keil.MDK-Plus::Network:CORE

#include "main.h"
#include "dc_motor.h"
#include "rc_receiver.h"
#include "ros_interface.h"
#include "system_config.h"
#include "data_store.h"
#include "two_wheel_odometry.h"
#include "two_wheel_kinematic.h"
#include "two_wheel_differential.h"

/* ------------------ Definitions --------------------*/
#define MESSAGE_QUEUE_SIZE 16
#define UPDATE_ODOMETRY_INTERVAL 20 // 20ms
#define MOTION_CONTROL_INTERVAL 20 // 20ms

/**
 * @brief Motion Control Flags
 * These flags are used to indicate the type of motion control operation.
 */
#define FLAG_MOTION_MOVE        0x0001    // Move command flag
#define FLAG_UPDATE_ODOMETRY    0x0002    // Update odometry flag

/* --------------- Static variables ---------------- */
static osThreadId_t threadId;
static bool isAutoPilotMode = true; // Flag to indicate if the robot is in manual mode

static float maxVelocity, maxOmega; // Maximum velocity and angular velocity
static float wheelRadius;

static float velocity, omega; // Current velocity and angular velocity
static float remoteVelocity, remoteOmega; // Receiver velocity and angular velocity

/* --------------- Static functions ---------------- */
static void ReceiverCallback(ReceiverValues_t* receiverValue);
static void UpdateOdometry(void);
static void MotionControl_Process(void *);
static void UpdateOdometryTimerCallback(void *arg);
static void MotionControlTimerCallback(void *arg);

/**
 * @brief Initialize the Motion Control System
 * This function initializes the motion control system by creating a message queue,
 * starting a thread for processing motion control, and setting up a periodic timer
 * to read receiver values.
 */
void MotionControl_Init(void)
{
    maxVelocity = DataStore_GetMaxVelocity();
    maxOmega = DataStore_GetMaxOmega();
	wheelRadius = DataStore_GetWheelRadius();

    ChassisTwoWheelDifferential_Init();
    TwoWheelDifferentialKinematic_Init();
    TwoWheelOdometry_Init();

    threadId = osThreadNew(MotionControl_Process, NULL, NULL);
    assert_param(threadId != NULL);
    
    osTimerId_t updateOdometryTimer = osTimerNew(UpdateOdometryTimerCallback, osTimerPeriodic, NULL, NULL);
    assert_param(updateOdometryTimer != NULL);
    osTimerStart(updateOdometryTimer, UPDATE_ODOMETRY_INTERVAL);
    osTimerId_t motionControlTimer = osTimerNew(MotionControlTimerCallback, osTimerPeriodic, NULL, NULL);
    assert_param(motionControlTimer != NULL);
    osTimerStart(motionControlTimer, MOTION_CONTROL_INTERVAL);

    bool result = RC_Receiver_Register_Callback(ReceiverCallback);
    assert_param(result);
}

/**
 * @brief Motion Control Process
 * This function is the main process for motion control. It initializes the system,
 * starts a periodic timer to read receiver values, and processes messages from the
 * message queue to control the robot's motion.
 * @param arg pointer to argument (not used)
 */
static void MotionControl_Process(void *arg)
{
    while (true)
    {
        uint32_t flags = osThreadFlagsWait(FLAG_MOTION_MOVE | FLAG_UPDATE_ODOMETRY, osFlagsWaitAny, osWaitForever);
        if (flags & FLAG_UPDATE_ODOMETRY)
        {
            UpdateOdometry();
        }
        if (flags & FLAG_MOTION_MOVE)
        {
            if (!isAutoPilotMode)
            {
                velocity = remoteVelocity;
                omega = remoteOmega;
            }
            ChassisTwoWheelDifferential_SetMotion(velocity, omega);
        }
    }
}

/**
 * @brief Initialize the System
 * This function initializes the motion control system components.
 * It initializes the two-wheel differential chassis and sets initial motion parameters.
 * @return None
 */
void UpdateOdometryTimerCallback(void *arg)
{
    (void)arg;
    osThreadFlagsSet(threadId, FLAG_UPDATE_ODOMETRY);
}

/**
 * @brief Motion Control Timer Callback
 * This function is called periodically by the motion control timer.
 * It sets a flag to indicate that a motion move operation should be performed.
 * @param arg pointer to argument (not used)
 */
void MotionControlTimerCallback(void *arg)
{
    (void)arg;
    osThreadFlagsSet(threadId, FLAG_MOTION_MOVE);
}

/**
 * @brief Move the robot with specified velocity and angular velocity
 * This function sends a message to the motion control process to move the robot.
 * @param velocity linear velocity in m/s
 * @param omega angular velocity in rad/s
 */
void MotionControl_Move(float velocity, float omega)
{
    velocity = velocity;
    omega = omega;
    osThreadFlagsSet(threadId, FLAG_MOTION_MOVE);
}

/**
 * @brief Receiver Callback
 * This function is called when new receiver values are available.
 * It updates the remote velocity and omega based on the receiver input.
 * @param receiverValue pointer to the receiver values structure
 */
void ReceiverCallback(ReceiverValues_t* receiverValue)
{
	if (receiverValue->failSafe || receiverValue->frameLost) return;
    isAutoPilotMode = receiverValue->autoMode;
    remoteVelocity = receiverValue->throttle * maxVelocity;
    remoteOmega = receiverValue->steering * maxOmega;
}

/**
 * @brief Update the odometry of the robot
 * This function reads the wheel positions from the motors and updates the odometry.
 */
void UpdateOdometry(void)
{
    static float wheelPositionArray[TOTAL_MOTOR_NUMBER];
    // Read wheel positions from motors
    for (int i = 0; i < TOTAL_MOTOR_NUMBER; i++)
    {
        wheelPositionArray[i] = DCMotor_GetEncoderValue(i) * (2*PI);
    }
    TwoWheelOdometry_Update(wheelPositionArray, (float)UPDATE_ODOMETRY_INTERVAL / 1000.0f);
}

/**
 * @brief Get the current odometry of the robot
 * This function retrieves the current odometry of the robot.
 * @param pX Pointer to store the x position in meters.
 * @param pY Pointer to store the y position in meters.
 * @param pTheta Pointer to store the orientation in radians.
 * @return true if odometry retrieval is successful, false otherwise.
 */
inline bool MotionControl_GetOdometry(float* pX, float* pY, float* pTheta, float* pVelocity, float* pOmega)
{
    return TwoWheelOdometry_GetOdometry(pX, pY, pTheta, pVelocity, pOmega);
}

/**
 * @brief Get the speed of a wheel
 * This function retrieves the angular speed of a motor and converts it to linear speed.
 * @param motorID The ID of the motor
 * @return The linear speed in m/s
 */
float MotionControl_GetWheelSpeed(uint32_t motorID)
{
    if (motorID >= TOTAL_MOTOR_NUMBER)
        return 0.0f; // Invalid motor ID, return 0.0f
    return DCMotor_GetAngularSpeed(motorID) * wheelRadius; // Convert angular speed to linear speed
}

/**
 * @brief Check if the robot is in manual mode
 * This function checks the current manual mode status of the robot.
 * @return true if in manual mode, false otherwise
 */
bool MotionControl_IsAutoPilotMode(void)
{
    return isAutoPilotMode; // Return the current manual mode status
}

/**
 * @brief Get the running status of a motor
 * This function retrieves the running status of a motor.
 * @param motorID The ID of the motor
 * @return The running status of the motor
 */
uint32_t MotionControl_GetMotorRunningStatus(uint32_t motorID)
{
    return 0;
}
