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

#include "rl_net.h" // Keil.MDK-Plus::Network:CORE
#include "main.h"
#include "motion_control.h"
#include "dc_motor.h"
#include "rc_receiver.h"
#include "ros_interface.h"
#include "system_config.h"

typedef struct motionMessage
{
    float velocity;
    float omega;
} MotionMessage_t;

/* ------------------ Definitions --------------------*/
#define MESSAGE_QUEUE_SIZE 16
#define MOTION_CONTROL_TIME_INTERVAL 20 // 20ms

/* --------------- Static variables ---------------- */
static osThreadId_t threadId;
static osMessageQueueId_t messageQueue;
static osTimerId_t periodicTimer;
static bool isAutoPilotMode = true; // Flag to indicate if the robot is in manual mode

/* --------------- Static functions ---------------- */
static void MotionControl_Process(void *);
static void InitSystem(void);
static void ReadReceiver(void *);

/**
 * @brief Initialize the Motion Control System
 * This function initializes the motion control system by creating a message queue,
 * starting a thread for processing motion control, and setting up a periodic timer
 * to read receiver values.
 */
void MotionControl_Init(void)
{
    messageQueue = osMessageQueueNew(MESSAGE_QUEUE_SIZE, sizeof(MotionMessage_t), NULL);
    threadId = osThreadNew(MotionControl_Process, NULL, NULL);
    periodicTimer = osTimerNew(ReadReceiver, osTimerPeriodic, NULL, NULL);
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
    InitSystem();
    MotionMessage_t msg;
    osTimerStart(periodicTimer, MOTION_CONTROL_TIME_INTERVAL);
    while (true)
    {
        osStatus_t status = osMessageQueueGet(messageQueue, &msg, NULL, osWaitForever);
        if (status != osOK)
            continue;
        float rightV = msg.velocity + msg.omega * (WHEELS_DISTANCE / 2);
        float leftV = msg.velocity - msg.omega * (WHEELS_DISTANCE / 2);
        if (rightV > MAX_VELOCITY)
            rightV = MAX_VELOCITY;
        else if (rightV < -MAX_VELOCITY)
            rightV = -MAX_VELOCITY;
        if (leftV > MAX_VELOCITY)
            leftV = MAX_VELOCITY;
        else if (leftV < -MAX_VELOCITY)
            leftV = -MAX_VELOCITY;
        DCMotor_SetAngularSpeed(0, leftV / WHEEL_RADIUS);
        DCMotor_SetAngularSpeed(1, rightV / WHEEL_RADIUS);
    }
}

/**
 * @brief Move the robot with specified velocity and angular velocity
 * This function sends a message to the motion control process to move the robot.
 * @param velocity linear velocity in m/s
 * @param omega angular velocity in rad/s
 */
void MotionControl_Move(float velocity, float omega)
{
    MotionMessage_t msg = {velocity, omega};
    osMessageQueuePut(messageQueue, &msg, 0, 0);
}

/**
 * @brief Read receiver values and send them to the motion control process
 * This function reads the receiver values and converts them into velocity and angular velocity.
 * It then sends these values to the motion control process via a message queue.
 * @param arg pointer to argument (not used)
 */
static void ReadReceiver(void *arg)
{
    ReceiverValues_t receiverValue = Receiver_Read();
    float omega, velocity;
    if (!receiverValue.failSafe && !receiverValue.frameLost)
    {
        // Turn receiver value to angular velocity and velocity.
        omega = -receiverValue.steering * MAX_ANGULAR_VELOCITY;
        velocity = receiverValue.throttle * MAX_VELOCITY;
        // Update manual mode status
        isAutoPilotMode = receiverValue.autoMode;
        if (!isAutoPilotMode)
        {
            // Send to motion control process.
            MotionMessage_t msg = {velocity, omega};
            osMessageQueuePut(messageQueue, &msg, 0, 0);
        }
    }
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
    return DCMotor_GetAngularSpeed(motorID) * WHEEL_RADIUS; // Convert angular speed to linear speed
}

/**
 * @brief Get the position of a wheel
 * This function retrieves the encoder value for a motor and converts it to wheel position.
 * @param motorID The ID of the motor
 * @return The wheel position in meters
 */
double MotionControl_GetWheelPosition(uint32_t motorID)
{
    if (motorID >= TOTAL_MOTOR_NUMBER)
        return 0.0; // Invalid motor ID, return 0.0
    return DCMotor_GetEncoderValue(motorID) * WHEEL_PERIMETER; // Get the encoder value for the motor
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

uint32_t MotionControl_GetMotorRunningStatus(uint32_t motorID)
{
    return 0;
}
