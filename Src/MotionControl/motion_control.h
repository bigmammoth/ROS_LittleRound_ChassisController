/**
 * @file motion_control.h
 * @brief Motion Control System Interface
 * This header file defines the interface for the Motion Control System,
 * including initialization, movement commands, wheel speed retrieval,
 * manual mode checking, and odometry retrieval.
 * @date 2025-10-26
 * @author Young.R <com.wang@hotmail.com>
 * @version 1.0
 * @note This module is part of the Motion Control system.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the Motion Control System
 * This function initializes the motion control system by creating a message queue,
 * starting a thread for processing motion control, and setting up a periodic timer
 * to read receiver values.
 */
void MotionControl_Init(void);

/**
 * @brief Move the robot with specified velocity and angular velocity
 * This function sends a message to the motion control process to move the robot.
 * @param velocity linear velocity in m/s
 * @param omega angular velocity in rad/s
 */
void MotionControl_Move(float velocity, float omega);

/**
 * @brief Get the speed of a wheel
 * This function retrieves the angular speed of a motor and converts it to linear speed.
 * @param motorID The ID of the motor
 * @return The linear speed in m/s
 */
float MotionControl_GetWheelSpeed(uint32_t motorID);

/**
 * @brief Check if the robot is in manual mode
 * This function checks the current manual mode status of the robot.
 * @return true if in manual mode, false otherwise
 */
bool MotionControl_IsAutoPilotMode(void);

/**
 * @brief Get the current odometry of the robot
 * This function retrieves the current odometry of the robot.
 * @param pX Pointer to store the x position in meters.
 * @param pY Pointer to store the y position in meters.
 * @param pTheta Pointer to store the orientation in radians.
 * @return true if odometry retrieval is successful, false otherwise.
 */
bool MotionControl_GetOdometry(float* pX, float* pY, float* pTheta, float* pVelocity, float* pOmega);
