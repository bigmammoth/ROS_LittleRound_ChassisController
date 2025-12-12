/**
 * @file two_wheel_kinematic.c
 * @brief Implementation of Two Wheel Differential Kinematic Model
 * This module provides functions for forward and inverse kinematics
 * calculations for a two-wheel differential drive robot.
 * @date 2025-10-26
 * @author Young.R <com.wang@hotmail.com>
 * @version 1.0
 * @note This module is part of the Motion Control system.
 */
#include "two_wheel_kinematic.h"
#include "data_store.h"
#include "arm_math.h"
#include <stdlib.h>

void TwoWheelDifferentialKinematic_Init(void)
{
}

/**
 * @brief Forward Kinematics for Two Wheel Differential Drive
 * This function calculates the linear velocity and angular velocity of the robot based on the wheel angular velocities.
 * @param[in] wheelVelocityInRadS Pointer to an array [2] containing the left and right wheel angular velocities in rad/s.
 * @param[out] velocity Pointer to float to store the calculated linear velocity in meters per second.
 * @param[out] omega Pointer to float to store the calculated angular velocity in radians per second.
 * @return None
 * @note This function assumes a differential drive robot with two wheels.
 */
void TwoWheelDifferentialKinematic_Forward(float* wheelVelocityInRadS, float* velocity, float* omega)
{
    
    if (wheelVelocityInRadS == NULL || velocity == NULL || omega == NULL)
    return;
    
    float wheelRadius = DataStore_GetWheelRadius();
    float trackWidth = DataStore_GetTrackWidth();

    // Calculate linear velocities of each wheel
    float linearL = wheelVelocityInRadS[0] * wheelRadius; // Left wheel linear speed
    float linearR = wheelVelocityInRadS[1] * wheelRadius; // Right wheel linear speed

    // Calculate robot linear and angular velocities
    *velocity = (linearL + linearR) / 2.0f;
    *omega = (linearR - linearL) / trackWidth;
}

/**
 * @brief Inverse Kinematics for Two Wheel Differential Drive
 * This function calculates the required wheel angular velocities to achieve a desired linear velocity and angular velocity.
 * @param[in] velocity Desired linear velocity in meters per second.
 * @param[in] omega Desired angular velocity in radians per second.
 * @param[out] wheelAngularSpeed Pointer to an array to store the calculated wheel angular speeds in rad/s.
 * @return None
 * @note This function assumes a differential drive robot with two wheels.
 */
void TwoWheelDifferentialKinematic_Inverse(float velocity, float omega, float* wheelAngularSpeed)
{
    if (wheelAngularSpeed == NULL) return;

    float wheelRadius = DataStore_GetWheelRadius();
    float trackWidth = DataStore_GetTrackWidth();

    // Calculate the left and right wheel linear speeds based on the desired velocity and omega
    float leftWheelSpeed = velocity - (trackWidth / 2.0f) * omega;
    float rightWheelSpeed = velocity + (trackWidth / 2.0f) * omega;

    // Calculate RPM from wheel speed
    wheelAngularSpeed[0] = leftWheelSpeed / wheelRadius;
    wheelAngularSpeed[1] = rightWheelSpeed / wheelRadius;
}
