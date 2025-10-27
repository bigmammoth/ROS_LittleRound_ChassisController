/**
 * @file two_wheel_kinematic.h
 * @brief Header file for Two Wheel Differential Kinematic Model
 * This header defines the interface for forward and inverse kinematics
 * calculations for a two-wheel differential drive robot.
 * @date 2025-10-26
 * @author Young.R <com.wang@hotmail.com>
 * @version 1.0
 * @note This module is part of the Motion Control system.
 * @see two_wheel_kinematic.c
 */
#pragma once

/**
 * @brief Initialize the Two Wheel Differential Kinematic Model
 * This function initializes the two-wheel differential kinematic model by setting up necessary parameters.
 */
void TwoWheelDifferentialKinematic_Init(void);

/**
 * @brief Inverse Kinematics for Two Wheel Differential Drive
 * This function calculates the required wheel angular velocities to achieve a desired linear velocity and angular velocity.
 * @param[in] velocity Desired linear velocity in meters per second.
 * @param[in] omega Desired angular velocity in radians per second.
 * @param[out] wheelAngularSpeed Pointer to an array to store the calculated wheel angular speeds in rad/s.
 * @return None
 * @note This function assumes a differential drive robot with two wheels.
 */
void TwoWheelDifferentialKinematic_Inverse(float velocity, float omega, float* wheelAngularSpeed);

/**
 * @brief Forward Kinematics for Two Wheel Differential Drive
 * This function calculates the linear velocity and angular velocity of the robot based on the wheel angular velocities.
 * @param[in] wheelVelocityInRadS Pointer to an array [2] containing the left and right wheel angular velocities in rad/s.
 * @param[out] velocity Pointer to float to store the calculated linear velocity in meters per second.
 * @param[out] omega Pointer to float to store the calculated angular velocity in radians per second.
 * @return None
 * @note This function assumes a differential drive robot with two wheels.
 */
void TwoWheelDifferentialKinematic_Forward(float* wheelVelocityInRadS, float* velocity, float* omega);
