#pragma once

/**
 * @file two_wheel_differential.h
 * @brief Two Wheel Differential Chassis Control Interface
 * This header file defines the interface for controlling a two-wheel differential chassis.
 * It includes functions for initializing the chassis, setting motion commands,
 * and retrieving the current motion state.
 * @date 2025-10-26
 * @author Young.R <com.wang@hotmail.com>
 * @version 1.0
 * @note This module is part of the Motion Control system.
 * @see two_wheel_differential.c
 */

/**
 * @brief Initialize the Two Wheel Differential Chassis
 * This function initializes the two-wheel differential chassis by setting up necessary parameters.
 */
void ChassisTwoWheelDifferential_Init(void);

/**
 * @brief Set the motion of the chassis.
 * This function sets the desired linear velocity and angular velocity of the chassis.
 * @param velocity Desired linear velocity in m/s.
 * @param omega Desired angular velocity in rad/s.
 * @return None
 */
void ChassisTwoWheelDifferential_SetMotion(float velocity, float omega);

/**
 * @brief Get the current motion of the chassis.
 * This function retrieves the current linear velocity and angular velocity of the chassis.
 * @param velocity Pointer to store the linear velocity in m/s.
 * @param omega Pointer to store the angular velocity in rad/s.
 * @return None
 */
void ChassisTwoWheelDifferential_GetMotion(float* velocity, float* omega);
