/**
 * @file two_wheel_odometry.h
 * @brief Header file for Two Wheel Odometry System
 * This header defines the interface for odometry calculations
 * for a two-wheel differential drive robot.
 * @date 2025-10-26
 * @author Young.R <com.wang@hotmail.com>
 * @version 1.0
 * @note This module is part of the Motion Control system.
 * @see two_wheel_odometry.c
 */
#pragma once

#include <stdbool.h>

/**
 * @brief Initialize the Two Wheel Odometry System
 * This function initializes the two-wheel odometry system by setting up necessary parameters.
 */
void TwoWheelOdometry_Init(void);

/**
 * @brief Get the current odometry of the chassis.
 * This function retrieves the current odometry of the chassis.
 * @param x Pointer to store the x position in meters.
 * @param y Pointer to store the y position in meters.
 * @param theta Pointer to store the orientation in radians.
 * @param velocity Pointer to store the linear velocity in m/s.
 * @param omega Pointer to store the angular velocity in rad/s.
 * @return true if odometry retrieval is successful, false otherwise.
 */
bool TwoWheelOdometry_GetOdometry(float* x, float* y, float* theta, float* velocity, float* omega);

/** @brief Update the odometry based on wheel position data.
 * This function updates the odometry of the chassis based on the provided wheel position data.
 * @param wheelPosArray Array containing the positions of the left and right wheels in radians.
 *                      wheelPosArray[0] is the left wheel position, wheelPosArray[1] is the right wheel position.
 * @param dt Time interval in seconds since the last update.
 * @return true if odometry update is successful, false otherwise.
 */
bool TwoWheelOdometry_Update(float* wheelPosArray, float dt);

/**
 * @brief Reset the odometry to the initial state.
 * This function resets the odometry of the chassis to the initial state.
 */
void TwoWheelOdometry_Reset(void);
