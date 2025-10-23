#pragma once

#include <stdbool.h>

/** 
 * @brief Initialize the chassis odometry system. 
 */
void ChassisOdometry_Init(void);

/** @brief Get the current odometry of the chassis.
 * This function retrieves the current odometry of the chassis.
 * @param x Pointer to store the x position in meters.
 * @param y Pointer to store the y position in meters.
 * @param theta Pointer to store the orientation in radians.
 * @param velocity Pointer to store the linear velocity in meters per second.
 * @param omega Pointer to store the angular velocity in radians per second.
 * @return true if odometry retrieval is successful, false otherwise.
 */
bool ChassisOdometry_GetOdometry(float* x, float* y, float* theta, float* velocity, float* omega);

void ChassisOdometry_Reset(void);
