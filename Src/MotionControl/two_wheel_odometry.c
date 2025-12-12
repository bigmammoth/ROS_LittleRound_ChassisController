/**
 * @file two_wheel_odometry.c
 * @brief Implementation of Two Wheel Odometry System
 * This module provides functions to initialize, update, retrieve, and reset
 * the odometry of a two-wheel differential drive robot.
 * @date 2025-10-26
 * @author Young.R <com.wang@hotmail.com>
 * @version 1.0
 * @note This module is part of the Motion Control system.
 */
#include "two_wheel_odometry.h"

#include "arm_math.h"
#include <stdlib.h>

#include "data_store.h"

/* ---------------------- Static Variables ------------------------------ */
static float lastPositionL = 0.0f;
static float lastPositionR = 0.0f;
static float x = 0.0f;      // X position in meters
static float y = 0.0f;      // Y position in meters
static float theta = 0.0f;  // Orientation in radians
static float velocity = 0.0f; // Linear velocity in m/s
static float omega = 0.0f; // Angular velocity in rad/s

/**
 * @brief Initialize the Two Wheel Odometry System
 * This function initializes the two-wheel odometry system by setting up necessary parameters.
 */
void TwoWheelOdometry_Init(void)
{
}

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
bool TwoWheelOdometry_GetOdometry(float* pX, float* pY, float* pTheta, float* pVelocity, float* pOmega)
{
    if (pX == NULL || pY == NULL || pTheta == NULL || pVelocity == NULL || pOmega == NULL) return false;
    *pX = x;
    *pY = y;
    *pTheta = theta;
    *pVelocity = velocity;
    *pOmega = omega;
    return true;
}

/** @brief Update the odometry based on wheel position data.
 * This function updates the odometry of the chassis based on the provided wheel position data.
 * @param wheelPosArray Array containing the positions of the left and right wheels in radians.
 *                      wheelPosArray[0] is the left wheel position, wheelPosArray[1] is the right wheel position.
 * @param dt Time interval in seconds since the last update.
 * @return true if odometry update is successful, false otherwise.
 */
bool TwoWheelOdometry_Update(float* wheelPosArray, float dt)
{
    float wheelRadius = DataStore_GetWheelRadius();
    float trackWidth = DataStore_GetTrackWidth();
    
    float positionL = wheelPosArray[0] * wheelRadius;
    float positionR = wheelPosArray[1] * wheelRadius;

    float deltaL = positionL - lastPositionL;
    float deltaR = positionR - lastPositionR;
    lastPositionL = positionL;
    lastPositionR = positionR;

    float dS = (deltaL + deltaR) / 2.0f;
    float dTheta = (deltaR - deltaL) / trackWidth;

    if (dTheta < 1e-6f && dTheta > -1e-6f)
    {
        // Straight movement
        x += dS * arm_cos_f32(theta);
        y += dS * arm_sin_f32(theta);
    }
    else
    {
        // Arc movement
        x += dS * arm_cos_f32(theta + dTheta / 2.0f);
        y += dS * arm_sin_f32(theta + dTheta / 2.0f);
        theta += dTheta;
        while (theta > PI) theta -= 2.0f * PI;
        while (theta < -PI) theta += 2.0f * PI;
    }

    velocity = dS / dt;
    omega = dTheta / dt;

    return true;
}

/**
 * @brief Reset the odometry to the initial state.
 * This function resets the odometry of the chassis to the initial state.
 */
void TwoWheelOdometry_Reset(void)
{
    lastPositionL = 0.0f;
    lastPositionR = 0.0f;
    x = 0.0f;
    y = 0.0f;
    theta = 0.0f;
    velocity = 0.0f;
    omega = 0.0f;
}
