/**
 * @file two_wheel_differential.c
 * @brief Implementation of Two Wheel Differential Chassis Control
 * This module provides functions to control a two-wheel differential chassis,
 * including initialization, setting motion commands, and retrieving the current motion state.
 * @date 2025-10-26
 * @author Young.R <com.wang@hotmail.com>
 * @version 1.0
 * @note This module is part of the Motion Control system.
 */

#include "two_wheel_differential.h"
#include "two_wheel_kinematic.h"
#include "data_store.h"
#include "dc_motor.h"

/* ------------------------- Static Variables ------------------------- */
// Total number of wheels for two-wheel differential chassis
#define TOTAL_WHEEL_NUMBER 2
// Maximum velocity and angular velocity
static float maxVelocity, maxOmega;

/**
 * @brief Initialize the Two Wheel Differential Chassis
 * This function initializes the two-wheel differential chassis by setting up necessary parameters.
 */
void ChassisTwoWheelDifferential_Init(void)
{
    maxVelocity = DataStore_GetMaxVelocity();
    maxOmega = DataStore_GetMaxOmega();
}

/**
 * @brief Set the motion of the chassis.
 * This function sets the desired linear velocity and angular velocity of the chassis.
 * @param velocity Desired linear velocity in m/s.
 * @param omega Desired angular velocity in rad/s.
 * @return None
 */
void ChassisTwoWheelDifferential_SetMotion(float velocity, float omega)
{
    float wheelAngularSpeed[TOTAL_WHEEL_NUMBER];
    TwoWheelDifferentialKinematic_Inverse(velocity, omega, wheelAngularSpeed);

    for (uint32_t i = 0; i < TOTAL_WHEEL_NUMBER; i++)
    {
        DCMotor_SetAngularSpeed(i, wheelAngularSpeed[i]);
    }
}

/**
 * @brief Get the current motion of the chassis.
 * This function retrieves the current linear velocity and angular velocity of the chassis.
 * @param velocity Pointer to store the linear velocity in m/s.
 * @param omega Pointer to store the angular velocity in rad/s.
 * @return None
 */
void ChassisTwoWheelDifferential_GetMotion(float* velocity, float* omega)
{
    float wheelVelocityInRadS[TOTAL_WHEEL_NUMBER];
    for (uint32_t i = 0; i < TOTAL_WHEEL_NUMBER; i++)
    {
        wheelVelocityInRadS[i] = DCMotor_GetAngularSpeed(i);
    }
    TwoWheelDifferentialKinematic_Forward(wheelVelocityInRadS, velocity, omega);
}
