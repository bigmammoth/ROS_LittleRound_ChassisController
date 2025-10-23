/**
 * @file chassis_motion.h
 * @brief Chassis motion control interface.
 * @details This file defines the interface for chassis motion control,
 *          including initialization, setting motion commands, and retrieving
 *          current motion states. It supports various chassis types and
 *          allows switching between manual and autonomous modes.
 * 
 * @see chassis_motion.c for implementation details.
 * 
 * @author Young.W <young.w@example.com>
 * @date 2025-09-15
 */
#pragma once

#include "ros_messages.h"

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the chassis motion system based on the chassis type.
 * This function selects the appropriate chassis motion implementation
 * based on the chassis type defined in the data store.
 * @details It initializes the chassis motion system by setting the function pointers
 * to the appropriate implementation based on the chassis type. The actual initialization
 * of the chassis motion system is done by calling the Init function pointer.
 */
void ChassisMotion_Init(void);

/**
 * @brief Set the Motion of the chassis motion system.
 * 
 * This function sets the velocity and omega of the chassis motion system by calling the
 * appropriate implementation's SetMotion function.
 * @param velocity The desired linear velocity to set, in meters per second.
 * @param omega The desired angular velocity to set, in radians per second.
 * @return None
 */
void ChassisMotion_SetMotion(float velocity, float omega);

/**
 * @brief Get the Motion of the chassis motion system.
 * 
 * This function retrieves the current Motion of the chassis motion system by calling the
 * appropriate implementation's GetMotion function.
 * @param velocity Pointer to store the current linear velocity, in meters per second.
 * @param omega Pointer to store the current angular velocity, in radians per second.
 * @return None
 */
void ChassisMotion_GetMotion(float* velocity, float* omega);

/**
 * @brief Park the chassis.
 * 
 * This function parks the chassis by setting the motion to zero.
 * @param park If true, the chassis is parked; otherwise, it is unparked.
 * @note This is a placeholder implementation and should be replaced with actual parking logic.
 */
void ChassisMotion_Park(bool park);

/**
 * @brief Set the auto mode of the chassis.
 * 
 * This function sets the auto mode of the chassis. When in auto mode, the chassis
 * will follow commands from the ROS interface instead of the remote controller.
 * @param autoMode If true, the chassis is set to auto mode; otherwise, it is set to manual mode.
 */
void ChassisMotion_SetAutoMode(bool autoMode);

/**
 * @brief Check if the chassis is in auto mode.
 * 
 * This function returns whether the chassis is currently in auto mode.
 * @return true if the chassis is in auto mode; false otherwise.
 */
bool ChassisMotion_IsAutoMode(void);

/**
 * @brief Get the current gear mode.
 * 
 * This function retrieves the current gear mode of the chassis by calling
 * the MotionState module.
 * @return The current gear mode.
 */
GearMode_t ChassisMotion_GetGearMode(void);

/**
 * @brief Get the current motion mode.
 * 
 * This function retrieves the current motion mode of the chassis.
 * @return The current motion mode.
 */
MotionMode_t ChassisMotion_GetMotionMode(void);
