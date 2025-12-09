/**
 * @file data_store.h
 * @brief Header file for the Data Store module.
 * This module provides thread-safe access to configuration and state data
 * for the robotic chassis controller. It includes functions to initialize
 * the data store, get and set various parameters, and notify other threads
 * of changes.
 * 
 * The data store is protected by a mutex to ensure safe concurrent access,
 * and changes can be signaled using event flags.
 * 
 * @author Youmg.W <com.wang@hotmail.com>
 * @date 2025-08-20
 * @version 1.0
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the Data Store module.
 * This function sets up the data store with default values and initializes
 * the necessary resources for thread-safe access. It must be called before
 * any other data store functions are used.
 */
void DataStore_Init(void);

/**
 * @brief Notify that the data store has been modified.
 * This function sets an event flag to indicate that the data store has been modified.
 * It can be called after any setter function to notify other threads of the change.
 */
void DataStore_SaveDataIfModified(void);

/**
 * @brief Get pulses per revolution of the motor.
 * This function retrieves the motor's pulses per revolution from the data store.
*/
float DataStore_GetMotorParamPulsePerRevolution(void);

/**
 * @brief Get gear ratio of the motor.
 * This function retrieves the motor's gear ratio from the data store.
*/
float DataStore_GetMotorParamGearRatio(void);

/**
 * @brief Get the maximum RPM of the motor.
 * This function retrieves the motor's maximum revolutions per minute from the data store.
 */
float DataStore_GetMotorParamMaxRpm(void);

/**
 * @brief Set pulses per revolution of the motor.
 * This function updates the motor's pulses per revolution in the data store.
 */
void DataStore_SetMotorParamPulsePerRevolution(float pulses);

/**
 * @brief Set gear ratio of the motor.
 * This function updates the motor's gear ratio in the data store.
 */
void DataStore_SetMotorParamGearRatio(float ratio);

/**
 * @brief Set the maximum RPM of the motor.
 * This function updates the motor's maximum revolutions per minute in the data store.
 */
void DataStore_SetMotorParamMaxRpm(float rpm);

/**
 * @brief Get the local UDP IP address.
 * This function retrieves the local UDP IP address from the data store.
 * @return uint32_t The local UDP IP address in IPv4 format.
 */
uint32_t DataStore_GetLocalIpAddress(void);

/**
 * @brief Set the local UDP IP address.
 * This function updates the local UDP IP address in the data store.
 * @param ip The new local UDP IP address in IPv4 format.
 */
void DataStore_SetLocalUdpAddress(uint32_t ip);

/**
 * @brief Get the local UDP port.
 * This function retrieves the local UDP port from the data store.
 * @return uint16_t The local UDP port.
 */
uint16_t DataStore_GetLocalUdpPort(void);

/**
 * @brief Set the local UDP port.
 * This function updates the local UDP port in the data store.
 * @param port The new local UDP port.
 */
void DataStore_SetLocalUdpPort(uint16_t port);

/**
 * @brief Set the number of wheels.
 * This function updates the number of wheels in the data store.
 * @param number The new number of wheels.
 */
float DataStore_GetWheelRadius(void);

/**
 * @brief Set the wheel radius.
 * This function updates the wheel radius in the data store.
 * @param radius The new wheel radius in meters.
 */
void DataStore_SetWheelRadius(float radius);

/**
 * @brief Get the track width.
 * This function retrieves the current track width from the data store.
 * @param None
 * @return float The track width in meters.
 */
float DataStore_GetTrackWidth(void);

/**
 * @brief Set the track width.
 * This function updates the wheel base in the data store.
 * @param base The new track width in meters.
 */
void DataStore_SetTrackWidth(float width);

/**
 * @brief Get the maximum linear velocity (canonical).
 * This function retrieves the stored maximum velocity used across modules.
 */
float DataStore_GetMaxVelocity(void);

/**
 * @brief Set the maximum linear velocity (canonical).
 * This function stores the maximum velocity used across modules.
 */
void DataStore_SetMaxVelocity(float velocity);

/**
 * @brief Get the maximum angular velocity (canonical).
 * This function retrieves the stored maximum angular velocity used across modules.
 */
float DataStore_GetMaxOmega(void);

/**
 * @brief Set the maximum angular velocity (canonical).
 * This function stores the maximum angular velocity used across modules.
 */
void DataStore_SetMaxOmega(float omega);

/**
 * @brief Get the maximum linear acceleration.
 * This function retrieves the current maximum linear acceleration from the data store.
 * @param None
 * @return float The maximum linear acceleration in meters per second squared.
 */
float DataStore_GetMaxLinearAcceleration(void);

/**
 * @brief Set the maximum linear acceleration.
 * This function updates the maximum linear acceleration in the data store.
 * @param acceleration The new maximum linear acceleration in meters per second squared.
 */
void DataStore_SetMaxLinearAcceleration(float acceleration);

/**
 * @brief Get the maximum angular acceleration.
 * This function retrieves the current maximum angular acceleration from the data store.
 * @param None
 * @return float The maximum angular acceleration in radians per second squared.
 */
float DataStore_GetMaxAngularAcceleration(void);

/**
 * @brief Set the maximum angular acceleration.
 * This function updates the maximum angular acceleration in the data store.
 * @param acceleration The new maximum angular acceleration in radians per second squared.
 */
void DataStore_SetMaxAngularAcceleration(float acceleration);

/**
 * @brief Get state feedback frequency.
 * This function retrieves the state feedback frequency from the data store.
 * @return float The state feedback frequency.
 */
float DataStore_GetStateFeedbackFrequency(void);

/**
 * @brief Set state feedback frequency.
 * This function updates the state feedback frequency in the data store.
 * @param frequency The new state feedback frequency.
 */
void DataStore_SetStateFeedbackFrequency(float frequency);

/**
 * @brief Set odometry feedback frequency.
 * This function updates the odometry feedback frequency in the data store.
 * @param frequency The new odometry feedback frequency.
 */
void DataStore_SetOdometryFeedbackFrequency(float frequency);

/**
 * @brief Get odometry feedback frequency.
 * This function retrieves the odometry feedback frequency from the data store.
 * @return float The odometry feedback frequency.
 */
float DataStore_GetOdometryFeedbackFrequency(void);
