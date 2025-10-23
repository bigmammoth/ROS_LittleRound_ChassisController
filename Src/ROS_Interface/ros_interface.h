#pragma once

#include <stdint.h>
#include <stdbool.h>

/* --------------- Callback types ---------------------- */
typedef void (*ROS_Interface_IncomingCallback_t)(const uint8_t *data, uint32_t size);
typedef void (*ROS_Interface_FeedbackCallback_t)(const void **data, uint32_t *size);

/* ---------------- Functions ---------------------------*/

/** 
 * @brief Initialize the ROS Interface
 * This function initializes the ROS interface by creating a message queue and
 * starting the ROS interface processing thread.
 * It sets up the message queue to handle incoming UDP messages and starts the
 * ROS interface process thread that will handle these messages.
 */
void ROS_Interface_Init(void);

/**
 * @brief Register an incoming callback
 * @param messageType the type of the incoming message
 * @param callback the incoming callback function
 * @return true if the callback was registered successfully, false otherwise
 */
bool ROS_Interface_RegisterIncomingCallback(uint32_t messageType, ROS_Interface_IncomingCallback_t callback);

/**
 * @brief Register a feedback callback
 * @param period the period for the feedback callback
 * @param callback the feedback callback function
 * @return true if the callback was registered successfully, false otherwise
 */
bool ROS_Interface_RegisterFeedbackCallback(uint32_t period, ROS_Interface_FeedbackCallback_t callback);

/**
 * @brief Update the heartbeat status
 * This function updates the status of the upper machine's heartbeat.
 * @param isAlive boolean indicating if the upper machine is alive
 */
void ROS_Interface_UpdateHeartbeatStatus(bool isAlive);

/**
 * @brief Send a message back to the upper machine
 * This function sends a message back to the upper machine via UDP.
 * @param data pointer to the data to be sent
 * @param size size of the data to be sent
 */
void ROS_Interface_SendBackMessage(const uint8_t *data, uint32_t size);
