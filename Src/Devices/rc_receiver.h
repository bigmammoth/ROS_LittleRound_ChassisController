#pragma once

#include <stdbool.h>
#include "system_config.h"

#ifdef RECEIVER_TYPE_WFLY
#include "rc_receiver_wfly.h"
#elif defined(RECEIVER_TYPE_HT8A)
#include "rc_receiver_ht8a.h"
#endif

typedef void (*RC_Receiver_Callback_t)(ReceiverValues_t* receiverValue);

/**
 * @brief Initialize the RC Receiver
 * This function initializes the RC receiver by setting up the USART and
 * starting the processing thread.
 */
void RC_Receiver_Init(void);

/**
 * @brief Read the current receiver values
 * @return The current receiver values
 */
ReceiverValues_t Receiver_Read(void);

/**
 * @brief Register a callback for receiver value updates
 * @param callback The callback function to register
 * @return true if registration is successful, false otherwise
 */
bool RC_Receiver_Register_Callback(RC_Receiver_Callback_t callback);
