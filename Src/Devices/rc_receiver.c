/**
 * @brief RC Receiver Implementation
 * This module handles the RC receiver input via S-Bus protocol.
 * It initializes the USART for S-Bus communication, processes incoming messages,
 * and provides the current receiver values.
 * @file rc_receiver.c
 * @date 2024-10-01
 * @author Young.R <com.wang@hotmail.com>
 * @version 1.0
 */

 #include "cmsis_os2.h"
#include "rc_receiver.h"
#include "usart.h"
#include "s_bus.h"
#include "usart.h"

/* ----------------- Definitions -------------------- */
#define RECEIVER_NO_SIGNAL_TIMEOUT  100
#define MESSAGE_QUEUE_SIZE 8
#define MAX_CALLBACK_NUMBER 8

/* ----------------- Static variables -------------------- */
static osThreadId_t threadID;
static osMessageQueueId_t messageQueue;
static ReceiverValues_t receiverValue;
static RC_Receiver_Callback_t callbackList[MAX_CALLBACK_NUMBER];
static uint32_t callbackCount;

/* ----------------- Static functions -------------------- */
static void RC_Receiver_Process(void* arg);
static void UART_Callback(uint8_t*);

/**
 * @brief Initialize the RC Receiver
 * This function initializes the RC receiver by setting up the USART and
 * starting the processing thread.
 */
void RC_Receiver_Init(void)
{
    USART_Init();
    threadID = osThreadNew(RC_Receiver_Process, NULL, NULL);
    messageQueue = osMessageQueueNew(MESSAGE_QUEUE_SIZE, S_BUS_MESSAGE_SIZE, NULL);
    USART_Register_Callback(UART_Callback);
}

/**
 * @brief RC Receiver Processing Thread
 * This thread processes incoming S-Bus messages from the RC receiver.
 */
static void RC_Receiver_Process(void* arg)
{
    uint8_t msg[S_BUS_MESSAGE_SIZE];
    S_Bus_Channel_t receiverChannel;
    for(;;)
    {
        osStatus_t status = osMessageQueueGet(messageQueue, msg, NULL, RECEIVER_NO_SIGNAL_TIMEOUT);
        if(status != osOK) continue;
        if(S_BUS_Parse(msg, &receiverChannel))
        {
            #ifdef RECEIVER_TYPE_WFLY
            Get_WFLY_Receiver_Values(&receiverValue, &receiverChannel);
            #elif defined(RECEIVER_TYPE_HT8A)
            Get_HT8A_Receiver_Values(&receiverValue, &receiverChannel);
            #endif
            // Call registered callbacks
            for(uint32_t i = 0; i < callbackCount; i++)
            {
                if(callbackList[i] != NULL) callbackList[i](&receiverValue);
            }
        }
    }
}

/**
 * @brief UART Callback Function
 * This function is called when a new S-Bus message is received via UART.
 */
static void UART_Callback(uint8_t* message)
{
    osMessageQueuePut(messageQueue, message, NULL, 0);
}

/**
 * @brief Read the current receiver values
 * @return The current receiver values
 */
ReceiverValues_t Receiver_Read(void)
{
    return receiverValue;
}

/**
 * @brief Register a callback for receiver value updates
 * @param callback The callback function to register
 * @return true if registration is successful, false otherwise
 */
bool RC_Receiver_Register_Callback(RC_Receiver_Callback_t callback)
{
    if(callbackCount >= MAX_CALLBACK_NUMBER) return false;
    callbackList[callbackCount++] = callback;
    return true;
}
