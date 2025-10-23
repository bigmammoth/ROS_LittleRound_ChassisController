#include "cmsis_os2.h"
#include "rc_receiver.h"
#include "usart.h"
#include "s_bus.h"
#include "usart.h"

#define RECEIVER_NO_SIGNAL_TIMEOUT  100
#define MESSSAGE_QUEUE_SIZE 8

/* ----------------- Static variables -------------------- */
static osThreadId_t threadID;
static osMessageQueueId_t messageQueue;
static ReceiverValues_t receiverValue;

/* ----------------- Static functions -------------------- */
static void RC_Receiver_Process(void* arg);
static void UART_Callback(uint8_t*);

void RC_Receiver_Init(void)
{
    USART_Init();
    threadID = osThreadNew(RC_Receiver_Process, NULL, NULL);
    messageQueue = osMessageQueueNew(MESSSAGE_QUEUE_SIZE, S_BUS_MESSAGE_SIZE, NULL);
    USART_Register_Callback(UART_Callback);
}

static void RC_Receiver_Process(void* arg)
{
    uint8_t msg[S_BUS_MESSAGE_SIZE];
    S_Bus_Channel_t receiverChannel;
    for(;;)
    {
        osStatus_t status = osMessageQueueGet(messageQueue, msg, NULL, RECEIVER_NO_SIGNAL_TIMEOUT);
        if(status != osOK)
        {
            if(status == osErrorTimeout) USART_Init();
            continue;
        }
        if(S_BUS_Parse(msg, &receiverChannel))
        {
            #ifdef RECEIVER_TYPE_WFLY
            Get_WFLY_Receiver_Values(&receiverValue, &receiverChannel);
            #elif defined(RECEIVER_TYPE_HT8A)
            Get_HT8A_Receiver_Values(&receiverValue, &receiverChannel);
            #endif
        }
    }
}

static void UART_Callback(uint8_t* message)
{
    osMessageQueuePut(messageQueue, message, NULL, 0);
}

ReceiverValues_t Receiver_Read(void)
{
    return receiverValue;
}
