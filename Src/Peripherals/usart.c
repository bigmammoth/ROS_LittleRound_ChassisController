#include "usart.h"
#include "main.h"

/*----------------------------- External variables -------------------------*/
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/* ---------------------------- Static variables ------------------------ */
#define RECEIVE_DATA_SIZE   25
static uint8_t receivedData[RECEIVE_DATA_SIZE];
static USART_Callback_t Uart3Callback;

/* ----------------------------- Static functions ----------------------- */
void UART3_DMA_TransferCompleteCallback(DMA_HandleTypeDef*);

void USART_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receivedData, RECEIVE_DATA_SIZE);
}

void USART_Register_Callback(USART_Callback_t callback)
{
    if (Uart3Callback == NULL) Uart3Callback = callback;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART3)
    {
        if(Size == RECEIVE_DATA_SIZE) Uart3Callback(receivedData);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receivedData, RECEIVE_DATA_SIZE);
    }
}
