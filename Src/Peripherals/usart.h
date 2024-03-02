#pragma once

#include <stdint.h>

typedef void (*USART_Callback_t)(uint8_t*);

void USART_Init(void);
void USART_Register_Callback(USART_Callback_t callback);
