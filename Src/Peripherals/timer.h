#pragma once

#include "main.h"

typedef void (*Timer_PeriodCallback_t)(void);
typedef void (*Timer_EncoderOverflowCallbak_t)(uint32_t, int32_t);

void Timer_RegisterPeriodCallback(Timer_PeriodCallback_t);
void Timer_RegisterEncoderOverflowCallback(Timer_EncoderOverflowCallbak_t);
void Timer_TimersForMotorInit(void);
void Timer_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
uint32_t Timer_ReadEncoder(uint32_t encoderID);
