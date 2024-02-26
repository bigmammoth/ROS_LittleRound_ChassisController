#pragma once

#include "main.h"

typedef void (*Timer_PeriodCallback_t)(void);
typedef void (*Timer_EncoderOverflowCallback_t)(uint32_t, int32_t);
typedef void (*Timer_InputCaptureCallback_t)(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);

void Timer_RegisterPeriodCallback(Timer_PeriodCallback_t);
void Timer_RegisterEncoderOverflowCallback(Timer_EncoderOverflowCallback_t);
void Timer_RegisterInputCaptureCallback(Timer_InputCaptureCallback_t);
void Timer_TimersForMotorInit(void);
void Timer_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
uint32_t Timer_ReadEncoder(uint32_t encoderID);
