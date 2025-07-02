#pragma once

#include "main.h"

typedef void (*Timer_PeriodCallback_t)(void);
typedef void (*Timer_EncoderOverflowCallback_t)(void);
typedef void (*Timer_InputCaptureCallback_t)(int32_t, int32_t, int32_t, int32_t, int32_t);

void Timer_RegisterPeriodCallback(Timer_PeriodCallback_t);
void Timer_RegisterEncoderOverflowCallback(uint32_t, Timer_EncoderOverflowCallback_t);
void Timer_TimersForMotorInit(void);
uint32_t Timer_ReadEncoder(uint32_t encoderID);
void Timer_PWM_SetDuty(uint32_t motorID, float duty);
