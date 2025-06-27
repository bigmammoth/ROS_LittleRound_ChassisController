#pragma once

#include <stdint.h>

void DCMotor_Init();
int64_t DCMotor_ReadEncoder(uint32_t motorId);
void DCMotor_SetAngularSpeed(uint32_t motorId, float angularSpeed);
float DCMotor_GetAngularSpeed(uint32_t motorId);
double DCMotor_GetEncoderValue(uint32_t motorId);