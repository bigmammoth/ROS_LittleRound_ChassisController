#pragma once

#include <stdint.h>
#include <stdbool.h>

void MotionControl_Init(void);
void MotionControl_Move(float velocity, float omega);
float MotionControl_GetWheelSpeed(uint32_t motorID);
double MotionControl_GetWheelPosition(uint32_t motorID);
bool MotionControl_IsAutoPilotMode(void);
