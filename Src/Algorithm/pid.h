#pragma once

#include <stdint.h>

typedef struct pid
{
    float kP;
    float kI;
    float kD;
    double object;
    double sumError;
    double lastError;
}PID_t;

void PID_Init(PID_t* instancePID, float kP, float kI, float kD);
float PID_Calc(PID_t* p, float measurement);
void PID_SetObject(PID_t* instancePID, float object);
