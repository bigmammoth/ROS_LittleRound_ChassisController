#include "pid.h"

void PID_Init(PID_t* pid, float kP, float kI, float kD)
{
    pid->kP = kP;
    pid->kI = kI;
    pid->kD = kD;
    pid->lastError = 0;
    pid->sumError = 0;
    pid->object = 0;
}

void PID_SetObject(PID_t* pid, float object)
{
    pid->object = object;
    pid->lastError = 0;
    pid->sumError = 0;
}

float PID_Calc(PID_t* pid, float measurement)
{
    float error = pid->object - measurement;
    float differentialError = error - pid->lastError;
    pid->sumError += error;
    pid->lastError = error;
    return (pid->kP*error + pid->kI*pid->sumError + pid->kD*differentialError);
}
