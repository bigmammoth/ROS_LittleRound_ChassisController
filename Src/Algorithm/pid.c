/**
 * @file pid.c
 * @author Young.R com.wang@hotmail.com
 * @brief PID control algorithm implementation
 * @version 0.1
 * @date 2025-07-02
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "pid.h"

#define MAX_SUM_ERROR   1000

/**
 * @brief Initialize the PID controller
 * @param pid Pointer to the PID controller structure
 * @param kP Proportional gain
 * @param kI Integral gain
 * @param kD Derivative gain
 */
void PID_Init(PID_t* pid, float kP, float kI, float kD)
{
    pid->kP = kP;
    pid->kI = kI;
    pid->kD = kD;
    pid->lastError = 0;
    pid->sumError = 0;
    pid->object = 0;
}

/**
 * @brief Set the target object for the PID controller
 * @param pid Pointer to the PID controller structure
 * @param object The target value that the PID controller should aim for
 */
void PID_SetObject(PID_t* pid, float object)
{
    pid->object = object;
    // pid->lastError = 0;
    // pid->sumError = 0;
}

/**
 * @brief Calculate the PID output based on the current measurement
 * @param pid Pointer to the PID controller structure
 * @param measurement The current measurement value
 * @return The calculated PID output
 */
float PID_Calc(PID_t* pid, float measurement)
{
    float error = pid->object - measurement;
    float differentialError = error - pid->lastError;
    pid->sumError += error;
    if(pid->sumError > MAX_SUM_ERROR) pid->sumError = MAX_SUM_ERROR;
    else if (pid->sumError < -MAX_SUM_ERROR) pid->sumError = -MAX_SUM_ERROR;
    pid->lastError = error;
    return (pid->kP*error + pid->kI*pid->sumError + pid->kD*differentialError);
}
