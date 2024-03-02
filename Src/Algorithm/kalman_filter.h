#pragma once

#include <stdint.h>

typedef struct kalman
{
    float kalmanGain;
    float estimateVariance;
    float measureVariance;
    float lastEstimateValue;
    float processErrorVariance;
} KalmanFilter_t;

void KalmanFilter_Init(KalmanFilter_t* , float estimateVariance, float measureVariance, float processErrorVariance);
float KalmanFilter_Calc(KalmanFilter_t* , float measurement);
