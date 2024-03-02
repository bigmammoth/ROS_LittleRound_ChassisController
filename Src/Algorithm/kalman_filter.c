#include "kalman_filter.h"
#include "math.h"

void KalmanFilter_Init(KalmanFilter_t* filter, float estimateVariance, float measureVariance, float processErrorVariance)
{
    filter->estimateVariance = estimateVariance;
    filter->measureVariance = measureVariance;
    filter->lastEstimateValue = 0;
    filter->processErrorVariance = processErrorVariance;
    filter->kalmanGain = 0;
}

float KalmanFilter_Calc(KalmanFilter_t* filter, float measurement)
{
    filter->kalmanGain = filter->estimateVariance / (filter->estimateVariance + filter->measureVariance);
    filter->estimateVariance = (1 - filter->kalmanGain) * filter->estimateVariance;
    filter->lastEstimateValue = filter->lastEstimateValue + filter->kalmanGain * (measurement - filter->lastEstimateValue);
    filter->estimateVariance += filter->processErrorVariance;
    return filter->lastEstimateValue;
}
