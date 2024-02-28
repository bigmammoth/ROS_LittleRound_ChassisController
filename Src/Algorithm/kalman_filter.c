#include "kalman_filter.h"
#include "math.h"

void KalmanFilter_Init(KalmanFilter_t* filter, float estimateVariance, float measureVariance, float processErrorVariance)
{
    filter->estimateVariance = estimateVariance;
    filter->measureVariance = measureVariance;
    filter->lastEstimateValue = 0;
    filter->processErrorVariance = processErrorVariance;
    filter->kalmanGain = 0;
    filter->measure4MSE = sqrt(measureVariance * 4);    // 4 Sigma
    // filter->abnormalMeasureCounter = 0;
    // filter->normalCounter = 0;
}

float KalmanFilter_Calc(KalmanFilter_t* filter, float measurement)
{
    // Discard abnormal measurements that exceed the 4-sigma range.
    // float upBound = filter->lastEstimateValue + filter->measure4MSE;
    // float downBound = filter->lastEstimateValue - filter->measure4MSE;
    // float lastKalmanGain = filter->kalmanGain;
    // float kalmanGain = filter->estimateVariance / (filter->estimateVariance + filter->measureVariance);
    // float diffKalmanGain = kalmanGain - lastKalmanGain;
    // if ((diffKalmanGain < 0.000000001 && diffKalmanGain > -0.000000001) && (measurement < downBound || measurement > upBound))
    // {
    //     if (!filter->abnormalMeasureCounter)
    //     {
    //         ++filter->abnormalMeasureCounter;
    //         return filter->lastEstimateValue;
    //     }
    // }
    // filter->abnormalMeasureCounter = 0;
    // if(++filter->normalCounter > 100000) filter->normalCounter = 100000;

    // Update Kalman gain, estimate variance and estimate value.
    // filter->kalmanGain = kalmanGain;
    filter->kalmanGain = filter->estimateVariance / (filter->estimateVariance + filter->measureVariance);
    filter->estimateVariance = (1 - filter->kalmanGain) * filter->estimateVariance;
    filter->lastEstimateValue = filter->lastEstimateValue + filter->kalmanGain * (measurement - filter->lastEstimateValue);
    filter->estimateVariance += filter->processErrorVariance;
    return filter->lastEstimateValue;
}
