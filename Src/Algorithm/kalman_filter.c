#include "kalman_filter.h"
#include "math.h"

/**
 * @file kalman_filter.c
 * @author Young.R
 * @brief Kalman filter implementation
 * @version 0.1
 */

/**
 * @brief Initialize the Kalman filter
 * 
 * @param filter Pointer to the Kalman filter structure
 * @param estimateVariance Initial variance of the estimate
 * @param measureVariance Variance of the measurement noise
 * @param processErrorVariance Variance of the process error
 */
void KalmanFilter_Init(KalmanFilter_t* filter, float estimateVariance, float measureVariance, float processErrorVariance)
{
    filter->estimateVariance = estimateVariance;
    filter->measureVariance = measureVariance;
    filter->lastEstimateValue = 0;
    filter->processErrorVariance = processErrorVariance;
    filter->kalmanGain = 0;
}

/**
 * @brief Calculate the Kalman filter output based on the measurement
 * 
 * @param filter Pointer to the Kalman filter structure
 * @param measurement The current measurement value
 * @return The filtered estimate value
 */
float KalmanFilter_Calc(KalmanFilter_t* filter, float measurement)
{
    filter->kalmanGain = filter->estimateVariance / (filter->estimateVariance + filter->measureVariance);
    filter->estimateVariance = (1 - filter->kalmanGain) * filter->estimateVariance;
    filter->lastEstimateValue = filter->lastEstimateValue + filter->kalmanGain * (measurement - filter->lastEstimateValue);
    filter->estimateVariance += filter->processErrorVariance;
    return filter->lastEstimateValue;
}
