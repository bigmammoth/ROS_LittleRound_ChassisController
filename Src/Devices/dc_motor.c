#include "dc_motor.h"
#include "timer.h"
#include "pid.h"
#include "kalman_filter.h"
#include "system_config.h"

/* ------------------ Definitions --------------------*/
// PI
#define PI 3.14159265358979323846
// The sum of the edge counts of the A and B phases of the encoder for every turn of the reducer.
#define EDGE_PER_ROUND	(13*30*4) // 13 pulses per turn, 30:1 reducer, 4 edges per pulse
// PID parameters
#define KP  0.1f
#define KI  0.01f
#define KD  0.01f
// Kalman parameter
#define KALMAN_ESTIMATE_VARIANCE    8.0f
#define KALMAN_MEASURE_VARIANCE     1.0f
#define KALMAN_PROCESS_VARIANCE     0.1f
// Encoder timer tick frequency
#define ENCODE_TIMER_FREQUENCY  4200000.0f
// Encoder timer period
#define ENCODER_TIMER_PERIOD    42000
// PID control frequency
#define PID_CONTROL_FREQUENCY   50      // 50Hz
// PID control period in seconds
#define PID_CONTROL_PERIOD_S    (1.0f / (float)PID_CONTROL_FREQUENCY)   // 20ms

/* --------------- Static functions ------------------- */
static void PeriodCallback(void);
static void InputCaptureCallback(int32_t motorID, int32_t channelID, int32_t captureCounter, int32_t edge, int32_t pairLevel);
static void Encoder0_OverflowCallback(void);
static void Encoder1_OverflowCallback(void);

/* ---------------- Static variables ------------------ */
static int64_t encoderPosition[TOTAL_MOTOR_NUMBER];
static PID_t pid[TOTAL_MOTOR_NUMBER];
static KalmanFilter_t filter[TOTAL_MOTOR_NUMBER];
// Angular velocity measured by encoders after passing a Kalman filter.
static float measuredAngularSpeed[TOTAL_MOTOR_NUMBER];
// Motor's encoder overflow counter
static int32_t encoderOverflowCounter[TOTAL_MOTOR_NUMBER];

/**
 * @brief Initialize the DC Motor control system
 * This function initializes the PID controllers and Kalman filters for each motor,
 * registers the encoder overflow callbacks, and initializes the timers for motor control.
 */
void DCMotor_Init(void)
{
    for (int i = 0; i < TOTAL_MOTOR_NUMBER; i++)
    {
        PID_Init(&pid[i], KP, KI, KD);
        KalmanFilter_Init(&filter[i], KALMAN_ESTIMATE_VARIANCE, KALMAN_MEASURE_VARIANCE, KALMAN_PROCESS_VARIANCE);
    }
    Timer_RegisterEncoderOverflowCallback(0, Encoder0_OverflowCallback);
    Timer_RegisterEncoderOverflowCallback(1, Encoder1_OverflowCallback);
    Timer_RegisterPeriodCallback(PeriodCallback);
    Timer_TimersForMotorInit();
}

/**
 * @brief Read the encoder value of the motor
 * @param motorId The ID of the motor
 * @return The encoder value in counts
 */
int64_t DCMotor_ReadEncoder(uint32_t motorId)
{
    return encoderPosition[motorId];
}

/**
 * @brief Set the angular speed of the motor
 * @param motorId The ID of the motor
 * @param angularSpeed The desired angular speed in rad/s
 */
void DCMotor_SetAngularSpeed(uint32_t motorId, float angularSpeed)
{
    PID_SetObject(&pid[motorId], angularSpeed);
}

/**
 * @brief Periodic callback function
 * This function is called periodically to update the motor control.
 */
static void PeriodCallback(void)
{
    for (int i = 0; i < TOTAL_MOTOR_NUMBER; ++i)
    {
        // Read the encoder value
        int64_t position = (int64_t)Timer_ReadEncoder(i) + encoderOverflowCounter[i] * 0x10000;
        // Calculate the position difference
        int64_t deltaPosition = position - encoderPosition[i];
        encoderPosition[i] = position;
        // Calculate the angular speed in rad/s
        float angularSpeed = (float)deltaPosition * (2.0f * PI * (float)PID_CONTROL_FREQUENCY  / (float)EDGE_PER_ROUND);
        // Apply Kalman filter to the measured angular speed
        measuredAngularSpeed[i] = KalmanFilter_Calc(&filter[i], angularSpeed);
        // PID control
        float output = PID_Calc(&pid[i], measuredAngularSpeed[i]);
        Timer_PWM_SetDuty(i, output);
    }
}

/**
 * @brief Get the angular speed of the motor
 * @param motorId The ID of the motor
 * @return The angular speed in rad/s
 */
float DCMotor_GetAngularSpeed(uint32_t motorId)
{
    return measuredAngularSpeed[motorId];
}

/**
 * @brief Get the encoder value of the motor
 * @param motorId The ID of the motor
 * @return The encoder value in rounds
 */
double DCMotor_GetEncoderValue(uint32_t motorId)
{
    return (double)encoderPosition[motorId] / (double)EDGE_PER_ROUND; // Convert encoder counts to rounds
}

/**
 * @brief Encoder counter overflow callback for motor 0.
 * If the count > 0x7FF, means it turns from 0 to 0xFFFF,
 * otherwise it means the counter turns from 0xFFFF to 0.
 * 
 * @param  
 */
void Encoder0_OverflowCallback(void)
{
    uint32_t count = Timer_ReadEncoder(0);
    if (count > 0x7FFF) --encoderOverflowCounter[0]; 
    else ++encoderOverflowCounter[0];
}

/**
 * @brief Encoder counter overflow callback for motor 1.
 * If the count > 0x7FF, means it turns from 0 to 0xFFFF,
 * otherwise it means the counter turns from 0xFFFF to 0.
 */
void Encoder1_OverflowCallback(void)
{
    uint32_t count = Timer_ReadEncoder(1);
    if (count > 0x7FFF) --encoderOverflowCounter[1]; 
    else ++encoderOverflowCounter[1];
}
