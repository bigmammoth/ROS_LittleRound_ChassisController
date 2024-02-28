#include "dc_motor.h"
#include "timer.h"
#include "./Algorithm/pid.h"
#include "./Algorithm/kalman_filter.h"

/* ------------------ Definitions --------------------*/
// PI
#define PI 3.14159265358979323846
// Total motor number
#define TOTAL_MOTOR_NUMBER  2
// The sum of the edge counts of the A and B phases of the encoder for every turn of the reducer.
#define EDGE_PER_ROUND	(390*4)
// PID parameters
#define KP  6000.0f
#define KI  200.0f
#define KD  500.0f
// Kalman parameter
#define KALMAN_ESTIMATE_VARIANCE    8
#define KALMAN_MEASURE_VARIANCE     1
#define KALMAN_PROCESS_VARIANCE     0.005
// Encoder timer tick frequency
#define ENCODE_TIMER_FREQUENCY  4200000.0f
// Encoder timer period
#define ENCODER_TIMER_PERIOD    42000
// Measureable minimal angular speed
#define MIN_MEASURE_ANGULAR_SPEED 0.01f

/* --------------- Static functions ------------------- */
static void PeriodCallback(void);
static void EncoderCaptureOverflowCallback(uint32_t channelNo);
static void InputCaptureCallback(int32_t motorID, int32_t channelID, int32_t captureCounter, int32_t edge, int32_t pairLevel);

/* ---------------- Static variables ------------------ */
static int64_t encoderValue[TOTAL_MOTOR_NUMBER];
static PID_t pid[TOTAL_MOTOR_NUMBER];
static KalmanFilter_t filter[TOTAL_MOTOR_NUMBER];
static float measuredAngularSpeed[TOTAL_MOTOR_NUMBER];
static int32_t lastCaputerCounter[TOTAL_MOTOR_NUMBER];
static int32_t encoderCaptureChange[TOTAL_MOTOR_NUMBER];
static int32_t direction[TOTAL_MOTOR_NUMBER];
static int32_t pwmOut[TOTAL_MOTOR_NUMBER];
static int32_t pwmOutMean[TOTAL_MOTOR_NUMBER];

static int32_t encoderCaptureChangeQueue[TOTAL_MOTOR_NUMBER][100];
static int32_t encoderCaptureChangeQueueIndex[TOTAL_MOTOR_NUMBER];

void DCMotor_Init(void)
{
    for (int i = 0; i < TOTAL_MOTOR_NUMBER; i++)
    {
        PID_Init(&pid[i], KP, KI, KD);
        KalmanFilter_Init(&filter[i], KALMAN_ESTIMATE_VARIANCE, KALMAN_MEASURE_VARIANCE, KALMAN_PROCESS_VARIANCE);
    }
    Timer_RegisterEncoderOverflowCallback(EncoderCaptureOverflowCallback);
    Timer_RegisterPeriodCallback(PeriodCallback);
    Timer_RegisterInputCaptureCallback(InputCaptureCallback);
    Timer_TimersForMotorInit();
}

int64_t DCMotor_ReadEncoder(uint32_t motorId)
{
    return encoderValue[motorId];
}

void DCMotor_SetAngularSpeed(uint32_t motorId, float angularSpeed)
{
    PID_SetObject(&pid[motorId], angularSpeed);
}

static void PeriodCallback(void)
{
    for (int i = 0; i < TOTAL_MOTOR_NUMBER; ++i)
    {
        pwmOut[i] = PID_Calc(&pid[i], measuredAngularSpeed[i]);
        if (pwmOut[i] > 65536) pwmOut[i] = 65536;
        else if(pwmOut[i] < -65536) pwmOut[i] = -65536;
        Timer_SetPWM(i, pwmOut[i] + pwmOutMean[i]);
        pwmOutMean[i] = pwmOutMean[i]*0.9 + pwmOut[i]*0.1;
    }
}

/**
 * @brief Encoder update event callback function
 * The encoder will produce an event when it overflows.
 * We record the event by arry encoderOverflows.
 * @param motorID 
 * @param direction 
 */
static void EncoderCaptureOverflowCallback(uint32_t motorID)
{
    encoderCaptureChange[motorID] += ENCODER_TIMER_PERIOD - lastCaputerCounter[motorID];
    lastCaputerCounter[motorID] = 0;
    if(encoderCaptureChange[motorID] > 10*ENCODER_TIMER_PERIOD)
    {
        measuredAngularSpeed[motorID] = 0;
        encoderCaptureChange[motorID] -= ENCODER_TIMER_PERIOD;
    }
}

/**
 * @brief Encoder input capture interrupt callback function
 * 1. Decode the A and B pulse to get the direction.
 * channel:        Pulse A             Pulse B
 * edge:           0      1           0      1
 * pairLevel:1    Dec    Inc         Inc    Dec
 * pairLevel:0    Inc    Dec         Dec    Inc
 * Assign Inc:=1  Dec:=-1 and edge 0:=-1 pairLevel 0:=-1 Pulse A:=-1
 * We can get direction = channel * edge * pairLevel
 * 
 * 2. Caculate the wheel's speeds.
 * 
 * @param motorID 
 * @param channelID 
 * @param captureCounter 
 * @param edge 
 * @param pairLevel 
 */
static void InputCaptureCallback(int32_t motorID, int32_t channelID, int32_t captureCounter, int32_t edge, int32_t pairLevel)
{
    if(!channelID) channelID = -1;
    if(!edge) edge = -1;
    if(!pairLevel) pairLevel = -1;
    direction[motorID] = channelID * edge * pairLevel;
    encoderValue[motorID] += direction[motorID];

    if (!encoderCaptureChange[motorID] && captureCounter < lastCaputerCounter[motorID])
    {
        lastCaputerCounter[motorID] = captureCounter;
        encoderCaptureChange[motorID] = 0;
        return;
    }

    encoderCaptureChange[motorID] += captureCounter - lastCaputerCounter[motorID];
    // encoderCaptureChangeQueue[motorID][encoderCaptureChangeQueueIndex[motorID]++] = encoderCaptureChange[motorID];
    // if(encoderCaptureChangeQueueIndex[motorID] >= 100) encoderCaptureChangeQueueIndex[motorID] = 0;
    float angularSpeed = 
        1.0f / EDGE_PER_ROUND * 2.0f * PI * ENCODE_TIMER_FREQUENCY / (float)encoderCaptureChange[motorID] * direction[motorID];
    measuredAngularSpeed[motorID] = KalmanFilter_Calc(&filter[motorID], angularSpeed);
    lastCaputerCounter[motorID] = captureCounter; 
    encoderCaptureChange[motorID] = 0;    
}
