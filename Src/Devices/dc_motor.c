#include "dc_motor.h"
#include "timer.h"
#include "./Algorithm/pid.h"

/* ------------------ Definitions --------------------*/
// PI
#define PI 3.14159265358979323846
// Total motor number
#define TOTAL_MOTOR_NUMBER  2
// The sum of the edge counts of the A and B phases of the encoder for every turn of the reducer.
#define EDGE_PER_ROUND	(390*4)
// PID parameters
#define KP  0.5f
#define KI  0.0001f
#define KD  0.2f
// Encoder timer tick frequency
#define ENCODE_TIMER_FREQUENCY  4200000.0f
// Measureable minimal angular speed
#define MIN_MEASURE_ANGULAR_SPEED 0.01f

/* --------------- Static functions ------------------- */
static void PeriodCallback(void);
static void EncoderCaptureOverflowCallback(uint32_t channelNo);
static void InputCaptureCallback(int32_t motorID, int32_t channelID, int32_t captureCounter, int32_t edge, int32_t pairLevel);

/* ---------------- Static variables ------------------ */
static int64_t encoderValue[TOTAL_MOTOR_NUMBER];
static PID_t pid[TOTAL_MOTOR_NUMBER];
static float measuredAngularSpeed[TOTAL_MOTOR_NUMBER];
static int32_t lastCaputerCounter[TOTAL_MOTOR_NUMBER];
static int32_t encoderCaptureChange[TOTAL_MOTOR_NUMBER];
static int32_t direction[TOTAL_MOTOR_NUMBER];

void DCMotor_Init(void)
{
    Timer_RegisterEncoderOverflowCallback(EncoderCaptureOverflowCallback);
    Timer_RegisterPeriodCallback(PeriodCallback);
    Timer_RegisterInputCaptureCallback(InputCaptureCallback);
    Timer_TimersForMotorInit();
    for (int i = 0; i < TOTAL_MOTOR_NUMBER; i++) PID_Init(&pid[i], KP, KI, KD);
}

int64_t DCMotor_ReadEncoder(uint32_t motorId)
{
    return encoderValue[motorId];
}

static void PeriodCallback(void)
{
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
    encoderCaptureChange[motorID] += 65536 - lastCaputerCounter[motorID];
    measuredAngularSpeed[motorID] = measuredAngularSpeed[motorID] * 0.5 \
        + 0.5 * 1.0f / EDGE_PER_ROUND * 2.0f * PI * ENCODE_TIMER_FREQUENCY / (float)encoderCaptureChange[motorID] * direction[motorID];
    lastCaputerCounter[motorID] = 0;
    if(measuredAngularSpeed[motorID] < MIN_MEASURE_ANGULAR_SPEED && measuredAngularSpeed[motorID] > -MIN_MEASURE_ANGULAR_SPEED)
    {
        measuredAngularSpeed[motorID] = 0;
        encoderCaptureChange[motorID] -= 65536;
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

    encoderCaptureChange[motorID] += captureCounter - lastCaputerCounter[motorID];
    measuredAngularSpeed[motorID] = measuredAngularSpeed[motorID] * 0.5 \
        + 0.5 * 1.0f / EDGE_PER_ROUND * 2.0f * PI * ENCODE_TIMER_FREQUENCY / (float)encoderCaptureChange[motorID] * direction[motorID];
    lastCaputerCounter[motorID] = captureCounter;
    encoderCaptureChange[motorID] = 0;    
}
