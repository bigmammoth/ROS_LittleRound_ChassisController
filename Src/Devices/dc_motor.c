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
#define KP  0.5
#define KI  0.0001
#define KD  0.2

/* --------------- Static functions ------------------- */
static void PeriodCallback(void);
static void EncoderCallback(uint32_t channelNo, int32_t direction);
static void InputCaptureCallback(uint32_t motorID, uint32_t channelID, uint32_t captureCounter, uint32_t edge, uint32_t pairLevel);

/* ---------------- Static variables ------------------ */
static int32_t encoderOverflows[TOTAL_MOTOR_NUMBER];
static int64_t encoderValue[TOTAL_MOTOR_NUMBER];//, lastEncoderValue[TOTAL_MOTOR_NUMBER];
static PID_t pid[TOTAL_MOTOR_NUMBER];
static int32_t speed[TOTAL_MOTOR_NUMBER];

void DCMotor_Init(void)
{
    Timer_RegisterEncoderOverflowCallback(EncoderCallback);
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
    for(int i = 0; i < TOTAL_MOTOR_NUMBER; i++)
    {
        // int64_t lastEncoderValue = encoderValue[i];
        // // Read and caculate current encoder value.
        // encoderValue[i] = (int64_t)(encoderOverflows[i] * 0xFFFF) + Timer_ReadEncoder(i);
        // // Caculate speed
        // speed[i] = (int32_t)(encoderValue[i] - lastEncoderValue);
        // // Motor PID control
        // float output = PID_Calc(&pid[i], (float)speed[i]);
    }
}

/**
 * @brief Encoder update event callback function
 * The encoder will produce an event when it overflows.
 * We record the event by arry encoderOverflows.
 * @param channelNo 
 * @param direction 
 */
static void EncoderCallback(uint32_t channelNo, int32_t direction)
{
    encoderOverflows[channelNo] += direction;
}

static void InputCaptureCallback(uint32_t motorID, uint32_t channelID, uint32_t captureCounter, uint32_t edge, uint32_t pairLevel)
{
    ++encoderValue[motorID];
}
