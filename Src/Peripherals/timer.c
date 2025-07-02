#include "timer.h"

#define TOTAL_ENCODER_NUMBER    2
#define TOTAL_MOTOR_NUMBER  TOTAL_ENCODER_NUMBER

typedef struct pwm_channel {
    TIM_HandleTypeDef pwmTimer;
    uint32_t pwmChannel0;
    uint32_t pwmChannel1;
} PWM_Channel_t;

/*----------------------------- External variables -------------------------*/
extern TIM_HandleTypeDef htim2;     // For motor0's PWM
extern TIM_HandleTypeDef htim3;     // For motor0's encoder
extern TIM_HandleTypeDef htim4;     // For motor1's encoder
extern TIM_HandleTypeDef htim9;     // For motor1's PWM
extern TIM_HandleTypeDef htim7;     // For motor's pid control period

/* --------------------- Static variables --------------------------------- */
static Timer_PeriodCallback_t PerioidCallback;
static Timer_EncoderOverflowCallback_t EncoderOverflowCallback[TOTAL_ENCODER_NUMBER];
static Timer_InputCaptureCallback_t InputCaptureCallback;
static TIM_HandleTypeDef encoderDictionary[TOTAL_ENCODER_NUMBER];
static PWM_Channel_t pwmChannel[TOTAL_MOTOR_NUMBER];

/* --------------------- Static Functions --------------------------------- */
static void Timer7_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
static void Timer3_PeriodElapsedCallback(TIM_HandleTypeDef *htim); 
static void Timer4_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/**
 * @brief Initialize timers for motor.
 * 
 */
void Timer_TimersForMotorInit(void)
{
    encoderDictionary[0] = htim3;
    encoderDictionary[1] = htim4;
    pwmChannel[0].pwmTimer = htim2;
    pwmChannel[0].pwmChannel0 = TIM_CHANNEL_1;
    pwmChannel[0].pwmChannel1 = TIM_CHANNEL_4;
    pwmChannel[1].pwmTimer = htim9;
    pwmChannel[1].pwmChannel0 = TIM_CHANNEL_1;
    pwmChannel[1].pwmChannel1 = TIM_CHANNEL_2;

    HAL_TIM_RegisterCallback(&htim7, HAL_TIM_PERIOD_ELAPSED_CB_ID, Timer7_PeriodElapsedCallback);
    HAL_TIM_RegisterCallback(&htim3, HAL_TIM_PERIOD_ELAPSED_CB_ID, Timer3_PeriodElapsedCallback);
    HAL_TIM_RegisterCallback(&htim4, HAL_TIM_PERIOD_ELAPSED_CB_ID, Timer4_PeriodElapsedCallback);

    for(int i = 0; i < TOTAL_ENCODER_NUMBER; i++)
        HAL_TIM_Base_Start_IT(&encoderDictionary[i]);

    for(int i = 0; i < TOTAL_MOTOR_NUMBER; i++)
    {
        HAL_TIM_PWM_Start(&pwmChannel[i].pwmTimer, pwmChannel[i].pwmChannel0);
        HAL_TIM_PWM_Start(&pwmChannel[i].pwmTimer, pwmChannel[i].pwmChannel1);
    }
    HAL_TIM_Base_Start_IT(&htim7);
}

/**
 * @brief Timer7 Period Elapsed Callback
 * 
 * It is be used to handle encoder overflow.
 */
void Timer7_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // This function is called every 1ms by TIM7
    if(PerioidCallback != NULL) PerioidCallback();
}

/**
 * @brief Timer3 overflow Callback
 * 
 * It is be used to handle encoder overflow.
 */
void Timer3_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // This function is called every 1ms by TIM3
    if(EncoderOverflowCallback[0] != NULL) EncoderOverflowCallback[0]();
}

/**
 * @brief Timer4 overflow callback
 * 
 * It is used to handle encoder overflow.
 */
void Timer4_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // This function is called every 1ms by TIM4
    if(EncoderOverflowCallback[1] != NULL) EncoderOverflowCallback[1]();
}

uint32_t Timer_ReadEncoder(uint32_t encoderID)
{
    return __HAL_TIM_GET_COUNTER(&encoderDictionary[encoderID]);
}

void Timer_RegisterPeriodCallback(Timer_PeriodCallback_t callback)
{
    if(PerioidCallback == NULL) PerioidCallback = callback;
}

void Timer_RegisterEncoderOverflowCallback(uint32_t encoderID, Timer_EncoderOverflowCallback_t callback)
{
    if (encoderID >= TOTAL_ENCODER_NUMBER)
        return; // Invalid encoder ID
    if (EncoderOverflowCallback[encoderID] == NULL) EncoderOverflowCallback[encoderID] = callback;
}

void Timer_PWM_SetDuty(uint32_t motorID, float duty)
{
    if (motorID >= TOTAL_MOTOR_NUMBER)
        return; // Invalid motor ID
    if (duty < -1.0f) duty = -1.0f; // Limit the duty cycle to -1.0f to 1.0f
    else if (duty > 1.0f) duty = 1.0f; // Limit the duty cycle to -1.0f to 1.0f
    uint16_t pwmValue;
    if (duty >= 0)
    {
        pwmValue = duty * __HAL_TIM_GET_AUTORELOAD(&pwmChannel[motorID].pwmTimer);
        __HAL_TIM_SetCompare(&pwmChannel[motorID].pwmTimer, pwmChannel[motorID].pwmChannel0, pwmValue);
        __HAL_TIM_SetCompare(&pwmChannel[motorID].pwmTimer, pwmChannel[motorID].pwmChannel1, 0);
    }
    else
    {
        pwmValue = -duty * __HAL_TIM_GET_AUTORELOAD(&pwmChannel[motorID].pwmTimer);
        __HAL_TIM_SetCompare(&pwmChannel[motorID].pwmTimer, pwmChannel[motorID].pwmChannel0, 0);
        __HAL_TIM_SetCompare(&pwmChannel[motorID].pwmTimer, pwmChannel[motorID].pwmChannel1, pwmValue);
    }
}
