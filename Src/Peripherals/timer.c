#include "timer.h"

#define TOTAL_ENCODER_NUMBER    2

/*----------------------------- External variables -------------------------*/
extern TIM_HandleTypeDef htim2;     // For motor0's PWM
extern TIM_HandleTypeDef htim3;     // For motor0's encoder
extern TIM_HandleTypeDef htim4;     // For motor1's encoder
extern TIM_HandleTypeDef htim9;     // For motor1's PWM
extern TIM_HandleTypeDef htim7;     // For motor's pid control period

/* --------------------- Static variables --------------------------------- */
static Timer_PeriodCallback_t PerioidCallback;
static Timer_EncoderOverflowCallback_t EncoderCallback;
static Timer_InputCaptureCallback_t InputCaptureCallback;
static TIM_HandleTypeDef encoderDictionary[TOTAL_ENCODER_NUMBER];

/**
 * @brief Initialize timers for motor.
 * 
 */
void Timer_TimersForMotorInit(void)
{
    encoderDictionary[0] = htim3;
    encoderDictionary[1] = htim4;
    for(int i = 0; i < TOTAL_ENCODER_NUMBER; i++)
    {
        HAL_TIM_Base_Start_IT(&encoderDictionary[i]);
        HAL_TIM_IC_Start_IT(&encoderDictionary[i], TIM_CHANNEL_1);
        HAL_TIM_IC_Start_IT(&encoderDictionary[i], TIM_CHANNEL_2);
    }
    HAL_TIM_Base_Start_IT(&htim7);
	// HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   // For motor0 PWM channel 1
	// HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);   // For motor0 PWM channel 2
	// HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);   // For motor1 PWM channel 1
	// HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);   // For motor1 PWM channel 2
}

/**
 * @brief Timer callback function
 * 
 * @param htim 
 */
void Timer_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {}
    else if(htim->Instance == TIM3) // For motor0's encoder
    {
        if(EncoderCallback != NULL) EncoderCallback(0);
    }
    else if(htim->Instance == TIM4) // For motor1's encoder
    {
        if(EncoderCallback != NULL)  EncoderCallback(1);
    }
    else if(htim->Instance == TIM9)
    {}
    else if(htim->Instance == TIM7) // Used for motor control cycle, interrupts every 1ms
    {
        if(PerioidCallback != NULL)  PerioidCallback();
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t motor, channel, capture, edge, pairLevel;
    if(htim->Instance == TIM3) 
    {
        motor = 0;
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            capture = htim->Instance->CCR1;
            edge = LL_GPIO_IsInputPinSet(MOTOR0_IC1_GPIO_Port, MOTOR0_IC1_Pin);
            pairLevel = LL_GPIO_IsInputPinSet(MOTOR0_IC2_GPIO_Port, MOTOR0_IC2_Pin);
        }
        else
        {
            capture = htim->Instance->CCR2;
            edge = LL_GPIO_IsInputPinSet(MOTOR0_IC2_GPIO_Port, MOTOR0_IC2_Pin);
            pairLevel = LL_GPIO_IsInputPinSet(MOTOR0_IC1_GPIO_Port, MOTOR0_IC1_Pin);
        }
    }
    else if(htim->Instance == TIM4)
    {
        motor = 1;
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            capture = htim->Instance->CCR1;
            edge = LL_GPIO_IsInputPinSet(MOTOR1_IC1_GPIO_Port, MOTOR1_IC1_Pin);
            pairLevel = LL_GPIO_IsInputPinSet(MOTOR1_IC2_GPIO_Port, MOTOR1_IC2_Pin);
        }
        else
        {
            capture = htim->Instance->CCR2;
            edge = LL_GPIO_IsInputPinSet(MOTOR1_IC2_GPIO_Port, MOTOR1_IC2_Pin);
            pairLevel = LL_GPIO_IsInputPinSet(MOTOR1_IC1_GPIO_Port, MOTOR1_IC1_Pin);
        }
    }
    channel = htim->Channel - 1;
    InputCaptureCallback(motor, channel, capture, edge, pairLevel);
}

uint32_t Timer_ReadEncoder(uint32_t encoderID)
{
    return __HAL_TIM_GET_COUNTER(&encoderDictionary[encoderID]);
}

void Timer_RegisterPeriodCallback(Timer_PeriodCallback_t callback)
{
    if(PerioidCallback == NULL) PerioidCallback = callback;
}

void Timer_RegisterEncoderOverflowCallback(Timer_EncoderOverflowCallback_t callback)
{
    if (EncoderCallback == NULL) EncoderCallback = callback;
}

void Timer_RegisterInputCaptureCallback(Timer_InputCaptureCallback_t callback)
{
    if(InputCaptureCallback == NULL) InputCaptureCallback = callback;
}
