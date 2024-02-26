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
static Timer_EncoderOverflowCallbak_t EncoderCallback;
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
        __HAL_TIM_CLEAR_FLAG(&encoderDictionary[i], TIM_FLAG_UPDATE);
        HAL_TIM_Base_Start_IT(&encoderDictionary[i]);
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
        // Get direction from CR1_DIR bit
        int32_t direction = 1;
		if(htim->Instance->CR1 & TIM_CR1_DIR) direction = -1;
        if(EncoderCallback != NULL) EncoderCallback(0, direction);
    }
    else if(htim->Instance == TIM4) // For motor1's encoder
    {
        // Get direction from CR1_DIR bit
        int32_t direction = 1;
		if(htim->Instance->CR1 & TIM_CR1_DIR) direction = -1;
        if(EncoderCallback != NULL)  EncoderCallback(1, direction);
    }
    else if(htim->Instance == TIM9)
    {}
    else if(htim->Instance == TIM7) // Used for motor control cycle, interrupts every 1ms
    {
        if(PerioidCallback != NULL)  PerioidCallback();
    }
}

uint32_t Timer_ReadEncoder(uint32_t encoderID)
{
    return __HAL_TIM_GET_COUNTER(&encoderDictionary[encoderID]);
}

void Timer_RegisterPeriodCallback(Timer_PeriodCallback_t callback)
{
    if(PerioidCallback == NULL) PerioidCallback = callback;
}

void Timer_RegisterEncoderOverflowCallback(Timer_EncoderOverflowCallbak_t callback)
{
    if (EncoderCallback == NULL) EncoderCallback = callback;
}
