/**
 * @file io.c
 * @brief Implementation of IO Control Module
 * This module provides functions to control input and output IO ports.
 * It allows setting output port levels and registering callbacks for input port state changes.
 * @date 2025-10-27
 * @author Young.R <com.wang@hotmail.com>
 * @version 1.0
 * @note This module is part of the Peripherals system.
 */

#include "main.h"
#include "io.h"

typedef struct {
    GPIO_TypeDef *Port;
    uint16_t Pin;
} IO_t;

/* -------------------- Static variables ---------------------- */
static osThreadId_t appIoInputThreadId;
// IO output ports map
static IO_t IO_Output[] = {
    {OUT0_GPIO_Port, OUT0_Pin},
    {OUT1_GPIO_Port, OUT1_Pin},
    {OUT2_GPIO_Port, OUT2_Pin},
};

// IO input ports map
static IO_t IO_Input[] = {
    {IN0_GPIO_Port, IN0_Pin},
    {IN1_GPIO_Port, IN1_Pin},
    {IN2_GPIO_Port, IN2_Pin},
};

#define IO_OUTPUT_NUMBER (sizeof(IO_Output)/sizeof(IO_t))
#define IO_INPUT_NUMBER (sizeof(IO_Input)/sizeof(IO_t))

/* --------------------- Static Variables --------------------- */
// IO input event callback function array
static IoCallback_t ioInputCallback[IO_INPUT_NUMBER];
// Read input ports interval
static const uint16_t READ_INPUT_INTERVAL = 100; // 100ms

/* --------------------- Static Functions --------------------- */
static void AppIoInputThread(void *arg);

/**
 * @brief Write to output IO port
 * @param ioPort IO port number
 * @param level Output level, high or low
 */
void IO_Write(uint16_t ioPort, bool level)
{
    if (ioPort >= IO_OUTPUT_NUMBER) return;
    HAL_GPIO_WritePin(IO_Output[ioPort].Port, IO_Output[ioPort].Pin, level);
}

/**
 * @brief Read input IO port status
 * @param ioPort IO port number
 * @retval Input status, high or low
 */
bool IO_Read(uint16_t ioPort)
{
    if (ioPort >= IO_INPUT_NUMBER) return false;
    GPIO_PinState status = HAL_GPIO_ReadPin(IO_Input[ioPort].Port, IO_Input[ioPort].Pin);
    return (bool)status;
}

/**
 * @brief Initialize AppIoInput thread.
 * @param None
 * @retval None
 */
void IO_Init(void)
{
    for (int i = 0; i < IO_INPUT_NUMBER; ++i) ioInputCallback[i] = NULL;
    appIoInputThreadId = osThreadNew(AppIoInputThread, NULL, NULL);
    // osThreadSetPriority(appIoInputThreadId, osPriorityBelowNormal);
}

/**
 * @brief Register IO input callback function
 * Provide a method to other thread to register a callback function.
 * Everytime input IO ports are read, corresponding callback will be called,
 * and the corresponding pin state will be passed to the registered function.
 * @param callback Callback function pointer.
 * @param pin The pin the callback function will be attached.
 * @retval None
 */
void IO_RegisterCallback(IoCallback_t callback, uint16_t pin)
{
    if (pin >= IO_INPUT_NUMBER)
        return;
    if (ioInputCallback[pin] == NULL) ioInputCallback[pin] = callback;
}

/**
 * @brief Thread for monitering the input IO ports
 * @param arg Pointer to the argument which was transfered to
 *  the thread while it was creating.
 * @retval None
 */
void AppIoInputThread(void *arg)
{
    while (true)
    {
        osDelay(READ_INPUT_INTERVAL);
        for (int i = 0; i < IO_INPUT_NUMBER; ++i)
        {
            if (ioInputCallback[i] != NULL)
			{
				GPIO_PinState status = HAL_GPIO_ReadPin(IO_Input[i].Port, IO_Input[i].Pin);
				ioInputCallback[i](status);
			}
        }
    }
}
