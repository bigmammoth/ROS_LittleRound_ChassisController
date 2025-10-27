/**
 * @file io.h
 * @brief Header file for IO Control Module
 * This header defines the interface for controlling input and output IO ports.
 * It includes functions for setting output port levels, initializing the IO module,
 * and registering callbacks for input port state changes.
 * @date 2025-10-27
 * @author Young.R <com.wang@hotmail.com>
 * @version 1.0
 * @note This module is part of the Peripherals system.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// IO input callback function type definition
typedef void (*IoCallback_t)(bool pinState);

/**
 * @brief Write to output IO port
 * @param ioPort IO port number
 * @param level Output level, high or low
 */
void IO_Write(uint16_t ioPort, bool level);

/**
 * @brief Read input IO port status
 * @param ioPort IO port number
 * @retval Input status, high or low
 */
bool IO_Read(uint16_t ioPort);

/**
 * @brief Initialize the IO module
 * @retval None
 */
void IO_Init(void);

/**
 * @brief Register IO input callback function
 * Provide a method to other thread to register a callback function.
 * Everytime input IO ports are read, corresponding callback will be called,
 * and the corresponding pin state will be passed to the registered function.
 * @param callback Callback function pointer.
 * @param pin The pin the callback function will be attached.
 * @retval None
 */
void IO_RegisterCallback(IoCallback_t callback, uint16_t pin);
