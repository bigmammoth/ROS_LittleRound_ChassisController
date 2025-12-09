/**
 * @file crc32.h
 * @brief CRC32 calculation implementation
 * @author Young.R com.wang@hotmail.com
 * @version 0.1
 * @date 2022-07-02
 * This module provides functions to initialize a lookup table for fast CRC-32
 * computation and to calculate CRC-32 checksums over arbitrary data buffers.
 * It uses the standard polynomial 0x04C11DB7, initial value 0xFFFFFFFF.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#define CRC32_INITIAL_VALUE 0xFFFFFFFF      // Initial value for CRC32 calculation

/**
 * @brief Caculate CRC32 for a series of 32bits data
 * @param crc Initial CRC value, if it is the first time, use 0xFFFFFFFF.
 * or accumulate CRC, use crc value got from last caculation.
 * @param input Input data buffer.
 * @param len Data length in 8bits.
 * @return CRC value.
 */
uint32_t Crc32(uint32_t crc, const void* input, uint32_t len);
