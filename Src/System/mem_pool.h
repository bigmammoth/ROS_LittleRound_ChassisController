#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize the memory pool system.
 * This function sets up the memory pools for different block sizes.
 */
void MemPool_Init(void);

/**
 * @brief Allocate a memory block from the pool.
 * @param size The size of the memory block to allocate.
 * @return Pointer to the allocated memory block, or NULL if allocation failed.
 */
void* MemPool_Alloc(uint32_t size);

/**
 * @brief Free a previously allocated memory block back to the pool.
 * @param ptr Pointer to the memory block to free.
 * @return true if the memory block was successfully freed, false otherwise.
 */
bool MemPool_Free(void* ptr);
