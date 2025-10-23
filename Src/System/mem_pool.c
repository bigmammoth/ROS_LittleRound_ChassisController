#include "mem_pool.h"
#include "cmsis_os2.h"

/* ----------------------- Definitions ----------------------- */
// Size of each memory pool
#define MEMPOOL_64_OBJECTS 64
#define MEMPOOL_128_OBJECTS 32
#define MEMPOOL_256_OBJECTS 16
#define MEMPOOL_512_OBJECTS 8
#define MEMPOOL_1024_OBJECTS 4
#define MEMPOOL_2048_OBJECTS 2

#define MEMPOOL_TYPES 6

/* ----------------------- Static Variables ----------------------- */
static struct _MEM_POOL
{
    osMemoryPoolId_t memPoolId;
    uint32_t memPoolObjects;
    uint32_t memSize;
} memPool[MEMPOOL_TYPES];

/**
 * @brief Initialize the memory pool system.
 * This function sets up the memory pools for different block sizes.
 */
void MemPool_Init(void)
{
    memPool[0].memPoolObjects = MEMPOOL_64_OBJECTS;
    memPool[1].memPoolObjects = MEMPOOL_128_OBJECTS;
    memPool[2].memPoolObjects = MEMPOOL_256_OBJECTS;
    memPool[3].memPoolObjects = MEMPOOL_512_OBJECTS;
    memPool[4].memPoolObjects = MEMPOOL_1024_OBJECTS;
    memPool[5].memPoolObjects = MEMPOOL_2048_OBJECTS;
    uint32_t size = 128;
    for (int i = 0; i < MEMPOOL_TYPES; ++i)
    {
        memPool[i].memSize = size;
        size <<= 1;
        memPool[i].memPoolId = osMemoryPoolNew(memPool[i].memPoolObjects, memPool[i].memSize, NULL);
    }
}

/**
 * @brief Allocate a memory block from the pool.
 * @param size The size of the memory block to allocate.
 * @return Pointer to the allocated memory block, or NULL if allocation failed.
 */
void* MemPool_Alloc(uint32_t size)
{
    for (int i = 0; i < MEMPOOL_TYPES; ++i)
    {
        if (size <= memPool[i].memSize)
        {
            return osMemoryPoolAlloc(memPool[i].memPoolId, 0);
        }
    }
    return NULL; // Requested size too large
}

/**
 * @brief Free a previously allocated memory block back to the pool.
 * @param ptr Pointer to the memory block to free.
 * @return true if the memory block was successfully freed, false otherwise.
 */
bool MemPool_Free(void* ptr)
{
    for (int i = 0; i < MEMPOOL_TYPES; ++i)
    {
        if (osMemoryPoolFree(memPool[i].memPoolId, ptr) == osOK)
            return true;
    }
    return false;
}
