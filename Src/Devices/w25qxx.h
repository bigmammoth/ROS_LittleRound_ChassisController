/**
 * @file w25qxx.h
 * @brief Driver for Winbond W25Qxx SPI NOR flash devices (single I/O mode).
 *
 * @details Implements basic operations for a subset of W25Q series parts
 * (8Mbit–128Mbit) including:
 *  - JEDEC / Manufacturer + Device ID read
 *  - Status register read / write
 *  - Sector erase (4KB) and full chip erase commands (chip erase wrapper TBD)
 *  - Page program (256-byte pages) with automatic sector erase handling
 *  - Linear read API for arbitrary address/length
 *
 * The module internally maintains a singleton instance (W25QXX_t) initialized
 * on first call to W25QXX_Init(). Write operations perform necessary sector
 * erasures when data spans sector boundaries. Busy polling uses status register
 * bit0 and yields to the RTOS via osDelay() for cooperative multitasking.
 *
 * Safety / Usage Notes:
 *  - Only single SPI mode supported (no dual/quad fast read acceleration here).
 *  - Ensure SPI bus is initialized before calling any API.
 *  - Concurrent access is not protected; serialize calls at a higher layer if
 *    multiple tasks may use the flash simultaneously.
 *  - Erase cycles are finite; avoid frequent small writes that force whole-sector erases.
 *
 * @author Young.W <com.wang@hotmail.com>
 * @date 2025-09-25
 * @version 1.0
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/** W25QXX flash control structure */
typedef struct
{
    /**
     * @brief Flash type (capacity)
     * Possible values:
     *  - W25Q80  : 0xEE13 (Winbond, W25Q80)
     *  - W25Q16  : 0xEE14 (Winbond, W25Q16)
     *  - W25Q32  : 0xEE15 (Winbond, W25Q32)
     *  - W25Q64  : 0xEE16 (Winbond, W25Q64)
     *  - W25Q128 : 0xEE17 (Winbond, W25Q128)
     */
    uint32_t type;

    /**
     * @brief Read the JEDEC ID of the flash device
     * @return 16-bit JEDEC ID (upper byte: manufacturer ID, lower byte: device ID)
     * Returns 0 on failure (e.g., if SPI communication fails)
     * Typical values:
     *  - W25Q80  : 0xEE13 (Winbond, W25Q80)
     *  - W25Q16  : 0xEE14 (Winbond, W25Q16)
     *  - W25Q32  : 0xEE15 (Winbond, W25Q32)
     *  - W25Q64  : 0xEE16 (Winbond, W25Q64)
     *  - W25Q128 : 0xEE17 (Winbond, W25Q128)
     */
    uint16_t (* ReadID)();

    /**
     * @brief Read data from flash memory
     * @param buffer Pointer to the buffer where read data will be stored
     * @param addr 24-bit flash address to read from
     * @param size Number of bytes to read
     * @return true if the operation was successful, false otherwise
     */
    bool (* Read)(uint8_t*, uint32_t, uint32_t);

    /**
     * @brief Write data to flash memory
     * @param buffer Pointer to the buffer containing data to write
     * @param addr 24-bit flash address to write to
     * @param size Number of bytes to write
     * @return true if the operation was successful, false otherwise
     * Note: This function handles necessary sector erasures automatically.
     */
    bool (* Write)(uint8_t*, uint32_t, uint32_t);
} W25QXX_t;

/**
 * @brief Initialize W25QXX flash interface
 * This function initializes the W25QXX flash interface and returns a pointer
 * to the control structure. It must be called before any other operations.
 * @param chipType The type of W25QXX chip to initialize (e.g., W25Q128)
 * @return pointer to W25QXX control structure
 * @note Singleton pattern: only one instance is created and reused.
 */
W25QXX_t* W25QXX_Init(uint32_t chipType);
