#pragma once

#include <stdint.h>
#include <stdbool.h>

// PI
#define PI 3.14159265358979323846
#define MAX_ANGULAR_VELOCITY (3 * PI)   // rad/s
#define MAX_VELOCITY 1.0                // m/s Determined by the rated speed of the motor MG513P30 293RPM and the wheel diameter of 64mm
#define WHEELS_DISTANCE 0.16            // 180mm
#define WHEEL_DIAMETER 0.064            // 64mm
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2)
#define WHEEL_PERIMETER (WHEEL_DIAMETER * PI)

// Total motor number
#define TOTAL_MOTOR_NUMBER  2

// Network configuration
#define UDP_ROS_LISTEN_PORT 12000   // Port for ROS interface

// Receiver configuration
//#define RECEIVER_TYPE_WFLY
#define RECEIVER_TYPE_HT8A

/* ----------------------- External Flash Definition ------------------------- */
typedef enum {
    EXT_FLASH_NONE = 0,
    EXT_FLASH_W25Q80    = 0xEF13, // Winbond W25Q80 external flash
    EXT_FLASH_W25Q16    = 0xEF14, // Winbond W25Q16 external flash
    EXT_FLASH_W25Q32    = 0xEF15, // Winbond W25Q32 external flash
    EXT_FLASH_W25Q64    = 0xEF16, // Winbond W25Q64 external flash
    EXT_FLASH_W25Q128   = 0xEF17, // Winbond W25Q128 external flash
} ExtFlashType_t;

#define EXT_FLASH_OTA_FILE_ADDRESS          0x00000000U
#define EXT_FLASH_OTA_FILE_SIZE             0x00800000U // 8MB OTA file
#define EXT_FLASH_PARAMETER_FILE_ADDRESS    0x00800000U
#define EXT_FLASH_PARAMETER_FILE_SIZE       0x00100000U // 1MB parameter
#define EXT_FLASH_LOG_FILE_ADDRESS          0x00900000U
#define EXT_FLASH_LOG_FILE_SIZE             0x00700000U // 7MB log file
#define EXT_FLASH_TOTAL_SIZE                0x01000000U // 16MB total size
#define EXT_FLASH_SECTOR_SIZE               0x00001000U // 4KB sector size
#define EXT_FLASH_PAGE_SIZE                 0x00000100U // 256B page size
