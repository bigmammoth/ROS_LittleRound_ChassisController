#pragma once

#include <stdint.h>
#include <stdbool.h>

// PI
#define PI 3.14159265358979323846

/* ----------------------- System Default Configuration Definitions ------------------------- */
#define DEFAULT_LOCAL_UDP_ADDRESS   "192.168.55.100"                // Default IP address
#define DEFAULT_LOCAL_UDP_PORT      12000                           // Default port
#define DEFAULT_CHASSIS_TYPE        CHASSIS_TYPE_DIFF               // Default chassis type
#define DEFAULT_WHEEL_DIAMETER      0.064                           // Default wheel diameter in meters
#define DEFAULT_WHEEL_RADIUS        (DEFAULT_WHEEL_DIAMETER / 2)    // Default wheel radius in meters
#define DEFAULT_WHEEL_PERIMETER     (DEFAULT_WHEEL_DIAMETER * PI)   // Default wheel perimeter in meters
#define DEFAULT_TRACK_WIDTH         0.164                           // Default track width in meters
#define DEFAULT_MAX_VELOCITY        1.0                             // Default maximum linear speed in m/s
#define DEFAULT_MAX_OMEGA           (2.0 * PI)                      // Default maximum angular speed in rad/s
#define DEFAULT_WHEEL_NUMBER        2                               // Default number of wheels
#define DEFAULT_PULSE_PER_REVOL     10000.0f                        // Default pulses per revolution
#define DEFAULT_STATE_FREQUENCY     10.0f                           // Default state feedback frequency in Hz
#define DEFAULT_ODOMETRY_FREQUENCY  20.0f                           // Default odometry feedback frequency in Hz

// Total motor number
#define TOTAL_MOTOR_NUMBER  2

// Receiver configuration
#define RECEIVER_TYPE_WFLY
// #define RECEIVER_TYPE_HT8A

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
#define EXT_FLASH_OTA_FILE_SIZE             0x00400000U // 4MB OTA file
#define EXT_FLASH_PARAMETER_FILE_ADDRESS    0x00400000U
#define EXT_FLASH_PARAMETER_FILE_SIZE       0x00100000U // 1MB parameter
#define EXT_FLASH_LOG_FILE_ADDRESS          0x00500000U
#define EXT_FLASH_LOG_FILE_SIZE             0x00300000U // 3MB log file
#define EXT_FLASH_TOTAL_SIZE                0x00800000U // 8MB total size
#define EXT_FLASH_SECTOR_SIZE               0x00001000U // 4KB sector size
#define EXT_FLASH_PAGE_SIZE                 0x00000100U // 256B page size
