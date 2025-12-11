/**
 * @file w25qxx.c
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
 * @dependencies w25qxx.h, spi.h, main.h, CMSIS-RTOS (for osDelay)
 * @author Young.W <com.wang@hotmail.com>
 * @date 2025-09-25
 * @version 1.0
 */
#include "w25qxx.h"

#include "spi.h"
#include "main.h"

/* ------------------------------ Static variables ------------------------------ */
/**
 * @brief W25QXX flash control commands definition
 */
enum _W25QXX_COMMAND
{
    W25QXX_WRITE_ENABLE =       0x06,
    W25QXX_WRITE_DISABLE =      0x04,
    W25QXX_READ_STATUS_REG =    0x05,
    W25QXX_WRITE_STATUS_REG =   0x01,
    W25QXX_READ_DATA =          0x03,
    W25QXX_FAST_READ_DATA =     0x0B,
    W25QXX_FAST_READ_DUAL =     0x3B,
    W25QXX_PAGE_PROGRAM =       0x02,
    W25QXX_BLOCK_ERASE =        0xD8,
    W25QXX_SECTOR_ERASE =       0x20,
    W25QXX_CHIP_ERASE =         0xC7,
    W25QXX_POWER_DOWN =         0xB9,
    W25QXX_RELEASE_POWER_DOWN = 0xAB,
    W25QXX_DEVICE_ID =          0xAB,
    W25QXX_MANUFACT_DEVICE_ID = 0x90,
    W25QXX_JEDEC_DEVICE_ID =    0x9F
};

/**
 * @brief W25QXX flash type definition
 */
enum _W25QXX_TYPE
{
    W25Q80  = 0xEF13,
    W25Q16  = 0xEF14,
    W25Q32  = 0xEF15,
    W25Q64  = 0xEF16,
    W25Q128 = 0xEF17
};

static const uint32_t SECTOR_SIZE = 4096;
static const uint32_t PAGE_SIZE = 256;
static const uint32_t SECTOR_MASK = 0xFFFFF000U;
static const uint32_t PAGE_MASK = 0xFFFFFF00U;
static W25QXX_t w25qxx = {0, 0, 0, 0};
static osMutexId_t w25qxxMutex = NULL;

/* ------------------------------- Static functions --------------------------- */
static bool Read(uint8_t *, uint32_t, uint32_t);
static bool Write(uint8_t *, uint32_t, uint32_t);
static uint8_t ReadSR(void);
static bool WriteSR(uint8_t);
static bool EnableWrite(void);
static bool DisableWrite(void);
static uint16_t ReadID(void);
static void WaitBusyBit(uint32_t);
static bool EraseSector(uint32_t sectorAddr);
static bool ProgramPage(uint8_t *buffer, uint32_t address, uint32_t size);

/**
 * @brief Initialize W25QXX flash interface
 * This function initializes the W25QXX flash interface and returns a pointer
 * to the control structure. It must be called before any other operations.
 * @param chipType The type of W25QXX chip to initialize (e.g., W25Q128)
 * @return pointer to W25QXX control structure
 * @note Singleton pattern: only one instance is created and reused.
 */
W25QXX_t *W25QXX_Init(uint32_t chipType)
{
    if (chipType != W25Q80 && chipType != W25Q16 && chipType != W25Q32 &&
        chipType != W25Q64 && chipType != W25Q128)
        return NULL; // Invalid chip type
    if (!w25qxx.type)
    {
        w25qxx.type = chipType;
        w25qxx.Read = Read;
        w25qxx.Write = Write;
        w25qxx.ReadID = ReadID;
        w25qxxMutex = osMutexNew(NULL);
        assert_param(w25qxxMutex != NULL);
    }
    return &w25qxx;
}

/**
 * @brief Read W25QXX status register
 * BIT7 6    5    4    3    2    1    0
 * SPR  RV   TB   BP2  BP1  BP0  WEL  BUSY
 * SPR: default 0, protection bit, work with WP
 * TB, BP2, BP1, BP0: Flash area write protection
 * WEL: Write eanble lock
 * BUSY: busy flag 1-busy 0-idle
 * default value: 0x00
 * @param none
 * @return status register value
 */
uint8_t ReadSR(void)
{
    uint8_t txBuff[2] = {W25QXX_READ_STATUS_REG, 0xFF};
    uint8_t rxBuff[2];
    osMutexAcquire(w25qxxMutex, osWaitForever);
    SPI_SetChipSelectLow();
    SPI_TransmitReceive(txBuff, rxBuff, 2);
    SPI_SetChipSelectHigh();
    osMutexRelease(w25qxxMutex);
    return rxBuff[1];
}

/**
 * @brief Write W25QXX status register
 * Only SPR, TB, BP2, BP1, BP0(bit 7,5,4,3,2) can be written.
 * @param status status data to be written
 * @return none
 */
bool WriteSR(uint8_t status)
{
    uint8_t txBuff[2] = {W25QXX_WRITE_STATUS_REG, status};
    uint8_t rxBuff[2];
    bool result;
    osMutexAcquire(w25qxxMutex, osWaitForever);
    SPI_SetChipSelectLow();
    result = SPI_TransmitReceive(txBuff, rxBuff, 2);
    SPI_SetChipSelectHigh();
    osMutexRelease(w25qxxMutex);
    return (result && (rxBuff[1] == status));
}

/**
 * @brief Read chip ID
 * @param none
 * @return chip manufacture ID
 *      @arg @ref W25Q80
 *      @arg @ref W25Q16
 *      @arg @ref W25Q32
 *      @arg @ref W25Q64
 *      @arg @ref W25Q128
 */
uint16_t ReadID(void)
{
    uint16_t id = 0;
    uint8_t txBuff[6] = {W25QXX_MANUFACT_DEVICE_ID, 0x00, 0x00, 0x00};
    uint8_t rxBuff[2];
    osMutexAcquire(w25qxxMutex, osWaitForever);
    SPI_SetChipSelectLow();
    SPI_Transmit(txBuff, 4);
    SPI_Receive(rxBuff, 2);
    SPI_SetChipSelectHigh();
    osMutexRelease(w25qxxMutex);
    id = rxBuff[0] << 8 | rxBuff[1];
    return id;
}

/**
 * @brief Read datas from flash
 * This function reads data from the flash memory.
 * @param buffer pointer to the buffer to store the read data
 * @param addr flash address to be read, 24 bits address
 * @param size data length in bytes
 * @return true if the operation was successful, false otherwise
 */
bool Read(uint8_t *buffer, uint32_t addr, uint32_t size)
{
    uint8_t txBuff[4] = {W25QXX_READ_DATA, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr};
    bool result;
    osMutexAcquire(w25qxxMutex, osWaitForever);
    SPI_SetChipSelectLow();
    result = SPI_Transmit(txBuff, sizeof(txBuff));
    result |= SPI_Receive(buffer, size);
    SPI_SetChipSelectHigh();
    osMutexRelease(w25qxxMutex);
    return result;
}

/**
 * @brief Write datas to flash
 * This function writes data to the flash memory, handling necessary sector erasures.
 * @param buffer pointer to the data to be written
 * @param address flash address to be written, 24 bits address
 * @param size data length in bytes
 * @return true if the operation was successful, false otherwise
 */
bool Write(uint8_t *buffer, uint32_t address, uint32_t size)
{
    bool result = true;
    // Erasing, if nessary, sectors
    if (0 == (address & (~SECTOR_MASK)))
    { // Write to a new sector, need to erase it firstly
        result &= EraseSector(address);
    }
    uint32_t endAddress = address + size - 1;
    uint32_t startSectorAddr = (address & SECTOR_MASK) + SECTOR_SIZE;
    uint32_t endSectorAddr = endAddress & SECTOR_MASK;
    while (startSectorAddr <= endSectorAddr)
    { // Erase all needed sectors if the data cross multiple sectors.
        result &= EraseSector(startSectorAddr);
        startSectorAddr += SECTOR_SIZE;
    }

    // Writing data
    uint32_t remainSize = size, writeAddress = address, pageRemainSize;
    uint8_t *readPtr = buffer;
    while (remainSize)
    {
        // Check if this page already has data and calculate its remain size;
        pageRemainSize = PAGE_SIZE - (writeAddress & (~PAGE_MASK));
        uint32_t writeSize = (remainSize < pageRemainSize) ? remainSize : pageRemainSize;
        result &= ProgramPage(readPtr, writeAddress, writeSize);
        remainSize -= writeSize;
        readPtr += writeSize;
        writeAddress += writeSize;
    }
    return result;
}

/**
 * @brief Enable flash write
 * @param none
 * @return true if the operation was successful, false otherwise
 */
bool EnableWrite(void)
{
    uint8_t txBuff[1] = {W25QXX_WRITE_ENABLE};
    uint8_t rxBuff[1];
    bool result;
    osMutexAcquire(w25qxxMutex, osWaitForever);
    SPI_SetChipSelectLow();
    result = SPI_TransmitReceive(txBuff, rxBuff, sizeof(txBuff));
    SPI_SetChipSelectHigh();
    osMutexRelease(w25qxxMutex);
    return result;
}

/**
 * @brief Disable flash write
 * @param none
 * @return true if the operation was successful, false otherwise
 */
bool DisableWrite(void)
{
    uint8_t txBuff[1] = {W25QXX_WRITE_DISABLE};
    uint8_t rxBuff[1];
    bool result;
    osMutexAcquire(w25qxxMutex, osWaitForever);
    SPI_SetChipSelectLow();
    result = SPI_TransmitReceive(txBuff, rxBuff, sizeof(txBuff));
    SPI_SetChipSelectHigh();
    osMutexRelease(w25qxxMutex);
    return result;
}

/** 
 * @brief Wait busy bit clear
 * @param period The time interval to check the busy bit again
 * @return none
 */
void WaitBusyBit(uint32_t period)
{
    while (ReadSR() & 0x01)
        osDelay(period);
}

/**
 * @brief Erase sector
 * This function erases a sector of the flash memory.
 * @param sectorAddr Sector address in 24bits
 * @return true if the operation was successful, false otherwise
 */
bool EraseSector(uint32_t sectorAddr)
{
    const uint32_t BUSY_WAIT_TIME = 5;
    uint8_t txBuff[] = {W25QXX_SECTOR_ERASE, (uint8_t)(sectorAddr >> 16), (uint8_t)(sectorAddr >> 8), (uint8_t)sectorAddr};
    bool result;
    if (!EnableWrite()) return false;
    WaitBusyBit(BUSY_WAIT_TIME);
    osMutexAcquire(w25qxxMutex, osWaitForever);
    SPI_SetChipSelectLow();
    result = SPI_Transmit(txBuff, sizeof(txBuff));
    SPI_SetChipSelectHigh();
    osMutexRelease(w25qxxMutex);
    WaitBusyBit(BUSY_WAIT_TIME);
    return result;
}

/**
 * @brief Program a page, 256 bytes length
 * Write a page, only one page can be written at a time.
 * @param buffer Pointer to the datas to be written.
 * @param address Flash address to be written
 * @param size Data length in bytes
 * @return true if the operation was successful, false otherwise
 */
bool ProgramPage(uint8_t *buffer, uint32_t address, uint32_t size)
{
    const uint32_t BUSY_WAIT_TIME = 1;
    uint8_t txBuff[] = {W25QXX_PAGE_PROGRAM, (uint8_t)(address >> 16), (uint8_t)(address >> 8), (uint8_t)address};
    bool result;
    if (!EnableWrite()) return false;
    osMutexAcquire(w25qxxMutex, osWaitForever);
    SPI_SetChipSelectLow();
    result = SPI_Transmit(txBuff, sizeof(txBuff));
    result |= SPI_Transmit(buffer, size);
    SPI_SetChipSelectHigh();
    osMutexRelease(w25qxxMutex);
    WaitBusyBit(BUSY_WAIT_TIME);
    return result;
}
