/**
 * @file store_file.c
 * @brief Persistent file abstraction backed by external SPI flash.
 *
 * @details Manages a circular storage region on W25Qxx flash, maintaining a
 * sequence of File Description Blocks (FDBs) that track file metadata such as
 * start offset, length, and CRC. Provides APIs to read/write bytes, reset file
 * pointers, compute and retrieve CRC32 values, and iterate to new files within
 * the allocated block. Flash interactions are delegated to the `W25QXX` driver
 * and memory buffering uses the shared mem-pool utilities.
 *
 * Responsibilities:
 *  - Initialize `StoreFile_t` instances and locate the latest valid FDB.
 *  - Persist new metadata records (FDBs) with CRC protection.
 *  - Handle wrap-around when data crosses the end of the storage segment.
 *
 * Flash memory layout overview:
 * @verbatim
 *  Flash block (base = fp->blockPosition, length = fp->blockLength)
 *
 *  ┌────────────────────────────────────────────────────────────────┐
 *  │   offset 0 (base = fp->blockPosition)                          │
 *  │                                                                │
 *  │   File Description Area (FILE_DESCRIPTION_AREA_SIZE bytes)     │
 *  │      ┌──────────┐                                              │
 *  │      │   FDB0   │   First FDB                                  │
 *  │      ├──────────┤                                              │
 *  │      │   …      │                                              │
 *  │      ├──────────┤                                              │
 *  │ ┌────│   FDBn   │◄─ fp->fdbPos (offset of latest valid FDB)    │
 *  │ |    └──────────┘                                              │
 *  │ |                                                              │
 *  │ | Data Area (fp->blockLength - FILE_DESCRIPTION_AREA_SIZE)     │
 *  │ | ┌─────────────────────────────────────────────────────────┐  │
 *  │ | │  Logical files stored in FDB sequence (oldest → latest) │  │
 *  │ | │  ┌──────────┐                                           │  │
 *  │ | │  │  File 0  │                                           │  │
 *  │ | │  ├──────────┤                                           │  │
 *  │ | │  │   …      │                                           │  │
 *  │ | │  ├──────────┤                                           │  │
 *  │ └───►│  File n  │◄─ fdb->filePos                            │  │
 *  │   │  └──────────┘                                           │  │
 *  │   │                                                         │  │
 *  │   │  Logical read pointer:                                  │  │
 *  │   │    fp->blockPosition + FILE_DESCRIPTION_AREA_SIZE       │  │
 *  │   │    + fp->readPos                                        │  │
 *  │   │                                                         │  │
 *  │   │  Logical write pointer:                                 │  │
 *  │   │    fp->blockPosition + FILE_DESCRIPTION_AREA_SIZE       │  │
 *  │   │    + fp->writePos                                       │  │
 *  │   │                                                         │  │
 *  │   └─────────────────────────────────────────────────────────┘  │
 *  │                                                                │
 *  │   offset fp->blockLength                                       │
 *  └────────────────────────────────────────────────────────────────┘
 *
 *  Summary:
 *    • fp->filePos selects the start of the latest logical file (FDBn) inside the
 *      data area. Dereferencing `base + fp->filePos` yields the file pointer for
 *      that entry, while earlier FDB indices map to preceding segments.
 *    • fp->readPos and fp->writePos are offsets from the data area base used by
 *      Read/Write helpers; they can advance past fp->filePos as the file grows.
 * @endverbatim
 *
 * Example usage:
 * @code{.c}
 * StoreFile_t store;
 * // Initialize the store file instance
 * StoreFile_Init(&store, EXT_FLASH_PARAMETER_FILE_ADDRESS, EXT_FLASH_PARAMETER_FILE_SIZE);
 * 
 * // Write data to the file
 * const uint8_t payload[] = {0x01, 0x02, 0x03};
 * store.Write(&store, payload, sizeof(payload));
 * 
 * // Update the file description block in flash
 * store.UpdateFileDescription(&store);
 * store.SetReadPos(&store, 0);
 * uint8_t buffer[sizeof(payload)];
 * 
 * // Read data back from the file
 * store.Read(&store, buffer, sizeof(buffer));
 * uint32_t crc = store.CalculateCRC(&store);
 * 
 * // Create a new file (FDB) in the storage region
 * store.NewFile(&store);
 * 
 * // Write more data to the new file
 * const uint8_t newPayload[] = {0x04, 0x05, 0x06};
 * store.Write(&store, newPayload, sizeof(newPayload));
 * 
 * // Update the file description block for the new file
 * store.UpdateFileDescription(&store);
 * 
 * // Read back the new data
 * store.SetReadPos(&store, 0);
 * uint8_t newBuffer[sizeof(newPayload)];
 * store.Read(&store, newBuffer, sizeof(newBuffer));
 * uint32_t newCrc = store.CalculateCRC(&store);
 *
 * @endcode
 *
 * @note All write operations assume exclusive access to the underlying flash
 * sector; callers should serialize usage across tasks.
 *
 * @dependencies store_file.h, w25qxx.h, crc32.h, mem_pool.h, system_config.h
 * @date 2025-09-27
 * @author Young.W <com.wang@hotmail.com>
 * @version 1.0
 */

#include "store_file.h"

#include "crc32.h"
#include "w25qxx.h"
#include "mem_pool.h"
#include "system_config.h"

#include <string.h>

/* -------------------------------------- Data Type Definitions -------------------------------- */
#define FILE_DESCRIPTION_BLOCK_HEADER 0xA5A55A5A    // File description block header
typedef struct {
    uint32_t fdbHeader;     // File description block header, should be 0xA5A55A5A
    uint32_t filePos;       // Start position of the file in the flash memory
    uint32_t length;        // Length of the file
    uint32_t fileCRC;       // CRC32 checksum of the file content
    uint32_t fdbCRC;
} FileDescriptionBlock_t;
#define FILE_DESCRIPTION_AREA_SIZE (1 * EXT_FLASH_SECTOR_SIZE) // 4KB for file description blocks

/* -------------------------------------- Static variables ------------------------------------- */
static W25QXX_t *w25qxx = NULL;     // W25QXX flash instance

/* -------------------------------------- Static functions ------------------------------------- */
static bool Write(const void*, const void *data, uint32_t size);
static int32_t Read(const void*, void *data, uint32_t size);
static void SetWritePos(const void*, uint32_t pos);
static void SetReadPos(const void*, uint32_t pos);
static uint32_t CalculateCRC(const void*);
static uint32_t ReadCRC(const void*);
static bool UpdateFileDescription(const void*);
static bool FindOutFileDescriptionBlock(const void* file, FileDescriptionBlock_t* fdb);
static bool NewFile(const void* file);

/**
 * @brief Initialize the update file storage structure
 * This function initializes the update file storage structure and its function pointers.
 * @param file pointer to the StoreFile_t structure to initialize
 * @param memoryPosition start position of the file in the flash memory
 * @param memoryLength length of the file in the flash memory
 * @return true if initialization was successful, false otherwise
 */
bool StoreFile_Init(StoreFile_t* file, uint32_t memoryPosition, uint32_t memoryLength)
{
    if (file == NULL) return false;
    w25qxx = W25QXX_Init(EXT_FLASH_W25Q128);
    if (w25qxx == NULL) return false;
    file->Read = Read;
    file->Write = Write;
    file->CalculateCRC = CalculateCRC;
    file->SetWritePos = SetWritePos;
    file->SetReadPos = SetReadPos;
    file->ReadCRC = ReadCRC;
    file->UpdateFileDescription = UpdateFileDescription;
    file->NewFile = NewFile;

    file->blockPosition = memoryPosition;
    file->blockLength = memoryLength;
    file->readPos = 0;
    file->writePos = 0;
    
    FileDescriptionBlock_t fdb;
    if (!FindOutFileDescriptionBlock(file, &fdb))
    {
        file->fdbPos = 0;
        file->filePos = FILE_DESCRIPTION_AREA_SIZE;
        file->crc = 0;
        file->length = 0;
    }
    else
    {
        file->filePos = fdb.filePos;
        file->crc = fdb.fileCRC;
        file->length = fdb.length;
    }
    return true;
}

/**
 * @brief Write data to the file
 * This function writes data to the file at the current write position.
 * @param file pointer to the StoreFile_t structure
 * @param data pointer to the data to be written
 * @param size number of bytes to write
 * @return true if the write operation was successful, false otherwise
 * @note This function does not update the file length or CRC32.
 */
static bool Write(const void* file, const void* data, uint32_t size)
{
    if (file == NULL || data == NULL || size == 0) return false;
    StoreFile_t* fp = (StoreFile_t*)file;
    if (fp->writePos + size > fp->blockLength - FILE_DESCRIPTION_AREA_SIZE)
    {
        uint32_t writeSize = fp->blockLength - FILE_DESCRIPTION_AREA_SIZE - fp->writePos;
        uint32_t remainingSize = size - writeSize;
        w25qxx->Write((uint8_t*)data, fp->blockPosition + FILE_DESCRIPTION_AREA_SIZE + fp->writePos, writeSize);
        fp->writePos = 0;
        w25qxx->Write((uint8_t*)data + writeSize, fp->blockPosition + FILE_DESCRIPTION_AREA_SIZE + fp->writePos, remainingSize);
        fp->writePos += remainingSize;
    }
    else
    {
        w25qxx->Write((uint8_t*)data, fp->blockPosition + FILE_DESCRIPTION_AREA_SIZE + fp->writePos, size);
        fp->writePos += size;
    }
    fp->length = fp->writePos;
    return true;
}

/**
 * @brief Read data from the file
 * This function reads data from the file at the current read position.
 * @param file pointer to the StoreFile_t structure
 * @param buffer pointer to the buffer to store the read data
 * @param size number of bytes to read
 * @return number of bytes actually read, or -1 on error
 */
static int32_t Read(const void* file, void* buffer, uint32_t size)
{
    if (file == NULL || buffer == NULL || size == 0) return -1;
    StoreFile_t* fp = (StoreFile_t*)file;
    if (fp->readPos >= fp->length) return 0;
    if (fp->readPos + size > fp->length) size = fp->length - fp->readPos;
    if (fp->readPos + size > fp->blockLength - FILE_DESCRIPTION_AREA_SIZE)
    {
        uint32_t readSize = fp->blockLength - FILE_DESCRIPTION_AREA_SIZE - fp->readPos;
        uint32_t remainingSize = size - readSize;
        w25qxx->Read(buffer, fp->blockPosition + FILE_DESCRIPTION_AREA_SIZE + fp->readPos, readSize);
        fp->readPos = 0;
        w25qxx->Read((uint8_t*)buffer + readSize, fp->blockPosition + FILE_DESCRIPTION_AREA_SIZE + fp->readPos, remainingSize);
        fp->readPos += remainingSize;
    }
    else
    {
        w25qxx->Read(buffer, fp->blockPosition + FILE_DESCRIPTION_AREA_SIZE + fp->readPos, size);
        fp->readPos += size;
    }
    return size;
}

/**
 * @brief Calculate the CRC32 of the file content
 * This function calculates the CRC32 checksum of the file content.
 * @return CRC32 checksum of the file content
 */
static uint32_t CalculateCRC(const void* file)
{
    if (file == NULL) return 0;
    const uint32_t MEM_BLOCK_SIZE = 2048;
    uint8_t* buff = MemPool_Alloc(MEM_BLOCK_SIZE);
    if (buff == NULL) return 0;

    StoreFile_t* fp = (StoreFile_t*)file;
    uint32_t originalReadPos = fp->readPos;
    uint32_t crc = 0xFFFFFFFF;
    fp->readPos = 0;
    uint32_t remaining = fp->length;

    while (remaining > 0)
    {
        uint32_t chunk = (remaining < MEM_BLOCK_SIZE) ? remaining : MEM_BLOCK_SIZE;
        int32_t readCount = Read(file, buff, chunk);
        if (readCount <= 0)
        {
            crc = 0;
            break;
        }
        crc = Crc32(crc, buff, (uint32_t)readCount);
        remaining -= (uint32_t)readCount;
    }

    MemPool_Free(buff);
    fp->readPos = originalReadPos;
    return crc;
}

/**
 * @brief Read the CRC32 of the file content
 * This function returns the stored CRC32 checksum of the file content.
 * @return CRC32 checksum of the file content
 */
static uint32_t ReadCRC(const void* file)
{
    if (file == NULL) return 0;
    return ((StoreFile_t*)file)->crc;
}

/**
 * @brief Set the write position in the file
 * This function sets the current write position in the file.
 * @param pos new write position
 */
static void SetWritePos(const void* file, uint32_t pos)
{
    if (file == NULL) return;
    ((StoreFile_t*)file)->writePos = pos;
}

/**
 * @brief Set the read position in the file
 * This function sets the current read position in the file.
 * @param pos new read position
 */
static void SetReadPos(const void* file, uint32_t pos)
{
    if (file == NULL || pos > ((StoreFile_t*)file)->length) return;
    ((StoreFile_t*)file)->readPos = pos;
}

/**
 * @brief Update the file description block in flash
 * This function updates the file description block in flash memory
 * with the current length and CRC32 of the file.
 * @param file pointer to the StoreFile_t structure
 * @return true if the update was successful, false otherwise
 */
static bool UpdateFileDescription(const void* file)
{
    if (file == NULL) return false;
    bool result;
    StoreFile_t* fp = (StoreFile_t*)file;
    FileDescriptionBlock_t fdb;
    fdb.fdbHeader = FILE_DESCRIPTION_BLOCK_HEADER;
    fdb.filePos = fp->filePos;
    fdb.length = fp->length;

    fp->crc = CalculateCRC(fp);
    fdb.fileCRC = fp->crc;
    
    // Compute CRC over all fields except the trailing fdbCRC itself
    fdb.fdbCRC = Crc32(0xFFFFFFFF, (uint8_t*)&fdb, sizeof(FileDescriptionBlock_t) - sizeof(uint32_t));
    result = w25qxx->Write((uint8_t*)&fdb, fp->blockPosition + fp->fdbPos, sizeof(FileDescriptionBlock_t));
    return result;
}

/**
 * @brief Find out the file description block in flash
 * This function searches for the file description block in flash memory
 * and updates the file structure with the found positions.
 * @param file pointer to the StoreFile_t structure
 * @param fdb pointer to a FileDescriptionBlock_t structure to store the found block
 * @return true if a valid file description block was found, false otherwise
 */
static bool FindOutFileDescriptionBlock(const void* file, FileDescriptionBlock_t* fdb)
{
    if (fdb == NULL || file == NULL) return false;
    StoreFile_t* fp = (StoreFile_t*)file;
    uint32_t slots = FILE_DESCRIPTION_AREA_SIZE / sizeof(FileDescriptionBlock_t);

    // If the very first slot is invalid, nothing to find
    FileDescriptionBlock_t tmp;
    w25qxx->Read((uint8_t*)&tmp, fp->blockPosition, sizeof(tmp));
    if (tmp.fdbHeader != FILE_DESCRIPTION_BLOCK_HEADER) return false;

    // Binary search the first invalid slot in [0, slots)
    uint32_t lo = 0, hi = slots;
    while (lo < hi) {
        uint32_t mid = lo + ((hi - lo) >> 1);
        uint32_t addr = fp->blockPosition + mid * sizeof(FileDescriptionBlock_t);
        w25qxx->Read((uint8_t*)&tmp, addr, sizeof(tmp));

        if (tmp.fdbHeader == FILE_DESCRIPTION_BLOCK_HEADER) {
            lo = mid + 1;  // valid at mid, search right
        } else {
            hi = mid;      // invalid at mid, search left
        }
    }

    if (lo == 0) return false; // No valid FDB found
    uint32_t lastIndex = lo - 1;
    uint32_t lastAddr = fp->blockPosition + lastIndex * sizeof(FileDescriptionBlock_t);

    // Read the last valid FDB entry found into the output buffer
    w25qxx->Read((uint8_t*)fdb, lastAddr, sizeof(FileDescriptionBlock_t));
    // Check if we found a valid FDB
    uint32_t crc = Crc32(0xFFFFFFFF, (uint8_t*)fdb, sizeof(FileDescriptionBlock_t) - sizeof(uint32_t));
    if (fdb->fdbHeader != FILE_DESCRIPTION_BLOCK_HEADER || fdb->fdbCRC != crc)
        return false; // invalid FDB
    
    fp->fdbPos = lastIndex * sizeof(FileDescriptionBlock_t);
    return true;
}

/**
 * @brief Create a new file
 * This function creates a new file by updating the file description block position
 * and resetting the file parameters.
 * @param file pointer to the StoreFile_t structure
 * @return true if the new file was created successfully, false otherwise
 */
bool NewFile(const void* file)
{
    if (file == NULL) return false;
    StoreFile_t* fp = (StoreFile_t*)file;
    fp->fdbPos += sizeof(FileDescriptionBlock_t);
    if (fp->fdbPos + sizeof(FileDescriptionBlock_t) >= FILE_DESCRIPTION_AREA_SIZE)
        fp->fdbPos = 0; // Reset to the beginning if exceeding the area size
    fp->filePos += fp->length;
    if (fp->filePos > fp->blockLength - FILE_DESCRIPTION_AREA_SIZE)
        fp->filePos -= (fp->blockLength - FILE_DESCRIPTION_AREA_SIZE);  // Wrap around
    fp->crc = 0;
    fp->length = 0;
    fp->readPos = 0;
    fp->writePos = 0;
    return true;
}

