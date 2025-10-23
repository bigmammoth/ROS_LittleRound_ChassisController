/**
 * @file store_file.h
 * @brief Interface definition for persistent files on external SPI flash.
 *
 * @details Declares the `StoreFile_t` control structure and initialization
 * helpers that back the implementation in `store_file.c`. The interface exposes
 * metadata fields and function pointers required to operate flash-resident
 * files with File Description Blocks (FDBs), CRC protection, and read/write
 * cursors.
 *
 * Responsibilities:
 *  - Describe the persistent metadata tracked for each logical file instance.
 *  - Publish function pointers for reading, writing, and maintaining CRC/FDB state.
 *  - Provide the initialization API used to bind a storage region to the
 *    persistence helper.
 *
 * @date 2025-09-27
 * @author Young.W <com.wang@hotmail.com>
 * @copyright All right reserved. Hinton Robotics
 * @version 1.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Structure for file storage operations
 */
typedef struct {
    uint32_t blockPosition; // Start position of the file block in the flash memory
    uint32_t blockLength;   // Length of the file block in the flash memory
    uint32_t fdbPos;        // Start position of the file description block in the flash memory
    uint32_t filePos;       // Start position of the file in the flash memory
    uint32_t length;        // Current length of the file
    uint32_t crc;           // CRC32 checksum of the file content
    uint32_t readPos;       // Current read position in the file
    uint32_t writePos;      // Current write position in the file

    /**
     * @brief Read data from the file
     * This function reads data from the file at the current read position.
     * @param file pointer to the StoreFile_t structure
     * @param buffer pointer to the buffer to store the read data
     * @param size number of bytes to read
     * @return number of bytes actually read, or -1 on error
     */
    int32_t (*Read)(const void*, void*, uint32_t);
    
    /**
     * @brief Write data to the file
     * This function writes data to the file at the current write position.
     * @param file pointer to the StoreFile_t structure
     * @param data pointer to the data to be written
     * @param size number of bytes to write
     * @return true if the write operation was successful, false otherwise
     * @note This function does not update the file length or CRC32.
     */
    bool (*Write)(const void*, const void*, uint32_t);

    /** 
     * @brief Function pointer to get the CRC32 of the file content
     * @return CRC32 checksum of the file content
     */
    void (*SetWritePos)(const void*, uint32_t);

    /** 
     * @brief Function pointer to set the read position in the file
     * @param pos new read position
     */
    void (*SetReadPos)(const void*, uint32_t);

    /** 
     * @brief Function pointer to calculate the CRC32 of the file content
     * @return CRC32 checksum of the file content
     */
    uint32_t (*CalculateCRC)(const void*);

    /** 
     * @brief Function pointer to read the CRC32 of the file content
     * @return CRC32 checksum of the file content
     */
    uint32_t (*ReadCRC)(const void*);

    /** 
     * @brief Function pointer to update the file description block in flash
     * @param file pointer to the StoreFile_t structure
     * @return true if the update was successful, false otherwise
     */
    bool (*UpdateFileDescription)(const void*);

    /** 
     * @brief Function pointer to create a new file
     * @param file pointer to the StoreFile_t structure
     * @return true if the new file was created successfully, false otherwise
     */
    bool (*NewFile)(const void*);
} StoreFile_t;

/**
 * @brief Initialize the update file storage structure
 * This function initializes the update file storage structure and its function pointers.
 * @param file pointer to the StoreFile_t structure to initialize
 * @param memoryPosition start position of the file in the flash memory
 * @param memoryLength length of the file in the flash memory
 * @return true if initialization was successful, false otherwise
 */
bool StoreFile_Init(StoreFile_t *file, uint32_t memoryPosition, uint32_t memoryLength);
