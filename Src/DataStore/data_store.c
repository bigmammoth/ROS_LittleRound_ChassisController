/**
 * @file data_store.c
 * @brief Data Store Module Implementation
 * @version 1.0
 * @date 2025-07-22
 * 
 * @details This module provides a centralized, thread-safe data storage system
 * for the chassis controller. It manages configuration parameters including:
 * 
 * - Network configuration (UDP IP address and port)
 * - Motor type and specifications
 * - Battery type and characteristics
 * - Vehicle physical parameters (wheels, dimensions, speed limits)
 * 
 * The module uses RTOS mutexes to ensure thread-safe access to all stored data,
 * making it suitable for use in multi-threaded environments. All data is stored
 * in RAM and initialized with default values from system_config.h.
 * 
 * @section Features
 * - Thread-safe getter/setter functions for all parameters
 * - Centralized configuration management
 * - Default value initialization from system configuration
 * - Support for vehicle physical characteristics
 * - Network parameter management for UDP communication
 * 
 * @section Data_Categories
 * The data store manages several categories of information:
 * 
 * @subsection Network_Config Network Configuration
 * - Local UDP IP address (IPv4)
 * - Local UDP port number
 * 
 * @subsection Hardware_Config Hardware Configuration
 * - Motor type (enumerated type)
 * - Battery type (enumerated type)
 * 
 * @subsection Vehicle_Config Vehicle Physical Parameters
 * - Number of wheels
 * - Wheel diameter (meters)
 * - Wheel base distance (meters)
 * - Track width (meters)
 * - Maximum speed (m/s)
 * 
 * @section Usage
 * 1. Call DataStore_Init() during system initialization
 * 2. Use getter functions to retrieve configuration values
 * 3. Use setter functions to update configuration values
 * 4. All operations are automatically thread-safe
 * 
 * @note All floating-point values use meters as the unit of measurement
 * @note The module requires system_config.h for default value definitions
 * @warning Always call DataStore_Init() before using any other functions
 * 
 * @author Young.R <com.wang@hotmail.com>
 * @version 1.0
 * 
 * @see system_config.h for default value definitions
 */

#include "data_store.h"

#include "main.h"
#include "crc32.h"
#include "store_file.h"
#include "system_config.h"

#include "rl_net.h" // For netIP_aton

#define EVENT_FLAG_DATA_STORE_MODIFIED 0x01 // Event flag for data store modification

/* ------------------ Data type declaration --------------------*/
typedef struct ipAddress {
    uint32_t ipv4; // IPv4 address
    uint16_t port; // Port number
} ipAddress_t;

typedef struct {
    float pulsePerRevolution; // Motor pulses per revolution
    float maxRpm;             // Maximum revolutions per minute
    float gearRatio;          // Gear ratio
} MotorParameters_t;

/* ------------------ Static variables definition --------------------*/
static struct {
    MotorParameters_t motorParams;  // Motor parameters
    ipAddress_t localUdpAddress;    // UDP server address
    float wheelRadius;
    float trackWidth;
    float stateFeedbackFrequency;
    float odometryFeedbackFrequency;
    float maxLinearAcceleration;
    float maxAngularAcceleration;
    float maxVelocity;
    float maxOmega;
} dataStore;

static osMutexId_t dataStoreMutex;
static osEventFlagsId_t dataStoreEventFlags;
static osThreadId_t threadIdDataStore;
static const osThreadAttr_t threadAttrDataStore = {
    .name = "ThreadDataStore",
    .stack_size = 1024,
    .priority = osPriorityBelowNormal,
};
static StoreFile_t paramFile;   // Pointer to parameter file interface

/* ------------------ Static functions declaration --------------------*/
static void DataStoreThread(void* arg);
static void SaveDataToFile(void);
static bool ReadDataFromFile(void);

/**
 * @brief Initialize the Data Store module.
 * This function sets up the data store with default values and initializes
 * the necessary resources for thread-safe access. It must be called before 
 * any other data store functions are used.
 */
void DataStore_Init(void)
{
    // Create mutex and event flags
    dataStoreMutex = osMutexNew(NULL);
    assert_param(dataStoreMutex != NULL);
    // Create event flags for data store modification notifications
    dataStoreEventFlags = osEventFlagsNew(NULL);
    assert_param(dataStoreEventFlags != NULL);
    // Create the data store thread
    threadIdDataStore = osThreadNew(DataStoreThread, NULL, &threadAttrDataStore);
    assert_param(threadIdDataStore != NULL);
    // Get the parameter file interface instance
    bool result = StoreFile_Init(&paramFile, EXT_FLASH_PARAMETER_FILE_ADDRESS, EXT_FLASH_PARAMETER_FILE_SIZE);
    assert_param(result);
    // Load existing parameters from the file, or initialize with defaults
    if (!ReadDataFromFile())
    {
        uint32_t defaultIP;
        netIP_aton(DEFAULT_LOCAL_UDP_ADDRESS, NET_ADDR_IP4, (uint8_t*)&defaultIP);
        dataStore.localUdpAddress.ipv4 = defaultIP;                 // Default IP address, declared in system_config.h
        dataStore.localUdpAddress.port = DEFAULT_LOCAL_UDP_PORT;    // Default port, declared in system_config.h
        dataStore.wheelRadius = DEFAULT_WHEEL_DIAMETER/2;           // Default wheel radius in meters, declared in system_config.h
        dataStore.trackWidth = DEFAULT_TRACK_WIDTH;                 // Default track width in meters, declared in system_config.h
        dataStore.motorParams.pulsePerRevolution = DEFAULT_PULSE_PER_REVOL; // Default pulses per revolution, declared in system_config.h
        dataStore.maxVelocity = DEFAULT_MAX_VELOCITY;               // Default maximum linear speed in m/s, declared in system_config.h
        dataStore.maxOmega = DEFAULT_MAX_OMEGA;                     // Default maximum angular speed in rad/s, declared in system_config.h
    }
}

/**
 * @brief Data Store Thread
 * This thread waits for modification events and saves the data store to the
 * parameter file when changes are detected.
 * @param arg pointer to argument (not used)
 */
void DataStoreThread(void* arg)
{
    (void)arg; // Suppress unused parameter warning
    for (;;)
    {
        osEventFlagsWait(dataStoreEventFlags, EVENT_FLAG_DATA_STORE_MODIFIED, osFlagsWaitAny, osWaitForever);
        uint32_t paramCRC, fileCRC;
        osMutexAcquire(dataStoreMutex, osWaitForever);
        paramCRC = Crc32(CRC32_INITIAL_VALUE, &dataStore, sizeof(dataStore));
        osMutexRelease(dataStoreMutex);
        fileCRC = paramFile.CalculateCRC(&paramFile);
        if (paramCRC != fileCRC) SaveDataToFile();
    }
}

/**
 * @brief Save the current data store to the parameter file.
 * This function writes the current state of the data store to the
 * parameter file in a thread-safe manner. It is called automatically
 * by the data store thread when a modification event is detected.
 */
void SaveDataToFile(void)
{
    paramFile.NewFile(&paramFile);
    osMutexAcquire(dataStoreMutex, osWaitForever);
    paramFile.Write(&paramFile, &dataStore, sizeof(dataStore));
    osMutexRelease(dataStoreMutex);
    paramFile.UpdateFileDescription(&paramFile);
}

/**
 * @brief Read the data store from the parameter file.
 * This function reads the data store from the parameter file in a
 * thread-safe manner. It verifies the integrity of the data using CRC32.
 * @return true if the data was read successfully and passed the CRC check,
 *         false otherwise.
 */
bool ReadDataFromFile(void)
{
    paramFile.SetReadPos(&paramFile, 0);
    osMutexAcquire(dataStoreMutex, osWaitForever);
    
    bool result = paramFile.Read(&paramFile, &dataStore, sizeof(dataStore));
    if (!result)
    {
        osMutexRelease(dataStoreMutex);
        return false;
    }

    uint32_t crc = Crc32(CRC32_INITIAL_VALUE, &dataStore, sizeof(dataStore));
    osMutexRelease(dataStoreMutex);
    uint32_t fileCrc = paramFile.ReadCRC(&paramFile);
    paramFile.SetReadPos(&paramFile, 0);
    return fileCrc == crc;
}

/**
 * @brief Notify that the data store has been modified.
 * This function sets an event flag to indicate that the data store has been modified.
 * It can be called after any setter function to notify other threads of the change.
 */
void DataStore_SaveDataIfModified(void)
{
    osEventFlagsSet(dataStoreEventFlags, EVENT_FLAG_DATA_STORE_MODIFIED);
}

/**
 * @brief Get state feedback frequency.
 * This function retrieves the state feedback frequency from the data store.
 * @return float The state feedback frequency.
 */
float DataStore_GetStateFeedbackFrequency(void)
{
    float freq;
    osMutexAcquire(dataStoreMutex, osWaitForever);
    freq = dataStore.stateFeedbackFrequency;
    osMutexRelease(dataStoreMutex);
    return freq;
}

/**
 * @brief Set state feedback frequency.
 * This function updates the state feedback frequency in the data store.
 * @param frequency The new state feedback frequency.
 */
void DataStore_SetStateFeedbackFrequency(float frequency)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    dataStore.stateFeedbackFrequency = frequency;
    osMutexRelease(dataStoreMutex);
}

/**
 * @brief Set odometry feedback frequency.
 * This function updates the odometry feedback frequency in the data store.
 * @param frequency The new odometry feedback frequency.
 */
void DataStore_SetOdometryFeedbackFrequency(float frequency)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    dataStore.odometryFeedbackFrequency = frequency;
    osMutexRelease(dataStoreMutex);
}

/**
 * @brief Get odometry feedback frequency.
 * This function retrieves the odometry feedback frequency from the data store.
 * @return float The odometry feedback frequency.
 */
float DataStore_GetOdometryFeedbackFrequency(void)
{
    float freq;
    osMutexAcquire(dataStoreMutex, osWaitForever);
    freq = dataStore.odometryFeedbackFrequency;
    osMutexRelease(dataStoreMutex);
    return freq;
}

/**
 * @brief Get pulses per revolution of the motor.
 * This function retrieves the motor's pulses per revolution from the data store.
*/
float DataStore_GetMotorParamPulsePerRevolution(void)
{
    float pulses;
    osMutexAcquire(dataStoreMutex, osWaitForever);
    pulses = dataStore.motorParams.pulsePerRevolution;
    osMutexRelease(dataStoreMutex);
    return pulses;
}

/**
 * @brief Get gear ratio of the motor.
 * This function retrieves the motor's gear ratio from the data store.
*/
float DataStore_GetMotorParamGearRatio(void)
{
    float ratio;
    osMutexAcquire(dataStoreMutex, osWaitForever);
    ratio = dataStore.motorParams.gearRatio;
    osMutexRelease(dataStoreMutex);
    return ratio;
}

/**
 * @brief Get the maximum RPM of the motor.
 * This function retrieves the motor's maximum revolutions per minute from the data store.
 */
float DataStore_GetMotorParamMaxRpm(void)
{
    float rpm;
    osMutexAcquire(dataStoreMutex, osWaitForever);
    rpm = dataStore.motorParams.maxRpm;
    osMutexRelease(dataStoreMutex);
    return rpm;
}

/**
 * @brief Set pulses per revolution of the motor.
 * This function updates the motor's pulses per revolution in the data store.
 */
void DataStore_SetMotorParamPulsePerRevolution(float pulses)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    dataStore.motorParams.pulsePerRevolution = pulses;
    osMutexRelease(dataStoreMutex);
}

/**
 * @brief Set gear ratio of the motor.
 * This function updates the motor's gear ratio in the data store.
 */
void DataStore_SetMotorParamGearRatio(float ratio)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    dataStore.motorParams.gearRatio = ratio;
    osMutexRelease(dataStoreMutex);
}

/**
 * @brief Set the maximum RPM of the motor.
 * This function updates the motor's maximum revolutions per minute in the data store.
 */
void DataStore_SetMotorParamMaxRpm(float rpm)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    dataStore.motorParams.maxRpm = rpm;
    osMutexRelease(dataStoreMutex);
}

/**
 * @brief Get the local UDP IP address.
 * This function retrieves the current local UDP IP address from the data store.
 * @param None
 * @return uint32_t The local UDP IP address.
 */
uint32_t DataStore_GetLocalIpAddress(void)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    uint32_t ip = dataStore.localUdpAddress.ipv4;
    osMutexRelease(dataStoreMutex);
    return ip;
}

/**
 * @brief Get the local UDP port.
 * This function retrieves the current local UDP port from the data store.
 * @param None
 * @return uint16_t The local UDP port.
 */
uint16_t DataStore_GetLocalUdpPort(void)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    uint16_t port = dataStore.localUdpAddress.port;
    osMutexRelease(dataStoreMutex);
    return port;
}

/**
 * @brief Set the local UDP IP address.
 * This function updates the local UDP IP address in the data store.
 * @param ip The new local UDP IP address.
 */
void DataStore_SetLocalUdpAddress(uint32_t ip)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    dataStore.localUdpAddress.ipv4 = ip;
    osMutexRelease(dataStoreMutex);
}

/**
 * @brief Set the local UDP port.
 * This function updates the local UDP port in the data store.
 * @param port The new local UDP port.
 */
void DataStore_SetLocalUdpPort(uint16_t port)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    dataStore.localUdpAddress.port = port;
    osMutexRelease(dataStoreMutex);
}

/**
 * @brief Get the wheel diameter.
 * This function retrieves the current wheel diameter from the data store.
 * @param None
 * @return float The wheel diameter in meters.
 */
float DataStore_GetWheelRadius(void)
{
    float radius;
    osMutexAcquire(dataStoreMutex, osWaitForever);
    radius = dataStore.wheelRadius;
    osMutexRelease(dataStoreMutex);
    return radius;
}

/**
 * @brief Set the wheel radius.
 * This function updates the wheel radius in the data store.
 * @param radius The new wheel radius in meters.
 */
void DataStore_SetWheelRadius(float radius)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    dataStore.wheelRadius = radius;
    osMutexRelease(dataStoreMutex);
}

/**
 * @brief Get the track width.
 * This function retrieves the current track width from the data store.
 * @param None
 * @return float The track width in meters.
 */
float DataStore_GetTrackWidth(void)
{
    float width;
    osMutexAcquire(dataStoreMutex, osWaitForever);
    width = dataStore.trackWidth;
    osMutexRelease(dataStoreMutex);
    return width;
}

/**
 * @brief Set the track width.
 * This function updates the wheel base in the data store.
 * @param base The new track width in meters.
 */
void DataStore_SetTrackWidth(float width)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    dataStore.trackWidth = width;
    osMutexRelease(dataStoreMutex);
}

/**
 * @brief Get the maximum linear velocity (canonical).
 * This function retrieves the stored maximum velocity used across modules.
 */
float DataStore_GetMaxVelocity(void)
{
    float velocity;
    osMutexAcquire(dataStoreMutex, osWaitForever);
    velocity = dataStore.maxVelocity;
    osMutexRelease(dataStoreMutex);
    return velocity;
}

/**
 * @brief Set the maximum linear velocity (canonical).
 * This function stores the maximum velocity used across modules.
 */
void DataStore_SetMaxVelocity(float velocity)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    dataStore.maxVelocity = velocity;
    osMutexRelease(dataStoreMutex);
}

/**
 * @brief Get the maximum angular velocity (canonical).
 * This function retrieves the stored maximum angular velocity used across modules.
 */
float DataStore_GetMaxOmega(void)
{
    float omega;
    osMutexAcquire(dataStoreMutex, osWaitForever);
    omega = dataStore.maxOmega;
    osMutexRelease(dataStoreMutex);
    return omega;
}

/**
 * @brief Set the maximum angular velocity (canonical).
 * This function stores the maximum angular velocity used across modules.
 */
void DataStore_SetMaxOmega(float omega)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    dataStore.maxOmega = omega;
    osMutexRelease(dataStoreMutex);
}

/**
 * @brief Get the maximum linear acceleration.
 * This function retrieves the current maximum linear acceleration from the data store.
 * @param None
 * @return float The maximum linear acceleration in meters per second squared.
 */
float DataStore_GetMaxLinearAcceleration(void)
{
    float acceleration;
    osMutexAcquire(dataStoreMutex, osWaitForever);
    acceleration = dataStore.maxLinearAcceleration;
    osMutexRelease(dataStoreMutex);
    return acceleration;
}

/**
 * @brief Set the maximum linear acceleration.
 * This function updates the maximum linear acceleration in the data store.
 * @param acceleration The new maximum linear acceleration in meters per second squared.
 */
void DataStore_SetMaxLinearAcceleration(float acceleration)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    dataStore.maxLinearAcceleration = acceleration;
    osMutexRelease(dataStoreMutex);
}

/**
 * @brief Get the maximum angular acceleration.
 * This function retrieves the current maximum angular acceleration from the data store.
 * @param None
 * @return float The maximum angular acceleration in radians per second squared.
 */
float DataStore_GetMaxAngularAcceleration(void)
{
    float acceleration;
    osMutexAcquire(dataStoreMutex, osWaitForever);
    acceleration = dataStore.maxAngularAcceleration;
    osMutexRelease(dataStoreMutex);
    return acceleration;
}

/**
 * @brief Set the maximum angular acceleration.
 * This function updates the maximum angular acceleration in the data store.
 * @param acceleration The new maximum angular acceleration in radians per second squared.
 */
void DataStore_SetMaxAngularAcceleration(float acceleration)
{
    osMutexAcquire(dataStoreMutex, osWaitForever);
    dataStore.maxAngularAcceleration = acceleration;
    osMutexRelease(dataStoreMutex);
}
