/**
 * @file spi.h
 * @brief SPI communication interface for STM32F4 devices.
 *
 * @details This file provides inline functions to handle SPI communication,
 * including setting the Chip Select (CS) pin, transmitting, and receiving data.
 * It uses the HAL library for SPI operations and assumes that the CS pin is
 * configured as a GPIO output.
 *
 * Functions:
 *  - SPI_SetChipSelectLow: Set the CS pin low to select the SPI device.
 *  - SPI_SetChipSelectHigh: Set the CS pin high to deselect the SPI device.
 *  - SPI_TransmitReceive: Transmit and receive data over SPI.
 *  - SPI_Transmit: Transmit data over SPI.
 *  - SPI_Receive: Receive data over SPI.
 * @note The actual GPIO port and pin for the CS line should be defined in the
 *       main.h or appropriate header file.
 * @author Young.W <com.wang@hotmail.com>
 * @date 2025-09-25
 */
#include "spi.h"

#include "main.h"

#include <string.h>

#define SPI_BUF_LEN		18
#define SPI_TIMEOUT	    5

/* -------------- External variables ---------------- */
extern SPI_HandleTypeDef hspi2;

/* -------------- Static variables ------------------ */

/**
 * @brief Set the Chip Select pin low to select the SPI device
 * This function sets the Chip Select (CS) pin low to select the SPI device for communication.
 * It is assumed that the CS pin is configured as a GPIO output.
 * @note The actual GPIO port and pin should be defined in the main.h or appropriate header file.
 */
inline void SPI_SetChipSelectLow(void)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Set the Chip Select pin high to deselect the SPI device
 * This function sets the Chip Select (CS) pin high to deselect the SPI device after communication.
 * It is assumed that the CS pin is configured as a GPIO output.
 * @note The actual GPIO port and pin should be defined in the main.h or appropriate header file.
 */
inline void SPI_SetChipSelectHigh(void)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Transmit and receive data over SPI
 * This function transmits data to the SPI device and simultaneously receives data from it.
 * It uses the HAL_SPI_TransmitReceive function from the STM32 HAL library.
 * @param txBuff Pointer to the buffer containing data to be transmitted.
 * @param rxBuff Pointer to the buffer where received data will be stored.
 * @param size Number of bytes to transmit and receive.
 */
inline bool SPI_TransmitReceive(const uint8_t *txBuff, uint8_t *rxBuff, uint32_t size)
{
    return (HAL_SPI_TransmitReceive(&hspi2, txBuff, rxBuff, size, SPI_TIMEOUT) == HAL_OK);
}

/**
 * @brief Transmit data over SPI
 * This function transmits data to the SPI device.
 * It uses the HAL_SPI_Transmit function from the STM32 HAL library.
 * @param txBuff Pointer to the buffer containing data to be transmitted.
 * @param size Number of bytes to transmit.
 */
inline bool SPI_Transmit(const uint8_t *txBuff, uint32_t size)
{
    return (HAL_SPI_Transmit(&hspi2, txBuff, size, SPI_TIMEOUT) == HAL_OK);
}

/**
 * @brief Receive data over SPI
 * This function receives data from the SPI device.
 * It uses the HAL_SPI_Receive function from the STM32 HAL library.
 * @param rxBuff Pointer to the buffer where received data will be stored.
 * @param size Number of bytes to receive.
 */
inline bool SPI_Receive(uint8_t *rxBuff, uint32_t size)
{
    return (HAL_SPI_Receive(&hspi2, rxBuff, size, SPI_TIMEOUT) == HAL_OK);
}
