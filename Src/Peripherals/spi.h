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
#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Set the Chip Select pin low to select the SPI device
 * This function sets the Chip Select (CS) pin low to select the SPI device for communication.
 * It is assumed that the CS pin is configured as a GPIO output.
 * @note The actual GPIO port and pin should be defined in the main.h or appropriate header file.
 */
void SPI_SetChipSelectLow(void);

/**
 * @brief Set the Chip Select pin high to deselect the SPI device
 * This function sets the Chip Select (CS) pin high to deselect the SPI device after communication.
 * It is assumed that the CS pin is configured as a GPIO output.
 * @note The actual GPIO port and pin should be defined in the main.h or appropriate header file.
 */
void SPI_SetChipSelectHigh(void);

/**
 * @brief Transmit and receive data over SPI
 * This function transmits data to the SPI device and simultaneously receives data from it.
 * It uses the HAL_SPI_TransmitReceive function from the STM32 HAL library.
 * @param txBuff Pointer to the buffer containing data to be transmitted.
 * @param rxBuff Pointer to the buffer where received data will be stored.
 * @param size Number of bytes to transmit and receive.
 * @return true if the operation was successful, false otherwise.
 */
bool SPI_TransmitReceive(const uint8_t *txBuff, uint8_t *rxBuff, uint32_t size);

/**
 * @brief Transmit data over SPI
 * This function transmits data to the SPI device.
 * It uses the HAL_SPI_Transmit function from the STM32 HAL library.
 * @param txBuff Pointer to the buffer containing data to be transmitted.
 * @param size Number of bytes to transmit.
 * @return true if the operation was successful, false otherwise.
 */
bool SPI_Transmit(const uint8_t *txBuff, uint32_t size);

/**
 * @brief Receive data over SPI
 * This function receives data from the SPI device.
 * It uses the HAL_SPI_Receive function from the STM32 HAL library.
 * @param rxBuff Pointer to the buffer where received data will be stored.
 * @param size Number of bytes to receive.
 * @return true if the operation was successful, false otherwise.
 */
bool SPI_Receive(uint8_t *rxBuff, uint32_t size);
