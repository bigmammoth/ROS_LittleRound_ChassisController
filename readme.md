# ChassisController

A real-time chassis control system built on STM32F407VET6 microcontroller with Ethernet connectivity and motion control capabilities.

## Overview

This project implements a chassis controller for a mobile robot platform with the following key features:

- **STM32F407VET6** microcontroller
- **Ethernet communication** via DP83848C PHY
- **Real-time operation** using CMSIS-RTOS2 (Keil RTX5)
- **Motion control** subsystem
- **Network stack** with TCP/UDP socket support
- **HTTP server** capabilities

## Hardware Requirements

- STM32F407VET6 development board
- Ethernet PHY (DP83848C)
- Motor drivers and encoders (for chassis control)
- Power supply (3.3V/5V)

## Software Architecture

### Core Components

- **RTOS**: CMSIS-RTOS2 with Keil RTX5 kernel
- **Network Stack**: MDK-Network middleware
  - Ethernet MAC/PHY drivers
  - TCP/UDP socket support
  - HTTP server functionality
- **Motion Control**: Custom motion control module
- **HAL**: STM32F4xx HAL drivers

### Project Structure

## Configuration

### Network Configuration

The system is configured with:
- **Host Name**: "ChassisControl"
- **Ethernet Interface**: ETH0 with configurable MAC address
- **Network Stack**: IPv4/IPv6 dual stack support
- **Memory Pool**: Configurable size for network buffers

### HTTP Server

- **Port**: Configurable (default web port)
- **Authentication**: Optional admin authentication
- **Root Folder**: Configurable web root directory

## Building the Project

### Prerequisites

- **Keil µVision 5** (version 5.42.0.0)
- **Keil Device Specific** (Keil::STM32F4xx_DFP 3.0.0)
- **ARM Packs** (ARM::CMSIS-6.1.0 ARM::CMSIS-Driver-2.10.0 ARM::CMSIS-Driver_STM32-1.2.0)
- **ARM Compiler 6**
- **STM32CubeMX** (version 6.15.0 STM32CubeF4_Firmware_Package_V1.28.2)

### Build Steps

1. Open the project in Keil µVision:
2. Select the target configuration:
- Target: `ChassisController`
- Device: `STM32F407VETx`

3. Build the project:
- Press `F7` or use **Project > Build Target**

4. Flash to target:
- Connect J-Link debugger
- Press `F8` or use **Flash > Download**

## Pin Configuration

The project uses STM32CubeMX for pin configuration. Key peripherals:

- **Ethernet**: RMII interface
- **USART**: For debugging/communication
- **GPIO**: Motor control and sensor interfaces
- **Timers**: PWM generation and encoder reading

## Usage

### Network Communication

The chassis controller provides network services:

1. **TCP/UDP Sockets**: For real-time communication
2. **HTTP Server**: For web-based control interface
3. **Motion Control API**: For chassis movement commands

### Motion Control

The motion control subsystem handles:
- Motor speed and direction control
- Encoder feedback processing
- Real-time motion updates

## Configuration Files

Key configuration files in [`RTE/`](RTE/):
- [`Net_Config.h`](RTE/Network/Net_Config.h): Core network settings
- [`Net_Config_ETH_0.h`](RTE/Network/Net_Config_ETH_0.h): Ethernet interface configuration
- [`Net_Config_HTTP_Server.h`](RTE/Network/Net_Config_HTTP_Server.h): HTTP server settings

## License

This project uses components with different licenses:
- **STM32 HAL/CMSIS**: STMicroelectronics license
- **CMSIS Components**: Apache 2.0 license
- **MDK-Network**: ARM Keil license

See individual component license files for details.

## Support

For technical support:
- Review STM32F4xx documentation
- Check Keil MDK documentation
- Consult CMSIS-RTOS2 API reference
- STM32CubeMX user manual

## Version Information

- **CMSIS Version**: 5.6.x
- **STM32F4xx HAL**: Latest version
- **MDK-Network**: 8.0.0
- **Keil RTX5**: 5.9.0
