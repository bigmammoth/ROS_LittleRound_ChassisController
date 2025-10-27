/**
 * @file init.c
 * @brief System bootstrap entry point for RTOS-based startup.
 *
 * @details Coordinates kernel start-up and invokes subsystem initializers in
 * the dedicated initialization thread (`ThreadSystemInit`). Responsibilities
 * include:
 *  - Starting CMSIS-RTOS2 kernel and spawning the init thread.
 *  - Initializing memory pool, data store, communication buses (CAN/CANopen),
 *    networking stack, ROS interface, motor/chassis control, and power
 *    controller subsystems.
 *
 * @note Keep initialization routines non-blocking where possible; long-running
 * hardware bring-up should be asynchronous to avoid delaying scheduler start.
 *
 * @author Young.W <com.wang@hotmail.com>
 * @date 2025-07-22
 * @copyright All right reserved. Hinton Robotics
 */
#include "init.h"

#include "main.h"
#include "dc_motor.h"
#include "motion_control.h"
#include "data_store.h"
#include "ros_interface.h"
#include "rc_receiver.h"
#include "battery.h"
#include "mem_pool.h"

#include "rl_net.h"

static void ThreadSystemInit(void* arg);

/**
 * @brief System initialization function
 */
void System_Init(void)
{
    osKernelInitialize(); // Initialize the OS kernel
	osThreadId_t threadID;
	threadID = osThreadNew(ThreadSystemInit, NULL, NULL);
	assert_param(threadID);
    osKernelStart();      // Start the OS kernel
}

void ThreadSystemInit(void* arg)
{
    (void)arg; // Unused parameter
    MemPool_Init();             // Initialize memory pool for dynamic allocations
    DataStore_Init();           // Initialize the data store with default configuration
    RC_Receiver_Init();         // Initialize remote controller interface
    DCMotor_Init();             // Initialize motor control interface
    MotionControl_Init();       // Initialize motion control subsystem
	osDelay(500);
	netStatus status = netInitialize();
	assert_param(status == netOK);
    ROS_Interface_Init();       // Initialize ROS interface for communication
}
