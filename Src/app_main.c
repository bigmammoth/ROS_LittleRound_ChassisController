/**
  ******************************************************************************
  * @file           : app_main.c
  * @brief          : Main thread, handle commands
  ******************************************************************************
  * Main thread which handle commands and messages from remote controller and
  * central computer. Calulate detail datas for hub moters.
  * 
  ******************************************************************************
  */
#include "rl_net.h" // Keil.MDK-Plus::Network:CORE
#include "main.h"
#include "app_main.h"
#include "motion_control.h"

/* --------------- Static functions ---------------- */
static void periodicTimerCallback(void *argument);

/* --------------- Static variables ---------------- */
static osThreadId_t threadId;

/**
 * @brief Main thread, handle commands from remote controller and upper computer
 * @param arg Pointer to an incoming parameter when creating the thread.
 * @return none
 */
void AppMain(void *arg)
{
    osDelay(500);
    MotionControl_Init();
	// Initialize network interface
	netStatus status = netInitialize();
	
	while (true)
	{
        osThreadYield();
	}
}

/**
 * @brief Periodiclly callback
 * Callback every 50ms, sends a timed message to the app_main
 * @param argument pointer to an argument from rtos
 * @return none
 */
void periodicTimerCallback(void *argument)
{
}

/**
 * @brief Initalize app_main
 * Create thrade and set its priority.
 */
void AppMain_Init(void)
{
	threadId = osThreadNew(AppMain, NULL, NULL);
	//osThreadSetPriority(threadId, osPriorityNormal1);
}
