#include "chassis_odometry.h"
#include "cmsis_os2.h"
#include "mem_pool.h"
#include "system_config.h"
#include "data_store.h"
#include "main.h"
#include "arm_math.h"
#include "chassis_motion.h"

/* ---------------------- Type Definitions ------------------------------ */
typedef void (*ChassisOdometryX_Init_t)(void);
typedef bool (*ChassisOdometryX_GetOdometry_t)(float* x, float* y, float* theta);
typedef bool (*ChassisOdometryX_Update_t)(float* wheelPosArray, float dt);
typedef void (*ChassisOdometryX_Reset_t)(void);

/* ---------------------- Static Variables ------------------------------ */
static osThreadId_t chassisOdometryThreadId;
static osThreadAttr_t chassisOdometryThreadAttr = {
    .name = "ChassisOdometryThread",
    .stack_size = 1024,
    .priority = osPriorityNormal,
};
static void* wheelPositionArray; // Pointer to wheel position array
static ChassisOdometryX_Init_t ChassisOdometryX_Init;   
static ChassisOdometryX_GetOdometry_t ChassisOdometryX_GetOdometry;
static ChassisOdometryX_Update_t ChassisOdometryX_Update;
static ChassisOdometryX_Reset_t ChassisOdometryX_Reset;

static float x = 0.0f;      // X position in meters
static float y = 0.0f;      // Y position in meters
static float theta = 0.0f;  // Orientation in radians
static float velocity = 0.0f; // Linear velocity in m/s
static float omega = 0.0f; // Angular velocity in rad/s

/* ---------------------- Static Functions ------------------------------ */
static void ChassisOdometry_Thread(void* arg);

/**
 * @brief Initialize the chassis odometry system.
 * This function initializes the chassis odometry system.
 * @return true if initialization is successful, false otherwise.
 */
void ChassisOdometry_Init(void)
{
    switch (DataStore_GetChassisType())
    {
        case CHASSIS_TYPE_TWO_WHEEL_DIFFERENTIAL:
            ChassisOdometryX_Init = TwoWheelOdometry_Init;
            ChassisOdometryX_GetOdometry = TwoWheelOdometry_GetOdometry;
            ChassisOdometryX_Update = TwoWheelOdometry_Update;
            ChassisOdometryX_Reset = TwoWheelOdometry_Reset;
            break;
        
        default:
            return;
    }
    assert_param(ChassisOdometryX_Init != NULL);
    assert_param(ChassisOdometryX_GetOdometry != NULL);
    assert_param(ChassisOdometryX_Update != NULL);
    assert_param(ChassisOdometryX_Reset != NULL);

    ChassisOdometryX_Init();
    chassisOdometryThreadId = osThreadNew(ChassisOdometry_Thread, NULL, &chassisOdometryThreadAttr);
    assert_param(chassisOdometryThreadId != NULL);
    wheelPositionArray = MemPool_Alloc(TOTAL_MOTOR_NUMBER * sizeof(float));
    assert_param(wheelPositionArray != NULL);
}

/**
 * @brief Get the current odometry of the chassis.
 * This function retrieves the current odometry of the chassis.
 * @param x Pointer to store the x position in meters.
 * @param y Pointer to store the y position in meters.
 * @param theta Pointer to store the orientation in radians.
 * @param velocity Pointer to store the linear velocity in meters per second.
 * @param omega Pointer to store the angular velocity in radians per second.
 * @return true if odometry retrieval is successful, false otherwise.
 */
bool ChassisOdometry_GetOdometry(float* pX, float* pY, float* pTheta, float* pVelocity, float* pOmega)
{
    if (pX == NULL || pY == NULL || pTheta == NULL || pVelocity == NULL || pOmega == NULL) return false;
    *pX = x;
    *pY = y;
    *pTheta = theta;
    *pVelocity = velocity;
    *pOmega = omega;
    return true;
}

/**
 * @brief Thread function for chassis odometry processing.
 * This function runs in a separate thread and is responsible for processing
 * odometry data from the chassis.
 * @param arg Pointer to the argument (not used).
 */
static void ChassisOdometry_Thread(void* arg)
{
	(void) arg;
    #define ODOMETRY_UPDATE_PERIOD_MS 10
    while (true)
    {
        osDelay(ODOMETRY_UPDATE_PERIOD_MS);
        float wheelPositionArray[TOTAL_MOTOR_NUMBER];
        // Read wheel positions from motors
        for (int i = 0; i < TOTAL_MOTOR_NUMBER; i++)
        {
            wheelPositionArray[i] = Motor_GetPosition(i);
        }
        ChassisOdometryX_Update(wheelPositionArray, (float)ODOMETRY_UPDATE_PERIOD_MS / 1000.0f);
        ChassisOdometryX_GetOdometry(&x, &y, &theta);
        ChassisMotion_GetMotion(&velocity, &omega);
    }
}

void ChassisOdometry_Reset(void)
{
    if (ChassisOdometryX_Update != NULL) ChassisOdometryX_Reset();
}
