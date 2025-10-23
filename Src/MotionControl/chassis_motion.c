/**
 * @file chassis_motion.c
 * @brief Chassis motion control implementation.
 * @details This file implements the chassis motion control interface defined in
 *          chassis_motion.h. It includes initialization, setting motion
 *          commands, and retrieving current motion states. The implementation
 *          details are abstracted away behind the interface.
 *          The chassis motion system supports various chassis types and allows
 *          switching between manual and autonomous modes.
 *          The actual chassis motion implementations (e.g., Two-Wheel Differential,
 *          Mecanum Wheel) are included as separate modules and selected at runtime.
 *          A periodic task is created to handle motion updates and remote control
 *          input.
 * @author Young.W <com.wang@hotmail.com>
 * @date 2025-09-15
 */
#include "chassis_motion.h"

#include "main.h"
#include "data_store.h"
// #include "./Ackermann/ackermann.h"
// #include "./FourWheelAckermann/four_wheel_ackermann.h"
// #include "./FourWheelDifferential/four_wheel_differential.h"
// #include "./FourWheelIndependent/four_wheel_independent.h"
// #include "./H-Drive/h_drive.h"
// #include "./MecanumWheel/mecanum_wheel.h"
// #include "./OmniWheel/omni_wheel.h"
// #include "./TrackedDifferential/tracked_differential.h"
// #include "./TricycleAckermann/tricycle_ackermann.h"
// #include "./TricycleDifferential/tricycle_differential.h"
#include "./TwoWheelDifferential/two_wheel_differential.h"
#include "./TwoWheelDifferential/two_wheel_kinematic.h"
#include "remote_controller.h"
#include "ros_subscriber_cmd_vel.h"
#include "chassis_odometry.h"
#include "motion_state.h"

typedef void (*ChassisMotionX_Init_t)(void);
typedef void (*ChassisMotionX_SetMotion_t)(float Motion, float omega);
typedef void (*ChassisMotionX_GetMotion_t)(float* Motion, float* omega);

/* ---------------------- Static Variables ------------------------------ */
static osThreadId_t chassisMotionThreadId;
static osThreadAttr_t chassisMotionThreadAttr = {
    .name = "ChassisMotionThread",
    .stack_size = 1024,
    .priority = osPriorityNormal,
};
#define FLAG_CHASSIS_MOTION_PERIODIC 0x01 // Periodic event flag
static osTimerId_t chassisMotionTimer;
static bool isAutoMode; // Flag to indicate if the chassis is in auto mode

/* ---------------------- Static Function Pointers ---------------------- */
static ChassisMotionX_Init_t ChassisMotionX_Init;
static ChassisMotionX_SetMotion_t ChassisMotionX_SetMotion;
static ChassisMotionX_GetMotion_t ChassisMotionX_GetMotion;

/* ---------------------- Static Functions ------------------------------ */
static void ThreadChassisMotion(void* arg);
static void PeriodicTimerCallback(void* arg);

/**
 * @brief Initialize the chassis motion system based on the chassis type.
 * This function selects the appropriate chassis motion implementation
 * based on the chassis type defined in the data store.
 * @details It initializes the chassis motion system by setting the function pointers
 * to the appropriate implementation based on the chassis type. The actual initialization
 * of the chassis motion system is done by calling the Init function pointer.
 */
void ChassisMotion_Init(void)
{
    chassisType_t chassisType = DataStore_GetChassisType();
    switch (chassisType)
    {
        case CHASSIS_TYPE_ACKERMANN:
            // ChassisMotionX_Init = Ackermann_Init;
            // ChassisMotionX_SetMotion = Ackermann_SetMotion;
            // ChassisMotionX_GetMotion = Ackermann_GetMotion;
            break;
        case CHASSIS_TYPE_FOUR_WHEEL_ACKERMANN:
            // ChassisMotionX_Init = FourWheelAckermann_Init;
            // ChassisMotionX_SetMotion = FourWheelAckermann_SetMotion;
            // ChassisMotionX_GetMotion = FourWheelAckermann_GetMotion;
            break;
        case CHASSIS_TYPE_FOUR_WHEEL_DIFFERENTIAL:
            // ChassisMotionX_Init = FourWheelDifferential_Init;
            // ChassisMotionX_SetMotion = FourWheelDifferential_SetMotion;
            // ChassisMotionX_GetMotion = FourWheelDifferential_GetMotion;
            break;
        case CHASSIS_TYPE_FOUR_WHEEL_INDEPENDENT:
            // ChassisMotionX_Init = FourWheelIndependent_Init;
            // ChassisMotionX_SetMotion = FourWheelIndependent_SetMotion;
            // ChassisMotionX_GetMotion = FourWheelIndependent_GetMotion;
            break;
        case CHASSIS_TYPE_H_DRIVE:
            // ChassisMotionX_Init = HDrive_Init;
            // ChassisMotionX_SetMotion = HDrive_SetMotion;
            // ChassisMotionX_GetMotion = HDrive_GetMotion;
            break;
        case CHASSIS_TYPE_MECANUM_WHEEL:
            // ChassisMotionX_Init = MecanumWheel_Init;
            // ChassisMotionX_SetMotion = MecanumWheel_SetMotion;
            // ChassisMotionX_GetMotion = MecanumWheel_GetMotion;
            break;
        case CHASSIS_TYPE_OMNI_WHEEL:
            // ChassisMotionX_Init = OmniWheel_Init;
            // ChassisMotionX_SetMotion = OmniWheel_SetMotion;
            // ChassisMotionX_GetMotion = OmniWheel_GetMotion;
            break;
        case CHASSIS_TYPE_TRACKED_DIFFERENTIAL:
            // ChassisMotionX_Init = TrackedDifferential_Init;
            // ChassisMotionX_SetMotion = TrackedDifferential_SetMotion;
            // ChassisMotionX_GetMotion = TrackedDifferential_GetMotion;
            break;
        case CHASSIS_TYPE_TRICYCLE_ACKERMANN:
            // ChassisMotionX_Init = TricycleAckermann_Init;
            // ChassisMotionX_SetMotion = TricycleAckermann_SetMotion;
            // ChassisMotionX_GetMotion = TricycleAckermann_GetMotion;
            break;
        case CHASSIS_TYPE_TRICYCLE_DIFFERENTIAL:
            // ChassisMotionX_Init = TricycleDifferential_Init;
            // ChassisMotionX_SetMotion = TricycleDifferential_SetMotion;
            // ChassisMotionX_GetMotion = TricycleDifferential_GetMotion;
            break;
        case CHASSIS_TYPE_TWO_WHEEL_DIFFERENTIAL:
            ChassisMotionX_Init = ChassisTwoWheelDifferential_Init;
            ChassisMotionX_SetMotion = ChassisTwoWheelDifferential_SetMotion;
            ChassisMotionX_GetMotion = ChassisTwoWheelDifferential_GetMotion;
            break;
        default:
			break;
    }

    assert_param(ChassisMotionX_Init != NULL);
    assert_param(ChassisMotionX_SetMotion != NULL);
    assert_param(ChassisMotionX_GetMotion != NULL);

    ChassisMotionX_Init();
    ChassisOdometry_Init();
    chassisMotionThreadId = osThreadNew(ThreadChassisMotion, NULL, &chassisMotionThreadAttr);
    assert_param(chassisMotionThreadId != NULL);
    chassisMotionTimer = osTimerNew(PeriodicTimerCallback, osTimerPeriodic, NULL, NULL);
    assert_param(chassisMotionTimer != NULL);
    osTimerStart(chassisMotionTimer, 20); // Start the periodic timer with a 20 ms interval
}

/**
 * @brief Set the Motion of the chassis motion system.
 * 
 * This function sets the velocity and omega of the chassis motion system by calling the
 * appropriate implementation's SetMotion function.
 * @param velocity The desired linear velocity to set, in meters per second.
 * @param omega The desired angular velocity to set, in radians per second.
 * @return None
 */
void ChassisMotion_SetMotion(float velocity, float omega)
{
    if (ChassisMotionX_SetMotion != NULL) ChassisMotionX_SetMotion(velocity, omega);
}

/**
 * @brief Get the Motion of the chassis motion system.
 * 
 * This function retrieves the current Motion of the chassis motion system by calling the
 * appropriate implementation's GetMotion function.
 * @param velocity Pointer to store the current linear velocity, in meters per second.
 * @param omega Pointer to store the current angular velocity, in radians per second.
 * @return None
 */
void ChassisMotion_GetMotion(float* velocity, float* omega)
{
    if (ChassisMotionX_GetMotion != NULL) ChassisMotionX_GetMotion(velocity, omega);
}

/**
 * @brief Thread function for chassis motion control.
 * 
 * This function runs in a separate thread and is responsible for controlling the
 * chassis motion based on the received remote control commands.
 * @param arg Pointer to the argument (not used)
 */
void ThreadChassisMotion(void* arg)
{
    (void)arg; // Unused parameter
    RemoteControlValue_t receiverValue;
    float maxVelocity = DataStore_GetMaxLinearVelocity();
    float maxOmega = DataStore_GetMaxAngularVelocity();
    float velocity, omega;
    for(;;)
    {
        uint32_t flags = osThreadFlagsWait(FLAG_CHASSIS_MOTION_PERIODIC, osFlagsWaitAny, osWaitForever);
        if (flags & FLAG_CHASSIS_MOTION_PERIODIC)
        {
            RemoteController_Read(&receiverValue);
            if (!receiverValue.frameLost && !receiverValue.failSafe && !receiverValue.autoMode)
            {
                isAutoMode = false;
                ChassisMotionX_SetMotion(receiverValue.throttle * maxVelocity, receiverValue.steering * maxOmega);
                continue;
            }
            if (isAutoMode)
            {
                ROS_SubscriberCmdVel_ReadVelocity(&velocity, &omega);
                ChassisMotionX_SetMotion(velocity, omega);
            }
        }
    }
}

/**
 * @brief Periodic Timer Callback
 * 
 * This function is called periodically by the timer and is responsible for
 * signaling the chassis motion thread to update the motion control.
 * @param arg Pointer to the argument (not used)
 */
void PeriodicTimerCallback(void* arg)
{
    (void)arg; // Unused parameter
    osThreadFlagsSet(chassisMotionThreadId, FLAG_CHASSIS_MOTION_PERIODIC); // Signal the chassis motion thread
}

/**
 * @brief Park the chassis.
 * 
 * This function parks the chassis by setting the motion to zero.
 * @param park If true, the chassis is parked; otherwise, it is unparked.
 * @note This is a placeholder implementation and should be replaced with actual parking logic.
 */
void ChassisMotion_Park(bool park)
{
    if (park)
    {
        ChassisMotionX_SetMotion(0.0f, 0.0f);
    }
    else
    {
        ChassisMotionX_SetMotion(0.0f, 0.0f);
    }
}

/**
 * @brief Set the auto mode of the chassis.
 * 
 * This function sets the auto mode of the chassis. When in auto mode, the chassis
 * will follow commands from the ROS interface instead of the remote controller.
 * @param autoMode If true, the chassis is set to auto mode; otherwise, it is set to manual mode.
 */
void ChassisMotion_SetAutoMode(bool autoMode)
{
    isAutoMode = autoMode;
}

/**
 * @brief Check if the chassis is in auto mode.
 * 
 * This function returns whether the chassis is currently in auto mode.
 * @return true if the chassis is in auto mode; false otherwise.
 */
bool ChassisMotion_IsAutoMode(void)
{
    return isAutoMode;
}

/**
 * @brief Get the current gear mode.
 * 
 * This function retrieves the current gear mode of the chassis by calling
 * the MotionState module.
 * @return The current gear mode.
 */
GearMode_t ChassisMotion_GetGearMode(void)
{
    return MotionState_GetCurrentGearMode();
}

/**
 * @brief Get the current motion mode.
 * 
 * This function retrieves the current motion mode of the chassis.
 * @return The current motion mode.
 */
MotionMode_t ChassisMotion_GetMotionMode(void)
{
    return MOTION_MODE_SPEED;
}
