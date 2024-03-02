#include "rl_net.h" // Keil.MDK-Plus::Network:CORE
#include "main.h"
#include "motion_control.h"
#include "dc_motor.h"
#include "rc_receiver.h"
#include "ros_interface.h"

typedef struct motionMessage {
    float velocity;
    float omega;
} MotionMessage_t;

/* ------------------ Definitions --------------------*/
// PI
#define PI 3.14159265358979323846
#define MESSAGE_QUEUE_SIZE              16
#define MOTION_CONTROL_TIME_INTERVAL    50          // 50ms
#define MAX_ANGULAR_VELOCITY            (1*PI)      // rad/s
#define MAX_VELOCITY                    1.0         // m/s
#define WHEELS_DISTANCE                 0.16        // 180mm
#define WHEEL_DIAMETER                  0.064       // 64mm
#define WHEEL_RADIUS                    (WHEEL_DIAMETER / 2)
#define WHEEL_SIDE_LEN                  (WHEEL_DIAMETER * 2 * PI)

/* --------------- Static variables ---------------- */
static osThreadId_t threadId;
static osMessageQueueId_t messageQueue;
static osTimerId_t periodicTimer;

/* --------------- Static functions ---------------- */
static void MotionControl_Process(void*);
static void InitSystem(void);
static void ReadReceiver(void*);

void MotionControl_Init(void)
{
    messageQueue = osMessageQueueNew(MESSAGE_QUEUE_SIZE, sizeof(MotionMessage_t), NULL);
    threadId = osThreadNew(MotionControl_Process, NULL, NULL);
    periodicTimer = osTimerNew(ReadReceiver, osTimerPeriodic, NULL, NULL);
}

static void MotionControl_Process(void* arg)
{
    InitSystem();
    MotionMessage_t msg;
    osTimerStart(periodicTimer, MOTION_CONTROL_TIME_INTERVAL);
	while (true)
	{
        osStatus_t status = osMessageQueueGet(messageQueue, &msg, NULL, osWaitForever);
        if(status != osOK) continue;
        float rightV = msg.velocity + msg.omega * (WHEELS_DISTANCE / 2);
        float leftV = msg.velocity - msg.omega * (WHEELS_DISTANCE / 2);
        if(rightV > MAX_VELOCITY) rightV = MAX_VELOCITY;
        else if(rightV < -MAX_VELOCITY) rightV = -MAX_VELOCITY;
        if(leftV > MAX_VELOCITY) leftV = MAX_VELOCITY;
        else if(leftV < -MAX_VELOCITY) leftV = -MAX_VELOCITY;
        DCMotor_SetAngularSpeed(0, leftV / WHEEL_RADIUS);
        DCMotor_SetAngularSpeed(1, rightV / WHEEL_RADIUS);
	}
}

void MotionControl_Move(float velocity, float omega)
{
    MotionMessage_t msg = {velocity, omega};
    osMessageQueuePut(messageQueue, &msg, 0, 0);
}

static void InitSystem(void)
{
    osDelay(500);
    DCMotor_Init();
    RC_Receiver_Init();
    ROS_Interface_Init();
	// Initialize network interface
	netStatus status = netInitialize();
}

static void ReadReceiver(void* arg)
{
    ReceiverValues_t receiverValue = Receiver_Read();
    // Turn receiver value to angular velocity and velocity.
    float omega = -(float)receiverValue.steering / MAX_RECEIVER_CHANNEL_SHIFT * MAX_ANGULAR_VELOCITY;
    float velocity = (float)receiverValue.throttle / (2 * MAX_RECEIVER_CHANNEL_SHIFT) * MAX_VELOCITY;
    // Send to motion control process.
    MotionMessage_t msg = {velocity, omega};
    osMessageQueuePut(messageQueue, &msg, 0, 0);
}
