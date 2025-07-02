#pragma once

#include <stdint.h>
#include <stdbool.h>

enum UDP_MSG_TYPE
{
    UDP_MSG_TYP_HEARTBEAT = 12000,
    UDP_MSG_TYPE_SET_ROBOT_MOTION,
    UDP_MSG_TYPE_MOTOR_INFO,
    UDP_MSG_TYPE_SYSTEM_STATUS
};

/* -------- Received UDP data from ROS ---------- */
#define MOTOR_TOTAL_NUM 2

typedef struct
{
    uint32_t msgType;
    float speed;
    float omega;
} UdpSetRobotMotion_t;

/* --------- UDP data reported to ROS ----------- */
// Motor information
typedef struct
{
    uint32_t msgTyp;
    uint32_t runningStatus[MOTOR_TOTAL_NUM]; // 0-Normal 1-Over volatage 2-Over current 3-Over load 4-Hall sensor error 5-Current overshoot 6-Encoder overshoot 7-Speed overshoot 8-Reference volate error
    float speed[MOTOR_TOTAL_NUM];            // current running speed in 0.1RPM
    double position[MOTOR_TOTAL_NUM];       // encouder counts
} UdpMotorInfo_t;

// System status
typedef struct
{
    uint32_t msgTyp;
    uint32_t upTime;     // Power on time
    uint32_t errorCode;  // 0 - Normal 1 - Motor fault 2 - Low Voltage 3 - Overheating 4 - VCU no response
    float voltage;       // Battery voltage
    float current;       // Total currency
    float capacity;      // Battery remain capacity in percentage
    uint16_t inCharge;   // If in charge or not
    uint16_t autoMode; // If the robot controlled manually.
} UdpSystemStatus_t;

/* ---------------- Functions ---------------------------*/
void ROS_Interface_Init(void);
