#pragma once

#include <stdint.h>
#include <stdbool.h>

// PI
#define PI 3.14159265358979323846
#define MAX_ANGULAR_VELOCITY (3 * PI)   // rad/s
#define MAX_VELOCITY 1.0                // m/s Determined by the rated speed of the motor MG513P30 293RPM and the wheel diameter of 64mm
#define WHEELS_DISTANCE 0.16            // 180mm
#define WHEEL_DIAMETER 0.064            // 64mm
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2)
#define WHEEL_PERIMETER (WHEEL_DIAMETER * PI)

// Total motor number
#define TOTAL_MOTOR_NUMBER  2

// Network configuration
#define UDP_ROS_LISTEN_PORT 12000   // Port for ROS interface

// Receiver configuration
//#define RECEIVER_TYPE_WFLY
#define RECEIVER_TYPE_HT8A
