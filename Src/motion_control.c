#include "motion_control.h"
#include "dc_motor.h"

/* ------------------ Definitions --------------------*/
// PI
#define PI 3.14159265358979323846
// 减速机每转一圈的AB相边沿数量之和
#define EDGE_PER_ROUND	(390*4)
// PID parameters
#define KP  0.5
#define KI  0.0001
#define KD  0.2

void MotionControl_Init(void)
{
    DCMotor_Init();
    DCMotor_SetAngularSpeed(0, 2*PI);
    DCMotor_SetAngularSpeed(1, 2*PI);
}
