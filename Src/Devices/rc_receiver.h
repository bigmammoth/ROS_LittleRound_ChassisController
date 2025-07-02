#pragma once

#include <stdbool.h>

typedef struct receiver_values {
    float steering; //  -1 to 1
    float throttle; //  0 to 1
    bool autoMode; // 0 - manual mode, 1 - automatic mode
    bool failSafe;
    bool frameLost;
} ReceiverValues_t;

void RC_Receiver_Init(void);
ReceiverValues_t Receiver_Read(void);
