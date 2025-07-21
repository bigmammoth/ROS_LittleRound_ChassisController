#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "./Protocol/s_bus.h"

typedef struct receiver_values {
    float steering; //  -1 to 1
    float throttle; //  -1 to 1
    bool autoMode; // 0 - manual mode, 1 - automatic mode
    bool failSafe;
    bool frameLost;
} ReceiverValues_t;

void Get_HT8A_Receiver_Values(ReceiverValues_t* receiverValue, S_Bus_Channel_t* receiverChannel);
