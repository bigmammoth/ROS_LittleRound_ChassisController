#pragma once

#define MAX_RECEIVER_CHANNEL_SHIFT  671
#define MID_RECEIVER_CHANNEL_VALUE  1024

typedef struct receiver_values {
    int16_t steering;
    int16_t throttle;
    uint16_t manualMode; // 0 - manual mode, 1 - automatic mode
    uint16_t failSafe;
    uint16_t frameLost;
} ReceiverValues_t;

void RC_Receiver_Init(void);
ReceiverValues_t Receiver_Read(void);
