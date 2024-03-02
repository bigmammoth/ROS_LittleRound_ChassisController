#pragma once

#define MAX_RECEIVER_CHANNEL_SHIFT  671
#define MID_RECEIVER_CHANNEL_VALUE  1024

typedef struct receiver_values {
    int16_t steering;
    int16_t throttle;
} ReceiverValues_t;

void RC_Receiver_Init(void);
ReceiverValues_t Receiver_Read(void);
