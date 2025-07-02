#pragma once

#include <stdint.h>

#define S_BUS_MESSAGE_SIZE      25
#define S_BUS_CHANNEL_NUMBER    16

typedef struct s_bus_channel {
    uint16_t channelValue[S_BUS_CHANNEL_NUMBER];
    uint16_t flagBit_Failsafe;
    uint16_t flagBit_FrameLost;
    uint16_t flagBit_CH16;
    uint16_t flagBit_CH17;
} S_Bus_Channel_t;

uint32_t S_BUS_Parse(const uint8_t*, S_Bus_Channel_t*);
