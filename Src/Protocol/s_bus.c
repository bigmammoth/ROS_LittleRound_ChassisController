#include "s_bus.h"

uint32_t S_BUS_Parse(uint8_t* inputMessage, S_Bus_Channel_t* channel)
{
    if (inputMessage[0] != 0x0F) return 0;
    uint16_t remain;
    uint32_t index = 0;
    uint64_t value = ((uint64_t*)inputMessage)[0];
    value >>= 8;
    for(int i = 0; i < 5; ++i)
    {
        channel->channelValue[index++] = (uint16_t)value & 0x7FF;
        value >>= 11;
    }
    remain = (uint16_t)value;
    value = ((uint64_t*)inputMessage)[1];
    channel->channelValue[index++] = (((uint16_t)value & 0x3FF)<<1) | remain;
    value >>= 10;
    for(int i = 0; i < 4; ++i)
    {
        channel->channelValue[index++] = (uint16_t)value & 0x7FF;
        value >>= 11;
    }
    remain = (uint16_t)value;
    value = ((uint64_t*)inputMessage)[2];
    channel->channelValue[index++] = (((uint16_t)value & 0x001)<<10) | remain;
    value >>= 1;
    for(int i = 0; i < 5; ++i)
    {
        channel->channelValue[index++] = (uint16_t)value & 0x7FF;
        value >>= 11;
    }
    channel->flagBit_Failsafe = (inputMessage[23]>>3) & 0x01;
    channel->flagBit_FrameLost = (inputMessage[23]>>2) & 0x01;
    channel->flagBit_CH16 = (inputMessage[23]>>1) & 0x01;
    channel->flagBit_CH17 = inputMessage[23] & 0x01;
    return 1;
}
