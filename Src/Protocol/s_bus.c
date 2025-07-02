#include "s_bus.h"
#include <string.h>

/**
 * @brief Parse the S-Bus message from the input byte array.
 * This function extracts the channel values and flags from the S-Bus message format.
 * The S-Bus message consists of 25 bytes, where the first byte is a header byte,
 * followed by 22 bytes containing channel values, and the last byte contains flags.
 * The channel values are 11 bits each, and the flags indicate the status of the channels.
 * This function fills the provided S_Bus_Channel_t structure with the parsed values.
 * * @note The S-Bus message format is as follows:
 * - Byte 0: Header byte (should be 0x0F)
 * * - Bytes 1-22: Channel values (11 bits each)
 * * - Byte 23: Flags
 * *   - Bit 0: CH17 (channel 17 status)
 * *   - Bit 1: CH16 (channel 16 status)
 * *   - Bit 2: Frame lost (indicates if the frame was lost)
 * *   - Bit 3: Failsafe (indicates if the failsafe mode is active)
 * * The function extracts the channel values and flags from the input byte array
 * and stores them in the provided S_Bus_Channel_t structure.
 * @param inputMessage Pointer to the input byte array containing the S-Bus message.
 * @param channel Pointer to the S_Bus_Channel_t structure to store the parsed values.
 * @return 1 if parsing was successful, 0 otherwise.
 */
uint32_t S_BUS_Parse(const uint8_t* inputMessage, S_Bus_Channel_t* channel)
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
