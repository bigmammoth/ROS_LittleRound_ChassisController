#include "rc_receiver_ht8a.h"

#define MAX_RECEIVER_CHANNEL_SHIFT  800
#define MID_RECEIVER_CHANNEL_VALUE  992

void Get_HT8A_Receiver_Values(ReceiverValues_t* receiverValue, S_Bus_Channel_t* receiverChannel)
{
    receiverValue->steering = (float)((int16_t)receiverChannel->channelValue[0] - MID_RECEIVER_CHANNEL_VALUE) / (float)MAX_RECEIVER_CHANNEL_SHIFT; // Convert to -1 to 1 range
    receiverValue->throttle = (float)((int16_t)receiverChannel->channelValue[2] - MID_RECEIVER_CHANNEL_VALUE) / (float)MAX_RECEIVER_CHANNEL_SHIFT; // Convert to -1 to 1 range
    receiverValue->autoMode = (receiverChannel->channelValue[4] > MID_RECEIVER_CHANNEL_VALUE) ? true : false; // Assuming channel 5 is used for manual/automatic mode
    receiverValue->failSafe = receiverChannel->flagBit_Failsafe;
    receiverValue->frameLost = receiverChannel->flagBit_FrameLost;
}
