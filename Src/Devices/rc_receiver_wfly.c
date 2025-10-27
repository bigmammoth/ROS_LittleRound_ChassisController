#include "rc_receiver_wfly.h"

#define MAX_RECEIVER_CHANNEL_SHIFT  671
#define MID_RECEIVER_CHANNEL_VALUE  1024

void Get_WFLY_Receiver_Values(ReceiverValues_t* receiverValue, S_Bus_Channel_t* receiverChannel)
{
    receiverValue->steering = (float)(MID_RECEIVER_CHANNEL_VALUE - (int16_t)receiverChannel->channelValue[0])\
                            / (float)MAX_RECEIVER_CHANNEL_SHIFT; // Convert to -1 to 1 range
    receiverValue->throttle = (float)((MID_RECEIVER_CHANNEL_VALUE + MAX_RECEIVER_CHANNEL_SHIFT) - (int16_t)receiverChannel->channelValue[2])\
                            / (2 * (float)MAX_RECEIVER_CHANNEL_SHIFT); // Convert to 0 to 1 range
    receiverValue->autoMode = (receiverChannel->channelValue[4] > MID_RECEIVER_CHANNEL_VALUE) ? true : false; // Assuming channel 5 is used for manual/automatic mode
    receiverValue->failSafe = receiverChannel->flagBit_Failsafe;
    receiverValue->frameLost = receiverChannel->flagBit_FrameLost;
}
