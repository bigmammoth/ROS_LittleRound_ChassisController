#pragma once

#include <stdbool.h>
#include "hardware_config.h"

#ifdef RECEIVER_TYPE_WFLY
#include "./Devices/rc_receiver_wfly.h"
#elif defined(RECEIVER_TYPE_HT8A)
#include "./Devices/rc_receiver_ht8a.h"
#endif

void RC_Receiver_Init(void);
ReceiverValues_t Receiver_Read(void);
