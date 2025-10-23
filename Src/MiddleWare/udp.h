#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "rl_net.h"                     // Keil.MDK-Plus::Network:CORE

typedef void (*UDP_Callback_t)(const uint8_t* data, uint32_t size);
bool UDP_SendData(int socket, const uint8_t* buff, const uint32_t size);
bool UDP_SendDataTo(int socket, const NET_ADDR* addr, const uint8_t* buff, uint32_t size);
int UDP_RegisterListener(uint16_t port, UDP_Callback_t callback);
int UDP_GetListenerSocketByPort(uint16_t port);
bool UDP_GetReceivedAddress(int socket, NET_ADDR *addr);
