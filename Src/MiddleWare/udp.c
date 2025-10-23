/*
 * UDP helper module
 *
 * Purpose:
 *   - Register UDP listeners and dispatch received datagrams to user callbacks.
 *   - Cache the last peer address per socket to enable convenient reply sends.
 *   - Provide simple send helpers for reply (UDP_SendData) and explicit destination (UDP_SendDataTo).
 *
 * Key API:
 *   int  UDP_RegisterListener(uint16_t port, UDP_Callback_t cb);
 *   bool UDP_SendData(int socket, const uint8_t *buf, uint32_t len);
 *   bool UDP_SendDataTo(int socket, const NET_ADDR *addr, const uint8_t *buf, uint32_t len);
 *
 * Operation:
 *   - UdpCallback is attached to each socket; on receive it updates the cached NET_ADDR
 *     and invokes the user-provided callback with the payload.
 *   - Send helpers allocate a transmit buffer via netUDP_GetBuffer and send via netUDP_Send.
 *
 * Limits/Notes:
 *   - Supports up to UDP_CALLBACK_NUMBER simultaneous listener/callback pairs.
 *   - UDP_SendData requires a previously received packet to have populated the cached address.
 *   - Not inherently thread-safe; serialize access if used from multiple contexts.
 *   - On listener open failure, errors are returned as negative netStatus values.
 */
#include "main.h"
#include "udp.h"
#include <string.h>

/* --------------- Data type definitions ----------- */
typedef struct
{
    int socket;              // Socket number
    NET_ADDR receivedAddr;   // Address of the last received message
    uint16_t port;           // Local port number
    UDP_Callback_t callback; // Callback function
} UDP_CallbackEntry_t;

/* --------------- Const value define ---------------*/
#define UDP_CALLBACK_NUMBER 8

/* --------------------- Static variables ---------------- */
static osThreadId_t threadId;
static osMessageQueueId_t messageQueueId;
static UDP_CallbackEntry_t callbackEntries[UDP_CALLBACK_NUMBER];

/* ------------------- static functions ------------ */
static uint32_t UdpCallback(int socket, const NET_ADDR *addr, const uint8_t *buf, uint32_t len);

uint32_t UdpCallback(int socket, const NET_ADDR *addr, const uint8_t *buf, uint32_t len)
{
    if (socket <= 0 || buf == NULL || len == 0)
        return 0;

    // Call the registered callback function
    for (int i = 0; i < UDP_CALLBACK_NUMBER; i++)
    {
        if (callbackEntries[i].socket == socket && callbackEntries[i].callback != NULL)
        {
            // Store the last received address
            memcpy(&callbackEntries[i].receivedAddr, addr, sizeof(NET_ADDR));
            // Call the callback function
            callbackEntries[i].callback(buf, len);
        }
    }

    return len; // Return the length of data processed
}

/**
 * @brief Send out UDP package
 * @param socket UDP socket number
 * @param buff pointer to data buff
 * @param size length of data need to be sent
 * @return false - failed true - success
 */
bool UDP_SendData(int socket, const uint8_t *buff, uint32_t size)
{
    if (socket <= 0)
        return false;

    // Find the callback entry for the given socket
    int i = 0;
    for (; i < UDP_CALLBACK_NUMBER; i++)
        if (callbackEntries[i].socket == socket)
            break;
    if (i >= UDP_CALLBACK_NUMBER)
        // No callback entry found for the socket
        return false;

    // Check if there's a valid address to send data
    if (callbackEntries[i].receivedAddr.port == 0)
        return false; // No valid address to send data

    uint8_t *sendBuff = NULL;
    sendBuff = netUDP_GetBuffer(size);
    if (sendBuff == NULL)
        return false;

    // Send data to the socket
    memcpy(sendBuff, buff, size);
    netStatus nSt = netUDP_Send(socket, &callbackEntries[i].receivedAddr, sendBuff, size);
    if (nSt == netOK)
        return true;
    else
        return false;
}

/**
 * @brief Send out UDP package to a socket
 * @param socket UDP socket number
 * @param addr pointer to NET_ADDR structure, which contains the address and port to send data
 * @param buff pointer to data buff
 * @param size length of data need to be sent
 * @return false - failed true - success
 */
bool UDP_SendDataTo(int socket, const NET_ADDR *addr, const uint8_t *buff, uint32_t size)
{
    if (socket <= 0)
        return false;

    uint8_t *sendBuff = NULL;
    sendBuff = netUDP_GetBuffer(size);
    if (sendBuff == NULL)
        return false;

    memcpy(sendBuff, buff, size);
    netStatus nSt = netUDP_Send(socket, addr, sendBuff, size);
    if (nSt == netOK)
        return true;
    else
        return false;
}

/**
 * @brief Register UDP listener.
 * A port and a callback function will be registered to a socket. When there're
 * any imcoming message to the port, the callback function will be called. After
 * a sucessful register, a socket number will be return to the caller, so the
 * caller can use this socket to send out message.
 * @param port UDP port to listen the incoming messages.
 * 		@arg 0 = system assigned local port.
 * @retval value >=0 : socket handle number
 * 	       value < 0 : error occurred, -value = netStatus.
 */
int UDP_RegisterListener(uint16_t port, UDP_Callback_t callback)
{
    if (callback == NULL || port == 0)
        return -netInvalidParameter;
    int socket = netUDP_GetSocket(UdpCallback);
    if (socket < 0)
        return socket;
    netStatus netSt = netUDP_Open(socket, port);
    if (netOK != netSt)
    {
        netUDP_ReleaseSocket(socket);
        return -netSt;
    }
    else
    {
        int i = 0;
        for (; i < UDP_CALLBACK_NUMBER; i++)
        {
            if (callbackEntries[i].callback == NULL && callbackEntries[i].socket == 0
                && callbackEntries[i].port == 0 && callbackEntries[i].port != port)
            {
                callbackEntries[i].callback = callback;
                callbackEntries[i].socket = socket;
                callbackEntries[i].port = port;
                break;
            }
        }
        if (i >= UDP_CALLBACK_NUMBER)
        {
            netUDP_ReleaseSocket(socket);
            return -netError;
        }
        return socket;
    }
}

/**
 * @brief Get the last received address for a socket.
 * @param socket UDP socket number
 * @param addr pointer to NET_ADDR structure to store the received address
 * @return false - failed true - success
 */
bool UDP_GetReceivedAddress(int socket, NET_ADDR *addr)
{
    if (socket <= 0 || addr == NULL)
        return false;

    // Find the callback entry for the given socket
    int i = 0;
    for (; i < UDP_CALLBACK_NUMBER; i++)
        if (callbackEntries[i].socket == socket)
            break;
    if (i >= UDP_CALLBACK_NUMBER)
        // No callback entry found for the socket
        return false;

    // Check if there's a valid address to send data
    if (callbackEntries[i].receivedAddr.port == 0)
        return false; // No valid address to send data

    memcpy(addr, &callbackEntries[i].receivedAddr, sizeof(NET_ADDR));
    return true;
}

/**
 * @brief Get the socket number for a registered listener by port.
 * @param port UDP port number
 * @return socket number if found, -1 if not found
 */
int UDP_GetListenerSocketByPort(uint16_t port)
{
    for (int i = 0; i < UDP_CALLBACK_NUMBER; i++)
        if (callbackEntries[i].port == port)
            return callbackEntries[i].socket;
    return -1;
}
