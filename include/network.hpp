#ifndef TI_AS_NETWORK_HPP
#define TI_AS_NETWORK_HPP


#include <ArduinoWebsockets.h>
using namespace websockets;


#define MAX_WS_CLIENTS 1
extern WebsocketsClient wsClients[MAX_WS_CLIENTS];
extern bool wsConnectedFlags[MAX_WS_CLIENTS]; 


void network_task(void *param);


#endif