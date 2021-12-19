#ifndef __CLIENT_COMM_H__
#define __CLIENT_COMM_H__

#include "EndpointRadarBase.h"
#include "Protocol.h"

#ifdef LOG_TOKEN
#undef LOG_TOKEN
#endif 

#define LOG_TOKEN Module_Comm

#define CLIENT_COMM_STATUS_OK                                   0
#define CLIENT_COMM_STATUS_ERROR_DATA_CALLBACK_NOT_REGISTERED  -9001
#define CLIENT_COMM_STATUS_ERROR_C_COMM_LIB_PROTOCOL_FAILURE   -9002
#define CLIENT_COMM_STATUS_ERROR_COULD_NOT_OPEN_COM_PORT       PROTOCOL_ERROR_COULD_NOT_OPEN_COM_PORT
#define CLIENT_COMM_STATUS_ERROR_SENSOR_BOARD_NOT_COMPATIABLE  PROTOCOL_ERROR_DEVICE_NOT_COMPATIBLE

// Import from EndpointRadarBase.h
typedef Callback_Data_Frame_t  Client_Comm_Callback_Data_Frame_t;
typedef Callback_Temperature_t Client_Comm_Callback_Temperature_t;

/** This C header file defines API which allows client application software
to communicate with Infineon's position2go C communication libary. It 
supports functions to request information from position2go sensor board
and to get information through callback mechanism. It acts like a wrapper
around different endpoints.
*/

// This function searches and connects to connected radar sensor board via com port.
// It returns the identifier for radar board used for further communications with this board.
int32_t clientCommConnectRadar();

// Callback functions
void clientCommRegisterDataCallback(int32_t radarProtocolId, Client_Comm_Callback_Data_Frame_t* callbackFcnReceiveData);
void clientCommRegisterTemperatureCallback(Client_Comm_Callback_Temperature_t* callbackFcnTemperature);
Client_Comm_Callback_Data_Frame_t* clientCommGetDataCallbackFuncPtr();
Client_Comm_Callback_Temperature_t* clientCommGetTempCallbackFuncPtr();


// Data capture functions
int32_t clientCommStartDataCapture(int32_t radarProtocolId, int32_t autoDataCaptureTriggerTimePeriod);
int32_t clientCommStopDataCapture(int32_t radarProtocolId);
int32_t clientCommGetDataFrame(int32_t radarProtocolId, uint32_t wait);

// Device configuration functions
int32_t clientCommStopAutoTrigger(int32_t radarProtocolId);
int32_t clientCommSetFrameFormat(int32_t radarProtocolId);

void clientCommDisconnectRadar();

#endif