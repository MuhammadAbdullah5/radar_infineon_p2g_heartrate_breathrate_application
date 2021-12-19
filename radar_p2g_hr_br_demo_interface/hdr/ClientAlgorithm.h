#ifndef __CLIENT_ALGORITHM_H__
#define __CLIENT_ALGORITHM_H__

#include <stdbool.h>
#include <ClientConfig.h>
#include <ClientComm.h>

#ifdef LOG_TOKEN
#undef LOG_TOKEN
#endif 

#define LOG_TOKEN  Module_Algorithm

#define PROCESSING_REAL_TIME_MODE_EN

typedef struct clientAlgorithmTargetResult_s
{
	float spectralMag;
	float rangeMeters;
	float veloctityMps;
} clientAlgorithmTargetResult_t;


void clientAlgorithmInit(int32_t radarProtocolId);
void clientAlgorithmCaptureData(int32_t radarProtocolId);
void clientAlgorithmReadWindow();
void clientAlgorithmReadTxRxLeakageData();
void clientAlgorithmReadBackgroundData();
bool clientAlgorithmIsDataCollected();
void clientAlgorithmRunPreProcess();
void clientAlgorithmRunMainProcess();
void clientAlgorithmRunPostProcess();




#endif //__CLIENT_ALGORITHM_H__