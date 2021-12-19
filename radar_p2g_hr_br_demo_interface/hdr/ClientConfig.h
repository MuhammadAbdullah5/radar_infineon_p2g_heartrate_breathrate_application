#ifndef __CLIENT_CONFIG_H__
#define __CLIENT_CONFIG_H__

#include <stdint.h>

#define MIN_RX_ANTENNAS           (1)
#define MAX_RX_ANTENNAS           (2)
#define MAX_RANGE_SAMPLES         (128)
#define MIN_RANGE_SAMPLES         (16)
#define MIN_DOPPLER_SAMPLES       (16)
#define MAX_DOPPLER_SAMPLES       (600)
#define MIN_PULSE_REP_INTRVL_MS   (0.0023)
#define MAX_PULSE_REP_INTRVL_MS   (200.0)
#define MIN_PLL_PULSE_REP_INTRVL_USEC (100)
#define MAX_PLL_PULSE_REP_INTRVL_USEC (1500)

#define MAX_FFT_LENGTH      (2048)
#define MAX_NUM_TARGETS_RUN (20)

typedef struct complexFloat_s
{
	float re;
	float im;
} complexFloat_t;

typedef enum {
	DATA_CAPTURE_MODE_MULTI_FRAME_SINGLE_CHIRP,
	DATA_CAPTURE_MODE_SINGLE_FRAME_MULTI_CHIRP,
	DATA_CAPTURE_MODE_INVALID
} DataCaptureMode_t;


typedef enum {
	FOURIER_SPECTRUM_INTERP_FACTOR_1,
	FOURIER_SPECTRUM_INTERP_FACTOR_2,
	FOURIER_SPECTRUM_INTERP_FACTOR_4,
	FOURIER_SPECTRUM_INTERP_FACTOR_8
} FourierSpectrumInterpFactor_t;

typedef enum TimeDomainWindowType_e
{
	WINDOW_BLACK_MAN,
	WINDOW_KAISER_ALPHA_5,
	WINDOW_KAISER_ALPHA_10,
	WINDOW_KAISER_ALPHA_15,
	WINDOW_INVALID
} TimeDomainWindowType_t;

typedef enum
{
	SAVE_NOTHING,
	SAVE_RADAR_DATA,
	SAVE_BACKGROUND_DATA,
	SAVE_TXRX_LEAKAGE
} SaveDataType_t;

#define vARIABLE_LIST(cONV) \
	cONV(float  ,                carrierFreqHz)         \
	cONV(float  ,                radarBandwidthHz)		\
	cONV(float  ,                pulseRepIntervalSec)	\
	cONV(float  ,                mtiFilterAlpha)		\
	cONV(float  ,                percentFftThreshold)	\
	cONV(int32_t,                numRxAntennas)		    \
	cONV(int32_t,                numChirpsTotal)		\
	cONV(int32_t,                numChirpsPerFrame)	    \
	cONV(int32_t,                numSamplesPerChirp)	\
	cONV(int32_t,                rangeFftLength)		\
	cONV(int32_t,                dopplerFftLength)		\
	cONV(TimeDomainWindowType_t, timeDomainWindow)		\
	cONV(SaveDataType_t,         dataSaveType)			\
	// TODO: may add more params here

/** This function should be called as part of 
* application closing procedure. This would 
* ensure to erase any internal state before
* next run of application can be called in exist-
* ing simulation.
*/
void clientConfigClear();


// Getter functions
// This allows access to read-only parameters
// from ClientConfig module. This means that 
// additional memory allocation in calling
// modules is by choice.
const float  * clientConfigGetCarrierFreqHz();
const float  * clientConfigGetRadarBandwidthHz();
const float  * clientConfigGetPulseRepIntervalSec();
const int32_t* clientConfigGetNumRxAntennas();
const int32_t* clientConfigGetNumChirpsTotal();
const int32_t* clientConfigGetNumChirpsPerFrame();
const int32_t* clientConfigGetNumSamplesPerChirp();
const int32_t* clientConfigGetRangeFftLength();
const int32_t* clientConfigGetDopplerFftLength();
const float  * clientConfigGetPercentFftThreshold();
const float  * clientConfigGetMtiFilterAlpha();
const TimeDomainWindowType_t* clientConfigGetTimeDomainWindow();
const SaveDataType_t* clientConfigGetDataSaveType();

// Setter functions
// Call these functions to set parameters in ClientConfig
// module such that they can be further used throughout
// application.
void clientConfigSetCarrierFreqHz(float carrierFreqHz);
void clientConfigSetRadarBandwidthHz(float radarBandwidth);
void clientConfigSetPulseRepIntervalSec(float pulseRepIntervalUsec);
void clientConfigSetNumRxAntennas(int32_t numRxAntennas);
void clientConfigSetNumChirpsTotal(int32_t numChirpsTotal);
void clientConfigSetNumChirpsPerFrame(int32_t numChirpsPerFrame);
void clientConfigSetNumSamplesPerChirp(int32_t numSamplesPerChirp);
void clientConfigSetRangeFftLength(int32_t rangeFftLen);
void clientConfigSetDopplerFftLength(int32_t dopplerFftLen);
void clientConfigSetPercentFftThreshold(float fftThresholdPercent);
void clientConfigSetMtiFilterAlpha(float mtiFilterAlpha);
void clientConfigSetTimeDomainWindow(TimeDomainWindowType_t windowType);
void clientConfigSetDataSaveType(SaveDataType_t dataSaveType);

#endif //__CLIENT_CONFIG_H__