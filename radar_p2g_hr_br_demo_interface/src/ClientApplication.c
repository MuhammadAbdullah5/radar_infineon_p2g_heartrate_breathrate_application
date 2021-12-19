
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ClientTimer.h>
#include "ClientAlgorithm.h"
#include <ClientConfig.h>
#include "ClientComm.h"
#include "ClientSave.h"
#include "ClientLog.h"

#include "ClientApplication.h"

#define FLOAT_PRECISION 0.000003
//#define PROCESSING_REAL_TIME_MODE_EN
static int32_t s_chirpTotal = 0;
static int32_t s_chirpCounter = 0;

typedef enum {
	APP_STATE_IDLE,
	APP_STATE_INIT,
	APP_STATE_WAIT_FOR_DATA_COMPLETION,
	APP_STATE_PRE_PROCESS,
	APP_STATE_PROCESS,
	APP_STATE_POST_PROCESS,
	APP_STATE_FRAME_DONE,
	APP_STATE_INVALID
} ClientApplicationState_t;

static int32_t s_cpiTimerId = 0;
static int32_t s_priTimerId = 0;
static int32_t s_reps;

static ClientApplicationConfiguration_t *s_pAppConfiguration = NULL;
static ClientApplicationState_t s_appState = APP_STATE_IDLE;

static int32_t s_radarCommProtocolId;
static int32_t s_numProcessedFrames = 0;

static void storeAppParamsToClientConfig(const ClientApplicationConfiguration_t* pAppConfiguration)
{
	if (pAppConfiguration->dataCaptureMode != CLIENT_APP_DATA_CAPTURE_MODE_MULTI_FRAME_SINGLE_CHIRP)
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Only single chirp per frame is supported at the moment. Mode=%d",
			pAppConfiguration->dataCaptureMode);
	}

	// Convert application interface parameter to internally defined parameter format
	int32_t numberOfChirps;
	int32_t rangeFftLength;
	int32_t dopplerFftLength;
	int32_t numberOfChirpsPerFrame;
	int32_t numberOfSamplesPerChirp;
	float pulseRepititionIntervalSec;
	TimeDomainWindowType_t configWindowType = WINDOW_INVALID;
	SaveDataType_t dataSaveType = SAVE_NOTHING;

	pulseRepititionIntervalSec = (1.0 / pAppConfiguration->maxDopplerSamplingFrequencyHz);
	numberOfChirps = floor(pAppConfiguration->maxDopplerSamplingFrequencyHz / pAppConfiguration->dopplerFrequencyResolutionHz);
	numberOfChirpsPerFrame = 1; // TODO: MABD hardcoded at the moment. multi chirp multi frame capture mode not supported yet.
	numberOfSamplesPerChirp = floor(4.0 * pAppConfiguration->maximumRangeMeters * pAppConfiguration->radarIfBandwidthHz / 3.0e8);
	rangeFftLength = (int32_t)pow(2.0, ceil(log2(numberOfSamplesPerChirp)));
	dopplerFftLength = (int32_t)pow(2.0, ceil(log2(numberOfChirps)));

	switch (pAppConfiguration->timeDomainWindowType)
	{
	case CLIENT_APP_WINDOW_BLACK_MAN:       configWindowType = WINDOW_BLACK_MAN;       break;
	case CLIENT_APP_WINDOW_KAISER_ALPHA_5:  configWindowType = WINDOW_KAISER_ALPHA_5;  break;
	case CLIENT_APP_WINDOW_KAISER_ALPHA_10: configWindowType = WINDOW_KAISER_ALPHA_10; break;
	case CLIENT_APP_WINDOW_KAISER_ALPHA_15: configWindowType = WINDOW_KAISER_ALPHA_15; break;
	default:
		CLIENT_LOG_FATAL(LOG_TOKEN, "Window type is invalid. Please select a valid window type.");
		break;
	}

	// TODO: MABD
	// put limits on sampling frequency based on capture mode

	if ((pAppConfiguration->mtiFilterAlpha < 0) ||
		(pAppConfiguration->mtiFilterAlpha > 1)
		)
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Alpha for MTI filtering should have a numeric value between 0 and 1 inclusive.");
	}

	if ((pAppConfiguration->numRxAntennas < MIN_RX_ANTENNAS) ||
		(pAppConfiguration->numRxAntennas > MAX_RX_ANTENNAS)
		)
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Number of receive antennas should 1 or 2.");
	}

	if (((numberOfChirpsPerFrame * numberOfChirps) < MIN_DOPPLER_SAMPLES) ||
		((numberOfChirpsPerFrame * numberOfChirps) > MAX_DOPPLER_SAMPLES)
		)
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Number of samples for doppler frequency detection must lie within range, [%d,%d]",
			MIN_DOPPLER_SAMPLES, MAX_DOPPLER_SAMPLES);
	}

	if ((numberOfSamplesPerChirp < MIN_RANGE_SAMPLES) ||
		(numberOfSamplesPerChirp > MAX_RANGE_SAMPLES)
		)
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Number of samples for beat frequency (range) detection must lie within range, [%d,%d]",
			MIN_RANGE_SAMPLES, MAX_RANGE_SAMPLES);
	}

	if (((pulseRepititionIntervalSec * 1.0e3) < (MIN_PULSE_REP_INTRVL_MS - FLOAT_PRECISION)) ||
		((pulseRepititionIntervalSec * 1.0e3) > (MAX_PULSE_REP_INTRVL_MS + FLOAT_PRECISION))
		)
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Pulse repetition interval must lie within range, [%5.4f,%5.4f] milliseconds seconds",
			MIN_PULSE_REP_INTRVL_MS, MAX_PULSE_REP_INTRVL_MS);
	}

	switch (pAppConfiguration->rangeSpectrumInterpFactor)
	{
	case FOURIER_SPECTRUM_INTERP_FACTOR_1: rangeFftLength <<= 0; break;
	case FOURIER_SPECTRUM_INTERP_FACTOR_2: rangeFftLength <<= 1; break;
	case FOURIER_SPECTRUM_INTERP_FACTOR_4: rangeFftLength <<= 2; break;
	case FOURIER_SPECTRUM_INTERP_FACTOR_8: rangeFftLength <<= 3; break;
	default: CLIENT_LOG_FATAL(LOG_TOKEN, "Invalid value for parameter rangeSpectrumInterpFactor in application interface parameters."); break;
	}

	switch (pAppConfiguration->dopplerSpectrumInterpFactor)
	{
	case FOURIER_SPECTRUM_INTERP_FACTOR_1: dopplerFftLength <<= 0; break;
	case FOURIER_SPECTRUM_INTERP_FACTOR_2: dopplerFftLength <<= 1; break;
	case FOURIER_SPECTRUM_INTERP_FACTOR_4: dopplerFftLength <<= 2; break;
	case FOURIER_SPECTRUM_INTERP_FACTOR_8: dopplerFftLength <<= 3; break;
	default: CLIENT_LOG_FATAL(LOG_TOKEN, "Invalid value for parameter dopplerSpectrumInterpFactor in application interface parameters."); break;
	}

	switch (pAppConfiguration->runMode)
	{
	case CLIENT_APP_RUN_MODE_FMCW_RADAR_SAVE_TX_RX_DATA:      dataSaveType = SAVE_TXRX_LEAKAGE;    break;
	case CLIENT_APP_RUN_MODE_FMCW_RADAR_SAVE_BACKGROUND_DATA: dataSaveType = SAVE_BACKGROUND_DATA; break;
	case CLIENT_APP_RUN_MODE_FMCW_RADAR_SAVE_DATA:            dataSaveType = SAVE_RADAR_DATA;      break;
	case CLIENT_APP_RUN_MODE_FMCW_RADAR_PROCESS:              dataSaveType = SAVE_NOTHING;         break;
	default: CLIENT_LOG_FATAL(LOG_TOKEN, "Invalid value for parameter runMode in application interface parameters."); break;
	}

	// Save to configuration manager
	clientConfigSetDataSaveType(dataSaveType);
	clientConfigSetRangeFftLength(rangeFftLength);
	clientConfigSetDopplerFftLength(dopplerFftLength);
	clientConfigSetTimeDomainWindow(configWindowType);
	clientConfigSetCarrierFreqHz(pAppConfiguration->radarRfCarrierFreqHz);
	clientConfigSetRadarBandwidthHz(pAppConfiguration->radarIfBandwidthHz);
	clientConfigSetMtiFilterAlpha(pAppConfiguration->mtiFilterAlpha);
	clientConfigSetNumRxAntennas(pAppConfiguration->numRxAntennas);
	clientConfigSetNumChirpsPerFrame(numberOfChirpsPerFrame);
	clientConfigSetNumChirpsTotal(numberOfChirps);
	clientConfigSetNumSamplesPerChirp(numberOfSamplesPerChirp);
	clientConfigSetPulseRepIntervalSec(pulseRepititionIntervalSec);
	clientConfigSetPercentFftThreshold(pAppConfiguration->fftThresholdPercent);

}

/** This function is intended to perform initialization tasks
* for client's application. These include establishing connection with radar sensor board,
* registering callbacks in Infineon's C communication library, initializing plotting and data 
* storing module.
*/
void appConfig(const ClientApplicationConfiguration_t* pAppConfiguration)
{
	// Do one time tasks here...
	s_appState = APP_STATE_INIT;

	storeAppParamsToClientConfig(pAppConfiguration);

	// Open connection with radar sensor board
	s_radarCommProtocolId = clientCommConnectRadar();
	if (s_radarCommProtocolId == CLIENT_COMM_STATUS_ERROR_COULD_NOT_OPEN_COM_PORT)
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "ClientComm could not open com port to communicate "
			"with sensor board. Check if board is connected to PC and try again.");
	}
	if (s_radarCommProtocolId == CLIENT_COMM_STATUS_ERROR_SENSOR_BOARD_NOT_COMPATIABLE)
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "ClientComm sensor board not compatible with C comm library "
			"Check firmware version and compatibility of on-board firmware.");
	}

	// Set automatic frame capture trigger
	if (clientCommSetFrameFormat(s_radarCommProtocolId) != 0)
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Application could not configure frame format on sensor board's firmware.");
	}
	
	// Initialize graphics library
	// TODO: MABD

	// Initialize data saving module
	clientSaveInit();
	clientAlgorithmReadWindow();

	// Initialize timer module
	clientTimerInit();

	s_priTimerId = 0;
	s_cpiTimerId = 0;
}

/** This function runs main finite state machine for client's application. 
This is supposed to execute recurrent operations like fetching periodic frame data
from connected radar sensor board, processing, plotting and storing based on pre-designed
architecture. This function can be executed indefinitely in a while loop.
*/
void appRun(int32_t numFrames)
{
	// To enter a valid simulation scenario
	// app_init() should be called first
	// with simulation parameters
	SaveDataType_t saveType = *clientConfigGetDataSaveType();

	bool endSim = false;
	while (endSim != true)
	{
		switch (s_appState)
		{
		case APP_STATE_IDLE:
		{
			if (s_numProcessedFrames == numFrames)
			{
				CLIENT_LOG_PRINT(LOG_TOKEN, "Simulation run completed.");
			}
			else if (s_numProcessedFrames == 0)
			{
				CLIENT_LOG_PRINT(LOG_TOKEN, "Simulation wasn't started. Consider calling appInit() before calling appRun()");
			}	
			else
			{
				CLIENT_LOG_FATAL(LOG_TOKEN, "Simulation couldn't complete. Unknown reason.");
			}
			endSim = true;
			break;
		}
		case APP_STATE_INIT:
		{
			// re-initialize algorithm for one frame data's processing
			clientAlgorithmInit(s_radarCommProtocolId);

			// Start data capture routine
			int32_t retStatus = clientCommStartDataCapture(s_radarCommProtocolId, *clientConfigGetPulseRepIntervalSec() * 1.0e6);
			if (retStatus != 0)
			{
				CLIENT_LOG_FATAL(LOG_TOKEN, "Could not start data capture procedure.");
			}

			s_reps = 0;

			// Start timer for inter-chirp and frame time computations
			if (s_priTimerId)
			{
				// reset if such a timer already exists
				clientTimerReset(s_priTimerId);
			}
			else
			{
				// create and start timer if no such timer exists
				s_priTimerId = clientTimerStart();
			}

			if (s_cpiTimerId)
			{
				// reset if such a timer already exists
				clientTimerReset(s_cpiTimerId);
			}
			else
			{
				// create and start timer if no such timer exists
				s_cpiTimerId = clientTimerStart();
			}

			// next state
			s_appState = APP_STATE_WAIT_FOR_DATA_COMPLETION;
			break;
		}
		case APP_STATE_WAIT_FOR_DATA_COMPLETION:
		{
			// Get data
			clientAlgorithmCaptureData(s_radarCommProtocolId);

			// Get elapsed time to find out actual pulse repetition interval
			double priActual = clientTimerInterval(s_priTimerId);
			s_reps++;
			CLIENT_LOG_PRINT(LOG_TOKEN, "Pulse Repetition Time: %5.6f seconds, Pulse # %3d/%3d", 
				priActual, s_reps, *clientConfigGetNumChirpsTotal());

#if 0
			float priTheoratical = *clientConfigGetPulseRepIntervalSec();
			if (abs(priActual - priTheoratical) > 0.005) // difference of 3 milliseconds is not tolerable
			{
				CLIENT_LOG_FATAL(LOG_TOKEN, "Pulse repeition interval from board is invalid.");
			}
#endif

			// Process if data capture is complete
			if (clientAlgorithmIsDataCollected())
			{
				// If running in non-processing mode i.e. for collection of background data 
				// then processing real time flag is ignored anyway.
				if (saveType == SAVE_NOTHING)
				{
#ifdef PROCESSING_REAL_TIME_MODE_EN
					if (s_chirpCounter == 0)
					{
						clientAlgorithmRunPreProcess();
					}

					s_chirpCounter++;
					if (s_chirpCounter == 10)
					{
						s_chirpCounter = 0;
						s_chirpTotal += 10;
					}
					
					// end the simulation
					if (s_chirpTotal >= ((numFrames - 1)* (*clientConfigGetNumChirpsTotal())))
					{
						s_numProcessedFrames = s_chirpTotal / (*clientConfigGetNumChirpsTotal());
						s_appState = APP_STATE_FRAME_DONE;
					}
#else
					// Stop data capture routine
					int32_t retStatus = clientCommStopDataCapture(s_radarCommProtocolId);
					if (retStatus != 0)
					{
						CLIENT_LOG_FATAL(LOG_TOKEN, "Could not end data capture procedure.");
					}
					s_appState = APP_STATE_PRE_PROCESS;
#endif
				}
				else if (saveType == SAVE_RADAR_DATA)
				{
					// Stop data capture routine
					int32_t retStatus = clientCommStopDataCapture(s_radarCommProtocolId);
					if (retStatus != 0)
					{
						CLIENT_LOG_FATAL(LOG_TOKEN, "Could not end data capture procedure.");
					}
					s_appState = APP_STATE_PRE_PROCESS;
				}
				else
				{
					s_appState = APP_STATE_FRAME_DONE;
				}

			}
			break;
		}
		case APP_STATE_PRE_PROCESS:
		{
			// Get elapsed time to find out coherent processing interval
			double cpi = clientTimerInterval(s_cpiTimerId);
			CLIENT_LOG_PRINT(LOG_TOKEN, "Frame time: %5.6f  seconds, Number of chirps: %3d, Average pulse repetition interval: %5.6f seconds", 
				cpi, s_reps, cpi/ s_reps);

			// Preprocess the received chirp(s)
			if (saveType == SAVE_NOTHING)
			{
				clientAlgorithmRunPreProcess();
			}
			s_appState = APP_STATE_PROCESS;
			break;
		}
		case APP_STATE_PROCESS:
		{
			if (saveType == SAVE_NOTHING)
			{
				clientAlgorithmRunMainProcess();
			}
			s_appState = APP_STATE_POST_PROCESS;
			break;
		}
		case APP_STATE_POST_PROCESS:
		{
			if (saveType == SAVE_NOTHING)
			{
				clientAlgorithmRunPostProcess();
			}
			s_appState = APP_STATE_FRAME_DONE;
			break;
		}
		case APP_STATE_FRAME_DONE:
		{
			s_numProcessedFrames++;

			if (s_numProcessedFrames == numFrames)
			{
				s_appState = APP_STATE_IDLE;
			}
			else
			{
				s_appState = APP_STATE_INIT;
			}
			break;
		}
		default:
		{
			CLIENT_LOG_FATAL(LOG_TOKEN, "Application is in invalid state: %d.", s_appState);
			break;
		}
		}
	}
}

/** This function performs closing of any opened files and plotting windows
in order to faciliate a proper shut down of application. 
*/

void appClose()
{
	// reset parameters
	s_pAppConfiguration = NULL;
	s_appState = APP_STATE_IDLE;
	s_radarCommProtocolId = -1;
	s_numProcessedFrames = 0;

	// Close figures	
	clientTimerClose();
	clientGraphicsCloseFigures();

	// 
	clientCommDisconnectRadar();
}

