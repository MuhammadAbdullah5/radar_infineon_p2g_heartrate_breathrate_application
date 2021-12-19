
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "fftw3.h"

#include <ClientSave.h>
#include <ClientComm.h>
#include <ClientGraphics.h>
#include <ClientConfig.h>
#include <ClientLog.h>
//#include <ClientAlgorithm.h>
#include <ClientTimer.h>

#define MAX_BUFFER_CAPACITY

#define tO_STRING(var) #var

// For further details on quadratic interpolation of spectral peaks, please see
// https://ccrma.stanford.edu/~jos/sasp/Quadratic_Interpolation_Spectral_Peaks.html
#define QUADRACTIC_INTERPOLATE_LOC(alpha, beta, gamma)  ((float)0.5 * (alpha-gamma) / (alpha - 2*beta + gamma))
#define QUADRACTIC_INTERPOLATE_MAG(alpha, beta, gamma, p)  (beta -	(float)0.25 * (alpha - gamma)*p)


typedef enum
{
	CHIRP_STATUS_IDLE,
	CHIRP_STATUS_PENDING,
	CHIRP_STATUS_COMPLETE
}ReceivedChirpStatus_t;


static int32_t s_frameReceveidTimerId = 0;
static int32_t s_numChirpsCollected = 0;
static ReceivedChirpStatus_t s_chirpDataStatus = CHIRP_STATUS_IDLE;

// Data will be laid out as
// range samples for chirp 1 rx antenna 1 in space of MAX_RANGE_SAMPLES followed by 
// range samples for chirp 2 rx antenna 1 in space of MAX_RANGE_SAMPLES followed by 
// ...
// range samples for chirp N rx antenna 1 in space of MAX_RANGE_SAMPLES followed by 

static float s_tdwindow[MAX_RANGE_SAMPLES];
static float s_fdwindow[MAX_DOPPLER_SAMPLES];

complexFloat_t s_txRxCoupling[MAX_RX_ANTENNAS][MAX_RANGE_SAMPLES];
complexFloat_t s_dataBackGround[MAX_RX_ANTENNAS][MAX_RANGE_SAMPLES];

static complexFloat_t s_dataMtiFilter[MAX_RX_ANTENNAS][MAX_RANGE_SAMPLES];
static complexFloat_t s_iqData[MAX_RX_ANTENNAS][MAX_DOPPLER_SAMPLES][MAX_RANGE_SAMPLES]; // 2 double for storage of complex numbers
static complexFloat_t s_rangeFftBuffer[MAX_RX_ANTENNAS][MAX_DOPPLER_SAMPLES][MAX_FFT_LENGTH];
static complexFloat_t s_dopplerFftBuffer[MAX_RX_ANTENNAS][MAX_FFT_LENGTH >> 1U][MAX_FFT_LENGTH];
static complexFloat_t s_dopplerFftTempBuffer[MAX_FFT_LENGTH];
static float          s_dopplerFftBufferSpectrum[MAX_RX_ANTENNAS][MAX_FFT_LENGTH >> 1U][MAX_FFT_LENGTH];

static int32_t s_frameCount         = 0;
static int32_t s_numCount           = 0;
static int32_t s_numSpectrumAverage = 4;
static float s_dopplerRangeSpectrumAccumBuffer[4][MAX_RX_ANTENNAS][MAX_FFT_LENGTH >> 1U][MAX_FFT_LENGTH];
static float s_dopplerRangeSpectrumAccum[MAX_RX_ANTENNAS][MAX_FFT_LENGTH >> 1U][MAX_FFT_LENGTH];

static clientAlgorithmTargetResult_t s_targetResults[MAX_RX_ANTENNAS][MAX_NUM_TARGETS_RUN];
static int32_t s_targetCount[MAX_RX_ANTENNAS] = { 0 };

fftwf_complex s_fftInMem [MAX_FFT_LENGTH];
fftwf_complex s_fftOutMem[MAX_FFT_LENGTH];

static void plotDopplerTimeDomainSignal(
	complexFloat_t *dopplerSignalRx1,
	float pulseRepetitionIntervalSec,
	int32_t numChirpsTotal,
	bool useReal,
	bool useLines,
	const char* title,
	const char* xlabel,
	const char* ylabel
)
{
	PlotAxisData_t xAxisData = { 0.0, pulseRepetitionIntervalSec, pulseRepetitionIntervalSec*(numChirpsTotal-1)};
	PlotAxisData_t yAxisData = { 0.0, 0.0, 0.0 };
	float* dopplerTimeDomainSignalRx1 = (float*)malloc(sizeof(float) * numChirpsTotal);

	for (int32_t db = 0; db < numChirpsTotal; db++) {
		dopplerTimeDomainSignalRx1[db] = useReal ? dopplerSignalRx1[db].re : dopplerSignalRx1[db].im;
	}
	yAxisData.start = -5;
	yAxisData.end   = 5;
	cLIENT_GRAPHICS_PLOT_2D_DATA(dopplerTimeDomainSignalRx1, "float", numChirpsTotal, useLines, "", &xAxisData, &yAxisData);
	cLIENT_GRAPHICS_SET_PLOT_TITLE(dopplerTimeDomainSignalRx1, title);
	cLIENT_GRAPHICS_SET_X_LABEL(dopplerTimeDomainSignalRx1, xlabel);
	cLIENT_GRAPHICS_SET_Y_LABEL(dopplerTimeDomainSignalRx1, ylabel);
	cLIENT_GRAPHICS_APPLY_SETTINGS(dopplerTimeDomainSignalRx1);
	free(dopplerTimeDomainSignalRx1);
}

static void plotRangeDopplerSpectrumRx1(float s_dopplerFftBufferSpectrum[][MAX_FFT_LENGTH >> 1U][MAX_FFT_LENGTH],
	float pulseRepetitionIntervalSec,
	int32_t numSamplesPerChirp,
	float radarBandwidthHz,
	int32_t rangeFftLen,
	int32_t dopplerFftLen,
	const char* title,
	const char* xlabel,
	const char* ylabel
)
{
	// Do plotting of heatmap here
	float deltaRange = (numSamplesPerChirp * 3.0e8) / (2.0 * radarBandwidthHz * rangeFftLen);
	float deltaVel = 1.0 / (pulseRepetitionIntervalSec * dopplerFftLen);
	PlotAxisData_t xAxisData = { -deltaVel * (dopplerFftLen >> 1), deltaVel, deltaVel* (dopplerFftLen >> 1) - deltaVel };
	PlotAxisData_t yAxisData = { 0.0, deltaRange, deltaRange * (rangeFftLen >> 1) - deltaRange };
	float* rangeDopplerSpectrumRx1 = (float*)malloc(sizeof(float) * dopplerFftLen * (rangeFftLen >> 1));

	int count = 0;
	for (int32_t rb = 0; rb < (rangeFftLen >> 1); rb++) {
		for (int32_t db = 0; db < dopplerFftLen; db++) {
			rangeDopplerSpectrumRx1[count++] = s_dopplerFftBufferSpectrum[0][rb][db];
		}
	}
	cLIENT_GRAPHICS_PLOT_3D_DATA(rangeDopplerSpectrumRx1, "float", dopplerFftLen, (rangeFftLen >> 1), true, "", &xAxisData, &yAxisData);
	cLIENT_GRAPHICS_SET_PLOT_TITLE(rangeDopplerSpectrumRx1, title);
	cLIENT_GRAPHICS_SET_X_LABEL(rangeDopplerSpectrumRx1, xlabel);
	cLIENT_GRAPHICS_SET_Y_LABEL(rangeDopplerSpectrumRx1, ylabel);
	cLIENT_GRAPHICS_APPLY_SETTINGS(rangeDopplerSpectrumRx1);
	free(rangeDopplerSpectrumRx1);
}

static void fillWindowData(TimeDomainWindowType_t windowType, int32_t tdNumSamples, int32_t fdNumSamples)
{
	if (windowType != WINDOW_BLACK_MAN)
	{
		CLIENT_LOG_WARNING(LOG_TOKEN, "Only blackman window supported at the moment. Using blackman window.");
	}

	// Prepare time domain window  coefficients

	// Change window type to blackman
	// TODO: MABD support other window as well
	windowType = WINDOW_BLACK_MAN;
	for (int32_t i = 0; i < tdNumSamples; i++)
	{
		s_tdwindow[i] = 0.42 - 0.5 * cos((2.0 * 3.14159265 * (float)i)/((float)tdNumSamples - 1.0)) + 0.08 * cos((4.0 * 3.14159265 * (float)i) / ((float)tdNumSamples - 1.0));
	}
	for (int32_t i = tdNumSamples; i < MAX_RANGE_SAMPLES; i++)
	{
		s_tdwindow[i] = 0.0;
	}

	// Prepare freq domain window  coefficients
	for (int32_t i = 0; i < fdNumSamples; i++)
	{
		s_fdwindow[i] = 0.42 - 0.5 * cos((2.0 * 3.14159265 * (float)i) / ((float)fdNumSamples - 1.0)) + 0.08 * cos((4.0 * 3.14159265 * (float)i) / ((float)fdNumSamples - 1.0));
	}
	for (int32_t i = fdNumSamples; i < MAX_DOPPLER_SAMPLES; i++)
	{
		s_fdwindow[i] = 0.0;
	}

	// cLIENT_SAVE_STORE_DATA_TXT_FILE(s_window, "float", totalSamples, false, false);
}

static void subtractMean(complexFloat_t* inOut, int32_t dataLen)
{
	float meanValueRe = 0.0;
	float meanValueIm = 0.0;

	for (int32_t val = 0; val < dataLen; val++)
	{
		meanValueRe += inOut[val].re;
		meanValueIm += inOut[val].im;
	}
	meanValueRe /= (float)dataLen;
	meanValueIm /= (float)dataLen;

	for (int32_t val = 0; val < dataLen; val++)
	{
		inOut[val].re -= meanValueRe;
		inOut[val].im -= meanValueIm;
	}
}

static void calcFftSpectrum(complexFloat_t* in, int32_t len, float* out, bool calcSqrt, float *maxVal, bool fftShift)
{
	float max = 0.0;
	float absSq = 0.0;
	for (int32_t i = 0; i < len; i++)
	{
		absSq = in[i].re * in[i].re + in[i].im * in[i].im;

		if (calcSqrt)
		{
			absSq = sqrt(absSq);
		}
		if (absSq > max)
		{
			max = absSq;
		}

		if (fftShift)
		{

			if (i < (len >> 1))
			{
				out[i + (len >> 1)] = absSq;
			}
			else
			{
				out[i - (len >> 1)] = absSq;
			}
		}
		else
		{
			out[i] = absSq;
		}
		
	}
	*maxVal = max;
}

static void createFftPlan(int fftLen, fftwf_plan* plan)
{
	*plan = fftwf_plan_dft_1d(fftLen, s_fftInMem, s_fftOutMem, FFTW_FORWARD, FFTW_ESTIMATE);
}

static void createIfftPlan(int fftLen, fftwf_plan* plan)
{
	*plan = fftwf_plan_dft_1d(fftLen, s_fftInMem, s_fftOutMem, FFTW_BACKWARD, FFTW_ESTIMATE);
}

static void destroyFftPlan(fftwf_plan *plan)
{
	fftwf_destroy_plan(*plan);
}


static void doFft(fftwf_plan* plan, complexFloat_t *inTdData, int32_t inLen, complexFloat_t* outFdData, int fftLen)
{
	memcpy((float*)s_fftInMem, (float*)inTdData, sizeof(complexFloat_t) * inLen);
	fftwf_execute(*plan);
	memcpy((float*)outFdData, (float*)s_fftOutMem, sizeof(complexFloat_t) * fftLen);
}

static void receiveCapturedData(
	void* context,
	int32_t protocol_handle,
	uint8_t endpoint,
	const Frame_Info_t* frame_info
)
{
	if (s_frameReceveidTimerId)
	{
		clientTimerReset(s_frameReceveidTimerId);
	}
	else
	{
		s_frameReceveidTimerId = clientTimerStart();
	}
	

	const int32_t numChirpsTotal     = *clientConfigGetNumChirpsTotal();
	const int32_t numChirpsPerFrame  = *clientConfigGetNumChirpsPerFrame();
	const int32_t numRxAntennas      = *clientConfigGetNumRxAntennas();
	const int32_t numSamplesPerChirp = *clientConfigGetNumSamplesPerChirp();

	if ((frame_info->num_chirps != numChirpsPerFrame) ||
		(frame_info->num_rx_antennas != numRxAntennas) ||
		(frame_info->num_samples_per_chirp != numSamplesPerChirp)
		)
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Radar configuration doesn't meet with Application algorithm configuration.")
	}

	for (int32_t c = s_numChirpsCollected; c < (s_numChirpsCollected + numChirpsPerFrame); c++)
	{
		for (int32_t rx = 0; rx < numRxAntennas; rx++)
		{
			int32_t readDataIndex = (numSamplesPerChirp * (c - s_numChirpsCollected) * numRxAntennas + rx * numSamplesPerChirp) << 1; // NOTE: multiply by 2 for reading complex data
			for (int32_t s = 0; s < numSamplesPerChirp; s++)
			{
				// Store data in local buffer for futher processing
				s_iqData[rx][c][s].re = frame_info->sample_data[s + readDataIndex + 0];
				s_iqData[rx][c][s].im = frame_info->sample_data[s + readDataIndex + numSamplesPerChirp];
			}
		}
	}

	// Move on to next reading.
	s_numChirpsCollected += numChirpsPerFrame;
	s_chirpDataStatus = (s_numChirpsCollected == numChirpsTotal) ? CHIRP_STATUS_COMPLETE : CHIRP_STATUS_PENDING;

	double timeElapsed = clientTimerInterval(s_frameReceveidTimerId);

	CLIENT_LOG_PRINT(LOG_TOKEN, "Elapsed time for received function is : %f\n", timeElapsed)

	if (s_chirpDataStatus == CHIRP_STATUS_COMPLETE)
	{
		SaveDataType_t saveType = *clientConfigGetDataSaveType();
		if (saveType != SAVE_NOTHING)
		{
			if (saveType == SAVE_BACKGROUND_DATA)
			{
				bool isComplex = true;
				int dataLen = numSamplesPerChirp * numChirpsTotal * numRxAntennas;
				float* backgroundData = (float*)malloc(dataLen * sizeof(float) * (isComplex ? 2 : 1)); // Name is important
				int32_t count = 0;
				for (int32_t rx = 0; rx < numRxAntennas; rx++)
				{
					for (int32_t c = 0; c < numChirpsTotal; c++)
					{
						for (int32_t s = 0; s < numSamplesPerChirp; s++)
						{
							backgroundData[count++] = s_iqData[rx][c][s].re;
							backgroundData[count++] = s_iqData[rx][c][s].im;
						}
					}
				}
				cLIENT_SAVE_STORE_DATA_TXT_FILE(backgroundData, "float", dataLen, isComplex, true);
				free(backgroundData);
			}
			else if (saveType == SAVE_TXRX_LEAKAGE)
			{
				bool isComplex = true;
				int dataLen = numSamplesPerChirp * numChirpsTotal * numRxAntennas;
				float* txRxLeakageData = (float*)malloc(dataLen * sizeof(float) * (isComplex ? 2 : 1)); // Name is important
				int32_t count = 0;
				for (int32_t rx = 0; rx < numRxAntennas; rx++)
				{
					for (int32_t c = 0; c < numChirpsTotal; c++)
					{
						for (int32_t s = 0; s < numSamplesPerChirp; s++)
						{
							txRxLeakageData[count++] = s_iqData[rx][c][s].re;
							txRxLeakageData[count++] = s_iqData[rx][c][s].im;
						}
					}
				}
				cLIENT_SAVE_STORE_DATA_TXT_FILE(txRxLeakageData, "float", dataLen, isComplex, true);
				free(txRxLeakageData);
			}
		}
	}
}

static void receiveTemperatureData(
	void* context,
	int32_t protocol_handle,
	uint8_t endpoint,
	uint8_t temp_sensor,
	int32_t temperature_001C
)
{
	// printf("Temperature Reading: %f\n", (double)temperature_001C * (float)0.001);
}

void clientAlgorithmInit(int32_t radarProtocolId)
{
	s_numChirpsCollected = 0;
	s_chirpDataStatus = CHIRP_STATUS_IDLE;

	// register callbacks
	if (clientCommGetDataCallbackFuncPtr() == NULL)
	{
		clientCommRegisterDataCallback(radarProtocolId, receiveCapturedData);
	}
	if (clientCommGetTempCallbackFuncPtr() == NULL)
	{
		clientCommRegisterTemperatureCallback(radarProtocolId, receiveTemperatureData);
	}

	SaveDataType_t saveType = *clientConfigGetDataSaveType();
	if (saveType == SAVE_NOTHING)
	{ 
		clientAlgorithmReadTxRxLeakageData();
		clientAlgorithmReadBackgroundData();
	}
}

void clientAlgorithmCaptureData(int32_t radarProtocolId)
{
	if (clientCommGetDataFrame(radarProtocolId, true))
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Failed attempt to request frame data from radar sensor.");
	}
	s_chirpDataStatus == CHIRP_STATUS_PENDING;
}

bool clientAlgorithmIsDataCollected()
{
	if (s_chirpDataStatus == CHIRP_STATUS_COMPLETE)
	{
		return true;
	}
	else if (s_chirpDataStatus == CHIRP_STATUS_PENDING)
	{
		return false;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Algorithm is IDLE. Call clientAlgorithmGetFrameData() to start data capture procedure.");
	}
}

// These functions read the values of variables from text file 
// and store into local memory
void clientAlgorithmReadWindow()
{
	TimeDomainWindowType_t window = *clientConfigGetTimeDomainWindow();
	int32_t timeDomainSamples     = *clientConfigGetNumSamplesPerChirp();
	int32_t freqDomainSamples     = *clientConfigGetNumChirpsTotal();
	fillWindowData(window, timeDomainSamples, freqDomainSamples);
}

void clientAlgorithmReadTxRxLeakageData()
{
	const int32_t numChirpsTotal     = *clientConfigGetNumChirpsTotal();
	const int32_t numRxAntennas      = *clientConfigGetNumRxAntennas();
	const int32_t numSamplesPerChirp = *clientConfigGetNumSamplesPerChirp();

	int32_t dataLen = numSamplesPerChirp * numRxAntennas * numChirpsTotal;
	float*  dataPtr = (float*)malloc(sizeof(float) * dataLen * 2);

	// Read data
	bool dataReadSuccess = cLIENT_SAVE_READ_DATA_TXT_FILE("txRxLeakageData", "float", dataPtr, dataLen, true);
	if (dataReadSuccess)
	{
		// Convert to format of static memory buffer And
		// Take mean across the chirps
		int32_t count = 0;
		for (int32_t rx = 0; rx < numRxAntennas; rx++)
		{
			for (int32_t c = 0; c < numChirpsTotal; c++)
			{
				for (int32_t s = 0; s < numSamplesPerChirp; s++)
				{
					if (c == 0)
					{
						s_txRxCoupling[rx][s].re = dataPtr[count++];
						s_txRxCoupling[rx][s].im = dataPtr[count++];
					}
					else
					{
						s_txRxCoupling[rx][s].re += dataPtr[count++];
						s_txRxCoupling[rx][s].im += dataPtr[count++];

						if (c == (numChirpsTotal - 1))
						{
							s_txRxCoupling[rx][s].re /= numChirpsTotal;
							s_txRxCoupling[rx][s].im /= numChirpsTotal;
						}
					}
				}
			}
		}
	}

	free(dataPtr);
}
void clientAlgorithmReadBackgroundData()
{
	const int32_t numChirpsTotal     = *clientConfigGetNumChirpsTotal();
	const int32_t numRxAntennas      = *clientConfigGetNumRxAntennas();
	const int32_t numSamplesPerChirp = *clientConfigGetNumSamplesPerChirp();

	int32_t dataLen = numSamplesPerChirp * numRxAntennas * numChirpsTotal;
	float* dataPtr = (float*)malloc(sizeof(float) * dataLen * 2);

	// Read data
	cLIENT_SAVE_READ_DATA_TXT_FILE("backgroundData", "float", dataPtr, dataLen, true);

	// Convert to format of static memory buffer And
	// Take mean across the chirps
	int32_t count = 0;
	for (int32_t rx = 0; rx < numRxAntennas; rx++)
	{
		for (int32_t c = 0; c < numChirpsTotal; c++)
		{
			for (int32_t s = 0; s < numSamplesPerChirp; s++)
			{
				if (c == 0)
				{
					s_dataBackGround[rx][s].re = dataPtr[count++];
					s_dataBackGround[rx][s].im = dataPtr[count++];
				}
				else
				{
					s_dataBackGround[rx][s].re += dataPtr[count++];
					s_dataBackGround[rx][s].im += dataPtr[count++];

					if (c == (numChirpsTotal - 1))
					{
						s_dataBackGround[rx][s].re /= numChirpsTotal;
						s_dataBackGround[rx][s].im /= numChirpsTotal;
					}
				}
			}
		}
	}
}

// Do the pre-processing
void clientAlgorithmRunPreProcess() 
{
	const int32_t numChirpsPerFrame = *clientConfigGetNumChirpsPerFrame();
	const int32_t numRxAntennas = *clientConfigGetNumRxAntennas();
	const int32_t numSamplesPerChirp = *clientConfigGetNumSamplesPerChirp();
	const TimeDomainWindowType_t windowType = *clientConfigGetTimeDomainWindow();
	const float mtiFilterAlpha = *clientConfigGetMtiFilterAlpha();
	const int32_t numChirpsTotal = *clientConfigGetNumChirpsTotal();

	// TODO: MABD only one chirp per frame is supported at the moment
	if (numChirpsPerFrame != 1)
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Only one chirp per frame mode is supported at the moment");
	}

	complexFloat_t dataMean[MAX_RX_ANTENNAS][MAX_DOPPLER_SAMPLES] = { 0.0 };
	for (int32_t rx = 0; rx < numRxAntennas; rx++)
	{
		for (int32_t c = 0; c < numChirpsTotal; c++)
		{
			for (int32_t s = 0; s < numSamplesPerChirp; s++)
			{
				// Tx-Rx coupling leakage cancellation
				s_iqData[rx][c][s].re -= s_txRxCoupling[rx][s].re;
				s_iqData[rx][c][s].im -= s_txRxCoupling[rx][s].im;

				// Background scene data cancellation
				//s_iqData[rx][c][s].re -= s_dataBackGround[rx][s].re;
				//s_iqData[rx][c][s].im -= s_dataBackGround[rx][s].im;

				// Calculate mean
				dataMean[rx][c].re += s_iqData[rx][c][s].re;
				dataMean[rx][c].im += s_iqData[rx][c][s].im;
			}
			dataMean[rx][c].re /= (float)numSamplesPerChirp;
			dataMean[rx][c].im /= (float)numSamplesPerChirp;
		}
	}

	// Convert to desired format first and then save
	// For IQ data, we choose format first rx, then
	// pulse and then fast time samples
	bool isComplex = true;
	int dataLen = numSamplesPerChirp * numChirpsTotal * numRxAntennas;
	float* rawIqData = (float*)malloc(dataLen * sizeof(float) * (isComplex ? 2:1)); // Name is important
	int32_t count = 0;
	for (int32_t rx = 0; rx < numRxAntennas; rx++)
	{
		for (int32_t c = 0; c < numChirpsTotal; c++)
		{
			for (int32_t s = 0; s < numSamplesPerChirp; s++)
			{
				rawIqData[count++] = s_iqData[rx][c][s].re;
				rawIqData[count++] = s_iqData[rx][c][s].im;
			}
		}
	}
	cLIENT_SAVE_STORE_DATA_TXT_FILE(rawIqData, "float", dataLen, isComplex, false);
	free(rawIqData);

	// Subtract mean and apply window (if enabled)
	for (int32_t rx = 0; rx < numRxAntennas; rx++)
	{
		for (int32_t c = 0; c < numChirpsTotal; c++)
		{
			for (int32_t s = 0; s < numSamplesPerChirp; s++)
			{
				// subtract mean
				s_iqData[rx][c][s].re -= dataMean[rx][c].re;
				s_iqData[rx][c][s].im -= dataMean[rx][c].im;

				// apply window
				if (windowType != WINDOW_INVALID)
				{
					s_iqData[rx][c][s].re *= s_tdwindow[s];
					s_iqData[rx][c][s].im *= s_tdwindow[s];
				}
			}
		}
	}

	// Background scene data cancellation
	for (int32_t rx = 0; rx < numRxAntennas; rx++)	
	{
		for (int32_t c = 0; c < numChirpsTotal; c++)
		{
			for (int32_t s = 0; s < numSamplesPerChirp; s++)
			{
				float re = s_iqData[rx][c][s].re;
				float im = s_iqData[rx][c][s].im;

				// cancel from current set of fast samples (obtained from current chirp)
				s_iqData[rx][c][s].re -= s_dataMtiFilter[rx][s].re;
				s_iqData[rx][c][s].im -= s_dataMtiFilter[rx][s].im;

				// remember a fraction of current sample
				s_dataMtiFilter[rx][s].re = s_dataMtiFilter[rx][s].re * (1.0 - mtiFilterAlpha);
				s_dataMtiFilter[rx][s].im = s_dataMtiFilter[rx][s].im * (1.0 - mtiFilterAlpha);

				s_dataMtiFilter[rx][s].re += re * mtiFilterAlpha;
				s_dataMtiFilter[rx][s].im += im * mtiFilterAlpha;
			}
		}
	}
 }

void clientAlgorithmRunMainProcess()
{
	const int32_t numRxAnt = *clientConfigGetNumRxAntennas();
	const int32_t numSamplesPerChirp = *clientConfigGetNumSamplesPerChirp();
	const int32_t numChirps = *clientConfigGetNumChirpsTotal();
	const float carrierFreqHz = *clientConfigGetCarrierFreqHz();
	const float radarBandwidthHz = *clientConfigGetRadarBandwidthHz();
	const float pulseRepetitionIntervalSec = *clientConfigGetPulseRepIntervalSec();
	float threshold = *clientConfigGetPercentFftThreshold();

	// Algorithm parameters
	int32_t rangeFftLen = *clientConfigGetRangeFftLength();
	int32_t dopplerFftLen = *clientConfigGetDopplerFftLength();

	fftwf_plan* plan;
	createFftPlan(rangeFftLen, &plan);
	// Take FFTs of fast time samples for all chirps
	for (int32_t rx = 0; rx < numRxAnt; rx++)
	{
		for (int32_t c = 0; c < numChirps; c++)
		{
			doFft(&plan, &s_iqData[rx][c][0], numSamplesPerChirp, &s_rangeFftBuffer[rx][c][0], rangeFftLen);
		}
	}
	destroyFftPlan(&plan);

	createFftPlan(dopplerFftLen, &plan);
	// FFTs of slow time samples across all positive range bins
	float globalPeakValue[MAX_RX_ANTENNAS];
	for (int32_t rx = 0; rx < numRxAnt; rx++)
	{
		globalPeakValue[rx] = 0.0;
	}

	for (int32_t rx = 0; rx < numRxAnt; rx++)
	{
		for (int32_t rb = 0; rb < (rangeFftLen >> 1); rb++)
		{
			float maxAcrossSlowTimeSpectrum = 0.0;
			int ignoreStart = 0;
			for (int32_t c = 0; c < ignoreStart; c++)
			{
				s_dopplerFftTempBuffer[c].re = 0.0;
				s_dopplerFftTempBuffer[c].im = 0.0;
			}
			for (int32_t c = ignoreStart; c < numChirps; c++)
			{
				s_dopplerFftTempBuffer[c] = s_rangeFftBuffer[rx][c][rb];
			}

			// subtract mean
			subtractMean(&s_dopplerFftTempBuffer[0], numChirps);

			// Apply window
			for (int32_t c = 0; c < numChirps; c++)
			{
				s_dopplerFftTempBuffer[c].re *= s_fdwindow[c];
				s_dopplerFftTempBuffer[c].im *= s_fdwindow[c];
			}

			// Take FFT across all chirps
			doFft(&plan, &s_dopplerFftTempBuffer[0], numChirps, &s_dopplerFftBuffer[rx][rb][0], dopplerFftLen);
			calcFftSpectrum(&s_dopplerFftBuffer[rx][rb][0], dopplerFftLen, &s_dopplerFftBufferSpectrum[rx][rb][0], false, &maxAcrossSlowTimeSpectrum, true);
			
			if (maxAcrossSlowTimeSpectrum > globalPeakValue[rx])
			{
				globalPeakValue[rx] = maxAcrossSlowTimeSpectrum;
			}
		}
	}
	destroyFftPlan(&plan);

	// 2D peak finding
	for (int32_t rx = 0; rx < numRxAnt; rx++)
	{
		s_targetCount[rx] = 0;
	}
	
	bool limitExceeded = false;
	for (int32_t rx = 0; rx < 1; rx++)
	{
		for (int32_t rb = 1; rb < (rangeFftLen >> 1)-1; rb++)
		{
			for (int32_t db = 1; db < dopplerFftLen - 1; db++)
			{
				if (s_targetCount[rx] < MAX_NUM_TARGETS_RUN)
				{
					if ((s_dopplerFftBufferSpectrum[rx][rb][db] >= s_dopplerFftBufferSpectrum[rx][rb][db - 1]) &&
						(s_dopplerFftBufferSpectrum[rx][rb][db] >= s_dopplerFftBufferSpectrum[rx][rb][db + 1]) &&
						(s_dopplerFftBufferSpectrum[rx][rb][db] >= s_dopplerFftBufferSpectrum[rx][rb - 1][db]) &&
						(s_dopplerFftBufferSpectrum[rx][rb][db] >= s_dopplerFftBufferSpectrum[rx][rb + 1][db]) &&
						(s_dopplerFftBufferSpectrum[rx][rb][db] >= (globalPeakValue[rx] * threshold * 0.01))
						)
					{
						float peakVal = s_dopplerFftBufferSpectrum[rx][rb][db];
						float prevRangeVal = s_dopplerFftBufferSpectrum[rx][rb-1][db];
						float follRangeVal = s_dopplerFftBufferSpectrum[rx][rb+1][db];
						float prevDoppVal = s_dopplerFftBufferSpectrum[rx][rb][db-1];
						float follDoppVal = s_dopplerFftBufferSpectrum[rx][rb][db+1];

						// do quadractic interpolation on spectral peaks
						float interpRangeFftPeak = QUADRACTIC_INTERPOLATE_LOC(prevRangeVal, peakVal, follRangeVal);
						float interpDopplerFftPeak = QUADRACTIC_INTERPOLATE_LOC(prevDoppVal, peakVal, follDoppVal);

						s_targetResults[rx]->spectralMag = 0.5 * QUADRACTIC_INTERPOLATE_MAG(prevRangeVal, peakVal, follRangeVal, interpRangeFftPeak) +
														   0.5 * QUADRACTIC_INTERPOLATE_MAG(prevDoppVal, peakVal, follDoppVal, interpDopplerFftPeak);

						interpRangeFftPeak += rb;
						interpDopplerFftPeak += db;

						// Calculate distance for target
						s_targetResults[rx]->rangeMeters = interpRangeFftPeak * (3.0e8 * numSamplesPerChirp) / (2 * radarBandwidthHz * rangeFftLen);

						// Convert to two sided spectrum
						interpDopplerFftPeak = interpDopplerFftPeak >= (dopplerFftLen / 2.0) ? (interpDopplerFftPeak - dopplerFftLen) : interpDopplerFftPeak;

						// Calculate velocity of target
						s_targetResults[rx]->veloctityMps = (interpDopplerFftPeak * 3.0e8) / (carrierFreqHz * 2 * dopplerFftLen * pulseRepetitionIntervalSec);
						//s_targetCount[rx]++;
					}
				}
				else
				{
					limitExceeded = true;
					break;
				}
			}
			if (limitExceeded)
			{
				break;
			}
		}
	}
}

void clientAlgorithmRunPostProcess()
{
	const int32_t numRxAnt = *clientConfigGetNumRxAntennas();
	const int32_t numSamplesPerChirp = *clientConfigGetNumSamplesPerChirp();
	const int32_t numChirps = *clientConfigGetNumChirpsTotal();
	int32_t rangeFftLen = *clientConfigGetRangeFftLength();
	int32_t dopplerFftLen = *clientConfigGetDopplerFftLength();
	const float radarBandwidthHz = *clientConfigGetRadarBandwidthHz();
	const float pulseRepetitionIntervalSec = *clientConfigGetPulseRepIntervalSec();


	s_numCount++;
	if (s_numCount > s_numSpectrumAverage)
	{
		s_numCount = s_numSpectrumAverage;
	}
	if (s_frameCount == s_numSpectrumAverage)
	{
		s_frameCount = 0;
	}

	// Store data here
	for (int32_t rx = 0; rx < numRxAnt; rx++)
	{
		for (int32_t rb = 0; rb < (rangeFftLen >> 1); rb++)
		{
			for (int32_t c = 0; c < dopplerFftLen; c++)
			{
				s_dopplerRangeSpectrumAccumBuffer[s_frameCount % s_numSpectrumAverage][rx][rb][c] = s_dopplerFftBufferSpectrum[rx][rb][c];
			}
		}
	}

	// combine data here
	for (int32_t f = 0; f < s_numCount; f++)
	{
		int32_t fidx = f % s_numSpectrumAverage;
		for (int32_t rx = 0; rx < numRxAnt; rx++)
		{
			for (int32_t rb = 0; rb < (rangeFftLen >> 1); rb++)
			{
				for (int32_t c = 0; c < dopplerFftLen; c++)
				{
					if (f == 0)
					{
						s_dopplerRangeSpectrumAccum[rx][rb][c] = s_dopplerRangeSpectrumAccumBuffer[fidx][rx][rb][c];
					}
					else
					{
						s_dopplerRangeSpectrumAccum[rx][rb][c] += s_dopplerRangeSpectrumAccumBuffer[fidx][rx][rb][c];
					}					
				}
			}
		}
	}

	// combine to get resultant average
	s_frameCount++;


	// Plot heat map here 
		// Plot heatmap
	plotRangeDopplerSpectrumRx1(s_dopplerRangeSpectrumAccum, pulseRepetitionIntervalSec, numSamplesPerChirp,
		radarBandwidthHz, rangeFftLen, dopplerFftLen, "2D FFT Heatmap", "Doppler Frequency (Hz)", "Range (meters)");


	// Processing of current frame is finished 
	// Declare data capture state as IDLE.
	s_chirpDataStatus == CHIRP_STATUS_IDLE;
}
