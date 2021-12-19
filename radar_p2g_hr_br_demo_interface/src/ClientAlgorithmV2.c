
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
#include <ClientAlgorithm.h>
#include <ClientTimer.h>

#define MAX_BUFFER_CAPACITY
#define MAX_TARGETS  (10)
#define MAX_BR_PEAKS (10)
#define MAX_HR_PEAKS (10)
#define NUM_CHIRPS_MEAN (4)
#define PI 3.14159265

#define tO_STRING(var) #var

#define timerStart(x) \
	if (x) \
	{\
		clientTimerReset(x);\
	}\
	else\
	{\
		x = clientTimerStart();\
	}

#define timerEnd(x) clientTimerInterval(x)

// For further details on quadratic interpolation of spectral peaks, please see
// https://ccrma.stanford.edu/~jos/sasp/Quadratic_Interpolation_Spectral_Peaks.html
#define QUADRACTIC_INTERPOLATE_LOC(alpha, beta, gamma)  ((float)0.5 * (alpha-gamma) / (alpha - 2*beta + gamma))
#define QUADRACTIC_INTERPOLATE_MAG(alpha, beta, gamma, p)  (beta -	(float)0.25 * (alpha - gamma)*p)

#define ABS(x) sqrt(x.re*x.re + x.im*x.im)
typedef enum
{
	CHIRP_STATUS_IDLE,
	CHIRP_STATUS_PENDING,
	CHIRP_STATUS_COMPLETE
}ReceivedChirpStatus_t;


static int32_t s_dopplerSpectrumAccumIndex = 0;
static int32_t s_dopplerSpectrumAccumCount = 0;

static bool s_spectrumPlottingDone = false;
static bool s_BrPlottingDone = false;
static bool s_HrPlottingDone = false;
static bool s_DdPlottingDone = false;
static bool s_PolarPlottingDone = false;
static int32_t s_chirpCIdx = 0;
static fftwf_plan s_rangeFftPlan;
static fftwf_plan s_dopplerFftPlan;
static fftwf_plan s_dopplerIfftPlan;
static int32_t s_callbackTimer = 0;
static int32_t s_frameReceveidTimerId = 0;
static int32_t s_frameProcessTimerId = 0;
static int32_t s_numChirpsCollected = 0;
static ReceivedChirpStatus_t s_chirpDataStatus = CHIRP_STATUS_IDLE;

// Data will be laid out as
// range samples for chirp 1 rx antenna 1 in space of MAX_RANGE_SAMPLES followed by 
// range samples for chirp 2 rx antenna 1 in space of MAX_RANGE_SAMPLES followed by 
// ...
// range samples for chirp N rx antenna 1 in space of MAX_RANGE_SAMPLES followed by 

static float s_tdwindow [MAX_RANGE_SAMPLES];
static float s_fdwindow [MAX_DOPPLER_SAMPLES];

complexFloat_t s_txRxCoupling   [MAX_RX_ANTENNAS][MAX_RANGE_SAMPLES];
complexFloat_t s_dataBackGround [MAX_RX_ANTENNAS][MAX_RANGE_SAMPLES];

static float          s_rawIqData                [MAX_RX_ANTENNAS * MAX_DOPPLER_SAMPLES * MAX_RANGE_SAMPLES * 2];
static complexFloat_t s_iqData                   [MAX_RANGE_SAMPLES];
static complexFloat_t s_rangeFftBuffer           [MAX_RX_ANTENNAS][MAX_DOPPLER_SAMPLES][MAX_FFT_LENGTH];
static complexFloat_t s_rangeFftOutTempBuffer    [MAX_RX_ANTENNAS][MAX_DOPPLER_SAMPLES][MAX_FFT_LENGTH];
static float          s_rangeDopplerNrgBuffer    [MAX_FFT_LENGTH];
static complexFloat_t s_dopplerFftInTempBuffer   [2][MAX_DOPPLER_SAMPLES];
static complexFloat_t s_dopplerFftBuffer         [MAX_RX_ANTENNAS * 2][MAX_FFT_LENGTH];
static float          s_dopplerFftBufferSpectrumAccum [NUM_CHIRPS_MEAN][MAX_FFT_LENGTH >> 1];
static float          s_dopplerFftBufferSpectrum [MAX_FFT_LENGTH >> 1];
static float          s_dopplerFftBufferSpectrumPlot[MAX_FFT_LENGTH >> 1];
static complexFloat_t s_dopplerIFftInBuffer      [MAX_FFT_LENGTH];
static complexFloat_t s_dopplerIFftBuffer        [MAX_FFT_LENGTH];


static int32_t s_frameCount         = 0;
static int32_t s_numCount           = 0;
static int32_t s_numSpectrumAverage = 4;
static float s_dopplerRangeSpectrumAccumBuffer [4][MAX_RX_ANTENNAS][MAX_FFT_LENGTH >> 1U][MAX_FFT_LENGTH];
static float s_dopplerRangeSpectrumAccum       [MAX_RX_ANTENNAS][MAX_FFT_LENGTH >> 1U][MAX_FFT_LENGTH];

static clientAlgorithmTargetResult_t s_targetResults [MAX_RX_ANTENNAS][MAX_NUM_TARGETS_RUN];
static int32_t s_targetCount [MAX_RX_ANTENNAS] = { 0 };

static fftwf_complex s_fftInMem  [MAX_FFT_LENGTH];
static fftwf_complex s_fftOutMem [MAX_FFT_LENGTH];

void filter1(const float* b, const float* a, int32_t filterLength, const float* in, float* out, int32_t length) {
	const float a0 = a[0];
	const float* a_end = &a[filterLength - 1];
	const float* out_start = out;
	a++;
	out--;
	int32_t m;
	for (m = 0; m < length; m++) {
		const float* b_macc = b;
		const float* in_macc = in;
		const float* a_macc = a;
		const float* out_macc = out;
		float b_acc = (*in_macc--) * (*b_macc++);
		float a_acc = 0;
		while (a_macc <= a_end && out_macc >= out_start) {
			b_acc += (*in_macc--) * (*b_macc++);
			a_acc += (*out_macc--) * (*a_macc++);
		}
		*++out = (b_acc - a_acc) / a0;
		in++;
	}
}

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
	cLIENT_GRAPHICS_PLOT_2D_DATA(dopplerTimeDomainSignalRx1, "float", numChirpsTotal, useLines, "", &xAxisData, &yAxisData, 0, 0, 0);
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
		s_tdwindow[i] = 0.42 - 0.5 * cos((2.0 * 3.14159265 * (float)i)/((float)tdNumSamples - 1.0)) + 0.08 * cos((4.0 * 3.14159265 * (float)i) / ((float)tdNumSamples - 1.0)) + 0.2;
	}
	for (int32_t i = tdNumSamples; i < MAX_RANGE_SAMPLES; i++)
	{
		s_tdwindow[i] = 0.0;
	}

	// Prepare freq domain window  coefficients
	for (int32_t i = 0; i < fdNumSamples; i++)
	{
		s_fdwindow[i] = 0.42 - 0.5 * cos((2.0 * 3.14159265 * (float)i) / ((float)fdNumSamples - 1.0)) + 0.08 * cos((4.0 * 3.14159265 * (float)i) / ((float)fdNumSamples - 1.0)) + 0.2;
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
	memset((float*)s_fftInMem, 0,  sizeof(fftwf_complex) * MAX_FFT_LENGTH);
	memset((float*)s_fftOutMem, 0, sizeof(fftwf_complex) * MAX_FFT_LENGTH);
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
	float rangeBufferSpectrum[MAX_FFT_LENGTH];
	int32_t rangeFftLen     = *clientConfigGetRangeFftLength();
	SaveDataType_t saveType = *clientConfigGetDataSaveType();
	for (int32_t rx = 0; rx < numRxAntennas; rx++)
	{
		int32_t rdIdx = (rx * numSamplesPerChirp) << 1;
		for (int32_t s = 0; s < numSamplesPerChirp; s++)
		{
			if (saveType == SAVE_NOTHING)
			{
				// subtract background data from the current data
				s_iqData[s].re = frame_info->sample_data[s + rdIdx + 0]                  - s_dataBackGround[rx][s].re;
				s_iqData[s].im = frame_info->sample_data[s + rdIdx + numSamplesPerChirp] - s_dataBackGround[rx][s].im;

				// multiply with blackman window
				s_iqData[s].re *= s_tdwindow[s];
				s_iqData[s].im *= s_tdwindow[s];
			}
			else
			{
				// Just store data 
				int linearBufferIndex = rdIdx + ((s_chirpCIdx * numSamplesPerChirp * numRxAntennas + s) << 1);
				s_rawIqData[linearBufferIndex + 0] = frame_info->sample_data[s + rdIdx + 0];
				s_rawIqData[linearBufferIndex + 1] = frame_info->sample_data[s + rdIdx + numSamplesPerChirp];
			}
		}
		if (saveType == SAVE_NOTHING)
		{
			// FFT
			doFft(&s_rangeFftPlan, s_iqData, numSamplesPerChirp, &s_rangeFftBuffer[rx][s_chirpCIdx][0], rangeFftLen);
		}
	}


#if 0
	for (int32_t i = 0; i < rangeFftLen / 2; i++)
	{
		rangeBufferSpectrum[i] = ABS(s_rangeFftBuffer[0][s_chirpCIdx][i]);
	}
	// Plot the breathing signal 
	PlotAxisData_t xAxisData1 = { .start = 0.0,  .delta = 1.0, .end = 127.0 };
	PlotAxisData_t yAxisData2 = { 0.0, 1.00, 60.0 };
	char str1[256];
	sprintf(str1, "Range Spectrum spectrum\n");
	cLIENT_GRAPHICS_PLOT_2D_DATA(rangeBufferSpectrum, "float", 128.0, true, str1, &xAxisData1, &yAxisData2, false, 0);
	if (!s_DdPlottingDone)
	{
		s_DdPlottingDone = true;
		cLIENT_GRAPHICS_SET_PLOT_TITLE(s_rangeDopplerNrgBuffer, "Range spectrum");
		cLIENT_GRAPHICS_SET_X_LABEL(s_rangeDopplerNrgBuffer, "Index");
		cLIENT_GRAPHICS_SET_Y_LABEL(s_rangeDopplerNrgBuffer, "Amplitude");
		cLIENT_GRAPHICS_APPLY_SETTINGS(s_rangeDopplerNrgBuffer);
	}
#endif

	// Move on to next reading.
	s_numChirpsCollected += numChirpsPerFrame;
	s_chirpCIdx          += numChirpsPerFrame;

	if (saveType == SAVE_NOTHING)
	{
		// Normal radar processing
		if (s_chirpCIdx == numChirpsTotal)
		{
			s_chirpDataStatus = CHIRP_STATUS_COMPLETE;
			s_chirpCIdx = 0;
		}
		// batch processing
#ifndef PROCESSING_REAL_TIME_MODE_EN
		else
		{
			s_chirpDataStatus = CHIRP_STATUS_PENDING;
		}
#else
		// real time processing
		else if (s_numChirpsCollected < numChirpsTotal)
		{
			s_chirpDataStatus = CHIRP_STATUS_PENDING;
		}
#endif
	}
	else
	{
		if (s_chirpCIdx == numChirpsTotal)
		{
			s_chirpDataStatus = CHIRP_STATUS_COMPLETE;
			s_chirpCIdx = 0;
		}
		else
		{
			s_chirpDataStatus = CHIRP_STATUS_PENDING;
		}
	}

	// All chirps should be saved in additional memory if file storage
	// is needed. For that purpose copy data from each chirp to buffer
	// and write it to file once at the end of the frame.
	if (s_chirpDataStatus == CHIRP_STATUS_COMPLETE)
	{
		SaveDataType_t saveType = *clientConfigGetDataSaveType();

		bool isComplex = true;
		int dataLen = numSamplesPerChirp * numChirpsTotal * numRxAntennas;
		if (saveType == SAVE_BACKGROUND_DATA)
		{
			float* backgroundData = s_rawIqData;
			cLIENT_SAVE_STORE_DATA_TXT_FILE(backgroundData, "float", dataLen, isComplex, true);
		}
		else if (saveType == SAVE_TXRX_LEAKAGE)
		{
			float* txRxLeakageData = s_rawIqData;
			cLIENT_SAVE_STORE_DATA_TXT_FILE(txRxLeakageData, "float", dataLen, isComplex, true);
		}
		else if(saveType == SAVE_RADAR_DATA)
		{
			float* rawIqData = s_rawIqData;
			cLIENT_SAVE_STORE_DATA_TXT_FILE(rawIqData, "float", dataLen, isComplex, false);
		}
	}

	//double timeElapsed = clientTimerInterval(s_frameReceveidTimerId);
	//CLIENT_LOG_PRINT(LOG_TOKEN, "Elapsed time for received function is : %f", timeElapsed)
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
	s_chirpCIdx = 0;
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

	// initialize the plans
	int32_t rangeFftLen = *clientConfigGetRangeFftLength();
	int32_t dopplerFftLen = *clientConfigGetDopplerFftLength();
	createFftPlan(rangeFftLen,   &s_rangeFftPlan);
	createFftPlan(dopplerFftLen, &s_dopplerFftPlan);
	createIfftPlan(dopplerFftLen, &s_dopplerIfftPlan);
}

void clientAlgorithmCaptureData(int32_t radarProtocolId)
{
#if 0
	if (s_callbackTimer)
	{
		clientTimerReset(s_callbackTimer);
	}
	else
	{
		s_callbackTimer = clientTimerStart();
	}
#endif

	if (clientCommGetDataFrame(radarProtocolId, true))
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Failed attempt to request frame data from radar sensor.");
	}

#if 0
	s_chirpDataStatus = CHIRP_STATUS_PENDING;
#endif
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

	// Read data
	bool dataReadSuccess = cLIENT_SAVE_READ_DATA_TXT_FILE("txRxLeakageData", "float", s_rawIqData, dataLen, true);
	if (dataReadSuccess)
	{
		// Convert to format of static memory buffer And
		// Take mean across the chirps
		int32_t count = 0;
		for (int32_t c = 0; c < numChirpsTotal; c++)
		{
			for (int32_t rx = 0; rx < numRxAntennas; rx++)
			{
				for (int32_t s = 0; s < numSamplesPerChirp; s++)
				{
					if (c == 0)
					{
						s_txRxCoupling[rx][s].re = s_rawIqData[count++];
						s_txRxCoupling[rx][s].im = s_rawIqData[count++];
					}
					else
					{
						s_txRxCoupling[rx][s].re += s_rawIqData[count++];
						s_txRxCoupling[rx][s].im += s_rawIqData[count++];

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
}

void clientAlgorithmReadBackgroundData()
{
	const int32_t numChirpsTotal     = *clientConfigGetNumChirpsTotal();
	const int32_t numRxAntennas      = *clientConfigGetNumRxAntennas();
	const int32_t numSamplesPerChirp = *clientConfigGetNumSamplesPerChirp();

	int32_t dataLen = numSamplesPerChirp * numRxAntennas * numChirpsTotal;
	float* dataPtr = (float*)malloc(sizeof(float) * dataLen * 2);

	// Read data
	cLIENT_SAVE_READ_DATA_TXT_FILE("backgroundData", "float", s_rawIqData, dataLen, true);

	// Convert to format of static memory buffer And
	// Take mean across the chirps
	int32_t count = 0;
	for (int32_t c = 0; c < numChirpsTotal; c++)
	{
		for (int32_t rx = 0; rx < numRxAntennas; rx++)
		{
			for (int32_t s = 0; s < numSamplesPerChirp; s++)
			{
				if (c == 0)
				{
					s_dataBackGround[rx][s].re = s_rawIqData[count++];
					s_dataBackGround[rx][s].im = s_rawIqData[count++];
				}
				else
				{
					s_dataBackGround[rx][s].re += s_rawIqData[count++];
					s_dataBackGround[rx][s].im += s_rawIqData[count++];

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
	//timerStart(s_frameProcessTimerId);
	const int32_t numChirpsPerFrame = *clientConfigGetNumChirpsPerFrame();
	const int32_t numRxAntennas = *clientConfigGetNumRxAntennas();
	const int32_t numSamplesPerChirp = *clientConfigGetNumSamplesPerChirp();
	const TimeDomainWindowType_t windowType = *clientConfigGetTimeDomainWindow();
	const int32_t numChirpsTotal = *clientConfigGetNumChirpsTotal();
	int32_t rangeFftLen = *clientConfigGetRangeFftLength();
	int32_t dopplerFftLen = *clientConfigGetDopplerFftLength();
	const float pulseRepetitionIntervalSec = *clientConfigGetPulseRepIntervalSec();
	rangeFftLen >>= 1;

	// TODO: MABD only one chirp per frame is supported at the moment
	if (numChirpsPerFrame != 1)
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Only one chirp per frame mode is supported at the moment");
	}

	/************************ Target selection ************************/

	// Subtract mean from doppler time domain samples and 
	// Sum their magnitude together
	int32_t chirpStartIdx = s_numChirpsCollected % numChirpsTotal;
	printf("-------------------> %d\n", chirpStartIdx);
	int32_t cIdx = chirpStartIdx;
	for (int32_t rx = 0; rx < numRxAntennas; rx++)
	{
		for (int32_t r = 0; r < rangeFftLen; r++)
		{
			complexFloat_t meanVal = {0.0, 0.0};
			float dopplerNrg = 0.0;
			cIdx = chirpStartIdx;
			for (int32_t c = 0; c < numChirpsTotal; c++)
			{
				meanVal.re += s_rangeFftBuffer[rx][cIdx][r].re;
				meanVal.im += s_rangeFftBuffer[rx][cIdx][r].im;

				// move to previous chirp
				cIdx++;
				if (cIdx == numChirpsTotal)
				{
					cIdx = 0;
				}
			}
			meanVal.re /= numChirpsTotal;
			meanVal.im /= numChirpsTotal;

			cIdx = chirpStartIdx;
			for (int32_t c = 0; c < numChirpsTotal; c++)
			{
				s_rangeFftOutTempBuffer[rx][c][r].re = s_rangeFftBuffer[rx][cIdx][r].re;
				s_rangeFftOutTempBuffer[rx][c][r].im = s_rangeFftBuffer[rx][cIdx][r].im;
				s_rangeFftOutTempBuffer[rx][c][r].re -= meanVal.re;
				s_rangeFftOutTempBuffer[rx][c][r].im -= meanVal.im;

				dopplerNrg += ABS(s_rangeFftOutTempBuffer[rx][c][r]);

				// move to previous chirp
				cIdx++;
				if (cIdx == numChirpsTotal)
				{
					cIdx = 0;
				}
			}
			if (rx == 0)
			{
				s_rangeDopplerNrgBuffer[r] = dopplerNrg;
			}
			else
			{
				s_rangeDopplerNrgBuffer[r] += dopplerNrg;
			}
		}
	}

	// Normalize and find peaks
	float dopplerNrgVectorThreshold = 0.7;
	float maxV = 0.0;
	for (int32_t r = 0; r < rangeFftLen; r++)
	{
		if (s_rangeDopplerNrgBuffer[r] > maxV)
		{
			maxV = s_rangeDopplerNrgBuffer[r];
		}
	}

	for (int32_t r = 0; r < rangeFftLen; r++)
	{
		s_rangeDopplerNrgBuffer[r] /= maxV;
	}

	int32_t targetRangeBins; // limit target to 1
	int32_t targetCount = 0;
	maxV = 0;
	for (int32_t r = 1; r < rangeFftLen - 1; r++)
	{
		if ((s_rangeDopplerNrgBuffer[r] > s_rangeDopplerNrgBuffer[r - 1]) &&
			(s_rangeDopplerNrgBuffer[r] > s_rangeDopplerNrgBuffer[r + 1]) &&
			(s_rangeDopplerNrgBuffer[r] > dopplerNrgVectorThreshold)
			)
		{
			if (s_rangeDopplerNrgBuffer[r] > maxV)
			{
				maxV = s_rangeDopplerNrgBuffer[r];
				targetRangeBins = r;
				targetCount = 1; // TODO: MABD hard code for now...!
			}
		}
	}

#if 0
	// Plot range spectrum of moving objects
	PlotAxisData_t xAxisData1 = { .start = 0.0,  .delta = 1.0, .end = 127.0 };
	PlotAxisData_t yAxisData2 = { 0.0, 0.01, 1.5 };
	char str1[256];
	sprintf(str1, "Doppler spectrum\n");
	cLIENT_GRAPHICS_PLOT_2D_DATA(s_rangeDopplerNrgBuffer, "float", 128.0, true, str1, &xAxisData1, &yAxisData2, false, 0);
	if (!s_DdPlottingDone)
	{
		s_DdPlottingDone = true;
		cLIENT_GRAPHICS_SET_PLOT_TITLE(s_rangeDopplerNrgBuffer, "Range spectrum");
		cLIENT_GRAPHICS_SET_X_LABEL(   s_rangeDopplerNrgBuffer, "Index");
		cLIENT_GRAPHICS_SET_Y_LABEL(   s_rangeDopplerNrgBuffer, "Amplitude");
		cLIENT_GRAPHICS_APPLY_SETTINGS(s_rangeDopplerNrgBuffer);
	}
#endif

	/************************ Doppler Processing ************************/
	float breathingFreqRegionHzStart = 0.15;
	float breathingFreqRegionHzEnd   = 0.6;
	float heartFreqRegionHzStart     = 0.9;
	float heartFreqRegionHzEnd       = 2.5;
	float dfD = 1.0 / (pulseRepetitionIntervalSec * (float)dopplerFftLen);

	int32_t breathingFreqRegionStartIndex = floor((float)breathingFreqRegionHzStart / dfD);
	int32_t breathingFreqRegionEndIndex   = floor((float)breathingFreqRegionHzEnd   / dfD);
	int32_t heartFreqRegionStartIndex     = floor((float)heartFreqRegionHzStart     / dfD);
	int32_t heartFreqRegionEndIndex       = floor((float)heartFreqRegionHzEnd       / dfD);
	int32_t peakWidthSamples =  dopplerFftLen / numChirpsTotal;

	// DC blocker filter coefficients
	float a[] = {1, -0.8};
	float b[] = {1, -1};

	float dopplerFftFilterInTempBufferRe [MAX_DOPPLER_SAMPLES];
	float dopplerFftFilterInTempBufferIm [MAX_DOPPLER_SAMPLES];
	float dopplerFftFilterOutTempBufferRe[MAX_DOPPLER_SAMPLES];
	float dopplerFftFilterOutTempBufferIm[MAX_DOPPLER_SAMPLES];
	for (int32_t t = 0; t < targetCount; t++)
	{

		/************************ Doppler FFT & and Accumulate ************************/
		int32_t rIdx = targetRangeBins;

		/************* Doppler FFT for both antennas************/
		for (int32_t rx = 0; rx < numRxAntennas; rx++)
		{
			// Doppler time domain signal
			for (int32_t c = 0; c < numChirpsTotal; c++)
			{
				dopplerFftFilterInTempBufferRe[c] = s_rangeFftOutTempBuffer[rx][c][rIdx].re;
				dopplerFftFilterInTempBufferIm[c] = s_rangeFftOutTempBuffer[rx][c][rIdx].im;
			}

			// DC removal filtering
			filter1(b, a, 2, dopplerFftFilterInTempBufferRe, dopplerFftFilterOutTempBufferRe, numChirpsTotal);
			filter1(b, a, 2, dopplerFftFilterInTempBufferIm, dopplerFftFilterOutTempBufferIm, numChirpsTotal);

			// Blackman window multiplication
			for (int32_t c = 0; c < numChirpsTotal; c++)
			{
				dopplerFftFilterOutTempBufferRe[c] *= s_fdwindow[c];
				dopplerFftFilterOutTempBufferIm[c] *= s_fdwindow[c];

				s_dopplerFftInTempBuffer[0][c].re = dopplerFftFilterOutTempBufferRe[c];
				s_dopplerFftInTempBuffer[0][c].im = 0.0;
				s_dopplerFftInTempBuffer[1][c].re = dopplerFftFilterOutTempBufferIm[c];
				s_dopplerFftInTempBuffer[1][c].im = 0.0;
				
			}

			// DOPPLER FFT
			doFft(&s_dopplerFftPlan, s_dopplerFftInTempBuffer[0], numChirpsTotal, &s_dopplerFftBuffer[rx * 2 + 0][0], dopplerFftLen);
			doFft(&s_dopplerFftPlan, s_dopplerFftInTempBuffer[1], numChirpsTotal, &s_dopplerFftBuffer[rx * 2 + 1][0], dopplerFftLen);
		}

		
		/************* Accumulate Doppler FFT spectrum  for all channels ************/
		for (int32_t ch = 0; ch < (2 * numRxAntennas); ch++)
		{
			for (int32_t d = 0; d < (dopplerFftLen >> 1); d++)
			{
				if (ch == 0)
				{
					s_dopplerFftBufferSpectrum[d] = ABS(s_dopplerFftBuffer[ch][d]);
				}
				else
				{
					s_dopplerFftBufferSpectrum[d] += ABS(s_dopplerFftBuffer[ch][d]);
				}
			}
		}

		/************* Accumulate Doppler FFT spectrum  For Previous Samples ************/
		// Add together and plot
		for (int32_t d = 0; d < (dopplerFftLen >> 1); d++)
		{
			s_dopplerFftBufferSpectrumAccum[s_dopplerSpectrumAccumIndex][d] = s_dopplerFftBufferSpectrum[d];
		}
		s_dopplerSpectrumAccumIndex++;
		if (s_dopplerSpectrumAccumIndex >= NUM_CHIRPS_MEAN)
		{
			s_dopplerSpectrumAccumIndex = 0;
		}
		if (s_dopplerSpectrumAccumCount < NUM_CHIRPS_MEAN)
		{
			s_dopplerSpectrumAccumCount++;
		}
		if (s_dopplerSpectrumAccumCount == NUM_CHIRPS_MEAN)
		{
			for (int32_t a = 0; a < NUM_CHIRPS_MEAN; a++)
			{
				for (int32_t d = 0; d < (dopplerFftLen >> 1); d++)
				{
					if (a == 0)
					{
						s_dopplerFftBufferSpectrum[d] = s_dopplerFftBufferSpectrumAccum[a][d];
					}
					else
					{
						s_dopplerFftBufferSpectrum[d] += s_dopplerFftBufferSpectrumAccum[a][d];
					}
				}
			}
		}

		/************* Prepare plotting data ************/
		// Spectrum is normalized here
		float maxValue = 0.0;
		for (int32_t d = 0; d < (dopplerFftLen >> 1); d++)
		{
			if (s_dopplerFftBufferSpectrum[d] > maxValue)
			{
				maxValue = s_dopplerFftBufferSpectrum[d];
			}
		}
		for (int32_t d = 0; d < (dopplerFftLen >> 1); d++)
		{
			s_dopplerFftBufferSpectrumPlot[d] = s_dopplerFftBufferSpectrum[d];
			s_dopplerFftBufferSpectrumPlot[d] /= maxValue;
		}

		/************* Prepare buffers for HR and BR ************/
		float brDopplerSpectrummBuffer[MAX_FFT_LENGTH];
		float hrDopplerSpectrummBuffer[MAX_FFT_LENGTH];
		for (int32_t d = 0; d < (dopplerFftLen >> 1); d++)
		{
			brDopplerSpectrummBuffer[d] = s_dopplerFftBufferSpectrum[d];
			hrDopplerSpectrummBuffer[d] = s_dopplerFftBufferSpectrum[d];
		}

		/************* Find peaks in BR region ************/
		// These peaks are initially filtered out for further
		// harmonics analysis

		// Maximum is found for picking up peaks
		float brRegionMaxVal = 0.0;
		float brPeakThreshold = 0.5;
		for (int32_t i = breathingFreqRegionStartIndex; i <= breathingFreqRegionEndIndex; i++)
		{
			if (brDopplerSpectrummBuffer[i] > brRegionMaxVal)
			{
				brRegionMaxVal = brDopplerSpectrummBuffer[i];
			}
		}
		brPeakThreshold *= brRegionMaxVal;

		// Find out peaks within heart region of doppler spectrum
		float hrRegionMaxVal = 0.0;
		float hrPeakThreshold = 0.1;
		for (int32_t i = heartFreqRegionStartIndex; i <= heartFreqRegionEndIndex; i++)
		{
			if (hrDopplerSpectrummBuffer[i] > hrRegionMaxVal)
			{
				hrRegionMaxVal = hrDopplerSpectrummBuffer[i];
			}
		}
		hrPeakThreshold *= hrRegionMaxVal;


		// Find out peaks within breathing region of doppler spectrum
		int32_t brRegionPeakList[MAX_BR_PEAKS];
		int32_t brPeakCount = 0;
		for (int32_t i = breathingFreqRegionStartIndex+2; i < breathingFreqRegionEndIndex-2; i++)
		{
			// Check for peak relative to each side 2 neighbors
			float llv  = brDopplerSpectrummBuffer[i - 2];
			float lv   = brDopplerSpectrummBuffer[i - 1];
			float cv   = brDopplerSpectrummBuffer[i    ];
			float rv   = brDopplerSpectrummBuffer[i + 1];
			float rrv  = brDopplerSpectrummBuffer[i + 2];

			if ((cv - lv    > 0) &&
				(lv - llv   > 0) &&
				(cv - rv    > 0) &&
				(rv - rrv   > 0) &&
				(cv > brPeakThreshold)
				)
			{
				// Get noise floor value
				int32_t nfCount=0;
				float nfVal = 0.0;
				for (int32_t l = breathingFreqRegionStartIndex; l < (i - 2); ++l)
				{
					nfVal += brDopplerSpectrummBuffer[l];
						nfCount++;
				}
				for (int32_t l = (i + 2); l < breathingFreqRegionEndIndex; ++l)
				{
					nfVal += brDopplerSpectrummBuffer[l];
					nfCount++;
				}
				nfVal /= nfCount;
				if ((cv / nfVal) > 2.0)
				{
					brRegionPeakList[brPeakCount++] = i;
				}
				if (brPeakCount == MAX_BR_PEAKS)
				{
					break;
				}
			}
		}


		/************************ Heart & Breathing Rate Harmonics Analysis ************************/
		int32_t downSamplingFactor = 1;

		// Extract time domain signal from the heart rate
		float breathingSignal[MAX_FFT_LENGTH];
		float brEstAngle = 180;
		int32_t brEstBin = 0;

		// TODO: MABD
		// Height of breathing peaks is a very good metric for a valid peak 
		bool brPeakFound = brPeakCount > 0 && brPeakCount <= 3  && brRegionMaxVal > hrRegionMaxVal;
		if (brPeakCount)
		{
			int32_t mIndexStart;
			int32_t mIndexEnd;
			int32_t hIndexStart;
			int32_t hIndexEnd;
			int32_t startDiscard = 0;
			int32_t endDiscard = 0;
			for (int32_t p = 0; p < brPeakCount; p++)
			{
				mIndexStart = brRegionPeakList[p] - peakWidthSamples;
				if (mIndexStart < 0)
				{
					startDiscard = -mIndexStart;
					mIndexStart = 0;
				}

				mIndexEnd = brRegionPeakList[p] + peakWidthSamples;
				if (mIndexEnd > ((dopplerFftLen >> 1) - 1))
				{
					endDiscard = mIndexEnd - ((dopplerFftLen >> 1) - 1);
					mIndexEnd = (dopplerFftLen >> 1) - 1;
				}

				for (int32_t h = 2; h <= 3; h++)
				{
					hIndexStart = h * brRegionPeakList[p] - peakWidthSamples;
					hIndexStart = hIndexStart + startDiscard;

					hIndexEnd = h * brRegionPeakList[p] + peakWidthSamples;
					hIndexEnd = hIndexEnd - endDiscard;

					if ((mIndexEnd - mIndexStart) != (hIndexEnd - hIndexStart))
					{
						CLIENT_LOG_FATAL(LOG_TOKEN, "Window size must be equal for harmonics combines");
					}

					for (int32_t i = 0; i < (mIndexEnd - mIndexStart + 1); i++)
					{
						brDopplerSpectrummBuffer[i + mIndexStart] += brDopplerSpectrummBuffer[i + hIndexStart];
					}
				}
			}

			// Find peak with maximum value
			float maxVal = 0.0;
			brEstBin = 0;
			for (int32_t p = 0; p < brPeakCount; p++)
			{
				if (brDopplerSpectrummBuffer[brRegionPeakList[p]] > maxVal)
				{
					brEstBin = brRegionPeakList[p];
					maxVal = brDopplerSpectrummBuffer[brRegionPeakList[p]];
				}
			}

			// Perform IFFT to get breathing rate signal
			// prepare input
			for (int i = 0; i < dopplerFftLen; i++)
			{
				s_dopplerIFftInBuffer[i].re = 0.0;
				s_dopplerIFftInBuffer[i].im = 0.0;
			}

			s_dopplerIFftInBuffer[brEstBin].re = 15.0 / (float)numChirpsTotal;
			s_dopplerIFftInBuffer[(dopplerFftLen >> 1) + brEstBin].im = 0.0;

			doFft(&s_dopplerIfftPlan, s_dopplerIFftInBuffer, dopplerFftLen, s_dopplerIFftBuffer, dopplerFftLen);

			int32_t ci = 0;
			for (int32_t i = 0; i < numChirpsTotal; i += downSamplingFactor)
			{
				breathingSignal[ci++] = s_dopplerIFftBuffer[i].im;
			}

			// Plot the breathing signal 
			PlotAxisData_t xAxisData = { .start = 0.0,  .delta = pulseRepetitionIntervalSec * downSamplingFactor, .end = (float)(numChirpsTotal / downSamplingFactor - 1) * pulseRepetitionIntervalSec * downSamplingFactor };
			PlotAxisData_t yAxisData1 = { -0.5, 0.01, 0.5 };
			char str[256];
			sprintf(str, "Breathing Rate = %4.3f BPM\n", (float)brEstBin * dfD * 60.0);
			cLIENT_GRAPHICS_PLOT_2D_DATA(breathingSignal, "float", numChirpsTotal / downSamplingFactor, true, str, &xAxisData, &yAxisData1, 0, 0, 0);
			if (!s_BrPlottingDone)
			{
				s_BrPlottingDone = true;
				cLIENT_GRAPHICS_SET_PLOT_TITLE(breathingSignal, "Respiration Time Domain Signal");
				cLIENT_GRAPHICS_SET_X_LABEL(breathingSignal, "Time (seconds)");
				cLIENT_GRAPHICS_SET_Y_LABEL(breathingSignal, "Amplitude");
				cLIENT_GRAPHICS_APPLY_SETTINGS(breathingSignal);
			}
		}
		else
		{
			int32_t ci = 0;
			for (int32_t i = 0; i < numChirpsTotal; i += downSamplingFactor)
			{
				breathingSignal[ci++] = 0.0;
			}

			PlotAxisData_t xAxisData = { .start = 0.0,  .delta = pulseRepetitionIntervalSec * downSamplingFactor, .
				end = (float)(numChirpsTotal / downSamplingFactor - 1) * pulseRepetitionIntervalSec * downSamplingFactor };
			PlotAxisData_t yAxisData1 = { -0.5, 0.01, 0.5 };
			char str[256];
			sprintf(str, "Breathing Rate = ---- BPM\n");
			cLIENT_GRAPHICS_PLOT_2D_DATA(breathingSignal, "float", numChirpsTotal / downSamplingFactor, true, str, &xAxisData, &yAxisData1, 0, 0, 0);
			if (!s_BrPlottingDone)
			{
				s_BrPlottingDone = true;
				cLIENT_GRAPHICS_SET_PLOT_TITLE(breathingSignal, "Respiration Time Domain Signal");
				cLIENT_GRAPHICS_SET_X_LABEL(breathingSignal, "Time (seconds)");
				cLIENT_GRAPHICS_SET_Y_LABEL(breathingSignal, "Amplitude");
				cLIENT_GRAPHICS_APPLY_SETTINGS(breathingSignal);
			}
		}

		/****************************** Heart rate region peak finding ******************************/
		int32_t hrRegionPeakList[MAX_HR_PEAKS];
		float   hrRegionPeakValList[MAX_HR_PEAKS];
		float   hrRegionPeakAccumValList[MAX_HR_PEAKS];
		bool hrIsBrHarmonic = false;
		int32_t hrPeakCount = 0;
		for (int32_t i = heartFreqRegionStartIndex + 2; i < heartFreqRegionEndIndex - 2; i++)
		{
			// Check for peak relative to each side 2 neighbors
			float llv = hrDopplerSpectrummBuffer[i - 2];
			float lv = hrDopplerSpectrummBuffer[i - 1];
			float cv = hrDopplerSpectrummBuffer[i];
			float rv = hrDopplerSpectrummBuffer[i + 1];
			float rrv = hrDopplerSpectrummBuffer[i + 2];

			if ((cv - lv > 0) &&
				(lv - llv > 0) &&
				(cv - rv > 0) &&
				(rv - rrv > 0) &&
				(cv > hrPeakThreshold)
				)
			{
				hrIsBrHarmonic = false;
				if (brPeakFound)
				{
					for (int32_t harmonic = 1; harmonic <= 7; harmonic++)
					{
						if ((i >= (harmonic * brEstBin - peakWidthSamples)) &&
							(i <= (harmonic * brEstBin + peakWidthSamples)))
						{
							hrIsBrHarmonic = true;
							break;
						}
					}

					
				}

				// Add to found heart rate list if it's not a breathing rate harmonic
				if (hrIsBrHarmonic == false)
				{
					hrRegionPeakList[hrPeakCount] = i;
					hrRegionPeakValList[hrPeakCount] = cv;
					hrPeakCount++;
					if (hrPeakCount == MAX_HR_PEAKS)
					{
						break;
					}
				}
			}
		}


		// Extract time domain signal from the heart rate
		float heartSignal[MAX_FFT_LENGTH];
		float hrEstAngle = 180;
		int32_t hrEstBin = 0;

		// TODO: MABD
		// Heart rate is the one peak gained most from combining
		// 
		bool hrPeakFound = hrPeakCount > 0 && hrPeakCount <= 12;
		if (hrPeakFound)
		{
			/************************ Heart Rate Analysis ************************/
			// find out peaks within breathing region of doppler spectrum
			// Maximum is found for picking up peaks
			int32_t mIndexStart;
			int32_t mIndexEnd;
			int32_t hIndexStart;
			int32_t hIndexEnd;
			int32_t startDiscard = 0;
			int32_t endDiscard = 0;
			for (int32_t p = 0; p < hrPeakCount; p++)
			{
				mIndexStart = hrRegionPeakList[p] - peakWidthSamples;
				if (mIndexStart < 0)
				{
					startDiscard = -mIndexStart;
					mIndexStart = 0;
				}

				mIndexEnd = hrRegionPeakList[p] + peakWidthSamples;
				if (mIndexEnd > ((dopplerFftLen >> 1) - 1))
				{
					endDiscard = mIndexEnd - ((dopplerFftLen >> 1) - 1);
					mIndexEnd = (dopplerFftLen >> 1) - 1;
				}

				for (int32_t h = 2; h <= 3; h++)
				{
					hIndexStart = h * hrRegionPeakList[p] - peakWidthSamples;
					hIndexStart = hIndexStart + startDiscard;

					hIndexEnd = h * hrRegionPeakList[p] + peakWidthSamples;
					hIndexEnd = hIndexEnd - endDiscard;

					if ((mIndexEnd - mIndexStart) != (hIndexEnd - hIndexStart))
					{
						CLIENT_LOG_FATAL(LOG_TOKEN, "Window size must be equal for harmonics combines");
					}

					for (int32_t i = 0; i < (mIndexEnd - mIndexStart + 1); i++)
					{
						hrDopplerSpectrummBuffer[i + mIndexStart] += hrDopplerSpectrummBuffer[i + hIndexStart];
					}
				}
			}

			// Find peak with maximum value
			float maxVal = 0.0;
			hrEstBin = 0;
			for (int32_t p = 0; p < hrPeakCount; p++)
			{
				if (hrDopplerSpectrummBuffer[hrRegionPeakList[p]] > maxVal)
				{
					hrEstBin = hrRegionPeakList[p];
					maxVal = hrDopplerSpectrummBuffer[hrRegionPeakList[p]];
				}
			}

			// Perform IFFT to get breathing rate signal
			// prepare input
			for (int i = 0; i < (dopplerFftLen >> 1); i++)
			{
				s_dopplerIFftInBuffer[i].re = 0.0;
				s_dopplerIFftInBuffer[i].im = 0.0;
			}

			s_dopplerIFftInBuffer[hrEstBin].re = 15.0 / (float)numChirpsTotal;
			s_dopplerIFftInBuffer[(dopplerFftLen >> 1) + hrEstBin].im = 0.0;

			doFft(&s_dopplerIfftPlan, s_dopplerIFftInBuffer, dopplerFftLen, s_dopplerIFftBuffer, dopplerFftLen);

			int32_t ci = 0;
			for (int32_t i = 0; i < numChirpsTotal / downSamplingFactor; i++)
			{
				heartSignal[i] = s_dopplerIFftBuffer[ci].re;
				ci += downSamplingFactor;
			}

			// Plot the breathing signal 
			PlotAxisData_t xAxisData1 = { .start = 0.0,  .delta = pulseRepetitionIntervalSec * downSamplingFactor, .end = (float)(numChirpsTotal / downSamplingFactor - 1) * pulseRepetitionIntervalSec * downSamplingFactor };
			PlotAxisData_t yAxisData2 = { -0.5, 0.01, 0.5 };
			char str[256];
			sprintf(str, "Heart Rate = %4.3f BPM\n", hrEstBin * dfD * 60.0, chirpStartIdx);
			cLIENT_GRAPHICS_PLOT_2D_DATA(heartSignal, "float", numChirpsTotal / downSamplingFactor, true, str, &xAxisData1, &yAxisData2, 0, 0, 0);
			if (!s_HrPlottingDone)
			{
				s_HrPlottingDone = true;
				cLIENT_GRAPHICS_SET_PLOT_TITLE(heartSignal, "Heart Beat Time Domain Signal");
				cLIENT_GRAPHICS_SET_X_LABEL(heartSignal, "Time (seconds)");
				cLIENT_GRAPHICS_SET_Y_LABEL(heartSignal, "Amplitude");
				cLIENT_GRAPHICS_APPLY_SETTINGS(heartSignal);
			}
		}
		else
		{
			int32_t ci = 0;
			for (int32_t i = 0; i < numChirpsTotal / downSamplingFactor; i++)
			{
				heartSignal[i] = 0.0;
				ci += downSamplingFactor;
			}

			// Plot the breathing signal 
			PlotAxisData_t xAxisData1 = { .start = 0.0,  .delta = pulseRepetitionIntervalSec * downSamplingFactor, .end = (float)(numChirpsTotal / downSamplingFactor - 1) * pulseRepetitionIntervalSec * downSamplingFactor };
			PlotAxisData_t yAxisData2 = { -0.5, 0.01, 0.5 };
			char str[256];
			sprintf(str, "Heart Rate = ---- BPM\n");
			cLIENT_GRAPHICS_PLOT_2D_DATA(heartSignal, "float", numChirpsTotal / downSamplingFactor, true, str, &xAxisData1, &yAxisData2, 0, 0, 0);
			if (!s_HrPlottingDone)
			{
				s_HrPlottingDone = true;
				cLIENT_GRAPHICS_SET_PLOT_TITLE(heartSignal, "Heart Beat Time Domain Signal");
				cLIENT_GRAPHICS_SET_X_LABEL(heartSignal, "Time (seconds)");
				cLIENT_GRAPHICS_SET_Y_LABEL(heartSignal, "Amplitude");
				cLIENT_GRAPHICS_APPLY_SETTINGS(heartSignal);
			}
		}

		/************************ Spectrum plotting ************************/\
		int32_t lenxPoints = 0;
		char xPointsDesc[2][64];
		int32_t xPoints[2];
		if (brPeakFound)
		{
			xPoints[lenxPoints] = brEstBin;
			sprintf(xPointsDesc[lenxPoints], "BR ");
			lenxPoints++;
		}
		if (hrPeakFound)
		{
			xPoints[lenxPoints] = hrEstBin;
			sprintf(xPointsDesc[lenxPoints], "HR ");
			lenxPoints++;
		}

		float del = 60.0 / (pulseRepetitionIntervalSec * dopplerFftLen);
		float end = (60.0 / pulseRepetitionIntervalSec) * (0.5 - (1.0 / dopplerFftLen));
		PlotAxisData_t time = { .start = 0.0,  .delta = del, .end = end };
		PlotAxisData_t yAxisData = { 0.0, 0.01, 1.5 };
		cLIENT_GRAPHICS_PLOT_2D_DATA(s_dopplerFftBufferSpectrumPlot, "float", (dopplerFftLen >> 1), true, "Doppler Spectrum", &time, &yAxisData, xPoints, lenxPoints, xPointsDesc);
		if (!s_spectrumPlottingDone)
		{
			s_spectrumPlottingDone = true;
			cLIENT_GRAPHICS_SET_PLOT_TITLE(s_dopplerFftBufferSpectrumPlot, "Frequency Spectrum of Heart Beat and Respiration");
			cLIENT_GRAPHICS_SET_X_LABEL(   s_dopplerFftBufferSpectrumPlot, "Beats per Minute (BPM)");
			cLIENT_GRAPHICS_SET_Y_LABEL(   s_dopplerFftBufferSpectrumPlot, "Magnitude");
			cLIENT_GRAPHICS_APPLY_SETTINGS(s_dopplerFftBufferSpectrumPlot);
		}

		if (hrPeakFound == true || brPeakFound == true)
		{
			// Find angle as well for polar plot
			float x = 0.0;
			float y = 0.0;
			for (int32_t c = 0; c < numChirpsTotal; c++)
			{
				complexFloat_t p1 = s_rangeFftOutTempBuffer[0][c][rIdx];
				complexFloat_t p2 = s_rangeFftOutTempBuffer[1][c][rIdx];
				x += (p1.re * p2.re + p1.im * p2.im);
				y += (p1.im * p2.re - p1.re * p2.im);
			}
			float phaseDiff = atan2(y, x);
			if (phaseDiff < 0)
			{
				phaseDiff = phaseDiff + 2 * PI;
			}
			phaseDiff = phaseDiff - PI;
			brEstAngle = asin((phaseDiff * 12.4) / (2 * PI * 6.22)) * 180 / PI;


			// Polar plot for position of person
			if (!s_PolarPlottingDone)
			{
				float humanLocInfo[2];
				humanLocInfo[0] = 0;
				humanLocInfo[1] = 3.5;
				cLIENT_GRAPHICS_PLOT_POLAR_POINTS(humanLocInfo, 1, "");
				s_PolarPlottingDone = true;

				cLIENT_GRAPHICS_SET_PLOT_TITLE(humanLocInfo, "Polar Plot For Target Human Location");
				cLIENT_GRAPHICS_SET_X_LABEL(humanLocInfo, "Distance (m)");
				cLIENT_GRAPHICS_APPLY_SETTINGS(humanLocInfo);
			}
			else
			{
				float humanLocInfo[2];
				humanLocInfo[0] = brEstAngle;
				humanLocInfo[1] = (float)rIdx * 0.375 * (float)numSamplesPerChirp / (float)rangeFftLen;
				cLIENT_GRAPHICS_PLOT_POLAR_POINTS(humanLocInfo, 1, "");
			}
		}
#if 0
		double processing_time = timerEnd(s_frameProcessTimerId);
		CLIENT_LOG_PRINT(LOG_TOKEN, "Frame processing took %f seconds", processing_time);
#endif

	}	// target information
}

void clientAlgorithmRunMainProcess()
{
}

void clientAlgorithmRunPostProcess()
{
	// Declare data capture state as IDLE.
	//s_chirpDataStatus == CHIRP_STATUS_IDLE;
}
