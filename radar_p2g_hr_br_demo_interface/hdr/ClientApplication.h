#ifndef __CLIENT_APPLICATION_H__
#define __CLIENT_APPLICATION_H__

#include <ClientAlgorithm.h>

#ifdef LOG_TOKEN
#undef LOG_TOKEN
#endif 
#define LOG_TOKEN  Module_ClientApplication


/** Two modes of data capture are supported.
* Which allow sampling frequencies to be in two ranges
* i.e. [5 Hz-20 Hz] and [107 Hz-2500 Hz]. First mode can 
* be used for applications such as heart beat and breath
* rate detection while mode can be used for moving person
* or vehicle detection. It should be noted that maximum 
* vehicular velocity supported is 28 km / hour. There 
* is no limit on duration of data capture and it should 
* depend upon upon memory budget for application host.
* 
* First is mode where device is radar sensor device is 
* configured to produce chirps periodically where doppler
* sampling frequency has a range of fs = [5 Hz-20 Hz].
* This is used to capture very small frequencies i.e. 
* upto +-10Hz. 
* 
* Second mode is used when much higher doppler frequencies 
* need to be captured. Following range of intra-frame doppler 
* sampling frequencies is supported [107 Hz-2500 Hz].  There
* is a capture time limit in this mode due to hardware memory
* constraints. It should be noted that maximum frequency
* is selectable within range of [107 Hz-2500] however
* selecting a lesser doppler frequency allows for longer
* data capture durations due to larger sampling interval.
*/

typedef enum {
	CLIENT_APP_RUN_MODE_FMCW_RADAR_SAVE_TX_RX_DATA,
	CLIENT_APP_RUN_MODE_FMCW_RADAR_SAVE_BACKGROUND_DATA,
	CLIENT_APP_RUN_MODE_FMCW_RADAR_SAVE_DATA,
	CLIENT_APP_RUN_MODE_FMCW_RADAR_PROCESS
} ClientApplicationRunMode_t;

typedef enum {
	CLIENT_APP_DATA_CAPTURE_MODE_MULTI_FRAME_SINGLE_CHIRP,
	CLIENT_APP_DATA_CAPTURE_MODE_SINGLE_FRAME_MULTI_CHIRP,
	CLIENT_APP_DATA_CAPTURE_MODE_INVALID
} ClientApplicationDataCaptureMode_t;

typedef enum {
	CLIENT_APP_FOURIER_SPECTRUM_INTERP_FACTOR_1,
	CLIENT_APP_FOURIER_SPECTRUM_INTERP_FACTOR_2,
	CLIENT_APP_FOURIER_SPECTRUM_INTERP_FACTOR_4,
	CLIENT_APP_FOURIER_SPECTRUM_INTERP_FACTOR_8
} ClientApplicationFourierSpectrumInterpFactor_t;

typedef enum ClientApplicationTimeDomainWindowType_e
{
	CLIENT_APP_WINDOW_BLACK_MAN,
	CLIENT_APP_WINDOW_KAISER_ALPHA_5,
	CLIENT_APP_WINDOW_KAISER_ALPHA_10,
	CLIENT_APP_WINDOW_KAISER_ALPHA_15,
	CLIENT_APP_WINDOW_DEFAULT = WINDOW_BLACK_MAN,
	CLIENT_APP_WINDOW_INVALID
} ClientApplicationTimeDomainWindowType_t;

typedef struct {
	ClientApplicationRunMode_t runMode;

	int32_t numRxAntennas;

	float radarRfCarrierFreqHz;

	float radarIfBandwidthHz;

	/** Select datacapture mode based on doppler frequency 
	* range and application. Select 
	* APP_DATA_CAPTURE_MODE_MULTI_FRAME_SINGLE_CHIRP if desired 
	* maximum doppler frequency lies between [-2.5Hz, +2.5Hz] and 
	* [-10Hz to 10Hz]. Select APP_DATA_CAPTURE_MODE_SINGLE_FRAME_MULTI_CHIRP 
	* if desired maximum doppler frequency lies in between [-2.5Hz, +2.5Hz]
	* of interest lie between from [-53.5Hz,+53.5Hz] and [-1250Hz , +1250Hz].
	*/
	ClientApplicationDataCaptureMode_t dataCaptureMode;

	/** Maximum frequency depends upon dataCaptureMode
	* selected. Please see field dataCaptureMode.
	*/
	float maxDopplerSamplingFrequencyHz;

	/** This field allows application to determine data 
	* capture duration to allow configured frequency resolution.
	* In case of a resultant capture duration being longest than
	* maximum, maximum capture durationis set and conveyed via 
	* output log. In case of data capture mode 
	* APP_DATA_CAPTURE_MODE_SINGLE_FRAME_MULTI_CHIRP, maximum 
	* resolution is limited to 1.0753Hz. In case of 
	* APP_DATA_CAPTURE_MODE_MULTI_FRAME_SINGLE_CHIRP, maximum
	* frequency resolution would be equal to 0.05Hz.
	*/
	float dopplerFrequencyResolutionHz;

	/** Set this parameter to set maximum distance covered by
	* application. Maximum value of this parameter is 48 (meters) 
	* applicable for both capture modes. Please note that choosing
	* smaller value allows application to capture longer duration 
	* of doppler data for capture mode = 
	* APP_DATA_CAPTURE_MODE_SINGLE_FRAME_MULTI_CHIRP.
	*/
	float maximumRangeMeters;

	/** Choose fourier spectrum interpolation factor for doppler
	* frequency measurement. Higher interpolation factor should 
	* allow for better accuracy. It should be noted that fft length
	* used will be derived as fft_length = interpFactor * 
	* 2^(ceil(log_2(data_sample_count))).
	*/
	ClientApplicationFourierSpectrumInterpFactor_t rangeSpectrumInterpFactor;

	/** Choose fourier spectrum interpolation factor for range (beat)
	* frequency measurement. Higher interpolation factor should
	* allow for better accuracy. It should be noted that fft length
	* used will be derived as fft_length = interpFactor *
	* 2^(ceil(log_2(data_sample_count))).
	*/
	ClientApplicationFourierSpectrumInterpFactor_t dopplerSpectrumInterpFactor;

	float mtiFilterAlpha;

	float fftThresholdPercent;

	ClientApplicationTimeDomainWindowType_t timeDomainWindowType;
} ClientApplicationConfiguration_t;


/** This function accepts as argument pointer towards
* configuration structure defined outside scope of 
* called appConfig(). This will register settings
* in clientApplication and subsequently appInit()
* can be called followed by subsequent calls to appRun()
* and appClose().
*/

void appConfig(const ClientApplicationConfiguration_t* pAppConfiguration);
void appRun(int32_t numFramesToRun);
void appClose();

#endif //__CLIENT_APPLICATION_H__
