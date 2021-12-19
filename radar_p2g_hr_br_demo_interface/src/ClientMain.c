/*
 ===================================================================================
 Name        : extract_raw_data.c
 Author      : Infineon Technologies
 Modified by : Muhammad Abdullah, EE, KAUST
 Version     : 0.1
 Copyright   : 2014-2017, Infineon Technologies AG
 Description : Entry point to client host based heart rate and breath rate measurement 
               library using Position2Go FMCW radar
 ===================================================================================
 */

#include <stdbool.h>
#include <errno.h>
#include <ClientApplication.h>

//#pragma comment (lib, "Shlwapi.lib")

 // 0.5000=1/2    0.2500=1/4    0.1667=1/6    0.1250=1/8    0.1000=1/10    0.0833=1/12
#define BACKGROUND_DATA_SAMPLING_FREQ_HZ (10.0)
#define BACKGROUND_DATA_RESOLUTION_HZ    (0.05)
#define DISTANCE_RANGE_METERES           (40)
#define MTI_FILTER_ALPHA                 (0.0)
 

const ClientApplicationConfiguration_t s_appSaveTxRxLeakage = {
.runMode                       = CLIENT_APP_RUN_MODE_FMCW_RADAR_SAVE_TX_RX_DATA,
.dataCaptureMode               = CLIENT_APP_DATA_CAPTURE_MODE_MULTI_FRAME_SINGLE_CHIRP,
.maxDopplerSamplingFrequencyHz = BACKGROUND_DATA_SAMPLING_FREQ_HZ,
.dopplerFrequencyResolutionHz  = BACKGROUND_DATA_RESOLUTION_HZ,
.maximumRangeMeters            = DISTANCE_RANGE_METERES,
.rangeSpectrumInterpFactor     = FOURIER_SPECTRUM_INTERP_FACTOR_4,
.dopplerSpectrumInterpFactor   = FOURIER_SPECTRUM_INTERP_FACTOR_4,
.mtiFilterAlpha                = MTI_FILTER_ALPHA,
.fftThresholdPercent           = 50.0,
.radarRfCarrierFreqHz          = 24.125e9,
.radarIfBandwidthHz            = 200.0e6,
.numRxAntennas                 = 2
};

const ClientApplicationConfiguration_t s_appSaveBackground = {
.runMode                       = CLIENT_APP_RUN_MODE_FMCW_RADAR_SAVE_BACKGROUND_DATA,
.dataCaptureMode               = CLIENT_APP_DATA_CAPTURE_MODE_MULTI_FRAME_SINGLE_CHIRP,
.maxDopplerSamplingFrequencyHz = BACKGROUND_DATA_SAMPLING_FREQ_HZ,
.dopplerFrequencyResolutionHz  = BACKGROUND_DATA_RESOLUTION_HZ,
.maximumRangeMeters            = DISTANCE_RANGE_METERES,
.rangeSpectrumInterpFactor     = FOURIER_SPECTRUM_INTERP_FACTOR_4,
.dopplerSpectrumInterpFactor   = FOURIER_SPECTRUM_INTERP_FACTOR_4,
.mtiFilterAlpha                = MTI_FILTER_ALPHA,
.fftThresholdPercent           = 50.0,
.radarRfCarrierFreqHz          = 24.125e9,
.radarIfBandwidthHz            = 200.0e6,
.numRxAntennas                 = 2
};

const ClientApplicationConfiguration_t s_appSaveRadarDataConfig= {
.runMode                       = CLIENT_APP_RUN_MODE_FMCW_RADAR_SAVE_DATA,
.dataCaptureMode               = CLIENT_APP_DATA_CAPTURE_MODE_MULTI_FRAME_SINGLE_CHIRP,
.maxDopplerSamplingFrequencyHz = BACKGROUND_DATA_SAMPLING_FREQ_HZ,
.dopplerFrequencyResolutionHz  = BACKGROUND_DATA_RESOLUTION_HZ,
.maximumRangeMeters            = DISTANCE_RANGE_METERES,
.rangeSpectrumInterpFactor     = FOURIER_SPECTRUM_INTERP_FACTOR_2,
.dopplerSpectrumInterpFactor   = FOURIER_SPECTRUM_INTERP_FACTOR_2,
.mtiFilterAlpha                = MTI_FILTER_ALPHA,
.fftThresholdPercent           = 50.0,
.radarRfCarrierFreqHz          = 24.125e9,
.radarIfBandwidthHz            = 200.0e6,
.numRxAntennas                 = 2
};

const ClientApplicationConfiguration_t s_appBreathHeartRateConfig= {
.runMode                       = CLIENT_APP_RUN_MODE_FMCW_RADAR_PROCESS,
.dataCaptureMode               = CLIENT_APP_DATA_CAPTURE_MODE_MULTI_FRAME_SINGLE_CHIRP,
.maxDopplerSamplingFrequencyHz = BACKGROUND_DATA_SAMPLING_FREQ_HZ,
.dopplerFrequencyResolutionHz  = BACKGROUND_DATA_RESOLUTION_HZ,
.maximumRangeMeters            = DISTANCE_RANGE_METERES,
.rangeSpectrumInterpFactor     = FOURIER_SPECTRUM_INTERP_FACTOR_2,
.dopplerSpectrumInterpFactor   = FOURIER_SPECTRUM_INTERP_FACTOR_2,
.mtiFilterAlpha                = MTI_FILTER_ALPHA,
.fftThresholdPercent           = 50.0,
.radarRfCarrierFreqHz          = 24.125e9,
.radarIfBandwidthHz            = 200.0e6,
.numRxAntennas                 = 2
};

int main(int argc, char* argv[])
{
    int ret = 0;

    errno = 0;
    ClientApplicationRunMode_t runMode = 0;
    if (argc > 1)
    {
        runMode = (ClientApplicationRunMode_t)(strtol(argv[1], NULL, 10));
    }
    else
    {
        // If no input was given, just run the post processing
        runMode = CLIENT_APP_RUN_MODE_FMCW_RADAR_PROCESS;
    }
    // check if invalid value was input
    bool range_error = errno == ERANGE;
    if (range_error)
    {
        return -1;
    }

    int32_t numIterations;

    if (runMode == CLIENT_APP_RUN_MODE_FMCW_RADAR_SAVE_TX_RX_DATA)
    {
        numIterations = 1;
        appConfig(&s_appSaveTxRxLeakage);
        appRun(numIterations);
        appClose();
    }
    else if (runMode == CLIENT_APP_RUN_MODE_FMCW_RADAR_SAVE_BACKGROUND_DATA)
    {
        numIterations = 1;
        appConfig(&s_appSaveBackground);
        appRun(numIterations);
        appClose();
    }
    else if (runMode == CLIENT_APP_RUN_MODE_FMCW_RADAR_SAVE_DATA)
    {
        numIterations = 30;
        appConfig(&s_appSaveRadarDataConfig);
        appRun(numIterations);
        appClose();
    }
    else if (runMode == CLIENT_APP_RUN_MODE_FMCW_RADAR_PROCESS)
    {
        numIterations = 100;
        appConfig(&s_appBreathHeartRateConfig);
        appRun(numIterations);
        appClose();
    }
    else
    {
        // No action needed because input mode was not valid
        return 0;
    }

    return ret;
}