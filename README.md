# PC Software Application to Demonstrate Heart Rate and Breathing Rate Measurements using Infineon's Position2Go FMCW Radar.

## Tested on: 
+ Microsoft Windows 10
+ Visual Studio 2019 Version 16.11.3, Microsoft .NET Framework Version 4.8.04084
+ Infineon Position2Go Radar Board: 24 GHz FMCW, Range doppler demonstration board
+ Gnuplot Release 5.2.8


## Description:

Software application runs for configured time period and displays results via GnuPlot. 
It also supports collection of Tx-Rx calibration data when no active target is present in radar's field of view. 
Starting point for software application is source file ClientMain.c
Reference structure for application configuration is given in ClientMain.c
Explanation for structure fields are well documented in ClientApplication.h
Main algorithm is present in file ClientAlgorithmV2.c
Number of doppler frequency readings can be adjusted via setting appropriate values in ClientMain.c

See below configurtion structure used in the application. 

```
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
```

## Test Setup:

Make sure that XMC4700's firmware specified in this respository is uploaded to Position2Go board and board is attached
to PC via USB (corresponding drivers must be installed on PC) otherwise software application will throw an error.

## Further Information

[Position2Go Radar](https://www.infineon.com/cms/en/product/evaluation-boards/demo-position2go/)

[Gnuplot](http://www.gnuplot.info/)

## Contact

Muhammad Abdullah
muhammad1.abdullah2@gmail.com

