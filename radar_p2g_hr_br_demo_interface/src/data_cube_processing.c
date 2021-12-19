/****************************************************************
 INCLUDES
*****************************************************************/
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "assert.h"
#include <Windows.h>

#include "fftw3.h"
#include "plot_frame_data.h"
#include "data_cube_processing.h"

/****************************************************************
 DEFINES
*****************************************************************/
#define MAX_NUM_FILTERED_RANGE_BINS             (20)
#define MAX_FRAME_BUFFER_LEN                    (20) // number of historical frames to keep data for
#define MAX_NUM_PEAKS_PER_FRAME                 (20) // number of raw peaks observed to be fulfilling threshold criterion
#define NUM_PEAKS_PER_FRAME                     (10)  // number of peaks with highest magnitude
#define MAX_FRAME_INFO_HIST_LEN                 (5) // frames to consider for peak filtering
#define WINDOW_SELECT                           (0)
#define RANGE_DOPPLER_SPECTRUM_NUM_ACCUM_FRAMES (2)



/****************************************************************
 TYPES
*****************************************************************/

/****************************************************************
 DATA
*****************************************************************/
typedef struct peakInfo_s
{
  float peakMag;
  float range;
  float dopplerFreq;
} peakInfo_t;

typedef struct filteredPeakInfo_t
{
  float peakMag;
  float range;
  float dopplerFreq;
  float count;
  bool isValid;
} filteredPeakInfo_t;

static bool backGroundDataAvailable = false;
static bool discardFrameProcessing = false;
static float sBackgroundData[64 * 256 * 4] = { 0 };

static int frameCount    = 0;
static int numAccumFrames = 0;
static int numHistFrames  = 0;
static int numPeaksHist[MAX_FRAME_BUFFER_LEN];
static peakInfo_t peakInfoHistBuffer[MAX_FRAME_BUFFER_LEN][MAX_NUM_PEAKS_PER_FRAME] = {0};
static filteredPeakInfo_t filteredPeakInfo[MAX_NUM_PEAKS_PER_FRAME] = {0};

float  adcCalibrationData[256 * 4];
float* p_adcCalibrationData[4] = { &adcCalibrationData[256 * 0], &adcCalibrationData[256 * 1], &adcCalibrationData[256 * 2], &adcCalibrationData[256 * 3] };
static float chebyshevTime_s_window[] = { 0.08000, 0.08229, 0.08912, 0.10044, 0.11612, 0.13602, 0.15993, 0.18762, 0.21881, 0.25319, 0.29043, 0.33014, 0.37194, 0.41541, 0.46012, 0.50562, 0.55147, 0.59720, 0.64236, 0.68650, 0.72919, 0.77000, 0.80852, 0.84438, 0.87720, 0.90668, 0.93251, 0.95445, 0.97226, 0.98578, 0.99486, 0.99943, 0.99943, 0.99486, 0.98578, 0.97226, 0.95445, 0.93251, 0.90668, 0.87720, 0.84438, 0.80852, 0.77000, 0.72919, 0.68650, 0.64236, 0.59720, 0.55147, 0.50562, 0.46012, 0.41541, 0.37194, 0.33014, 0.29043, 0.25319, 0.21881, 0.18762, 0.15993, 0.13602, 0.11612, 0.10044, 0.08912, 0.08229, 0.08000 };
static float chebyshevFreq_s_window[] = { 0.06245, 0.05241, 0.07305, 0.09787, 0.12706, 0.16073, 0.19885, 0.24130, 0.28784, 0.33806, 0.39147, 0.44743, 0.50519, 0.56390, 0.62263, 0.68042, 0.73623, 0.78904, 0.83785, 0.88171, 0.91974, 0.95116, 0.97532, 0.99171, 1.00000, 1.00000, 0.99171, 0.97532, 0.95116, 0.91974, 0.88171, 0.83785, 0.78904, 0.73623, 0.68042, 0.62263, 0.56390, 0.50519, 0.44743, 0.39147, 0.33806, 0.28784, 0.24130, 0.19885, 0.16073, 0.12706, 0.09787, 0.07305, 0.05241, 0.06245 };
static float HammingTime_s_window[] = { 0.08000, 0.08229, 0.08912, 0.10044, 0.11612, 0.13602, 0.15993, 0.18762, 0.21881, 0.25319, 0.29043, 0.33014, 0.37194, 0.41541, 0.46012, 0.50562, 0.55147, 0.59720, 0.64236, 0.68650, 0.72919, 0.77000, 0.80852, 0.84438, 0.87720, 0.90668, 0.93251, 0.95445, 0.97226, 0.98578, 0.99486, 0.99943, 0.99943, 0.99486, 0.98578, 0.97226, 0.95445, 0.93251, 0.90668, 0.87720, 0.84438, 0.80852, 0.77000, 0.72919, 0.68650, 0.64236, 0.59720, 0.55147, 0.50562, 0.46012, 0.41541, 0.37194, 0.33014, 0.29043, 0.25319, 0.21881, 0.18762, 0.15993, 0.13602, 0.11612, 0.10044, 0.08912, 0.08229, 0.08000 };
static float HammingFreq_s_window[] = { 0.21264, 0.22694, 0.25511, 0.29636, 0.34954, 0.41326, 0.48595, 0.56594, 0.65158, 0.74124, 0.83343, 0.92676, 1.01998, 1.11195, 1.20159, 1.28789, 1.36984, 1.44640, 1.51656, 1.57925, 1.63344, 1.67817, 1.71256, 1.73591, 1.74771, 1.74771, 1.73591, 1.71256, 1.67817, 1.63344, 1.57925, 1.51656, 1.44640, 1.36984, 1.28789, 1.20159, 1.11195, 1.01998, 0.92676, 0.83343, 0.74124, 0.65158, 0.56594, 0.48595, 0.41326, 0.34954, 0.29636, 0.25511, 0.22694, 0.21264 };
static float TaylorTime_s_window[] = { 0.11122, 0.10703, 0.11717, 0.14828, 0.18895, 0.23172, 0.27962, 0.33422, 0.39344, 0.45737, 0.52683, 0.60073, 0.67839, 0.76000, 0.84469, 0.93141, 1.01985, 1.10917, 1.19814, 1.28610, 1.37223, 1.45526, 1.53437, 1.60880, 1.67744, 1.73945, 1.79427, 1.84105, 1.87912, 1.90820, 1.92783, 1.93762, 1.93762, 1.92783, 1.90820, 1.87912, 1.84105, 1.79427, 1.73945, 1.67744, 1.60880, 1.53437, 1.45526, 1.37223, 1.28610, 1.19814, 1.10917, 1.01985, 0.93141, 0.84469, 0.76000, 0.67839, 0.60073, 0.52683, 0.45737, 0.39344, 0.33422, 0.27962, 0.23172, 0.18895, 0.14828, 0.11717, 0.10703, 0.11122 };
static float TaylorFreq_s_window[] = { 0.11053, 0.10861, 0.13729, 0.18812, 0.24351, 0.30841, 0.38248, 0.46407, 0.55446, 0.65154, 0.75501, 0.86361, 0.97544, 1.08951, 1.20345, 1.31565, 1.42414, 1.52665, 1.62162, 1.70682, 1.78075, 1.84190, 1.88880, 1.92078, 1.93684, 1.93684, 1.92078, 1.88880, 1.84190, 1.78075, 1.70682, 1.62162, 1.52665, 1.42414, 1.31565, 1.20345, 1.08951, 0.97544, 0.86361, 0.75501, 0.65154, 0.55446, 0.46407, 0.38248, 0.30841, 0.24351, 0.18812, 0.13729, 0.10861, 0.11053 };
static float TukeyTime_s_window[] = { 0.00000, 0.00248, 0.00991, 0.02221, 0.03926, 0.06089, 0.08688, 0.11698, 0.15088, 0.18826, 0.22873, 0.27189, 0.31733, 0.36458, 0.41318, 0.46263, 0.51247, 0.56217, 0.61126, 0.65924, 0.70564, 0.75000, 0.79187, 0.83084, 0.86653, 0.89857, 0.92665, 0.95048, 0.96985, 0.98454, 0.99442, 0.99938, 0.99938, 0.99442, 0.98454, 0.96985, 0.95048, 0.92665, 0.89857, 0.86653, 0.83084, 0.79187, 0.75000, 0.70564, 0.65924, 0.61126, 0.56217, 0.51247, 0.46263, 0.41318, 0.36458, 0.31733, 0.27189, 0.22873, 0.18826, 0.15088, 0.11698, 0.08688, 0.06089, 0.03926, 0.02221, 0.00991, 0.00248, 0.00000 };
static float TukeyFreq_s_window[] = { 0.00000, 0.00410, 0.01635, 0.03654, 0.06434, 0.09929, 0.14083, 0.18826, 0.24080, 0.29761, 0.35774, 0.42020, 0.48397, 0.54801, 0.61126, 0.67268, 0.73127, 0.78606, 0.83615, 0.88072, 0.91904, 0.95048, 0.97453, 0.99078, 0.99897, 0.99897, 0.99078, 0.97453, 0.95048, 0.91904, 0.88072, 0.83615, 0.78606, 0.73127, 0.67268, 0.61126, 0.54801, 0.48397, 0.42020, 0.35774, 0.29761, 0.24080, 0.18826, 0.14083, 0.09929, 0.06434, 0.03654, 0.01635, 0.00410, 0.00000 };



static float* sTimeWindow[] = { chebyshevTime_s_window, HammingTime_s_window, TaylorTime_s_window, TukeyTime_s_window }; // select here
static float* sFreqWindow[] = { chebyshevFreq_s_window, HammingFreq_s_window, TaylorFreq_s_window, TukeyFreq_s_window};  // select here
static float rangeDopplerSpectrum[RANGE_DOPPLER_SPECTRUM_NUM_ACCUM_FRAMES][(RANGE_FFT_SIZE >> 1) * DOPPLER_FFT_SIZE] = {{0}};
static float rangeDopplerFftAccum[RANGE_DOPPLER_SPECTRUM_NUM_ACCUM_FRAMES][(RANGE_FFT_SIZE >> 1) * DOPPLER_FFT_SIZE * 2] = { {0} };

/****************************************************************
 LOCAL FUNCTIONS
*****************************************************************/
static void normalize_data(float* data, int len)
{
  float max = 0.0;
  for (int i = 0; i < len; i++)
  {
    if (data[i] > max)
    {
      max = data[i];
    }
  }

  for (int i = 0; i < len; i++)
  {
    data[i] /= max;
  }
}

static void adjustMean(float* in, int len)
{
  float mean = 0;
  for (int i = 0; i < len; i++)
  {
    mean += in[i]; 
  }
  mean /= len;
  for (int i = 0; i < len; i++)
  {
    in[i] -= mean;
  }
}

static void applyTimeWindow(float *in, int len, bool timeDomain)
{
  for (int i = 0; i < len; i++)
  {
      if (timeDomain)
      {
          in[i] = in[i] * sTimeWindow[WINDOW_SELECT][i];
      }
      else
      {
          in[i] = in[i] * sFreqWindow[WINDOW_SELECT][i];
      }
  }
}


static void applyFourierTransform(float *i, float *q, int inLen, float* out, bool inverseFt, bool rangeFft)
{
  int j;
  fftw_complex* sFftInMem  = (fftw_complex*)malloc(sizeof(fftw_complex) * (rangeFft ? RANGE_FFT_SIZE : DOPPLER_FFT_SIZE));
  fftw_complex* sFftOutMem = (fftw_complex*)malloc(sizeof(fftw_complex) * (rangeFft ? RANGE_FFT_SIZE : DOPPLER_FFT_SIZE));

  int fftSize = rangeFft ? RANGE_FFT_SIZE : DOPPLER_FFT_SIZE;

  // Copy data to fftw friendly complex data type
  for (j = 0; j < inLen; j++)
  {
    sFftInMem[j][0] = i[j];
    sFftInMem[j][1] = q[j];
    sFftOutMem[j][0] = 0.0;
    sFftOutMem[j][1] = 0.0;
  }
  for (j = inLen; j < fftSize; j++)
  {
    sFftInMem[j][0] = 0.0;
    sFftInMem[j][1] = 0.0;
    sFftOutMem[j][0] = 0.0;
    sFftOutMem[j][1] = 0.0;
  }

  fftw_plan fftPlan = fftw_plan_dft_1d(fftSize, sFftInMem, sFftOutMem, inverseFt ? FFTW_BACKWARD : FFTW_FORWARD, FFTW_ESTIMATE);
  fftw_execute(fftPlan);

  for (j = 0; j < fftSize; j++)
  {
    out[(j << 1) + 0] = sFftOutMem[j][0];
    out[(j << 1) + 1] = sFftOutMem[j][1];
  }

  free(sFftInMem);
  free(sFftOutMem);
  fftw_destroy_plan(fftPlan);
}

static inline void calculateFftSpectrum(float* in, int len, float* out, bool calcSqrt)
{
  int j;
  for (j = 0; j < len; j++)
  {
    out[j] = in[(j << 1) + 0] * in[(j << 1) + 0] + in[(j << 1) + 1] * in[(j << 1) + 1];
    if (calcSqrt)
    {
      out[j] = sqrt(out[j]);
    }
  }
}

static void circularShift(float* in, int len)
{
  bool lenOdd = len & 1;
  int swapLength = len >> 1;
  int j;
  for (j = 0; j < swapLength; j++)
  {
    float swapper = in[j];
    in[j] = in[j + swapLength + lenOdd];
    in[j + swapLength + lenOdd] = swapper;
  }
}

/****************************************************************
 EXPORTED FUNCTIONS
*****************************************************************/
void dataCubeProcessing(
    const Frame_Info_t* frame_info,
    int  antennaNum,
    int  doFiltering,
    int  timeWindowEnabled,
    bool sApplyCalibration
)
{
    // Extract frame info 
    int numChirps = frame_info->num_chirps;
    int numSamplesPerChirp = frame_info->num_samples_per_chirp;
    int frameNum = frame_info->frame_number;
    float* inData = frame_info->sample_data;
    int numAntennas = 1; //int numAntennas = frame_info->num_rx_antennas;

    discardFrameProcessing = false;
    // if no previous data available and there is request, store data
    if ((sApplyCalibration == true) && (backGroundDataAvailable == false))
    {
        discardFrameProcessing = true;  // ignore current frame as data is stord
        backGroundDataAvailable = true; // start saying that data is available
        for (int i = 0; i < numSamplesPerChirp * numChirps * 4; i++)
        {
            sBackgroundData[i] = inData[i];
        }
    }
    
    // if there is no request, don't apply background removal from processing
    if (sApplyCalibration == false)
    {
        backGroundDataAvailable = false;
    }


    // Check if previously available data is present (implies to use data for background removal)
    // and there is no request to apply use current frame for calibration
    if ((discardFrameProcessing == false) && (backGroundDataAvailable == true))
    {
        for (int i = 0; i < numSamplesPerChirp * numChirps * 4; i++)
        {
            inData[i] -= sBackgroundData[i];
        }
    }

    if (discardFrameProcessing == false)
    {
        // Calculate required size for range and doppler ffts
        int rangeFftBufferLen = RANGE_FFT_SIZE * numChirps * numAntennas;
        int dopplerFftBufferLen = DOPPLER_FFT_SIZE * (RANGE_FFT_SIZE >> 1)* numAntennas;
        int interpFactor = BLACKMAN_TIME_WINDOW_LEN / numSamplesPerChirp;

        // Allocate dynamic memories here
        float* rangeFft = (float*)malloc(sizeof(float) * rangeFftBufferLen * 2); // multiply by 2 to accomodate complex values
        float* rangeFftSpectrum = (float*)malloc(sizeof(float) * rangeFftBufferLen);
        float* rangeFftSpectrumAccum = (float*)malloc(sizeof(float) * RANGE_FFT_SIZE);

        float* dopplerFftAccum = (float*)malloc(sizeof(float) * dopplerFftBufferLen * 2);
        float* dopplerFftSpectrum = (float*)malloc(sizeof(float) * dopplerFftBufferLen);
        float* dopplerFft = (float*)malloc(sizeof(float) * DOPPLER_FFT_SIZE * 2); // multiply by 2 to accomodate complex values

        // Set buffer to initial entries of zero (0)
        memset((void*)rangeFftSpectrum, 0, sizeof(float) * rangeFftBufferLen);
        memset((void*)dopplerFftSpectrum, 0, sizeof(float) * dopplerFftBufferLen);
        memset((void*)rangeFftSpectrumAccum, 0, sizeof(float) * RANGE_FFT_SIZE);

        // Iterate over number of antennas
        for (int rxIdx = 0; rxIdx < numAntennas; rxIdx++)
        {
            /**** Range FFTs ****/
            for (int chirpIdx = 0; chirpIdx < numChirps; chirpIdx++)
            {
                // Get pointer to chirp data address within buffer according to current chirp and rx antenna
                int chirpOffset = ((chirpIdx << 2) + (rxIdx << 1)) * numSamplesPerChirp;
                float* in_i = &inData[chirpOffset + 0];
                float* in_q = &inData[chirpOffset + numSamplesPerChirp];

                // Get pointer to range FFT according to current chirp and rx antenna
                float* rangeFftPerChirp = &rangeFft[RANGE_FFT_SIZE * (chirpIdx + rxIdx * numChirps) * 2];
                float* rangeFftSpectrumPerChirp = &rangeFftSpectrum[RANGE_FFT_SIZE * (chirpIdx + rxIdx * numChirps)];

                // Correct for radar calibration data
                int idx = 0;
                for (int sampleIdx = 0; sampleIdx < numSamplesPerChirp; sampleIdx++)
                {
                    in_i[sampleIdx] -= p_adcCalibrationData[(rxIdx << 1) + 0][idx];
                    in_q[sampleIdx] -= p_adcCalibrationData[(rxIdx << 1) + 1][idx];
                    idx += interpFactor;
                }

                /* Signal processing part */

                // Adjust mean of raw samples of current chirp
                adjustMean(in_i, numSamplesPerChirp);
                adjustMean(in_q, numSamplesPerChirp);

                if (timeWindowEnabled)
                {
                    // Apply window on each chirp
                    applyTimeWindow(in_i, numSamplesPerChirp, true);
                    applyTimeWindow(in_q, numSamplesPerChirp, true);
                }

                // FFT
                applyFourierTransform(in_i, in_q, numSamplesPerChirp, rangeFftPerChirp, false, true);

                // calculate spectrum
                calculateFftSpectrum(rangeFftPerChirp, RANGE_FFT_SIZE, rangeFftSpectrumPerChirp, false);

#if 0
                // Accumate range FFTs across the pulses.
                for (int i = 0; i < RANGE_FFT_SIZE; i++)
                {
                    rangeFftSpectrumAccum[i] += rangeFftSpectrumPerChirp[i];
                }
#endif
            }

#if 0


            // Find range bins where targets were detected
            float maxMag = 0;
            for (int i = 0; i < (RANGE_FFT_SIZE >> 1); i++)
            {
                if (rangeFftSpectrumAccum[i] > maxMag)
                {
                    maxMag = rangeFftSpectrumAccum[i];
                }
            }

            int filteredRangeBins[MAX_NUM_FILTERED_RANGE_BINS];
            float rangeThreshold = maxMag * 0.4;
            int filteredRangeBinCount = 0;
            for (int i = 1; i < (RANGE_FFT_SIZE >> 1) - 1; i++)
            {
                if ((rangeFftSpectrumAccum[i] > rangeFftSpectrumAccum[i - 1]) &&
                    (rangeFftSpectrumAccum[i] > rangeFftSpectrumAccum[i + 1]) &&
                    (rangeFftSpectrumAccum[i] > rangeThreshold))
                {
                    filteredRangeBins[filteredRangeBinCount] = i;
                    filteredRangeBinCount++;
                }
            }
#endif

            /**** Doppler FFT ****/
                // Consider only positive frequencies of range fft
            int numRangeBins = (RANGE_FFT_SIZE >> 1);

            float* in_i = (float*)malloc(sizeof(float) * numChirps);
            float* in_q = (float*)malloc(sizeof(float) * numChirps);
#if 0
            for (int rangeBin = 0; rangeBin < filteredRangeBinCount; rangeBin++)
#else
            for (int rangeBin = 0; rangeBin < numRangeBins; rangeBin++)
#endif
            {
#if 0
                // Get slow time samples across all pulses against current range bin
                int fftIdx = (filteredRangeBins[rangeBin] << 1);
#else
                // Get slow time samples across all pulses against current range bin
                int fftIdx = (rangeBin << 1);
#endif
                for (int i = 0; i < numChirps; i++)
                {
                    in_i[i] = rangeFft[fftIdx + 0];
                    in_q[i] = rangeFft[fftIdx + 1];
                    fftIdx += (RANGE_FFT_SIZE * 2); //  * 2 for complex FFT length
            }

#if 0
                float* dopplerFftSpectrumPerChirp = &dopplerFftSpectrum[DOPPLER_FFT_SIZE * (filteredRangeBins[rangeBin] + rxIdx * numRangeBins)];
#else
                float* dopplerFftSpectrumPerChirp = &dopplerFftSpectrum[DOPPLER_FFT_SIZE * (rangeBin + rxIdx * numRangeBins)];
#endif

                //// Adjust mean of raw samples of current chirp
                adjustMean(in_i, numChirps);
                adjustMean(in_q, numChirps);

                // take fourier transform and save this
                applyFourierTransform(in_i, in_q, numChirps, dopplerFft, false, false);

                // calculate spectrum
                calculateFftSpectrum(dopplerFft, DOPPLER_FFT_SIZE, dopplerFftSpectrumPerChirp, false);

                // Circularly rotate doppler spectrum
                circularShift(dopplerFftSpectrumPerChirp, DOPPLER_FFT_SIZE);

                memcpy((void*)&dopplerFftAccum[2 * DOPPLER_FFT_SIZE * (rangeBin + rxIdx * numRangeBins)], (void*)dopplerFft, sizeof(float) * 2 * DOPPLER_FFT_SIZE);

                circularShift(&dopplerFftAccum[2 * DOPPLER_FFT_SIZE * (rangeBin + rxIdx * numRangeBins)], 2 * DOPPLER_FFT_SIZE);


#if 0
                // normalize doppler spectrum
#endif
        }

            free(in_i);
            free(in_q);
    }

        // normalize data before storing or accumulation
        //normalize_data(dopplerFftSpectrum, dopplerFftBufferLen);


        int numRangeBins = (RANGE_FFT_SIZE >> 1);
        int frameIdx = frameCount % RANGE_DOPPLER_SPECTRUM_NUM_ACCUM_FRAMES;

        // TODO: MABD only done for one antenna at the moment
        for (int sample = 0; sample < (numRangeBins * DOPPLER_FFT_SIZE); sample++)
        {
            rangeDopplerSpectrum[frameIdx][sample] = dopplerFftSpectrum[sample];
            rangeDopplerFftAccum[frameIdx][sample * 2 + 0] = dopplerFftAccum[sample * 2 + 0];
            rangeDopplerFftAccum[frameIdx][sample * 2 + 1] = dopplerFftAccum[sample * 2 + 1];
        }

        // Increment frame number
        numAccumFrames++;

        // Check if frames collected are greater or equal len frame history length
        if (numAccumFrames >= RANGE_DOPPLER_SPECTRUM_NUM_ACCUM_FRAMES)
        {
            numAccumFrames = RANGE_DOPPLER_SPECTRUM_NUM_ACCUM_FRAMES;
        }

        // No need to process or display if accumulated data is not available...
        // If accumulated data is available, then find peaks and filter using frequency criterion...
        if (numAccumFrames == RANGE_DOPPLER_SPECTRUM_NUM_ACCUM_FRAMES)
        {
            int startFrameIdx = frameIdx - RANGE_DOPPLER_SPECTRUM_NUM_ACCUM_FRAMES + 1;
            if (startFrameIdx < 0)
            {
                startFrameIdx += RANGE_DOPPLER_SPECTRUM_NUM_ACCUM_FRAMES;
            }

            // Accumulate
            // TODO: this can be replaced by delete oldest + add newest + store sequence for efficiency purposes
            for (int sample = 0; sample < (DOPPLER_FFT_SIZE * numRangeBins); sample++)
            {
                dopplerFftSpectrum[sample] = 0;
                int frameIdx = startFrameIdx;
                for (int frame = 0; frame < RANGE_DOPPLER_SPECTRUM_NUM_ACCUM_FRAMES; frame++)
                {
                    dopplerFftSpectrum[sample] += //sqrt((rangeDopplerFftAccum[frameIdx][2 * sample + 0] * rangeDopplerFftAccum[frameIdx][2 * sample + 0]) + 
                                                  //     (rangeDopplerFftAccum[frameIdx][2 * sample + 1] * rangeDopplerFftAccum[frameIdx][2 * sample + 1]) );
                        rangeDopplerSpectrum[frameIdx][sample];
                    frameIdx++;
                    if (frameIdx == RANGE_DOPPLER_SPECTRUM_NUM_ACCUM_FRAMES)
                    {
                        frameIdx = 0;
                    }
                }
            }

            normalize_data(dopplerFftSpectrum, dopplerFftBufferLen);

            // Peak finding code
            int cIndex = frameCount % MAX_FRAME_BUFFER_LEN;

            float threshold = 0.25;
            float rangeDistanceThreshold = 0.25;
            float dopplerDistanceThreshold = 0.5;
            float occuranceThreshold = 0.25;

            float maxValue;
            int peakIdx = 0;
            int numRangeBins = (RANGE_FFT_SIZE >> 1); // Consider only positive frequencies of range fft
            for (int rxIdx = 0; rxIdx < numAntennas; rxIdx++)
            {
                for (int rangeBin = 1; rangeBin < numRangeBins - 1; rangeBin++)
                {
                    float* dopplerFftSpectrumPerChirp = &dopplerFftSpectrum[DOPPLER_FFT_SIZE * (rangeBin + rxIdx * numRangeBins)];
                    for (int sampleIdx = 1; sampleIdx < DOPPLER_FFT_SIZE - 1; sampleIdx++)
                    {
                        if ((dopplerFftSpectrumPerChirp[sampleIdx] > dopplerFftSpectrumPerChirp[sampleIdx - 1]) &&
                            (dopplerFftSpectrumPerChirp[sampleIdx] > dopplerFftSpectrumPerChirp[sampleIdx + 1]) &&
                            (dopplerFftSpectrumPerChirp[sampleIdx] > threshold))
                        {
                            // This is true peak if it exceeds it's neighbouring range samples as well
                            float precedingRangeSample = dopplerFftSpectrum[DOPPLER_FFT_SIZE * (rangeBin - 1 + rxIdx * numRangeBins) + sampleIdx];
                            float followingRangeSample = dopplerFftSpectrum[DOPPLER_FFT_SIZE * (rangeBin + 1 + rxIdx * numRangeBins) + sampleIdx];

                            if ((dopplerFftSpectrumPerChirp[sampleIdx] > precedingRangeSample) &&
                                (dopplerFftSpectrumPerChirp[sampleIdx] > followingRangeSample))
                            {
                                double range = (3.0 / 16.0) * rangeBin;
                                double doppler_freqs = (sampleIdx - (DOPPLER_FFT_SIZE / 2)) * (1.0e6 / ((double)(DOPPLER_FFT_SIZE) * 6300));
                                peakInfoHistBuffer[cIndex][peakIdx].range = range;
                                peakInfoHistBuffer[cIndex][peakIdx].dopplerFreq = doppler_freqs;
                                peakInfoHistBuffer[cIndex][peakIdx].peakMag = dopplerFftSpectrumPerChirp[sampleIdx];
                                peakIdx++;
                                if (peakIdx == MAX_NUM_PEAKS_PER_FRAME)
                                {
                                    peakIdx = MAX_NUM_PEAKS_PER_FRAME - 1;
                                }
                            }
                        }
                    }
                }
            }
            numPeaksHist[cIndex] = peakIdx;

            // Sort peaks according to descending order of peaks
            for (int i = 0; i < numPeaksHist[cIndex]; i++)
            {
                for (int j = i + 1; j < numPeaksHist[cIndex]; j++)
                {

                    if (peakInfoHistBuffer[cIndex][i].peakMag < peakInfoHistBuffer[cIndex][j].peakMag)
                    {
                        peakInfo_t tmp = peakInfoHistBuffer[cIndex][i];
                        peakInfoHistBuffer[cIndex][i] = peakInfoHistBuffer[cIndex][j];
                        peakInfoHistBuffer[cIndex][j] = tmp;
                    }
                }
            }

            // Just consider highly weighted magnitude peaks for further filtering across the frames
            numPeaksHist[cIndex] = numPeaksHist[cIndex] >= NUM_PEAKS_PER_FRAME ? NUM_PEAKS_PER_FRAME : numPeaksHist[cIndex];

            // Increment frame number
            numHistFrames++;

            // Check if frames collected are greater or equal len frame history length
            if (numHistFrames >= MAX_FRAME_INFO_HIST_LEN)
            {
                numHistFrames = MAX_FRAME_INFO_HIST_LEN;
            }

            // If enough (MAX_FRAME_INFO_HIST_LEN) number of frames are processed, filter peaks based
            // on particular criterion. Here we only pick out peaks which occur mostly in MAX_FRAME_INFO_HIST_LEN
            // past (including current) frames.
            if (numHistFrames == MAX_FRAME_INFO_HIST_LEN)
            {
                int mindex = 0;
                int maxNumPeaks = 0;
                int mIndex = cIndex;
                int ccIndex = cIndex;
                // check which frame has maximum number of peaks
                // use this number to filter out frames in order
                // to not miss a particular frame
                for (int i = 0; i < MAX_FRAME_INFO_HIST_LEN; i++)
                {
                    if (numPeaksHist[ccIndex] > maxNumPeaks)
                    {
                        maxNumPeaks = numPeaksHist[ccIndex];
                        mIndex = ccIndex;
                    }
                    ccIndex--;
                    if (ccIndex == -1)
                    {
                        ccIndex = MAX_FRAME_BUFFER_LEN - 1;
                    }
                }

                int fpeakIdx = 0;
                for (int i = 0; i < MAX_NUM_PEAKS_PER_FRAME; i++)
                {
                    filteredPeakInfo[i].isValid = false;
                    filteredPeakInfo[i].count = 0;
                    filteredPeakInfo[i].range = 0;
                    filteredPeakInfo[i].peakMag = 0;
                    filteredPeakInfo[i].dopplerFreq = 0;
                }

                // Search for peaks which are close to each other
                // according to some distance threshold
                for (int i = 0; i < maxNumPeaks; i++)
                {
                    peakInfo_t peakInfo1 = peakInfoHistBuffer[mIndex][i];

                    // Check if similar peak appears in historical frame results
                    ccIndex = cIndex;
                    for (int n = 0; n < MAX_FRAME_INFO_HIST_LEN; n++)
                    {
                        if (ccIndex != mIndex)
                        {
                            for (int j = 0; j < numPeaksHist[ccIndex]; j++)
                            {
                                peakInfo_t peakInfo2 = peakInfoHistBuffer[ccIndex][j];

                                if ((abs(peakInfo1.range - peakInfo2.range) < rangeDistanceThreshold) &&
                                    ((abs(peakInfo1.dopplerFreq) - abs(peakInfo2.dopplerFreq)) < dopplerDistanceThreshold))
                                {
                                    filteredPeakInfo[i].count++;
                                    if (filteredPeakInfo[i].count == 1)
                                    {
                                        filteredPeakInfo[i].isValid = true;
                                        filteredPeakInfo[i].dopplerFreq = abs(peakInfo1.dopplerFreq);
                                        filteredPeakInfo[i].range = peakInfo1.range;
                                        filteredPeakInfo[i].peakMag = peakInfo1.peakMag;
                                    }
                                }
                            }
                        }
                        ccIndex--;
                        if (ccIndex == -1)
                        {
                            ccIndex = MAX_FRAME_BUFFER_LEN - 1;
                        }
                    }
                }


                // Check and clear duplicate peaks
                bool oomPoint = false;
                for (int i = 0; i < maxNumPeaks; i++)
                {
                    filteredPeakInfo_t peakInfo1 = filteredPeakInfo[i];

                    if ((peakInfo1.count >= (occuranceThreshold * (float)MAX_FRAME_INFO_HIST_LEN)) && (peakInfo1.isValid == true))
                    {
                        oomPoint = true;
                        // remove duplicates
                        for (int j = i + 1; j < maxNumPeaks; j++)
                        {
                            filteredPeakInfo_t peakInfo2 = filteredPeakInfo[j];

                            if ((abs(peakInfo1.range - peakInfo2.range) < rangeDistanceThreshold) &&
                                (abs(peakInfo1.dopplerFreq - peakInfo2.dopplerFreq) < dopplerDistanceThreshold))
                            {
                                filteredPeakInfo[j].isValid = false;
                            }
                        }
                    }
                    else
                    {
                        filteredPeakInfo[i].isValid = false;
                    }
                }

                if (oomPoint)
                {
                    // Do plotting of peaks
                    float rangeStart = 0;
                    float rangeEnd = (3.0 / 4.0) * ((double)numSamplesPerChirp / (double)RANGE_FFT_SIZE) * (double)(numRangeBins - 1);
                    float dopplerStart = (0 - (DOPPLER_FFT_SIZE / 2)) * (1.0e6 / ((double)(DOPPLER_FFT_SIZE) * 9300));
                    float dopplerEnd = (DOPPLER_FFT_SIZE - 1 - (DOPPLER_FFT_SIZE / 2)) * (1.0e6 / ((double)(DOPPLER_FFT_SIZE) * 9300));

                    FILE* handle = get_window_handle((int)PLOT_DETECTED_OBJECTS);

                    fprintf(handle, "reset\n");
                    //fprintf(handle, "plot [%f:%f] [%f:%f] '-' using ($1 < 0 ? ($1-8):($1+8)):($2):(sprintf(\"%f\",stringcolumn(1))) title \"\" with labels , '-' using 1:2:3 with points pt 7 ps 1\n", dopplerStart, dopplerEnd, rangeStart, rangeEnd);
                    fprintf(handle, "set xlabel \"Doppler Frequency (Hz)\"\n");
                    fprintf(handle, "set ylabel \"Range (meters)\"\n");
                    fprintf(handle, "set title \"Detected Targets\"\n");
                    fprintf(handle, "set xtics 10\n");
                    fprintf(handle, "set mxtics 2\n");
                    fprintf(handle, "set ytics 2\n");
                    fprintf(handle, "set grid mxtics xtics ytics\n");
                    fprintf(handle, "plot [%f:%f] [%f:%f] '-' using 1:2:3 title \"\" with points pt 7 ps 1.5\n", dopplerStart, dopplerEnd, rangeStart, rangeEnd);

                    // Copy peaks to history
                    for (int i = 0; i < maxNumPeaks; i++)
                    {
                        if (filteredPeakInfo[i].isValid)
                        {
                            fprintf(handle, "%g %g %g\n", filteredPeakInfo[i].dopplerFreq, filteredPeakInfo[i].range, filteredPeakInfo[i].peakMag);
                        }
                    }
                    fprintf(handle, "%s\n", "e");
                    fflush(handle);
                }
            }

            frameCount++;

            if ((frameCount % 60) == 0)
            {
                SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 12); // red color
                printf("\n\n-----------Changing target position now-----------\n\n");
            }
            else
            {
                SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 15); // white color
            }

            // Do plotting
            printf("Plotting Results:  frame %d\n", frameNum);

            // TEMP

            float rangeStart = 0;
            float rangeEnd = (3.0 / 4.0) * ((double)numSamplesPerChirp / (double)RANGE_FFT_SIZE) * (double)(numRangeBins - 1);

            FILE* handle = get_window_handle((int)PLOT_RANGE_FFT);

            fprintf(handle, "plot [%f:%f] [%f:%f] '-' using ($1):($2):($3) with image\n", (float)0.0, (float)49.0, rangeStart, rangeEnd);
            for (int rxIdx = 0; rxIdx < numAntennas; rxIdx++)
            {
                for (int rangeBin = 0; rangeBin < numRangeBins; rangeBin++)
                {
                    double range = (3.0 / 4.0) * ((double)numSamplesPerChirp / (double)RANGE_FFT_SIZE) * (double)(rangeBin);
                    for (int chirpIdx = 0; chirpIdx < numChirps; chirpIdx++)
                    {
                        fprintf(handle, "%g %g %g\n", (double)chirpIdx, range, (double)rangeFftSpectrum[RANGE_FFT_SIZE * chirpIdx + rangeBin]);
                    }
                    fprintf(handle, "\n");
                }
            }
            fprintf(handle, "%s\n", "e");
            fflush(handle);

            /****  Plot raw ADC data (DC corrected and windowed (optionally)) ****/
            handle = get_window_handle((int)PLOT_RAW_ADC_DATA);
            fprintf(handle, "plot [%f:%f] [%f:%f] '-' with lines title \"frame number: %d\" lt rgb \"blue\" \n", 0.0, (float)(frame_info->num_chirps * frame_info->num_samples_per_chirp * 0.5), -0.5, 0.5, frameNum);
            int gsampleIdx = 0;
            for (int rxIdx = 0; rxIdx < numAntennas; rxIdx++)
            {
                for (int chirpIdx = 0; chirpIdx < numChirps; chirpIdx++)
                {
                    int chirpOffset = ((chirpIdx << 2) + (rxIdx << 1)) * numSamplesPerChirp;
                    float* in_i = &inData[chirpOffset + 0];
                    float* in_q = &inData[chirpOffset + numSamplesPerChirp];
                    for (int sampleIdx = 0; sampleIdx < numSamplesPerChirp; sampleIdx++)
                    {
                        fprintf(handle, "%g %g\n", (double)gsampleIdx, (double)in_i[sampleIdx]);
                        gsampleIdx++;
                    }
                }
            }
            fprintf(handle, "%s\n", "e");
            fflush(handle);



            float dopplerStart = (0 - (DOPPLER_FFT_SIZE / 2)) * (1.0e6 / ((double)(DOPPLER_FFT_SIZE) * 9300));
            float dopplerEnd = (DOPPLER_FFT_SIZE - 1 - (DOPPLER_FFT_SIZE / 2)) * (1.0e6 / ((double)(DOPPLER_FFT_SIZE) * 9300));

            handle = get_window_handle((int)PLOT_DOPPLER_FFT);

            fprintf(handle, "plot [%f:%f] [%f:%f] '-' using ($2):($1):($3) with image\n", dopplerStart, dopplerEnd, rangeStart, rangeEnd);
            for (int rxIdx = 0; rxIdx < numAntennas; rxIdx++)
            {
                for (int rangeBin = 0; rangeBin < numRangeBins; rangeBin++)
                {
                    float* dopplerFftSpectrumPerChirp = &dopplerFftSpectrum[DOPPLER_FFT_SIZE * (rangeBin + rxIdx * numRangeBins)];
                    double range = (3.0 / 4.0) * ((double)numSamplesPerChirp / (double)RANGE_FFT_SIZE) * (double)(rangeBin);
                    for (int sampleIdx = 0; sampleIdx < DOPPLER_FFT_SIZE; sampleIdx++)
                    {
                        double doppler_freqs = (sampleIdx - (DOPPLER_FFT_SIZE / 2)) * (1.0e6 / ((double)(DOPPLER_FFT_SIZE) * 9300));
                        fprintf(handle, "%g %g %g\n", range, doppler_freqs, (double)dopplerFftSpectrumPerChirp[sampleIdx]);
                    }
                    fprintf(handle, "\n");
                }
            }
            fprintf(handle, "%s\n", "e");
            fflush(handle);

        }

        free(rangeFft);
        free(dopplerFft);
        free(rangeFftSpectrum);
        free(dopplerFftSpectrum);
    }
}


