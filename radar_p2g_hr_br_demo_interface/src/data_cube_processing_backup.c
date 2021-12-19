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
#define MAX_NUM_FILTERED_RANGE_BINS (20)
#define MAX_FRAME_BUFFER_LEN    (20) // number of historical frames to keep data for
#define MAX_NUM_PEAKS_PER_FRAME (20) // number of raw peaks observed to be fulfilling threshold criterion
#define NUM_PEAKS_PER_FRAME     (10)  // number of peaks with highest magnitude
#define MAX_FRAME_INFO_HIST_LEN (10) // frames to consider for peak filtering

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

static int frameCount = 0;
static int numHistFrames = 0;
static int numPeaksHist[MAX_FRAME_BUFFER_LEN];
static peakInfo_t peakInfoHistBuffer[MAX_FRAME_BUFFER_LEN][MAX_NUM_PEAKS_PER_FRAME] = {0};
static filteredPeakInfo_t filteredPeakInfo[MAX_NUM_PEAKS_PER_FRAME] = {0};

float  adcCalibrationData[256 * 4];
float* p_adcCalibrationData[4] = { &adcCalibrationData[256 * 0], &adcCalibrationData[256 * 1], &adcCalibrationData[256 * 2], &adcCalibrationData[256 * 3] };

// It's actually Kaiser Window ;-)
static float sKaiserTimeWindow[] = {
  0.014873, 0.017040, 0.019341, 0.021781, 0.024363, 0.027090, 0.029966, 0.032993, 0.036174, 0.039513, 0.043012, 0.046674, 0.050501, 0.054497, 0.058662, 0.063000,
  0.067513, 0.072202, 0.077070, 0.082117, 0.087346, 0.092758, 0.098353, 0.104134, 0.110100, 0.116252, 0.122591, 0.129118, 0.135831, 0.142731, 0.149818, 0.157091,
  0.164550, 0.172193, 0.180020, 0.188028, 0.196217, 0.204585, 0.213130, 0.221850, 0.230741, 0.239802, 0.249030, 0.258422, 0.267975, 0.277684, 0.287547, 0.297559,
  0.307717, 0.318015, 0.328451, 0.339018, 0.349712, 0.360528, 0.371461, 0.382505, 0.393654, 0.404902, 0.416245, 0.427674, 0.439184, 0.450769, 0.462422, 0.474135,
  0.485902, 0.497716, 0.509569, 0.521455, 0.533365, 0.545292, 0.557228, 0.569166, 0.581097, 0.593013, 0.604907, 0.616771, 0.628595, 0.640373, 0.652094, 0.663753,
  0.675339, 0.686844, 0.698261, 0.709581, 0.720795, 0.731895, 0.742874, 0.753722, 0.764431, 0.774993, 0.785401, 0.795646, 0.805720, 0.815615, 0.825325, 0.834840,
  0.844153, 0.853258, 0.862147, 0.870812, 0.879247, 0.887446, 0.895400, 0.903105, 0.910553, 0.917739, 0.924657, 0.931300, 0.937665, 0.943744, 0.949534, 0.955030,
  0.960226, 0.965120, 0.969705, 0.973980, 0.977939, 0.981581, 0.984901, 0.987897, 0.990567, 0.992907, 0.994918, 0.996595, 0.997939, 0.998948, 0.999621, 0.999958,
  0.999958, 0.999621, 0.998948, 0.997939, 0.996595, 0.994918, 0.992907, 0.990567, 0.987897, 0.984901, 0.981581, 0.977939, 0.973980, 0.969705, 0.965120, 0.960226,
  0.955030, 0.949534, 0.943744, 0.937665, 0.931300, 0.924657, 0.917739, 0.910553, 0.903105, 0.895400, 0.887446, 0.879247, 0.870812, 0.862147, 0.853258, 0.844153,
  0.834840, 0.825325, 0.815615, 0.805720, 0.795646, 0.785401, 0.774993, 0.764431, 0.753722, 0.742874, 0.731895, 0.720795, 0.709581, 0.698261, 0.686844, 0.675339,
  0.663753, 0.652094, 0.640373, 0.628595, 0.616771, 0.604907, 0.593013, 0.581097, 0.569166, 0.557228, 0.545292, 0.533365, 0.521455, 0.509569, 0.497716, 0.485902,
  0.474135, 0.462422, 0.450769, 0.439184, 0.427674, 0.416245, 0.404902, 0.393654, 0.382505, 0.371461, 0.360528, 0.349712, 0.339018, 0.328451, 0.318015, 0.307717,
  0.297559, 0.287547, 0.277684, 0.267975, 0.258422, 0.249030, 0.239802, 0.230741, 0.221850, 0.213130, 0.204585, 0.196217, 0.188028, 0.180020, 0.172193, 0.164550,
  0.157091, 0.149818, 0.142731, 0.135831, 0.129118, 0.122591, 0.116252, 0.110100, 0.104134, 0.098353, 0.092758, 0.087346, 0.082117, 0.077070, 0.072202, 0.067513,
  0.063000, 0.058662, 0.054497, 0.050501, 0.046674, 0.043012, 0.039513, 0.036174, 0.032993, 0.029966, 0.027090, 0.024363, 0.021781, 0.019341, 0.017040, 0.014873
};


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

static void applyTimeWindow(float *in, int len)
{
  int decIdx = 0;
  int decFactor = BLACKMAN_TIME_WINDOW_LEN / len;
  assert(decFactor >= 1);
  for (int i = 0; i < len; i++)
  {
    in[i] = in[i] * sKaiserTimeWindow[decIdx];
    decIdx += decFactor;
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
  int antennaNum,
  int doFiltering,
  int timeWindowEnabled
)
{
  // Extract frame info 
  int numChirps = frame_info->num_chirps;
  int numSamplesPerChirp = frame_info->num_samples_per_chirp;
  int frameNum = frame_info->frame_number;
  float* inData = frame_info->sample_data;
  int numAntennas = 1; //int numAntennas = frame_info->num_rx_antennas;

  // Calculate required size for range and doppler ffts
  int rangeFftBufferLen   = RANGE_FFT_SIZE * numChirps * numAntennas;
  int dopplerFftBufferLen = DOPPLER_FFT_SIZE * (RANGE_FFT_SIZE >> 1) * numAntennas;
  int interpFactor = BLACKMAN_TIME_WINDOW_LEN / numSamplesPerChirp;

  // Allocate dynamic memories here
  float* rangeFft           = (float *) malloc(sizeof(float) * rangeFftBufferLen * 2); // multiply by 2 to accomodate complex values
  float* rangeFftSpectrum   = (float *) malloc(sizeof(float) * rangeFftBufferLen);
  float* rangeFftSpectrumAccum = (float*)malloc(sizeof(float) * RANGE_FFT_SIZE);

  float* dopplerFftSpectrum = (float*) malloc(sizeof(float) * dopplerFftBufferLen);
  float* dopplerFft = (float*)malloc(sizeof(float) * DOPPLER_FFT_SIZE * 2); // multiply by 2 to accomodate complex values

  // Set buffer to initial entries of zero (0)
  memset((void*)rangeFftSpectrum,   0, sizeof(float) * rangeFftBufferLen);
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
      float* rangeFftPerChirp         = &rangeFft        [RANGE_FFT_SIZE * (chirpIdx + rxIdx * numChirps) * 2];
      float* rangeFftSpectrumPerChirp = &rangeFftSpectrum[RANGE_FFT_SIZE * (chirpIdx + rxIdx * numChirps)    ];

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
        applyTimeWindow(in_i, numSamplesPerChirp);
        applyTimeWindow(in_q, numSamplesPerChirp);
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
    for (int i = 1; i < (RANGE_FFT_SIZE >> 1) -1; i++)
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

#if 0
      // normalize doppler spectrum
#endif
    }

    free(in_i);
    free(in_q);
   }

   normalize_data(dopplerFftSpectrum, dopplerFftBufferLen);


   // Peak finding code
   int cIndex = frameCount % MAX_FRAME_BUFFER_LEN;

   float threshold = 0.25;
   float rangeDistanceThreshold = 0.25;
   float dopplerDistanceThreshold = 0.5;
   float occuranceThreshold = 0.8;

   float maxValue;
   int peakIdx = 0;
   int numRangeBins = (RANGE_FFT_SIZE >> 1); // Consider only positive frequencies of range fft
   for (int rxIdx = 0; rxIdx < numAntennas; rxIdx++)
   {
     for (int rangeBin = 1; rangeBin < numRangeBins-1; rangeBin++)
     {
       float* dopplerFftSpectrumPerChirp = &dopplerFftSpectrum[DOPPLER_FFT_SIZE * (rangeBin + rxIdx * numRangeBins)];
       for (int sampleIdx = 1; sampleIdx < DOPPLER_FFT_SIZE-1; sampleIdx++)
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
             peakInfoHistBuffer[cIndex][peakIdx].range       = range;
             peakInfoHistBuffer[cIndex][peakIdx].dopplerFreq = doppler_freqs;
             peakInfoHistBuffer[cIndex][peakIdx].peakMag     = dopplerFftSpectrumPerChirp[sampleIdx];
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
       filteredPeakInfo[i].isValid     = false;
       filteredPeakInfo[i].count       = 0;
       filteredPeakInfo[i].range       = 0;
       filteredPeakInfo[i].peakMag     = 0;
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

             if ((abs(peakInfo1.range        - peakInfo2.range           ) < rangeDistanceThreshold  ) &&
                ((abs(peakInfo1.dopplerFreq) - abs(peakInfo2.dopplerFreq)) < dopplerDistanceThreshold))
             {
               filteredPeakInfo[i].count++;
               if (filteredPeakInfo[i].count == 1)
               {
                 filteredPeakInfo[i].isValid     = true;
                 filteredPeakInfo[i].dopplerFreq = abs(peakInfo1.dopplerFreq);
                 filteredPeakInfo[i].range       = peakInfo1.range;
                 filteredPeakInfo[i].peakMag     = peakInfo1.peakMag;
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

       if ((peakInfo1.count >= (occuranceThreshold *(float)MAX_FRAME_INFO_HIST_LEN)) && (peakInfo1.isValid==true))
       {
         oomPoint = true;
         // remove duplicates
         for (int j = i+1; j < maxNumPeaks; j++)
         {
           filteredPeakInfo_t peakInfo2 = filteredPeakInfo[j];
         
           if ((abs(peakInfo1.range       - peakInfo2.range)       < rangeDistanceThreshold) &&
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
       float rangeEnd = (3.0 / 16.0) * numRangeBins;
       float dopplerStart = (0 - (DOPPLER_FFT_SIZE / 2)) * (1.0e6 / ((double)(DOPPLER_FFT_SIZE) * 6300));
       float dopplerEnd = (DOPPLER_FFT_SIZE - 1 - (DOPPLER_FFT_SIZE / 2)) * (1.0e6 / ((double)(DOPPLER_FFT_SIZE) * 6300));

       FILE* handle = get_window_handle((int)PLOT_RANGE_FFT);

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
   float rangeEnd = (3.0 / 16.0) * (numRangeBins - 1);

   FILE* handle = get_window_handle((int)PLOT_RAW_ADC_DATA);

   fprintf(handle, "plot [%f:%f] [%f:%f] '-' using ($1):($2):($3) with image\n", (float)0.0, (float)49.0, rangeStart, rangeEnd);
   //fprintf(handle, "plot '-' using 2:1:3 with image\n");
   for (int rxIdx = 0; rxIdx < numAntennas; rxIdx++)
   {
       for (int rangeBin = 0; rangeBin < numRangeBins; rangeBin++)
       {
           double range = (3.0 / 16.0) * rangeBin;
           for (int chirpIdx = 0; chirpIdx < numChirps; chirpIdx++)
           {
               fprintf(handle, "%g %g %g\n", (double)chirpIdx, range, (double)rangeFftSpectrum[256 * chirpIdx + rangeBin]);
           }
           fprintf(handle, "\n");
       }
   }
   fprintf(handle, "%s\n", "e");
   fflush(handle);

   ///****  Plot raw ADC data (DC corrected and windowed (optionally)) ****/ 
   //FILE* handle = get_window_handle((int)PLOT_RAW_ADC_DATA);
   //fprintf(handle, "plot '-' with lines title \"frame number: %d\" lt rgb \"blue\" \n", /*frame_info->num_chirps* frame_info->num_samples_per_chirp,*/ frameNum);
   //int gsampleIdx = 0;
   //for (int rxIdx = 0; rxIdx < numAntennas; rxIdx++)
   //{
   //  for (int chirpIdx = 0; chirpIdx < numChirps; chirpIdx++)
   //  {
   //    int chirpOffset = ((chirpIdx << 2) + (rxIdx << 1)) * numSamplesPerChirp;
   //    float* in_i = &inData[chirpOffset + 0];
   //    float* in_q = &inData[chirpOffset + numSamplesPerChirp];
   //    for (int sampleIdx = 0; sampleIdx < numSamplesPerChirp; sampleIdx++)
   //    {
   //      fprintf(handle, "%g %g\n", (double)gsampleIdx, (double)in_i[sampleIdx]);
   //      gsampleIdx++;
   //    }
   //  }
   //}   
   //fprintf(handle, "%s\n", "e");
   //fflush(handle);



   float dopplerStart = (0 - (DOPPLER_FFT_SIZE / 2)) * (1.0e6 / ((double)(DOPPLER_FFT_SIZE) * 6300));
   float dopplerEnd   = (DOPPLER_FFT_SIZE - 1 - (DOPPLER_FFT_SIZE / 2)) * (1.0e6 / ((double)(DOPPLER_FFT_SIZE) * 6300));

   handle = get_window_handle((int)PLOT_DOPPLER_FFT);

   fprintf(handle, "plot [%f:%f] [%f:%f] '-' using ($2):($1):($3) with image\n", dopplerStart, dopplerEnd, rangeStart, rangeEnd);
   for (int rxIdx = 0; rxIdx < numAntennas; rxIdx++)
   {
     for (int rangeBin = 0; rangeBin < numRangeBins; rangeBin++)
     {
       float* dopplerFftSpectrumPerChirp = &dopplerFftSpectrum[DOPPLER_FFT_SIZE * (rangeBin + rxIdx * numRangeBins)];
       double range = (3.0 / 16.0) * rangeBin;
       for (int sampleIdx = 0; sampleIdx < DOPPLER_FFT_SIZE; sampleIdx++)
       {
         double doppler_freqs = (sampleIdx - (DOPPLER_FFT_SIZE / 2)) * (1.0e6 / ((double)(DOPPLER_FFT_SIZE) * 6300));
         fprintf(handle, "%g %g %g\n", range, doppler_freqs, (double)dopplerFftSpectrumPerChirp[sampleIdx]);
       }
       fprintf(handle, "\n");
     }
   }
   fprintf(handle, "%s\n", "e");
   fflush(handle);


   free(rangeFft);
   free(dopplerFft);
   free(rangeFftSpectrum);
   free(dopplerFftSpectrum);
}


