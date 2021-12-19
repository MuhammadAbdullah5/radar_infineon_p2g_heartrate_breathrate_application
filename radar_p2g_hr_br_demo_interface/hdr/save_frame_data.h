#ifndef __SAVE_FRAME_DATA_H__
#define __SAVE_FRAME_DATA_H__

#include <stdint.h>
#include <stdbool.h>


/** This function saves the raw data from position2go board
* to matlab's .mat file in structure named raw_data
* This structure has following data format.
* 1) structure array Nx1 with each containing following elements
*   1a) frame_number
*   1b) detected_targets
*   1c) adc_data_len
*   1d) adc_data
*/

#ifdef __cplusplus
extern "C" {
#endif

  void saveRadarRawData(
    double* data,
    double  dataLen,
    double* targetRange,
    double  frameNum,
    double  totalFrames,
    bool    reset
  );

#ifdef __cplusplus
}
#endif



#endif // __SAVE_FRAME_DATA_H__
