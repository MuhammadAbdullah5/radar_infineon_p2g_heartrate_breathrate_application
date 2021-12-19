#ifndef __DATA_CUBE_PROCESSING_H__
#define __DATA_CUBE_PROCESSING_H__

#ifdef __cplusplus
extern "C" {
#endif

  /****************************************************************
   INCLUDES
  *****************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "EndpointRadarBase.h"

  /****************************************************************
   DEFINES
  *****************************************************************/
#define BLACKMAN_TIME_WINDOW_LEN 256

#define RANGE_FFT_SIZE   512
#define DOPPLER_FFT_SIZE 256

  /****************************************************************
   TYPES
  *****************************************************************/

  /****************************************************************
   DATA
  *****************************************************************/

  /****************************************************************
   LOCAL FUNCTIONS
  *****************************************************************/

  /****************************************************************
   EXPORTED FUNCTIONS
  *****************************************************************/

  void dataCubeProcessing(
    const Frame_Info_t* frame_info,
    int           averageAntenna,
    int           doFiltering,
    int           timeWindowEnabled,
    bool          sApplyCalibration
  );



#ifdef __cplusplus
}
#endif

#endif // __PLOT_FRAME_DATA_H__
