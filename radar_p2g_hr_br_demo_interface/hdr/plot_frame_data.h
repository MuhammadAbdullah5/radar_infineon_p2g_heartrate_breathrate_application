#ifndef __PLOT_FRAME_DATA_H__
#define __PLOT_FRAME_DATA_H__

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
#define ENABLE_PLOT_RAW_ADC_DATA
#define ENABLE_PLOT_RANGE_FFT
#define ENABLE_PLOT_DOPPLER_FFT
#define ENABLE_PLOT_DETECTED_OBJECTS
/****************************************************************
 TYPES
*****************************************************************/
	// Add here plots that one wants to add
	// Each etnry must have an associated define and entry in openWindows()
typedef enum
{
#ifdef ENABLE_PLOT_RAW_ADC_DATA
PLOT_RAW_ADC_DATA,
#endif
#ifdef ENABLE_PLOT_RANGE_FFT
PLOT_RANGE_FFT,
#endif
#ifdef ENABLE_PLOT_DOPPLER_FFT
PLOT_DOPPLER_FFT,
#endif
#ifdef ENABLE_PLOT_DETECTED_OBJECTS
PLOT_DETECTED_OBJECTS,
#endif
PLOT_COUNT
} PlotId_t;

/****************************************************************
 DATA
*****************************************************************/


/****************************************************************
 LOCAL FUNCTIONS
*****************************************************************/

/****************************************************************
 EXPORTED FUNCTIONS
*****************************************************************/
/** Open a new GNU plot window and set it's plotting attributes e.g.
*   title, labels and tics. This function will return a unique ID
*   to identify a plot window and passed in when plotting on a particular
*   window or closing a particular window.
*   A return code of 0 or greater is a unique window ID
*   A return code of -1 means that any of essential input arguments are not valid.
*   A return code of -2 means that internal error occured while opening gnuplot window
*/

  int open_gnu_plot_window(const char* window_title,
    const char* plot_title,
    const char* x_label,
    const char* y_label,
    const char* x_range,
    const char* y_range,
    const char* xtics,
    const char* ytics,
      int xPosition,
      int yPosition,
      int width,
      int height,
    bool  grid_enabled);

  /** Plot the data in a particular GNUPLOT window identified by window_id
  * parameter. Specify data array pointer, data length, a legend for data
  * and whether plotting over existing plot or replace existing plot by using
  * parameter hold_on)enabled
  */
  int plot_gnu_plot_window(int    window_id,
    const float* data,
    int    data_len,
    bool   hold_on_enabled,
    const char* legend);

  /** Close a GNUPLOT window identified by input argument window_id.
  */
  int close_gnu_plot_window(int window_id);

  void openWindows(const Frame_Info_t* frame_info, int width, int height);

  void closeWindows();

  void plotData(
    const Frame_Info_t *frame_info,
    const float* rawAdcData,
    const float* rangeFftData,
    const float* dopplerFftData
  );

  FILE* get_window_handle(int window_id);

#ifdef __cplusplus
}
#endif

#endif // __PLOT_FRAME_DATA_H__
