/****************************************************************
 INCLUDES
*****************************************************************/
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "data_cube_processing.h"
#include "plot_frame_data.h"

/****************************************************************
 DEFINES
*****************************************************************/
#define SPRINTF_CHAR_BUFFLEN (500)

/****************************************************************
 TYPES
*****************************************************************/

/****************************************************************
 DATA
*****************************************************************/
// Use num_window_handles as window_ID
// NOTE: access FILE from window_ID as window_handle[window_ID-1]
static int   num_window_handles = 0;
static FILE** window_handle = NULL;
static int s_window_id[PLOT_COUNT ? PLOT_COUNT : 1];

static bool _firstTime = true;

typedef struct fifo_s
{
	int fill;
	int plot;
}fifo_t;
static fifo_t fifo = {.fill=0, .plot=0};
/****************************************************************
 LOCAL FUNCTIONS
*****************************************************************/
void openWindows(const Frame_Info_t* frame_info, 
	int width, 
	int height
)
{
#ifdef	ENABLE_PLOT_RAW_ADC_DATA
	char string1[100];
	sprintf(string1, "0:%d", frame_info->num_chirps * frame_info->num_samples_per_chirp);
	s_window_id[PLOT_RAW_ADC_DATA] =
		open_gnu_plot_window("Heart beat detection demo", "Time domain data", "samples", "magnitude", "*:*", "*:*", NULL, NULL, 0,0, (int)(0.4792 * (float)width), (int)(0.5 * (float)height), true);
#endif
#ifdef ENABLE_PLOT_RANGE_FFT
	s_window_id[PLOT_RANGE_FFT] =
		open_gnu_plot_window("Heart beat detection demo", "Heat map Pulse no. vs. Range", "Pulse Number", "Range (meters)", "*:*", "*:*", NULL, NULL, (int)(0.4792 * (float)width), (int)(0.5 * (float)height), (int)(0.4792 * (float)width), (int)(0.5 * (float)height), true);
#endif
	
#ifdef ENABLE_PLOT_DOPPLER_FFT
		s_window_id[PLOT_DOPPLER_FFT] =
		open_gnu_plot_window("Heart beat detection demo", "Heat map Doppler frequency vs. Range", "Doppler Frequency (Hz)", "Range (meters)", "*:*", "*:*", NULL, NULL, (int)(0.4792 * (float)width), 0, (int)(0.4792 * (float)width), (int)(0.5 * (float)height), true);
#endif

#ifdef ENABLE_PLOT_DETECTED_OBJECTS
		s_window_id[PLOT_DETECTED_OBJECTS] =
			open_gnu_plot_window("Heart beat detection demo", "Detected Targets", "Doppler Frequency (Hz)", "Range (meters)", "*:*", "*:*", NULL, NULL, (int)(0.4792 * (float)width), 0, (int)(0.4792 * (float)width), (int)(0.5 * (float)height), true);
#endif

}    

void closeWindows()
{
	for (size_t i = 0; i < (size_t)num_window_handles; i++)
	{
		close_gnu_plot_window(s_window_id[i]);
	}
}

/** This function takes input old window handle
* and re-allocates handle buffer memory with a
* plus 1 size and returns the window ID.
*/
static void push_new_window_handle()
{
	// increase handle number
	num_window_handles++;

	// assign newer memory
	FILE** new_mem = (FILE**)malloc(num_window_handles * sizeof(FILE*));
	if (window_handle != NULL)
	{
		(void*)memcpy(new_mem, window_handle, sizeof(FILE*) * (num_window_handles - 1));
		free(window_handle);
	}
	window_handle = new_mem;
}

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

/****************************************************************
 EXPORTED FUNCTIONS
*****************************************************************/

void plotData(
	const Frame_Info_t* frame_info,
	const float* rawAdcData,
	const float* rangeFftData,
	const float* dopplerFftData
)
{
	char legend[100];
	sprintf(legend, "frameNum: %d", frame_info->frame_number);

	// Plot time domain data
#ifdef	ENABLE_PLOT_RAW_ADC_DATA
	plot_gnu_plot_window(
		// this data contains first Nc*Ns samples of real values and then Nc*Ns values of imaginary samples
		s_window_id[PLOT_RAW_ADC_DATA], rawAdcData, frame_info->num_samples_per_chirp * frame_info->num_chirps, false, legend);
#endif
#ifdef	ENABLE_PLOT_RANGE_FFT

	// Save file to local directory
	char fileNameBuffer[100];
	sprintf(fileNameBuffer, "range_spectrum_3d_%d", fifo.fill);
	FILE* fileRangePlot   = fopen(fileNameBuffer, "w");

	sprintf(fileNameBuffer, "doppler_spectrum_3d_%d", fifo.fill);
	FILE* fileDopplerPlot = fopen(fileNameBuffer, "w");

	int sampleIdx = 0;
	for (int row = 0; row < frame_info->num_chirps; row++)
	{
		for (int col = 0; col < (RANGE_FFT_SIZE >> 1); col++)
		{
			float xAxisVal = col * (3.0 / 16.0);
			float yAxisVal = (float)row;
			fprintf(fileRangePlot, "%f  %f  %f \n", xAxisVal, yAxisVal, rangeFftData[sampleIdx]);
			sampleIdx++;
		}
	}

	sampleIdx = 0;
	for (int col = 0; col < (RANGE_FFT_SIZE >> 1); col++)
	{
		for (int row = 0; row < DOPPLER_FFT_SIZE; row++)
		{
			float rangeAxisVal = col * (3.0 / 16.0);
			float fdAxisVal = -25.0 + row * 0.3906;
			fprintf(fileDopplerPlot, "%f  %f  %f \n", rangeAxisVal, fdAxisVal, dopplerFftData[sampleIdx]);
 			sampleIdx++;
		}
	}


	fflush(fileDopplerPlot);
	fflush(fileRangePlot);
	fclose(fileDopplerPlot);
	fclose(fileRangePlot);

	fifo.fill += 1;
	if (fifo.fill == 3)
		fifo.fill = 0;

	FILE* handle;
	char cmd[SPRINTF_CHAR_BUFFLEN];

	// Write range plot commands to gnuplot
	handle = window_handle[PLOT_RANGE_FFT];
	sprintf(fileNameBuffer, "range_spectrum_3d_%d", fifo.plot);
	sprintf(cmd, "plot \"%s\" with image pixels\n", fileNameBuffer);
	fprintf(handle, cmd);
	fflush(handle);

	// Write doppler plot commands to gnuplot
	handle = window_handle[PLOT_DOPPLER_FFT];
	sprintf(fileNameBuffer, "doppler_spectrum_3d_%d", fifo.plot);
	sprintf(cmd, "plot \"%s\" with image pixels\n", fileNameBuffer);
	fprintf(handle, cmd);
	fflush(handle);


	fifo.plot += 1;
	if (fifo.plot == 3)
	{
		fifo.plot = 0;
	}

#endif
}


/** Open a new GNU plot window and set it's plotting attributes e.g.
*   title, labels and tics. This function will return a unique ID
*   to identify a plot window and passed in when plotting on a particular
*   window or closing a particular window.
*   A return code of 0 or greater is a unique window ID
*   A return code of -1 means that any of essential input arguments are not valid.
*   A return code of -2 means that internal error occured while opening gnuplot window
*/

int open_gnu_plot_window(
	const char* window_title,
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
	bool  grid_enabled)
{
	if ((window_title == NULL) ||
		(plot_title == NULL) ||
		(x_label == NULL) ||
		(y_label == NULL) ||
		(x_range == NULL) ||
		(y_range == NULL)
		)
	{
		printf("New window %d could not be opened: current opened windows: %d\n", num_window_handles, num_window_handles);
		return -1;
	}

	// Try opening a window
	FILE* handle = _popen("gnuplot", "w");

	// if gnuplot window could be opened
	// set it's parameters
	if (handle)
	{
		// Allocate memory for newer handle and work out ID
		push_new_window_handle();
		int current_window_id = num_window_handles - 1;

		// open gnuplot window
		window_handle[current_window_id] = handle;

		// prepare string to send to gnuplot
		char terminalCmd[SPRINTF_CHAR_BUFFLEN];
		sprintf(terminalCmd,
			"set term wxt %d noraise title '%s' position %d,%d size 804,437\n"
			"set title  '%s' \n"
			"set xlabel '%s' \n"
			"set ylabel '%s' \n"
			"set xrange [%s] \n"
			"set yrange [%s] \n",
			current_window_id,
			window_title,
			xPosition,
			yPosition,
			plot_title,
			x_label,
			y_label,
			x_range,
			y_range);

		if (xtics != NULL)
		{
			sprintf(terminalCmd, "%s "
				"set xtics %s \n",
				terminalCmd,
				xtics);
		}

		if (ytics != NULL)
		{
			sprintf(terminalCmd, "%s "
				"set ytics %s \n",
				terminalCmd,
				ytics);
		}

		if (grid_enabled)
		{
			sprintf(terminalCmd, "%s "
				"%s \n",
				terminalCmd,
				"set grid\n");
		}

		// send attributes string to gnuplot window
		fprintf(handle, terminalCmd);

		printf("Successfuly opened gnu plot window %d\n", current_window_id);
	}
	else
	{
		printf("New window %d could not be opened: current opened windows: %d\n", num_window_handles, num_window_handles);
		return -2;
	}

	return num_window_handles - 1;
}

/** Plot the data in a particular GNUPLOT window identified by window_id
* parameter. Specify data array pointer, data length, a legend for data
* and whether plotting over existing plot or replace existing plot by using
* parameter hold_on)enabled
* Return 0 in case everything is fine.
* Return -1 in case of invalid window ID
* Return any other non-zero number in case of any other possible issue.
*/
int plot_gnu_plot_window(
	int    window_id,
	const float* data,
	int    data_len,
	bool   hold_on_enabled,
	const char* legend)
{
	if ((window_id >= 0) || (window_id <= num_window_handles))
	{
		FILE* handle = window_handle[window_id];
		char cmd[SPRINTF_CHAR_BUFFLEN];

		sprintf(cmd, "plot '-' with lines title \"live data: %s\" lt rgb \"blue\" \n", legend);
		fprintf(handle, cmd);

		// Print the sampled data which can be found in frame_info->sample_data
		int i;
		for (i = 0; i < data_len; i++)
		{
			fprintf(handle, "%g\n", (double)data[i]);
		}

		fprintf(handle, "%s\n", "e");
		fflush(handle);
	}
	else
	{
		return -1;
	}

	return 0;
}

/** Close a GNUPLOT window identified by input argument window_id.
*/
int close_gnu_plot_window(int window_id)
{
	_pclose(window_handle[window_id]);
	if ((window_handle) && (window_id == (num_window_handles-1)))
	{
		free(window_handle);
	}
	return 0;
}

FILE* get_window_handle(int window_id)
{
	if (window_handle[window_id])
	{
		return window_handle[window_id];
	}
	else
	{
		return NULL;
	}
}