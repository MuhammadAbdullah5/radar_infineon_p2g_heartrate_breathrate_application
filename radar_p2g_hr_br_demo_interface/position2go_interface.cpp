/*
 ===================================================================================
 Name        : extract_raw_data.c
 Author      : Infineon Technologies
 Version     :
 Copyright   : 2014-2017, Infineon Technologies AG
 Description : Example of how to extract raw data using the C communication library
 ===================================================================================
 */

 /*
	* Copyright (c) 2014-2017, Infineon Technologies AG
	* All rights reserved.
	*
	* Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
	* following conditions are met:
	*
	* Redistributions of source code must retain the above copyright notice, this list of conditions and the following
	* disclaimer.
	*
	* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
	* disclaimer in the documentation and/or other materials provided with the distribution.
	*
	* Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
	* products derived from this software without specific prior written permission.  
	*
	* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
	* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
	* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
	* WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
	*/

#include <conio.h>
#include <stdio.h>
#include <string.h>
#include "Protocol.h"
#include "COMPort.h"
#include "EndpointRadarBase.h"
#include "EndpointTargetDetection.h"
#include "EndpointCalibration.h"
#include "plot_frame_data.h"
#include <Windows.h>
#include "data_cube_processing.h"
#include "fftw3.h"
#include "assert.h"
#include <save_frame_data.h>

#define AUTOMATIC_DATA_FRAME_TRIGGER  (1)		    // define if automatic trigger is active or not
#define AUTOMATIC_DATA_TRIGER_TIME_US (1000000)	// get ADC data each 1ms in automatic trigger mode
#define NUM_FRAME_TO_SAVE (1000)

//#define GET_TARGET_INFO

extern "C" float *p_adcCalibrationData[4];
static int sApplyFiltering = 0; // dont apply filter by default
static int sAntennaSelection = 0; // use average results from both rx antennas by default
static int sTimeWindowEnabled = 0; // time window disabled by default
static bool    sStartRawDataSave = false;
static double sTargetRange[2] = { 0, 0 };
static bool firstFrameReceived = false;
static int horizontal;
static int vertical;
static bool sApplyCalibration = false;


void get_adc_calibration_data(
	void*        context,
	int32_t      protocol_handle,
	uint8_t      endpoint,
	const float* calibration_data_ptr,
	uint16_t     num_of_samples
)
{
	// This is hardcoded...
	assert(num_of_samples == 1024);

	for (int i = 0; i < 256; i++)
	{
		p_adcCalibrationData[0][i] = calibration_data_ptr[256 * 0 + i];
		p_adcCalibrationData[1][i] = calibration_data_ptr[256 * 1 + i];
		p_adcCalibrationData[2][i] = calibration_data_ptr[256 * 2 + i];
		p_adcCalibrationData[3][i] = calibration_data_ptr[256 * 3 + i];
	}

	printf("Calibration: Requesting Flash ADC calibration data, Status: COMPLETED!\n");
}


void get_temperature_value(
	void* context, 
	int32_t protocol_handle,
	uint8_t endpoint, 
	uint8_t temp_sensor,
	int32_t temperature_001C
) 
{
	printf("Temperature Reading: %f\n", (double)temperature_001C * (float)0.001);
}

#ifdef GET_TARGET_INFO
void target_processing(void*   context,
                       int32_t protocol_handle,
                       uint8_t endpoint,
                       const   Target_Info_t* targets,
                       uint8_t num_targets)
{
	for (size_t i = 0; i < num_targets; i++)
	{
		printf("target_id: %4d, range: % 6.2f (meters), gain: % 6.2f, speed: % 6.2f, angle:% 6.2f\n", 
			targets[i].target_id, 
			targets[i].radius * 0.01, 
			targets[i].level, 
			targets[i].radial_speed, 
			targets[i].azimuth
		);
	}
}
#endif

	// called every time ep_radar_base_get_frame_data method is called to return measured time domain signals
void received_frame_data(
	void* context,
	int32_t protocol_handle,
	uint8_t endpoint,
	const Frame_Info_t* frame_info
)
{
	printf("Signal Processing: frame %d\n", frame_info->frame_number);

	if (firstFrameReceived == false)
	{
		firstFrameReceived = true;
		HWND consoleWindow = GetConsoleWindow();
		SetWindowPos(consoleWindow, 0, 0, 525, 820, 500, SWP_NOZORDER);
		RECT desktop;
		// Get a handle to the desktop window
		const HWND hDesktop = GetDesktopWindow();

		// Get the size of screen to the variable desktop
		GetWindowRect(hDesktop, &desktop);
		horizontal = desktop.right;
		vertical = desktop.bottom;

				// Open plotting windows once according to frame inforation received
		openWindows(frame_info, horizontal, vertical);
	}

	// call data cube processing
 	dataCubeProcessing(
		frame_info, 
		sAntennaSelection,
		sApplyFiltering,
		sTimeWindowEnabled,
		sApplyCalibration
	);


	// Convert data to double format for saving to MATLAB compatible MAT-file
	double data_len = (double)frame_info->num_samples_per_chirp * frame_info->num_chirps * frame_info->num_rx_antennas * (frame_info->data_format == 0 ? 1 : 2);
	double* dataDouble = (double*)malloc(data_len * sizeof(double));
	for (size_t i = 0; i < data_len; i++)
	{
		dataDouble[i] = frame_info->sample_data[i];
	}

	printf("Storing Results:   frame %d\n", frame_info->frame_number);
	saveRadarRawData(
		dataDouble,
		data_len,
		sTargetRange,
		(double)frame_info->frame_number,
		(double)NUM_FRAME_TO_SAVE,
		sStartRawDataSave
	);

	free(dataDouble);
}

int radar_auto_connect(void)
{
	int radar_handle = 0;
	int num_of_ports = 0;
	char comp_port_list[256];
	char* comport;
	const char* delim = ";"; 

	//----------------------------------------------------------------------------

	num_of_ports = com_get_port_list(comp_port_list, (size_t)256);

	if (num_of_ports == 0)
	{
		return -1;
	}
	else
	{
		comport = strtok(comp_port_list, delim);

		while (num_of_ports > 0)
		{
			num_of_ports--;

			// open COM port
			radar_handle = protocol_connect(comport);

			if (radar_handle >= 0)
			{
				break;
			}

			comport = strtok(NULL, delim);
		}

		return radar_handle;
	}

}

int main(void)
{
	int res = -1;
	int protocolHandle = 0;
	int endpointRadarBase = 0;
#ifdef GET_TARGET_INFO
	int endpointTargetDetection = 0;
#endif

	int endpointCalibration = 0;

	// open COM port
	protocolHandle = radar_auto_connect();

	// get endpoint ids
	if (protocolHandle >= 0)
	{
		for (int i = 1; i <= protocol_get_num_endpoints(protocolHandle); ++i) {
			// current endpoint is radar base endpoint
			if (ep_radar_base_is_compatible_endpoint(protocolHandle, i) == 0) {
				endpointRadarBase = i;
			}
#ifdef GET_TARGET_INFO
			else if (ep_targetdetect_is_compatible_endpoint(protocolHandle, i) == 0)
			{
				endpointTargetDetection = i;
			}
#endif
			else if (ep_calibration_is_compatible_endpoint(protocolHandle, i) == 0)
			{
				endpointCalibration = i;
			}
		}
	}


	if ((endpointRadarBase >= 0) && 
#ifdef GET_TARGET_INFO
		(endpointTargetDetection >= 0) && 
#endif
		(endpointCalibration >= 0))
	{

#ifdef GET_TARGET_INFO
		// register call back function to get information list of detected targets
		ep_targetdetect_set_callback_target_processing(target_processing, NULL);
#endif

		// register call back functions for adc data, range and doppler frequency data
		ep_radar_base_set_callback_data_frame(received_frame_data, NULL);

		// register call back function to monitor temperature
		ep_radar_base_set_callback_temperature(get_temperature_value, NULL);

		// register callback function for receiving adc calibration data
		ep_calibration_set_callback_adc_calibration_data(get_adc_calibration_data, NULL);

		// enable/disable automatic trigger
		if (AUTOMATIC_DATA_FRAME_TRIGGER)
		{
			res = ep_radar_base_set_automatic_frame_trigger(protocolHandle,
				endpointRadarBase,
				AUTOMATIC_DATA_TRIGER_TIME_US);
		}
		else
		{
			res = ep_radar_base_set_automatic_frame_trigger(protocolHandle,
				endpointRadarBase,
				0);
		}


		printf("Press o to get antenna 1 results!\n");
		printf("Press t to get antenna 2 results!\n");
		printf("Press a to get antenna 1&2 averaged results!\n");

		printf("Press c to calibrate radar sensor!\n");
		printf("Press d to de-calibrate radar sensor!\n");

		printf("Press f to apply doppler low pass filter!\n");
		printf("Press u to up-apply doppler low pass filter!\n");


		printf("Press e to end the plot session!\n");

		// Send message to board to get frame data
		while (1)
		{

#ifdef GET_TARGET_INFO
			// get result of algorithm processing
			res = ep_targetdetect_get_targets(protocolHandle, endpointTargetDetection);
#endif

			// get raw data
			res = ep_radar_base_get_frame_data(protocolHandle, endpointRadarBase, 1);

			// disable by default
			sStartRawDataSave = false;

			// get which keystroke was pressed
			if (_kbhit())
			{
				char hit = _getch();

				if (hit == 'e')
				{
					break;
				}
				else if (hit == 'c')
				{
					printf("Calibration: Requesting ADC calibration, Status: STARTED!\n");
					// do calibration in SRAM memory required for each power up/down session
					res = ep_calibration_set_adc_sram_calibration_data(protocolHandle, endpointCalibration);
					// getting Flash contents of calibration
					res = ep_calibration_get_adc_sram_calibration_data(protocolHandle, endpointCalibration);
					sApplyCalibration = true;
				}
				else if (hit == 'd')
				{
					printf("Calibration: Requesting ADC de-calibration data, Status: STARTED!\n");
					// clear the calibration data
					res = ep_calibration_clear_adc_sram_calibration_data(protocolHandle, endpointCalibration);
					// getting Flash contents of calibration
					res = ep_calibration_get_adc_sram_calibration_data(protocolHandle, endpointCalibration);
					sApplyCalibration = false;
				}
				else if (hit == 'f')
				{
					printf("Filtering: Low pass filtering ENABLED\n");
					sApplyFiltering = 1;
				}
				else if (hit == 'u')
				{
					printf("Filtering: Low pass filtering DISABLED\n");
					sApplyFiltering = 0;
				}
				else if (hit == 'a')
				{
					printf("Antennas: Showing antenna averaged results!\n");
					sAntennaSelection = 0;
				}
				else if (hit == 'o')
				{
					printf("Antennas: Showing antenna 1 results!\n");
					sAntennaSelection = 1;
				}
				else if (hit == 't')
				{
					printf("Antennas: Showing antenna 2 results!\n");
					sAntennaSelection = 2;
				}
				else if (hit == 'w')
				{
					printf("Time Domain Window: Time domain window ENABLED!\n");
					sTimeWindowEnabled = 1;
				}
				else if (hit == 'n')
				{
					printf("Time Domain Window: Time domain window DISABLED!\n");
					sTimeWindowEnabled = 0;
				}
				else if (hit == 's')
				{
					sStartRawDataSave = true;
				}

			}
		}


		closeWindows();

	}

	return	;
}  
