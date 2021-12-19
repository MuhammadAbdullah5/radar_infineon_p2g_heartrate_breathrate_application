#include <string.h>
#include <stdbool.h>

#include <ClientConfig.h>
#include <ClientLog.h>
#include "Protocol.h"
#include "EndpointRadarBase.h"

#include "ClientComm.h"

#define RADAR_PRI_ADVANCE_USEC (0)

static int32_t s_endpointRadarBase = -1;
static bool s_dataCaptureActive = false;
static int32_t s_radarHandle = -1;
static Callback_Data_Frame_t* s_getFrameCallbackFcn = NULL;
static Client_Comm_Callback_Temperature_t* s_getTempCallbackFunc = NULL;

static int32_t ClientCommStopAutoTrigger(int32_t radarProtocolId)
{
	int32_t res = -1;
	if (s_endpointRadarBase >= 0)
	{
		int32_t res = ep_radar_base_set_automatic_frame_trigger(radarProtocolId, s_endpointRadarBase, 0);
		res = ((res >> 16) - s_endpointRadarBase);
		return res;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "EndpointRadarBase in radar board is not compatible with it's counterpart in C Comms library.");
	}
}

/** This function returns radar handle identifier which is greater or equal to 0
* In case of failure, following negative status codes are returned.
* CLIENT_COMM_STATUS_ERROR_COULD_NOT_OPEN_COM_PORT
* CLIENT_COMM_STATUS_ERROR_SENSOR_BOARD_NOT_COMPATIABLE
*/
int32_t clientCommConnectRadar()
{
	int radar_handle = 0;
	int num_of_ports = 0;
	char comp_port_list[256];
	char* comport;
	const char* delim = ";";

	// Get list of all com ports
	num_of_ports = com_get_port_list(comp_port_list, (size_t)256);

	if (num_of_ports == 0)
	{
		return CLIENT_COMM_STATUS_ERROR_COULD_NOT_OPEN_COM_PORT;
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
				s_radarHandle = radar_handle;
				break;
			}

			comport = strtok(NULL, delim);
		}

		// get endpoint ids
		if (radar_handle >= 0)
		{
			for (int32_t i = 1; i <= protocol_get_num_endpoints(radar_handle); ++i)
			{
				// current endpoint is radar base endpoint
				if (ep_radar_base_is_compatible_endpoint(radar_handle, i) == 0)
				{
					s_endpointRadarBase = i;
				}
			}
		}
		return radar_handle;
	}
}

Client_Comm_Callback_Data_Frame_t* clientCommGetDataCallbackFuncPtr()
{
	return s_getFrameCallbackFcn;
}

Client_Comm_Callback_Temperature_t* clientCommGetTempCallbackFuncPtr()
{
	return s_getTempCallbackFunc;
}


/** This function registers a callback with
* EndpointRadarBase in sensor board to receive
* frame data samples once clientCommGetDataFrame
* is issued. This registration event is saved
* internally such that dependent functions like
* clientCommSetPeriodFrameCapture can issue 
* warning of this callback function not being 
* registered. Callback_Data_Frame_t pointer 
* is declared in ClientComm.h 
*/
void clientCommRegisterDataCallback(
	int32_t radarProtocolId,
	Client_Comm_Callback_Data_Frame_t* callbackFcnReceiveData
)
{
	if (s_endpointRadarBase >= 0)
	{
		// register call back functions for adc data, range and doppler frequency data
		ep_radar_base_set_callback_data_frame(callbackFcnReceiveData, NULL);

		// keep an internal record for registration for any dependent functions
		s_getFrameCallbackFcn = callbackFcnReceiveData;
	}
}

/** This function registers callback function
* for reception of temperature information 
* from sensor board. Callback type is declared
* in file ClientComm.h
*/
void clientCommRegisterTemperatureCallback(
	Client_Comm_Callback_Temperature_t*callbackFcnTemperature
)
{
	if (s_endpointRadarBase >= 0)
	{
		// register call back function to monitor temperature
		ep_radar_base_set_callback_temperature(callbackFcnTemperature, NULL);

		// Keep an internal record
		s_getTempCallbackFunc = callbackFcnTemperature;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "EndpointRadarBase in radar board is not compatible with it's counterpart in C Comms library.");
	}
}

int32_t clientCommStartDataCapture(int32_t radarProtocolId, int32_t autoDataCaptureTriggerTimePeriodUsec)
{
	if (s_getFrameCallbackFcn)
	{
		if (s_endpointRadarBase >= 0)
		{
			int32_t res = ep_radar_base_set_automatic_frame_trigger(radarProtocolId, s_endpointRadarBase, autoDataCaptureTriggerTimePeriodUsec + RADAR_PRI_ADVANCE_USEC);
			res = ((res >> 16) - s_endpointRadarBase);
			s_dataCaptureActive = true;
			return res;
		}
		else
		{
			CLIENT_LOG_FATAL(LOG_TOKEN, "EndpointRadarBase in radar board is not compatible with it's counterpart in C Comms library.");
		}
	}
	else
	{
		return CLIENT_COMM_STATUS_ERROR_DATA_CALLBACK_NOT_REGISTERED;
	}
}

int32_t clientCommStopDataCapture(int32_t radarProtocolId)
{
	if (s_endpointRadarBase >= 0)
	{
		if (s_dataCaptureActive == true)
		{
			int32_t res = ep_radar_base_set_automatic_frame_trigger(radarProtocolId, s_endpointRadarBase, 0);
			res = ((res >> 16) - s_endpointRadarBase);

			s_dataCaptureActive == false;

			return res;
		}
		else
		{
			CLIENT_LOG_WARNING(LOG_TOKEN, "Could not Data capture was never started.");
		}
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "EndpointRadarBase in radar board is not compatible with it's counterpart in C Comms library.");
	}
}

int32_t clientCommGetDataFrame(int32_t radarProtocolId, uint32_t wait)
{
	int32_t res = -1;

	if (s_endpointRadarBase >= 0)
	{
		res = ep_radar_base_get_frame_data(radarProtocolId, s_endpointRadarBase, wait);
		res = (res >> 16) - s_endpointRadarBase;
		return res;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "EndpointRadarBase in radar board is not compatible with it's counterpart in C Comms library.");
	}
}

int32_t clientCommSetFrameFormat(int32_t radarProtocolId)
{
	int32_t res = -1;
	int32_t numChirps = *clientConfigGetNumChirpsPerFrame();
	int32_t numSamplesPerChirp = *clientConfigGetNumSamplesPerChirp();

	Frame_Format_t frameFormat;
	frameFormat.num_chirps_per_frame = numChirps;
	frameFormat.num_samples_per_chirp = numSamplesPerChirp;
	frameFormat.eSignalPart = EP_RADAR_BASE_SIGNAL_I_AND_Q;
	frameFormat.rx_mask = 3; // 2 rx antennas

	// First stop automatric trigger.
	if (ClientCommStopAutoTrigger(radarProtocolId) == 0)
	{
		res = ep_radar_base_set_frame_format(radarProtocolId, s_endpointRadarBase, &frameFormat);
		res = (res >> 16) - s_endpointRadarBase;
		return res;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Could not stop auto trigger mode of sensor radar board.");
	}
}

void clientCommDisconnectRadar()
{
	protocol_disconnect(s_radarHandle);
}
