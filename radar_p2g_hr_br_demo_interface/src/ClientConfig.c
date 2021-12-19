#include <ClientLog.h>
#include <ClientConfig.h>

#define GET_BIT(var, pos)    (((1 << pos) & var) >> pos)
#define SET_BIT(var, pos)    var=((1 << pos) | var)
#define CLEAR_BIT(var, pos)  var=(~(1 << pos) & var)

#define GENERATE_STRUCT(TYPE, VAR) TYPE VAR;
#define GENERATE_ENUM(TYPE, ENUM) bit_position_##ENUM,
#define GENERATE_STRING(TYPE, STRING) #STRING,

typedef struct
{
	vARIABLE_LIST(GENERATE_STRUCT)
}FmcwRadarParams_t;

typedef enum variable_bit_position_enum {
	vARIABLE_LIST(GENERATE_ENUM)
	bit_position_num_variables
};

static const char* variable_bit_position_desc[] = {
	vARIABLE_LIST(GENERATE_STRING)
};

static FmcwRadarParams_t s_radarParams;

static int32_t s_variableSetBitmask = 0;

static void clientConfigClearSetBitmask()
{
	// Clear bitmask for variable setting
	for (int32_t variableBitPosition = 0; variableBitPosition < (int32_t)bit_position_num_variables; variableBitPosition++)
	{
		CLEAR_BIT(s_variableSetBitmask, variableBitPosition);
	}
}

void clientConfigClear()
{
	clientConfigClearSetBitmask();
}

// Getter functions
const float* clientConfigGetCarrierFreqHz()
{
	if (GET_BIT(s_variableSetBitmask, bit_position_carrierFreqHz))
	{
		return &s_radarParams.carrierFreqHz;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "%s parameter is not configured in ClientConfig. Something went wrong.", 
			variable_bit_position_desc[bit_position_carrierFreqHz]);
	}
}
const float* clientConfigGetRadarBandwidthHz()
{
	if (GET_BIT(s_variableSetBitmask, bit_position_radarBandwidthHz))
	{
		return &s_radarParams.radarBandwidthHz;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "%s parameter is not configured in ClientConfig. Something went wrong.", 
			variable_bit_position_desc[bit_position_radarBandwidthHz]);
	}
}
const float* clientConfigGetPulseRepIntervalSec()
{
	if (GET_BIT(s_variableSetBitmask, bit_position_pulseRepIntervalSec))
	{
		return &s_radarParams.pulseRepIntervalSec;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "%s parameter is not configured in ClientConfig. Something went wrong.", 
			variable_bit_position_desc[bit_position_pulseRepIntervalSec]);
	}
}
const int32_t* clientConfigGetNumRxAntennas()
{
	if (GET_BIT(s_variableSetBitmask, bit_position_numRxAntennas))
	{
		return &s_radarParams.numRxAntennas;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "%s parameter is not configured in ClientConfig. Something went wrong.", 
			variable_bit_position_desc[bit_position_numRxAntennas]);
	}
}
const int32_t* clientConfigGetNumChirpsTotal()
{
	if (GET_BIT(s_variableSetBitmask, bit_position_numChirpsTotal))
	{
		return &s_radarParams.numChirpsTotal;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "%s parameter is not configured in ClientConfig. Something went wrong.", 
			variable_bit_position_desc[bit_position_numChirpsTotal]);
	}
}
const int32_t* clientConfigGetNumChirpsPerFrame()
{
	if (GET_BIT(s_variableSetBitmask, bit_position_numChirpsPerFrame))
	{
		return &s_radarParams.numChirpsPerFrame;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "%s parameter is not configured in ClientConfig. Something went wrong.", 
			variable_bit_position_desc[bit_position_numChirpsPerFrame]);
	}
}
const int32_t* clientConfigGetNumSamplesPerChirp()
{
	if (GET_BIT(s_variableSetBitmask, bit_position_numSamplesPerChirp))
	{
		return &s_radarParams.numSamplesPerChirp;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "%s parameter is not configured in ClientConfig. Something went wrong.", 
			variable_bit_position_desc[bit_position_numSamplesPerChirp]);
	}
}
const int32_t* clientConfigGetRangeFftLength()
{
	if (GET_BIT(s_variableSetBitmask, bit_position_rangeFftLength))
	{
		return &s_radarParams.rangeFftLength;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "%s parameter is not configured in ClientConfig. Something went wrong.",
			variable_bit_position_desc[bit_position_rangeFftLength]);
	}
}
const int32_t* clientConfigGetDopplerFftLength()
{
	if (GET_BIT(s_variableSetBitmask, bit_position_dopplerFftLength))
	{
		return &s_radarParams.dopplerFftLength;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "%s parameter is not configured in ClientConfig. Something went wrong.", 
			variable_bit_position_desc[bit_position_dopplerFftLength]);
	}
}
const float* clientConfigGetPercentFftThreshold()
{
	if (GET_BIT(s_variableSetBitmask, bit_position_percentFftThreshold))
	{
		return &s_radarParams.percentFftThreshold;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "%s parameter is not configured in ClientConfig. Something went wrong.",
			variable_bit_position_desc[bit_position_percentFftThreshold]);
	}
}
const float* clientConfigGetMtiFilterAlpha()
{
	if (GET_BIT(s_variableSetBitmask, bit_position_mtiFilterAlpha))
	{
		return &s_radarParams.mtiFilterAlpha;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "%s parameter is not configured in ClientConfig. Something went wrong.",
			variable_bit_position_desc[bit_position_mtiFilterAlpha]);
	}
}
const TimeDomainWindowType_t* clientConfigGetTimeDomainWindow()
{
	if (GET_BIT(s_variableSetBitmask, bit_position_timeDomainWindow))
	{
		return &s_radarParams.timeDomainWindow;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "%s parameter is not configured in ClientConfig. Something went wrong.",
			variable_bit_position_desc[bit_position_timeDomainWindow]);
	}
}
const SaveDataType_t* clientConfigGetDataSaveType()
{
	if (GET_BIT(s_variableSetBitmask, bit_position_dataSaveType))
	{
		return &s_radarParams.dataSaveType;
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "%s parameter is not configured in ClientConfig. Something went wrong.",
			variable_bit_position_desc[bit_position_dataSaveType]);
	}
}

// Setter functions
void clientConfigSetCarrierFreqHz(float carrierFreqHz)
{
	s_radarParams.carrierFreqHz = carrierFreqHz;
	SET_BIT(s_variableSetBitmask, bit_position_carrierFreqHz);
}
void clientConfigSetRadarBandwidthHz(float radarBandwidth)
{
	s_radarParams.radarBandwidthHz = radarBandwidth;
	SET_BIT(s_variableSetBitmask, bit_position_radarBandwidthHz);
}
void clientConfigSetPulseRepIntervalSec(float pulseRepIntervalSec)
{
	s_radarParams.pulseRepIntervalSec = pulseRepIntervalSec;
	SET_BIT(s_variableSetBitmask, bit_position_pulseRepIntervalSec);
}
void clientConfigSetNumRxAntennas(int32_t numRxAntennas)
{
	s_radarParams.numRxAntennas = numRxAntennas;
	SET_BIT(s_variableSetBitmask, bit_position_numRxAntennas);
}
void clientConfigSetNumChirpsTotal(int32_t numChirpsTotal)
{
	s_radarParams.numChirpsTotal = numChirpsTotal;
	SET_BIT(s_variableSetBitmask, bit_position_numChirpsTotal);
}
void clientConfigSetNumChirpsPerFrame(int32_t numChirpsPerFrame)
{
	s_radarParams.numChirpsPerFrame = numChirpsPerFrame;
	SET_BIT(s_variableSetBitmask, bit_position_numChirpsPerFrame);
}
void clientConfigSetNumSamplesPerChirp(int32_t numSamplesPerChirp)
{
	s_radarParams.numSamplesPerChirp = numSamplesPerChirp;
	SET_BIT(s_variableSetBitmask, bit_position_numSamplesPerChirp);
}
void clientConfigSetRangeFftLength(int32_t rangeFftLen)
{
	s_radarParams.rangeFftLength = rangeFftLen;
	SET_BIT(s_variableSetBitmask, bit_position_rangeFftLength);
}

void clientConfigSetDopplerFftLength(int32_t dopplerFftLen)
{
	s_radarParams.dopplerFftLength = dopplerFftLen;
	SET_BIT(s_variableSetBitmask, bit_position_dopplerFftLength);
}
void clientConfigSetPercentFftThreshold(float fftThresholdPercent)
{
	s_radarParams.percentFftThreshold = fftThresholdPercent;
	SET_BIT(s_variableSetBitmask, bit_position_percentFftThreshold);
}
void clientConfigSetMtiFilterAlpha(float mtiFilterAlpha)
{
	s_radarParams.mtiFilterAlpha = mtiFilterAlpha;
	SET_BIT(s_variableSetBitmask, bit_position_mtiFilterAlpha);
}
void clientConfigSetTimeDomainWindow(TimeDomainWindowType_t windowType)
{
	s_radarParams.timeDomainWindow = windowType;
	SET_BIT(s_variableSetBitmask, bit_position_timeDomainWindow);
}
void clientConfigSetDataSaveType(SaveDataType_t dataSaveType)
{
	s_radarParams.dataSaveType = dataSaveType;
	SET_BIT(s_variableSetBitmask, bit_position_dataSaveType);
}
