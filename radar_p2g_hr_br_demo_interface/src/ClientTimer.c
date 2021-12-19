#include <stdbool.h>
#include <ClientLog.h>
#include <ClientTimer.h>

#ifdef LOG_TOKEN
#undef LOG_TOKEN
#endif 

#define LOG_TOKEN  Module_Timer

typedef struct ClientTimerInstance_s
{
	int32_t timerId;
	clock_t startTime;

	struct clientTimer_s* _next;
} ClientTimerInstance_t;


static ClientTimerInstance_t* headInstance = NULL;
static int32_t s_timerInstanceCount;

static ClientTimerInstance_t* fillANode(int32_t timerId)
{
	ClientTimerInstance_t* p_instance = (ClientTimerInstance_t*)malloc(sizeof(ClientTimerInstance_t));
	p_instance->timerId = timerId;
	p_instance->startTime = clock();
	p_instance->_next = NULL;
	//CLIENT_LOG_PRINT(LOG_TOKEN, "Starting timer ID: %d to time %d", timerId, p_instance->startTime);
	return p_instance;
}

static ClientTimerInstance_t* findNode(int32_t timerId)
{
	ClientTimerInstance_t* p_instance = headInstance;
	if (headInstance)
	{
		while (p_instance != NULL)
		{
			if (p_instance->timerId == timerId)
			{
				return p_instance;
			}
			p_instance = p_instance->_next;
		}
	}
	return NULL;
}

void clientTimerInit()
{
	// Free allocated memory 
	// because of previous state
	if (headInstance)
	{
		clientTimerClose();
	}
	headInstance = NULL;
	s_timerInstanceCount = 1;
}

int32_t clientTimerStart() 
{
	ClientTimerInstance_t* p_instance = headInstance;
	int32_t timerId = s_timerInstanceCount;

	if (p_instance ==  NULL)
	{ 
		headInstance = fillANode(s_timerInstanceCount);
	}
	else
	{
		// Append at the end
		while (p_instance->_next != NULL)
		{
			p_instance = p_instance->_next;
		}
		p_instance->_next = fillANode(s_timerInstanceCount);
	}

	s_timerInstanceCount++;

	return timerId;
}

void clientTimerReset(int32_t timerId)
{
	ClientTimerInstance_t* p_instance = findNode(timerId);

	if (p_instance)
	{
		p_instance->startTime = clock();
		//CLIENT_LOG_PRINT(LOG_TOKEN, "Reseting timer ID: %d to time %d", timerId, p_instance->startTime);
	}

}

double clientTimerInterval(int32_t timerId)
{
	ClientTimerInstance_t* p_instance = headInstance;
	while (p_instance != NULL)
	{
		if (p_instance->timerId == timerId)
		{
			// find elapsed time
			double timerIntervalSec;
			clock_t currTime = clock();
			timerIntervalSec = (double)((double)currTime - (double)p_instance->startTime) / CLOCKS_PER_SEC;
			//CLIENT_LOG_PRINT(LOG_TOKEN, "Finding interval timer ID: %d to time %d", timerId, currTime);

			// save current time for next interval end
			p_instance->startTime = clock();

			// return elapsed time period in seconds
			return timerIntervalSec;
		}
		p_instance = p_instance->_next;
	}
	CLIENT_LOG_WARNING(LOG_TOKEN, "Given timer ID is invalid");
	return (double)0.0;
}

void clientTimerClose()
{
	if (headInstance != NULL) 
	{
		ClientTimerInstance_t* p_instance = headInstance;
		ClientTimerInstance_t* p_next;
		while (p_instance != NULL)
		{
			p_next = p_instance->_next;
			free(p_instance);
			p_instance = p_next;
		}
	}
	headInstance = NULL;
}
