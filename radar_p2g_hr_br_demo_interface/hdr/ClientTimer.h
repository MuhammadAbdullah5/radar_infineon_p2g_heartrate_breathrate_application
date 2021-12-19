#ifndef __CLIENT_TIMER_H__
#define __CLIENT_TIMER_H__

#include <stdint.h>
#include <time.h>

void clientTimerInit();
void clientTimerClose();

// Start a timer at current time and save
// ID returned by it. Use this ID to find
// out elapsed time in seconds later on using
// function clientTimerInterval()
int32_t clientTimerStart();

void clientTimerReset(int32_t timerId);

// Get time elapsed in seconds since last call to
// clientTimerStart() or clientTimerInterval()
// against a timer identified by timerId.
double clientTimerInterval(int32_t timerId);

#endif //__CLIENT_TIMER_H__
