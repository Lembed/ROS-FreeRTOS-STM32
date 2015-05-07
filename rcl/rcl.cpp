#include "rcl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"



void spinLoop(void (*callback)(void), unsigned int period)
{
	LOOP(period,
	// start while
	callback();
	// end while
	)
}
