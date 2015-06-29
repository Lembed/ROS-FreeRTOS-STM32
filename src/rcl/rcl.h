//#define QUEUE_MSG_SIZE 128
#include <stdint.h>
#include "stm32f4xx.h"

void spinLoop(void (*callback)(void), unsigned int period);
/**
 * LOOP macro: Used to include a periodic code.
 * 	The advantage of this macro is that the user will not need to explicitly call vTaskDelayUntil function, so that kernel functions remain invisible.
 * 	First argument: Period.
 * 	Second argument: periodic code.
 */
#define LOOP(period, code) portTickType xLastWakeTime=xTaskGetTickCount(); \
					while(1) \
					{ \
					code \
					vTaskDelayUntil(&xLastWakeTime, period); \
				   }


void LOG(const char* fmt, ...);
