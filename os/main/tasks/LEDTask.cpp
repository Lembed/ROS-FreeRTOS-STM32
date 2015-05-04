#include <tasks/LEDTask.h>

#include "stm32f4_discovery.h"

void LEDTask::task(void* p)
{
	STM_EVAL_LEDInit(LED4);
	for( ;; ) {
			// toggle LED4 each 250ms
	        STM_EVAL_LEDToggle(LED4);
	        vTaskDelay(250);
	      }
}

LEDTask::LEDTask(char const*name, unsigned portBASE_TYPE priority, unsigned portSHORT stackDepth)
: Task(name, priority, stackDepth, &task)
{

}
