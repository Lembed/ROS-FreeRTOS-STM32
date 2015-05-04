#ifndef TaskCPP_H
#define TaskCPP_H

#include "FreeRTOS.h"
#include "task.h"

class TaskBase {
public:
	TaskBase();
	xTaskHandle handle;
};

class Task : public TaskBase {
public:
	Task(char const*name, unsigned portBASE_TYPE priority, unsigned portSHORT stackDepth, pdTASK_CODE taskfun);
	static void task(void* p);
	~Task() {}
};
#endif /* TaskCPP_H */
