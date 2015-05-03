#include <Task.h>

TaskBase::TaskBase()
{
	handle = NULL;
}

Task::Task(char const*name, unsigned portBASE_TYPE priority, unsigned portSHORT stackDepth, pdTASK_CODE taskfun)
{
	xTaskCreate(taskfun, (signed char*)name, stackDepth, this, priority, &handle);
}

void Task::task(void* p)
{
	vTaskDelete(NULL);
}
