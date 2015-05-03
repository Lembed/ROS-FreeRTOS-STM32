#include <tasks/ROSMainTask.h>
extern "C" void ros_main(void* p);
ROSMainTask::ROSMainTask(char const*name, unsigned portBASE_TYPE priority, unsigned portSHORT stackDepth)
: Task(name, priority, stackDepth, &task) {}



void ROSMainTask::task(void* p)
{
	ros_main(NULL);
}
