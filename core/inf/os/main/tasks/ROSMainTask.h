#ifndef TASKS_ROSMAINTASK_H_
#define TASKS_ROSMAINTASK_H_

#include <Task.h>

class ROSMainTask: public Task {
public:
	ROSMainTask(char const*name, unsigned portBASE_TYPE priority, unsigned portSHORT stackDepth);
	static void task(void* p);
};

#endif /* TASKS_ROSMAINTASK_H_ */
