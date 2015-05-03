#ifndef TASKS_LEDTASK_H_
#define TASKS_LEDTASK_H_

#include <Task.h>

class LEDTask: public Task {
public:
	LEDTask(char const*name, unsigned portBASE_TYPE priority, unsigned portSHORT stackDepth);
	static void task(void* p);
};

#endif /* TASKS_LEDTASK_H_ */
