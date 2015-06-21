#include "test.h"
#include "rcl.h"
extern "C" {
#include "ros.h"
}
#include "Queue.h"

//Queue* testQ = 0;

extern Queue* testQ;

uint32_t num = 0;
void testLoop()
{
	if (testQ)
	{
	 testQ->dequeue(&num);
	os_printf("item from queue: %d\n", num);
	}
}

void test(void* p)
{

	spinLoop(testLoop, 1000);
}
