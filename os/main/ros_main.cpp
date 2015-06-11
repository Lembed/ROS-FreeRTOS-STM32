/* includes of RTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
extern "C"
{

#include "ros.h"
#include "rcl.h"
#include "transport.h"
#include <string.h>
}

#include "Node.h"
#include "Subscriber.h"
#include "Publisher.h"

#include <XMLRPCServer.h>
#include <application_tasks.h>

#include "wiring.h"

extern "C"
void InitNodesTask(void* params)
{
	unsigned int num_nodes = sizeof(nodes)/sizeof(node_decriptor);
	for (unsigned int i=0; i< num_nodes; i++)
	{
		xTaskCreate(nodes[i].function, (const signed char*)nodes[i].name, configMINIMAL_STACK_SIZE*4, NULL, tskIDLE_PRIORITY + 2, NULL);
	}



	vTaskDelete(NULL);
}

#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

using namespace std_msgs;

void mycallback(const Int32& msg)
{
	os_printf("My Callback: %d\n", msg.data);
}


void spinLoop(void (*callback)(void), unsigned int period)
{
	LOOP(period,
	// start while
	callback();
	// end while
	)
}

extern "C" void* os_malloc(unsigned int);
void* operator new(unsigned int sz) {
	return os_malloc(sz);
}
void *operator new[](unsigned int sz)
{
    // TODO: is this correct?
	return os_malloc(sz);
}
void operator delete(void* ptr)
{
	// Do nothing, since once allocated memory cannot be freed!
}

void operator delete[](void *ptr)
{
	// Do nothing, since once allocated memory cannot be freed!
}

void ros_main(void* p)
{
	enableTiming();
	tr_init();
	vTaskDelay(1000);
	XMLRPCServer::start();
    xTaskCreate(InitNodesTask, (const signed char*)"InitNodesTask", 128, NULL, tskIDLE_PRIORITY + 2, NULL);

    vTaskDelete(NULL);
}
