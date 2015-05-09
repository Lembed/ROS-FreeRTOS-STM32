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

#include "application_tasks.h"

extern "C"
void RXTask(void* params)
{
	for (;;)
	{
		unsigned char msg[QUEUE_MSG_SIZE]; // Change this definition name later, there must be no UDP in it.

		tr_take_msg(msg, NULL); // block until msg sequence arrives.



		int topicLen = (int)msg[0];
	    char topic[topicLen+1];
	    memcpy(topic, &msg[1], topicLen);
	    topic[topicLen] = '\0';

		//void *new_msg;
		//tr_extract_msg(topic, msg, new_msg);
	    /*os_printf("topic\n");
	    //os_printf("topic:%s\n", ((ros::Subscriber_*) ros::Subscriber_::list[0])->topic);
	    vTaskDelay(4);
	    continue;*/

		// TODO: Why does RXTask crash/block the OS if remote PC is already publishing at the time STM32 is started?
	    for (unsigned int i=0; i<MAX_SUBSCRIBERS; i++)
		{
			ros::Subscriber_* sub = (ros::Subscriber_*) ros::Subscriber_::list[i];
			//if (sub != NULL)
			if (sub != NULL && !strcmp(topic, sub->topic))
			{
				if (xSemaphoreTake(sub->dataAccess, 0)) // proceed only if the previous subscriber callback function has finished working on the message.
				{
					//os_printf("Topic: %s\n", sub->topic);
					sub->deserialize(&msg[1+topicLen]);

					// in order not to block the RX task, tell the subscriber task (by signaling) to call its callback instead of calling the callback directly.
					xSemaphoreGive(sub->signal);
				}
				//os_printf("Topic: %s\n", sub->topic);
			}
		}
	}
}



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

extern void xmlrpc_task(void* p);

void ros_main(void* p)
{
	tr_init();
	vTaskDelay(2000);
	xTaskCreate(xmlrpc_task, (const signed char*)"RXTask2", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
	/*vTaskDelay(2000);
	tr_init();
	xTaskCreate(RXTask, (const signed char*)"RXTask", 1024, NULL, tskIDLE_PRIORITY + 3, NULL);

    vTaskDelay(5000); // TODO: Replace this sleep with semaphore signals to make sure network etc. has been setup successfully.

    //xTaskCreate(InitNodesTask, (const signed char*)"InitNodesTask", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
*/
    vTaskDelete(NULL);
}
