#include <tasks/ROSMainTask.h>
/* includes of RTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

extern "C"
{
#include "application_tasks.h"
#include "ros.h"
#include "rcl.h"
#include "transport.h"
}
#include <string.h>

extern "C"
{
int serialize_int32(Message* msg, unsigned char *outbuffer);
int deserialize_int32(Message* msg, unsigned char *inbuffer);
int serialize_float32(Message* msg, unsigned char *outbuffer);
int deserialize_float32(Message* msg, unsigned char *inbuffer);
}

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
		ListItem* subscribers = getSubscribers(topic);

	    //Node* node = Nodes[0];
	    //ListItem* subscribers = node->subscribers;
		while(subscribers != NULL)
		{
			Subscriber* sub = (Subscriber*) subscribers->object;
			//if (sub != NULL)
			if (sub != NULL && !strcmp(topic, sub->topicName))
			{
				if (xSemaphoreTake(sub->dataAccess, 0)) // proceed only if the previous subscriber callback function has finished working on the message.
				{
					if (sub->msg->type == RCL_MSG_TYPE_UINT32)
						deserialize_int32(sub->msg, &msg[1+topicLen]);
					else if (sub->msg->type == RCL_MSG_TYPE_FLOAT)
						deserialize_float32(sub->msg, &msg[1+topicLen]);
					// in order not to block the RX task, tell the subscriber task (by signaling) to call its callback instead of calling the callback directly.
					xSemaphoreGive(sub->signal);
				}
			}
			subscribers = subscribers->next;
		}
	}
}

extern "C"
void InitNodesTask(void* params)
{
	unsigned int num_nodes = sizeof(nodes)/sizeof(node_decriptor);
	for (unsigned int i=0; i< num_nodes; i++)
	{
		xTaskCreate(nodes[i].function, (const signed char*)nodes[i].name, configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
	}
	vTaskDelete(NULL);
}

ROSMainTask::ROSMainTask(char const*name, unsigned portBASE_TYPE priority, unsigned portSHORT stackDepth)
: Task(name, priority, stackDepth, &task) {}


void ROSMainTask::task(void* p)
{
	tr_init();

	xTaskCreate(RXTask, (const signed char*)"RXTask", 1024, NULL, tskIDLE_PRIORITY + 3, NULL);

    vTaskDelay(5000); // TODO: Replace this sleep with semaphore signals to make sure network etc. has been setup successfully.
    xTaskCreate(InitNodesTask, (const signed char*)"InitNodesTask", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
	while(1)
	{
		vTaskDelay(60000);
	}
    vTaskDelete(NULL);
}
