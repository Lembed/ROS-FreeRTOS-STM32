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
#include <string.h>
}

#include "Node.h"
#include "Subscriber.h"
#include "Publisher.h"


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
			else
			{
				vTaskDelay(4);
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
		xTaskCreate(nodes[i].function, (const signed char*)nodes[i].name, configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
	}
	vTaskDelete(NULL);
}

#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

using namespace std_msgs;
unsigned char buffer[30];

void mycallback(const Int32& msg)
{
	os_printf("My Callback: %d\n", msg.data);
}


char taskName[32];
void ros_main(void* p)
{
	tr_init();
	xTaskCreate(RXTask, (const signed char*)"RXTask", 1024, NULL, tskIDLE_PRIORITY + 3, NULL);

    vTaskDelay(5000); // TODO: Replace this sleep with semaphore signals to make sure network etc. has been setup successfully.
	Float32 msg, msg2;
	ros::Node* n = new ros::Node("node");
	ros::Publisher* pub = new ros::Publisher;
	pub->advertise<Float32>(n, "sqrt");
	ros::Subscriber<Int32>* sub = new ros::Subscriber<Int32>(n, "sub", mycallback);

	msg.data = 4.6f;
	msg2.data = 4.5f;

    /*xTaskCreate(InitNodesTask, (const signed char*)"InitNodesTask", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
	while(1)
	{
		msg.serialize(buffer);
		msg2.deserialize(buffer);
		pub->publish(msg2);

		os_printf("Num: %s %d\n", buffer, msg2.data);
		vTaskDelay(1000);
	}*/
    vTaskDelete(NULL);
}
