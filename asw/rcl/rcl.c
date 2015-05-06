#include "rcl.h"
#include "ros.h"
typedef struct node_descriptor {
	char name[32];
	void (*function)(void* params);

} node_decriptor;
extern node_decriptor nodes;

Node* Nodes[sizeof(nodes) / sizeof(node_decriptor)];

int lastNodeIndex = -1;
unsigned int subscriberCount = 0;
unsigned int publisherCount = 0;


Node* createNode(const char* name)
{
	Node* node = (Node*) os_malloc(sizeof(Node));
	Nodes[++lastNodeIndex] = node;
	strcpy(node->name, name);
	node->publishers = (ListItem*)os_malloc(sizeof(ListItem));
	node->publishers->object = NULL;
	node->publishers->next = NULL;
	node->subscribers = (ListItem*)os_malloc(sizeof(ListItem));
	node->subscribers->object = NULL;
	node->subscribers->next = NULL;
	return node;
}

Message* createMessage(MessageType type)
{
	Message* message = (Message*) os_malloc(sizeof(Message));
	message->type = type;
	switch(type)
	{
		case RCL_MSG_TYPE_FLOAT:
			;
			message->data = os_malloc(sizeof(float));
			message->length = sizeof(float);
		break;
		case RCL_MSG_TYPE_UINT32:
			;
			message->data = os_malloc(sizeof(uint32_t));
			message->length = sizeof(uint32_t);
		break;
		default:
			message->data = NULL;
			message->length = 0;
		break;
	}

	return message;
}

Publisher* createPublisher(Node* node, const char* topic_name, MessageType msg_type)
{
	Publisher* publisher = (Publisher*) os_malloc(sizeof(Publisher));

	addItemToList(node->publishers, publisher);

	++publisherCount;
	publisher->node = node;
	strcpy(publisher->topicName, topic_name);
	publisher->msgType = msg_type;
	publisher->msg = createMessage(msg_type);

	// Used for letting ros_server know which topic we are publishing to.
	char registerTopic[] = "advertise";
	unsigned int offset = 0;
	unsigned char data[QUEUE_MSG_SIZE];
	data[0] = (char)strlen(registerTopic);
	memcpy(&data[1], registerTopic, strlen(registerTopic));
	offset = 1 + strlen(registerTopic);
	data[offset++] = (uint8_t) msg_type;
	memcpy(&data[offset], publisher->topicName, strlen(publisher->topicName));
	data[offset+strlen(publisher->topicName)] = '\0';
	tr_publish(data, NULL);

	return publisher;
}

#include "msg_int32.h"
#include "msg_float32.h"

void publish(Publisher* publisher)
{
	Message* msg = publisher->msg;

	unsigned int offset = 0;
	unsigned char data[QUEUE_MSG_SIZE];
	data[0] = (char)strlen(publisher->topicName);
	memcpy(&data[1], publisher->topicName, strlen(publisher->topicName));
	offset = 1 + strlen(publisher->topicName);

	switch(publisher->msgType)
	{
		case RCL_MSG_TYPE_FLOAT:
			;
			offset += serialize_float32(msg, &data[offset]);
		break;
		case RCL_MSG_TYPE_UINT32:
			;
			offset += serialize_int32(msg, &data[offset]);
		break;
		default:
			os_printf("Message type not supported!");
		return;
	}

	tr_publish(data, (void*)publisher->topicName);

}

Subscriber* createSubscriber(Node* node, const char* topic_name, MessageType msg_type, void (*callback)(const Message* msg))
{
	Subscriber* subscriber = (Subscriber*) os_malloc(sizeof(Subscriber));

	addItemToList(node->subscribers, subscriber);
	++subscriberCount;
	subscriber->node = node;
	strcpy(subscriber->topicName, topic_name);

	subscriber->msg = createMessage(msg_type);

	subscriber->callback = callback;

	vSemaphoreCreateBinary(subscriber->signal);
	vSemaphoreCreateBinary(subscriber->dataAccess);
	xSemaphoreTake(subscriber->signal, 0); // do this operation to initialize the semaphore with 0 resources

	return subscriber;
}

void publisherTimerTask(void* params)
{
	Publisher* publisher = (Publisher*) params;
	if (publisher != NULL)
	{
		portTickType xLastWakeTime;
		const portTickType xFrequency = 10; // = publisher->timerPeriod;

		// Initialize the xLastWakeTime variable with the current time.
		xLastWakeTime = xTaskGetTickCount();
		for( ;; )
		{
			// publisher->timer();
			// Wait for the next cycle.
		    vTaskDelayUntil(&xLastWakeTime, xFrequency);
		}
	}
	else
	{
		os_printf("Publisher is null! Deleting timer task.");
		vTaskDelete(NULL); // kill yourself
	}
}

void subscriberCallbackTask(void* params)
{
	Subscriber* subscriber = (Subscriber*) params;

	if (subscriber != NULL)
	{
		while(1)
		{
			if (xSemaphoreTake(subscriber->signal, portMAX_DELAY))
			{
				Message* msg = subscriber->msg; // get message data from shared memory
				if (msg->data != NULL && msg->length > 0)
					subscriber->callback(msg);
				else
					os_printf("Invalid callback message!");

				xSemaphoreGive(subscriber->dataAccess); // the RX task is allowed to proceed accessing shared data now.
			}
		}
	}
	else
	{
		os_printf("Subscriber is null! Deleting callback task.");
		vTaskDelete(NULL); // kill yourself
	}
}


void spin(Node* node)
{
    char taskName[32];

    ListItem* it = node->subscribers;
    while (it != NULL)
    {
    	Subscriber* sub = (Subscriber*)it->object;
		sprintf(taskName, "subscriber_%s_%s", node->name, sub->topicName);
		xTaskCreate(subscriberCallbackTask, taskName, 1024, (void*) sub, tskIDLE_PRIORITY + 2, NULL);

        it = it->next;
    }
    vTaskDelete(NULL);
}

void spinLoop(Node* node, void (*callback)(void), unsigned int period)
{
    char taskName[32];

    ListItem* it = node->subscribers;
    while (it != NULL)
    {
    	Subscriber* sub = (Subscriber*)it->object;
		sprintf(taskName, "subscriber_%s_%s", node->name, sub->topicName);
		xTaskCreate(subscriberCallbackTask, taskName, 1024, (void*) sub, tskIDLE_PRIORITY + 2, NULL);

        it = it->next;
    }
	LOOP(period,
	// start while
	callback();
	// end while
	)
}

ListItem emptyListHead;

ListItem* getSubscribers(const char* topic)
{
	ListItem* subscribers = &emptyListHead;
	subscribers->object = NULL;
	subscribers->next = NULL;
	for (unsigned int i = 0; i<NODE_COUNT; i++)
	{
	    Node* node = Nodes[i];
	    if (node != NULL)
	    {
	    	ListItem* it = node->subscribers;
			while (it != NULL)
			{
				Subscriber* sub = (Subscriber*)it->object;

				if (!strcmp(topic, sub->topicName))
				{
					//os_printf("%s == %s", sub->topicName, topic);
					addItemToList(subscribers, (void*) sub);
				}
				it = it->next;
			}
	    }
	}

	return subscribers;
}
