/*! \file rcl.h
    \brief ROS Client Library.
*/

/* includes of RTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "stm32f4xx.h"


#define QUEUE_MSG_SIZE 128
#define NODE_COUNT 3

/**
 * LOOP macro: Used to include a periodic code.
 * 	The advantage of this macro is that the user will not need to explicitly call vTaskDelayUntil function, so that kernel functions remain invisible.
 * 	First argument: Period.
 * 	Second argument: periodic code.
 */
#define LOOP(period, code) portTickType xLastWakeTime=xTaskGetTickCount(); \
					while(1) \
					{ \
					code \
					vTaskDelayUntil(&xLastWakeTime, period); \
				   }
/**
 *	Struct for a list item, which can also be used as the head of a list.
 *	This struct suffices for a list implementation.
 */
typedef struct ListItem {
    void* object; /**< void* object. Data of an arbitrary type can be stored here. NULL if list is empty. */
    struct ListItem* next; /**< Pointer to the next ListItem in the list. NULL if it is the last element. */
} ListItem;


/**
 * Data structure for a ROS node.
 */
typedef struct Node
{
	char name[32];
	ListItem* subscribers;
	ListItem* publishers;
} Node;

typedef enum {RCL_MSG_TYPE_FLOAT, RCL_MSG_TYPE_UINT32} MessageType;

/**
 * Data structure for a ROS message.
 */
typedef struct Message
{
	void* data;
	unsigned short length;
	MessageType type;
} Message;

/**
 * Data structure for a ROS subscriber.
 */
typedef struct Subscriber
{
	Message* msg;
	char topicName[32];
	Node* node;
	xSemaphoreHandle signal, dataAccess;
	void (*callback)(Message* msg);
	MessageType msgType; // necessary?
} Subscriber;

/**
 * Data structure for a ROS publisher.
 */
typedef struct Publisher
{
	Node* node;
	char topicName[32];
	Message* msg;
	MessageType msgType; // necessary?
} Publisher;


Node* createNode(const char* name);
Message* createMessage(MessageType type);
Publisher* createPublisher(Node* node, const char* topic_name, MessageType msg_type);
void publish(Publisher* publisher);
Subscriber* createSubscriber(Node* node, const char* topic_name, MessageType msg_type, void (*callback)(const Message* msg));
void spin(Node* node);
void spinNode(Node* node, void (*callback)(void), unsigned int period);
ListItem* getSubscribers(const char* topic);
