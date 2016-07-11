#ifndef RMW_TOPICWRITER_H_
#define RMW_TOPICWRITER_H_
#include <stdlib.h>
#include "FreeRTOS.h"
#include "semphr.h"
#define QUEUE_LEN 5
#define QUEUE_MSG_SIZE sizeof(UDPMessage)
#define RXTIMEOUT 100
#define MAX_TOPIC_LEN 48
#include "tcpip.h"
#include <string.h>

typedef struct UDPMessage {
	char topic[MAX_TOPIC_LEN];
	char data[250]; // TODO: Implement new functionality to set this while creating TopicWriter so that there is no redundant space in queue.
} UDPMessage;

#define MAX_UDP_CONNECTIONS 20

extern "C" void os_printf(const char* fmt, ...);


class UDPConnection
{
private:
	static uint32_t ID;
	uint16_t port;
public:
	UDPConnection(uint16_t port)
	{
		ID++;
		this->port = port;
	}
	uint32_t getID() const { return ID; }
	uint16_t getPort() const { return port; }
	void setPort(uint16_t port) { this->port = port; }
	bool isValid() const { return port != 0; }
};
#include "rcl.h"
#include "msg.h"



class UDPHandler
{
private:
	xQueueHandle qHandle;
	static UDPHandler* _instance;
	UDPHandler()
	{
		qHandle = xQueueCreate(QUEUE_LEN, QUEUE_MSG_SIZE);
	}
public:
	void enqueueMessage(const UDPMessage *msg)
	{
		// Initialize memory (in stack) for message.
		unsigned char data[QUEUE_MSG_SIZE];
		// Copy message into the previously initialized memory.
		if (msg != NULL) {
			memcpy(data, msg, QUEUE_MSG_SIZE);
			// Try to send message if queue is non-full.
			// TODO: Check if we are still "connected" to the end point. (i.e. the node at the remote end is still running)
			if (xQueueSend(qHandle, &data, 0)) {

			}
			/*	os_printf("Enqueueing data!\n");
			else
			    os_printf("Queue is full!\n");*/
		} else
			os_printf("UDPHandler::enqueueMessage msg is NULL!\n");
	}
	void dequeueMessage(UDPMessage* msg)
	{
		// Initialize memory (in stack) for message.
		unsigned char data[QUEUE_MSG_SIZE];

		// Try to receive message, put the task to sleep for at most RXTIMEOUT ticks if queue is empty.
		for (;;) {
			if (xQueueReceive(qHandle, data, RXTIMEOUT)) {
				if (msg != NULL)
					memcpy(msg, data, QUEUE_MSG_SIZE);
				else
					os_printf("UDPHandler::dequeueMessage msg is NULL!\n");
				break;
			}
		}
	}
	static UDPHandler *instance()
	{
		if (!_instance)
			_instance = new UDPHandler;
		return _instance;
	}
};


#define UDP_LOCAL_PORT 46552

class TopicWriter
{
	char topic[MAX_TOPIC_LEN];
	uint16_t lastConnectionsIndex;
	UDPConnection* connections[MAX_UDP_CONNECTIONS];
	xQueueHandle qHandle;
	UDPConnection* createConnection(uint16_t port);
public:
	TopicWriter(const char* callerID, const char* topic, const char* msgType);
	void serializeMsg(const ros::Msg& msg, unsigned char* outbuffer);
	void publishMsg(const ros::Msg& msg);
	UDPConnection* getConnection(uint16_t port);
	UDPConnection* const* getConnections();
	const char* getTopic();
	static void connectSubscribers(const void* obj, const char* data);

	void deleteConnection(uint16_t port); // TODO:Implement this!
};

#endif /* RMW_TOPICWRITER_H_ */
