#include "TopicReader.h"
#include "XMLRPCServer.h"

#include <string.h>
#include "XMLRequest.h"
#include "device_config.h"

extern "C"
{
#include "ros.h"
}



TopicReader::TopicReader(const char* callerID, const char* topic, const char* md5sum, const char* msgType)
{
	strcpy(this->topic, topic);
	strcpy(this->callerID, callerID);
	strcpy(this->md5sum, md5sum);
	strcpy(this->msgType, msgType);
	qHandle = xQueueCreate(3, RX_QUEUE_MSG_SIZE);
	connectionID = 0;
	XMLRequest* req = new RegisterRequest("registerSubscriber", ROS_MASTER_IP, callerID, topic, msgType);
	XMLRPCServer::sendRequest(req->getData(), SERVER_PORT_NUM, connectPublishers, this);
	// TODO: make a unique task name
	xTaskCreate(task, (const signed char*)topic, 250, (void*)this, tskIDLE_PRIORITY + 2, NULL);
}

void TopicReader::addCallback(void(*callback)(void* data, void* obj), void* obj)
{
	static int lastIndex = 0;
	if (lastIndex < MAX_CALLBACKS) {
		callbacks[lastIndex] = callback;
		objects[lastIndex] = obj;
		lastIndex++;
	}
}

void TopicReader::task(void* arg)
{
	TopicReader* self = (TopicReader*) arg;
	unsigned char data[RX_QUEUE_MSG_SIZE];
	for (;;) {
		// Try to receive message, put the task to sleep for at most RXTIMEOUT ticks if queue is empty.
		if (xQueueReceive(self->qHandle, data, RXTIMEOUT)) {
			for (int i = 0; i < MAX_CALLBACKS; i++) {
				if (self->callbacks[i] != NULL)
					self->callbacks[i]((void*)&data[4], self->objects[i]);
			}
		}
	}
}

const char* TopicReader::getTopic()
{
	return topic;
}

uint32_t TopicReader::getConnectionID()
{
	return connectionID;
}

void TopicReader::enqueueMessage(const char* msg)
{
	// Initialize memory (in stack) for message.
	unsigned char data[RX_QUEUE_MSG_SIZE];
	// Copy message into the previously initialized memory.
	if (msg != NULL) {
		memcpy(data, msg, RX_QUEUE_MSG_SIZE);
		// Try to send message if queue is non-full.
		// TODO: Check if we are still "connected" to the end point. (i.e. the node at the remote end is still running)
		if (xQueueSend(qHandle, &data, 0)) {

		}
		/*	os_printf("Enqueueing data!\n");*/
		else
			os_printf("Queue is full!\n");
	} else {
		os_printf("TopicReader::enqueueMessage msg is NULL\n");
	}
}
void TopicReader::dequeueMessage(char* msg)
{
	// Initialize memory (in stack) for message.
	unsigned char data[RX_QUEUE_MSG_SIZE];

	// Try to receive message, put the task to sleep for at most RXTIMEOUT ticks if queue is empty.
	for (;;) {
		if (xQueueReceive(qHandle, data, RXTIMEOUT)) {
			if (msg != NULL)
				memcpy(msg, data, RX_QUEUE_MSG_SIZE);
			else
				os_printf("TopicReader::dequeueMessage msg is NULL!\n");
			break;
		}
	}
}

void TopicReader::onResponse(const void* obj, const char* data)
{
	char* pos0 = strstr((char*)data, "UDPROS");
	char* pos01 = strstr((char*)pos0, "<i4>");
	char* pos = strstr((char*)pos01 + 4, "<i4>");
	char* pos2 = strstr((char*)pos, "</i4>");
	if (pos2 > pos && pos != NULL) {
		int offset = strlen("<i4>");
		char connID[pos2 - pos - offset + 1];
		strncpy (connID, pos + offset, pos2 - pos - offset);
		TopicReader* self = (TopicReader*) obj;
		self->connectionID = atoi(connID);
		os_printf("Connection ID: %d, topic:%s\n", self->connectionID, self->topic);
	}
}

void TopicReader::requestTopic(const char* ip, uint16_t serverPort)
{
	XMLRequest* req = new TopicRequest("requestTopic", ROS_MASTER_IP, callerID, topic, md5sum, msgType);
	XMLRPCServer::sendRequest(req->getData(), serverPort, onResponse, this);
}

void TopicReader::connectPublishers(const void* obj, const char* data)
{
	char text[100];
	char* pos = strstr((char*)data, "Subscribed to");
	if (pos != 0) {
		while (1) {
			char* pos2 = strstr((char*)pos, "<value><string>");
			char* pos3 = strstr((char*)pos2, "</string></value>");
			if (pos2 == NULL || pos3 == NULL)
				break;
			if (pos3 > pos2) {
				int offset = strlen("<value><string>");
				char uri[pos3 - pos2 - offset + 1];
				strncpy (uri, pos2 + offset, pos3 - pos2 - offset);
				uri[pos3 - pos2 - offset] = 0;
				uint16_t port;
				char ip[32];
				XMLRPCServer::extractURI(uri, ip, &port);
				os_printf("URI: %s:::%d\n", ip, port);
				//os_printf("URI: %s\n", uri);
				// Check if this uri already exists in a "PublisherURIs" list.
				if (strcmp(ip, IP_ADDR)) { // TODO: replace this with a method to check if ip is not equal self ip
					TopicReader* self = (TopicReader*) obj;
					self->requestTopic(ROS_MASTER_IP, port);
				}
			}
			pos = pos3;
		}
	} else
		os_printf("pos is NULL\n");

}
