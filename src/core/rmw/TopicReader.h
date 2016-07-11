#ifndef RMW_TOPICREADER_H_
#define RMW_TOPICREADER_H_
#define MAX_TOPIC_LEN 48

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

class TopicReader
{
	char topic[MAX_TOPIC_LEN];
	char callerID[MAX_TOPIC_LEN];
	char md5sum[MAX_TOPIC_LEN];
	char msgType[MAX_TOPIC_LEN];
	uint32_t connectionID;
	xQueueHandle qHandle;

	static void onResponse(const void* obj, const char* data);


	static void connectPublishers(const void* obj, const char* data);
	static const int RX_QUEUE_MSG_SIZE = 128;
	static const int MAX_CALLBACKS = 5;

	void(*callbacks[MAX_CALLBACKS])(void* data, void* obj);
	void* objects[MAX_CALLBACKS];


public:
	TopicReader(const char* callerID, const char* topic, const char* md5sum, const char* msgType);
	void addCallback(void(*callback)(void* data, void* obj), void* obj);
	static void task(void* arg);
	const char* getTopic();
	void requestTopic(const char* ip, uint16_t serverPort);
	uint32_t getConnectionID();
	void enqueueMessage(const char* msg);
	void dequeueMessage(char* msg);
};


#endif /* RMW_TOPICREADER_H_ */
