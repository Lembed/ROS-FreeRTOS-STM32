#ifndef RMW_XMLRPCSERVER_H_
#define RMW_XMLRPCSERVER_H_

#include "TopicWriter.h"
#include "TopicReader.h"

class XMLRPCServer
{
private:
static bool isUDPReceiveTaskCreated;

public:
	static void UDPSend(void* params);
	static TopicWriter* getTopicWriter(const uint16_t port);
	static TopicWriter* getTopicWriter(const char* topic);
	static TopicReader* getTopicReader(const char* topic);
	static TopicReader* getTopicReader(const uint32_t connectionID);
	static void XMLRPCServerReceiveCallback(const char* data, char* buffer);
	static void start();
	static TopicWriter* registerPublisher(const char* callerID, const char* topic, const char* msgType);
	static TopicReader* registerSubscriber(const char* callerID, const char* topic, const char* md5sum, const char* msgType);
	static void UDPreceive(void* params);

	static void sendRequest(const char* data, uint16_t port, void(*receiveCallback)(const void* obj, const char* data) = NULL, void* obj = NULL);
	static void extractURI(const char* uri, char* ip, uint16_t* port);
};

#endif /* RMW_XMLRPCSERVER_H_ */
