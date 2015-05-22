#include "TopicWriter.h"
#include "XMLRequest.h"
#include "XMLRPCServer.h"
UDPHandler* UDPHandler::_instance = NULL;
uint32_t UDPConnection::ID = 10000;
TopicWriter::TopicWriter(const char* callerID, const char* topic, const char* msgType)
{
	strcpy(this->topic, topic);
	lastConnectionsIndex = 0;
	qHandle = xQueueCreate(QUEUE_LEN, QUEUE_MSG_SIZE);

	XMLRequest* req = new RegisterRequest("registerPublisher", MASTER_URI, callerID, topic, msgType);
	XMLRPCServer::sendRequest(req->getData(), 11311);
}
void TopicWriter::serializeMsg(const ros::Msg& msg, unsigned char* outbuffer)
{
	unsigned char stream1[100];
	uint32_t offset = msg.serialize(stream1);
	memcpy(outbuffer, &offset, sizeof(uint32_t));
	memcpy(outbuffer+sizeof(uint32_t), stream1, offset);
}

void TopicWriter::publishMsg(const ros::Msg& msg)
{
	UDPMessage udpMessage;
	strcpy(udpMessage.topic, topic);
	serializeMsg(msg, (unsigned char*)udpMessage.data);
	UDPHandler* uh = UDPHandler::instance();
	uh->enqueueMessage(&udpMessage);
}

UDPConnection* TopicWriter::getConnection(uint16_t port)
{
	if (lastConnectionsIndex < MAX_TOPIC_LEN)
	{
		for(uint16_t i=0; i<MAX_UDP_CONNECTIONS; i++)
		{
			if (connections[i] != NULL && connections[i]->getPort() == port)
			{
				return connections[i];
			}
		}

		UDPConnection* conn = new UDPConnection(port);
		connections[lastConnectionsIndex++] = conn;
		return conn;
	}
	return NULL;
}
UDPConnection* const* TopicWriter::getConnections()
{
	return connections;
}
const char* TopicWriter::getTopic()
{
	return topic;
}
