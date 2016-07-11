#include "TopicWriter.h"
#include "XMLRequest.h"
#include "XMLRPCServer.h"
#include "device_config.h"

UDPHandler* UDPHandler::_instance = NULL;
uint32_t UDPConnection::ID = 10000;

extern "C"
{
#include "ros.h"
}

TopicWriter::TopicWriter(const char* callerID, const char* topic, const char* msgType)
{
	strcpy(this->topic, topic);
	lastConnectionsIndex = 0;
	qHandle = xQueueCreate(QUEUE_LEN, QUEUE_MSG_SIZE);

	XMLRequest* req = new RegisterRequest("registerPublisher", ROS_MASTER_IP, callerID, topic, msgType);
	XMLRPCServer::sendRequest(req->getData(), SERVER_PORT_NUM, connectSubscribers, this);
}
void TopicWriter::serializeMsg(const ros::Msg& msg, unsigned char* outbuffer)
{
	unsigned char stream1[sizeof(UDPMessage)];
	uint32_t offset = msg.serialize(stream1);
    if (outbuffer != NULL)
    {
        memcpy(outbuffer, &offset, sizeof(uint32_t));
        memcpy(outbuffer+sizeof(uint32_t), stream1, offset);
    }
    else
        os_printf("TopicWriter::serializeMsg msg is NULL!\n");
}

void TopicWriter::publishMsg(const ros::Msg& msg)
{
	UDPMessage udpMessage;
	strcpy(udpMessage.topic, topic);
	serializeMsg(msg, (unsigned char*)udpMessage.data);
	UDPHandler* uh = UDPHandler::instance();
	uh->enqueueMessage(&udpMessage);
}

UDPConnection* TopicWriter::createConnection(uint16_t port)
{
	if (lastConnectionsIndex < MAX_UDP_CONNECTIONS)
	{
		UDPConnection* conn = new UDPConnection(port);
		connections[lastConnectionsIndex] = conn;
		lastConnectionsIndex++;
		os_printf("Created connection to port %d.\n", port);
		return conn;
	}
	else
	{
		// Look for connections with port = 0
		for(uint16_t i=0; i<MAX_UDP_CONNECTIONS; i++)
		{
			if (connections[i] != NULL && connections[i]->getPort() == 0)
			{
				return connections[i];
			}
		}

		os_printf("Cannot create more than %d UDP connections!\n", MAX_UDP_CONNECTIONS);
	}
}

UDPConnection* TopicWriter::getConnection(uint16_t port)
{
	if (port == 0)
	{
		os_printf("0 is an invalid UDP port!\n");
		return NULL;
	}

	if (lastConnectionsIndex < MAX_TOPIC_LEN)
	{
		for(uint16_t i=0; i<MAX_UDP_CONNECTIONS; i++)
		{
			if (connections[i] != NULL && connections[i]->getPort() == port)
			{
				return connections[i];
			}
		}

		return createConnection(port);
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

void TopicWriter::connectSubscribers(const void* obj, const char* data)
{
	os_printf("Connect subscribers!\n");
	char* pos = strstr((char*)data, "as publisher of");
	if (pos != 0)
	{
		while(1)
		{
			char* pos2 = strstr((char*)pos, "<value><string>");
			char* pos3 = strstr((char*)pos2, "</string></value>");
			if (pos2 == NULL || pos3 == NULL)
				break;
			if (pos3 > pos2)
			{
				int offset = strlen("<value><string>");
				char uri[pos3-pos2-offset+1];
				strncpy (uri, pos2+offset, pos3-pos2-offset);
				uri[pos3-pos2-offset] = 0;
				uint16_t port;
				char ip[32];
				XMLRPCServer::extractURI(uri, ip, &port);
				os_printf("URI: %s:::%d\n", ip, port);
				//os_printf("URI: %s\n", uri);
				// Check if this uri already exists in a "PublisherURIs" list.

				if (strcmp(ip, IP_ADDR))  // TODO: replace this with a method to check if ip is not equal self ip
				{
					TopicWriter* self = (TopicWriter*) obj;
					// TODO: Send publisher update to each remote subscriber
					XMLRequest* req = new PublisherUpdate(self->topic, uri);
					XMLRPCServer::sendRequest(req->getData(), port);
				}
			}
			pos = pos3;
		}
	}
	else
		os_printf("pos is NULL\n");

}


void TopicWriter::deleteConnection(uint16_t port)
{
	for(uint16_t i=0; i<MAX_UDP_CONNECTIONS; i++)
	{
		if (connections[i] != NULL && connections[i]->getPort() == port)
		{
			connections[i]->setPort(0);
			return;
		}
	}

}
