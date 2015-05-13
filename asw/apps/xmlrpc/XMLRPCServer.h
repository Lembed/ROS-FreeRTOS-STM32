#include <stdio.h>
#include <string.h>
#include "tcp.h"
#include <lwip/sockets.h>
#include "lwip/ip_addr.h"
#include "tcpip.h"
#include "api.h"

#include <stdlib.h>
#define XMLRPC_PORT 40000

#define SENDER_IP_ADDR "10.3.84.99"

#define SERVER_PORT_NUM 11311
#define SERVER_IP_ADDRESS "10.3.84.100"

#define TOPIC_COUNT 20
#define MAX_TOPIC_LEN 48
char topics[TOPIC_COUNT][MAX_TOPIC_LEN];
#include "netconf.h"

#define TCP_DATA_SIZE 1200
class HTTPClient
{
private:
	struct TCPData
	{
		uint16_t serverPort;
		uint32_t serverIP;
		char data[TCP_DATA_SIZE];
	};

	static void tcptask(void* arg)
	{
		static uint16_t port = 30000;
		HTTPClient* self = (HTTPClient*) arg;


	    TCPData data;

	    for(;;)
	    {
			if (xQueueReceive(self->qHandle, &data, 100))
			{
			    port++;

				int socket_fd;
			    struct sockaddr_in sa,ra;

			    int recv_data;  // Creates an TCP socket (SOCK_STREAM) with Internet Protocol Family (PF_INET).
			    // Protocol family and Address family related. For example PF_INET Protocol Family and AF_INET family are coupled.

			    socket_fd = socket(PF_INET, SOCK_STREAM, 0);

			    if ( socket_fd < 0 )
			    {

			        os_printf("socket call failed");

			    }


			    memset(&sa, 0, sizeof(struct sockaddr_in));
			    sa.sin_family = AF_INET;
			    sa.sin_addr.s_addr = inet_addr(SENDER_IP_ADDR);
			    sa.sin_port = htons(port);

			    os_printf("Binding to Port Number %d ,IP address %s\n", port, SENDER_IP_ADDR);
			    /* Bind the TCP socket to the port SENDER_PORT_NUM and to the current
			    * machines IP address (Its defined by SENDER_IP_ADDR).
			    * Once bind is successful for UDP sockets application can operate
			    * on the socket descriptor for sending or receiving data.
			    */
			    if (bind(socket_fd, (struct sockaddr *)&sa, sizeof(struct sockaddr_in)) == -1)
			    {
			    os_printf("Bind to Port Number %d ,IP address %s failed\n", port, SENDER_IP_ADDR);
			    close(socket_fd);

			    }

				os_printf("Data received! %d\n", data.serverPort);


				// Receiver connects to server ip-address.
				memset(&ra, 0, sizeof(struct sockaddr_in));
				ra.sin_family = AF_INET;
				ra.sin_addr.s_addr = data.serverIP;
				ra.sin_port = htons(data.serverPort);
				os_printf("Connecting to Port Number %d ,IP address %08x\n", data.serverPort, data.serverIP);


				if(connect(socket_fd,(const sockaddr*)&ra,sizeof(struct sockaddr_in)) < 0)
				{

					os_printf("connect failed \n");
					close(socket_fd);
				}

				else
				{
					self->onConnected(data.serverPort);
					os_printf("bufferLen:%d\n", strlen(data.data));
					send(socket_fd, data.data, strlen(data.data), 0);
					self->onSent();

					recv_data = recv(socket_fd,self->rxBuffer, sizeof(self->rxBuffer), 0);
					if(recv_data < 0)
					{

						os_printf("recv failed \n");
						close(socket_fd);

					}
					else
					{
						self->onReceive(self->rxBuffer);
					}

					close(socket_fd);
				}
			}
	    }

	    vTaskDelete(NULL);
	}


	HTTPClient()
	{
		qHandle = xQueueCreate(4, sizeof(TCPData));
		xTaskCreate(tcptask, (const signed char*)"HTTPClient", 2048, this, tskIDLE_PRIORITY + 2, NULL);
	}

	static HTTPClient* _instance;

	void onConnected(uint16_t port)
	{
		os_printf("Connected, serverport:%d!\n", port);
	}
	void onReceive(const char* data)
	{
		os_printf("Received %d bytes!\n", strlen(data));
		if (receiveCallback != NULL)
		{
			receiveCallback(data);
		}
	}
	void onSent()
	{
		os_printf("Sent!\n");
	}
	void(*receiveCallback)(const char* data);

	xQueueHandle qHandle;
	char rxBuffer[1024];
public:
    static HTTPClient *instance()
    {
        if (!_instance)
          _instance = new HTTPClient();
        return _instance;
    }
    void sendData(const char* data, uint16_t port, void(*receiveCallback)(const char* data) = NULL)
    {
    	TCPData tcpData;
    	memcpy(tcpData.data, data, TCP_DATA_SIZE);
    	tcpData.serverIP = inet_addr("10.3.84.100");
    	tcpData.serverPort = port;
    	this->receiveCallback = receiveCallback;
    	if (xQueueSend(qHandle, &tcpData, 0))
			os_printf("Enqueueing data!\n");
		else
			os_printf("Queue is full!\n");
    }

};

HTTPClient* HTTPClient::_instance = NULL;


class TCPServerBase
{
private:
	static void close_conn(struct tcp_pcb *pcb){
	      tcp_arg(pcb, NULL);
	      tcp_sent(pcb, NULL);
	      tcp_recv(pcb, NULL);
	      tcp_close(pcb);
	}
	static err_t data_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
	{
		TCPServerBase* self = (TCPServerBase*) arg;
		self->onSendAcknowledged();
	}
	static err_t echo_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
	{
		TCPServerBase* self = (TCPServerBase*) arg;
		int i;
	    int len;
	    char *pc;
	    if (err == ERR_OK && p != NULL)
	    {
	    	// Inform TCP that we have taken the data.
	        tcp_recved(pcb, p->tot_len);

	        //pointer to the pay load
	        pc=(char *)p->payload;

	        //size of the pay load
	        len = p->tot_len;

	        self->onReceive(pc);
	        self->receiveCallback(pc, self->buffer);

	        pbuf_free(p);

	        if (self->buffer != NULL && self->bufferLength > 0)
	        {
				err = tcp_write(pcb, self->buffer, self->bufferLength, 0);
				tcp_sent(pcb, data_sent);
	        }

	        close_conn(pcb);

	    }
	    if (err == ERR_OK && p == NULL)
	    {
	    	close_conn(pcb);
	    }
	    return ERR_OK;
	}

	static err_t echo_accept(void *arg, struct tcp_pcb *pcb, err_t err)
	{
		TCPServerBase* self = (TCPServerBase*) arg;
		self->onAccept();
		tcp_setprio(pcb, TCP_PRIO_MIN);
	    tcp_recv(pcb, echo_recv);
	    tcp_err(pcb, NULL); //Don't care about error here
	    tcp_poll(pcb, NULL, 4); //No polling here
	    return ERR_OK;
	}

	static void tcptask(void* arg)
	{
		struct tcp_pcb *pcb = tcp_new();
		TCPServerBase* self = (TCPServerBase*) arg;
	    tcp_arg(pcb, arg);
	    tcp_bind(pcb, IP_ADDR_ANY, self->port);
	    while(1)
	    {
	    	pcb = tcp_listen(pcb);
			tcp_accept(pcb, echo_accept);
			vTaskDelay(10);
	    }
	    vTaskDelete(NULL);
	}
protected:
	void createBuffer(uint16_t size)
	{
		// Buffer can only be created once.
		if (buffer == NULL)
		{
			bufferLength = size;
			buffer = new char[bufferLength];
		}
	}
public:
	TCPServerBase(const char* taskName, uint16_t port, void(*receiveCallback)(const char* data, char* buffer) = NULL)
	{
		this->port = port;
		this->receiveCallback = receiveCallback;
		xTaskCreate(tcptask, (const signed char*)taskName, 128, this, tskIDLE_PRIORITY + 2, NULL);
	}
private:
	char* buffer;
	uint16_t bufferLength;
protected:
	uint16_t port;
	virtual void onAccept() = 0;
	virtual void onReceive(const char* data) = 0;
	virtual void onSendAcknowledged() = 0;
	void(*receiveCallback)(const char* data, char* buffer);

};

#define HTTP_SERVER_BUFFER_SIZE 1500

class HTTPServer : public TCPServerBase
{
public:
	HTTPServer(const char* taskName, uint16_t port, void(*receiveCallback)(const char* data, char* buffer) = NULL)
	: TCPServerBase(taskName, port, receiveCallback)
	{
		createBuffer(1500);
	}
private:
	virtual void onAccept()
	{
		os_printf("Accept, port:%d!\n", port);
	}
	virtual void onReceive(const char* data)
	{
		os_printf("Received %d bytes!\n", strlen(data));
	}
	virtual void onSendAcknowledged()
	{

	}




};

class XMLRequest
{
protected:
	static void createRequestHeader(const char* hostURI, int contentLength, char* data)
	{
		if (data != NULL)
		{
			strcpy(data, "POST / HTTP/1.1\nUser-Agent: curl/7.35.0\nHost: ");
			strcat(data, hostURI);
			strcat(data, "\\Accept: */*\nContent-Length: ");
			char contentLen[16];
			sprintf(contentLen, "%d", contentLength);
			strcat(data, contentLen);
			strcat(data, "\nContent-Type: application/x-www-form-urlencoded\n\n");
		}
	}
	static void createResponseHeader(int contentLength, char* data)
	{
		if (data != NULL)
		{
    		strcpy(data, "HTTP/1.0 200 OK\nServer: BaseHTTP/0.3 Python/2.7.6\n");
    		strcat(data, "Date: Sat, 01 January 1970 00:00:00 GMT\nContent-type: text/xml\nContent-length: ");
            char contentLen[16];
    		sprintf(contentLen, "%d", contentLength);
    		strcat(data, contentLen);
    		strcat(data, "\n\n");
		}
	}

	char data[1500];
	char header[256];
	char xml[1024];
public:
	const char* getData()
	{
		return data;
	}
};

class RegisterRequest : public XMLRequest
{
public:
	RegisterRequest(const char* methodName, const char* uri, const char* callerID, const char* topic, const char* msgType)
	{
		strcpy(xml, "<?xml version=\"1.0\"?> <methodCall> <methodName>");
		strcat(xml, methodName);
		strcat(xml, "</methodName> <params> <param> <value>/");
		strcat(xml, callerID);
		strcat(xml, "</value> </param> <param> <value>");
		strcat(xml, topic);
		strcat(xml, "</value> <param> <value>");
		strcat(xml, msgType);
		strcat(xml, "</value> </param> <param> <value>");
		char portStr[6];
		sprintf(portStr, "%d", XMLRPC_PORT);
		strcat(xml, "http://10.3.84.99:");
		strcat(xml, portStr);
		strcat(xml, "</value> </param> </param> </params> </methodCall>");


		createRequestHeader(uri, strlen(xml), header);

		strcpy(data, header);
		strcat(data, xml);
	}
};

class TopicRequest : public XMLRequest
{
public:
	TopicRequest(const char* methodName, const char* uri)
	{
		strcpy(xml, "<?xml version=\"1.0\"?> <methodCall> <methodName>");
		strcat(xml, methodName);
		strcat(xml, "</methodName>");
		strcat(xml, "<params><param><value>/listener</value></param><param><value>/chatter</value></param><param><value><array><data><value><array><data><value>UDPROS</value><value><base64>EgAAAGNhbGxlcmlkPS9saXN0ZW5lcicAAABtZDVzdW09OTkyY2U4YTE2ODdjZWM4YzhiZDg4");
		strcat(xml, "M2VjNzNjYTQxZDEOAAAAdG9waWM9L2NoYXR0ZXIUAAAAdHlwZT1zdGRfbXNncy9TdHJpbmc=</base64></value><value>10.3.84.99</value><value><i4>44100</i4></value><value><i4>1500</i4></value></data></array></value></data></array></value></param></params></methodCall>");

		createRequestHeader(uri, strlen(xml), header);

		strcpy(data, header);
		strcat(data, xml);
	}
};

class TopicResponse : public XMLRequest
{
public:
	TopicResponse(const char* localIP, const uint16_t& localPort, const uint32_t& connectionID)
	{
		char tmp[15];
		strcpy(xml, "<?xml version=\"1.0\"?><methodResponse><params><param><value><array><data><value><i4>1</i4></value>");
		strcat(xml, "<value></value><value><array><data><value>UDPROS</value><value>");
		strcat(xml, localIP);
		strcat(xml, "</value><value><i4>");
		sprintf(tmp, "%d", localPort);
		strcat(xml, tmp);
		strcat(xml, "</i4>");
		strcat(xml, "</value><value><i4>");
		sprintf(tmp, "%d", connectionID);
		strcat(xml, tmp);
		strcat(xml, "</i4>");
		strcat(xml, "</value><value><i4>1500</i4></value><value><base64>");
		// TODO: Build the base64encoded data (Connection Header) dynamically.
		strcat(xml, "EAAAAGNhbGxlcmlkPS90YWxrZXInAAAAbWQ1c3VtPTk5MmNlOGExNjg3Y2VjOGM4YmQ4ODNlYzczY2E0MWQxHwAAAG1lc3NhZ2VfZGVmaW5pdGlvbj1zdHJpbmcgZGF0YQoOAAAAdG9waWM9L2NoYXR0ZXIUAAAAdHlwZT1zdGRfbXNncy9TdHJpbmc=");
		strcat(xml, "</base64></value></data></array></value></data></array></value></param></params></methodResponse>");

		createResponseHeader(strlen(xml), header);

		strcpy(data, header);
		strcat(data, xml);
	}

};

#define QUEUE_LEN 10
#define QUEUE_MSG_SIZE sizeof(UDPMessage)
#define RXTIMEOUT 100

typedef struct EndPoint
{
	uint32_t connectionID;
	struct ip_addr ip;
	uint16_t port;
} EndPoint;

typedef struct UDPMessage
{
	char topic[MAX_TOPIC_LEN];
	char data[128];
} UDPMessage;

#define MAX_UDP_CONNECTIONS 20




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
};
#include "rcl.h"
#include "msg.h"



uint32_t UDPConnection::ID = 10000;




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
		memcpy(data, msg, QUEUE_MSG_SIZE);
		// Try to send message if queue is non-full.
		// TODO: Check if we are still "connected" to the end point. (i.e. the node at the remote end is still running)
		if (xQueueSend(qHandle, &data, 0))
			os_printf("Enqueueing data!\n");
		else
			os_printf("Queue is full!\n");
	}
	void dequeueMessage(UDPMessage* msg)
	{
		// Initialize memory (in stack) for message.
		unsigned char data[QUEUE_MSG_SIZE];

		// Try to receive message, put the task to sleep for at most RXTIMEOUT ticks if queue is empty.
		for(;;)
		{
			if (xQueueReceive(qHandle, data, RXTIMEOUT))
			{
				memcpy(msg, data, QUEUE_MSG_SIZE);
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

UDPHandler* UDPHandler::_instance = NULL;
#define UDP_LOCAL_PORT 46552

#define MASTER_URI "10.3.84.100:11311"

class TopicWriter
{
	char topic[MAX_TOPIC_LEN];
	uint16_t lastConnectionsIndex;
	UDPConnection* connections[MAX_UDP_CONNECTIONS];
	xQueueHandle qHandle;
public:
	TopicWriter(const char* topic)
	{
		strcpy(this->topic, topic);
		lastConnectionsIndex = 0;
		qHandle = xQueueCreate(QUEUE_LEN, QUEUE_MSG_SIZE);
	}
	void serializeMsg(const ros::Msg& msg, unsigned char* outbuffer)
	{
		unsigned char stream1[100];
		uint32_t offset = msg.serialize(stream1);
		memcpy(outbuffer, &offset, sizeof(uint32_t));
		memcpy(outbuffer+sizeof(uint32_t), stream1, offset);
	}

	void publishMsg(const ros::Msg& msg)
	{
		UDPMessage udpMessage;
		strcpy(udpMessage.topic, topic);
		serializeMsg(msg, (unsigned char*)udpMessage.data);
		UDPHandler* uh = UDPHandler::instance();
		uh->enqueueMessage(&udpMessage);
	}
	UDPConnection* getConnection(uint16_t port)
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
	UDPConnection* const* getConnections()
	{
		return connections;
	}
	const char* getTopic()
	{
		return topic;
	}
};

#define MAX_TOPIC_WRITERS 10
TopicWriter* topicWriters[MAX_TOPIC_WRITERS];

class XMLRPCServer
{
private:


public:
	static void UDPSend(void* params)
	{
		UDPHandler* uh = UDPHandler::instance();
		struct netconn* conn = netconn_new( NETCONN_UDP );
	    netconn_bind(conn, IP_ADDR_ANY, UDP_LOCAL_PORT);
	    static uint8_t counter = 1;

		for(;;)
		{
			UDPMessage msg;
			uh->dequeueMessage(&msg);

			TopicWriter* tw = getTopicWriter(msg.topic);
			if (tw != NULL)
			{
				EndPoint endpoint;
				endpoint.ip.addr = inet_addr("10.3.84.100");
				UDPConnection* const* connections = tw->getConnections();
				if (connections != NULL)
				{
					for(int i= 0; i<MAX_UDP_CONNECTIONS; i++)
					{
						const UDPConnection* connection = connections[i];
						if (connection)
						{
							endpoint.port = connection->getPort();
							endpoint.connectionID = connection->getID();
							err_t err = netconn_connect(conn, &endpoint.ip, endpoint.port);
							os_printf("Connecting %s:%d, err:%d\n",endpoint.ip, endpoint.port, err);
							struct netbuf *buf = netbuf_new();
							char msgHeader[8];
							memcpy(&msgHeader[0], &endpoint.connectionID, sizeof(uint32_t));
							msgHeader[4] = 0;
							msgHeader[5] = counter++;
							msgHeader[6] = 0x01;
							msgHeader[7] = 0;
							uint32_t msgLen = *((uint32_t*) msg.data)+4;
							void* data = netbuf_alloc(buf, msgLen+sizeof(msgHeader)); // Also deallocated with netbuf_delete(buf)

							memcpy (data, msgHeader, sizeof (msgHeader));
							memcpy (data+sizeof (msgHeader), msg.data, msgLen);
							netconn_send(conn, buf);
							netbuf_delete(buf);
						}
					}
				}
			}
		}
	}
	static TopicWriter* getTopicWriter(const char* topic)
	{
		for(uint16_t i=0; i<MAX_TOPIC_WRITERS;i++)
		{
			if (topicWriters[i] != NULL)
			{
				TopicWriter* tw = topicWriters[i];
				if (!strcmp(tw->getTopic(), topic))
				{
					return tw;
				}
			}
		}
		return NULL;
	}
	static void XMLRPCServerReceiveCallback(const char* data, char* buffer)
	{
		{
	    	char* pos = strstr((char*)data, "<methodName>");
			char* pos2 = strstr((char*)data, "</methodName>");
			if (pos2 > pos)
			{
				  char methodName[pos2-pos-12];
			  strncpy (methodName, pos+12, pos2-pos-12);
			  methodName[pos2-pos-12] = 0;
			  if (!strcmp(methodName, "requestTopic"))
			  {
				  os_printf("name:%s\n",methodName);
			  }
			  else
				  return;
			}
	    }
	    os_printf("Strlen:%d\n",strlen(data));

	    /*for (int i=0; i<strlen(data)+128; i=i+127)
	    {
	    	static char temp[128];
	    	strncpy(temp, data+i,127);
	    	temp[127] = 0;
	    	os_printf(temp);
	    }
	    os_printf("\n");*/

	    // TODO: Why does strstr(data, "<i4>") not work? Why does it work sometimes with the data+550 hack?
		char* pos = strstr(data+550, "<i4>");
	    char* pos2 = strstr(data+550, "</i4>");
	    os_printf("pos:%d, pos2:%d\n", pos, pos2);

	    if (pos < pos2)
	    {
	    	char portStr[pos2-pos-5];
	    	strncpy (portStr, pos+4, pos2-pos-4);
	    	portStr[pos2-pos-4] = 0;
	    	uint16_t port = atoi(portStr);
	    	os_printf("Port: %d\n",port);


	    	char* pos3 = strstr((char*)data, "</value></param><param><value>/");
	    	char* pos4;
	    	int len = strlen("</value></param><param><value>/");

	    	if (pos3)
	    	{
	    		pos4 = strstr((char*)pos3+len, "</value>");
	    		//os_printf("_pos:%d, _pos2:%d\n", pos3, pos4);
	    		if (pos4 > pos3)
	    		{
					char topic[pos4-pos3-len+1];
					strncpy (topic, pos3+len, pos4-pos3-len);
					topic[pos4-pos3-len] = 0;
					os_printf("_pos:%d, _pos2:%d, %s\n", pos3, pos4, topic);

					// TODO: Move UDPConnection to registerPublishers. Then extract topic name from data. Afterwards, find the corresponding connection.
					TopicWriter* tw = getTopicWriter(topic);
					if (tw != NULL)
					{
						UDPConnection* connection = tw->getConnection(port);
						if (connection!= NULL)
						{
							os_printf("Connection ID: %d\n", connection->getID());
							XMLRequest* response = new TopicResponse(SENDER_IP_ADDR, UDP_LOCAL_PORT, connection->getID());
							strcpy(buffer, response->getData());
						}
					}
	    		}
	    	}
	    }
	}

	static void start()
	{
		HTTPServer* server = new HTTPServer("HTTPServer", XMLRPC_PORT, XMLRPCServerReceiveCallback);
		xTaskCreate(UDPSend, (const signed char*)"UDPSend", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
	}

	static TopicWriter* registerPublisher(const char* callerID, const char* topic, const char* msgType)
	{
		static uint16_t port = 41000;
		XMLRequest* req = new RegisterRequest("registerPublisher", MASTER_URI, callerID, topic, msgType);
		static uint16_t publisherID = 1;
		char clientname[32];
		sprintf(clientname, "pclient%d", publisherID++);
		static uint16_t lastTopicWriterIndex = 0;
		TopicWriter* tw = new TopicWriter(topic);
		topicWriters[lastTopicWriterIndex++] = tw;
		HTTPClient::instance()->sendData(req->getData(), 11311);
		//HTTPClient* client = new HTTPClient(clientname, req->getData(), strlen(req->getData()), port++, SERVER_IP_ADDRESS, 11311);
		return tw;
	}

	static void extractURI(const char* uri, char* ip, uint16_t* port)
	{
		char* p = strstr(uri, "://");
		if (p != NULL)
		{
			char* pos = strstr(p+3, ":");
			if (pos != NULL && ip!= NULL)
			{
				memcpy(ip, p+3, pos-p-3);
				char portStr[6];
				strcpy(portStr, pos+1);
				*port = atoi(portStr);
			}
		}
	}

	static void connectPublishers(const char* data)
	{
		char text[100];
		char* pos = strstr((char*)data, "Subscribed to");
		if (pos != 0)
		{
			//strncpy(text, pos, 99);
			//os_printf("Text: %s\n", text);
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
					extractURI(uri, ip, &port);
					os_printf("URI: %s:::%d\n", ip, port);
					//os_printf("URI: %s\n", uri);
					// Check if this uri already exists in a "PublisherURIs" list.
					if (strcmp(ip, "10.3.84.99"))
					{
					requestTopic(SERVER_IP_ADDRESS, port);
					break; // TODO: remove this!
					}
				}
				pos = pos3;
			}
		}
		else
			os_printf("pos is NULL\n");

	}

	static void registerSubscriber(const char* callerID, const char* topic, const char* msgType)
	{
		static uint16_t port = 42000;
		XMLRequest* req = new RegisterRequest("registerSubscriber", MASTER_URI, callerID, topic, msgType);
		static uint16_t subscriberID = 1;
		char clientname[32];
		sprintf(clientname, "sclient%d", subscriberID++);
		HTTPClient::instance()->sendData(req->getData(), 11311, connectPublishers);
		//HTTPClient* client = new HTTPClient(clientname, req->getData(), strlen(req->getData()), port++, SERVER_IP_ADDRESS, 11311, connectPublishers);
	}
	static void UDPreceive(void* params)
	{
		struct netconn *conn;
		static unsigned short port;
		struct netbuf *buf;
		err_t err;

		// Initialize memory (in stack) for message.
		char message[60];
		os_printf("Test!\n");
		conn = netconn_new(NETCONN_UDP);
		for(;;)
		{
			// Check if connection was created successfully.
			if (conn!= NULL)
			{
				err = netconn_bind(conn, IP_ADDR_ANY, 44100);

				// Check if we were able to bind to port.
			    if (err == ERR_OK)
			    {
			    	vTaskDelay(200);
			    	portTickType xLastWakeTime;
					// Initialize the xLastWakeTime variable with the current time.
					xLastWakeTime = xTaskGetTickCount();

					// Start periodic loop.
					while (1)
					{
						buf = netconn_recv(conn);
						if (buf!= NULL)
						{
							os_printf("Receive!\n");
							//vTaskDelay(200);
							// Copy received data into message.
							uint16_t len = netbuf_len(buf);
							if (len>15)
							{
								netbuf_copy (buf, &message, len);
								uint32_t length = *((uint32_t*)&message[12]);
								if (len> length+15)
								{
									message[16+length] = 0;
									os_printf("%s\n", &message[16]);
								}
							}
							// Deallocate previously created memory.
							netbuf_delete(buf);
						}
						// Use delay until to guarantee periodic execution of each loop iteration.
						vTaskDelayUntil(&xLastWakeTime, 10);
					}
			    }
			    else
			    {
			    	os_printf("cannot bind netconn\n");
			    }
			}
			else
			{
				os_printf("cannot create new UDP netconn\n");
			}
			// If connection failed, wait for 50 ms before retrying.
			vTaskDelay(50);
		}
	}

	static void requestTopicResponse(const char* data)
	{
		// TODO: extract connectionID from data and create a new TopicReader instance with it
		xTaskCreate(UDPreceive, (const signed char*)"UDPReceive", 128, NULL, tskIDLE_PRIORITY + 3, NULL);
	}

	static void requestTopic(const char* ip, uint16_t serverPort)
	{
		static uint16_t port = 43000;
		XMLRequest* req = new TopicRequest("requestTopic", MASTER_URI);
		static uint16_t reqID = 1;
		char clientname[32];
		sprintf(clientname, "reqclient%d", reqID++);
		HTTPClient::instance()->sendData(req->getData(), serverPort, requestTopicResponse);

		//HTTPClient* client = new HTTPClient(clientname, req->getData(), strlen(req->getData()), port++, ip, serverPort, requestTopicResponse);
	}
};
