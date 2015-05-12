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


class TCPClientBase
{
private:
	static void tcptask(void* arg)
	{
		TCPClientBase* self = (TCPClientBase*) arg;
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
	    sa.sin_port = htons(self->port);

	    os_printf("Binding to Port Number %d ,IP address %s\n", self->port, SENDER_IP_ADDR);
	    /* Bind the TCP socket to the port SENDER_PORT_NUM and to the current
	    * machines IP address (Its defined by SENDER_IP_ADDR).
	    * Once bind is successful for UDP sockets application can operate
	    * on the socket descriptor for sending or receiving data.
	    */
	    if (bind(socket_fd, (struct sockaddr *)&sa, sizeof(struct sockaddr_in)) == -1)
	    {
	    os_printf("Bind to Port Number %d ,IP address %s failed\n", self->port, SENDER_IP_ADDR);
	    close(socket_fd);

	    }
	    // Receiver connects to server ip-address.

	    memset(&ra, 0, sizeof(struct sockaddr_in));
	    ra.sin_family = AF_INET;
	    ra.sin_addr.s_addr = inet_addr(self->serverIP);
	    ra.sin_port = htons(self->serverPort);
	    os_printf("Connecting to Port Number %d ,IP address %s\n", self->serverPort, self->serverIP);


	    if(connect(socket_fd,(const sockaddr*)&ra,sizeof(struct sockaddr_in)) < 0)
	    {

	        os_printf("connect failed \n");
	        close(socket_fd);
	    }

	    else
	    {
			self->onConnected();
	    	os_printf("bufferLen:%d\n", strlen(self->buffer));
			send(socket_fd, self->buffer, strlen(self->buffer), 0);
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

	    vTaskDelete(NULL);
	}
public:
	TCPClientBase(const char* taskName, const char* buffer, uint16_t bufferLength, uint16_t port, const char* serverIP, uint16_t serverPort)
	{
		this->buffer = (char*)buffer;
		this->port = port;
		this->serverPort = serverPort;
		this->serverIP = serverIP;
		//memcpy(&this->serverIP, serverIP, sizeof(struct ip_addr));
		xTaskCreate(tcptask, (const signed char*)taskName, 512, this, tskIDLE_PRIORITY + 2, NULL);
	}
private:
	char* buffer;
	char rxBuffer[1024];
	uint16_t bufferLength;
protected:
	uint16_t port, serverPort;
	const char* serverIP;
	virtual void onConnected() = 0;
	virtual void onReceive(const char* data) = 0;
	virtual void onSent() = 0;

};

class HTTPClient : public TCPClientBase
{
public:
	HTTPClient(const char* taskName, const char* buffer, uint16_t bufferLength, uint16_t port, const char* serverIP, uint16_t serverPort, void(*receiveCallback)(const char* data) = NULL)
	: TCPClientBase(taskName, buffer, bufferLength, port, serverIP, serverPort)
	{
		this->receiveCallback = receiveCallback;
	}
private:
	virtual void onConnected()
	{
		os_printf("Connected, serverport:%d!\n", serverPort);
	}
	virtual void onReceive(const char* data)
	{
		os_printf("Received %d bytes!\n", strlen(data));
		if (receiveCallback != NULL)
		{
			receiveCallback(data);
		}
	}
	virtual void onSent()
	{
		os_printf("Sent!\n");
	}
	void(*receiveCallback)(const char* data);

};


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
	EndPoint endpoint;
	char data[118];
} UDPMessage;

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
class XMLRPCServer
{
private:


public:
	static void UDPSend(void* params)
	{
		// TODO: port should already be included in the dequeued message.
		uint16_t port = *((uint16_t*)params);
		os_printf("My port:%d\n", port);
		UDPHandler* uh = UDPHandler::instance();
		struct netconn* conn = netconn_new( NETCONN_UDP );
	    netconn_bind(conn, IP_ADDR_ANY, UDP_LOCAL_PORT);
		for(;;)
		{
			UDPMessage msg;
			uh->dequeueMessage(&msg);
			msg.endpoint.port = port; // TODO: port should already be included in the dequeued message.
			netconn_connect(conn, &msg.endpoint.ip, msg.endpoint.port);
			struct netbuf *buf = netbuf_new();
		    static char msgHeader[] = {
		    		0x03, 0x00, 0x00, 0x00,
		    		0x00, 0x01, 0x01, 0x00
		    };
			uint32_t msgLen = *((uint32_t*) msg.data)+4;
		    void* data = netbuf_alloc(buf, msgLen+sizeof(msgHeader)); // Also deallocated with netbuf_delete(buf)

		    msgHeader[5]++;

		    memcpy (data, msgHeader, sizeof (msgHeader));
		    memcpy (data+sizeof (msgHeader), msg.data, msgLen);
		    netconn_send(conn, buf);
		    netbuf_delete(buf);
		}

	}
	static void XMLRPCServerReceiveCallback(const char* data, char* buffer)
	{
		char* pos = strstr(data+550, "<i4>");
	    char* pos2 = strstr(data+550, "</i4>");

	    os_printf("pos:%d, pos2:%d\n", pos, pos2);

	    if (pos < pos2)
	    {
	    	char portStr[pos2-pos-5];
	    	strncpy (portStr, pos+4, pos2-pos-4);
	    	portStr[pos2-pos-4] = 0;
	    	static uint16_t port = atoi(portStr);
	    	os_printf("Port: %d\n",port);

	    	xTaskCreate(UDPSend, (const signed char*)"UDPSend", 256, &port, tskIDLE_PRIORITY + 2, NULL);

			XMLRequest* response = new TopicResponse(SENDER_IP_ADDR, UDP_LOCAL_PORT, 3);
			strcpy(buffer, response->getData());

	    }
	}

	static void start()
	{
		HTTPServer* server = new HTTPServer("HTTPServer", XMLRPC_PORT, XMLRPCServerReceiveCallback);
	}

	static void registerPublisher(const char* callerID, const char* topic, const char* msgType)
	{
		static uint16_t port = 41000;
		XMLRequest* req = new RegisterRequest("registerPublisher", MASTER_URI, callerID, topic, msgType);
		static uint16_t publisherID = 1;
		char clientname[32];
		sprintf(clientname, "pclient%d", publisherID++);
		HTTPClient* client = new HTTPClient(clientname, req->getData(), strlen(req->getData()), port++, SERVER_IP_ADDRESS, 11311);
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

					requestTopic(SERVER_IP_ADDRESS, port);
					break; // TODO: remove this!
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
		HTTPClient* client = new HTTPClient(clientname, req->getData(), strlen(req->getData()), port++, SERVER_IP_ADDRESS, 11311, connectPublishers);
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
		xTaskCreate(UDPreceive, (const signed char*)"UDPReceive", 128, NULL, tskIDLE_PRIORITY + 3, NULL);
	}

	static void requestTopic(const char* ip, uint16_t serverPort)
	{
		static uint16_t port = 43000;
		XMLRequest* req = new TopicRequest("requestTopic", MASTER_URI);
		static uint16_t reqID = 1;
		char clientname[32];
		sprintf(clientname, "reqclient%d", reqID++);
		HTTPClient* client = new HTTPClient(clientname, req->getData(), strlen(req->getData()), port++, ip, serverPort, requestTopicResponse);
	}
};
