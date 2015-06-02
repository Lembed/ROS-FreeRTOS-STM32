#include "XMLRPCServer.h"

#include "tcp.h"
#include <lwip/sockets.h>
#include "lwip/ip_addr.h"
#include "tcpip.h"
#include "api.h"

extern "C"
{
#include "ros.h"
}

#include "XMLRequest.h"

#define SENDER_IP_ADDR "10.3.84.99"

#define SERVER_PORT_NUM 11311
#define SERVER_IP_ADDRESS "10.3.84.100"

#define TOPIC_COUNT 20
#define MAX_TOPIC_LEN 48


#include "netconf.h"


bool XMLRPCServer::isUDPReceiveTaskCreated = false;

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
			struct netconn *conn = netconn_new(NETCONN_TCP);
			  err_t err;
			if (conn!=NULL) {
			    // Bind connection to the specified number
				os_printf("Binding port %d\n", port);
			    err = netconn_bind(conn, NULL, port);

			      if (err == ERR_OK)
			      {
			    	  struct ip_addr ip;
			    	  ip.addr = data.serverIP;
			    	  os_printf("Connecting port %d\n", data.serverPort);
			    	  err = netconn_connect (conn, &ip, data.serverPort);

			    	  if (err == ERR_OK)
			    	  {
			    		  os_printf("Writing data!\n");
			    		  netconn_write(conn, data.data, TCP_DATA_SIZE, NETCONN_COPY);

					          struct netbuf *buf;
					          char *data;
					          u16_t len;
					          uint32_t offset = 0;
					          if ((buf = netconn_recv(conn)) != NULL) {
					            do {
					              netbuf_data(buf, (void**)&data, &len);
					              memcpy(self->rxBuffer+offset, data, len);

					              offset +=len;
					              os_printf("Netconn received %d bytes\n", len);


					            } while (netbuf_next(buf) >= 0);
			        	        self->onReceive(self->rxBuffer);
			        	        netbuf_delete(buf);
					          }
			    	  }
			      }

			  }
			netconn_close (conn );
			netconn_delete (conn );
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
			if (obj != NULL)
				receiveCallback(obj, data);
		}
	}
	void onSent()
	{
		os_printf("Sent!\n");
	}
	void(*receiveCallback)(const void* obj, const char* data);
	void* obj;

	xQueueHandle qHandle;
	char rxBuffer[1024];
public:
    static HTTPClient *instance()
    {
        if (!_instance)
          _instance = new HTTPClient();
        return _instance;
    }
    void sendData(const char* data, uint16_t port, void(*receiveCallback)(const void* obj, const char* data) = NULL, void* obj = NULL)
    {
    	TCPData tcpData;
    	memcpy(tcpData.data, data, TCP_DATA_SIZE);
    	tcpData.serverIP = inet_addr("10.3.84.100");
    	tcpData.serverPort = port;
    	// TODO: Are obj & receiveCallback thread-safe?
    	this->obj = obj;
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
	/*static void tcptask(void* arg)
	{
		TCPServerBase* self = (TCPServerBase*) arg;
		  struct netconn *conn, *newconn;
		  err_t err;

		  LWIP_UNUSED_ARG(arg);

		  // Create a new connection identifier.
		  conn = netconn_new(NETCONN_TCP);

		  if (conn!=NULL) {
		    // Bind connection to the specified port number.
		    err = netconn_bind(conn, NULL, self->port);

		    if (err == ERR_OK) {
		      // Tell connection to go into listening mode.
		      netconn_listen(conn);

		      while (1) {
		    	  os_printf("netconn Accepting...\n");
		        // Grab new connection.
		        newconn = netconn_accept(conn);

		        // Process the new connection.
		        if (newconn) {
		          struct netbuf *buf;
		          char *data;
		          u16_t len;
		          os_printf("netconn Accepted...\n");

		          while(!ERR_IS_FATAL(newconn->err)) { //Fatal errors include connections being closed, reset, aborted, etc

		          uint32_t offset = 0;
		          while ((buf = netconn_recv(newconn)) != NULL) {
		            do {
		            	//taskENTER_CRITICAL();
		              netbuf_data(buf, (void**)&data, &len);
		              memcpy(self->rxBuffer+offset, data, len);
		              //netbuf_copy(buf, self->rxBuffer+offset, len);
		              //taskEXIT_CRITICAL();
		              offset +=len;
		              os_printf("Netconn received %d bytes\n", len);


		            } while (netbuf_next(buf) >= 0);
        	        self->onReceive(self->rxBuffer);
        	        self->receiveCallback(self->rxBuffer, self->buffer);
        	        netconn_write(newconn, self->buffer, strlen(self->buffer), NETCONN_NOCOPY);
        	        netbuf_delete(buf);
		          }

		          // Close connection and discard connection identifier.
		          netconn_close(newconn);
		          netconn_delete(newconn);
		        }
		        }
		        else
		        	os_printf("Fatal Error!!!\n");
		      }
		    } else {
		      os_printf(" can not bind TCP netconn\n");
		    }
		  } else {
			  os_printf("can not create TCP netconn\n");
		  }


	    vTaskDelete(NULL);
	}*/


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
		xTaskCreate(tcptask, (const signed char*)taskName, 512, this, tskIDLE_PRIORITY + 2, NULL);
	}
private:
	char* buffer;
	uint16_t bufferLength;
	char rxBuffer[1024];
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



#define MAX_TOPIC_WRITERS 10
TopicWriter* topicWriters[MAX_TOPIC_WRITERS];

#define MAX_TOPIC_READERS 10
TopicReader* topicReaders[MAX_TOPIC_READERS];

void XMLRPCServer::UDPSend(void* params)
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

TopicWriter* XMLRPCServer::getTopicWriter(const char* topic)
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

TopicReader* XMLRPCServer::getTopicReader(const uint32_t connectionID)
{
	for(uint16_t i=0; i<MAX_TOPIC_READERS;i++)
	{
		if (topicReaders[i] != NULL)
		{
			TopicReader* tr = topicReaders[i];
			if (tr->getConnectionID() == connectionID)
			{
				return tr;
			}
		}
	}
	return NULL;
}

void XMLRPCServer::XMLRPCServerReceiveCallback(const char* data, char* buffer)
{
	os_printf("Receive callback!\n");

	char methodName[48];
	{
		char* pos = strstr((char*)data, "<methodName>");
		char* pos2 = strstr((char*)data, "</methodName>");
		if (pos2 > pos)
		{

		  strncpy (methodName, pos+12, pos2-pos-12);
		  methodName[pos2-pos-12] = 0;
		}
	}

	os_printf("name:%s\n",methodName);

	os_printf("Strlen:%d\n",strlen(data));

	/*for (int i=0; i<strlen(data)+200; i=i+127)
	{
		char temp[128];
		if (strlen(data+i) >=127)
		strncpy(temp, data+i,127);
		else
			strcpy(temp, data+i);
		temp[127] = 0;
		os_printf(temp);
	}
	os_printf("\n");*/

	if (!strcmp(methodName, "requestTopic"))
	{
		char* pos = strstr(data, "<i4>");

		char* pos2 = strstr(data, "</i4>");
		os_printf("pos:%d, pos2:%d\n", pos, pos2);

		if (pos < pos2)
		{
			char portStr[pos2-pos-3];
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
	else if (!strcmp(methodName, "publisherUpdate"))
	{
		//static char data[] = "<?xml version='1.0'?><methodCall><methodName>publisherUpdate</methodName><params><param><value><string>/master</string></value></param><param><value><string>/chatter</string></value></param><param><value><array><data><value><string>http://10.3.84.99:40000</string></value></data></array></value></param></params></methodCall>";

		char* pos = strstr(data, "<value><string>/master</string></value>");
		if (pos != 0)
		{
			//strncpy(text, pos, 99);
			//os_printf("Text: %s\n", text);
			while(1)
			{
				char* pos2 = strstr((char*)pos, "<value><string>");
				char* pos3 = strstr((char*)pos2, "</string></value>");
				os_printf("_pos:%d, _pos2:%d\n", pos2, pos3);
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
				}
				pos = pos3;
			}
		}
		else
			os_printf("pos is NULL\n");

	}
}


void XMLRPCServer::start()
{
	HTTPServer* server = new HTTPServer("HTTPServer", XMLRPC_PORT, XMLRPCServerReceiveCallback);
	xTaskCreate(UDPSend, (const signed char*)"UDPSend", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
	isUDPReceiveTaskCreated = false;

	xTaskCreate(UDPreceive, (const signed char*)"UDPReceive", 256, NULL, tskIDLE_PRIORITY + 3, NULL);

}

TopicWriter* XMLRPCServer::registerPublisher(const char* callerID, const char* topic, const char* msgType)
{
	static uint16_t lastIndex = 0;
	TopicWriter* tw = new TopicWriter(callerID, topic, msgType);
	topicWriters[lastIndex++] = tw;
	return tw;
}

TopicReader* XMLRPCServer::registerSubscriber(const char* callerID, const char* topic, const char* msgType)
{
	static uint16_t lastIndex = 0;
	TopicReader* tr = new TopicReader(callerID, topic, msgType);
	topicReaders[lastIndex++] = tr;
	return tr;
}

void XMLRPCServer::UDPreceive(void* params)
{
	struct netconn *conn;
	struct netbuf *buf;
	err_t err;

	// Initialize memory (in stack) for message.
	char message[60];
	// TODO: Replace this delay with a signal!
	vTaskDelay(6000);
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
				portTickType xLastWakeTime;
				// Initialize the xLastWakeTime variable with the current time.
				xLastWakeTime = xTaskGetTickCount();

				// Start periodic loop.
				while (1)
				{
					buf = netconn_recv(conn);
					if (buf!= NULL)
					{
						struct ip_addr* ip;
						uint16_t port;
						ip = buf->addr;
						port = buf->port;
						if(ip != NULL)
						os_printf("Received from %d:%d!\n", ip->addr, port);
						// Copy received data into message.
						uint16_t len = netbuf_len(buf);
						if (len>15)
						{
							netbuf_copy (buf, &message, len);
							uint32_t connectionID = *((uint32_t*)&message[0]);
							TopicReader* tr = getTopicReader(connectionID);
							if (tr != NULL)
							{
								tr->enqueueMessage(&message[8]);
								os_printf("ConnectionID: %d, topic:%s\n", connectionID, tr->getTopic());
								/*uint32_t length = *((uint32_t*)&message[12]);
								if (len> length+15)
								{
									message[16+length] = 0;
									os_printf("%s\n", &message[16]);
								}*/
							}
						}
						// Deallocate previously created memory.
						netbuf_delete(buf);
					}
					else
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
			conn = netconn_new(NETCONN_UDP);
		}
		// If connection failed, wait for 50 ms before retrying.
		vTaskDelay(50);
	}
}

void XMLRPCServer::extractURI(const char* uri, char* ip, uint16_t* port)
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

void XMLRPCServer::sendRequest(const char* data, uint16_t port, void(*receiveCallback)(const void* obj, const char* data), void* obj)
{
	HTTPClient::instance()->sendData(data, port, receiveCallback, obj);
}
