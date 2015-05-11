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
class XMLRPCHandler
{
public:
	static void createHeader(const char* hostURI, int contentLength, char* data)
	{
		if (data != NULL)
		{

			strcpy(data, "POST / HTTP/1.1\nUser-Agent: curl/7.35.0\nHost: ");
			strcat(data, hostURI);
			strcat(data, "\\Accept: */*\nContent-Length: ");
			static char contentLen[16];
			sprintf(contentLen, "%d", contentLength);
			strcat(data, contentLen);
			strcat(data, "\nContent-Type: application/x-www-form-urlencoded\n\n");
		}
	}

	static void registerPublisher(const char* callerID, const char* topic, const char* msgType)
	{
		char data[512];

		static uint16_t topicIndex = 0;
		strcpy(topics[topicIndex], "/");
		strcat(topics[topicIndex], topic);
		topicIndex++;

		strcpy(data, "<?xml version=\"1.0\"?> <methodCall> <methodName>registerPublisher</methodName> <params> <param> <value>/");
		strcat(data, callerID);
		strcat(data, "</value> </param> <param> <value>");
		strcat(data, topic);
		strcat(data, "</value> <param> <value>");
		strcat(data, msgType);
		strcat(data, "</value> </param> <param> <value>");
		char portStr[6];
		sprintf(portStr, "%d", XMLRPC_PORT);
		strcat(data, "http://10.3.84.99:");
		strcat(data, portStr);
		strcat(data, "</value> </param> </param> </params> </methodCall>");

		char header[256];
		createHeader("10.3.84.100:11311", strlen(data), header);
		char xmlrpc[768];
		strcpy(xmlrpc, header);
		strcat(xmlrpc, data);

		makeRequest(xmlrpc, XMLRPC_PORT, 11311);


	}

	static void connectPublisherPort(const char* data)
	{
		/* Example response:
		 * <?xml version="1.0"?>
<methodResponse><params><param>
	<value><array><data><value><i4>1</i4></value><value></value><value><array><data><value>TCPROS</value><value>10.3.84.100</value><value><i4>44973</i4></value></data></array></value></data></array></value>
</param></params></methodResponse>
		 *
		 */
		static const unsigned char connectionHeader[] =
		{0xb0, 0x00, 0x00, 0x00,
		   0x20, 0x00, 0x00, 0x00,
		      0x6d, 0x65, 0x73, 0x73, 0x61, 0x67, 0x65, 0x5f, 0x64, 0x65, 0x66, 0x69, 0x6e, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x3d, 0x73, 0x74, 0x72, 0x69, 0x6e, 0x67,
		      0x20, 0x64, 0x61, 0x74, 0x61, 0x0a, 0x0a,
		   0x25, 0x00, 0x00, 0x00,
		      0x63, 0x61, 0x6c, 0x6c, 0x65, 0x72, 0x69, 0x64, 0x3d, 0x2f, 0x72, 0x6f, 0x73, 0x74, 0x6f, 0x70, 0x69, 0x63, 0x5f, 0x34, 0x37, 0x36, 0x37, 0x5f, 0x31,
		      0x33, 0x31, 0x36, 0x39, 0x31, 0x32, 0x37, 0x34, 0x31, 0x35, 0x35, 0x37,
		   0x0a, 0x00, 0x00, 0x00,
		      0x6c, 0x61, 0x74, 0x63, 0x68, 0x69, 0x6e, 0x67, 0x3d, 0x31,
		   0x27, 0x00, 0x00, 0x00,
		      0x6d, 0x64, 0x35, 0x73, 0x75, 0x6d, 0x3d, 0x39, 0x39, 0x32, 0x63, 0x65, 0x38, 0x61, 0x31, 0x36, 0x38, 0x37, 0x63, 0x65, 0x63, 0x38, 0x63, 0x38, 0x62,
		      0x64, 0x38, 0x38, 0x33, 0x65, 0x63, 0x37, 0x33, 0x63, 0x61, 0x34, 0x31, 0x64, 0x31,
		   0x0e, 0x00, 0x00, 0x00,
		      0x74, 0x6f, 0x70, 0x69, 0x63, 0x3d, 0x2f, 0x63, 0x68, 0x61, 0x74, 0x74, 0x65, 0x72,
		   0x14, 0x00, 0x00, 0x00,
		      0x74, 0x79, 0x70, 0x65, 0x3d, 0x73, 0x74, 0x64, 0x5f, 0x6d, 0x73, 0x67, 0x73, 0x2f, 0x53, 0x74, 0x72, 0x69, 0x6e, 0x67
		};

		char* pos = strstr((char*)data, "<i4>");
	    char* pos2 = strstr(pos+4, "<i4>");
	    char* pos3 = strstr(pos2+4, "</i4>");

	    if (pos2 < pos3)
	    {
		  char port[pos3-pos2-5];
	  	  strncpy (port, pos2+4, pos3-pos2-4);
	  	  port[pos3-pos2-4] = 0;
		  uint16_t num = atoi(port);
		  os_printf("Port: %d\n", num);
			int socket_fd;
		    struct sockaddr_in sa,ra;
		    char data_buffer[128];

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
		    sa.sin_port = htons(30000);


		    /* Bind the TCP socket to the port SENDER_PORT_NUM and to the current
		    * machines IP address (Its defined by SENDER_IP_ADDR).
		    * Once bind is successful for UDP sockets application can operate
		    * on the socket descriptor for sending or receiving data.
		    */
		    if (bind(socket_fd, (struct sockaddr *)&sa, sizeof(struct sockaddr_in)) == -1)
		    {
		    os_printf("Bind to Port Number %d ,IP address %s failed\n", 30000, SENDER_IP_ADDR);
		    close(socket_fd);

		    }
		    // Receiver connects to server ip-address.

		    memset(&ra, 0, sizeof(struct sockaddr_in));
		    ra.sin_family = AF_INET;
		    ra.sin_addr.s_addr = inet_addr(SERVER_IP_ADDRESS);
		    ra.sin_port = htons(num);


		    if(connect(socket_fd,(const sockaddr*)&ra,sizeof(struct sockaddr_in)) < 0)
		    {

		        os_printf("connect failed \n");
		        close(socket_fd);

		    }
		    char string[] = "|";
		    send(socket_fd, (const char*)connectionHeader, strlen((const char*)connectionHeader), 0);

		    recv_data = recv(socket_fd,data_buffer, sizeof(data_buffer),0);
		    if(recv_data < 0)
		    {

		        os_printf("recv failed \n");
		        close(socket_fd);

		    }
		    data_buffer[recv_data] = '\0';

			/*for(int i=0; i< strlen(data_buffer); i++)
			{
				os_printf("%c", data_buffer[i]);
			}
			os_printf("\n");*/

		    close(socket_fd);

	    }
	}

	static void UDPreceive(void* params)
	{
		struct netconn *conn;
		static unsigned short port;
		struct netbuf *buf;
		err_t err;

		// Initialize memory (in stack) for message.
		char message[60];

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
							//os_printf("Receive!\n");
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


	static void registerSubscriber(const char* callerID, const char* topic, const char* msgType)
	{
		char data[512];

		static uint16_t topicIndex = 0;
		strcpy(topics[topicIndex], "/");
		strcat(topics[topicIndex], topic);
		topicIndex++;

		strcpy(data, "<?xml version=\"1.0\"?> <methodCall> <methodName>registerSubscriber</methodName> <params> <param> <value>/");
		strcat(data, callerID);
		strcat(data, "</value> </param> <param> <value>");
		strcat(data, topic);
		strcat(data, "</value> <param> <value>");
		strcat(data, msgType);
		strcat(data, "</value> </param> <param> <value>");
		char portStr[6];
		sprintf(portStr, "%d", XMLRPC_PORT);
		strcat(data, "http://10.3.84.99:");
		strcat(data, portStr);
		strcat(data, "</value> </param> </param> </params> </methodCall>");

		char header[256];
		createHeader("10.3.84.100:11311", strlen(data), header);
		char xmlrpc[768];
		strcpy(xmlrpc, header);
		strcat(xmlrpc, data);

		makeRequest(xmlrpc, XMLRPC_PORT, 11311);
		/*Example response:
		 * <?xml version='1.0'?>
<methodResponse>
<params>
<param>
<value><array><data>
<value><int>1</int></value>
<value><string>Subscribed to [/chatter]</string></value>
<value><array><data>
<value><string>http://10.3.84.99:40000</string></value>
<value><string>http://10.3.84.100:57800/</string></value>
</data></array></value>
</data></array></value>
</param>
</params>
</methodResponse>
		 *
		 */

		// TODO: Process the response or wait for the next updatePublishers xmlrpc call to get all publisher uri's for the topic
		// Then send a requestTopic xmlrpc to each publisher, get the tcp port number from the response
		// Then connect to the tcp port(s). if there are multiple tcp ports, create a task for each one.
		// Deserialize incoming data

		/*strcpy(data, "<?xml version=\"1.0\"?> <methodCall> <methodName>requestTopic</methodName> <params> <param> <value>/");
		strcat(data, callerID);
		strcat(data, "</value> </param> <param> <value>");
		strcat(data, topic);
		strcat(data, "</value> <param> <value><array><data><value><array><data><value><string>TCPROS</string></value>");
		strcat(data, "</data></array></value></data></array></value> </param> </params> </methodCall>");*/
		vTaskDelay(2000);
		strcpy(data,"<?xml version=\"1.0\"?>");
		strcat(data, "<methodCall><methodName>requestTopic</methodName>");
		strcat(data, "<params><param><value>/listener</value></param><param><value>/chatter</value></param><param><value><array><data><value><array><data><value>UDPROS</value><value><base64>EgAAAGNhbGxlcmlkPS9saXN0ZW5lcicAAABtZDVzdW09OTkyY2U4YTE2ODdjZWM4YzhiZDg4");
		strcat(data, "M2VjNzNjYTQxZDEOAAAAdG9waWM9L2NoYXR0ZXIUAAAAdHlwZT1zdGRfbXNncy9TdHJpbmc=</base64></value><value>10.3.84.99</value><value><i4>44100</i4></value><value><i4>1500</i4></value></data></array></value></data></array></value></param></params></methodCall>");

		/*strcpy(data, "<?xml version=\"1.0\"?> <methodCall> <methodName>requestTopic</methodName>");
		strcat(data, "<params> <param> <value>/rostopic</value> </param> <param> <value>chatter");
		strcat(data, "</value> </param> <param> <value><array><data><value><array><data><value><string>");
		strcat(data, "TCPROS</string></value></data></array></value></data></array></value>");
		strcat(data, "</param> </params> </methodCall>");*/
		createHeader("10.3.84.100:36464", strlen(data), header);
		strcpy(xmlrpc, header);
		strcat(xmlrpc, data);
		//makeRequest(xmlrpc, XMLRPC_PORT, 59513, connectPublisherPort);
		makeRequest(xmlrpc, XMLRPC_PORT, 36464, NULL);
		xTaskCreate(UDPreceive, (const signed char*)"myudp1", 128, NULL, tskIDLE_PRIORITY + 3, NULL);


	}
	static void makeRequest(const char* req, uint16_t port, uint16_t serverPort, void(*responseCallback)(const char* data) = NULL) // TODO: replace port with uri to be able to make request to multiple ip addresses
	{
		int socket_fd;
	    struct sockaddr_in sa,ra;
	    char data_buffer[1024];

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
	    // Receiver connects to server ip-address.

	    memset(&ra, 0, sizeof(struct sockaddr_in));
	    ra.sin_family = AF_INET;
	    ra.sin_addr.s_addr = inet_addr(SERVER_IP_ADDRESS);
	    ra.sin_port = htons(serverPort);


	    while(connect(socket_fd,(const sockaddr*)&ra,sizeof(struct sockaddr_in)) < 0)
	    {

	        os_printf("connect failed \n");
	        close(socket_fd);
	    }

	    send(socket_fd, req, strlen(req), 0);

	    recv_data = recv(socket_fd,data_buffer, sizeof(data_buffer),0);
	    while (recv_data < 0)
	    {

	        os_printf("recv failed \n");
	        close(socket_fd);

	    }
	    data_buffer[recv_data] = '\0';

		/*for(int i=0; i< strlen(data_buffer); i++)
		{
			os_printf("%c", data_buffer[i]);
		}
		os_printf("\n");*/

		if (responseCallback != NULL)
		{
			responseCallback(data_buffer);
		}

	    close(socket_fd);

	}
	static void close_conn(struct tcp_pcb *pcb){
	      tcp_arg(pcb, NULL);
	      tcp_sent(pcb, NULL);
	      tcp_recv(pcb, NULL);
	      tcp_close(pcb);
	}

	static void udptask(void* p)
	{
		uint16_t port = *((uint16_t*)p);

		struct netconn* conn = netconn_new( NETCONN_UDP );
	    netconn_bind(conn, IP_ADDR_ANY, 46552);
	    struct ip_addr ip;
	    ip.addr = inet_addr("10.3.84.100");
	    netconn_connect(conn, &ip, port);
	    char msg[] = {
	    		0x03, 0x00, 0x00, 0x00,
	    		0x00, 0x01, 0x01, 0x00,
				0x15, 0x00, 0x00, 0x00,
				0x11, 0x00, 0x00, 0x00,
				0x68, 0x65, 0x6c, 0x6c,
				0x6f, 0x20, 0x77, 0x6f,
				0x72, 0x6c, 0x64, 0x5f,
				0x20, 0x35, 0x38, 0x36,
				0x31
	    };


		for(;;)
		{
				// Try to receive message, block the task for at most UDPReceiveTimeout ticks if queue is empty.
				//if (xQueueReceive(vars->queueHandle, &msg, vars->queueReceiveTimeout > 0 ? vars->queueReceiveTimeout : 100))
				//if (xQueueReceive(LogQueueHandle, &msg, 100))
				{
					// Methods for UDP send.
					os_printf("Sending... %s\n", &msg[16]);
					struct netbuf *buf = netbuf_new();
					msg[5]++;
				    void* data = netbuf_alloc(buf, sizeof(msg)); // Also deallocated with netbuf_delete(buf)
				    memcpy (data, msg, sizeof (msg));
				    netconn_send(conn, buf);
				    netbuf_delete(buf); // Deallocate packet buffer

				    vTaskDelay(100);
				}

		}
	}


	static err_t echo_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err){
		  //char data[900];
		  char buffer[900];
	      int i;
	      int len;
	      char *pc;
	      if (err == ERR_OK && p != NULL) {
	            // Inform TCP that we have taken the data.
	            tcp_recved(pcb, p->tot_len);

	            //pointer to the pay load
	            pc=(char *)p->payload;

	            //size of the pay load
	            len =p->tot_len;
	            //copy to our own buffer
	            /*for (i=0; i<len; i++)
	            {
	            	data[i]= pc[i];
	            }*/

	    		//os_printf("Len:%d\n", len);
	    		/*for(int i=0; i< len; i++)
	    		{
	    			os_printf("%c", pc[i]);
	    		}*/
	    		/*for (int i=0; i<len; i= i+128)
	    		{
	    			pc[i+127] = 0;
	    			os_printf(&pc[i]);
	    		}


	    		os_printf("\n");*/
	    		//vTaskDelay(500);


	    		char* pos = strstr(pc+550, "<i4>");
	    	    char* pos2 = strstr(pc+550, "</i4>");

	    	    //os_printf("pos:%d, pos2:%d\n", pos, pos2);

	    	    if (pos < pos2)
	    	    {
	    		  char port[pos2-pos-5];
	    	  	  strncpy (port, pos+4, pos2-pos-4);
	    	  	  port[pos2-pos-4] = 0;
	    		  uint16_t num = atoi(port);
	    		  //os_printf("Port: %d\n",num);
	    		  xTaskCreate(udptask, (const signed char*)"myudp", 256, &port, tskIDLE_PRIORITY + 2, NULL);
	    	    }



	            static char xml[] = "<?xml version=\"1.0\"?><methodResponse><params><param><value><array><data><value><i4>1</i4></value><value></value><value><array><data><value>UDPROS</value><value>10.3.84.99</value><value><i4>46552</i4></value><value><i4>3</i4></value><value><i4>1500</i4></value><value><base64>EAAAAGNhbGxlcmlkPS90YWxrZXInAAAAbWQ1c3VtPTk5MmNlOGExNjg3Y2VjOGM4YmQ4ODNlYzczY2E0MWQxHwAAAG1lc3NhZ2VfZGVmaW5pdGlvbj1zdHJpbmcgZGF0YQoOAAAAdG9waWM9L2NoYXR0ZXIUAAAAdHlwZT1zdGRfbXNncy9TdHJpbmc=</base64></value></data></array></value></data></array></value></param></params></methodResponse>";
	    		strcpy(buffer, "HTTP/1.0 200 OK\nServer: BaseHTTP/0.3 Python/2.7.6\n");
	    		strcat(buffer, "Date: Sat, 09 May 2015 21:53:33 GMT\nContent-type: text/xml\nContent-length: ");
	            char contentLen[16];
	    		sprintf(contentLen, "%d", strlen(xml));
	    		strcat(buffer, contentLen);
	    		strcat(buffer, "\n\n");
	    		strcat(buffer, xml);

	    		//Free the packet buffer
	    		pbuf_free(p);
	            err = tcp_write(pcb, buffer, strlen(buffer), 0);
	            tcp_sent(pcb, NULL);


	    		/*
	    		uint16_t port = 0;
	    	    char* pos = strstr((char*)data, "<methodName>");
	    	    char* pos2 = strstr((char*)data, "</methodName>");
	    	    if (pos2 > pos)
	    	    {
	    			  char methodName[pos2-pos-12];
	    	  	  strncpy (methodName, pos+12, pos2-pos-12);
	    	  	  methodName[pos2-pos-12] = 0;
	    	  	  if (!strcmp(methodName, "requestTopic"))
	    	  	  {
	    	            char* pos = strstr((char*)data, "<value>");
	    	            char* pos2 = strstr((char*)data, "</value>");
	    	            if (pos2 >pos)
	    	            {
	    					  char caller_id[pos2-pos-7];
	    					  strncpy (caller_id, pos+7, pos2-pos-7);
	    					  caller_id[pos2-pos-7] = 0;

	    					  os_printf("caller_id:%s\n", caller_id);
	    	            }
	    	            char* pos3 = strstr((char*)pos2+7, "<value>");
	    	            char* pos4 = strstr((char*)pos2+7, "</value>");
	    	            if (pos4 > pos3)
	    	            {
	    					  char topic[pos4-pos3-7];
	    					  strncpy (topic, pos3+7, pos4-pos3-7);
	    					  topic[pos4-pos3-7] = 0;
	    					  for(uint16_t i=0; i<TOPIC_COUNT; i++)
	    					  {
	    						  if (!strcmp(topic, topics[i]))
	    						  {
	    							  port = 50000 + i;
	    							  break;
	    						  }
	    					  }

	    					  os_printf("topic:%s, port:%d\n", topic, port);
	    	            }
	    	            char xml[300];

	    	            strcpy(xml, "<?xml version=\"1.0\"?><methodResponse><params><param><value><array><data><value><i4>1</i4></value>");
	    	            strcat(xml, "<value></value><value><array><data><value>TCPROS</value><value>10.3.84.99</value><value><i4>");

	    	            char portStr[16];
	    	    		sprintf(portStr, "%d", port);
	    	    		strcat(xml, portStr);
	    	    		strcat(xml, "</i4></value></data></array></value></data></array></value></param></params></methodResponse>");

	    	    		strcpy(buffer, "HTTP/1.0 200 OK\nServer: BaseHTTP/0.3 Python/2.7.6\n");
	    	    		strcat(buffer, "Date: Sat, 09 May 2015 21:53:33 GMT\nContent-type: text/xml\nContent-length: ");
	    	            char contentLen[16];
	    	    		sprintf(contentLen, "%d", strlen(xml));
	    	    		strcat(buffer, contentLen);
	    	    		strcat(buffer, "\n\n");
	    	    		strcat(buffer, xml);

	    	    		//Free the packet buffer
	    	    		pbuf_free(p);
	    	            err = tcp_write(pcb, buffer, strlen(buffer), 0);
	    	            tcp_sent(pcb, NULL);

	    	  	  }
	    	  	  else
	    	  	  {
	    	  		  os_printf("Not equal %s!\n", methodName);
	    	  		  //Free the packet buffer
	    	  		  pbuf_free(p);

	    	  	  }
	    	    }*/
	    	    close_conn(pcb);


	      } else {
	            pbuf_free(p);
	      }

	      if (err == ERR_OK && p == NULL) {
	            close_conn(pcb);
	      }
	      return ERR_OK;
	}

	static err_t echo_accept(void *arg, struct tcp_pcb *pcb, err_t err){
		//os_printf("Accept!\n");
		tcp_setprio(pcb, TCP_PRIO_MIN);
	      tcp_recv(pcb, echo_recv);
	      tcp_err(pcb, NULL); //Don't care about error here
	      tcp_poll(pcb, NULL, 4); //No polling here
	      return ERR_OK;
	}


	static void mytcp(void* p){
	      struct tcp_pcb *pcb = tcp_new();
	      tcp_bind(pcb, IP_ADDR_ANY, XMLRPC_PORT);
	      while(1)
	      {
	      pcb = tcp_listen(pcb);
	      tcp_accept(pcb, echo_accept);
	      }

	      os_printf("XMLRPC response sent. Deleting task!\n");
	      vTaskDelete(NULL);
	}

	static void waitForRequest()
	{
		static int taskID = 1;
		char taskName[16];
		sprintf(taskName, "rpcServer_%d", taskID++);
		taskName[15] = 0;
		os_printf("Taskname: %s\n", taskName);
		xTaskCreate(mytcp, (const signed char*)taskName, 2048, NULL, tskIDLE_PRIORITY + 2, NULL);
	}
};
