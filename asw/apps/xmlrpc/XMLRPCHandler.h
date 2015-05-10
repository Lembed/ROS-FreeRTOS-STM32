#include <stdio.h>
#include <string.h>
#include "tcp.h"
#include <lwip/sockets.h>
#define SENDER_PORT_NUM 47855
#define SENDER_IP_ADDR "10.3.84.99"

#define SERVER_PORT_NUM 11311
#define SERVER_IP_ADDRESS "10.3.84.100"
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

	static void registerPublisher(const char* callerID, const char* topic, const char* msgType, const char* uri)
	{
		char data[512];
		static char firstPart[] = "<?xml version=\"1.0\"?> <methodCall> <methodName>registerPublisher</methodName> <params> <param> <value>/";
		static char secondPart[] = "</value> </param> <param> <value>";
		static char thirdPart[] = "</value><param> <value>";
		static char finalPart[] = "</value> </param> </param> </params> </methodCall>";

		strcpy(data, firstPart);
		strcat(data, callerID);
		strcat(data, secondPart);
		strcat(data, topic);
		strcat(data, thirdPart);
		strcat(data, msgType);
		strcat(data, secondPart);
		strcat(data, uri);
		strcat(data, finalPart);

		char header[256];
		createHeader("SI-Z0M81:11311", strlen(data), header);
		char xmlrpc[768];
		strcpy(xmlrpc, header);
		strcat(xmlrpc, data);

		/*for(int i=0; i< strlen(xmlrpc); i++)
		{
			os_printf("%c", xmlrpc[i]);
		}
		os_printf("\n");*/

		makeRequest(xmlrpc);


	}

	static void registerSubscriber(const char* callerID, const char* topic, const char* msgType, const char* uri)
	{
		static char data[512];
		for (int i=0; i< sizeof(data); i++) data[i] = 0;
		static char firstPart[] = "<?xml version=\"1.0\"?> <methodCall> <methodName>registerSubscriber</methodName> <params> <param> <value>/";
		static char secondPart[] = "</value> </param> <param> <value>";
		static char thirdPart[] = "</value> <param> <value>";
		static char finalPart[] = "</value> </param> </param> </params> </methodCall>";

		strcpy(data, firstPart);
		strcat(data, callerID);
		strcat(data, secondPart);
		strcat(data, topic);
		strcat(data, thirdPart);
		strcat(data, msgType);
		strcat(data, secondPart);
		strcat(data, uri);
		strcat(data, finalPart);

		static char header[256];
		//for (int i=0; i< sizeof(header); i++) header[i] = 0;
		createHeader("SI-Z0M81:11311", strlen(data), header);
		static char xmlrpc[768];
		//for (int i=0; i< sizeof(xmlrpc); i++) xmlrpc[i] = 0;
		strcpy(xmlrpc, header);
		strcat(xmlrpc, data);

		/*for(int i=0; i< strlen(xmlrpc); i++)
		{
			os_printf("%c", xmlrpc[i]);
		}
		os_printf("\n");*/

		makeRequest(xmlrpc);


	}

	static void makeRequest(const char* req)
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
	    sa.sin_port = htons(SENDER_PORT_NUM);


	    /* Bind the TCP socket to the port SENDER_PORT_NUM and to the current
	    * machines IP address (Its defined by SENDER_IP_ADDR).
	    * Once bind is successful for UDP sockets application can operate
	    * on the socket descriptor for sending or receiving data.
	    */
	    if (bind(socket_fd, (struct sockaddr *)&sa, sizeof(struct sockaddr_in)) == -1)
	    {
	    os_printf("Bind to Port Number %d ,IP address %s failed\n",SENDER_PORT_NUM,SENDER_IP_ADDR);
	    close(socket_fd);

	    }
	    // Receiver connects to server ip-address.

	    memset(&ra, 0, sizeof(struct sockaddr_in));
	    ra.sin_family = AF_INET;
	    ra.sin_addr.s_addr = inet_addr(SERVER_IP_ADDRESS);
	    ra.sin_port = htons(SERVER_PORT_NUM);


	    if(connect(socket_fd,(const sockaddr*)&ra,sizeof(struct sockaddr_in)) < 0)
	    {

	        os_printf("connect failed \n");
	        close(socket_fd);

	    }

	    send(socket_fd, req, strlen(req), 0);

	    recv_data = recv(socket_fd,data_buffer, sizeof(data_buffer),0);
	    if(recv_data < 0)
	    {

	        os_printf("recv failed \n");
	        close(socket_fd);

	    }
	    data_buffer[recv_data] = '\0';

		for(int i=0; i< strlen(data_buffer); i++)
		{
			os_printf("%c", data_buffer[i]);
		}
		os_printf("\n");

	    close(socket_fd);

	}
	static void close_conn(struct tcp_pcb *pcb){
	      tcp_arg(pcb, NULL);
	      tcp_sent(pcb, NULL);
	      tcp_recv(pcb, NULL);
	      tcp_close(pcb);
	}
	static err_t echo_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err){
		  static char data[1024];
		  static char buffer[1024];
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
	            for (i=0; i<len; i++)
	            	data[i]= pc[i];
	    		for(int i=0; i< sizeof(data); i++)
	    		{
	    			os_printf("%c", data[i]);
	    		}
	    		os_printf("\n");

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
	    					  os_printf("topic:%s\n", topic);
	    	            }
	    	            static char xml[300];
	    	            for (int i=0; i< sizeof(xml); i++) xml[i] = 0;

	    	            strcpy(xml, "<?xml version=\"1.0\"?><methodResponse><params><param><value><array><data><value><i4>1</i4></value><value></value><value><array><data><value>TCPROS</value><value>10.3.84.99</value><value><i4>");

	    	            static char portStr[16];
	    	            uint16_t port = 50000;
	    	    		sprintf(portStr, "%d", port);
	    	    		strcat(xml, portStr);
	    	    		strcat(xml, "</i4></value></data></array></value></data></array></value></param></params></methodResponse>");

	    	    		strcpy(buffer, "HTTP/1.0 200 OK\nServer: BaseHTTP/0.3 Python/2.7.6\nDate: Sat, 09 May 2015 21:53:33 GMT\nContent-type: text/xml\nContent-length: ");
	    	            static char contentLen[16];
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
	    	    }
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

	      tcp_setprio(pcb, TCP_PRIO_MIN);
	      tcp_recv(pcb, echo_recv);
	      tcp_err(pcb, NULL); //Don't care about error here
	      tcp_poll(pcb, NULL, 4); //No polling here
	      return ERR_OK;
	}


	static void mytcp(void* p){
	      struct tcp_pcb *pcb = tcp_new();
	      tcp_bind(pcb, IP_ADDR_ANY, *((uint16_t*)p));
	      while(1)
	      {
	      pcb = tcp_listen(pcb);
	      tcp_accept(pcb, echo_accept);
	      }

	      os_printf("XMLRPC response sent. Deleting task!\n");
	      vTaskDelete(NULL);
	}

	static void waitForRequest(uint16_t port)
	{
		xTaskCreate(mytcp, (const signed char*)"tcpserver", 2048, &port, tskIDLE_PRIORITY + 2, NULL);
	}
};
