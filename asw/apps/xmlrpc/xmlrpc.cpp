#include "err.h"
#include "api.h"
#include "lwip/ip_addr.h"
#include <string.h>
#include "tcp.h"
#define TCP_PORT 11311


/*
 * Notes:
 * registerPublisher(caller_id, topic, topic_type, caller_api) for registering publisher
 * -> all subscribers to this topic are then notified and each of them calls requestTopic(caller_id, topic, protocols)
 * As a response, each receives the protocol type (e.g. TCPROS), the ip address, and the port.
 * Then they connect to the 'TCP server' of the publisher.
 *
 * Problem: requestTopic requires STM32 to parse something like this:
 * <?xml version="1.0"?> <methodCall> <methodName>requestTopic</methodName> <params> <param>
 * <value>/listener</value> </param> <param> <value>chatter</value> <param> <value>
 * <array><data><value><array><data><value><string>TCPROS/</string></value></data>
 * </array></value></data></array></value> </param> </params> </methodCall>
 * Solution: get the first string after <methodName> until < is found. ->requestTopic (then same for the first two values)
 */



char req[] = "POST / HTTP/1.1\nUser-Agent: curl/7.35.0\nHost: SI-Z0M81:11311\Accept: */*\nContent-Length: 316\nContent-Type: application/x-www-form-urlencoded\n\n<?xml version=\"1.0\"?> <methodCall> <methodName>registerPublisher</methodName> <params> <param> <value>/rostopic_4767_1316912741557</value> </param> <param> <value>chatter</value> <param> <value>std_msgs/String</value> </param> <param> <value>http://10.3.84.99:47855/</value> </param> </param> </params> </methodCall>";

#include <lwip/sockets.h>

#define SENDER_PORT_NUM 47855
#define SENDER_IP_ADDR "10.3.84.99"

#define SERVER_PORT_NUM 11311
#define SERVER_IP_ADDRESS "10.3.84.100"

char data_buffer[1000];
char temp[100];
#include "xmlrpc.h"

extern "C" void os_printf(char* fmt, ...);

void test_main(void)
{

    int socket_fd;
    struct sockaddr_in sa,ra;

    int recv_data;  /* Creates an TCP socket (SOCK_STREAM) with Internet Protocol Family (PF_INET).
     * Protocol family and Address family related. For example PF_INET Protocol Family and AF_INET family are coupled.
    */

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
    /* Receiver connects to server ip-address. */

    memset(&ra, 0, sizeof(struct sockaddr_in));
    ra.sin_family = AF_INET;
    ra.sin_addr.s_addr = inet_addr(SERVER_IP_ADDRESS);
    ra.sin_port = htons(SERVER_PORT_NUM);


    if(connect(socket_fd,(const sockaddr*)&ra,sizeof(struct sockaddr_in)) < 0)
    {

        os_printf("connect failed \n");
        close(socket_fd);

    }

    send(socket_fd, req, sizeof(req), 0);

    recv_data = recv(socket_fd,data_buffer,sizeof(data_buffer),0);
    if(recv_data < 0)
    {

        os_printf("recv failed \n");
        close(socket_fd);

    }
    data_buffer[recv_data] = '\0';

    data_buffer[127] = '\0';
    os_printf("%s",data_buffer);

    data_buffer[127+128] = '\0';
    os_printf("%s",&data_buffer[128]);

    os_printf("\n");
    //os_printf("Len:%d\n",recv_data);
    /*int i = recv_data;
    while(i>0)
    {

    	strcpy(temp, &data_buffer[recv_data-i]);
    	temp[99] = '\0';
    	os_printf("%s", temp);
    	i = i-95;
    }*/
    close(socket_fd);

}


char buffer[1024];
void server(){
int lSocket;
struct sockaddr_in sLocalAddr;

lSocket = lwip_socket(AF_INET, SOCK_STREAM, 0);
if (lSocket < 0) return;

memset((char *)&sLocalAddr, 0, sizeof(sLocalAddr));
sLocalAddr.sin_family = AF_INET;
sLocalAddr.sin_len = sizeof(sLocalAddr);
sLocalAddr.sin_addr.s_addr = htonl(INADDR_ANY);
sLocalAddr.sin_port = SENDER_PORT_NUM;

if (lwip_bind(lSocket, (struct sockaddr *)&sLocalAddr, sizeof(sLocalAddr)) < 0) {
        lwip_close(lSocket);
        os_printf("Cannot bind!\n");
        return;
}

if ( lwip_listen(lSocket, 20) != 0 ){
        lwip_close(lSocket);
        os_printf("Cannot listen!\n");
        return;
}

while (1) {
		os_printf("While..\n");
        int clientfd;
        struct sockaddr_in client_addr;
        int addrlen=sizeof(client_addr);

        int nbytes;

        clientfd = lwip_accept(lSocket, (struct sockaddr*)&client_addr, (socklen_t*)&addrlen);
        if (clientfd>0){
        	os_printf("Accept..\n");
            do{
                nbytes=lwip_recv(clientfd, buffer, sizeof(buffer),0);
                buffer[126] = '\0';
                os_printf("%s\n", buffer);
                if (nbytes>0) lwip_send(clientfd, buffer, nbytes, 0);
            }  while (nbytes>0);

             lwip_close(clientfd);
          }
        else
        	os_printf("No Accept..\n");
        vTaskDelay(5);
    }
    lwip_close(lSocket);
}


#include "lwip/opt.h"

#include "lwip/sys.h"
#include "lwip/api.h"

char tempbuf[128];
char xmlbuffer[512];
char xmlhttpbuffer[512];
/*-----------------------------------------------------------------------------------*/
void tcpecho_thread()
{
  struct netconn *conn, *newconn;
  err_t err;


  /* Create a new connection identifier. */
  conn = netconn_new(NETCONN_TCP);

  if (conn!=NULL) {
    /* Bind connection to well known port number 7. */
    err = netconn_bind(conn, NULL, SENDER_PORT_NUM);

    if (err == ERR_OK) {
      /* Tell connection to go into listening mode. */
      netconn_listen(conn);

      while (1) {
    	  //os_printf("Grab new connection\n");
        /* Grab new connection. */
        newconn = netconn_accept(conn);
        os_printf("Accept..\n");

        /* Process the new connection. */
        if (newconn) {
          struct netbuf *buf;
          void *data;
          u16_t len;

          while ((buf = netconn_recv(newconn)) != NULL) {
            do {
              netbuf_data(buf, &data, &len);

              /*    POST / HTTP/1.1
					User-Agent: XMLRPC++ 0.7
					Host: 10.3.84.99:47855
					Content-Type: text/xml
					Content-="1.0"?>
					<methodCall><methodName>requestTopic</methodName>
					<params><param><value>/listener</valueter</value></param><param>
					<value><array><data><value><array><data><value>TCPROS</value></data>
					</arrvalue></param></params></methodCall>
               */

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
                      int publisherPort = 50000;
                      sprintf(xmlbuffer, "<?xml version=\"1.0\"?><methodResponse><params><param><value><array><data><value><i4>1</i4></value><value></value><value><array><data><value>TCPROS</value><value>%s</value><value><i4>%d</i4></value></data></array></value></data></array></value></param></params></methodResponse>", "10.3.84.100", publisherPort); //SENDER_IP_ADDR, publisherPort);
                      struct ip_addr addr;
                      u16_t port;
                      netconn_getaddr 	(newconn,
                      		&addr,
                      		&port,
                      		0
                      	);
                      os_printf("addr: %08x, port:%d\n", (u32_t)addr.addr, port);

                      sprintf(xmlhttpbuffer, "POST / HTTP/1.1\nUser-Agent: curl/7.35.0\nHost: 10.3.84.100:%d\Accept: */*\nContent-Length: %d\nContent-Type: application/x-www-form-urlencoded\n\n%s", port, strlen(xmlbuffer), xmlbuffer);
                      netconn_write(newconn, xmlhttpbuffer, strlen(xmlhttpbuffer) , NETCONN_COPY);
            	  }
            	  else
            		  os_printf("Not equal %s!\n", methodName);
              }
              /*for (int i=0;i*127 < len;i++)
              {
            	  memcpy(tempbuf, &((char*)data)[i*127], 127);
				  tempbuf[127] = '\0';
				  os_printf(tempbuf);
              }*/
              //os_printf("%s\n", tempbuf);
              //netconn_write(newconn, data, len, NETCONN_COPY);

            } while (netbuf_next(buf) >= 0);

            netbuf_delete(buf);
          }

          /* Close connection and discard connection identifier. */
          netconn_close(newconn);
          netconn_delete(newconn);
        }
      }
    } else {
      os_printf(" can not bind TCP netconn\n");
    }
  } else {
	  os_printf("can not create TCP netconn\n");
  }
}


void xmlrpc_task(void* p)
{


	//server();
	test_main();
	vTaskDelay(1000);
	for (;;)
	{
		tcpecho_thread();
		vTaskDelay(100);



	}
}

