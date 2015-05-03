#include "err.h"
#include "api.h"
#include "lwip/ip_addr.h"
#include <string.h>
#include "tcp.h"
#define TCP_PORT 11311
char req[] = "POST / HTTP/1.1\nUser-Agent: curl/7.35.0\nHost: SI-Z0M81:11311\Accept: */*\nContent-Length: 296\nContent-Type: application/x-www-form-urlencoded\n\n<?xml version=\"1.0\"?> <methodCall> <methodName>registerSubscriber</methodName> <params> <param> <value>/script</value> </param> <param> <value>mytopic</value> <param> <value>std_msgs/String</value> </param> <param> <value>http://10.3.84.99:11311/</value> </param> </param> </params> </methodCall>";

/*static void EchoRequest( struct netconn *pxNetCon ) {
        struct netbuf *pxRxBuffer;
        portCHAR *pcRxString;
        unsigned portSHORT usLength;

        pxRxBuffer = netconn_recv( pxNetCon );< /FONT >
        if ( pxRxBuffer != NULL ){
                netbuf_data( pxRxBuffer, ( void * ) &pcRxString, &usLength );
                if (  pcRxString != NULL){
                        netconn_write( pxNetCon, pcRxString, (u16_t) usLength, NETCONN_COPY );
                }
        netbuf_delete( pxRxBuffer );
        }
}

portTASK_FUNCTION( vBasicTFTPServer, pvParameters ){
        struct netconn *pxTCPListener, *pxNewConnection;
        pxTCPListener = netconn_new( NETCONN_TCP );

        netconn_bind(pxTCPListener, NULL, 23 );
        netconn_listen( pxTCPListener );

        for( ;; ){
                pxNewConnection = netconn_accept(pxTCPListener);
                if(pxNewConnection != NULL){
                        EchoRequest(pxNewConnection);
                }
        }

}*/

#include <lwip/sockets.h>

#define SENDER_PORT_NUM 11310
#define SENDER_IP_ADDR "10.3.84.99"

#define SERVER_PORT_NUM 11311
#define SERVER_IP_ADDRESS "10.3.84.100"

char data_buffer[1000];
char temp[128];
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


    if(connect(socket_fd,(struct sockaddr_in*)&ra,sizeof(struct sockaddr_in)) < 0)
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
    //os_printf("%127s",data_buffer);
    os_printf("Len:%d\n",recv_data);
    int i = recv_data;
    while(i>0)
    {

    	strcpy(temp, &data_buffer[recv_data-i]);
    	temp[127] = '\0';
    	os_printf("%s", temp);
    	i = i-120;
    }
    close(socket_fd);

}

void send_xmlrpc()
{
	struct netconn *conn, *newconn;
		  err_t err;
		  // Create a new connection identifier.
		  conn = netconn_new(NETCONN_TCP);
		  struct ip_addr local_ip;
		  struct ip_addr remote_ip;
		  os_printf("connecting...");
		  if (conn!=NULL) {
			  u16_t port;
			  netconn_getaddr(conn, &local_ip, &port, 1);
			// Bind connection to port number TCP_PORT.
			err = netconn_bind(conn, &local_ip, TCP_PORT);
			os_printf("binding...");
			if (err == ERR_OK) {
				//IP4_ADDR(&remote_ip, 100, 84, 3, 10);
				IP4_ADDR(&remote_ip, 10, 3, 84, 100);
			  // Connect to remote ip
				os_printf("connecting2...");
			  err = netconn_connect(conn, &remote_ip, TCP_PORT);
			  if (err == ERR_OK)
			  {
				  char data[256];
				  u16_t usLength;
				  os_printf("writing...");
				  netconn_write(conn, req, strlen(req), NETCONN_NOCOPY);

				  for (; ;)
				  {
				  struct netbuf *buf = netconn_recv(conn);
			        if ( buf != NULL ){
			        	/*
			                netbuf_data( pxRxBuffer, ( void * ) &data, &usLength );
			                if (  data != NULL){
			                	os_printf("%s", data);
			                }*/
			        	  do {
			        	      //char *data;
			        	      int len;

			        	      /* obtain a pointer to the data in the fragment */
			        	      netbuf_data(buf, &data, &len);
			        	      os_printf("%s", data);
			        	      /* do something with the data */
			        	  } while(netbuf_next(buf) >= 0);
			        netbuf_delete( buf );
			        }
			        vTaskDelay(5);
				  }


			  }
			  // Close the connection (server closes in HTTP)
			  netconn_close(conn);
			  netconn_delete (conn);
			}
		  }
}
