#include "ultrasonic_sensor.h"
#include "rcl.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SR04_TRIG   (1<<6)
#define SR04_ECHO   (1<<7)
#define STM32_TICKS_PER_US          168
#define STM32_DELAY_US_MULT         (STM32_TICKS_PER_US/3)

/**
 * @brief Delay the given number of microseconds.
 *
 * @param us Number of microseconds to delay.
 */
static inline void delay_us(uint32_t us) {
    us *= STM32_DELAY_US_MULT;

    /* fudge for function call overhead  */
    //us--;
    asm volatile("   mov r0, %[us]          \n\t"
                 "1: subs r0, #1            \n\t"
                 "   bhi 1b                 \n\t"
                 :
                 : [us] "r" (us)
                 : "r0");
}

/*void delay_us(uint32_t us)
{
    us *= 3;
    while(us--)
    {
        __NOP();
    }
}*/





/*
  Configure SR04 GPIO
 */
void SR04_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  // configuring clock sources for GPIOC
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* Configure SR04 pins: PC6 - TRIGGER */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure SR04 pins: PC7 - ECHO */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // hold TRIG pin low
  GPIO_ResetBits(GPIOC, SR04_TRIG);
}
#define HCSR04_MAX_RANGE 200
#define HCSR04_TIMEOUT 50000
#define HCSR04_NUMBER ((float)0.0171821)
#define HCSR04_MAX_ECHO_DURATION HCSR04_MAX_RANGE * 59
float SR04_Read()
{
	taskDISABLE_INTERRUPTS();
	uint32_t time, timeout;
	/* Trigger low */
	GPIOC->BSRRH = SR04_TRIG;
	/* Delay 2 us */
	delay_us(2);
	/* Trigger high for 10us */
	GPIOC->BSRRL = SR04_TRIG;
	/* Delay 10 us */
	delay_us(10);
	/* Trigger low */
	GPIOC->BSRRH = SR04_TRIG;


	/* Give some time for response
	timeout = HCSR04_TIMEOUT;
	while (GPIO_ReadInputDataBit(GPIOC, SR04_ECHO)) {
		if (timeout-- == 0x00) {
			return -1;
		}
	}*/
	/* Give some time for response */
	timeout = HCSR04_TIMEOUT;
	while (!GPIO_ReadInputDataBit(GPIOC, SR04_ECHO)) {
		if (timeout-- == 0x00) {
			return 0;
		}
	}
	/* Start time */
	time = 0;
	/* Wait till signal is low */
	while (GPIO_ReadInputDataBit(GPIOC, SR04_ECHO)) {
		/* Increase time and check for max duration */
		if (time++ > 10000)
			return HCSR04_MAX_RANGE;
		/* Delay 1us */
		delay_us(1);

	}
	taskENABLE_INTERRUPTS();
	/* Convert us to cm */
	float distance =  (float)time * HCSR04_NUMBER;

	if (distance > HCSR04_MAX_RANGE)
		return HCSR04_MAX_RANGE;

	/* Return distance */
	return distance;
}

void loop()
{
	long distance_cm;

	distance_cm = SR04_Read();

	if (distance_cm > -1)
    os_printf("Distance: %d cm", distance_cm);
}
void loop1()
{
	/*if (GPIO_ReadInputDataBit(GPIOC, SR04_ECHO))
	{
		os_printf("Trig low");
	}
	else
	{
		os_printf("Trig high");
	}*/
	//GPIOC->BSRRL = 1<<6;
	GPIOC->ODR ^= 1<<6;
}
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

void ultrasonic_sensor(void* params)
{
	// Periodic code goes here.
	// First argument: Period.
	// Second argument periodic code.
	// Note: A periodic loop must always be included.
	Node* node = createNode("nodeD");
	Publisher *square_pub = createPublisher(node, "squared", RCL_MSG_TYPE_FLOAT);
	SR04_Init();
	LOOP(200,

	float distance_cm = SR04_Read();
	//if (distance_cm > -1 && distance_cm < 200)
	//os_printf("Distance: %d cm\n", (long)distance_cm);

	if (distance_cm > -1)
	{
    //os_printf("Distance: %d cm\n", (long)distance_cm);
    *((float*)square_pub->msg->data) = distance_cm;
    publish(square_pub);
	}
	// start while
	//os_printf("Distance:\t%d\tcm", SR04read());
	// end while
	)
}
