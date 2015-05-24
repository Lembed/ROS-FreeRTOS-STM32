/*! Transport Layer */
#include "transport.h"
#include "netconf.h"
#include "tcpip.h"

#define UDP_RX_QUEUE_LEN 10
#define UDP_RX_QUEUE_MSG_SIZE 128
#define UDP_RX_PERIOD 1
#define UDP_RX_PORT 32002

#define UDP_TX_LOCAL_PORT 32000
#define UDP_TX_REMOTE_PORT 32001
#define LOG_LOCAL_PORT 32005
#define LOG_REMOTE_PORT 32006

xQueueHandle UDPTXQueueHandle = 0;
xQueueHandle UDPRXQueueHandle = 0;
xQueueHandle LogQueueHandle = 0;

void tr_log(const char* msg)
{
	// Initialize memory (in stack) for message.
	char msgCopy[UDP_RX_QUEUE_MSG_SIZE];
	// Copy message into the previously initialized memory, strcpy since we want to have NULL-terminated string.
	strcpy(msgCopy, msg);
	// Try to send message if queue is non-full.
	xQueueSend(LogQueueHandle, &msgCopy, 0);
}

typedef struct tr_vars_t
{
	unsigned int localPort, remotePort;
	xQueueHandle queueHandle;
	unsigned int maxMessageLength;
	unsigned int queueReceiveTimeout;
} tr_vars_t;

struct netconn* tr_connect(tr_vars_t* vars)
{
	struct netconn* conn = netconn_new( NETCONN_UDP );
    netconn_bind(conn, IP_ADDR_ANY, vars->localPort);
    netconn_connect(conn, IP_ADDR_BROADCAST, vars->remotePort);

    return conn;
}

void tr_UDPsend(void* params)
{
	tr_vars_t* vars = (tr_vars_t*) params;
	struct netconn* conn = tr_connect(vars);
    char msg[vars->maxMessageLength > 0 ? vars->maxMessageLength : 256];

	for(;;)
	{
			// Try to receive message, block the task for at most UDPReceiveTimeout ticks if queue is empty.
			if (xQueueReceive(vars->queueHandle, &msg, vars->queueReceiveTimeout > 0 ? vars->queueReceiveTimeout : 100))
			//if (xQueueReceive(LogQueueHandle, &msg, 100))
			{
				// Methods for UDP send.
				struct netbuf *buf = netbuf_new();
			    char * data = netbuf_alloc(buf, sizeof(msg)); // Also deallocated with netbuf_delete(buf)
			    memcpy (data, msg, sizeof (msg));
			    netconn_send(conn, buf);
			    netbuf_delete(buf); // Deallocate packet buffer
			}

	}
}

tr_vars_t shellVars, udpRXVars;
void tr_init() // a function of transport layer
{
	LogQueueHandle = xQueueCreate(UDP_RX_QUEUE_LEN, sizeof(char) * UDP_RX_QUEUE_MSG_SIZE);

	shellVars.localPort = LOG_LOCAL_PORT;
	shellVars.remotePort = LOG_REMOTE_PORT;
	shellVars.queueHandle = LogQueueHandle;
	shellVars.maxMessageLength = UDP_RX_QUEUE_MSG_SIZE;
	shellVars.queueReceiveTimeout = 60000;

	xTaskCreate(tr_UDPsend, "transport_Log", 512, (void*) &shellVars, tskIDLE_PRIORITY + 3, NULL);
}
