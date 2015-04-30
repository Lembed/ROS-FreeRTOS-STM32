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

void tr_publish(void* msg, void* topic) // a function of transport
{
	// UDP:
	// data = serialize(message, topic)
	// create_conn (retry if unsuccessful)
	// bind to port (retry if unsuccessful)
	// connect (retry if unsuccessful)

	// msg = wait_or_dequeue(tx_queue)
	// buf = create()
	// data = copy(buf, msg)
	// send_to(conn, buf)
	// delete(buf)

	// Initialize memory (in stack) for message.
	unsigned char data[UDP_RX_QUEUE_MSG_SIZE];
	// Copy message into the previously initialized memory.
	memcpy(data, msg, UDP_RX_QUEUE_MSG_SIZE);
	// Try to send message if queue is non-full.
	xQueueSend(UDPTXQueueHandle, &data, 0);
	// Send message for inter-task communication.
	xQueueSend(UDPRXQueueHandle, &data, 0);


	// DDS:
	// convert ros msg to dds idl
	// data_writer = get_data_writer(topic)
	// data_writer->register_instance()
	// data_writer->write(dds_idl)
}



void tr_UDPreceive(void* params)
{
	struct netconn *conn;
	static unsigned short port;
	struct netbuf *buf;
	err_t err;

	// Initialize memory (in stack) for message.
	char message[UDP_RX_QUEUE_MSG_SIZE];

	conn = netconn_new(NETCONN_UDP);

	for(;;)
	{
		// Check if connection was created successfully.
		if (conn!= NULL)
		{
			err = netconn_bind(conn, IP_ADDR_ANY, UDP_RX_PORT);
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
						// Copy received data into message.
						netbuf_copy (buf, &message, UDP_RX_QUEUE_MSG_SIZE);
						// Send message to UDPRX queue for RXTask to dequeue and process it.
						xQueueSend(UDPRXQueueHandle, &message, 0);
						// Deallocate previously created memory.
						netbuf_delete(buf);
					}
					// Use delay until to guarantee periodic execution of each loop iteration.
					vTaskDelayUntil(&xLastWakeTime, UDP_RX_PERIOD);
				}
		    }
		    else
		    {
		    	os_printf("cannot bind netconn");
		    }
		}
		else
		{
			os_printf("cannot create new UDP netconn");
		}
		// If connection failed, wait for 50 ms before retrying.
		vTaskDelay(50);
	}
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
	UDPRXQueueHandle = xQueueCreate(UDP_RX_QUEUE_LEN, sizeof(char) * UDP_RX_QUEUE_MSG_SIZE);
	UDPTXQueueHandle = xQueueCreate(UDP_RX_QUEUE_LEN, sizeof(char) * UDP_RX_QUEUE_MSG_SIZE);
	LogQueueHandle = xQueueCreate(UDP_RX_QUEUE_LEN, sizeof(char) * UDP_RX_QUEUE_MSG_SIZE);


	udpRXVars.localPort = UDP_TX_LOCAL_PORT;
	udpRXVars.remotePort = UDP_TX_REMOTE_PORT;
	udpRXVars.queueHandle = UDPTXQueueHandle;
	udpRXVars.maxMessageLength = UDP_RX_QUEUE_MSG_SIZE;
	udpRXVars.queueReceiveTimeout = 60000;

	shellVars.localPort = LOG_LOCAL_PORT;
	shellVars.remotePort = LOG_REMOTE_PORT;
	shellVars.queueHandle = LogQueueHandle;
	shellVars.maxMessageLength = UDP_RX_QUEUE_MSG_SIZE;
	shellVars.queueReceiveTimeout = 60000;



	xTaskCreate(tr_UDPsend, "transport_Log", 512, (void*) &shellVars, tskIDLE_PRIORITY + 3, NULL);
	xTaskCreate(tr_UDPsend, "transport_UDPSend", 512, (void*) &udpRXVars, tskIDLE_PRIORITY + 3, NULL);
	xTaskCreate(tr_UDPreceive, "transport_UDPReceive", 512, NULL, tskIDLE_PRIORITY + 5, NULL);
}
#define TAKE_MESSAGE_TIMEOUT 60000

void tr_take_msg(void* msg, void* topic) // a function of transport layer
{
	// UDP:
	// buf = recv_from(conn) // wait for the UDP queue
	// data = get_data(buf)
	// topic = get_topic(data)
	unsigned char message[UDP_RX_QUEUE_MSG_SIZE];
	for (; ;)
	{
		if (xQueueReceive(UDPRXQueueHandle, &message, TAKE_MESSAGE_TIMEOUT))
		{
			// Is this copy operation necessary? How to optimize?
			memcpy(msg, message, UDP_RX_QUEUE_MSG_SIZE);
			break;
		}
	}
	// DDS: Each topic would have a designated task for taking a DDS message.
	// Each task would then pass a <topic,msg_sequence> pair to this task.
	// msg_seq = data_reader->take(topic) // does take return a message with only a specific topic or can it return all messages?
}

void tr_extract_msg(void* topic, void* orig_msg, void* new_msg) //  function of transport layer
{
	//UDP:
	// raw -> ros msg using topic info
	new_msg = orig_msg; // replace this by a process_msg operation

	//DDS:
	// convert dds idl to ros msg
}



