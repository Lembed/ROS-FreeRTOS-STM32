#include "ros.h"
#include "rcl.h"
#include <stdarg.h>
#include <stdio.h>

// application memory
char memory[20 * 1024];
unsigned int offset = 0;


void* os_malloc(unsigned int size)
{
	unsigned int index = offset;
	taskENTER_CRITICAL();
	offset += size;
	taskEXIT_CRITICAL();
	return &memory[index];
}

#define MAX_CHARS 128
#define TERMINAL_QUEUE_LEN 10
#define TERMINAL_QUEUE_TIMEOUT 10

xQueueHandle terminalQueue;

//#define USART_TERMINAL 1

#ifdef USART_TERMINAL
void TerminalTask( void *pvParameters )
{
	terminalQueue = xQueueCreate(TERMINAL_QUEUE_LEN, MAX_CHARS);
	// Initialize memory (in stack) for message.

	static unsigned char string[MAX_CHARS];
	// Try to receive message, put the task to sleep for at most RXTIMEOUT ticks if queue is empty.
	for(;;)
	{
		if (xQueueReceive(terminalQueue, string, TERMINAL_QUEUE_TIMEOUT))
		{
		    taskDISABLE_INTERRUPTS();
		    USART_puts(USART1, (volatile char*)string);
		    taskENABLE_INTERRUPTS();
		}
	}
}
#else
#include "netconf.h"
#include "tcpip.h"

#define LOG_LOCAL_PORT 32005
#define LOG_REMOTE_PORT 32006

void TerminalTask(void* params)
{
	terminalQueue = xQueueCreate(TERMINAL_QUEUE_LEN, MAX_CHARS);

	char msg[MAX_CHARS];
	struct netconn* conn = netconn_new( NETCONN_UDP );
    netconn_bind(conn, IP_ADDR_ANY, LOG_LOCAL_PORT);
    struct ip_addr ip;
    ip.addr = inet_addr("10.3.84.100");
    netconn_connect(conn, &ip, LOG_REMOTE_PORT);
	for(;;)
	{
			// Try to receive message, block the task for at most TERMINAL_QUEUE_TIMEOUT ticks if queue is empty.
			if (xQueueReceive(terminalQueue, &msg, TERMINAL_QUEUE_TIMEOUT))
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
#endif // USART_TERMINAL

#include "USARTHandler.h"
void os_printf(const char* fmt, ...)
{
    va_list ap;
    char string[MAX_CHARS];

    va_start(ap, fmt);
    vsprintf(string, fmt, ap);
    va_end(ap);
#ifdef USART_TERMINAL
    if (strlen(string) > 2 && strlen(string) < 128 && string[strlen(string)-1] == '\n' && string[strlen(string)-2] != '\r')
    {
    	string[strlen(string)-1] = '\r';
    	string[strlen(string)] = '\n';
    }
#endif// USART_TERMINAL

    if (terminalQueue != NULL)
    {
    	xQueueSend(terminalQueue, &string, 0);
    }
    /*
    taskDISABLE_INTERRUPTS();
    USART_puts(USART1, (volatile char*)string);
    taskENABLE_INTERRUPTS();*/
    //tr_log(string);
}




