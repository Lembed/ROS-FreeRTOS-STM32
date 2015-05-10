#include "err.h"
#include "api.h"
#include "lwip/ip_addr.h"
#include <string.h>
#include "tcp.h"

#define TX_QUEUE_LEN 10
#define TX_QUEUE_MSG_SIZE 128
#define TXTimeout 100

class ConnectionHandler
{
private:
	xQueueHandle txQueueHandle;
	bool isConnectionInitialized;
	uint16_t port;
	char msg[ TX_QUEUE_MSG_SIZE];


	static void close_conn(struct tcp_pcb *pcb){
      tcp_arg(pcb, NULL);
      tcp_sent(pcb, NULL);
      tcp_recv(pcb, NULL);
      tcp_close(pcb);
	}
	static err_t dataWasSent(void *arg, struct tcp_pcb *pcb, u16_t len)
	{


		err_t err = 0;
		ConnectionHandler* params = (ConnectionHandler*) arg;
		params->isConnectionInitialized = true;
		xQueueHandle qHandle = (xQueueHandle) params->txQueueHandle;
		for (;;)
		{
			// Try to receive message, put the task to sleep for at most TXTimeout ticks if queue is empty.
			if (xQueueReceive(qHandle, &params->msg, TXTimeout))
			{
				/*char text[5];
				strncpy(text, &params->msg[8],5);
				os_printf("Text: %d,%d,%s\n", *((uint32_t*)params->msg), *((uint32_t*)&params->msg[4]), text);*/
				uint16_t msgLength = (uint16_t)(4 + *((uint32_t*)params->msg));
				err = tcp_write(pcb, params->msg, msgLength, 0);
				tcp_sent(pcb, dataWasSent);
				return err;
			}
		}
		return err;
	}
	static err_t echo_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err){
	  char data[128];

		static const unsigned char stream[] =
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
		      0x74, 0x79, 0x70, 0x65, 0x3d, 0x73, 0x74, 0x64, 0x5f, 0x6d, 0x73, 0x67, 0x73, 0x2f, 0x53, 0x74, 0x72, 0x69, 0x6e, 0x67,
		/*0x09, 0x00, 0x00, 0x00,
		   0x05, 0x00, 0x00, 0x00,
		      0x68, 0x65, 0x6c, 0x6c, 0x6f*/
		};

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

    		//Free the packet buffer
    		pbuf_free(p);
            err = tcp_write(pcb, stream, sizeof(stream), 0);
            tcp_sent(pcb, dataWasSent);
      } else {
            pbuf_free(p);
      }

      if (err == ERR_OK && p == NULL) {
    	    ConnectionHandler* params = (ConnectionHandler*) arg;
    	    params->isConnectionInitialized = false;
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
      ConnectionHandler* params = (ConnectionHandler*) p;
      tcp_arg(pcb, params);
      tcp_bind(pcb, IP_ADDR_ANY, params->port);
      while(1)
      {
		  pcb = tcp_listen(pcb);
		  tcp_accept(pcb, echo_accept);
      }

      os_printf("XMLRPC response sent. Deleting task!\n");
      vTaskDelete(NULL);
	}
public:
	ConnectionHandler() {}
	void initPublisherEndpoint(uint16_t port)
	{
		this->port = port;
		isConnectionInitialized = false;
		txQueueHandle = xQueueCreate(TX_QUEUE_LEN, sizeof(char) * TX_QUEUE_MSG_SIZE);

		xTaskCreate(mytcp, (const signed char*)"tcpserver2", 1024, this, tskIDLE_PRIORITY + 2, NULL);
	}
	void sendMessage(const char* msg)
	{
		if (isConnectionInitialized)
		{
			// Initialize memory (in stack) for message.
			unsigned char data[TX_QUEUE_MSG_SIZE];
			// Copy message into the previously initialized memory.
			memcpy(data, msg, TX_QUEUE_MSG_SIZE);
			// Try to send message if queue is non-full.

			if (xQueueSend(txQueueHandle, &data, 0))
				os_printf("Enqueueing data!\n");
			else
				os_printf("Queue is full!\n");
		}
		else
			os_printf("Connection is not initialized yet!\n");
	}

};
