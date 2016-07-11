#ifndef QUEUE_H_
#define QUEUE_H_

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "string.h"

#define DEFAULT_QUEUE_TIMEOUT 60000



class Queue
{
public:
	Queue(uint16_t queueLength, uint16_t messageSize)
	{
		this->messageSize = messageSize;
		queue = xQueueCreate(queueLength, messageSize);

		if (queueMutex == NULL) {
			vSemaphoreCreateBinary(queueMutex);
			xSemaphoreGive(queueMutex);
		}
	}
	void enqueue(void* item)
	{
		// TODO: Does item need to be copied?
		// Initialize memory (in stack) for message.
		unsigned char data[messageSize];
		// Copy message into the previously initialized memory.
		memcpy(data, item, messageSize);

		if (xQueueSend(queue, data, 0)) {

		} else { // If queue is full, dequeue one item and then enqueue. TODO: Is this a thread-safe operation? Is mutex required?
			// mutex lock
			while (!xSemaphoreTake(queueMutex, 20));
			xQueueReceive(queue, data, 0);
			enqueue(item);
			// mutex unlock
			xSemaphoreGive(queueMutex);

		}
	}
	void dequeue(void* msg)
	{
		if (xQueueReceive(queue, msg, DEFAULT_QUEUE_TIMEOUT)) {

		} else { // If queue is empty, wait forever
			dequeue(msg);
		}
	}
private:
	xQueueHandle queue;
	uint16_t messageSize;
	static xSemaphoreHandle queueMutex;
};



#endif // QUEUE_H_
