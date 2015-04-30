/*
c-pthread-queue - c implementation of a bounded buffer queue using posix threads
Copyright (C) 2008  Matthew Dickinson

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <pthread.h>
#include <stdio.h>

#ifndef _QUEUE_H
#define _QUEUE_H

#define QUEUE_INITIALIZER(buffer) { buffer, sizeof(buffer), 0, 0, 0, PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, PTHREAD_COND_INITIALIZER, 128 }

typedef struct queue
{
	char *buffer;
	//char** buffer;
	const int capacity;
	int size;
	int in;
	int out;
	pthread_mutex_t mutex;
	pthread_cond_t cond_full;
	pthread_cond_t cond_empty;
	int elem_size;
} queue_t;

void queue_enqueue(queue_t *queue, char *value)
{
	pthread_mutex_lock(&(queue->mutex));
	while (queue->size == queue->capacity)
		pthread_cond_wait(&(queue->cond_full), &(queue->mutex));
	//printf("enqueue %s\n", (const char*) value);
	//queue->buffer[queue->in] = value;
	memcpy(&queue->buffer[queue->in], value, queue->elem_size);
	
	//++ queue->size;
	queue->size += queue->elem_size;
	//++ queue->in;
	queue->in += queue->elem_size;
	queue->in %= queue->capacity;
	//printf("enqueue %s size:%d in:%d \n", (const char*) value, queue->size, queue->in);
	pthread_mutex_unlock(&(queue->mutex));
	pthread_cond_broadcast(&(queue->cond_empty));
}

char *queue_dequeue(queue_t *queue)
{
	pthread_mutex_lock(&(queue->mutex));
	while (queue->size == 0)
		pthread_cond_wait(&(queue->cond_empty), &(queue->mutex));
	//void *value = queue->buffer[queue->out];
	char* value = (char*)malloc(queue->elem_size);
	memcpy(value, &queue->buffer[queue->out], queue->elem_size);
	//printf("dequeue %s\n", (const char*) value);
	//-- queue->size;
	queue->size -= queue->elem_size;
	//++ queue->out;
	queue->out += queue->elem_size;
	queue->out %= queue->capacity;
	//printf("dequeue %s size: %d in: %d\n", (const char*) value, queue->size, queue->out);
	pthread_mutex_unlock(&(queue->mutex));
	pthread_cond_broadcast(&(queue->cond_full));
	return value;
}

int queue_size(queue_t *queue)
{
	pthread_mutex_lock(&(queue->mutex));
	int size = queue->size;
	pthread_mutex_unlock(&(queue->mutex));
	return size;
}

#endif
