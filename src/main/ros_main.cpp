/* includes of RTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
extern "C"
{

#include "ros.h"
#include "rcl.h"
#include <string.h>
}

#include "Node.h"
#include "Subscriber.h"
#include "Publisher.h"

#include <XMLRPCServer.h>
#include <nodes.h>

#include "wiring.h"

extern "C"
void InitNodesTask(void* params)
{
	unsigned int num_nodes = sizeof(nodes) / sizeof(node_decriptor);
	for (unsigned int i = 0; i < num_nodes; i++) {
#ifdef DEADLINE_SCHEDULING
		xDeadlineTaskCreate(nodes[i].function, (const signed char*)nodes[i].name, configMINIMAL_STACK_SIZE * 4, NULL, nodes[i].deadline, NULL);
#else
		xTaskCreate(nodes[i].function, (const signed char*)nodes[i].name, configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 4UL , NULL);
#endif
	}

	vTaskDelete(NULL);
}

extern "C" void* os_malloc(unsigned int);
void* operator new(unsigned int sz)
{
	return os_malloc(sz);
}
void *operator new[](unsigned int sz)
{
	// TODO: is this correct?
	return os_malloc(sz);
}
void operator delete(void* ptr)
{
	// Do nothing, since once allocated memory cannot be freed!
}

void operator delete[](void *ptr)
{
	// Do nothing, since once allocated memory cannot be freed!
}


#define TSK_workload_PRIO                   ( tskIDLE_PRIORITY + 1UL )
#define TSK_monitor_PRIO                    ( tskIDLE_PRIORITY + 5UL )
#define TSK_monitor_PERIOD          ( ( portTickType ) 5000   / portTICK_RATE_MS )
#define PRINT_LOAD true
#define TSK_Monitor_STACK_SIZE				200
long int load_counter;
extern "C" void workload( void *pvParameters )
{
	( void ) pvParameters;
	load_counter = 0;
	for ( ;; ) {
		taskENTER_CRITICAL();
		load_counter++;
		taskEXIT_CRITICAL();
		/*if (load_counter % 100000000 == 0)
		{
		  //os_printf("Counter = %d\r\n", load_counter);
		  vTaskDelay(1);
		}*/
	}
}

#define MAX_LOAD_COUNTER 25087398.0
//#define MAX_LOAD_COUNTER 64511682.0

#include "math.h"
extern "C"
void Monitor( void *pvParameters )
{
	long int newCounter = 0;
	long int oldCounter = 0;
	long int Delta;
	portTickType xTimeToWait = TSK_monitor_PERIOD;
	portTickType xLastExecutionTime;
	int processor_workload;
	( void ) pvParameters;

	xLastExecutionTime = xTaskGetTickCount();
	// create workload task
	xTaskCreate( workload, ( signed portCHAR * ) "wload", configMINIMAL_STACK_SIZE, NULL, TSK_workload_PRIO, NULL );
	for ( ;; ) {
		// resume for 5 sec
		vTaskDelayUntil( &xLastExecutionTime, xTimeToWait );
		taskENTER_CRITICAL();
		newCounter = load_counter;
		taskEXIT_CRITICAL();
		Delta = newCounter - oldCounter;
		if ( Delta < 0 ) {
			Delta = newCounter - oldCounter + + pow(2, 32);
		}
		oldCounter = newCounter;
		processor_workload = ( (int) ((1 - Delta / (MAX_LOAD_COUNTER)) * 100.0));

		if (PRINT_LOAD) {
			os_printf("Counter = %d Delta = %d, workload during last 5 sec = %d percent \r\n", load_counter, Delta, processor_workload);
		}
	}
}

#define HIGHLOAD_COUNTER 200000
#define HIGHLOAD_PERIOD 50000


extern "C"
void highLoadTask( void *pvParameters )
{
	for (;;)
		for (uint32_t i = 0; i < HIGHLOAD_COUNTER; i++) {
			if (i == 0) {
				//os_printf("0\r\n");
				vTaskDelay(HIGHLOAD_PERIOD);
			} else {

			}
		}
}

extern "C" void TerminalTask(void*);

void ros_main(void* p)
{
	//xTaskCreate( Monitor, (const signed char*)"load", TSK_Monitor_STACK_SIZE, NULL, TSK_monitor_PRIO, NULL);
	xTaskCreate(TerminalTask, (const signed char*)"TerminalTask", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
	enableTiming();
	// TODO: Why is this delay necessary? Put a signaling mechanism instead, if the tasks below have to wait for some initialization.
	vTaskDelay(4000);
	XMLRPCServer::start();
	//xTaskCreate(highLoadTask, (const signed char*)"HighLoadTask", 128, NULL, tskIDLE_PRIORITY + 3, NULL);

	xTaskCreate(InitNodesTask, (const signed char*)"InitNodesTask", 128, NULL, tskIDLE_PRIORITY + 2, NULL);

	vTaskDelete(NULL);
}
