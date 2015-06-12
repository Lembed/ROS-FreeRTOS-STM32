#include "Node.h"
/*extern "C" int __aeabi_atexit(void *obj, void (*dtr)(void *), void *dso_h) {
    (void) obj;
    (void) dtr;
    (void) dso_h;
    return 0;
}

void *__dso_handle = 0;

extern "C" void __cxa_pure_virtual() {
        while (1)
                ;
}

namespace __gnu_cxx {

void __verbose_terminate_handler() {
        while(1)
                ;
}

}*/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
extern "C"
{
	void ETH_BSP_Config();
	void LwIP_Init();

	void delayMillis(unsigned long millis)
	{
		vTaskDelay(millis);
	}

	void SystemInit();
}
extern void ros_main(void*);


#include "stm32f4_discovery.h"
#include "wiring.h"

extern "C"
{
#include "USARTHandler.h"
}

void led_task(void* p)
{
	pinMode(GPIO_PD12, OUTPUT);
	for( ;; ) {
			// toggle LED4 each 250ms
	        //STM_EVAL_LEDToggle(LED4);
			digitalWrite(GPIO_PD12, HIGH);
	        vTaskDelay(250);
			digitalWrite(GPIO_PD12, LOW);
	        vTaskDelay(250);
	      }
}

/*-----------------------------------------------*/
/* we need this "forklift" task to let OS run before other interrupts corrupt the scheduler */
/*-----------------------------------------------*/
extern "C" void MainTask(void* args)
{
    /* configure Ethernet (GPIOs, clocks, MAC, DMA) */
    ETH_BSP_Config();

    /* Initialize the LwIP stack */
	LwIP_Init();

	xTaskCreate(ros_main, (const signed char*)"ROSMain", 128, NULL, 2, NULL);
	xTaskCreate(led_task, (const signed char*)"LedTask", 128, NULL, 2, NULL);

    vTaskDelete(NULL);
}

#define USART_BAUD_RATE 9600

int main()
{
	SystemInit();
	init_USART1(USART_BAUD_RATE); // initialize USART1 @ 9600 baud

	xTaskCreate(MainTask, (const signed char*)"MainTask", 1024, NULL, 1, NULL);
	vTaskStartScheduler();
	return 0;
}
extern "C"
void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/
extern "C"
void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/
extern "C"
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

