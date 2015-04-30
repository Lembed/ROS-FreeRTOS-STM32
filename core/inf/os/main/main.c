
/*********************************************************/
/* This a sample for making use of lwIP stack together with
 * freeRTOS
 *
 * including some parameterization of tasks and set up of shell
 *
 * Amos Albert (BOSP/PAA)   Initial version 23.12.2014
 */
/*********************************************************/

/* includes of RTOS */
	#include "FreeRTOS.h"
	#include "task.h"
	#include "timers.h"
	#include "semphr.h"

	#include "stm32f4_discovery.h"
/* includes of task parameterizations */
	#include "main.h"

/**
  * @brief  Toggle Led4 task
  * @param  pvParameters not used
  * @retval None
  */
void ToggleLed4(void * pvParameters)
{
	for( ;; ) {
		/* toggle LED4 each 250ms */
        STM_EVAL_LEDToggle(LED4);
        vTaskDelay(250);
      }
}

/*-----------------------------------------------------------*/
/*  definition of semaphores */
/*-----------------------------------------------------------*/
xSemaphoreHandle xSemaphore = NULL;

#include "ros.h"
int Maintask(void)
{
    /* configure Ethernet (GPIOs, clocks, MAC, DMA) */
    ETH_BSP_Config();

    /* Initialize the LwIP stack */
	LwIP_Init();

	/* Start toogleLed4 task : Toggle LED4  every 250ms */
    xTaskCreate(ToggleLed4, "LED4", configMINIMAL_STACK_SIZE, NULL, LED_TASK_PRIO, NULL);
    //xTaskCreate(os_main, "OS_Main", configMINIMAL_STACK_SIZE*4, NULL, LED_TASK_PRIO, NULL);
    xTaskCreate(ros_main, "OS_Main", configMINIMAL_STACK_SIZE*4, NULL, LED_TASK_PRIO, NULL);

    //xTaskCreate( Shell_DMA_Check, "ShlDMA", TSK_Shell_DMA_Check_STACK_SIZE,  NULL, TSK_Shell_DMA_Check_PRIO, NULL );

    vTaskDelete(NULL); // kill yourself
}


/*-----------------------------------------------*/
/* we need this "forklift" task to let OS run before other interrupts corrupt the scheduler */
/*-----------------------------------------------*/

int main(void) {

	/* Setup STM32 system (clock, PLL and Flash configuration) */
	SystemInit();
	/* Setup the UART */
    //USART_Configuration();
    //USART3_DMA_Config();

    /*Initialize LED  */
    STM_EVAL_LEDInit(LED4);

    /* forklift task */
	xTaskCreate(Maintask, (const signed portCHAR *)"MAIN", configMINIMAL_STACK_SIZE*4, NULL, 1, NULL);
	vTaskStartScheduler();
}


/*-----------------------------------------------------------*/

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


