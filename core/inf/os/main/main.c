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


/* includes of shell specific routines */
	#include "Shell.h"
	#include "USART_Setup.h"
	#include "stm32f4xx.h"
	#include "stm32f4xx_conf.h"
	#include "DMA_Setup.h"
    #include "stm32f4xx_dma.h"

/* includes of TCP/IP specific routines */

 	#include "stm32f4x7_eth.h"
 	#include "netconf.h"
 	#include "tcpip.h"

	#include "stm32f4_discovery.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
	extern struct netif xnetif;
__IO uint32_t test;
/* Private function prototypes -----------------------------------------------*/
 	extern void tcpecho_init(void);
	extern void udpecho_init(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Toggle Led4 task
  * @param  pvParameters not used
  * @retval None
  */
void ToggleLed4(void * pvParameters)
{
  while (1)
  {
	test = xnetif.ip_addr.addr;
    //Al test = 1;
	  /*check if IP address assigned*/
    if (test !=0) {
      for( ;; ) {
        /* toggle LED4 each 250ms */
        STM_EVAL_LEDToggle(LED4);
        vTaskDelay(250);
      }
    }
  }
}



/* includes of task parameterizations */
	#include "main.h"

/*-----------------------------------------------------------*/
/* definition of potentially variable task periods
 * in handle list (definition as variables) */
/*-----------------------------------------------------------*/

//long int TSK_TASKC_PERIOD_ms 	= 1000;

/*-----------------------------------------------------------*/
/*  definition of function prototypes */
/*-----------------------------------------------------------*/

extern void ShellDaemon ( void *pvParameters );
static void prvSetupHardware( void );
void 		ShellPrintf (const char *, ...);
extern void Shell_DMA_Check(void *pvParameters);
void 		Monitor( void *pvParameters );
void 		saveTaskHandle( void );

/*-----------------------------------------------------------*/
/*  definition of task handles for handle list */
/*-----------------------------------------------------------*/
//xTaskHandle xHandleTASKC;

//int runtest_TASKC_1 = FALSE;
//int runtest_TASKC_2 = FALSE;

HandleList savedhandleTAB[] =
{
//	{"TASKC" ,0 ,&runtest_TASKC_1,&runtest_TASKC_2,&TSK_TASKC_PERIOD_ms},
};

const int g_i16_Main_NUM_hl = sizeof(savedhandleTAB)/sizeof(HandleList);

/*-----------------------------------------------------------*/
/*  definition of semaphores */
/*-----------------------------------------------------------*/
xSemaphoreHandle xSemaphore = NULL;


/*-----------------------------------------------------------*/
/*  some variables for workload 							 */
/*-----------------------------------------------------------*/
long   int Zaehler ;
extern int g_i16_showLoad;


long int globalErrorVector = 0;

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/





int Maintask(void)
{
	/* Configure the hardware */
	//Al prvSetupHardware();

    /* configure Ethernet (GPIOs, clocks, MAC, DMA) */
    ETH_BSP_Config();


    /* Initialize the LwIP stack */
	LwIP_Init();

    /* Initialize tcp echo server */
//	tcpecho_init();

    /* Initialize udp echo server */
//	udpecho_init();


   /* Initialize webserver demo */
    http_server_netconn_init();


	ShellPrintf("\r\n============================================================\r\n");
    ShellPrintf("==================== STM32F4 running FreeRTOS ==============\r\n");
    ShellPrintf("============================================================\r\n");
    ShellPrintf("======================= UDP/TCP EchoServer =================\r\n");
    ShellPrintf("=======================       or       =================\r\n");
    ShellPrintf("======================= HTTP Server =================\r\n");
    ShellPrintf("============================================================\r\n\r\n");
    ShellPrintf("Shell>");


    // Create all required Semaphores
//    vSemaphoreCreateBinary( Sema1 );

    // Create all required Tasks
    // Shell currently running via UART3

	#ifdef USE_DHCP
    /* Start DHCPClient */
    	xTaskCreate(LwIP_DHCP_task, "DHCPClient", configMINIMAL_STACK_SIZE * 2, NULL,DHCP_TASK_PRIO, NULL);
	#endif

  /* Start toogleLed4 task : Toggle LED4  every 250ms */
  xTaskCreate(ToggleLed4, "LED4", configMINIMAL_STACK_SIZE, NULL, LED_TASK_PRIO, NULL);


    xTaskCreate( ShellDaemon, ( signed portCHAR * ) "Shell", TSK_Shell_STACK_SIZE, NULL, TSK_Shell_PRIO, NULL );
    xTaskCreate( Shell_DMA_Check, "ShlDMA", TSK_Shell_DMA_Check_STACK_SIZE,  NULL, TSK_Shell_DMA_Check_PRIO, NULL );
    xTaskCreate( Monitor, "load", TSK_Monitor_STACK_SIZE, NULL, TSK_monitor_PRIO, NULL);
//    xTaskCreate( TASKC, "TASKC", TSK_TASKC_STACK_SIZE, NULL, TSK_TASKC_PRIO, &xHandleTASKC);

    /* create task for commands from shell */
    saveTaskHandle();
	/* Start the scheduler. */
//Al	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached !!!! */
    //Al    for( ;; );

    vTaskDelete(NULL); // kill yourself
}


/*-----------------------------------------------*/
/* we need this "forklift" task to let OS run before other interrupts corrupt the scheduler */
/*-----------------------------------------------*/

int main(void) {

	/* Setup STM32 system (clock, PLL and Flash configuration) */
	SystemInit();
	/* Setup the UART */
    USART_Configuration();
    USART3_DMA_Config();

    /*Initialize LED  */
    STM_EVAL_LEDInit(LED4);

    /* forklift task */
	xTaskCreate(Maintask, (const signed portCHAR *)"MAIN", configMINIMAL_STACK_SIZE*4, NULL, 1, NULL);
	vTaskStartScheduler();
}



/*------------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Setup STM32 system (clock, PLL and Flash configuration) */
	SystemInit();
	/* Setup the UART */
    USART_Configuration();
    USART3_DMA_Config();

    /*Initialize LED  */
    STM_EVAL_LEDInit(LED4);



    /* configure Ethernet (GPIOs, clocks, MAC, DMA) */
    ETH_BSP_Config();

    //Initialize I2C Port with its peripheral
//    I2C_LowLevel_Init();

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

/*-----------------------------------------------------------*/

void saveTaskHandle( void )
{
	/* handle list MUST coincide with savedhandleTAB definition */
//	savedhandleTAB[0].hd = xHandleTestForPID;
//	savedhandleTAB[1].hd = xHandleTestForDisplay;
}

/*-----------------------------------------------------------*/

static void workload( void *pvParameters )
{
  ( void ) pvParameters;
  Zaehler = 0;
  for( ;; )
  {
	        Zaehler++;
  }
}

/*-----------------------------------------------------------*/

void Monitor( void *pvParameters )
{
    long int Zaehler_neu = 0;
    long int Zaehler_alt = 0;
    long int Delta;
	portTickType xTimeToWait = TSK_monitor_PERIOD;
	portTickType xLastExecutionTime;
	int processor_workload;
	( void ) pvParameters;

  	 xLastExecutionTime = xTaskGetTickCount();
		// create workload task
		xTaskCreate( workload, ( signed portCHAR * ) "wload", configMINIMAL_STACK_SIZE, NULL, TSK_workload_PRIO, NULL );
     for( ;; )
     {
        // resume for 5 sec
    	vTaskDelayUntil( &xLastExecutionTime, xTimeToWait );
    	Zaehler_neu = Zaehler;
    	Delta = Zaehler_neu - Zaehler_alt;
    	if( Delta < 0 )
    	{
    		Delta = Zaehler_neu - Zaehler_alt + + pow(2,32);
    	}
    	Zaehler_alt = Zaehler_neu;
    	processor_workload = ( (int) ((1-Delta/(64511682.0))*100.0));

        if (g_i16_showLoad == TRUE)
		{
     	   ShellPrintf("Delta = %d, workload during last 5 sec = %d percent \r\n", Delta, processor_workload);
		}
     }
}


