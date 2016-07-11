/*********************************************************/
/* This a sample file for main.h
 *
 * including some parametrization of tasks
 *
 */
/*********************************************************/

/* includes of RTOS */
	#include "FreeRTOS.h"
	#include "task.h"
	#include "timers.h"
	#include "semphr.h"

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4x7_eth_bsp.h"

#define USE_LCD        /* enable LCD  */
//Al #define USE_DHCP       /* enable DHCP, if disabled static address is used*/


/* MII and RMII mode selection, for STM324xG-EVAL Board(MB786) RevB ***********/
#define RMII_MODE


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Time_Update(void);
void Delay(uint32_t nCount);

/*-----------------------------------------------------------*/
/* definition of (initial) task priorities */
/*-----------------------------------------------------------*/
/* task 'workload' needs to be one above of IDLE and task
 * monitor to be sufficiently high */

#define TSK_Shell_PRIO						( tskIDLE_PRIORITY + 4UL )
#define TSK_Shell_DMA_Check_PRIO       		( tskIDLE_PRIORITY + 3UL )
#define TSK_workload_PRIO                   ( tskIDLE_PRIORITY + 1UL )
#define TSK_monitor_PRIO                    ( tskIDLE_PRIORITY + 4UL )

#define DHCP_TASK_PRIO   ( tskIDLE_PRIORITY + 2 )
#define LED_TASK_PRIO    ( tskIDLE_PRIORITY + 1 )


/*-----------------------------------------------------------*/
/* definition of task stack sizes */
/*-----------------------------------------------------------*/
// #define TSK_Shell_STACK_SIZE		configMINIMAL_STACK_SIZE
#define TSK_Shell_STACK_SIZE				800
#define TSK_Shell_DMA_Check_STACK_SIZE		200
#define TSK_Monitor_STACK_SIZE				200
#define TSK_PID_STACK_SIZE					300
#define TSK_DISPLAY_STACK_SIZE				300
#define TSK_TASKA_STACK_SIZE				300
#define TSK_TASKB_STACK_SIZE				300
#define TSK_TASKC_STACK_SIZE				300

/*-----------------------------------------------------------*/
/* definition of task periods */
/*-----------------------------------------------------------*/
#define TSK_monitor_PERIOD          ( ( portTickType ) 5000   / portTICK_RATE_MS )
#define TSK_Shell_PERIOD_ms         (  100 )
#define TSK_Shell_PERIOD            ( ( portTickType ) TSK_Shell_PERIOD_ms   / portTICK_RATE_MS )

//#define TSK_DISPLAY_PERIOD_ms       (  1000 )
//#define TSK_DISPLAY_PERIOD			( ( portTickType ) TSK_DISPLAY_PERIOD_ms   / portTICK_RATE_MS )

#define TRUE   1
#define FALSE  0

