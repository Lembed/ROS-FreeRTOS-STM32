/*! ************************************************************************************************
 *
 * \file
 *
 * (C) Robert Bosch GmbH
 *
 * *************************************************************************************************
 *
 * 	File:			Shell.c
 *
 * 	Purpose:	    Definitions and prototypes for Shell.
 *
 *           Author		Date			Comments
 *        Super Mario	09-Jan-2013		Initial Version
* **************************************************************************************************
*/
/*
 ***************************************************************************************************
 * Includes
 ***************************************************************************************************
 */
#include "Shell.h"
#include "USART_Setup.h"
#include "DMA_Setup.h"
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"

/* includes of RTOS */
	#include "FreeRTOS.h"
	#include "task.h"
	#include "timers.h"
	#include "semphr.h"

#include "main.h"


//#include "stm32f4xx_dac.h"

/*
 ***************************************************************************************************
 * Type definitions
 ***************************************************************************************************
// Name, Minimale Anzahl Paramter, maximale Anzahl Parameter, Pointer auf Routine, Parameter,
 */
CMDSHELL CMDSHELLTAB[] =
{
	{"help",  0, 0, helpShell, "Display this help message", " "},
	{"lu",  0, 0, lu, "Listing user tasks", " "},
	{"errvec",  0, 0, showGlobalErrorVectorAndClear, "show global error vector", ""},
	{"workload",  0, 0, loadSTM32F4, "show STM32F4 workload (abort -> return)", ""},
	{"activate",  1, 3, Change, "to activate the task","type hl for help"},
	{"terminate", 1, 1, Change, "to terminate the task","type hl for help"},
	{"suspend", 1, 1, Change, "to suspend the task","type hl for help"},
	{"continue", 1, 1, Change, "to continue the task","type hl for help"},
	{"delete", 1, 1, Change, "to delete the task","type hl for help"},
	{"hl", 0, 0, helpTaskHandleList, "check the task handle of every task",""} //,
//	{"loadPID", 6, 6, LoadPID," PID param: P, I, D, setpoint, windup, dfilter"," "},
//	{"display", 0, 0, BB_Display_Function, "Display function"," "},
//	{"MD03", 0, 0, MD03_status, "Display MD03 status"," "},
//	{"MD03run", 2, 2, MD03_run, "Program MD03"," "}
};
/*
 ***************************************************************************************************
 * Variables
 ***************************************************************************************************
 */
int g_i16_showLoad 		= FALSE;
portCHAR cListBuffer[ LU_LIST_BUFFER_SIZE ];
int  i16_Flag_koo;
const int NUM_CMDSHELL  = sizeof(CMDSHELLTAB)/sizeof(CMDSHELL);
char ShellCommand[80];
/* pointer Shell buffer */
char* 		ptrReadShellBuffer;
char* 		ptrEndShellBuffer;
char* 		ptrStartShellBuffer;
char*       ptrWriter;

extern long int globalErrorVector;
extern const int g_i16_Main_NUM_hl;
extern  char g_c_DMA_USART3Buffer1[];
extern uint16_t testbuffer[];
extern HandleList savedhandleTAB[];

/* Messages */
static const char PROMPT[] = "\r\nShell>";
static const char HELPMSG[]= "\r\nEnter 'help' for help.\n\n";
static const char SYNTAXSHELL[] = "\r\nError: Invalid syntax for: %s\n";
static const char INVCMD[] = "\r\nError: No such command: %s\n";
static const char INVALUESHELL[] = "\r\nError: Invalid value: %s\n";
//Al static const char HELPFORMATSHELL[] = "%8s %-40s %s %s\n";
static const char HELPFORMATSHELL[] = "%14s %-40s %s\n";
static const char HELPHANDLESHELL[] = "   %s\t\t%lu\r\n";
static const char INVOPT[] = "\r\nError:  Invalid set/show option: %s\n";
/*
 * *************************************************************************************************
 * Functions
 * *************************************************************************************************
 */
static uint32_t
get_valueShell (char *s, int *success, int base)
{
	uint32_t value;
	char *p;

	value = strtoul(s,&p,base);
	if ((value == 0) && (p == s))
	{
		*success = FALSE;
		return 0;
	}
	else
	{
		*success = TRUE;
		return value;
	}
}

void
initPtrShellBuffer (void)
{
	  ptrStartShellBuffer = g_c_DMA_USART3Buffer1;
	  ptrEndShellBuffer = ( g_c_DMA_USART3Buffer1 + ( BUFFERSIZEDMARX ) -1 );
	  ptrReadShellBuffer = ptrEndShellBuffer;
	  ptrWriter = ptrStartShellBuffer;
}

char*
movePtrShell (char* ptrTemp)
{
	if (ptrTemp >= ptrEndShellBuffer)
	{
		ptrTemp = ptrStartShellBuffer;
		#if (PrintDebug >= 2)
			ShellPrintf ("reached ptrEndShellBuffer %#x \r\n", (long int) ptrEndShellBuffer);
			ShellPrintf ("ptrTemp = ptrStartShellBuffer %#x \r\n", (long int) ptrStartShellBuffer);
		#endif
	}
	else
	{
		ptrTemp++;
	}
	return ptrTemp;
}

int
nextPtrEqDMAWritePtr (void)
{
	char* ptrReadShellBufferTemp;
	ptrWriter = g_c_DMA_USART3Buffer1 + ( size_DMA_Buffer_Shell ) - ( DMA_GetCurrDataCounter( DMA1_Stream1 ) );
	if(ptrWriter == g_c_DMA_USART3Buffer1 + ( size_DMA_Buffer_Shell ))
	{
		ptrWriter = g_c_DMA_USART3Buffer1;
	}
	ptrReadShellBufferTemp = movePtrShell(ptrReadShellBuffer);
	if ((char*) ptrWriter  == ptrReadShellBufferTemp )
	{
          	#if (PrintDebug >= 2)
	        ShellPrintf ("next ptrReadShellBuffer == schreibzeiger %#x \r\n", (long int) schreibzeiger);
        	#endif
		return TRUE;
	}
	else
	{
		ptrReadShellBuffer = ptrReadShellBufferTemp;
		return FALSE;
	}
}

int
make_argv (char *cmdline, char *argv[])
{
    int argc, i, in_text;

    /*
     * Break cmdline into strings and argv
     * It is permissible for argv to be NULL, in which case
     * the purpose of this routine becomes to count args
     */

    argc = 0;
    i = 0;
   	in_text = FALSE;

    while (cmdline[i] != '\0')  /* getline() must place 0x00 on end */
    {
        if (((cmdline[i] == ' ') || (cmdline[i] == '\t')) )
        {
            if (in_text)
            {
                /* end of command line argument */
                cmdline[i] = '\0';
                in_text = FALSE;
            }
            else
            {
                /* still looking for next argument */

            }
        }
        else
        {
            /* got non-whitespace character */
            if (in_text)
            {
            }
            else
            {
                /* start of an argument */
                in_text = TRUE;
                if (argc < MAX_ARGS)
                {
                    if (argv != NULL)
                        argv[argc] = &cmdline[i];
                    argc++;
                }
                else
                    /*return argc;*/
                    break;
            }

        }
        i++;    /* proceed to next character */
    }
    if (argv != NULL)
        argv[argc] = NULL;
    return argc;
}

static void
removeBsDel (char Buffer[])
{
	int pos = 0;
	int k;
	while (*(Buffer+pos) != 0)
	{
		switch (*(Buffer+pos))
		{
			case 0x08:		/* Backspace */
			case 0x7F:		/* Delete */

				if (pos == 0)
				{
					k = 0;
					while (*(Buffer+k) != 0)
					{
						*(Buffer+k) = *(Buffer+k+1);
						k++;
					}
					*(Buffer+k+1) = 0;
					pos = -1;
				}

				if (pos > 0)
				{
					k = pos;
					while (*(Buffer+k) != 0)
					{
						*(Buffer+k-1) = *(Buffer+k+1);
						k++;
					}
					*(Buffer+k+1) = 0;
					pos = pos - 2;
				}
				break;
		}
		pos++;
	}
}

void
commandinterpreter(char Buffer[])
{
	int argc;

	char *argv[MAX_ARGS + 1];	/* One extra for NULL terminator */

	removeBsDel(Buffer);

	argc = make_argv(Buffer,argv);

	if (argc)
	{
		int i;
		for (i = 0; i < NUM_CMDSHELL; i++)
		{
			if (strcasecmp(CMDSHELLTAB[i].cmd,argv[0]) == 0)
			{
				if (((argc-1) >= CMDSHELLTAB[i].min_args) && ((argc-1) <= CMDSHELLTAB[i].max_args))
				{
					CMDSHELLTAB[i].func(argc,argv);
					return;
				}
				else
				{
					ShellPrintf(SYNTAXSHELL,argv[0]);
					ShellPrintf(PROMPT);
					return;
				}
			}
		}
		ShellPrintf(INVCMD,argv[0]);
		ShellPrintf(HELPMSG);
	}
	ShellPrintf(PROMPT);
}

void
parseShell (void)
{
	int i = 0;
	int k = 0;
	char ShellCommandLocal[MAX_LINE];
	char* ptrReadShellBufferTemp = ptrReadShellBuffer;

	#if (PrintDebug >= 2)
		ShellPrintf ("START parseShell ptrReadShellBuffer %#x\r\n", ptrReadShellBuffer);
	#endif

	while (!nextPtrEqDMAWritePtr())
	{
		ShellCommandLocal[i] = *ptrReadShellBuffer;
		if (ShellCommandLocal[i] == '\r')
		{
			g_i16_showLoad 	= FALSE;
			break;
		}
		i++;
	}

	if(ShellCommandLocal[i] == '\r')
	{
		for (k=0; k <= i; k++)
		{
			ShellCommand[k] = ShellCommandLocal[k];
		}
		ShellCommand[k-1] = 0;  // overwriting \r with zero terminate char
		commandinterpreter(ShellCommand);

	}
	else
	{
		ptrReadShellBuffer = ptrReadShellBufferTemp;
	}
}

void

helpTaskHandleList(int argc, char **argv)
{
	int index;

	(void)argc;
	(void)argv;

	ShellPrintf("\r\n");
	ShellPrintf("command structure to change task status:\r\n");
	ShellPrintf("  activate || terminate || suspend || continue\r\n");
	ShellPrintf("\r\n");
	ShellPrintf("  action     task                     description\r\n");
	ShellPrintf("  activate  TaskName  all Period(ms)  *periodically activate the task\r\n");
	ShellPrintf("  activate  TaskName                  *one time run\r\n");
	ShellPrintf("  terminate TaskName                  *terminate it\r\n");
	ShellPrintf("  suspend   TaskName                  *suspend it\r\n");
	ShellPrintf("  continue  TaskName                  *continue it\r\n");
	ShellPrintf("\r\n");
	ShellPrintf("  you can check the task name and period below\r\n");
	ShellPrintf("\r\n");
	ShellPrintf("   Task\t\tCurrentPeriod\r\n");
	for (index = 0; index < g_i16_Main_NUM_hl; index++)
	{
		ShellPrintf(HELPHANDLESHELL,
			savedhandleTAB[index].name,
			*savedhandleTAB[index].perio);
	}
	ShellPrintf(PROMPT);
}

void
helpShell (int argc, char **argv)
{
	int index;

	(void)argc;
	(void)argv;

	ShellPrintf("\r\n");
	for (index = 0; index < NUM_CMDSHELL; index++)
	{
		ShellPrintf(HELPFORMATSHELL,
			CMDSHELLTAB[index].cmd,
			CMDSHELLTAB[index].description,
//			CMDSHELLTAB[index].cmd,
			CMDSHELLTAB[index].syntax);
	}
	ShellPrintf(PROMPT);
}

/*----------------------------------------------------*/
void
lu (int argc, char **argv)
{
	(void)argc;
	(void)argv;

	ShellPrintf("\ntask table\n");
    ShellPrintf("================================================\n");
    ShellPrintf("Task              State   Prio   Stack   queue# \n");
    ShellPrintf("================================================\n");
      vTaskList( ( signed portCHAR * ) cListBuffer );
      taskENTER_CRITICAL();

      ShellPrintf( "%s", cListBuffer );
      taskEXIT_CRITICAL();

	ShellPrintf(PROMPT);
}



/*----------------------------------------------------*/

void
Change (int argc, char **argv)
{
	//ShellPrintf(PROMPT);
	int index = 1;
  	int omg;
  	int i;
  	int success;
    const char *dummy[4] ;
    long int perio;
    const char *passport = "all";

	if (argc < 2)
	{
		ShellPrintf("Error: Invalid argument list\r\n");
		return;
	}

	for (index = 0; index < 4; index++)
	{
		dummy[index] = argv[index];
		//k = argv[index+1] -1;
		//*k = '\0';
	}
	ChangeTaskStatus(dummy[0]);
	omg = i16_Flag_koo;
	if(omg == 1)
	{
		for(i=0;i<g_i16_Main_NUM_hl;i++)
		{
			if(strcasecmp(dummy[1],savedhandleTAB[i].name) == 0)
			{
				if(strcasecmp(dummy[2],passport) == 0)
				{
					perio = get_valueShell(dummy[3],&success,10);
					if( perio == 0 )
					{
						ShellPrintf("Error: Invalid argument list\r\n");
						return;
					}
					else
					{
						*savedhandleTAB[i].flag1 = TRUE;
						*savedhandleTAB[i].flag2 = TRUE;
						*savedhandleTAB[i].perio = perio;
						vTaskResume( savedhandleTAB[i].hd );
						return;
					}
				}
				else
				{
					*savedhandleTAB[i].flag1 = TRUE;
					*savedhandleTAB[i].flag2 = FALSE;
					//*savedhandleTAB[i].perio = perio;
					vTaskResume( savedhandleTAB[i].hd );
					return;
				}
			}
		}
		ShellPrintf("\r\nError: Invalid TaskName\r\n");
		return;
	}
	else if(omg == 2)
	{
		for(i=0;i<g_i16_Main_NUM_hl;i++)
		{
			if(strcasecmp(dummy[1],savedhandleTAB[i].name) == 0)
			{
				*savedhandleTAB[i].flag1 = FALSE;
				return;
			}
		}
		ShellPrintf("\r\nError: Invalid TaskName\r\n");
		return;
	}
	else if(omg == 3)
	{
		for(i=0;i<g_i16_Main_NUM_hl;i++)
		{
			if(strcasecmp(dummy[1],savedhandleTAB[i].name) == 0)
			{
				vTaskSuspend( savedhandleTAB[i].hd );
				ShellPrintf("Task has been suspended\r\n");
				return;
			}
		}
		ShellPrintf("\r\nError: Invalid TaskName\r\n");
		return;
	}
	else if(omg == 4)
	{
		for(i=0;i<g_i16_Main_NUM_hl;i++)
		{
			if(strcasecmp(dummy[1],savedhandleTAB[i].name) == 0)
			{
				vTaskResume( savedhandleTAB[i].hd );
				ShellPrintf("Task continues\r\n");
				return;
			}
		}
		ShellPrintf("\r\nError: Invalid TaskName\r\n");
		return;
	}
	else if(omg == 5)
	{
		for(i=0;i<g_i16_Main_NUM_hl;i++)
		{
			if(strcasecmp(dummy[1],savedhandleTAB[i].name) == 0)
			{
				vTaskDelete( savedhandleTAB[i].hd );
				ShellPrintf("Task has been deleted\r\n");
				return;
			}
		}
		ShellPrintf("\r\nError: Invalid TaskName\r\n");
		return;
	}
	else
	{
		ShellPrintf("Error: Invalid argument list\r\n");
	}
}

/* -------------------------------------------------------- */

void
showGlobalErrorVectorAndClear (int argc, char **argv)
{
	(void)argc;
	(void)argv;

	ShellPrintf("\nGlobalErrorVector before clear = %#x", globalErrorVector);
    globalErrorVector = 0;
	ShellPrintf("\nGlobalErrorVector  after clear = %#x", globalErrorVector);
	ShellPrintf(PROMPT);
}

/* -------------------------------------------------------- */

void
loadSTM32F4 (int argc, char **argv)
{
	(void)argc;
	(void)argv;

    ShellPrintf("\nMonitoring starts within 5 sec\r\n");
    g_i16_showLoad = TRUE;
}

/* -------------------------------------------------------- */

void
ShellDaemon (void *pvParameters)
{
	portTickType xLastExecutionTime;
    portTickType xTimeToWait = TSK_Shell_PERIOD;

 	(void) pvParameters;

    xLastExecutionTime = xTaskGetTickCount();
	initPtrShellBuffer();

     for(;;)
     {
  		parseShell();
        vTaskDelayUntil(&xLastExecutionTime,xTimeToWait);

      }
     // should never reach this line
     ShellPrintf("shit happens!!, uDaemon terminate itself\r\n");
     vTaskDelete(NULL);
}

/* -------------------------------------------------------- */

void
ChangeTaskStatus(const char *s)
{
	const char *macro1 = "activate";
	const char *macro2 = "terminate";
	const char *macro3 = "suspend";
	const char *macro4 = "continue";
	const char *macro5 = "delete";
	int load = 0;
	if( strcasecmp (macro1,s) == 0 )
	{
		load = 1 ;
		i16_Flag_koo = load;
		return;
	}
	if( strcasecmp (macro2,s) == 0 )
	{
		load = 2 ;
		i16_Flag_koo = load;
		return;
	}
	if( strcasecmp (macro3,s) == 0 )
	{
		load = 3 ;
		i16_Flag_koo = load;
		return;
	}
	if( strcasecmp (macro4,s) == 0 )
	{
		load = 4 ;
		i16_Flag_koo = load;
		return;
	}
	if( strcasecmp (macro5,s) == 0 )
	{
		load = 5 ;
		i16_Flag_koo = load;
		return;
	}
	else
	{
		load = 0 ;
		i16_Flag_koo = load;
		return;
	}
}

/*
void LoadPID(int argc,char **argv)
{
	int i;
	double dummy[6];

	   if(argc==2)
		{
			ShellPrintf("\r\nError: Invalid argument list\r\n");
			ShellPrintf(PROMPT);
			return;
		}
		for(i=0;i<6;i++)
		{
			dummy[i] = (double)atof(argv[i+1]);
		}

		PID_Init(&test,dummy);
		ShellPrintf("\r\nPID is successfully loaded ,with the value of parameters:\r\n");
		ShellPrintf("P=%d\r\n",(int)test.pgain);
		ShellPrintf("I=%d\r\n",(int)test.igain);
		ShellPrintf("D=%d\r\n",(int)test.dgain);
		ShellPrintf("Setpoint=%d\r\n",(int)test.sp);
		ShellPrintf("Windup=%d\r\n",(int)test.windup);
		ShellPrintf("DFilter=%d\r\n",(int)test.dfilter);
}

*/
