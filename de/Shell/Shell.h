/*! ************************************************************************************************
 *
 * \file
 *
 * (C) Robert Bosch GmbH
 *
 * *************************************************************************************************
 *
 * 	File:			Shell.h
 *
 * 	Purpose:	    Definitions and prototypes for Shell.
 *
 *           Author		Date			Comments
 *        Super Mario	09-Jan-2013		Initial Version
* **************************************************************************************************
*/

#ifndef SHELL_H_
#define SHELL_H_
/*
 ***************************************************************************************************
 * Defines
 ***************************************************************************************************
 */
#define MAX_ARGS	15
#define MAX_LINE	80
#define MAX_FN		40

#define DebugShell        0
#define LU_LIST_BUFFER_SIZE	 2048

/*
 ***************************************************************************************************
 * Type definitions
 ***************************************************************************************************
 */
typedef const struct
{
	char *	cmd;					/* command name user types, ie. GO	*/
	int		min_args;				/* min num of args command accepts	*/
	int		max_args;				/* max num of args command accepts	*/
	void	(*func)(int, char **);	/* actual function to call			*/
	char *	description;			/* brief description of command		*/
	char *	syntax;					/* syntax of command				*/
} CMDSHELL;

typedef void * xTaskHandle;

typedef struct
{
	char*          		name;
	xTaskHandle    		hd;
	int*           		flag1;
	int*				flag2;
	long int*  			perio;
} HandleList;

typedef const struct
{
    char *  option;
    void    (*func)(int, char **);
    char *  syntax;
} SETCMDSHELL;
/*
 * *************************************************************************************************
 * Function prototypes
 * *************************************************************************************************
 */
void helpShell 		(int, char **);
void lu 			(int, char **);
void MD03_status 	(int, char **);
void MD03_run 		(int argc,char **argv);
void showGlobalErrorVectorAndClear (int, char **);
void loadSTM32F4 	(int, char **);
void Change 		(int, char **);
void helpTaskHandleList(int argc, char **argv);
void ShellDaemon	(void *pvParameters);
void ChangeTaskStatus(const char *s);
void initPtrShellBuffer (void);
char* movePtrShell 	(char* ptrTemp);
int nextPtrEqDMAWritePtr (void);
int make_argv 		(char *cmdline, char *argv[]);
static void removeBsDel (char Buffer[]);
void commandinterpreter(char Buffer[]);
void parseShell 	(void);
void LoadPID 		(int argc,char **argv);

#endif
