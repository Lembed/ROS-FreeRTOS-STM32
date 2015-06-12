#include "ros.h"
#include "rcl.h"
#include <stdarg.h>
#include <stdio.h>

// application memory
char memory[20 * 1024];
unsigned int offset = 0;


void* os_malloc(unsigned int size)
{
	unsigned int index = offset;
	offset += size;
	return &memory[index];
}

#include "USARTHandler.h"
void os_printf(const char* fmt, ...)
{
    va_list ap;
    char string[128];

    va_start(ap, fmt);
    vsprintf(string, fmt, ap);
    va_end(ap);

    if (strlen(string) > 2 && strlen(string) < 128 && string[strlen(string)-1] == '\n' && string[strlen(string)-2] != '\r')
    {
    	string[strlen(string)-1] = '\r';
    	string[strlen(string)] = '\n';
    }

    taskDISABLE_INTERRUPTS();
    USART_puts(USART1, (volatile char*)string);
    taskENABLE_INTERRUPTS();
    //tr_log(string);
}
