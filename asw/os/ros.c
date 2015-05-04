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


void os_printf(const char* fmt, ...)
{
    va_list ap;
    char string[128];

    va_start(ap, fmt);
    vsprintf(string, fmt, ap);
    va_end(ap);

    tr_log(string);
}

void addItemToList(ListItem* head, void* object)
{
    ListItem* current = head;
    if (current->object == NULL)
    {
        current->object = object;
        current->next = NULL;
    }
    else
    {
		while (current->next != NULL) {
			current = current->next;

		}

		/* now we can add a new variable */
		current->next = (ListItem*)os_malloc(sizeof(ListItem));
		current->next->object = object;
		current->next->next = NULL;
    }
}
