/**************************************************************************//*****
 * @file     stdio.c
 * @brief    Implementation of newlib syscall
 ********************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
//#include "LCD_Dotmatrix.h"

#undef errno
extern int errno;
extern int  _end;

caddr_t _sbrk ( int incr )
{
  static unsigned char *heap = NULL;
  unsigned char *prev_heap;

  if (heap == NULL) {
    heap = (unsigned char *)&_end;
  }
  prev_heap = heap;

  heap += incr;

  return (caddr_t) prev_heap;
}

int link(char *old, char *new) {
return -1;
}

int _close(int file)
{
  return -1;
}

int _fstat(int file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  return 1;
}

int _lseek(int file, int ptr, int dir)
{
  return 0;
}

int _read(int file, char *ptr, int len)
{
  return 0;
}

int _write(int file, char *ptr, int len)
{
	/* Place your implementation of fputc here */

	  /* e.g. write a character to the USART */

	        int counter;



	        counter = len;

	        for (; counter > 0; counter--)

	        {

	                        if (*ptr == 0) break;

	                        //USART_SendData(USART2, (uint16_t) (*ptr));
	                        //lcd_putchar((*ptr));
	                        /* Loop until the end of transmission */

	                        //while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

	                        ptr++;

	        }
  return len;
}

void abort(void)
{
  /* Abort called */
  while(1);
}
          
/* --------------------------------- End Of File ------------------------------ */
