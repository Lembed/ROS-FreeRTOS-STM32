/*! ************************************************************************************************
 *
 * \file
 *
 * (C) All rights reserved by Institute of Automatic Control, Leibniz University Hannover
 *
 * *************************************************************************************************
 *
 * 	File:			printf-stdarg.c
 *
 * 	Purpose:	    Driver of superChip123, delivers yaw, pitch and roll angle
 *
 *           Author		Date			Comments
 *        Super Mario		04-Sep-12		Initial Creation
* **************************************************************************************************
*/
/*
 ***************************************************************************************************
 * Includes
 ***************************************************************************************************
 */
#include <stdarg.h>
#include "stm32f4xx_usart.h"
#include <string.h>
#include "DMA_Setup.h"
#include "FreeRTOS.h"
#include "task.h"
#include "printf-stdarg.h"
/*
 ***************************************************************************************************
 * Variables
 ***************************************************************************************************
 */
char c_Shell_dataisrdy = 0;
extern char *g_cptr_DMA_tempBsptr;
extern char *g_cptr_DMA_Momptr1;
extern char *g_cptr_DMA_Momptr2;
extern char g_c_DMAOUTPUT1[BUFFERSIZEDMATX];
extern char g_c_DMAOUTPUT2[BUFFERSIZEDMATX];
extern int g_i16_DMA_msgvalue;
/*
 * *************************************************************************************************
 * Functions
 * *************************************************************************************************
 */

/*
static void printchar(char **str, int c)
{//	extern int putchar(int c);

//copied from uart_putchar, modified by Al;

  while(USART_GetITStatus(UART5, USART_IT_TXE) == RESET);        
  USART_SendData(UART5, (uint8_t)c);

   // Send the character
  //  MCF_UART_UTB(0) = (uint8)c;


	if (str) {
//		**str = (char)c;
//		++(*str);
	}
	else
	{ 
//		(void)putchar(c);
	}

//end of modify 

}
*/

static int prints(char **out, const char *string, int width, int pad)
{
	register int pc = 0, padchar = ' ';

	if (width > 0) {
		register int len = 0;
		register const char *ptr;
		for (ptr = string; *ptr; ++ptr) ++len;
		if (len >= width) width = 0;
		else width -= len;
		if (pad & PAD_ZERO) padchar = '0';
	}
	if (!(pad & PAD_RIGHT)) {
		for ( ; width > 0; --width) {
			printchar (out, padchar);
			++pc;
		}
	}
	for ( ; *string ; ++string) {
		printchar (out, *string);
		++pc;
	}
	for ( ; width > 0; --width) {
		printchar (out, padchar);
		++pc;
	}

	return pc;
}

static int printi(char **out, int i, int b, int sg, int width, int pad, int letbase)
{
	char print_buf[PRINT_BUF_LEN];
	register char *s;
	register int t, neg = 0, pc = 0;
	register unsigned int u = (unsigned int)i;

	if (i == 0) {
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return prints (out, print_buf, width, pad);
	}

	if (sg && b == 10 && i < 0) {
		neg = 1;
		u = (unsigned int)-i;
	}

	s = print_buf + PRINT_BUF_LEN-1;
	*s = '\0';

	while (u) {
		t = (int)u % b;
		if( t >= 10 )
			t += letbase - '0' - 10;
		*--s = (char)(t + '0');
		u /= b;
	}

	if (neg) {
		if( width && (pad & PAD_ZERO) ) {
			printchar (out, '-');
			++pc;
			--width;
		}
		else {
			*--s = '-';
		}
	}

	return pc + prints (out, s, width, pad);
}

static int print( char **out, const char *format, va_list args )
{
	register int width, pad;
	register int pc = 0;
	char scr[2];

	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			width = pad = 0;
			if (*format == '\0') break;
			if (*format == '%') goto out;
			if (*format == '-') {
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') {
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; *format >= '0' && *format <= '9'; ++format) {
				width *= 10;
				width += *format - '0';
			}
			if( *format == 's' ) {
				register char *s = (char *)va_arg( args, int );
				pc += prints (out, s?s:"(null)", width, pad);
				continue;
			}
			if( *format == 'd' ) {
				pc += printi (out, va_arg( args, int ), 10, 1, width, pad, 'a');
				continue;
			}
			if( *format == 'x' ) {
				pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'X' ) {
				pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'A');
				continue;
			}
			if( *format == 'u' ) {
				pc += printi (out, va_arg( args, int ), 10, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'c' ) {
				/* char are converted to int then pushed on the stack */
				scr[0] = (char)va_arg( args, int );
				scr[1] = '\0';
				pc += prints (out, scr, width, pad);
				continue;
			}
		}
		else {
		out:
			printchar (out, *format);
			++pc;
		}
	}
	if (out) **out = '\0';
	va_end( args );
	return pc;
}

/*
int printf(const char *format, ...)
{
        va_list args;
        
        va_start( args, format );
        return print( 0, format, args );
}
*/
/*
int sprintf(char *out, const char *format, ...)
{
        va_list args;
        
        va_start( args, format );
        return print( &out, format, args );
}
*/
/*
int snprintf( char *buf, unsigned int count, const char *format, ... )
{
        va_list args;
        
        ( void ) count;
        
        va_start( args, format );
        return print( &buf, format, args );
}
*/

/*
#ifdef TEST_PRINTF
int main(void)
{
	char *ptr = "Hello world!";
	char *np = 0;
	int i = 5;
	unsigned int bs = sizeof(int)*8;
	int mi;
	char buf[80];

	mi = (1 << (bs-1)) + 1;
	printf("%s\n", ptr);
	printf("printf test\n");
	printf("%s is null pointer\n", np);
	printf("%d = 5\n", i);
	printf("%d = - max int\n", mi);
	printf("char %c = 'a'\n", 'a');
	printf("hex %x = ff\n", 0xff);
	printf("hex %02x = 00\n", 0);
	printf("signed %d = unsigned %u = hex %x\n", -3, -3, -3);
	printf("%d %s(s)%", 0, "message");
	printf("\n");
	printf("%d %s(s) with %%\n", 0, "message");
	sprintf(buf, "justif: \"%-10s\"\n", "left"); printf("%s", buf);
	sprintf(buf, "justif: \"%10s\"\n", "right"); printf("%s", buf);
	sprintf(buf, " 3: %04d zero padded\n", 3); printf("%s", buf);
	sprintf(buf, " 3: %-4d left justif.\n", 3); printf("%s", buf);
	sprintf(buf, " 3: %4d right justif.\n", 3); printf("%s", buf);
	sprintf(buf, "-3: %04d zero padded\n", -3); printf("%s", buf);
	sprintf(buf, "-3: %-4d left justif.\n", -3); printf("%s", buf);
	sprintf(buf, "-3: %4d right justif.\n", -3); printf("%s", buf);

	return 0;
}
*/

/*
 * if you compile this file with
 *   gcc -Wall $(YOUR_C_OPTIONS) -DTEST_PRINTF -c printf.c
 * you will get a normal warning:
 *   printf.c:214: warning: spurious trailing `%' in format
 * this line is testing an invalid % at the end of the format string.
 *
 * this should display (on 32bit int machine) :
 *
 * Hello world!
 * printf test
 * (null) is null pointer
 * 5 = 5
 * -2147483647 = - max int
 * char a = 'a'
 * hex ff = ff
 * hex 00 = 00
 * signed -3 = unsigned 4294967293 = hex fffffffd
 * 0 message(s)
 * 0 message(s) with %
 * justif: "left      "
 * justif: "     right"
 *  3: 0003 zero padded
 *  3: 3    left justif.
 *  3:    3 right justif.
 * -3: -003 zero padded
 * -3: -3   left justif.
 * -3:   -3 right justif.
 */

//#endif

int	write( int i, char* c, int n)
{
	(void)i;
	(void)n;
	(void)c;
	return 0;
}

static void
printk_putc (int c, int *count, PRINTK_INFO *info)
{
	switch (info->dest)
	{
		case DEST_CONSOLE:
			info->func((char)c);
			break;
		case DEST_STRING:
			*(info->loc) = (unsigned char)c;
			++(info->loc);
			break;
		default:
			break;
	}
	*count += 1;
}

static int
printk_mknumstr (char *numstr, void *nump, int neg, int radix)
{
	int a,b,c;
	unsigned int ua,ub,uc;

	int nlen;
	char *nstrp;

	nlen = 0;
	nstrp = numstr;
	*nstrp++ = '\0';

	if (neg)
	{
		a = *(int *)nump;
		if (a == 0)
		{
			*nstrp = '0';
			++nlen;
			goto done;
		}
		while (a != 0)
		{
			b = (int)a / (int)radix;
			c = (int)a - ((int)b * (int)radix);
			if (c < 0)
			{
				c = ~c + 1 + '0';
			}
			else
			{
				c = c + '0';
			}
			a = b;
			*nstrp++ = (char)c;
			++nlen;
		}
	}
	else
	{
		ua = *(unsigned int *)nump;
		if (ua == 0)
		{
			*nstrp = '0';
			++nlen;
			goto done;
		}
		while (ua != 0)
		{
			ub = (unsigned int)ua / (unsigned int)radix;
			uc = (unsigned int)ua - ((unsigned int)ub * (unsigned int)radix);
			if (uc < 10)
			{
				uc = uc + '0';
			}
			else
			{
				uc = uc - 10 + 'A';
			}
			ua = ub;
			*nstrp++ = (char)uc;
			++nlen;
		}
	}
	done:
	return nlen;
}

static void
printk_pad_zero (int curlen, int field_width, int *count, PRINTK_INFO *info)
{
	int i;

	for (i = curlen; i < field_width; i++)
	{
		printk_putc('0',count, info);
	}
}

static void
printk_pad_space (int curlen, int field_width, int *count, PRINTK_INFO *info)
{
	int i;

	for (i = curlen; i < field_width; i++)
	{
		printk_putc(' ',count, info);
	}
}

int
printk (PRINTK_INFO *info, const char *fmt, va_list ap)
{
	/* va_list ap; */
	char *p;
	int c;

	char vstr[33];
	char *vstrp;
	int vlen;

	int done;
	int count = 0;

	int	flags_used;
	int	field_width;
#if 0
	int	precision_used;
	int	precision_width;
	int	length_modifier;
#endif

	int	ival;
	int schar, dschar;
	int *ivalp;
	char *sval;
	int cval;
	unsigned int uval;

	/*
	 * Start parsing apart the format string and display appropriate
	 * formats and data.
	 */
	for (p = (char *)fmt; (c = *p) != 0; p++)
	{
		/*
		 * All formats begin with a '%' marker.  Special chars like
		 * '\n' or '\t' are normally converted to the appropriate
		 * character by the __compiler__.  Thus, no need for this
		 * routine to account for the '\' character.
		 */
		if (c != '%')
		{
			/*
			 * This needs to be replaced with something like
			 * 'out_char()' or call an OS routine.
			 */
#ifndef UNIX_DEBUG
			if (c != '\n')
			{
				printk_putc(c, &count, info);
			}
			else
			{
				printk_putc(0x0D /* CR */, &count, info);
				printk_putc(0x0A /* LF */, &count, info);
			}
#else
			printk_putc(c, &count, info);
#endif

			/*
			 * By using 'continue', the next iteration of the loop
			 * is used, skipping the code that follows.
			 */
			continue;
		}

		/*
		 * First check for specification modifier flags.
		 */
		flags_used = 0;
		done = FALSE;
		while (!done)
		{
			switch (/* c = */ *++p)
			{
				case '-':
					flags_used |= FLAGS_MINUS;
					break;
				case '+':
					flags_used |= FLAGS_PLUS;
					break;
				case ' ':
					flags_used |= FLAGS_SPACE;
					break;
				case '0':
					flags_used |= FLAGS_ZERO;
					break;
				case '#':
					flags_used |= FLAGS_POUND;
					break;
				default:
					/* we've gone one char too far */
					--p;
					done = TRUE;
					break;
			}
		}

		/*
		 * Next check for minimum field width.
		 */
		field_width = 0;
		done = FALSE;
		while (!done)
		{
			switch (c = *++p)
			{
				case '0':
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
					field_width = (field_width * 10) + (c - '0');
					break;
				default:
					/* we've gone one char too far */
					--p;
					done = TRUE;
					break;
			}
		}

		/*
		 * Next check for the width and precision field separator.
		 */
		if (/* (c = *++p) */ *++p == '.')
		{
			/* precision_used = TRUE; */

			/*
			 * Must get precision field width, if present.
			 */
			/* precision_width = 0; */
			done = FALSE;
			while (!done)
			{
				switch (/* c = uncomment if used below */ *++p)
				{
					case '0':
					case '1':
					case '2':
					case '3':
					case '4':
					case '5':
					case '6':
					case '7':
					case '8':
					case '9':
#if 0
						precision_width = (precision_width * 10) +
							(c - '0');
#endif
						break;
					default:
						/* we've gone one char too far */
						--p;
						done = TRUE;
						break;
				}
			}
		}
		else
		{
			/* we've gone one char too far */
			--p;
#if 0
			precision_used = FALSE;
			precision_width = 0;
#endif
		}

		/*
		 * Check for the length modifier.
		 */
		/* length_modifier = 0; */
		switch (/* c = */ *++p)
		{
			case 'h':
				/* length_modifier |= LENMOD_h; */
				break;
			case 'l':
				/* length_modifier |= LENMOD_l; */
				break;
			case 'L':
				/* length_modifier |= LENMOD_L; */
				break;
			default:
				/* we've gone one char too far */
				--p;
				break;
		}

		/*
		 * Now we're ready to examine the format.
		 */
		switch (c = *++p)
		{
			case 'd':
			case 'i':
				ival = (int)va_arg(ap, int);
				vlen = printk_mknumstr(vstr,&ival,TRUE,10);
				vstrp = &vstr[vlen];

				if (ival < 0)
				{
					schar = '-';
					++vlen;
				}
				else
				{
					if (IS_FLAG_PLUS(flags_used))
					{
						schar = '+';
						++vlen;
					}
					else
					{
						if (IS_FLAG_SPACE(flags_used))
						{
							schar = ' ';
							++vlen;
						}
						else
						{
							schar = 0;
						}
					}
				}
				dschar = FALSE;
			
				/*
				 * do the ZERO pad.
				 */
				if (IS_FLAG_ZERO(flags_used))
				{
					if (schar)
						printk_putc(schar, &count, info);
					dschar = TRUE;
			
					printk_pad_zero (vlen, field_width, &count, info);
					vlen = field_width;
				}
				else
				{
					if (!IS_FLAG_MINUS(flags_used))
					{
						printk_pad_space (vlen, field_width, &count, info);
			
						if (schar)
							printk_putc(schar, &count, info);
						dschar = TRUE;
					}
				}
			
				/* the string was built in reverse order, now display in */
				/* correct order */
				if (!dschar && schar)
				{
					printk_putc(schar, &count, info);
				}
				goto cont_xd;

			case 'x':
			case 'X':
				uval = (unsigned int)va_arg(ap, unsigned int);
				vlen = printk_mknumstr(vstr,&uval,FALSE,16);
				vstrp = &vstr[vlen];

				dschar = FALSE;
				if (IS_FLAG_ZERO(flags_used))
				{
					if (IS_FLAG_POUND(flags_used))
					{
						printk_putc('0', &count, info);
						printk_putc('x', &count, info);
						/*vlen += 2;*/
						dschar = TRUE;
					}
					printk_pad_zero (vlen, field_width, &count, info);
					vlen = field_width;
				}
				else
				{
					if (!IS_FLAG_MINUS(flags_used))
					{
						if (IS_FLAG_POUND(flags_used))
						{
							vlen += 2;
						}
						printk_pad_space (vlen, field_width, &count, info);
						if (IS_FLAG_POUND(flags_used))
						{
							printk_putc('0', &count, info);
							printk_putc('x', &count, info);
							dschar = TRUE;
						}
					}
				}

				if ((IS_FLAG_POUND(flags_used)) && !dschar)
				{
					printk_putc('0', &count, info);
					printk_putc('x', &count, info);
					vlen += 2;
				}
				goto cont_xd;

			case 'o':
				uval = (unsigned int)va_arg(ap, unsigned int);
				vlen = printk_mknumstr(vstr,&uval,FALSE,8);
				goto cont_u;
			case 'b':
				uval = (unsigned int)va_arg(ap, unsigned int);
				vlen = printk_mknumstr(vstr,&uval,FALSE,2);
				goto cont_u;
			case 'p':
				uval = (unsigned int)va_arg(ap, void *);
				vlen = printk_mknumstr(vstr,&uval,FALSE,16);
				goto cont_u;
			case 'u':
				uval = (unsigned int)va_arg(ap, unsigned int);
				vlen = printk_mknumstr(vstr,&uval,FALSE,10);

				cont_u:
					vstrp = &vstr[vlen];

					if (IS_FLAG_ZERO(flags_used))
					{
						printk_pad_zero (vlen, field_width, &count, info);
						vlen = field_width;
					}
					else
					{
						if (!IS_FLAG_MINUS(flags_used))
						{
							printk_pad_space (vlen, field_width, &count, info);
						}
					}

				cont_xd:
					while (*vstrp)
						printk_putc(*vstrp--, &count, info);

					if (IS_FLAG_MINUS(flags_used))
					{
						printk_pad_space (vlen, field_width, &count, info);
					}
				break;

			case 'c':
				cval = (char)va_arg(ap, unsigned int);
				printk_putc(cval,&count, info);
				break;
			case 's':
				sval = (char *)va_arg(ap, char *);
				if (sval)
				{
					vlen = strlen(sval);
					if (!IS_FLAG_MINUS(flags_used))
					{
						printk_pad_space (vlen, field_width, &count, info);
					}
					while (*sval)
						printk_putc(*sval++,&count, info);
					if (IS_FLAG_MINUS(flags_used))
					{
						printk_pad_space (vlen, field_width, &count, info);
					}
				}
				break;
			case 'n':
				ivalp = (int *)va_arg(ap, int *);
				*ivalp = count;
				break;
			default:
				printk_putc(c,&count, info);
				break;
		}
	}
	return count;
}

void
out_char (char ch)
{

	//while(USART_GetITStatus(USART3, USART_IT_TXE) == RESET);
	//USART_SendData(USART3, ch);
	if(g_cptr_DMA_tempBsptr == g_c_DMAOUTPUT1)
	{
		*g_cptr_DMA_Momptr2 = ch;
		g_cptr_DMA_Momptr2++;
		/*check buffersize*/
		if(g_cptr_DMA_Momptr2 >= g_c_DMAOUTPUT2 + 2048)
			g_cptr_DMA_Momptr2 = g_c_DMAOUTPUT2;
	}
	else
	{
		*g_cptr_DMA_Momptr1 = ch;
		g_cptr_DMA_Momptr1++;
		if(g_cptr_DMA_Momptr1 >= g_c_DMAOUTPUT1 + 2048)
			g_cptr_DMA_Momptr1 = g_c_DMAOUTPUT1;
	}

}

/*void
out_char1(char ch)
{
	while(USART_GetITStatus(USART3, USART_IT_TXE) == RESET);
	USART_SendData(USART3, ch);
}*/

void
ShellPrintf (const char *fmt, ...)
{
	va_list ap;
	int rvalue;
	PRINTK_INFO info;


	info.dest = DEST_CONSOLE;
	info.func = &out_char;

	 // Initialize the pointer to the variable length argument list.

	va_start(ap, fmt);
	rvalue = printk(&info, fmt, ap);

	 // Cleanup the variable length argument list.

	va_end(ap);
	c_Shell_dataisrdy = 1;
}

/*int
ShellPrintf (const char *fmt, ...)
{
	va_list ap;
	int rvalue;
	PRINTK_INFO info;


	info.dest = DEST_CONSOLE;
	info.func = &out_char;
	
	 // Initialize the pointer to the variable length argument list.
	
	va_start(ap, fmt);
	rvalue = printk(&info, fmt, ap);
	
	 // Cleanup the variable length argument list.
	 
	va_end(ap);
	return rvalue;
}*/

/*
int
sprintf (char *s, const char *fmt, ...)
{
	va_list ap;
	int rvalue = 0;
	PRINTK_INFO info;


	// * Initialize the pointer to the variable length argument list.

	if (s != 0)
	{
		info.dest = DEST_STRING;
		info.loc = s;
		va_start(ap, fmt);
		rvalue = printk(&info, fmt, ap);
		*info.loc = '\0';
		va_end(ap);
	}
	return rvalue;
}

*/
void Shell_DMA_Check(void *pvParameters)
{
	portTickType xLastExecutionTime;
    portTickType xTimeToWait = TSK_Test_PERIOD;
    xLastExecutionTime = xTaskGetTickCount();

    for(;;)
	{
    	if(c_Shell_dataisrdy == 1 && DMA_GetCmdStatus(DMA1_Stream3) == DISABLE)

    	{
    		c_Shell_dataisrdy = 0;
    		if(g_cptr_DMA_tempBsptr == g_c_DMAOUTPUT1)
    		{
    			g_cptr_DMA_tempBsptr = g_c_DMAOUTPUT2;
    			g_i16_DMA_msgvalue = g_cptr_DMA_Momptr2-g_cptr_DMA_tempBsptr;
    			g_cptr_DMA_Momptr2 = g_c_DMAOUTPUT2;
    			DMATxInit();
    		}
    		else
    		{
    			g_cptr_DMA_tempBsptr = g_c_DMAOUTPUT1;
    			g_i16_DMA_msgvalue = g_cptr_DMA_Momptr1 -g_cptr_DMA_tempBsptr;
    			g_cptr_DMA_Momptr1 = g_c_DMAOUTPUT1;
    			DMATxInit();
    		}
    	}
    	else;
    	vTaskDelayUntil(&xLastExecutionTime,xTimeToWait);
	}
}


