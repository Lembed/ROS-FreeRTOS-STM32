/*! *************************************************************************************************
 *
 * \file
 *
 * (C) All rights reserved by Institute of Automatic Control, Leibniz University Hannover
 *
 * **************************************************************************************************
 *
 * 	File:			printf-stdarg.h
 *
 * 	Purpose:	    Driver of superChip123, delivers yaw, pitch and roll angle
 *
 *           Author		Date			Comments
 *        Super Mario		04-Sep-12		Initial Creation
*
*****************************************************************************************************
*/

#ifndef _PRINTF-STDARG_H_
#define _PRINTF-STDARG_H_

/*
 ***************************************************************************************************
 * Defines
 ***************************************************************************************************
 */
//#define putchar(c) c
#define TRUE  1
#define FALSE 0
#define TSK_Test_PERIOD_ms         (  200 )
#define TSK_Test_PERIOD            ( ( portTickType ) TSK_Test_PERIOD_ms   / portTICK_RATE_MS )
#define PAD_RIGHT 1
#define PAD_ZERO 2
#define PRINT_BUF_LEN 12

#define DEST_CONSOLE	(1)
#define DEST_STRING		(2)

#define FLAGS_MINUS		(0x01)
#define FLAGS_PLUS		(0x02)
#define FLAGS_SPACE		(0x04)
#define FLAGS_ZERO		(0x08)
#define FLAGS_POUND		(0x10)

#define IS_FLAG_MINUS(a)	(a & FLAGS_MINUS)
#define IS_FLAG_PLUS(a)		(a & FLAGS_PLUS)
#define IS_FLAG_SPACE(a)	(a & FLAGS_SPACE)
#define IS_FLAG_ZERO(a)		(a & FLAGS_ZERO)
#define IS_FLAG_POUND(a)	(a & FLAGS_POUND)

#define LENMOD_h		(0x01)
#define LENMOD_l		(0x02)
#define LENMOD_L		(0x04)

#define IS_LENMOD_h(a)	(a & LENMOD_h)
#define IS_LENMOD_l(a)	(a & LENMOD_l)
#define IS_LENMOD_L(a)	(a & LENMOD_L)

#define FMT_d	(0x0001)
#define FMT_o	(0x0002)
#define FMT_x	(0x0004)
#define FMT_X	(0x0008)
#define FMT_u	(0x0010)
#define FMT_c	(0x0020)
#define FMT_s	(0x0040)
#define FMT_p	(0x0080)
#define FMT_n	(0x0100)

#define IS_FMT_d(a)		(a & FMT_d)
#define IS_FMT_o(a)		(a & FMT_o)
#define IS_FMT_x(a)		(a & FMT_x)
#define IS_FMT_X(a)		(a & FMT_X)
#define IS_FMT_u(a)		(a & FMT_u)
#define IS_FMT_c(a)		(a & FMT_c)
#define IS_FMT_s(a)		(a & FMT_s)
#define IS_FMT_p(a)		(a & FMT_p)
#define IS_FMT_n(a)		(a & FMT_n)
/*
 ***************************************************************************************************
 * Type definitions
 ***************************************************************************************************
 */
typedef struct
{
	int	dest;
	void (*func)(char);
	char* loc;
} PRINTK_INFO;
/*
 * *************************************************************************************************
 * Function prototypes
 * *************************************************************************************************
 */
static void printchar(char **str, int c);
static int prints(char **out, const char *string, int width, int pad);
static int printi(char **out, int i, int b, int sg, int width, int pad, int letbase);
static int print( char **out, const char *format, va_list args );
//int snprintf( char *buf, unsigned int count, const char *format, ... );
int	write( int i, char* c, int n);
static void printk_putc (int c, int *count, PRINTK_INFO *info);
static int printk_mknumstr (char *numstr, void *nump, int neg, int radix);
static void printk_pad_zero (int curlen, int field_width, int *count, PRINTK_INFO *info);
static void printk_pad_space (int curlen, int field_width, int *count, PRINTK_INFO *info);
int printk (PRINTK_INFO *info, const char *fmt, va_list ap);
void out_char (char ch);
void ShellPrintf (const char *fmt, ...);
//int sprintf (char *s, const char *fmt, ...);
void Shell_DMA_Check(void *pvParameters);

#endif
