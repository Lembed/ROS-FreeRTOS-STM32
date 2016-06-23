/*! *************************************************************************************************
 *
 * \file
 *
 * (C) All rights reserved by Institute of Automatic Control, Leibniz University Hannover
 *
 * **************************************************************************************************
 *
 * 	File:			DMA_Setup.h
 *
 * 	Purpose:	    DMA setup of the Shell for input and output.
 *
 *           Author		Date			Comments
 *        Super Mario		04-Sep-12		Initial Creation
*
*****************************************************************************************************
*/
#ifndef DMA_SETUP_H_
#define DMA_SETUP_H_
/*
 ***************************************************************************************************
 * Defines
 ***************************************************************************************************
 */
#define USART1_DR_ADDRESS     ((uint32_t)0x40013804)
#define USART2_DR_ADDRESS     ((uint32_t)0x40004404)
#define USART3_DR_ADDRESS     ((uint32_t)0x40004804)

#define BUFFERSIZEDMARX			256
#define BUFFERSIZEDMATX			2048
/*
 * *************************************************************************************************
 * Function prototypes
 * *************************************************************************************************
 */
void USART3_DMA_Config(void);
void DMATxInit(void);

#endif

