/*! ************************************************************************************************
 *
 * \file
 *
 * (C) All rights reserved by Institute of Automatic Control, Leibniz University Hannover
 *
 * *************************************************************************************************
 *
 * 	File:			DMA_Setup.c
 *
 * 	Purpose:	    DMA setup of the Shell for input and output.
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
#include "stm32f4_discovery.h"
#include <stdio.h>
#include "USART_Setup.h"
#include "stm32f4xx_dma.h"
#include "DMA_Setup.h"
/*
 ***************************************************************************************************
 * Variables
 ***************************************************************************************************
 */
char g_c_DMA_USART3Buffer1[BUFFERSIZEDMARX];
char *g_cptr_DMA_tempBsptr;
char *g_cptr_DMA_Momptr1;
char *g_cptr_DMA_Momptr2;
int  g_i16_DMA_msgvalue;
char g_c_DMAOUTPUT1[BUFFERSIZEDMATX];
char g_c_DMAOUTPUT2[BUFFERSIZEDMATX];
/*
 * *************************************************************************************************
 * Functions
 * *************************************************************************************************
 */
void USART3_DMA_Config(void)
{
  g_cptr_DMA_tempBsptr = g_c_DMAOUTPUT1;
  g_cptr_DMA_Momptr1 = g_c_DMAOUTPUT1;
  g_cptr_DMA_Momptr2 = g_c_DMAOUTPUT2;
  DMA_InitTypeDef       DMA_InitStructure;
  DMA_InitTypeDef       DMA_InitStructure1;
  /* Enable DMA1 and clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  /* DMA1 Stream1 channel4 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USART3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&g_c_DMA_USART3Buffer1;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = size_DMA_Buffer_Shell;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream1, &DMA_InitStructure);
  DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);

  /* DMA1 Stream3 channel4 configuration **************************************/
  DMA_InitStructure1.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure1.DMA_PeripheralBaseAddr = (uint32_t)USART3_DR_ADDRESS;
  DMA_InitStructure1.DMA_Memory0BaseAddr = (uint32_t)g_cptr_DMA_tempBsptr;
  DMA_InitStructure1.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure1.DMA_BufferSize = g_i16_DMA_msgvalue;
  DMA_InitStructure1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure1.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure1.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure1.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure1.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure1.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure1.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure1.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure1.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure1.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream3, &DMA_InitStructure1);
  DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);

  /* Enable USART3 DMA */
  USART_DMACmd(USART3,USART_DMAReq_Rx, ENABLE);
  USART_DMACmd(USART3,USART_DMAReq_Tx, ENABLE);
  DMA_Cmd(DMA1_Stream1, ENABLE);
  //DMA_Cmd(DMA1_Stream3, ENABLE);
}

void DMATxInit(void)
{
	  DMA_InitTypeDef       DMA_InitStructure1;
	  DMA_InitStructure1.DMA_Channel = DMA_Channel_4;
	  DMA_InitStructure1.DMA_PeripheralBaseAddr = (uint32_t)USART3_DR_ADDRESS;
	  DMA_InitStructure1.DMA_Memory0BaseAddr = (uint32_t)g_cptr_DMA_tempBsptr;
	  DMA_InitStructure1.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	  DMA_InitStructure1.DMA_BufferSize = g_i16_DMA_msgvalue;
	  DMA_InitStructure1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	  DMA_InitStructure1.DMA_MemoryInc = DMA_MemoryInc_Enable;
	  DMA_InitStructure1.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	  DMA_InitStructure1.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	  DMA_InitStructure1.DMA_Mode = DMA_Mode_Normal;
	  DMA_InitStructure1.DMA_Priority = DMA_Priority_Medium;
	  DMA_InitStructure1.DMA_FIFOMode = DMA_FIFOMode_Disable;
	  DMA_InitStructure1.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	  DMA_InitStructure1.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	  DMA_InitStructure1.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	  DMA_Init(DMA1_Stream3, &DMA_InitStructure1);
	  DMA_ClearFlag(DMA1_Stream3, DMA_IT_TCIF3);
	  DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
	  DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
	  USART_DMACmd(USART3,USART_DMAReq_Tx, ENABLE);
	  DMA_Cmd(DMA1_Stream3, ENABLE);
	  DMA_Cmd(DMA1_Stream3, ENABLE);
}

/*void DMA1_IRQHandler(void)
{
  while(USART_GetITStatus(USART3, USART_IT_RXNE) == RESET)
  {
	  USART_ReceiveData(USART3);
	  DMA_Cmd(DMA1_Stream1, ENABLE);
	  while (DMA_GetFlagStatus(DMA1_Stream1, DMA_FLAG_TCIF1) != RESET)
	    {
			DMA_ClearFlag(DMA1_Stream1, DMA_IT_TCIF1);
			DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
			DMA_Cmd(DMA1_Stream3, ENABLE);
	  	    while(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)
	  	      {

	  		    char ch1;
	  		    out_char (ch1);
	  		    DMA_ClearFlag(DMA1_Stream3, DMA_IT_TCIF3);
	  		    DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
	          }
	    }
  }
}
*/
