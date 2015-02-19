#include "stm32f4xx_usart.h"
#include "USART_Setup.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "USART_Setup.h"
#include "DMA_Setup.h"

#define AbfrageSTACK_SIZE		configMINIMAL_STACK_SIZE

//static portTASK_FUNCTION_PROTO( vTastaturAbfrageTask, pvParameters );

// alle USARTs aufsetzen
void USART_Configuration( void ) {
  
 /*+-------------------------------------------------------------+     
   |                     |      TX          |        RX          |
   |---------------------|------------------|--------------------|             
   |       USART1        |      PB6         |        PA10        |
   |---------------------|------------------|--------------------|  
   |       USART2        |      PA2         |        PA3         |
   |---------------------|------------------|--------------------|  
   |       USART3        |      PD8         |        PD9         |
   |---------------------|------------------|--------------------|  
   |       UART4         |      PC10        |        PC11        |
   |---------------------|------------------|--------------------|
   |       UART5         |      PC12        |        PD2         |
   |---------------------|------------------|--------------------|
   |       USART6        |      PC6         |        PC7         |
   +-------------------------------------------------------------+*/    
  
  
    GPIO_InitTypeDef  GPIO_InitStructure1;  
    GPIO_InitTypeDef  GPIO_InitStructure2;
    GPIO_InitTypeDef  GPIO_InitStructure3;
    GPIO_InitTypeDef  GPIO_InitStructure4;
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStructure;  
  
    /* Enable peripheral clocks */
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
//Al    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    
    /* enable the GPIO clocks */
//Al     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
//Al     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
//Al     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

    
    //Rx USART 1 | Tx und Rx USART 2
    /*GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_10;
    GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF; 
    GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP ;
    GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure1.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &GPIO_InitStructure1);*/
          
    //Tx USART 1
    /*GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AF; 
    GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP ;
    GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOB, &GPIO_InitStructure2);*/
   
    // Tx und Rx UART 4 | TX UART 5 | TX und RX USART 6
    //GPIO_InitStructure3.GPIO_Pin = GPIO_Pin_12;/*GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_6 | GPIO_Pin_7;*/
    /*GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure3.GPIO_OType = GPIO_OType_PP ;
    GPIO_InitStructure3.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure3.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure3);*/
    
    // Tx und Rx USART 3 | RX UART 5
//Al    GPIO_InitStructure4.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_2;
    GPIO_InitStructure4.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 ;
    GPIO_InitStructure4.GPIO_Mode = GPIO_Mode_AF; 
    GPIO_InitStructure4.GPIO_OType = GPIO_OType_PP ;
    GPIO_InitStructure4.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure4.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOD, &GPIO_InitStructure4);
    
    
    // PinConfig USART 1
    //GPIO_PinAFConfig  ( GPIOB, GPIO_PinSource6  , GPIO_AF_USART1) ;
    //GPIO_PinAFConfig  ( GPIOA, GPIO_PinSource10 , GPIO_AF_USART1) ;
    
    // PinConfig USART 2
    //GPIO_PinAFConfig  ( GPIOA, GPIO_PinSource2 , GPIO_AF_USART2) ;
    //GPIO_PinAFConfig  ( GPIOA, GPIO_PinSource3 , GPIO_AF_USART2) ;
    
    // PinConfig USART 3
    GPIO_PinAFConfig  ( GPIOD, GPIO_PinSource8 , GPIO_AF_USART3) ;
    GPIO_PinAFConfig  ( GPIOD, GPIO_PinSource9 , GPIO_AF_USART3) ; 
    
    // PinConfig UART 4
    //GPIO_PinAFConfig  ( GPIOC, GPIO_PinSource10 , GPIO_AF_UART4) ;
    //GPIO_PinAFConfig  ( GPIOC, GPIO_PinSource11 , GPIO_AF_UART4) ;
    
    // PinConfig UART 5
//Al     GPIO_PinAFConfig  ( GPIOC, GPIO_PinSource12 , GPIO_AF_UART5) ;
//Al     GPIO_PinAFConfig  ( GPIOD, GPIO_PinSource2  , GPIO_AF_UART5) ;
    
    // PinConfig USART 6
    //GPIO_PinAFConfig  ( GPIOC, GPIO_PinSource6  , GPIO_AF_USART6) ;
    //GPIO_PinAFConfig  ( GPIOC, GPIO_PinSource7  , GPIO_AF_USART6) ;
    
   // Setup transmit complete irq.
    //USART_ITConfig( USART1, USART_IT_TC, ENABLE );
    //USART_ITConfig( USART2, USART_IT_TC, ENABLE );
    USART_ITConfig( USART3, USART_IT_TC, ENABLE );
    //USART_ITConfig( UART4, USART_IT_TC, ENABLE );
//Al    USART_ITConfig( UART5, USART_IT_TC, ENABLE );
    //USART_ITConfig( USART6, USART_IT_TC, ENABLE );

    // Clocks
    USART_ClockStructInit(&USART_ClockInitStructure);
    //USART_ClockInit(USART1, &USART_ClockInitStructure);
    //USART_ClockInit(USART2, &USART_ClockInitStructure);
    USART_ClockInit(USART3, &USART_ClockInitStructure);
    //USART_ClockInit(UART4, &USART_ClockInitStructure);
//Al    USART_ClockInit(UART5, &USART_ClockInitStructure);
    //USART_ClockInit(USART6, &USART_ClockInitStructure);
    
    // USART initialisieren
    /*USART_InitStructure.USART_BaudRate = USART_BaudRate_9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_2;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);
    USART_Init(USART2, &USART_InitStructure);
    USART_Init(USART1, &USART_InitStructure);
    USART_Init(USART6, &USART_InitStructure);*/

    //USART1 und 3     
    USART_InitStructure.USART_BaudRate = USART_BaudRate_115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
//Al    USART_Init(UART5, &USART_InitStructure);
    

    //USART ItConfig
    //USART_ITConfig( USART1, USART_IT_RXNE, ENABLE);
    //USART_ITConfig( USART1, USART_IT_TXE,  ENABLE);
    
    //USART_ITConfig( USART2, USART_IT_RXNE, ENABLE);
    //USART_ITConfig( USART2, USART_IT_TXE,  ENABLE);
    
    USART_ITConfig( USART3, USART_IT_RXNE, ENABLE);
    USART_ITConfig( USART3, USART_IT_TXE,  ENABLE);
    
    //USART_ITConfig( UART4, USART_IT_RXNE, ENABLE);
    //USART_ITConfig( UART4, USART_IT_TXE,  ENABLE);
    
//Al    USART_ITConfig( UART5, USART_IT_RXNE, ENABLE);
//Al    USART_ITConfig( UART5, USART_IT_TXE,  ENABLE);
    
    //USART_ITConfig( USART6, USART_IT_RXNE, ENABLE);
    //USART_ITConfig( USART6, USART_IT_TXE,  ENABLE);
    
//    //NVIC
//    NVIC_InitTypeDef NVIC_InitStructure;
//
//        /* Enable the USARTx Interrupt */
//    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
    
    // Enable the USART
    //USART_Cmd(USART1, ENABLE);
    //USART_Cmd(USART2, ENABLE);
    USART_Cmd(USART3, ENABLE);
    //USART_Cmd(UART4 , ENABLE);
//Al    USART_Cmd(UART5 , ENABLE);
    //USART_Cmd(USART6, ENABLE);
    
}
 /*+-------------------------------------------------------------+*/

