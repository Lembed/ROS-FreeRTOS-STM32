#include "wiring.h"

extern "C"
{
	#include "stm32f4xx.h"
	#include "stm32f4xx_rcc.h"
}

#define STM32_TICKS_PER_US          168
#define STM32_DELAY_US_MULT         (STM32_TICKS_PER_US/3)

/**
 * @brief Delay the given number of microseconds.
 *
 * @param us Number of microseconds to delay.
 */
void delayMicroseconds(uint32_t us) {
    us *= STM32_DELAY_US_MULT;

    /* fudge for function call overhead  */
    //us--;
    asm volatile("   mov r0, %[us]          \n\t"
                 "1: subs r0, #1            \n\t"
                 "   bhi 1b                 \n\t"
                 :
                 : [us] "r" (us)
                 : "r0");
}


void digitalWrite(uint16_t pin, bool value)
{
	GPIO_TypeDef* port = (GPIO_TypeDef*)(AHB1PERIPH_BASE + (0x0400 * ((pin >> 4) & 0xF)));
	if (value == HIGH)
		port->BSRRL = 0x1 << (pin & 0xF);
	else
		port->BSRRH = 0x1 << (pin & 0xF);
}

bool digitalRead(uint16_t pin)
{
	GPIO_TypeDef* port = (GPIO_TypeDef*)(AHB1PERIPH_BASE + (0x0400 * ((pin >> 4) & 0xF)));
	return GPIO_ReadInputDataBit(port, 0x1 << (pin & 0xF));
}
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)

// Pins: 0-15: PortA
// Pins: 16-31: PortB
// ...
// Pins: 128-153: Port I
void pinMode(uint16_t pin, Mode mode)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Enable the clock */
	RCC_AHB1PeriphClockCmd(0x1 << (pin >> 4), ENABLE);
	if (mode == OUTPUT)
	{
		/* Configure the pin */
		GPIO_InitStructure.GPIO_Pin = 0x1 << (pin & 0xF);
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_TypeDef* port = (GPIO_TypeDef*)(AHB1PERIPH_BASE + (0x0400 * ((pin >> 4) & 0xF)));
		GPIO_Init(port, &GPIO_InitStructure);
	}
	else if (mode == INPUT)
	{
        GPIO_InitStructure.GPIO_Pin = 0x1 << (pin & 0xF);
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        GPIO_TypeDef* port = (GPIO_TypeDef*)(AHB1PERIPH_BASE + (0x0400 * ((pin >> 4) & 0xF)));
        GPIO_Init(port, &GPIO_InitStructure);
	}
	else if (mode == TRIGGER_FALLING || mode == TRIGGER_RISING || mode == TRIGGER_RISING_FALLING)
	{
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	    GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IN;
	    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	    GPIO_InitStructure.GPIO_Pin |= 0x1 << (pin & 0xF);
	    GPIO_TypeDef* port = (GPIO_TypeDef*)(AHB1PERIPH_BASE + (0x0400 * ((pin >> 4) & 0xF)));
	    GPIO_Init(port, &GPIO_InitStructure);

	    /* Connect Button EXTI Line to Button GPIO Pin */
	    SYSCFG_EXTILineConfig(((pin >> 4) & 0xF), pin & 0xF);

	    /* Configure Button EXTI line */
	    EXTI_InitStructure.EXTI_Line = 0x1 << (pin & 0xF);
	    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	    if (mode == TRIGGER_FALLING)
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	    else if (mode == TRIGGER_RISING)
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	    else if (mode == TRIGGER_RISING_FALLING)
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	    EXTI_Init(&EXTI_InitStructure);

	    /* Enable and set Button EXTI Interrupt to the lowest priority */
	    if ((pin & 0xF) >= 0x0 && (pin & 0xF) <= 0x4)
	    {
			if ((pin & 0xF) == 0x0)
				NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
			else if ((pin & 0xF) == 0x1)
				NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
			else if ((pin & 0xF) == 0x2)
				NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
			else if ((pin & 0xF) == 0x3)
				NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
			else if ((pin & 0xF) == 0x4)
				NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

			NVIC_Init(&NVIC_InitStructure);
	    }
	}
}

volatile uint32_t *DWT_CYCCNT   = (volatile uint32_t *)0xE0001004; //address of the register
volatile unsigned int *DWT_CONTROL  = (volatile unsigned int *)0xE0001000; //address of the register
volatile unsigned int *SCB_DEMCR    = (volatile unsigned int *)0xE000EDFC; //address of the register

void enableTiming(void)
{
    static int enabled = 0;

    if (!enabled)
    {
        *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
        *DWT_CYCCNT = 0; // reset the counter
        *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter

        enabled = 1;
    }
}

#define CYCCNT_PER_MICROSECOND 168
uint32_t micros()
{
	return (*DWT_CYCCNT) / CYCCNT_PER_MICROSECOND;
}
