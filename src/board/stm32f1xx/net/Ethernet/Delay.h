#ifndef __DELAY_H__
#define __DELAY_H__

#include "stm32f10x.h"

extern __IO uint32_t TimingDelay;
extern uint32_t MillisCounter;

void TimingDelay_Decrement(void);

void Delay_Ms(__IO uint32_t nTime);
void Delay_us(__IO uint32_t nTime);

void IncreaseMillis(void);
uint32_t Millis(void);

#endif /* __DELAY_H__ */
