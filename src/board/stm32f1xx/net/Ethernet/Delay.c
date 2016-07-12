/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "Delay.h"

__IO uint32_t TimingDelay;
uint32_t MillisCounter = 0;

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

void Delay_Ms(__IO uint32_t nTime)
{ 
	int i;
  //TimingDelay = nTime;

  //while(TimingDelay != 0);
	for ( i = 0; i < nTime * 1000; i++);
}

void Delay_us(__IO uint32_t nTime)
{
    nTime = nTime * 50;
    while (nTime > 0)
    {
        nTime--;
    }
}

void IncreaseMillis(void)
{
    MillisCounter++;
}

uint32_t Millis(void)
{
    return MillisCounter;
}
