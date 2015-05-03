#include "ultrasonic_sensor.h"
#include "rcl.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SR04_TRIG   (1<<6)
#define SR04_ECHO   (1<<7)
#define STM32_TICKS_PER_US          168
#define STM32_DELAY_US_MULT         (STM32_TICKS_PER_US/3)

/**
 * @brief Delay the given number of microseconds.
 *
 * @param us Number of microseconds to delay.
 */
static inline void delay_us(uint32_t us) {
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

/*
  Configure SR04 GPIO
 */
void SR04_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  // configuring clock sources for GPIOC
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* Configure SR04 pins: PC6 - TRIGGER */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure SR04 pins: PC7 - ECHO */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // hold TRIG pin low
  GPIO_ResetBits(GPIOC, SR04_TRIG);
}
#define HCSR04_MAX_RANGE 200
#define HCSR04_TIMEOUT 50000
#define HCSR04_NUMBER ((float)0.0171821)
#define HCSR04_MAX_ECHO_DURATION HCSR04_MAX_RANGE * 59
float SR04_Read()
{
	taskDISABLE_INTERRUPTS();
	uint32_t time, timeout;
	/* Trigger low */
	GPIOC->BSRRH = SR04_TRIG;
	/* Delay 2 us */
	delay_us(2);
	/* Trigger high for 10us */
	GPIOC->BSRRL = SR04_TRIG;
	/* Delay 10 us */
	delay_us(10);
	/* Trigger low */
	GPIOC->BSRRH = SR04_TRIG;


	/* Give some time for response
	timeout = HCSR04_TIMEOUT;
	while (GPIO_ReadInputDataBit(GPIOC, SR04_ECHO)) {
		if (timeout-- == 0x00) {
			return -1;
		}
	}*/
	/* Give some time for response */
	timeout = HCSR04_TIMEOUT;
	while (!GPIO_ReadInputDataBit(GPIOC, SR04_ECHO)) {
		if (timeout-- == 0x00) {
			return 0;
		}
	}
	/* Start time */
	time = 0;
	/* Wait till signal is low */
	while (GPIO_ReadInputDataBit(GPIOC, SR04_ECHO)) {
		/* Increase time and check for max duration */
		if (time++ > 10000)
			return HCSR04_MAX_RANGE;
		/* Delay 1us */
		delay_us(1);

	}
	taskENABLE_INTERRUPTS();
	/* Convert us to cm */
	float distance =  (float)time * HCSR04_NUMBER;

	if (distance > HCSR04_MAX_RANGE)
		return HCSR04_MAX_RANGE;

	/* Return distance */
	return distance;
}

void ultrasonic_sensor(void* params)
{
	// Periodic code goes here.
	// First argument: Period.
	// Second argument periodic code.
	// Note: A periodic loop must always be included.
	Node* node = createNode("nodeD");
	Publisher *square_pub = createPublisher(node, "squared", RCL_MSG_TYPE_FLOAT);
	SR04_Init();
	LOOP(200,

	float distance_cm = SR04_Read();
	//if (distance_cm > -1 && distance_cm < 200)
	//os_printf("Distance: %d cm\n", (long)distance_cm);

	if (distance_cm > -1)
	{
    //os_printf("Distance: %d cm\n", (long)distance_cm);
    *((float*)square_pub->msg->data) = distance_cm;
    publish(square_pub);
	}
	// start while
	//os_printf("Distance:\t%d\tcm", SR04read());
	// end while
	)
}
