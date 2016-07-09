#include <libs/HCSR04Sensor/HCSR04.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "wiring.h"


#define ECHO_PIN GPIO_PA0
#define TRIG_PIN GPIO_PC6
#define HCSR04_MAX_RANGE 500
#define HCSR04_MIN_RANGE 3
#define HCSR04_NUMBER ((float)0.0171821)
#define MAX_ULTRASOUND_DURATION_US   HCSR04_MAX_RANGE / HCSR04_NUMBER
#define MAX_ULTRASOUND_DURATION_MS (uint32_t)(MAX_ULTRASOUND_DURATION_US / 1000.0f)

volatile uint32_t ultrasound_duration = 0;
uint32_t lastMicros = 0;

xSemaphoreHandle sensorReadSignal;

void HCSR04::init(void)
{
	// TODO: Why does publishing not work without this delay hack?
	vTaskDelay(4000);
	vSemaphoreCreateBinary(sensorReadSignal);
	pinMode(GPIO_PC6, OUTPUT);
	pinMode(ECHO_PIN, TRIGGER_RISING_FALLING);
	// hold TRIG pin low
	digitalWrite(TRIG_PIN, LOW);
}

extern "C"
void EXTI0_IRQHandler(void)
{
	long taskWoken = 0;
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		bool isRisingEdge = digitalRead(GPIO_PA0);
		//digitalWrite(GPIO_PD11, isRisingEdge);

		if (isRisingEdge) {
			lastMicros = micros();
		} else {
			ultrasound_duration =  micros() - lastMicros;

			if (ultrasound_duration > MAX_ULTRASOUND_DURATION_US)
				ultrasound_duration = MAX_ULTRASOUND_DURATION_US;
			xSemaphoreGiveFromISR(sensorReadSignal, &taskWoken);
		}

		EXTI_ClearITPendingBit(EXTI_Line0);
	}
	if (taskWoken)
		vPortYieldFromISR();
}



float HCSR04::ping()
{
	taskDISABLE_INTERRUPTS();
	/* Trigger low */
	digitalWrite(TRIG_PIN, LOW);
	/* Delay 2 us */
	delayMicroseconds(2);
	/* Trigger high for 10us */
	digitalWrite(TRIG_PIN, HIGH);
	/* Delay 10 us */
	delayMicroseconds(10);
	/* Trigger low */
	digitalWrite(TRIG_PIN, LOW);
	taskENABLE_INTERRUPTS();
	float distance = NO_ECHO;
	// TODO: Depending on the period, this might cause extra delays!
	if (xSemaphoreTake(sensorReadSignal, MAX_ULTRASOUND_DURATION_MS))	{
		// Get distance in us and convert us to cm
		distance =  (float)ultrasound_duration * HCSR04_NUMBER;
	}
	// every value > range is out of range!
	if (distance > HCSR04_MAX_RANGE)
		return NO_ECHO;

	if (distance < HCSR04_MIN_RANGE)
		return HCSR04_MIN_RANGE;

	/* Return distance */
	return distance;
}


#define PING_MEDIAN_PERIOD 40
float HCSR04::pingMedian(uint8_t it)
{
	float uS[it], last;
	uint8_t j, i = 0;
	unsigned long t;
	uS[0] = NO_ECHO;
	portTickType xLastWakeTime = xTaskGetTickCount();
	while (i < it) {
		last = ping(); // Send ping.

		if (last != NO_ECHO) { // Ping in range, include as part of median.
			if (i > 0) {               // Don't start sort till second ping.
				for (j = i; j > 0 && uS[j - 1] < last; j--) // Insertion sort loop.
					uS[j] = uS[j - 1]; // Shift ping array to correct position for sort insertion.
			} else j = 0;              // First ping is sort starting point.
			uS[j] = last;              // Add last ping to array in sorted position.
			i++;                       // Move to next ping.
		} else it--;           // Ping out of range, skip and don't include as part of median.

		// Millisecond delay between pings.
		vTaskDelayUntil(&xLastWakeTime, PING_MEDIAN_PERIOD);

	}
	return (uS[it >> 1]); // Return the ping distance median.
}

