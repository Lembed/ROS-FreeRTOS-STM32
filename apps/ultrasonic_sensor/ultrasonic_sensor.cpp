#include "ultrasonic_sensor.h"

#include "rcl.h"
#include "Node.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "sensor_msgs/Range.h"

#include "wiring.h"

#define ECHO_PIN GPIO_PA0
#define TRIG_PIN GPIO_PC6


xSemaphoreHandle sensorReadSignal;

void SR04_Init(void)
{
	// TODO: Why does publishing not work without this delay hack?
	vTaskDelay(4000);
	vSemaphoreCreateBinary(sensorReadSignal);
	pinMode(GPIO_PC6, OUTPUT);
	pinMode(ECHO_PIN, TRIGGER_RISING_FALLING);
	// hold TRIG pin low
	digitalWrite(TRIG_PIN, LOW);
}

#define HCSR04_MAX_RANGE 300
#define HCSR04_NUMBER ((float)0.0171821)

volatile uint32_t ultrasound_duration;
uint32_t lastTick = 0;

extern "C"
void EXTI0_IRQHandler(void)
{
	long taskWoken = 0;
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		bool isRisingEdge = digitalRead(GPIO_PA0);
		//digitalWrite(GPIO_PD11, isRisingEdge);

		if (isRisingEdge)
		{
			lastTick = micros()/168;
		}
		else
		{
			ultrasound_duration =  micros()/168 - lastTick;
			if (ultrasound_duration > 30000)
				ultrasound_duration = 30000;
			xSemaphoreGiveFromISR(sensorReadSignal, &taskWoken);
		}

	    EXTI_ClearITPendingBit(EXTI_Line0);
	}
	if (taskWoken)
		vPortYieldFromISR();
}

float ping()
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
	float distance = -1;
	if (xSemaphoreTake(sensorReadSignal, 50))
	{
		// Get distance in us and convert us to cm
		distance =  (float)ultrasound_duration * HCSR04_NUMBER;
	}

	if (distance > HCSR04_MAX_RANGE)
		return HCSR04_MAX_RANGE;

	/* Return distance */
	return distance;
}

using namespace sensor_msgs;

#define NO_ECHO -1
#define PING_MEDIAN_PERIOD 40
float pingMedian(uint8_t it) {
	float uS[it], last;
	uint8_t j, i = 0;
	unsigned long t;
	uS[0] = NO_ECHO;
	portTickType xLastWakeTime=xTaskGetTickCount();
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


ros::Publisher* ultrasonic_pub;
void ultrasonicLoop()
{
	Range msg;
	msg.radiation_type = Range::ULTRASOUND;
	msg.min_range = 0.03f;
	msg.max_range = 2.0f;
	float distance_m = pingMedian(5)/100.0f; //ping() / 100.0f;

	if (distance_m > -1)
	{
		msg.range = distance_m;
		if (msg.range < 0.03f)
			msg.range = 0.03f;
		ultrasonic_pub->publish(msg);
	}
}

void ultrasonic_sensor(void* params)
{
	ros::Node* n = new ros::Node("nodeD");
	ultrasonic_pub = new ros::Publisher;
	ultrasonic_pub->advertise<Range>(n, "ultrasound");


	SR04_Init();
	spinLoop(ultrasonicLoop, 100);
}
