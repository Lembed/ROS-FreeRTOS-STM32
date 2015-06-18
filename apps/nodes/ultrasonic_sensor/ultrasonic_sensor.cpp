#include "ultrasonic_sensor.h"

#include "rcl.h"
#include "Node.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "sensor_msgs/Range.h"
#include "HCSR04Sensor/HCSR04.h"

using namespace sensor_msgs;

ros::Publisher* ultrasonic_pub;

#include "Queue.h"
Queue* testQ;
#define QUEUE_LENGTH 20
#define QUEUE_SIZE 4
uint32_t counter1 = 0;
void ultrasonicLoop()
{
	counter1++;
	testQ->enqueue((void*) &counter1);
	os_printf("enqueue %d!\n", counter1);
	return;
	Range msg;
	msg.radiation_type = Range::ULTRASOUND;
	msg.min_range = 0.03f;
	msg.max_range = 2.0f;
	float distance_m = HCSR04::pingMedian(5)/100.0f; //ping() / 100.0f;

	if (distance_m > -1)
	{
		msg.range = distance_m;
		ultrasonic_pub->publish(msg);
	}
}

void ultrasonic_sensor(void* params)
{
	ros::Node* n = new ros::Node("nodeD");
	ultrasonic_pub = new ros::Publisher;
	ultrasonic_pub->advertise<Range>(n, "ultrasound");

	testQ = new Queue(QUEUE_LENGTH, QUEUE_SIZE);
	//HCSR04::init();
	spinLoop(ultrasonicLoop, 30);
}
