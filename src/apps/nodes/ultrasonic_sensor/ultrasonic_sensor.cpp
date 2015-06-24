#include "ultrasonic_sensor.h"

#include "rcl.h"
#include "Node.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "sensor_msgs/Range.h"
#include "HCSR04Sensor/HCSR04.h"

using namespace sensor_msgs;

ros::Publisher* ultrasonic_pub;

void ultrasonicLoop()
{
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
	ros::Node* n = new ros::Node("ultrasonic_sensor");
	ultrasonic_pub = new ros::Publisher;
	ultrasonic_pub->advertise<Range>(n, "ultrasound");

	HCSR04::init();
	spinLoop(ultrasonicLoop, 300);
}
