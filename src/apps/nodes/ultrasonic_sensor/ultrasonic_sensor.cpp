#include "ultrasonic_sensor.h"
#include "rcl.h"
#include "Node.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "sensor_msgs/Range.h"
#include "HCSR04Sensor/HCSR04.h"

// Period in milliseconds
#define PUBLISH_PERIOD 300

using namespace sensor_msgs;

ros::Publisher* ultrasonic_pub;

void ultrasonicLoop()
{
    Range msg;
    msg.radiation_type = Range::ULTRASOUND;
    msg.min_range = 0.03f;
    msg.max_range = 2.0f;

    // Get distance value acquired by ultrasonic sensor in meters.
    float distance_m = HCSR04::pingMedian(5)/100.0f;

    if (distance_m > -1)
    {
        msg.range = distance_m;
        ultrasonic_pub->publish(msg);
    }
}

void ultrasonic_sensor(void* params)
{
    // Register node in the ROS system and create a publisher with ultrasound topic.
    ros::Node* n = new ros::Node("ultrasonic_sensor");
    ultrasonic_pub = new ros::Publisher;
    ultrasonic_pub->advertise<Range>(n, "ultrasound");

    // Initialize sensor.
    HCSR04::init();

    // Begin periodic loop with PUBLISH_PERIOD in milliseconds.
    spinLoop(ultrasonicLoop, PUBLISH_PERIOD);

    // Code never reaches here, deleting allocated memory is not necessary.
}
