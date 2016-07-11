
## notes

In order to add a node to the system, the node needs to be registered in apps/nodes.h.

``` cpp
#ifndef ASW_APPS_APPLICATION_TASKS_H_
#define ASW_APPS_APPLICATION_TASKS_H_

#include "nodes/ultrasonic_sensor/ultrasonic_sensor.h"
#include "nodes/imu_sensor/imu_sensor.h"
#include "nodes/my_node/my_node.h"

typedef struct node_descriptor {
	char name[32];
	void (*function)(void* params);
	int deadline; // if deadline scheduling is enabled

} node_decriptor;

node_decriptor nodes[] = {
		{"ultrasonic_sensor", ultrasonic_sensor, 50},
		{"imu_sensor", imu_sensor, 30},
		{"my_node", my_node_function, NO_DEADLINE},
};


#endif /* ASW_APPS_APPLICATION_TASKS_H_ */
```

For each function registered in nodes.h (i.e. ultrasonic_sensor, imu_sensor, and my_node_function), 
a function has to be declared and defined, which has an argument of type void*. 
Here is an example code for ultrasonic_sensor node function.


``` cpp
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
```

