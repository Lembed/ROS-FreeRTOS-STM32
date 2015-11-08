# Program Real-Time ROS Nodes on STM32

## Description
An embedded system supporting multiple ROS nodes that are able to communicate with one another using inter-process communication and with other ROS-compatible devices over a network. This system consists of an STM32F4Discovery board, the FreeRTOS operating system, an embedded ROS middleware, and an embedded ROS client library. Application layer is the layer for ROS nodes and sensor libraries.

With this software, ROS developers do not have to know the complexities of the real-time operating system. ROS Client Library allows ROS developers to create Nodes, Publishers, Subscribers, and define ROS messages. This is why they can program their nodes as if they were writing code on a general-purpose computer.

The software includes two different scheduling schemes: priority and deadline. If deadline scheduling is enabled, application developers can assign deadlines to periodic tasks in order to make sure that they are completed before this deadline as long as the system is not overloaded. 

|           Layers          |
| ---------------------------------| 
|           application           |
|        ROS Client Library       |
|         ROS Middleware          |
|            XMLRPC+UDP           |
|             FreeRTOS            |
|        Hardware (STM32F4)       |

## Software Concept

The software concept is shown in the below image. Blue blocks represent FreeRTOS tasks.

![](https://github.com/bosch-ros-pkg/stm32/blob/master/doc/ROS.png)

Node functions are specified in the header file (node table) nodes.h, where developers need to type in the entry point of each ROS node on embedded system manually. Data transfer between PC and embedded system is realized using <a href="http://wiki.ros.org/ROS/UDPROS">UDPROS</a> over Ethernet. There is a very simple XMLRPC handling, which does not use any XML parser, to make negotiations with ROS master on a PC. ROS messages are serialized on embedded software using C++ headers generated with <a href="http://wiki.ros.org/rosserial">rosserial</a>'s message generator based on Python.

## Example Node Function
Node functions are located in apps/nodes. In order to add a node to the system, the node needs to be registered in apps/nodes.h.

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

For each function registered in nodes.h (i.e. ultrasonic_sensor, imu_sensor, and my_node_function), a function has to be declared and defined, which has an argument of type void*. Here is an example code for ultrasonic_sensor node function.


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

## How to Setup

A tutorial on how to set up the system is given <a href="https://github.com/bosch-ros-pkg/stm32/wiki">here</a>.
