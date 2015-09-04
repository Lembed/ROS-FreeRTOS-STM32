#include "new_task.h"
#include "rcl.h"
#include "Node.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "std_msgs/Int32.h"

// Period in milliseconds
#define PUBLISH_PERIOD 15

using namespace std_msgs;
using namespace ros;
Publisher* new_pub;

void newLoop()
{
    static int counter=0;
    for (volatile int i = 0; i< 30000; i++);
    Int32 msg;
    msg.data = counter++;
    new_pub->publish(msg);
}

void new_task(void* p)
{
	// Register node in the ROS system and create a publisher with imu topic.
    Node* n = new Node("new_task");
    new_pub = new Publisher;
    new_pub->advertise<Int32>(n, "new_task");

	// Begin periodic loop with PUBLISH_PERIOD in milliseconds.
    spinLoop(newLoop, PUBLISH_PERIOD);

	// Code never reaches here, deleting allocated memory is not necessary.
}
