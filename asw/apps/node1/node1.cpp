#include "node1.h"
#include "rcl.h"
#include "Node.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "math.h"
using namespace ros;
using namespace std_msgs;


Publisher* sqrt_pub;
void sqrtCallback(const Int32& msg)
{
	// Get data from msg.
	int32_t num = msg.data;
	os_printf("Received number %d!\n", msg.data);
	// Set data for msg to be published.
	Float32 m;
	m.data = (float)sqrt((double) msg.data);
	// Publish msg to "sqrt" topic.
	sqrt_pub->publish(m);
}
unsigned long counter = 0;
void myloop()
{
	counter++;
	os_printf("Counter:%d\n", counter);
}



void node1(void* params)
{
	Node* n = new ros::Node("nodeB"); // Register node with the name 'nodeB' in RCL.
	sqrt_pub = new ros::Publisher;
	sqrt_pub->advertise<Float32>(n, "sqrt"); // Advertise to "sqrt" topic.
	ros::Subscriber<Int32>* sub = new ros::Subscriber<Int32>(n, "sub", sqrtCallback); // Subscribe to "sub" topic.
	spinLoop(myloop, 250); // Spin node for being able to receive subscriber callbacks and run loop with period of 250ms.

}
