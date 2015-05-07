#include "node2.h"
#include "rcl.h"
#include "Node.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "std_msgs/Float32.h"

using namespace ros;
using namespace std_msgs;

Publisher *square_pub;

void print_float(float f)
{
	float square = f * 100;
	long square_int = (long)(square / 100);
	long period = ((long)square) % 100;
	os_printf("Squared:%d.%d\n", square_int, period);
}

void squareCallback(const Float32& msg)
{
	// Get data from msg.
	float f = msg.data;
	print_float(f*f); // Since formatting float (%f) does not work, we use a workaround
	// Set data for msg to be published.
	Float32 m;
	m.data = f*f;
	// Publish msg to "squared" topic.
	square_pub->publish(m);
}
void loop2()
{

}

void node2(void* params)
{
	ros::Node* n = new ros::Node("nodeC"); // Register node with the name 'nodeC' in RCL.
	square_pub = new ros::Publisher;
	square_pub->advertise<Float32>(n, "squared");// Advertise to "squared" topic.
	ros::Subscriber<Float32>* sub = new ros::Subscriber<Float32>(n, "sqrt", squareCallback); // Subscribe to "sqrt" topic.

    spinLoop(loop2, 250);

}
