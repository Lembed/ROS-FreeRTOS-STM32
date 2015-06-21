#include "node2.h"

#include "rcl.h"
#include "Node.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"


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
uint32_t square = 0;
float sqrtNum = 0;
void squareCallback(const Float32& msg)
{
	// Get data from msg.
	float f = msg.data;
	//os_printf("Squared:%d\n", (uint32_t)(f*f));

	//print_float(f*f); // Since formatting float (%f) does not work, we use a workaround
	// Set data for msg to be published.
	//Float32 m;
	//m.data = f*f;
	// Publish msg to "squared" topic.
	//square_pub->publish(m);
	square = f*f;
	sqrtNum = f;
}
void loop2()
{
	os_printf("int:%d\n", square);
	print_float(sqrtNum);
}

void node2(void* params)
{
	ros::Node* n = new ros::Node("nodeC"); // Register node with the name 'nodeC' in RCL.
	//square_pub = new ros::Publisher;
	//square_pub->advertise<Float32>(n, "squared");// Advertise to "squared" topic.
	ros::Subscriber<Float32>* sub = new ros::Subscriber<Float32>(n, "sqrt", squareCallback); // Subscribe to "sqrt" topic.

	//square_pub = new ros::Publisher;
	//square_pub->advertise<Float32>(n, "squared");

    spinLoop(loop2, 250);

}
