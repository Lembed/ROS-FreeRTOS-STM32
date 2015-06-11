#include "node1.h"

#include "rcl.h"
#include "Node.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "math.h"
#include "msg.h"
#include "std_msgs/String.h"
using namespace ros;
using namespace std_msgs;


#include "wiring.h"

bool pin = true;
bool pin2 = true;

Publisher* sqrt_pub;
void sqrtCallback(const Int32& msg)
{
	digitalWrite(GPIO_PD9, pin2);
	// Get data from msg.
	int32_t num = msg.data;
	//os_printf("Received number %d!\n", msg.data);
	// Set data for msg to be published.
	Float32 m;
	m.data = (float)sqrt((double) msg.data);
	// Publish msg to "sqrt" topic.
	sqrt_pub->publish(m);
	pin2 = !pin2;
}

ros::Publisher* pub;

void testCallback(const String& msg)
{
	os_printf("Received string: %s\n", msg.data);
	pub->publish(msg);
}

unsigned long counter = 0;



void myloop()
{
	digitalWrite(GPIO_PD11, pin);

	/*counter++;
	os_printf("Counter:%d\n", counter);
	char message[16];
	sprintf(message, "%d", counter);
	String msg;
	msg.data = message;
	pub->publish(msg);*/
	//digitalWrite(GPIO_PD11, HIGH);
	//counter++;
	//os_printf("Counter:%d\n", counter);
	pin = !pin;

}


/*char string[] = "Hello ROS!";
String str;
str.data = string;*/

/*Int32 str;
str.data = 32;*/

/*ColorRGBA str;
str.r = 200;
str.g = 100;
str.b = 22;
str.a = 234;


Range str;
str.radiation_type = Range::ULTRASOUND;
str.min_range = 0.03f;
str.max_range = 2.0f;
str.range = 0.5f;

char string[] = "Hello ROS!";
String str1;
str1.data = string;*/



void node1(void* params)
{
	Node* n = new ros::Node("nodeB"); // Register node with the name 'nodeB' in RCL.
	//pub = new ros::Publisher;
	//pub->advertise<String>(n, "chatter5");

	sqrt_pub = new ros::Publisher;
	sqrt_pub->advertise<Float32>(n, "sqrt");
	pinMode(GPIO_PD11, OUTPUT);
	pinMode(GPIO_PD9, OUTPUT);

	//sqrt_pub->advertise<Float32>(n, "sqrt"); // Advertise to "sqrt" topic.
	ros::Subscriber<Int32>* sub = new ros::Subscriber<Int32>(n, "counter", sqrtCallback); // Subscribe to "sub" topic.
	//ros::Subscriber<String>* sub = new ros::Subscriber<String>(n, "chatter", testCallback); // Subscribe to "sub" topic.


	spinLoop(myloop, 10); // Spin node for being able to receive subscriber callbacks and run loop with period of 250ms.

}
