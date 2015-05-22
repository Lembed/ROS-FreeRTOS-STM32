#include <string.h>
#include "xmlrpc.h"

extern "C" void os_printf(char* fmt, ...);

#include <xmlrpc/XMLRPCServer.h>
#include "msg.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/ColorRGBA.h"
#include "sensor_msgs/Range.h"
#include "rcl.h"
using namespace std_msgs;
using namespace sensor_msgs;

class MySubscriber
{
	void(*callback)(String& data);
public:
	static void subCallback(void* data, void* obj)
	{
		MySubscriber* self = (MySubscriber*) obj;
		String msg;
		msg.deserialize((unsigned char*)data);
		self->callback(msg);
	}

	MySubscriber(const char* topic, void(*callback)(String& data))
	{
		TopicReader* tr = XMLRPCServer::registerSubscriber("listener", topic, "std_msgs/String");
		this->callback = callback;
		tr->addCallback(subCallback, this);
	}
};

void mycallback(String& msg)
{
	os_printf("Received: %s\n", msg.data);
}

void xmlrpc_task(void* p)
{
	XMLRPCServer::start();
	//TopicWriter* tw2 = XMLRPCServer::registerPublisher("talker2", "chatter2", "std_msgs/String");
	//TopicReader* tr = XMLRPCServer::registerSubscriber("listener", "chatter", "std_msgs/String");

	MySubscriber("chatter", mycallback);

	/*char string[] = "Hello ROS!";
	String str;
	str.data = string;*/

	/*Int32 str;
	str.data = 32;*/

	/*ColorRGBA str;
	str.r = 200;
	str.g = 100;
	str.b = 22;
	str.a = 234;*/


	Range str;
	str.radiation_type = Range::ULTRASOUND;
	str.min_range = 0.03f;
	str.max_range = 2.0f;
	str.range = 0.5f;

	char string[] = "Hello ROS!";
	String str1;
	str1.data = string;


	//TODO: Why does system crash with rostopic echo chatter?
	//TODO: Connection won't be initialized if PC side subscriber is created before STM32 side publisher.

	LOOP(200,
			//publishMsg(str1, &endpoint1);
			//tw->publishMsg(str1);
			//tw2->publishMsg(str);
			vTaskDelay(200);
	)

	vTaskDelete(NULL);
}
