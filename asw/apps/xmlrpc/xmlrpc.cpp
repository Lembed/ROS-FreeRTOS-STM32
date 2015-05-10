#include <string.h>
#include "xmlrpc.h"

extern "C" void os_printf(char* fmt, ...);

#include "XMLRPCHandler.h"
#include "ConnectionHandler.h"
#include "msg.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/ColorRGBA.h"
#include "sensor_msgs/Range.h"
using namespace std_msgs;
using namespace sensor_msgs;

void serializeMsg(const ros::Msg& msg, unsigned char* outbuffer)
{
	unsigned char stream1[100];
	uint32_t offset = msg.serialize(stream1);
	memcpy(outbuffer, &offset, sizeof(uint32_t));
	memcpy(outbuffer+sizeof(uint32_t), stream1, offset);
}

void xmlrpc_task(void* p)
{
	XMLRPCHandler::registerPublisher("rostopic_4767_1316912741557", "chatter", "std_msgs/String", "http://10.3.84.99:47855/");
	//XMLRPCHandler::registerSubscriber("rostopic_4767", "chatter", "std_msgs/String", "http://10.3.84.99:47857/");
	//XMLRPCHandler::registerPublisher("rostopic_4767_13169", "chatter", "std_msgs/String", "http://10.3.84.99:47856/");

	vTaskDelay(2000);
	ConnectionHandler* connection = new ConnectionHandler;
	connection->initPublisherEndpoint(50000);

	XMLRPCHandler::waitForRequest(47855);

	char string[] = "Hello ROS!";
	/*String str;
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

	unsigned char stream[100];
	serializeMsg(str, stream);


	//TODO: Why does system crash with rostopic echo chatter?
	//TODO: Connection won't be initialized if PC side subscriber is created before STM32 side publisher.
	for (;;)
	{
		connection->sendMessage((const char*)stream);
		vTaskDelay(500);
	}

	vTaskDelete(NULL);
}

