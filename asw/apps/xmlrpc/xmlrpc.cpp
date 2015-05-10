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

static uint16_t xmlPort = 40000;
static uint16_t tcpPort = 50000;

ConnectionHandler* createPublisher(const char* nodeName, const char* topicName, const char* msgType)
{
	XMLRPCHandler::registerPublisher(nodeName, topicName, msgType, xmlPort);
	ConnectionHandler* connection = new ConnectionHandler;
	connection->initPublisherEndpoint(tcpPort);
	XMLRPCHandler::waitForRequest(xmlPort, tcpPort);

	xmlPort++;
	tcpPort++;
	return connection;
}

void xmlrpc_task(void* p)
{
	ConnectionHandler* connection = createPublisher("rostopic_4767_1316912741557", "chatter", "std_msgs/String");
	ConnectionHandler* connection2 = createPublisher("rostopic_476", "chatter2", "std_msgs/String");

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

	unsigned char stream[100];
	serializeMsg(str, stream);
	unsigned char stream2[100];
	serializeMsg(str, stream2);


	//TODO: Why does system crash with rostopic echo chatter?
	//TODO: Connection won't be initialized if PC side subscriber is created before STM32 side publisher.
	for (;;)
	{
		connection->sendMessage((const char*)stream);
		connection2->sendMessage((const char*)stream2);
		vTaskDelay(500);
	}

	vTaskDelete(NULL);
}

