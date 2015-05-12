#include <string.h>
#include "xmlrpc.h"

extern "C" void os_printf(char* fmt, ...);

#include <xmlrpc/XMLRPCServer.h>
#include "ConnectionHandler.h"
#include "msg.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/ColorRGBA.h"
#include "sensor_msgs/Range.h"
#include "rcl.h"
using namespace std_msgs;
using namespace sensor_msgs;

void serializeMsg(const ros::Msg& msg, unsigned char* outbuffer)
{
	unsigned char stream1[100];
	uint32_t offset = msg.serialize(stream1);
	memcpy(outbuffer, &offset, sizeof(uint32_t));
	memcpy(outbuffer+sizeof(uint32_t), stream1, offset);
}

void publishMsg(const ros::Msg& msg, EndPoint* endpoint)
{
	UDPMessage udpMessage;
	memcpy(&udpMessage.endpoint, endpoint, sizeof(EndPoint));
	serializeMsg(msg, (unsigned char*)udpMessage.data);
	UDPHandler* uh = UDPHandler::instance();
	uh->enqueueMessage(&udpMessage);
}



static uint16_t tcpPort = 50000;
/*
ConnectionHandler* createPublisher(const char* nodeName, const char* topicName, const char* msgType)
{
	XMLRPCHandler::registerPublisher(nodeName, topicName, msgType);
	ConnectionHandler* connection = new ConnectionHandler;
	connection->initPublisherEndpoint(tcpPort);

	tcpPort++;
	return connection;
}

ConnectionHandler* createSubscriber(const char* nodeName, const char* topicName, const char* msgType)
{
	XMLRPCHandler::registerSubscriber(nodeName, topicName, msgType);
	ConnectionHandler* connection = NULL;
	return connection;


}*/

/*
 * UDP bytestream
header
03 00 00 00 -> connection id
00 01 01 00 -> opcode=0x00(DATA0), msgid=0x01(incremented at each msg), block = 0x01 (16bit)
data
15 00 00 00
11 00 00 00
68 65 6c 6c
6f 20 77 6f
72 6c 64 5f
20 35 38 36
31

header
03 00 00 00 -> connection id
00 02 01 00 -> opcode=0x00(DATA0), msgid=0x02(incremented at each msg), block = 0x01 (16bit)
data
15 00 00 00
11 00 00 00
68 65 6c 6c
6f 20 77 6f
72 6c 64 5f
20 35 38 36
31

request:
<?xml version="1.0"?>
<methodCall><methodName>requestTopic</methodName>
<params><param><value>/listener</value></param><param><value>/chatter</value></param><param><value><array><data><value><array><data><value>UDPROS</value><value><base64>EgAAAGNhbGxlcmlkPS9saXN0ZW5lcicAAABtZDVzdW09OTkyY2U4YTE2ODdjZWM4YzhiZDg4
M2VjNzNjYTQxZDEOAAAAdG9waWM9L2NoYXR0ZXIUAAAAdHlwZT1zdGRfbXNncy9TdHJpbmc=</base64></value><value>SI-Z0M81</value><value><i4>44100</i4></value><value><i4>1500</i4></value></data></array></value></data></array></value></param></params></methodCall>
response:
<?xml version="1.0"?>
<methodResponse><params><param>
	<value><array><data><value><i4>1</i4></value><value></value><value><array><data><value>UDPROS</value><value>SI-Z0M81</value><value><i4>46552</i4></value><value><i4>3</i4></value><value><i4>1500</i4></value><value><base64>EAAAAGNhbGxlcmlkPS90YWxrZXInAAAAbWQ1c3VtPTk5MmNlOGExNjg3Y2VjOGM4YmQ4ODNl
YzczY2E0MWQxHwAAAG1lc3NhZ2VfZGVmaW5pdGlvbj1zdHJpbmcgZGF0YQoOAAAAdG9waWM9
L2NoYXR0ZXIUAAAAdHlwZT1zdGRfbXNncy9TdHJpbmc=</base64></value></data></array></value></data></array></value>
</param></params></methodResponse>

base64 connection headers

*/
void xmlrpc_task(void* p)
{
	/*ConnectionHandler* connection = createPublisher("rostopic_4767_1316912741557", "chatter", "std_msgs/String");
	ConnectionHandler* connection2 = createPublisher("rostopic_476", "chatter2", "std_msgs/String");*/

	//ConnectionHandler* connection3 = createSubscriber("rostopic", "chatter", "std_msgs/String");

	XMLRPCServer::start();

	//XMLRPCHandler::registerPublisher("talker", "chatter", "std_msgs/String");
	//vTaskDelay(4000);

	XMLRPCServer::registerPublisher("talker", "chatter", "std_msgs/String");
	//XMLRPCServer::registerPublisher("talker1", "chatter", "std_msgs/String");
	//XMLRPCServer::registerPublisher("talker2", "chatter", "std_msgs/String");
	//XMLRPCServer::registerSubscriber("listener", "chatter", "std_msgs/String");

	//XMLRPCHandler::waitForRequest();

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


	char string[] = "Hello ROS!";
	String str1;
	str1.data = string;

	EndPoint endpoint;
	endpoint.connectionID = 3;
	endpoint.ip.addr = inet_addr("10.3.84.100");


	//TODO: Why does system crash with rostopic echo chatter?
	//TODO: Connection won't be initialized if PC side subscriber is created before STM32 side publisher.

	LOOP(200,

			//connection->sendMessage((const char*)stream);
			//connection2->sendMessage((const char*)stream2);
			publishMsg(str, &endpoint);
			vTaskDelay(200);
	)

	vTaskDelete(NULL);
}

