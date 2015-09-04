#include "device_config.h"
#include "logger.h"
#include "rcl.h"
#include "Node.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "std_msgs/String.h"

using namespace std_msgs;

extern "C" xQueueHandle terminalQueue;
#define MAX_CHARS 128
#define TERMINAL_QUEUE_LEN 10
#define TERMINAL_QUEUE_TIMEOUT 10

ros::Publisher* logger_pub;

void loggerLoop()
{

}
#include <stdarg.h>

void LOG(const char* fmt, ...)
{
	if (logger_pub)
	{
	    va_list ap;
	    char string[MAX_CHARS];

	    va_start(ap, fmt);
	    vsprintf(string, fmt, ap);
	    va_end(ap);

		String msg;
		msg.data = string;
		logger_pub->publish(msg);
	}
}

void logger(void* params)
{
    // Register node in the ROS system.
    ros::Node* n = new ros::Node("stm32_logger_"ROS_NODE_UNIQUE_ID); // TODO: Unique ID may need to be added later in case multiple STM32s will be connected to the same bus.
    logger_pub = new ros::Publisher;
    logger_pub->advertise<String>(n, "logger");

    spinLoop(loggerLoop, 10000);
    // Code never reaches here, deleting allocated memory is not necessary.
}
