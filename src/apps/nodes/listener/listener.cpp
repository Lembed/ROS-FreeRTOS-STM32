#include <nodes/listener/listener.h>
#include "rcl.h"
#include "Node.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "std_msgs/String.h"

using namespace std_msgs;

void chatterCallback(const String& msg)
{
	// Print message
	//LOG("Listener: %s\n", msg.data);
	os_printf("Listener: %s\n", msg.data);
}

void voidLoop()
{

}

void listener(void* params)
{
    // Register node in the ROS system
    ros::Node* n = new ros::Node("listener"); // TODO: Unique ID may need to be added later in case multiple STM32s will be connected to the same bus.

    // Subscribe to "chatter" topic.
    ros::Subscriber<String>* sub = new ros::Subscriber<String>(n, "chatter", chatterCallback);

 	spinLoop(voidLoop, 10000);
    // Code never reaches here, deleting allocated memory is not necessary.
}
