#include "node1.h"
#include "rcl.h"

Publisher* sqrt_pub;
void sqrtCallback(const Message* msg)
{
	// Get data from msg.
	int32_t num = *((int32_t*)msg->data);
	// Set data for msg to be published.
	*((float*)sqrt_pub->msg->data) = (float)sqrt((double)*((int32_t*)msg->data));
	os_printf("Sqrt of %d: %d", num, (long)*((float*)sqrt_pub->msg->data));

	// Publish msg to "sqrt" topic.
	publish(sqrt_pub);
}
unsigned long counter = 0;
void myloop()
{
	counter++;
	os_printf("Counter:%d\n", counter);
}
/*Publisher p;
p.advertise<Int32>(...);*/
void node1(void* params)
{
	Node* node = createNode("nodeB"); // Register node with the name 'nodeB' in RCL.
	sqrt_pub = createPublisher(node, "sqrt", RCL_MSG_TYPE_FLOAT); // Advertise to "sqrt" topic.
	Subscriber* sub = createSubscriber(node, "sub", RCL_MSG_TYPE_UINT32, sqrtCallback); // Subscribe to "sub" topic.
	spinLoop(node, myloop, 250); // Spin node for being able to receive subscriber callbacks and run loop with period of 250ms.

}
