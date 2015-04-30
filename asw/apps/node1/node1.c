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

void node1(void* params)
{
	Node* node = createNode("nodeB"); // Register node with the name 'nodeB' in RCL.
	sqrt_pub = createPublisher(node, "sqrt", RCL_MSG_TYPE_FLOAT); // Advertise to "sqrt" topic.
	Subscriber* sub = createSubscriber(node, "sub", RCL_MSG_TYPE_UINT32, sqrtCallback); // Subscribe to "sub" topic.
	spin(node); // Spin node for being able to receive subscriber callbacks.

	unsigned long counter = 0;
	// Periodic code goes here.
	// First argument: Period.
	// Second argument periodic code.
	// Note: A periodic loop must always be included.
	LOOP(250,
	// start while
	counter++;
	os_printf("Counter:%d\n", counter);
	// end while
	)
}
