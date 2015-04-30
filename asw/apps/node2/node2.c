#include "node2.h"
#include "rcl.h"

Publisher *square_pub;

void print_float(float f)
{
	float square = f * 100;
	long square_int = (long)(square / 100);
	long period = ((long)square) % 100;
	os_printf("Squared:%d.%d", square_int, period);
}

void squareCallback(const Message* msg)
{
	// Get data from msg.
	float f = *((float*)msg->data);
	print_float(f*f); // Since formatting float (%f) does not work, we use a workaround
	// Set data for msg to be published.
	*((float*)square_pub->msg->data) = f*f;
	//Message m;
	//m.data = f;
	// Publish msg to "squared" topic.
	publish(square_pub);
}
void loop2()
{

}
void node2(void* params)
{
	Node* node = createNode("nodeC"); // Register node with the name 'nodeC' in RCL.
	square_pub = createPublisher(node, "squared", RCL_MSG_TYPE_FLOAT); // Advertise to "squared" topic.
	Subscriber* sub = createSubscriber(node, "sqrt", RCL_MSG_TYPE_FLOAT, squareCallback); // Subscribe to "sqrt" topic.
	spin(node); //--> TASK DELETE
    //spinLoop(node, loop2, 25);


	unsigned long counter = 0;
	// Periodic code goes here.
	// First argument: Period.
	// Second argument periodic code.
	// Note: A periodic loop must always be included.
	LOOP(250,
	// start while
	// end while
	)
}
