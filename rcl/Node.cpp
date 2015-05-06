#include <Node.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include <string.h>
#include <stdio.h>
#include "rcl.h"
extern "C" void* os_malloc(unsigned int);
namespace ros {


static int lastNodeIndex = -1;

typedef struct node_descriptor {
	char name[32];
	void (*function)(void* params);

} node_decriptor;
extern node_decriptor nodes;

Node* Nodes_[sizeof(nodes) / sizeof(node_decriptor)];

Node** Node::list = Nodes_;
char taskName[32];
Node::Node(const char* name)
{
	list[++lastNodeIndex] = this;
	this->name = name;
	//strcpy(node->name, name);
	sprintf(taskName, "node_%s", name);
	init();
	xTaskCreate(nodeTask, (const signed char*)taskName, 128, (void*) this, tskIDLE_PRIORITY + 2, NULL);
}

void Node::nodeTask(void* args)
{
	Node* self = (Node*) args;
	LOOP(self->getPeriod(),
	// start while
	self->run();
	// end while
	)
}

} /* namespace ros */
