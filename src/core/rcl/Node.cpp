#include <Node.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include <string.h>
extern "C" void* os_malloc(unsigned int);
namespace ros {


static int lastNodeIndex = -1;

typedef struct node_descriptor {
	char name[32];
	void (*function)(void* params);

} node_decriptor;
extern node_decriptor nodes;

Node::Node(const char* name)
{
	strcpy(this->name, name);
}


} /* namespace ros */
