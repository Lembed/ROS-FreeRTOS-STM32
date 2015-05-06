#include <Node.h>
#include <stdlib.h>
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

Node::Node(const char* name)
{
	list[++lastNodeIndex] = this;
	this->name = name;
	//strcpy(node->name, name);
}


} /* namespace ros */
