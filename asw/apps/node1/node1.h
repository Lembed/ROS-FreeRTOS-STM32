#include "Node.h"
using namespace ros;
class Node1 : public Node
{
public:
	Node1(const char* name) : Node(name) {}
	void run();
	void init();
};
