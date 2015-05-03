#ifndef NODE_H_
#define NODE_H_

namespace ros {
extern "C" void os_printf(const char* fmt, ...);
class Node {
public:
	Node();
	void print()
	{
		os_printf("Test!\n");
	}
};

} /* namespace ros */

#endif /* NODE_H_ */
