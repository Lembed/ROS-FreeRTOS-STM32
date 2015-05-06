#ifndef NODE_H_
#define NODE_H_
extern "C" void os_printf(const char* fmt, ...);
namespace ros {

class Node {
public:
	Node(const char* name);
	void print()
	{
		os_printf("Test!\n");
	}
	void addSharedObject(const char* name, void* object);
	void* getSharedObject(const char* name);
	static Node** list;
	const char* name;
private:


};


} /* namespace ros */

#endif /* NODE_H_ */
