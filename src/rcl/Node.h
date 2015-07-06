#ifndef NODE_H_
#define NODE_H_
extern "C" void os_printf(const char* fmt, ...);
namespace ros {

class Node {
public:
	Node(const char* name);
	void addSharedObject(const char* name, void* object);
	void* getSharedObject(const char* name);
	char name[32];
private:


};


} /* namespace ros */

#endif /* NODE_H_ */
