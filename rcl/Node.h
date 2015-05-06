#ifndef NODE_H_
#define NODE_H_
extern "C" void os_printf(const char* fmt, ...);
namespace ros {

class Node {
public:
	Node(const char* name);
	void addSharedObject(const char* name, void* object);
	void* getSharedObject(const char* name);
	static Node** list;
	const char* name;
	static void nodeTask(void* args);
	virtual void init() {};
	virtual void run() {};
	void setLoopPeriod(unsigned int period)
	{
		this->period = period;
	}
	unsigned int getPeriod() {return period;}
private:
	unsigned int period;

};


} /* namespace ros */

#endif /* NODE_H_ */
