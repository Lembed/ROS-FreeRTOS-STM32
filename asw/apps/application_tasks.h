#ifndef ASW_APPS_APPLICATION_TASKS_H_
#define ASW_APPS_APPLICATION_TASKS_H_

#include "ultrasonic_sensor/ultrasonic_sensor.h"
#include "node1/node1.h"
#include "node2/node2.h"
#include "xmlrpc/xmlrpc.h"

typedef struct node_descriptor {
	char name[32];
	void (*function)(void* params);

} node_decriptor;

node_decriptor nodes[] = {
		/*{"ultrasonic_sensor", ultrasonic_sensor},
		{"node1", node1},
		{"node2", node2},*/
		{"xmlrpc", xmlrpc_task}
};



#endif /* ASW_APPS_APPLICATION_TASKS_H_ */
