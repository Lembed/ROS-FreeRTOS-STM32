#ifndef ASW_APPS_APPLICATION_TASKS_H_
#define ASW_APPS_APPLICATION_TASKS_H_

#include "nodes/node1/node1.h"
#include "nodes/node2/node2.h"
#include "nodes/ultrasonic_sensor/ultrasonic_sensor.h"

typedef struct node_descriptor {
	char name[32];
	void (*function)(void* params);

} node_decriptor;

node_decriptor nodes[] = {
		{"ultrasonic_sensor", ultrasonic_sensor},
		//{"node1", node1},
		//{"node2", node2}
};



#endif /* ASW_APPS_APPLICATION_TASKS_H_ */
