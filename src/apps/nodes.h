#ifndef ASW_APPS_APPLICATION_TASKS_H_
#define ASW_APPS_APPLICATION_TASKS_H_

#include "nodes/ultrasonic_sensor/ultrasonic_sensor.h"
#include "nodes/imu_sensor/imu_sensor.h"
#include "nodes/new_task/new_task.h"
#include "nodes/logger/logger.h"
#include "nodes/listener/listener.h"

typedef struct node_descriptor {
	char name[32];
	void (*function)(void* params);

} node_decriptor;

node_decriptor nodes[] = {
        {"ultrasonic_sensor", ultrasonic_sensor},
        {"imu_sensor", imu_sensor},
        {"new_task", new_task},
        //{"logger", logger},
        //{"listener", listener}
};

#endif /* ASW_APPS_APPLICATION_TASKS_H_ */
