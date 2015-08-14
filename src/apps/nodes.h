#ifndef ASW_APPS_APPLICATION_TASKS_H_
#define ASW_APPS_APPLICATION_TASKS_H_

#include "nodes/ultrasonic_sensor/ultrasonic_sensor.h"
#include "nodes/imu_sensor/imu_sensor.h"
#include "nodes/new_task/new_task.h"
#include "nodes/logger/logger.h"
#include "nodes/listener/listener.h"
#include "nodes/speed_setter/speed_setter.h"

typedef struct node_descriptor {
	char name[32];
	void (*function)(void* params);
    unsigned long deadline;

} node_decriptor;

node_decriptor nodes[] = {
        {"ultrasonic_sensor", ultrasonic_sensor, 50},
        {"imu_sensor", imu_sensor, 100},
        {"new_task", new_task, 10},
        {"speed_setter", speed_setter, 40}
};

#endif /* ASW_APPS_APPLICATION_TASKS_H_ */
