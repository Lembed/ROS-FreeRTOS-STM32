#ifndef ASW_OS_ROS_H_
#define ASW_OS_ROS_H_
/* includes of RTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

void os_printf(const char* fmt, ...);
void* os_malloc(unsigned int size);


#endif /* ASW_OS_ROS_H_ */
