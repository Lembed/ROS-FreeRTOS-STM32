#ifndef APPS_LIBS_HCSR04SENSOR_HCSR04_H_
#define APPS_LIBS_HCSR04SENSOR_HCSR04_H_

#include "stdint.h"

class HCSR04
{
public:
	static void init(void);
	static float pingMedian(uint8_t it);
	static float ping();

};

#endif /* APPS_LIBS_HCSR04SENSOR_HCSR04_H_ */
