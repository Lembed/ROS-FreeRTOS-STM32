#include "speed_setter.h"
#include "rcl.h"
#include "Node.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "math.h"
#include "sensor_msgs/Range.h"
#include "PID/PID.h"


// Period in milliseconds
#define PUBLISH_PERIOD 40

using namespace ros;
using namespace sensor_msgs;

#define SYSTEM_VMAX 30
#define SYSTEM_DRIVEAMAX 10
#define SYSTEM_DT 0.04f
#define CRUISE_SPEED 25
#define MAX_DISTANCE 45.0f


float sign(float num)
{
    return num < 0 ? -1 : 1;
}

float wheelVelocityActuator(float v_old, float v_des)
{
    float v_new, v_temp;

    // do not exceed maximum velocity value
    v_temp = v_des;
    if (abs(v_temp) > SYSTEM_VMAX)
        v_temp = SYSTEM_VMAX*sign(v_temp);

    if (abs(v_temp-v_old)/SYSTEM_DT < SYSTEM_DRIVEAMAX  )// desired velocity reachable
        v_new = v_temp;
    else  // desired velocity cannot be reached
        v_new = v_old + SYSTEM_DT*SYSTEM_DRIVEAMAX*sign(v_temp-v_old);

    return v_new;
}

float currentVelocity = 0.0f;
float currentDistance = 0.0f;


void distanceCallback(const Range& msg)
{
    currentDistance = msg.range * 500.0f; // Only for now
}




PID* pid;


void speedSetterLoop()
{
    float desiredVelocity;
    os_printf("Distance: %d\n", (int)currentDistance);
    if (currentDistance > MAX_DISTANCE)
    {
        desiredVelocity = CRUISE_SPEED;
    }
    else
    {
        desiredVelocity = currentDistance / 1.8f;
    }

    float newVelocity = 0.0f;
    pid->compute(desiredVelocity, currentVelocity, &newVelocity);

    currentVelocity = wheelVelocityActuator(currentVelocity, newVelocity);

    os_printf("v: desired: %d, current: %d\n", (int)desiredVelocity, (int)(currentVelocity* 1000));
}

void speed_setter(void* p)
{
	// Register node in the ROS system and create a publisher with imu topic.
    Node* n = new Node("speed_setter");

    // Subscribe to "chatter" topic.
    ros::Subscriber<Range>* sub = new ros::Subscriber<Range>(n, "ultrasound", distanceCallback);

    float kp =0.15f, ki = 0.035f, kd =0.12f;
    pid = new PID(kp, ki, kd);

	// Begin periodic loop with PUBLISH_PERIOD in milliseconds.
    spinLoop(speedSetterLoop, PUBLISH_PERIOD);

	// Code never reaches here, deleting allocated memory is not necessary.
}
