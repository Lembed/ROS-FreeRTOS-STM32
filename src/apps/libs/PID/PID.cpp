#include "PID.h"

PID::PID(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
}

void PID::compute(float Setpoint, float Input, float* Output)
{
    static float errSum = 0.0f;
    static float lastErr = 0.0f;

    /*Compute all the working error variables*/
    float error = Setpoint - Input;
    errSum += error;
    float dErr = (error - lastErr);

    /*Compute PID Output*/
    *Output = kp * error + ki * errSum + kd * dErr;

    // TODO: Threshold of output required??

    /*Remember some variables for next time*/
    lastErr = error;
}
