class PID {
public:
    PID(float p, float i, float d);
    void compute(float Setpoint, float Input, float* Output);

private:
    float kp, ki, kd;
};
