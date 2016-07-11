class MPU6050 {
public:
	typedef struct imu
	{
	    float x_accel;
	    float y_accel;
	    float z_accel;
	    float x_gyro;
	    float y_gyro;
	    float z_gyro;
	} IMU;


	static void init();
	static void readIMU(IMU* data);
private:
	static void calibrateSensor();
};
