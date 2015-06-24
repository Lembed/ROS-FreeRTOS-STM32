#include "imu_sensor.h"
#include "rcl.h"
extern "C" {
#include "ros.h"
}

#include "Node.h"
#include "Publisher.h"
#include "Subscriber.h"

#include "MPU6050/MPU6050.h"
#include "sensor_msgs/Imu.h"

using namespace sensor_msgs;
using namespace ros;
Publisher* imu_pub;

void imuLoop()
{
	MPU6050::IMU data;
	MPU6050::readIMU(&data);

	Imu msg;

	msg.linear_acceleration.x = data.x_accel;
	msg.linear_acceleration.y = data.y_accel;
	msg.linear_acceleration.z = data.z_accel;

	msg.angular_velocity.x = data.x_gyro;
	msg.angular_velocity.y = data.y_gyro;
	msg.angular_velocity.z = data.z_gyro;
	os_printf("Sze: %d\n",sizeof(Imu));
	imu_pub->publish(msg);

}



void imu_sensor(void* p)
{
	Node* n = new Node("imu_sensor");
	imu_pub = new Publisher;
	imu_pub->advertise<Imu>(n, "imu");
	MPU6050::init();

	spinLoop(imuLoop, 100);
}
