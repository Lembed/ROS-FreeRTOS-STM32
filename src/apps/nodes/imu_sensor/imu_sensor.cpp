#include "device_config.h"
#include "imu_sensor.h"
#include "rcl.h"
#include "Node.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "MPU6050/MPU6050.h"
#include "sensor_msgs/Imu.h"

// Period in milliseconds
#define PUBLISH_PERIOD 100

using namespace sensor_msgs;
using namespace ros;
Publisher* imu_pub;

void imuLoop()
{
	// Read IMU data from sensor.
	MPU6050::IMU data;
	MPU6050::readIMU(&data);

	Imu msg;

	msg.linear_acceleration.x = data.x_accel;
	msg.linear_acceleration.y = data.y_accel;
	msg.linear_acceleration.z = data.z_accel;

	msg.angular_velocity.x = data.x_gyro;
	msg.angular_velocity.y = data.y_gyro;
	msg.angular_velocity.z = data.z_gyro;

	imu_pub->publish(msg);
}

void imu_sensor(void* p)
{
	// Register node in the ROS system and create a publisher with imu topic.
	Node* n = new Node("imu_sensor_"ROS_NODE_UNIQUE_ID);
	imu_pub = new Publisher;
	imu_pub->advertise<Imu>(n, "imu");

	// Initialize sensor.
	MPU6050::init();

	// Begin periodic loop with PUBLISH_PERIOD in milliseconds.
	spinLoop(imuLoop, PUBLISH_PERIOD);

	// Code never reaches here, deleting allocated memory is not necessary.
}
