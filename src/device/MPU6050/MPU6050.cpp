#include "MPU6050.h"
#include "I2C/i2c.h"
#include "wiring.h"

#include "rcl.h"
extern "C" {
#include "ros.h"
}

#define MPU6050_ADDRESS 0x68

#define ACC_SCALE 9.81f / 8192
#define GYRO_SCALE 1.0f / 65.5f

//  Use the following global variables and access functions
//  to calibrate the acceleration sensor
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;

float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;

void printfloat(const char* label, float f)
{
	int32_t dec = f * 1000;
	int32_t num = dec / 1000;
	int32_t period = dec % 1000;
	if (period < 0)
		period = period * -1;

	os_printf("%s: %d.%03d\n", label, num, period);
}

void MPU6050::init()
{
	I2C::init(GPIO_PB6, GPIO_PB7);

	// Clear the 'sleep' bit to start the sensor.
	I2C::writeByteToRegister(MPU6050_ADDRESS, 0x6B, 0);

	// Set range: +/- 500 degrees/sec, LSB sensitivity: 65.5 LSB/degrees/sec
	I2C::writeByteToRegister(MPU6050_ADDRESS, 27, 1);

	// Set range: +/- 4g, LSB sensitivity: 8192 LSB/g
	I2C::writeByteToRegister(MPU6050_ADDRESS, 28, 1);


	calibrateSensor();
}

void MPU6050::readIMU(IMU* data)
{
	uint8_t mpu6050Data[14];
	I2C::readBytes(MPU6050_ADDRESS, 0x3B, 14, mpu6050Data);

	int16_t accX = ((mpu6050Data[0] & 0xFF) << 8) | (mpu6050Data[1] & 0xFF);
	int16_t accY = ((mpu6050Data[2] & 0xFF) << 8) | (mpu6050Data[3] & 0xFF);
	int16_t accZ = ((mpu6050Data[4] & 0xFF) << 8) | (mpu6050Data[5] & 0xFF);

	int16_t gyroX = ((mpu6050Data[8] & 0xFF) << 8) | (mpu6050Data[9] & 0xFF);
	int16_t gyroY = ((mpu6050Data[10] & 0xFF) << 8) | (mpu6050Data[11] & 0xFF);
	int16_t gyroZ = ((mpu6050Data[12] & 0xFF) << 8) | (mpu6050Data[13] & 0xFF);

	data->x_accel = (float)accX * ACC_SCALE - base_x_accel;
	data->y_accel = (float)accY * ACC_SCALE - base_y_accel;
	data->z_accel = (float)accZ * ACC_SCALE - base_z_accel;

	data->x_gyro = (float)gyroX * GYRO_SCALE - base_x_gyro;
	data->y_gyro = (float)gyroY * GYRO_SCALE - base_y_gyro;
	data->z_gyro = (float)gyroZ * GYRO_SCALE - base_z_gyro;

	/*printfloat("accX", data->x_accel);
	printfloat("accY", data->y_accel);
	printfloat("accZ", data->z_accel);
	printfloat("gyroX", data->x_gyro);
	printfloat("gyroY", data->y_gyro);
	printfloat("gyroZ", data->z_gyro);*/
}

void MPU6050::calibrateSensor()
{
	  int                   num_readings = 10;
	  float                 x_accel = 0;
	  float                 y_accel = 0;
	  float                 z_accel = 0;
	  float                 x_gyro = 0;
	  float                 y_gyro = 0;
	  float                 z_gyro = 0;
	  IMU data;

	  os_printf("Starting Calibration\n");
	  // Give MPU6050 time to initialize.
	  vTaskDelay(200);
	  // Discard the first set of values read from the IMU
	  readIMU(&data);

	  // Read and average the raw values from the IMU
	  for (int i = 0; i < num_readings; i++) {
		  readIMU(&data);
	    x_accel += data.x_accel;
	    y_accel += data.y_accel;
	    z_accel += data.z_accel;
	    x_gyro += data.x_gyro;
	    y_gyro += data.y_gyro;
	    z_gyro += data.z_gyro;
	    vTaskDelay(100);
	  }
	  x_accel /= num_readings;
	  y_accel /= num_readings;
	  z_accel /= num_readings;
	  x_gyro /= num_readings;
	  y_gyro /= num_readings;
	  z_gyro /= num_readings;

	  // Store the raw calibration values globally
	  base_x_accel = x_accel;
	  base_y_accel = y_accel;
	  base_z_accel = z_accel;
	  base_x_gyro = x_gyro;
	  base_y_gyro = y_gyro;
	  base_z_gyro = z_gyro;

	  os_printf("Finishing Calibration\n");
}
