#pragma once

//	SpiritLevel.h

#define		FALSE						0
#define		TRUE						1

// This sets the number of milliseconds per loop, and hence, the sample/refresh rate of the angle calculations
#define		LOOP_TIME_MS								4000	// 4ms, or 250 Hz

#define		GYRO_CAL_ITERATIONS							1000
#define		GYRO_CAL_DOT_PERIOD							125

#define		ACC_CAL_X									-2.1
#define		ACC_CAL_Y									-1.2

#define		I2C_ADDR_MPU6050_1							0x68
#define		I2C_ADDR_MPU6050_2							0x69

#define		MPU6050_REG_CONFIG							0x1A
#define		MPU6050_REG_GYRO_CONFIG						0x1B
#define		MPU6050_REG_ACCEL_CONFIG					0x1C

#define		MPU6050_REG_ACCEL_XOUT_H					0x3B
#define		MPU6050_REG_RAW_GYRO_VALUES					0x43

#define		MPU6050_REG_PWR_MGMT_1						0x6B
#define		MPU6050_REG_WHO_AM_I						0x75

#define		MPU6050_REG_PWR_MGMT_1_ACTIVATE_GYRO		0x00

#define		MPU6050_REG_GYRO_CONFIG_250DPS_FULL_SCALE	0x00
#define		MPU6050_REG_GYRO_CONFIG_500DPS_FULL_SCALE	0x08

#define		MPU6050_REG_ACCEL_CONFIG_4G_FULL_SCALE		0x08
#define		MPU6050_REG_ACCEL_CONFIG_8G_FULL_SCALE		0x10

#define		MPU6050_REG_CONFIG_LOW_PASS_43HZ			0x03


