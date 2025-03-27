/*
 * mpu6050.h
 *
 *  Created on: Mar 9, 2025
 *      Author: FR
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

enum dataRegisters
{
	ACCEL_XOUT_H,
	ACCEL_XOUT_L,
	ACCEL_YOUT_H,
	ACCEL_YOUT_L,
	ACCEL_ZOUT_H,
	ACCEL_ZOUT_L,
	TEMP_OUT_H,
	TEMP_OUT_L,
	GYRO_XOUT_H,
	GYRO_XOUT_L,
	GYRO_YOUT_H,
	GYRO_YOUT_L,
	GYRO_ZOUT_H,
	GYRO_ZOUT_L

};

enum interruptStates
{
	readData,
	control_X_Axis,
	control_Y_Axis,
	control_Z_Axis
};

#define DEVICE_ADRESS 0b1101000 //when AD0 is low on MPU6050
#define DLPF_VALUE 0b0 //Value of the Digital Low Pass Filter -> see Table in datasheet section 4.3 Configuration

#define REG_PWR_MGMT_1 107
#define REG_INT_PIN_CFG 55
#define REG_INT_ENABLE 56
#define REG_CONFIG 26

void mpu6050_init(bool);
void mpu6050_readData();

#endif /* INC_MPU6050_H_ */
