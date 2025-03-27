/*
 * mpu6050.cpp
 *
 *  Created on: Mar 9, 2025
 *      Author: FR
 */

#include <stdio.h>
#include <math.h>

#include "mpu6050.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim4;
uint8_t dataMPU[14];

void mpu6050_init(bool interruptEnable)
{
	//Check connection
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, ((DEVICE_ADRESS) <<1) + 0, 1, 100); //Makro muss in Klammern sein
	if(ret == HAL_OK)
	{
		printf("The device is ready \n");
	}
	else
	{
		printf("The device is not ready \n");
	}

	//PWR_MGMT_1
	//Exit sleep mode
	uint8_t config = 0; //To disable Temperature Sensor Set 3rd Bit to HIGH
	ret = HAL_I2C_Mem_Write(&hi2c1, ((DEVICE_ADRESS) <<1) + 0, REG_PWR_MGMT_1, 1, &config, 1, 100); //+ 0 because of Read/Write bit
	if(ret == HAL_OK)
	{
		printf("Exited sleep mode \n");
	}
	else
	{
		printf("Couldn't exit sleep mode \n");
	}

	//DLPF
	//Set Value of Digital Low Pass
	// Reads the current value of the CONFIG (26) register
	config = HAL_I2C_Mem_Read(&hi2c1, ((DEVICE_ADRESS) <<1) + 1, REG_CONFIG, 1, &config, 1, 100);

	//Sets the last 3-Bits of the Register according to the DLPF_VALUE, which are responsible for the Digital Low Pass
	config &= ((~(0b111)) | (DLPF_VALUE & 0b111));

	//Writes the updated value back to the CONFIG register
	ret = HAL_I2C_Mem_Write(&hi2c1, ((DEVICE_ADRESS) <<1) + 0, REG_CONFIG, 1, &config, 1, 100); //+ 0 because of Read/Write bit
	if(ret == HAL_OK)
	{
		printf("Set digital Low Pass to value %d \n", DLPF_VALUE);
	}
	else
	{
		printf("Couldn't set the digital Low Pass value \n");
	}

	if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
	{
		// Starting Error
	    Error_Handler();
	 }

	//Interrupt-Enable
	if(interruptEnable)
	{
		//INT_PIN_CFG
		// Reads the current value of the INT_PIN_CFG register
		config = HAL_I2C_Mem_Read(&hi2c1, ((DEVICE_ADRESS) <<1) + 1, REG_INT_PIN_CFG, 1, &config, 1, 100);

		// Set the INT_OPEN bit (bit 6) to open-drain-mode and the INT_LEVEL bit (bit 7) to active LOW
		config |= (0b11 << 6); // Set bit 7 and 6 to 1

		// Writes the updated value back to the INT_PIN_CFG register
		ret = HAL_I2C_Mem_Write(&hi2c1, ((DEVICE_ADRESS) <<1) + 0, REG_INT_PIN_CFG, 1, &config, 1, 100); //+ 0 because of Read/Write bit
		if(ret == HAL_OK)
		{
			printf("Set Int-Pin to open-drain and active LOW \n");
		}
		else
		{
			printf("Couldn't set Int-Pin mode \n");
		}

		//INT_ENABLE
		// Reads the current value of the INT_ENABLE register
		config = HAL_I2C_Mem_Read(&hi2c1, ((DEVICE_ADRESS) <<1) + 1, REG_INT_ENABLE, 1, &config, 1, 100);

		// Set the DATA_RDY_EN bit (bit 0) disable
		config |= (0); // Set bit 0 to 0

		// Writes the updated value back to the INT_ENABLE register
		ret = HAL_I2C_Mem_Write(&hi2c1, ((DEVICE_ADRESS) <<1) + 0, REG_INT_ENABLE, 1, &config, 1, 100); //+ 0 because of Read/Write bit
		if(ret == HAL_OK)
		{
			printf("Activated Data Ready Interrupt \n");
		}
		else
		{
			printf("Couldn't activate Interrupt \n");
		}
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t interruptState = readData;
    if (htim->Instance == TIM4)  // Check if the interrupt comes from TIM4
    {
        switch(interruptState)
        {
        	case readData:
        		mpu6050_readData();
        		interruptState++;
        	break;

        	case control_X_Axis:
				interruptState++;
			break;

        	case control_Y_Axis:
				interruptState++;
			break;

        	case control_Z_Axis:
				interruptState = readData;
			break;

        }
    }
}

void mpu6050_readData()
{
	if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
	{
		HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, ((DEVICE_ADRESS) <<1) + 1, 59, 1, dataMPU, 14, 100);

		if(ret == HAL_OK)
		{
			double accX, accY, accZ, gyroX, gyroY, gyroZ, elapsedTime, accRoll, accPitch;
			static double gyroAngleX = 0.0, gyroAngleY = 0.0, gyroYaw = 0.0;
			static uint32_t currentTime, previousTime;

			//Accelerometer Data (Registers 59 to 64)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			accX = ((int16_t)((dataMPU[ACCEL_XOUT_H] << 8) | dataMPU[ACCEL_XOUT_L]))/16384.0;
			//printf("Accel-X [g]: %f \n", accX);

			accY = ((int16_t)((dataMPU[ACCEL_YOUT_H] << 8) | dataMPU[ACCEL_YOUT_L]))/16384.0;
			//printf("Accel-Y [g]: %f \n", accY);

			accZ = ((int16_t)((dataMPU[ACCEL_ZOUT_H] << 8) | dataMPU[ACCEL_ZOUT_L]))/16384.0;
			//printf("Accel-Z [g]: %f \n", accZ);

			//Roll and Pitch Angles from Accelerometer
			accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.2958; //* 57.2958 conversion from rad to deg (180°/PI)
			//printf("Pitch: %f \n", accPitch);
			/*works only if sensor is level (small pitch)
			accRoll = atan2(accY, accZ) * 57.3;
			printf("Roll1: %f \n", accRoll);
			*/
			accRoll = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 57.2958; //* 57.2958 conversion from rad to deg (180°/PI)
			//printf("Roll: %f \n", accRoll);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			//Temperature Data (Registers 65 and 66)

			//Gyroscope Data (Registers 67 to 72)
			currentTime = HAL_GetTick();
			elapsedTime = (currentTime - previousTime) / 1000.0; // divided by 1000 for conversion between milliseconds and seconds

			gyroX = ((int16_t)((dataMPU[GYRO_XOUT_H] << 8) | dataMPU[GYRO_XOUT_L]))/131.0;
			printf("Gyro-X [°/s]: %f \n", gyroX);

			gyroY = ((int16_t)((dataMPU[GYRO_YOUT_H] << 8) | dataMPU[GYRO_YOUT_L]))/131.0;
			printf("Gyro-Y [°/s]: %f \n", gyroY);

			gyroZ = ((int16_t)((dataMPU[GYRO_ZOUT_H] << 8) | dataMPU[GYRO_ZOUT_L]))/131.0;
			printf("Gyro-Z [°/s]: %f \n", gyroZ);

			//Angles and yaw from Gyroscope
			gyroAngleX += gyroX * elapsedTime;
			gyroAngleY += gyroY * elapsedTime;
			gyroYaw += gyroZ * elapsedTime;

			previousTime = currentTime;
		}
	}
}

//Code for reading the Sensor Data with the Interrupt Pin
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Check if it's the wanted Pin (PC08)
	if (GPIO_Pin == GPIO_PIN_8)
	{
		//read the INT Status Register to reset the Interrupt Flag
		uint8_t dummy;
		HAL_I2C_Mem_Read(&hi2c1, ((DEVICE_ADRESS) <<1) + 1, 58, 1, &dummy, 1, 100);

		//read the data via I2C if ready
		if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
		{
			mpu6050_readData();
		}
	}
}




