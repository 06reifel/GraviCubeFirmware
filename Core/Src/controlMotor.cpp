/*
 * controlMotor.cpp
 *
 *  Created on: Mar 28, 2025
 *      Author: FR
 */

#include <stdio.h>

#include "controlMotor.h"
#include "mpu6050.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;
extern bool balanceMode;
extern double gyroYaw, filterRoll, filterPitch;

/*
 **********************************
 **		  TIMER-INTERRUPT	     **
 **********************************
*/

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
        		controlRoll();
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

Motor::Motor(TIM_HandleTypeDef *htim, unsigned int Channel, GPIO_TypeDef *motorPort, uint16_t directionPin, uint16_t enablePin, uint16_t brakePin)
{
	Motor::timer = htim;
	Motor::timerChannel = Channel;
	Motor::motorPort = motorPort;
	Motor::directionPin = directionPin;
	Motor::enablePin = enablePin;
	Motor::brakePin = brakePin;

	HAL_TIM_PWM_Start(timer, timerChannel);  // Start PWM

	changeDirection(CCW);

	changeBrakeState(enableBrake);

	changeMotorState(disableMotor);

}

void Motor::changeSpeed(uint8_t newMotorSpeed)
{
	speed = newMotorSpeed;

	uint32_t CCR_Value = (uint32_t)((__HAL_TIM_GET_AUTORELOAD(timer) + 1) * ((100.0 - speed) / 100.0));

	__HAL_TIM_SET_COMPARE(timer, timerChannel, CCR_Value);
}

void Motor::changeDirection(bool newMotorDirection)
{
	direction = newMotorDirection;
	HAL_GPIO_WritePin(motorPort, directionPin, (GPIO_PinState)direction);
}

void Motor::changeBrakeState(bool newBrakeState)
{
	brakeState = newBrakeState;
	HAL_GPIO_WritePin(motorPort, brakePin, (GPIO_PinState)brakeState);
}

void Motor::changeMotorState(bool newMotorState)
{
	motorState = newMotorState;
	HAL_GPIO_WritePin(motorPort, enablePin, (GPIO_PinState)motorState);
}

void controlRoll()
{
	double error;
	switch(balanceMode)
	{
		case oneDimensional:
			error = 45 - filterRoll;
		break;

		case threeDimensional:
			error = 45 - filterRoll;
		break;
	}
}






