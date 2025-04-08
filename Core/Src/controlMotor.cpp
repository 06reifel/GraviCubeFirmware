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
extern uint8_t balanceMode;

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

/*
 **********************************
 **		  MOTOR-CLASS-DEF	     **
 **********************************
*/

Motor::Motor(TIM_HandleTypeDef *htim, unsigned int Channel, GPIO_TypeDef *motorPort, uint16_t directionPin, uint16_t enablePin, uint16_t brakePin)
{
	Motor::timer = htim;
	Motor::timerChannel = Channel;
	Motor::motorPort = motorPort;
	Motor::directionPin = directionPin;
	Motor::enablePin = enablePin;
	Motor::brakePin = brakePin;

	changeSpeed(MOTOR_BASE_SPEED);

	HAL_TIM_PWM_Start(timer, timerChannel);  // Start PWM

	changeDirection(CCW);

	changeBrakeState(enableBrake);

	changeMotorState(disableMotor);
}

void Motor::changeSpeed(double newMotorSpeed)
{
	speed = newMotorSpeed;

	uint32_t CCR_Value = (uint32_t)((double)(__HAL_TIM_GET_AUTORELOAD(timer) + 1) * ((100.0 - speed) / 100.0));

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

void Motor::testMotor()
{
	static uint8_t PWMspeed = 0;
	static uint32_t timeSaveMotorTest = 0;
	if(HAL_GetTick() - timeSaveMotorTest >= 10000)
	{
		uint8_t newSpeed; // in %
		switch(PWMspeed)
		{
			case 0:
				newSpeed = 50;
				changeSpeed(newSpeed);
				PWMspeed++;
			break;

			case 1:
				newSpeed = 60;
				changeSpeed(newSpeed);
				PWMspeed++;
			break;

			case 2:
				newSpeed = 25;
				changeSpeed(newSpeed);
				PWMspeed = 0;
			break;
		}
		timeSaveMotorTest = HAL_GetTick();
	}
}

bool Motor::getDirection()
{
	return direction;
}

double Motor::getSpeed()
{
	return speed;
}

/*
 **********************************
 **		    MOTOR-CONTROL	     **
 **********************************
*/
extern Motor Motor_3;
extern double gyroYaw, filterRoll, filterPitch;
double Kp = /*160;*/ 0.001;
double Ki = /*10.5;*/ 0.0;
double Kd = /*0.03;*/ 0.03;
double alpha = 0.74;

void controlRoll()
{
	double error, errorDerivative, output, absOutput, dt, filterYaw;
	static uint32_t lastTime = 0;
	static double previousError = 0, errorIntegral = 0, motor_speed_X = 0;

	switch(balanceMode)
	{
		case oneDimensional:
		{
			//PID

			error = 46.5 /*47.3*/ - filterRoll; //links = 0°, rechts = 90°
			dt = (HAL_GetTick() - lastTime) / 1000.0;
			errorIntegral += error * dt;
			errorDerivative = (error - previousError) / dt;
			previousError = error;
			lastTime = HAL_GetTick();
			output = Kp * error + Ki * errorIntegral + Kd * errorDerivative;

			//schneller = nach links, langsamer = nach rechts
			if (output > 20) output = 20;
			if (output < -20) output = -20;

			//LQR
			/*
			filterYaw = alpha * gyroYaw + (1 - alpha) * filterYaw;
			output = Kp * filterRoll + Ki * filterYaw + Kd * motor_speed_X;
			*/

			absOutput = (output < 0) ? -output : output;

			//motor_speed_X += absOutput;

			if(output > 0)
			{
				//langsamer werden
				Motor_3.changeBrakeState(enableBrake);
				Motor_3.changeSpeed((MOTOR_BASE_SPEED - absOutput));
				Motor_3.changeBrakeState(disableBrake);
			}
			else if(output < 0)
			{
				//schneller werden
				Motor_3.changeSpeed((MOTOR_BASE_SPEED + absOutput));
			}


		break;
		}
		case threeDimensional:
			error = 45 - filterRoll;
		break;

		case test:
			Motor_3.testMotor();
		break;
	}
}






