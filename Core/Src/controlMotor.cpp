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
extern Motor* Motor_3;
extern double gyroYaw, gyroX, filterRoll, filterPitch;

//Control-Parameters
double Kp = 25;
double Ki = 0.0;
double Kd = 2.0;
double alpha = 0.8;

void controlRoll()
{
	double error, output, dt;
	static uint32_t lastTime = 0;
	static double errorIntegral = 0, previousOutput = 0, filteredGyroX = 0;

	switch(balanceMode)
	{
		case oneDimensional:
		{
			//PID
			uint32_t currentTime = HAL_GetTick();
			if (lastTime == 0)
			{
				lastTime = currentTime;
				return; // Skip first iteration
			}

			dt = (currentTime - lastTime) / 1000.0;
			lastTime = currentTime;

			// Skip if dt is too large (system was paused)
			if (dt > 0.1)
			{
			    errorIntegral = 0;
			    return;
			}

			error = 46.3 - filterRoll; //links = 90°, rechts = 0° => links negativer Fehler

			//deadband to reduce noise
			if (gyroX > -0.5 && gyroX < 0.5) {
			    gyroX = 0;
			}

			filteredGyroX = alpha * gyroX + (1 - alpha) * filteredGyroX;

			errorIntegral += error * dt;

			// Integral windup protection
			if (errorIntegral > 50) errorIntegral = 50;
			if (errorIntegral < -50) errorIntegral = -50;

			output = Kp * error + Ki * errorIntegral - Kd * filteredGyroX;

			// Rate limiting for smooth transitions
			double outputChange = output - previousOutput;
			if (outputChange > MAX_PWM_CHANGE_RATE)
			{
				output = previousOutput + MAX_PWM_CHANGE_RATE;
			}
			else if (outputChange < -MAX_PWM_CHANGE_RATE)
			{
				output = previousOutput - MAX_PWM_CHANGE_RATE;
			}

			previousOutput = output;

			double targetSpeed = MOTOR_BASE_SPEED + output;

			//The reaction wheel is spinning to the right
			//faster speed = correction to the left
			//slower speed = correction to the right

			if (targetSpeed > MOTOR_MAX_SPEED) targetSpeed = MOTOR_MAX_SPEED;
			if (targetSpeed < MOTOR_MIN_SPEED) targetSpeed = MOTOR_MIN_SPEED;

	        uint32_t brakeCurrentTime = HAL_GetTick();  // Separate timing for brake

	        // Check if strong braking (slowing down significantly)
	        bool needsBraking = (output > BRAKE_THRESHOLD && targetSpeed < MOTOR_BASE_SPEED);

			Motor_3->changeSpeed(targetSpeed);

		break;
		}
		case threeDimensional:
			error = 45 - filterRoll;
		break;

		case test:
			Motor_3->testMotor();
		break;
	}
}






