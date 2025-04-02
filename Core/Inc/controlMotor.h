/*
 * controlMotor.h
 *
 *  Created on: Mar 28, 2025
 *      Author: FR
 */

#ifndef INC_CONTROLMOTOR_H_
#define INC_CONTROLMOTOR_H_

#include "main.h"

enum interruptStates
{
	readData,
	control_X_Axis,
	control_Y_Axis,
	control_Z_Axis
};

enum balanceModes
{
	oneDimensional,
	threeDimensional
};

enum motorDirections
{
	CCW,	//Low = Counter Clock Wise
	CW		//High = Clock Wise
};

enum motorBrakeStates
{
	enableBrake,	//Low = enabled
	disableBrake	//High = disabled

};

enum motorStates
{
	enableMotor,	//Low = enabled
	disableMotor	//High = disabled
};

class Motor
{
	public:
		void changeSpeed(uint8_t);
		void changeDirection(bool);
		void changeBrakeState(bool);
		void changeMotorState(bool);
		Motor(TIM_HandleTypeDef *, uint32_t, GPIO_TypeDef, uint16_t, uint16_t, uint16_t,);

	private:
		TIM_HandleTypeDef timer;
		uint32_t timerChannel;
		GPIO_TypeDef motorPort;
		uint16_t directionPin;
		uint16_t enablePin;
		uint16_t brakePin;

		uint8_t speed; //in %
		bool direction;
		bool brakeState;
		bool motorState;
};

Motor::Motor(TIM_HandleTypeDef *htim, uint32_t Channel, GPIO_TypeDef motorPort, uint16_t directionPin, uint16_t enablePin, uint16_t brakePin)
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

	uint32_t CCR_Value = (__HAL_TIM_GET_AUTORELOAD(timer) + 1) / (100/(100-speed))

	__HAL_TIM_SET_COMPARE(timer, timerChannel, CCR_Value);
}

void Motor::changeDirection(bool newMotorDirection)
{
	direction = newMotorDirection;
	HAL_GPIO_WritePin(motorPort, directionPin, direction);
}

void Motor::changeBrakeState(bool newBrakeState)
{
	brakeState = newBrakeState;
	HAL_GPIO_WritePin(motorPort, brakePin, brakeState);
}

void Motor::changeMotorState(bool newMotorState)
{
	motorState = newMotorState;
	HAL_GPIO_WritePin(motorPort, enablePin, motorState);
}

void controlRoll();

#endif /* INC_CONTROLMOTOR_H_ */
