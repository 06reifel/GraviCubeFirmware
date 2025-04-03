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
	idle,
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
		void changeSpeed(double);
		void changeDirection(bool);
		void changeBrakeState(bool);
		void changeMotorState(bool);
		void testMotor();
		bool getDirection();
		Motor(TIM_HandleTypeDef *, unsigned int, GPIO_TypeDef *, uint16_t, uint16_t, uint16_t);

	private:
		TIM_HandleTypeDef* timer;
		unsigned int timerChannel;
		GPIO_TypeDef* motorPort;
		uint16_t directionPin;
		uint16_t enablePin;
		uint16_t brakePin;

		double speed; //in %
		bool direction;
		bool brakeState;
		bool motorState;
};

void controlRoll();

#endif /* INC_CONTROLMOTOR_H_ */
