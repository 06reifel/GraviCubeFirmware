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

void controlRoll()
{
	switch(balanceMode)
	{
		case oneDimensional:
			double error = 45 - filteredRoll;
		break;

		case threeDimensional:
		break;
	}
}






