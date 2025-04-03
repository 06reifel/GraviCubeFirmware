/*
 * hc05BT.cpp
 *
 *  Created on: Mar 24, 2025
 *      Author: FR
 */

#include <stdio.h>
#include <string.h>

#include "hc05BT.h"
#include "main.h"

extern UART_HandleTypeDef huart2;
extern bool receivedStart_Flag_1D;
uint8_t receivedBTData;

/*
 **********************************
 **		  BT-MODULE INIT	     **
 **********************************
*/

void hc05_init()
{
	//Bluetooth-Test-message
	char test_string[] = "Hello, welcome to GraviCube\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)test_string, strlen(test_string), 100);

	//Enable Bluetooth Interrupt
	HAL_UART_Receive_IT(&huart2, &receivedBTData, 1);
}

/*
 **********************************
 **		  BT-MSG INTERRUPT	     **
 **********************************
*/
extern double Kp, Ki, Kd;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		if(receivedBTData == 'S' && !receivedStart_Flag_1D)
		{
			receivedStart_Flag_1D = true;
		}

		switch(receivedBTData)
		{
			case 'a':
				Kp += 0.1;
			break;

			case 'b':
				Kp += 0.1;
			break;

			case 'c':
				Ki += 0.01;
			break;

			case 'd':
				Ki -= 0.01;
			break;

			case 'e':
				Kd += 0.01;
			break;

			case 'f':
				Kd -= 0.01;
			break;
		}

		HAL_UART_Receive_IT(&huart2, &receivedBTData, 1); //Re-enable the Interrupt
	}
}


