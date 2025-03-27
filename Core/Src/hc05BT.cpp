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
extern bool receivedStart_Flag;
uint8_t receivedBTData;

void hc05_init()
{
	//Bluetooth-Test-message
	char test_string[] = "Hello, welcome to GraviCube\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)test_string, strlen(test_string), 100);

	//Enable Bluetooth Interrupt
	HAL_UART_Receive_IT(&huart2, &receivedBTData, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		if(receivedBTData == 'S' && !receivedStart_Flag)
		{
			receivedStart_Flag = true;
		}

		HAL_UART_Receive_IT(&huart2, &receivedBTData, 1); //Re-enable the Interrupt
	}
}


