/*
 * functions.cpp
 *
 *  Created on: Apr 9, 2025
 *      Author: FR
 */

#include "main.h"
#include "functions.h"

extern ADC_HandleTypeDef hadc1;

void blinkLED(uint32_t interval = 1000)
{
	static uint32_t timeSaveBlink = HAL_GetTick();
	if (HAL_GetTick() - timeSaveBlink >= interval)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
		timeSaveBlink = HAL_GetTick();
	}
}

float readBatteryVoltage()
{
  uint32_t adcValue;
  float voltage;

  // Start ADC conversion
  HAL_ADC_Start(&hadc1);

  // Wait for conversion to complete
  HAL_ADC_PollForConversion(&hadc1, 100);

  // Read ADC value
  adcValue = HAL_ADC_GetValue(&hadc1);

  // Convert ADC value to voltage
  // For 12-bit ADC (0-4095), reference voltage 3.3V
  voltage = (float)adcValue * (3.3f / 4095.0f);

  // Scale back to actual battery voltage using voltage divider ratio
  // Total resistance: 9.1kΩ + 3.3kΩ = 12.4kΩ
  // Division ratio: 12.4 / 3.3 = 3.76
  float batteryVoltage = voltage * (12.4f / 3.3f);

  return batteryVoltage;
}


