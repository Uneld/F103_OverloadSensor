/*
 * load_cell.c
 *
 *  Created on: Jan 9, 2024
 *      Author: Uneld
 */

#include "load_cell.h"
#include "delay_user.h"

#define COUNT_FLT_AVR_LOAD 3

int16_t proc_hx711_getValue() {
	static uint8_t switchGetData = 0;
	static uint32_t startTime = 0;
	static uint32_t outData = 0;
	static int32_t summAvrLoad = 0, countDiv = 0;
	static int16_t outDataInt16;

	switch (switchGetData) {
	case 0:
		startTime = HAL_GetTick();
		switchGetData = 1;
		break;
	case 1:
		uint32_t currentTime = HAL_GetTick();
		if (currentTime - startTime > 400) {
			switchGetData = 2;
		}
		break;
	case 2:
		if (HAL_GPIO_ReadPin(HX711_DT_GPIO_Port, HX711_DT_Pin) == GPIO_PIN_RESET) {
			switchGetData = 3;
			outData = 0;
		}
		break;
	case 3:
		for (uint8_t i = 0; i < 24; i++) {
			HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET);

			delay_tick_one();
			delay_tick_one();
			HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);

			outData = (outData << 1) | (((HX711_DT_GPIO_Port->IDR & (HX711_DT_Pin)) != (uint32_t) GPIO_PIN_RESET));
		}

		if (countDiv > COUNT_FLT_AVR_LOAD) {
			outDataInt16 = summAvrLoad / COUNT_FLT_AVR_LOAD;
			countDiv = 0;
			summAvrLoad = 0;
		}

		summAvrLoad = summAvrLoad + (int16_t)(outData >> 8);
		countDiv++;

		HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET);

		delay_tick_one();
		delay_tick_one();

		HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);

		switchGetData = 2;
		break;
	}

	return outDataInt16;
}
