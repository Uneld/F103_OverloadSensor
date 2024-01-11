/*
 * load_cell.c
 *
 *  Created on: Jan 9, 2024
 *      Author: Uneld
 */

#include "load_cell.h"
#include "delay_user.h"

uint8_t countFltAvrLoad = 2;

void setCountFltAvrLoad(uint8_t _countFltAvrLoad){
	countFltAvrLoad = _countFltAvrLoad;
}

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
		if ((HX711_DT_GPIO_Port->IDR & (HX711_DT_Pin)) == 0) {
			switchGetData = 3;
			outData = 0;
		}
		break;
	case 3:
		for (uint8_t i = 0; i < 24; i++) {
			HX711_SCK_GPIO_Port->BSRR = HX711_SCK_Pin;

			delay_tick_one();
			delay_tick_one();
			HX711_SCK_GPIO_Port->BRR = HX711_SCK_Pin;

			outData = (outData << 1) + ((HX711_DT_GPIO_Port->IDR & HX711_DT_Pin) != 0);
		}

		if (countDiv > countFltAvrLoad) {
			outDataInt16 = summAvrLoad / countFltAvrLoad;
			countDiv = 0;
			summAvrLoad = 0;
		}

		summAvrLoad = summAvrLoad + (int16_t)(outData >> 8);
		countDiv++;

		HX711_SCK_GPIO_Port->BSRR = HX711_SCK_Pin;

		delay_tick_one();
		delay_tick_one();

		HX711_SCK_GPIO_Port->BRR = HX711_SCK_Pin;

		switchGetData = 2;
		break;
	}

	return outDataInt16;
}
