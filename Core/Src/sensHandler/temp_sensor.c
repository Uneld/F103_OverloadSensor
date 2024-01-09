/*
 * temp_sensor.c
 *
 *  Created on: Jan 9, 2024
 *      Author: Uneld
 */

#include "../sensHandler/temp_sensor.h"

#define COUNT_REQUEST     20
#define COUNT_FLT_AVR_TEMP 20

uint16_t adc_buf[COUNT_REQUEST] = { 0, };
int32_t summAvrTemp = 0, countDiv = 0, valAvrTemp = 0;
uint32_t adc_ch = 0;

void init_conversion_tempSensor() {
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buf, COUNT_REQUEST);
}

float proc_tempSensor() {

	float outFltTemp = 0;
	uint32_t adc_ch_sum = 0;
	int32_t temperature = 0;

	for (uint16_t i = 0; i < COUNT_REQUEST; i++) {
		adc_ch_sum += adc_buf[i];
	}
	adc_ch = adc_ch_sum / COUNT_REQUEST;

	temperature = (165 * adc_ch - 102380) / 2048;

	if (countDiv > COUNT_FLT_AVR_TEMP) {
		valAvrTemp = summAvrTemp / COUNT_FLT_AVR_TEMP;
		countDiv = 0;
		summAvrTemp = 0;
	}
	summAvrTemp += temperature;
	countDiv++;

	outFltTemp = temperature * 0.5 + valAvrTemp * 0.5;

	return outFltTemp;
}
