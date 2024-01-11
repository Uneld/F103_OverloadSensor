/*
 * temp_sensor.c
 *
 *  Created on: Jan 9, 2024
 *      Author: Uneld
 */

#include "temp_sensor.h"

#define COUNT_REQUEST     20
#define COUNT_FLT_AVR_TEMP 20
#define ERROR_SRC_HI_ACPVAL 2000
#define ERROR_SRC_LO_ACPVAL 50
#define TIME_CHECK_ERROR 1000 //+- 1000ms

uint16_t adc_buf[COUNT_REQUEST] = { 0, };
int32_t summAvrTemp = 0, countDiv = 0, valAvrTemp = 0;
uint32_t adc_ch = 0;


uint8_t errSRCHi, errSRCLow;
uint8_t flagErrorsPresent = 0;

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

	flagErrorsPresent = checkErrorTemp(adc_ch);
	if (flagErrorsPresent) {
		outFltTemp = 0;
	}

	return outFltTemp;
}

uint8_t checkErrorTemp(uint32_t adc_ch) {
	static uint16_t deltaTimeMs, oldTimeMs;

	uint8_t flagCheck = (adc_ch <= ERROR_SRC_LO_ACPVAL) || (adc_ch >= ERROR_SRC_HI_ACPVAL);

	if (flagCheck) {
		deltaTimeMs = HAL_GetTick() - oldTimeMs;
		if (deltaTimeMs > TIME_CHECK_ERROR) {
			if (adc_ch <= ERROR_SRC_LO_ACPVAL && !errSRCHi) {
				errSRCLow = 1;
				errSRCHi = 0;
			} else {
				errSRCLow = 0;
				errSRCHi = 1;
			}

			return 1;
		}
	} else {
		oldTimeMs = HAL_GetTick();
		errSRCLow = errSRCHi = 0;
	}

	return 0;
}

uint8_t getTempFlagErrorsPresent() {
	return flagErrorsPresent;
}

uint8_t getTempErrSRCHi() {
	return errSRCHi;
}

uint8_t getTempErrSRCLow() {
	return errSRCLow;
}
