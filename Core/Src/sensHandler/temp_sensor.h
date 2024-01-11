/*
 * temp_sensor.h
 *
 *  Created on: Jan 9, 2024
 *      Author: Uneld
 */

#ifndef SRC_SENSHANDLER_TEMP_SENSOR_H_
#define SRC_SENSHANDLER_TEMP_SENSOR_H_

#include "main.h"
#include "adc.h"

void init_conversion_tempSensor();
float proc_tempSensor();
uint8_t checkErrorTemp(uint32_t adc_ch);
uint8_t getTempFlagErrorsPresent();
uint8_t getTempErrSRCHi();
uint8_t getTempErrSRCLow();

#endif /* SRC_SENSHANDLER_TEMP_SENSOR_H_ */
