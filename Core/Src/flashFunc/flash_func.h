/*
 * flash_func.h
 *
 *  Created on: 9 ����. 2021 �.
 *      Author: Uneld
 */

#ifndef INC_FLASH_FUNC_H_
#define INC_FLASH_FUNC_H_

#include "main.h"

#define PARAMETERS_PAGE_ADDRESS 0x08001400

extern uint8_t flagErroFlash;
extern uint8_t	errHallFlash;

void flash_erase();
uint8_t flash_program (uint32_t address, uint32_t* data_fl, uint16_t size_to_fl);

#endif /* INC_FLASH_FUNC_H_ */
