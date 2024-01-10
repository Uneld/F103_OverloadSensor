/*
 * flash_func.c
 *
 *  Created on: 9 нояб. 2021 г.
 *      Author: Uneld
 */

#include "flash_func.h"

uint8_t flagErroFlash;
uint8_t errHallFlash;

void flash_erase() {
	FLASH_EraseInitTypeDef EraseInitStruct;

	EraseInitStruct.Banks = FLASH_BANK_1;
	EraseInitStruct.NbPages = 1;
	EraseInitStruct.PageAddress = PARAMETERS_PAGE_ADDRESS;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;

	uint32_t page_error = 0;
	HAL_FLASH_Unlock();
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &page_error) != HAL_OK) {
		flagErroFlash = 1;
		while (1) {
		}
	}
	HAL_FLASH_Lock();
}

uint8_t flash_program(uint32_t address, uint32_t *data_fl, uint16_t size_to_fl) {
	HAL_FLASH_Unlock();

	uint16_t countFor = ((size_to_fl - 1) / sizeof(uint32_t) + 1);
	uint8_t halRes = 0;
	for (uint32_t i = 0; i < countFor; i++) { // 128 - 512/4
		halRes = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, *data_fl);
		if (halRes != HAL_OK) {
			errHallFlash = HAL_FLASH_GetError();
			flagErroFlash = 1;
			while (1) {
			}
		}
		address += 4;
		data_fl++;
	}
	HAL_FLASH_Lock();
	return halRes;
}
