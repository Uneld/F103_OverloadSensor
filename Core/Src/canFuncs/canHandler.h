/*
 * canHandler.h
 *
 *  Created on: Jan 9, 2024
 *      Author: Uneld
 */

#include "main.h"

const uint32_t
			CAN_STD = 0,
			CAN_EXT = 1;

uint32_t canTXMessage(uint32_t IDE, uint32_t ID, uint32_t RTR, uint32_t DLC, uint8_t *data);
uint32_t canStart();
uint32_t canSetHWFlt(uint32_t IDE, uint32_t fltrNum, uint32_t FIFO, uint32_t filterID, uint32_t maskID);
uint8_t canRxMessage(uint32_t RxFifo, CAN_RxHeaderTypeDef* pRxHeader, uint8_t* rxData);
