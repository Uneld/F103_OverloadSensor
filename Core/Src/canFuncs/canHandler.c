/*
 * canHandler.c
 *
 *  Created on: Jan 9, 2024
 *      Author: Uneld
 */

#include "can.h"

/**
 *@brief fun transmit msg to CAN
 *@param IDE format CAN msg 0-Std or 1-Ext
 *@param ID CAN id
 *@param RTR Request bit 0 - Data 1 - Remote
 *@param DLC Length msg data 0..8
 *@param *data pointer to tx data
 *@return 0 status OK
 *@error  return 1 status msg not transmit
 */
uint32_t canTXMessage(uint32_t IDE, uint32_t ID, uint32_t RTR, uint32_t DLC, uint8_t *data) {

	CAN_TxHeaderTypeDef pHeader;
	uint32_t TxMailbox;

	if (IDE == 0) {
		pHeader.StdId = ID;
		pHeader.IDE = CAN_ID_STD; //set identifier to standard
	} else {
		pHeader.ExtId = ID;
		pHeader.IDE = CAN_ID_EXT; //set identifier to extended
	}

	if (RTR == 0) {
		pHeader.RTR = CAN_RTR_DATA;
	} else {
		pHeader.RTR = CAN_RTR_REMOTE;
	}
	pHeader.DLC = DLC; //give message size of 1 byte

	return HAL_CAN_AddTxMessage(&hcan, &pHeader, data, &TxMailbox);
}

/**
 *@brief fun start CAN
 *@param canNum  Can number 1 or 2
 *@return 0 - OK
 *@error return 1 - CAN not start
 */
uint32_t canStart() {
	return HAL_CAN_Start(&hcan);
}

/**
 *@brief fun set hardware CAN filt
 *@param canNum  Can number 1 or 2
 *@param IDE format CAN msg 0-Std or 1-Ext
 *@param fltrNum bank number 1-13
 *@param filterID msg id
 *@return 0 status OK
 *@error  return 1 filter not assigned
 *@error  return 2 wrong canNum
 *@error  return 3 wrong fltrNum
 */
uint32_t canSetHWFlt(uint32_t IDE, uint32_t fltrNum, uint32_t FIFO, uint32_t filterID, uint32_t maskID) {

	CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure

	if (fltrNum > 13 || fltrNum < 1) {
		return 3;
	}

	if (IDE) {
		sFilterConfig.FilterIdHigh = ((filterID << 3) >> 16);
		sFilterConfig.FilterIdLow = (filterID << 3) | CAN_ID_EXT;
		sFilterConfig.FilterMaskIdHigh = ((maskID << 3) >> 16);
		sFilterConfig.FilterMaskIdLow = (maskID << 3) | CAN_ID_EXT;
	} else {
		sFilterConfig.FilterIdHigh = filterID << 5;
		sFilterConfig.FilterIdLow = 0;
		sFilterConfig.FilterMaskIdHigh = 0xFFFF << 5;
		sFilterConfig.FilterMaskIdLow = 0;
	}

	sFilterConfig.FilterFIFOAssignment = FIFO; //set fifo assignment
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	sFilterConfig.FilterBank = fltrNum;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) { //configure CAN filter
		return 1;
	}

	return 0;
}

//CAN_RX_FIFO0
//CAN_RX_FIFO1
uint8_t canRxMessage(uint32_t RxFifo, CAN_RxHeaderTypeDef *pRxHeader, uint8_t *rxData) {
	return HAL_CAN_GetRxMessage(&hcan, RxFifo, pRxHeader, rxData);
}

