/*
 * load_cell.h
 *
 *  Created on: Jan 9, 2024
 *      Author: Uneld
 */

#ifndef SRC_SENSHANDLER_LOAD_CELL_H_
#define SRC_SENSHANDLER_LOAD_CELL_H_

#include "main.h"

void setCountFltAvrLoad(uint8_t _countFltAvrLoad);
int16_t proc_hx711_getValue();

#endif /* SRC_SENSHANDLER_LOAD_CELL_H_ */
