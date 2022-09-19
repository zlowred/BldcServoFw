/*
 * acs711.c
 *
 *  Created on: 4 Sep 2022
 *      Author: matveev
 */

#include "acs711.h"

#include "main.h"

uint8_t hasAcs711Failure() {
	return HAL_GPIO_ReadPin(CUR_SENSE_FAULT_GPIO_Port, CUR_SENSE_FAULT_Pin);
}
