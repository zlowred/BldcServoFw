/*
 * drv8321h.c
 *
 *  Created on: 4 Sep 2022
 *      Author: matveev
 */

#include "drv8320h.h"

#include "main.h"
#include "tmc4671.h"

typedef enum {
	OPEN,
	HIGH,
	LOW,
} PinState;

void setPinState(GPIO_TypeDef *port, uint16_t pin, PinState pinState) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pin;
	if (pinState == OPEN) {
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	} else {
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	}
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(port, &GPIO_InitStruct);
	if (pinState == HIGH) {
		HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	} else if (pinState == LOW) {
		HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
	}
}

void initDrv8320h() {
	Tmc4671State tmc4671State = getTmc4671State();
	setTmc4671State(TMC4671_DISABLED);

	setDrv8320hMode(DefaultDrv8320hMode);
  setDrv8320hIDrive(DefaultDrv8320hDriveCurrent);
  setDrv8320hVdsOvercurrent(DefaultDrv8320hVdsOvercurrent);

	setTmc4671State(tmc4671State);
}

uint8_t hasDrv8320hFailure() {
  return !HAL_GPIO_ReadPin(M_FAULT_GPIO_Port, M_FAULT_Pin);
}

void setDrv8320hMode(Drv8320hMode mode) {
	Tmc4671State tmc4671State = getTmc4671State();
	setTmc4671State(TMC4671_DISABLED);

	switch(mode) {
	case PWM_6X:
		setPinState(MODE_0_GPIO_Port, MODE_0_Pin, LOW);
		setPinState(MODE_45_GPIO_Port, MODE_45_Pin, OPEN);
		break;
	case PWM_3X:
		setPinState(MODE_0_GPIO_Port, MODE_0_Pin, OPEN);
		setPinState(MODE_45_GPIO_Port, MODE_45_Pin, LOW);
	break;
	case PWM_1X:
		setPinState(MODE_0_GPIO_Port, MODE_0_Pin, OPEN);
		setPinState(MODE_45_GPIO_Port, MODE_45_Pin, OPEN);
	break;
	case PWM_INDEPENDENT:
		setPinState(MODE_0_GPIO_Port, MODE_0_Pin, HIGH);
		setPinState(MODE_45_GPIO_Port, MODE_45_Pin, OPEN);
	break;
	}

	setTmc4671State(tmc4671State);
}

void setDrv8320hIDrive(Drv8320hDriveCurrent current) {
	Tmc4671State tmc4671State = getTmc4671State();
	setTmc4671State(TMC4671_DISABLED);

	switch(current) {
	case IDRV_10MA:
		setPinState(IDRIVE_0_GPIO_Port, IDRIVE_0_Pin, LOW);
		setPinState(IDRIVE_18_GPIO_Port, IDRIVE_18_Pin, OPEN);
		setPinState(IDRIVE_75_GPIO_Port, IDRIVE_75_Pin, OPEN);
	break;
	case IDRV_30MA:
		setPinState(IDRIVE_0_GPIO_Port, IDRIVE_0_Pin, OPEN);
		setPinState(IDRIVE_18_GPIO_Port, IDRIVE_18_Pin, LOW);
		setPinState(IDRIVE_75_GPIO_Port, IDRIVE_75_Pin, OPEN);
	break;
	case IDRV_60MA:
		setPinState(IDRIVE_0_GPIO_Port, IDRIVE_0_Pin, OPEN);
		setPinState(IDRIVE_18_GPIO_Port, IDRIVE_18_Pin, OPEN);
		setPinState(IDRIVE_75_GPIO_Port, IDRIVE_75_Pin, LOW);
	break;
	case IDRV_120MA:
		setPinState(IDRIVE_0_GPIO_Port, IDRIVE_0_Pin, OPEN);
		setPinState(IDRIVE_18_GPIO_Port, IDRIVE_18_Pin, OPEN);
		setPinState(IDRIVE_75_GPIO_Port, IDRIVE_75_Pin, OPEN);
	break;
	case IDRV_260MA:
		setPinState(IDRIVE_0_GPIO_Port, IDRIVE_0_Pin, OPEN);
		setPinState(IDRIVE_18_GPIO_Port, IDRIVE_18_Pin, OPEN);
		setPinState(IDRIVE_75_GPIO_Port, IDRIVE_75_Pin, HIGH);
	break;
	case IDRV_570MA:
		setPinState(IDRIVE_0_GPIO_Port, IDRIVE_0_Pin, OPEN);
		setPinState(IDRIVE_18_GPIO_Port, IDRIVE_18_Pin, HIGH);
		setPinState(IDRIVE_75_GPIO_Port, IDRIVE_75_Pin, OPEN);
	break;
	case IDRV_1000MA:
		setPinState(IDRIVE_0_GPIO_Port, IDRIVE_0_Pin, HIGH);
		setPinState(IDRIVE_18_GPIO_Port, IDRIVE_18_Pin, OPEN);
		setPinState(IDRIVE_75_GPIO_Port, IDRIVE_75_Pin, OPEN);
	break;
	}


	setTmc4671State(tmc4671State);
}

void setDrv8320hVdsOvercurrent(Drv8320hVdsOvercurrent tripVoltage) {
	Tmc4671State tmc4671State = getTmc4671State();
	setTmc4671State(TMC4671_DISABLED);

	switch(tripVoltage) {
	case VDS_60MV:
		setPinState(VDS_0_GPIO_Port, VDS_0_Pin, LOW);
		setPinState(VDS_18_GPIO_Port, VDS_18_Pin, OPEN);
		setPinState(VDS_75_GPIO_Port, VDS_75_Pin, OPEN);
	break;
	case VDS_130MV:
		setPinState(VDS_0_GPIO_Port, VDS_0_Pin, OPEN);
		setPinState(VDS_18_GPIO_Port, VDS_18_Pin, LOW);
		setPinState(VDS_75_GPIO_Port, VDS_75_Pin, OPEN);
	break;
	case VDS_260MV:
		setPinState(VDS_0_GPIO_Port, VDS_0_Pin, OPEN);
		setPinState(VDS_18_GPIO_Port, VDS_18_Pin, OPEN);
		setPinState(VDS_75_GPIO_Port, VDS_75_Pin, LOW);
	break;
	case VDS_600MV:
		setPinState(VDS_0_GPIO_Port, VDS_0_Pin, OPEN);
		setPinState(VDS_18_GPIO_Port, VDS_18_Pin, OPEN);
		setPinState(VDS_75_GPIO_Port, VDS_75_Pin, OPEN);
	break;
	case VDS_1130MV:
		setPinState(VDS_0_GPIO_Port, VDS_0_Pin, OPEN);
		setPinState(VDS_18_GPIO_Port, VDS_18_Pin, OPEN);
		setPinState(VDS_75_GPIO_Port, VDS_75_Pin, HIGH);
	break;
	case VDS_1880MV:
		setPinState(VDS_0_GPIO_Port, VDS_0_Pin, OPEN);
		setPinState(VDS_18_GPIO_Port, VDS_18_Pin, HIGH);
		setPinState(VDS_75_GPIO_Port, VDS_75_Pin, OPEN);
	break;
	case VDS_DISABLED:
		setPinState(VDS_0_GPIO_Port, VDS_0_Pin, HIGH);
		setPinState(VDS_18_GPIO_Port, VDS_18_Pin, OPEN);
		setPinState(VDS_75_GPIO_Port, VDS_75_Pin, OPEN);
	break;
	}

	setTmc4671State(tmc4671State);
}

void resetDrv8320hFault() {
	Tmc4671State tmc4671State = getTmc4671State();
	setTmc4671State(TMC4671_ENABLED);
	// ~20uS delay
	uint8_t counter = 255;
	while (counter--) {
		asm("");
	}
	setTmc4671State(TMC4671_DISABLED);
	// ~20uS delay
	counter = 255;
	while (counter--) {
		asm("");
	}
	setTmc4671State(TMC4671_ENABLED);
	// ~20uS delay
	counter = 255;
	while (counter--) {
		asm("");
	}
	setTmc4671State(tmc4671State);

}
