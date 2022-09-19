/*
 * tmc4671.c
 *
 *  Created on: 4 Sep 2022
 *      Author: matveev
 */

#include "tmc4671.h"
#include "main.h"

#include "cmsis_os.h"

#include "tmc/ic/TMC4671/TMC4671.h"

extern SPI_HandleTypeDef hspi1;
extern osSemaphoreId_t spi1RxCompletedHandle;
extern osSemaphoreId_t spi1TxCompletedHandle;

static Tmc4671State tmc4671State = TMC4671_DISABLED;

void initTmc4671() {
	setTmc4671State(TMC4671_ENABLED);

	// Motor type &  PWM configuration
	tmc4671_writeInt(TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x0003000E);
	tmc4671_writeInt(TMC4671_PWM_POLARITIES, 0x00000000);
	tmc4671_writeInt(TMC4671_PWM_MAXCNT, 0x00000F9F);
	tmc4671_writeInt(TMC4671_PWM_BBM_H_BBM_L, 0x00001414);
	tmc4671_writeInt(TMC4671_PWM_SV_CHOP, 0x00000007);

	// ADC configuration
	tmc4671_writeInt(TMC4671_ADC_I_SELECT, 0x18000100);
	tmc4671_writeInt(TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
	tmc4671_writeInt(TMC4671_dsADC_MCLK_A, 0x20000000);
	tmc4671_writeInt(TMC4671_dsADC_MCLK_B, 0x20000000);
	tmc4671_writeInt(TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E);
	tmc4671_writeInt(TMC4671_ADC_I0_SCALE_OFFSET, 0x010083A3);
	tmc4671_writeInt(TMC4671_ADC_I1_SCALE_OFFSET, 0xFF008181);

	// ABN encoder settings
	tmc4671_writeInt(TMC4671_ABN_DECODER_MODE, 0x00000000);
	tmc4671_writeInt(TMC4671_ABN_DECODER_PPR, 0x00009C40);
	tmc4671_writeInt(TMC4671_ABN_DECODER_COUNT, 0x00001E71);
	tmc4671_writeInt(TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);

	// Limits
	tmc4671_writeInt(TMC4671_PID_TORQUE_FLUX_LIMITS, 0x000003E8);

	// PI settings
	tmc4671_writeInt(TMC4671_PID_TORQUE_P_TORQUE_I, 0x01000100);
	tmc4671_writeInt(TMC4671_PID_FLUX_P_FLUX_I, 0x01000100);

	// ===== ABN encoder test drive =====

	setTmc4671State(TMC4671_ENABLED);

	// Init encoder (mode 0)
	tmc4671_writeInt(TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);
	tmc4671_writeInt(TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
	tmc4671_writeInt(TMC4671_PHI_E_SELECTION, 0x00000001);
	tmc4671_writeInt(TMC4671_PHI_E_EXT, 0x00000000);
	tmc4671_writeInt(TMC4671_UQ_UD_EXT, 0x00000FA0);
	osDelay(1000);
	tmc4671_writeInt(TMC4671_ABN_DECODER_COUNT, 0x00000000);

	// Feedback selection
	tmc4671_writeInt(TMC4671_PHI_E_SELECTION, 0x00000003);
	tmc4671_writeInt(TMC4671_VELOCITY_SELECTION, 0x00000009);

	// Switch to torque mode
	tmc4671_writeInt(TMC4671_MODE_RAMP_MODE_MOTION, 0x00000001);

	// Rotate right
	tmc4671_writeInt(TMC4671_PID_TORQUE_FLUX_TARGET, 0x03E80000);
	osDelay(300);

	// Rotate left
	tmc4671_writeInt(TMC4671_PID_TORQUE_FLUX_TARGET, 0xFC180000);
	osDelay(300);

	// Stop
	tmc4671_writeInt(TMC4671_PID_TORQUE_FLUX_TARGET, 0x00000000);

}

void setTmc4671State(Tmc4671State state) {
	tmc4671State = state;
	switch (tmc4671State) {
	case TMC4671_ENABLED:
		HAL_GPIO_WritePin(F_EN_GPIO_Port, F_EN_Pin, GPIO_PIN_SET);
		break;
	case TMC4671_DISABLED:
		HAL_GPIO_WritePin(F_EN_GPIO_Port, F_EN_Pin, GPIO_PIN_RESET);
		break;
	}
}

Tmc4671State getTmc4671State() {
	return tmc4671State;
}

uint8_t hasTmc4671Failure() {
	return HAL_GPIO_ReadPin(F_STATUS_GPIO_Port, F_STATUS_Pin);
}

//uint8_t tmc4671_readwriteByte(uint8_t motor, uint8_t data, uint8_t isAddress, uint8_t isReadOp, uint8_t lastTransfer) {
//	uint8_t result;
//
//	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_RESET);
//
//	HAL_SPI_TransmitReceive(&hspi1, &data, &result, 1, HAL_MAX_DELAY);
//
//	if (lastTransfer) {
//		HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_SET);
//	}
//
//	if (isAddress && isReadOp) {
//		// >500nS delay
//		volatile uint8_t counter = 8;
//		while (counter--) {
//			asm("");
//		}
//	}
//
//  return result;
//}

typedef struct {
	uint8_t address;
	uint32_t data;
} __attribute__((packed)) Tmc4671Datagram;

int32_t tmc4671_readInt(uint8_t address) {
	// clear write bit
	address &= 0x7F;

	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit_DMA(&hspi1, &address, 1);
	osSemaphoreAcquire(spi1TxCompletedHandle, 10);

	// >500nS delay
	uint8_t counter = 8;
	while (counter--) {
		asm("");
	}

	int32_t result;
	HAL_SPI_Receive_DMA(&hspi1, (uint8_t*) &result, 4);
	osSemaphoreAcquire(spi1RxCompletedHandle, 10);
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_SET);

	return ((result >> 24) & 0xff) | // move byte 3 to byte 0
			((result << 8) & 0xff0000) | // move byte 1 to byte 2
			((result >> 8) & 0xff00) | // move byte 2 to byte 1
			((result << 24) & 0xff000000); // byte 0 to byte 3;;
}

void tmc4671_writeInt(uint8_t address, int32_t value) {
	Tmc4671Datagram tx;
	tx.address = address | 0x80;
	tx.data = ((value >> 24) & 0xff) | // move byte 3 to byte 0
			((value << 8) & 0xff0000) | // move byte 1 to byte 2
			((value >> 8) & 0xff00) | // move byte 2 to byte 1
			((value << 24) & 0xff000000); // byte 0 to byte 3;

	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*) &tx, 5);
	osSemaphoreAcquire(spi1TxCompletedHandle, 10);

	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_SET);
}
