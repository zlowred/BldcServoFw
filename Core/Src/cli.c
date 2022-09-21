/*
 * cli.c
 *
 *  Created on: Sep 21, 2022
 *      Author: matveev
 */

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "cli.h"
#include "eeprom_emul.h"
#include "cmsis_os.h"

extern volatile uint32_t isErasing;

typedef enum {
	START,
	DFU_PRE,
	DFU_POST,
	CRC_PRE,
	CRC_POST,
	READ_FLASH_PRE,
	READ_FLASH_POST,
	WRITE_FLASH_PRE,
	WRITE_FLASH_POST,
} CliState;

static char dfu[] = "dfu(";
static char crcTest[] = "crcTest(";
static char readFlash[] = "readFlash(";
static char writeFlash[] = "writeFlash(";

static CliState state = START;
static char token[50];
static int tokenIdx;

void handleNewline();
void incompleteCommand();
void doCrcTest();
char commandMatch(char command[]);
void doReadFlash();
void doWriteFlash();

void processCli(char c) {
	if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')
			|| c == '(' || c == ')') {
		if (tokenIdx < sizeof(token)) {
			token[tokenIdx++] = c;
		}
	}

	switch (state) {
	case START:
		if (commandMatch(dfu)) {
			state = DFU_PRE;
			tokenIdx = 0;
		} else if (commandMatch(crcTest)) {
			state = CRC_PRE;
			tokenIdx = 0;
		} else if (commandMatch(readFlash)) {
			state = READ_FLASH_PRE;
			tokenIdx = 0;
		} else if (commandMatch(writeFlash)) {
			state = WRITE_FLASH_PRE;
			tokenIdx = 0;
		}
		break;
	case DFU_PRE:
		if (tokenIdx == 1 && token[0] == ')') {
			state = DFU_POST;
			tokenIdx = 0;
		}
		break;
	case DFU_POST:
		break;
	case CRC_PRE:
		if (tokenIdx == 1 && token[0] == ')') {
			state = CRC_POST;
			tokenIdx = 0;
		}
		break;
	case CRC_POST:
		break;
	case READ_FLASH_PRE:
		if (tokenIdx == 1 && token[0] == ')') {
			state = READ_FLASH_POST;
			tokenIdx = 0;
		}
		break;
	case READ_FLASH_POST:
		break;
	case WRITE_FLASH_PRE:
		if (tokenIdx == 1 && token[0] == ')') {
			state = WRITE_FLASH_POST;
			tokenIdx = 0;
		}
		break;
	case WRITE_FLASH_POST:
		break;
	}

	if (c == '\n') {
		handleNewline();
	}
}

void handleNewline() {
	if (tokenIdx != 0) {
		printf("Can't parse: [%.*s%s]\n", tokenIdx, token,
				tokenIdx == sizeof(token) ? "..." : "");
		tokenIdx = 0;
		state = START;
		return;
	}
	switch (state) {
	case START:
		break;
	case DFU_PRE:
	case CRC_PRE:
	case READ_FLASH_PRE:
	case WRITE_FLASH_PRE:
		incompleteCommand();
		break;
	case DFU_POST:
		printf("Entering DFU mode\n");
		startDfu();
		break;
		incompleteCommand();
		break;
	case CRC_POST:
		doCrcTest();
		tokenIdx = 0;
		state = START;
		break;
	case READ_FLASH_POST:
		doReadFlash();
		tokenIdx = 0;
		state = START;
		break;
	case WRITE_FLASH_POST:
		doWriteFlash();
		tokenIdx = 0;
		state = START;
		break;
	}
}

void incompleteCommand() {
	printf("Expected ')' at end of input\n");
	tokenIdx = 0;
	state = START;
}

char commandMatch(char command[]) {
	return tokenIdx == strlen(command) && !memcmp(command, token, strlen(command));
}

void doCrcTest() {
	printf("CRC test\n");
	uint32_t crcdata[] = { 0x12345678, 0x87654321 };
	uint32_t crc = calculateCrc((uint8_t*) crcdata, 2);
	printf("crc1: %08lx\n", crc);
	crc = calculateCrc((uint8_t*) crcdata, 8);
	printf("crc2: %08lx\n", crc);
	crcdata[1] = 0x12345678;
	crcdata[0] = 0x87654321;
	crc = calculateCrc((uint8_t*) crcdata, 2);
	printf("crc3: %08lx\n", crc);
	// https://crccalc.com/?crc=7856&method=crc32&datatype=hex&outtype=0
	uint8_t crcdata2[] = { 0x78, 0x56 };
	crc = calculateCrc(crcdata2, 2);
	printf("crc4: %08lx\n", crc);
}

void doReadFlash() {
	uint32_t flashData;
	EE_ReadVariable32bits(/* idx = */1, &flashData);
	printf("Read: 0x%08lX\n", flashData);
}

void doWriteFlash() {
	HAL_FLASH_Unlock();
	EE_Status ee_status = EE_OK;

	uint32_t flashData = xTaskGetTickCount();

	ee_status = EE_WriteVariable32bits(/* idx = */1, flashData);

	/* Start cleanup IT mode, if cleanup is needed */
	if ((ee_status & EE_STATUSMASK_CLEANUP ) == EE_STATUSMASK_CLEANUP) {
		isErasing = 1;
		ee_status |= EE_CleanUp_IT();
	}
	if ((ee_status & EE_STATUSMASK_ERROR ) == EE_STATUSMASK_ERROR) {
		Error_Handler();
	}

	while (isErasing) {
	}

	/* Lock the Flash Program Erase controller */
	HAL_FLASH_Lock();
	printf("Written: 0x%08lX\n", flashData);
}
