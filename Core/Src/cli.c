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
#include "acs711.h"
#include "tmc4671.h"
#include "drv8320h.h"

extern volatile uint32_t isErasing;

typedef enum {
	SYNTAX_ERROR,
	START,
	DFU_PRE,
	DFU_POST,
	CRC_PRE,
	CRC_POST,
	READ_FLASH_PRE,
	READ_FLASH_POST,
	WRITE_FLASH_PRE,
	WRITE_FLASH_POST,
	INIT_PRE,
	INIT_POST,
	RESET_DRV_PRE,
	RESET_DRV_POST,
	RS_485_PRE,
	RS_485_POST,
	FD_CAN_PRE,
	FD_CAN_POST,
	DEV,
	CONFIG,
	ADDR,
	ADDR_ASSIGN,
	NEXT,
	NEXT_ASSIGN,
} CliState;

static char dfu[] = "dfu(";
static char crcTest[] = "crcTest(";
static char readFlash[] = "readFlash(";
static char writeFlash[] = "writeFlash(";
static char init[] = "init(";
static char resetDrv[] = "resetDrv(";
static char rs485Test[] = "rs485Test(";
static char fdCanTest[] = "fdCanTest(";
static char dev[] = "dev";
static char config[] = "config";
static char addr[] = "addr";
static char next[] = "next";

static CliState state = START;
static char token[50];
static int tokenIdx;
static char origin[100];
static int originIdx;

void handleNewline();
void handleDot();
void handleAssignment();
void incompleteCommand();
void doCrcTest();
char commandMatch(char command[]);
void doReadFlash();
void doWriteFlash();
void doInit();
void doResetDrv();
void invalidValue();

void processCli(char c) {
	if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')
			|| c == '(' || c == ')') {
		if (tokenIdx < sizeof(token)) {
			token[tokenIdx++] = c;
		}
	}
	if (originIdx < sizeof(origin) && c != '\n') {
		origin[originIdx++] = c;
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
		} else if (commandMatch(init)) {
			state = INIT_PRE;
			tokenIdx = 0;
		} else if (commandMatch(resetDrv)) {
			state = RESET_DRV_PRE;
			tokenIdx = 0;
		} else if (commandMatch(rs485Test)) {
			state = RS_485_PRE;
			tokenIdx = 0;
		} else if (commandMatch(fdCanTest)) {
			state = FD_CAN_PRE;
			tokenIdx = 0;
		}
		break;
	case SYNTAX_ERROR:
	case DFU_POST:
	case CRC_POST:
	case READ_FLASH_POST:
	case WRITE_FLASH_POST:
	case INIT_POST:
	case RESET_DRV_POST:
	case RS_485_POST:
	case FD_CAN_POST:
	case DEV:
	case CONFIG:
	case ADDR:
	case ADDR_ASSIGN:
	case NEXT:
	case NEXT_ASSIGN:
		break;
	case DFU_PRE:
		if (tokenIdx == 1 && token[0] == ')') {
			state = DFU_POST;
			tokenIdx = 0;
		}
		break;
	case CRC_PRE:
		if (tokenIdx == 1 && token[0] == ')') {
			state = CRC_POST;
			tokenIdx = 0;
		}
		break;
	case READ_FLASH_PRE:
		if (tokenIdx == 1 && token[0] == ')') {
			state = READ_FLASH_POST;
			tokenIdx = 0;
		}
		break;
	case WRITE_FLASH_PRE:
		if (tokenIdx == 1 && token[0] == ')') {
			state = WRITE_FLASH_POST;
			tokenIdx = 0;
		}
		break;
	case INIT_PRE:
		if (tokenIdx == 1 && token[0] == ')') {
			state = INIT_POST;
			tokenIdx = 0;
		}
		break;
	case RESET_DRV_PRE:
		if (tokenIdx == 1 && token[0] == ')') {
			state = RESET_DRV_POST;
			tokenIdx = 0;
		}
		break;
	case RS_485_PRE:
		if (tokenIdx == 1 && token[0] == ')') {
			state = RS_485_POST;
			tokenIdx = 0;
		}
		break;
	case FD_CAN_PRE:
		if (tokenIdx == 1 && token[0] == ')') {
			state = FD_CAN_POST;
			tokenIdx = 0;
		}
		break;
	}

	if (c == '\n') {
		handleNewline();
	} else if (c == '.') {
		handleDot();
	} else if (c == '=') {
		handleAssignment();
	}
}

void handleDot() {
	if (tokenIdx == 0) {
		state = SYNTAX_ERROR;
		return;
	}

	switch (state) {
	case START:
		if (commandMatch(dev)) {
			state = DEV;
			tokenIdx = 0;
		}
		break;
	case DEV:
		if (commandMatch(config)) {
			state = CONFIG;
			tokenIdx = 0;
		}
		break;
	case CONFIG:
		if (commandMatch(addr)) {
			state = ADDR;
			tokenIdx = 0;
		} else if (commandMatch(next)) {
			state = NEXT;
			tokenIdx = 0;
		}
		break;

	default:
		break;
	}
}

void handleAssignment() {
	if (tokenIdx == 0) {
		state = SYNTAX_ERROR;
		return;
	}

	switch (state) {
	case CONFIG:
		if (commandMatch(addr)) {
			state = ADDR_ASSIGN;
			tokenIdx = 0;
		} else if (commandMatch(next)) {
			state = NEXT_ASSIGN;
			tokenIdx = 0;
		}
		break;

	default:
		break;
	}
}

void handleNewline() {
	switch (state) {
	case SYNTAX_ERROR:
		printf("Syntax error: [%.*s%s]\n", originIdx, origin,
				originIdx == sizeof(origin) ? "..." : "");
		state = START;
		tokenIdx = 0;
		originIdx = 0;
		break;

	case START:
		if (commandMatch(dev)) {
			state = DEV;
			tokenIdx = 0;
		}
		break;
	case DEV:
		if (commandMatch(config)) {
			state = CONFIG;
			tokenIdx = 0;
		}
		break;
	case CONFIG:
		if (commandMatch(addr)) {
			state = ADDR;
			tokenIdx = 0;
		} else if (commandMatch(next)) {
			state = NEXT;
			tokenIdx = 0;
		}
		break;
	case ADDR_ASSIGN:
		// TODO: Actually parse and return error if failed
		if (tokenIdx == 0) {
			invalidValue();
		} else {
			printf("Assign [%.*s] to dev.config.addr\n", tokenIdx, token);
		}
		tokenIdx = 0;
		originIdx = 0;
		state = START;
		return;
	case NEXT_ASSIGN:
		// TODO: Actually parse and return error if failed
		if (tokenIdx == 0) {
			invalidValue();
		} else {
			printf("Assign [%.*s] to dev.config.next\n", tokenIdx, token);
		}
		tokenIdx = 0;
		originIdx = 0;
		state = START;
		return;

	default:
		break;
	}

	if (tokenIdx != 0) {
		printf("Can't parse: [%.*s%s]\n", originIdx, origin,
				originIdx == sizeof(origin) ? "..." : "");
		tokenIdx = 0;
		originIdx = 0;
		state = START;
		return;
	}
	switch (state) {
	case SYNTAX_ERROR:
	case START:
	case ADDR_ASSIGN:
	case NEXT_ASSIGN:
		break;
	case DFU_PRE:
	case CRC_PRE:
	case READ_FLASH_PRE:
	case WRITE_FLASH_PRE:
	case INIT_PRE:
	case RESET_DRV_PRE:
	case RS_485_PRE:
	case FD_CAN_PRE:
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
		originIdx = 0;
		state = START;
		break;
	case READ_FLASH_POST:
		doReadFlash();
		tokenIdx = 0;
		originIdx = 0;
		state = START;
		break;
	case WRITE_FLASH_POST:
		doWriteFlash();
		tokenIdx = 0;
		originIdx = 0;
		state = START;
		break;
	case INIT_POST:
		doInit();
		tokenIdx = 0;
		originIdx = 0;
		state = START;
		break;
	case RESET_DRV_POST:
		doResetDrv();
		tokenIdx = 0;
		originIdx = 0;
		state = START;
		break;
	case RS_485_POST:
		doRs485Test();
		tokenIdx = 0;
		originIdx = 0;
		state = START;
		break;
	case FD_CAN_POST:
		doFdCanTest();
		tokenIdx = 0;
		originIdx = 0;
		state = START;
		break;
	case DEV:
		printf("Print entire 'dev' setting tree\n");
		tokenIdx = 0;
		originIdx = 0;
		state = START;
		break;
	case CONFIG:
		printf("Print 'dev.config' setting subtree\n");
		tokenIdx = 0;
		originIdx = 0;
		state = START;
		break;
	case ADDR:
		printf("Print 'dev.config.addr' value\n");
		tokenIdx = 0;
		originIdx = 0;
		state = START;
		break;
	case NEXT:
		printf("Print 'dev.config.next' value\n");
		tokenIdx = 0;
		originIdx = 0;
		state = START;
		break;
	}
}

void incompleteCommand() {
	printf("Expected ')' at end of input: [%.*s%s]\n", originIdx, origin,
			originIdx == sizeof(origin) ? "..." : "");
	tokenIdx = 0;
	originIdx = 0;
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

void doInit() {
	initDrv8320h();
	initTmc4671();
	resetDrv8320hFault();
}

void doResetDrv() {
	resetDrv8320hFault();
}

void invalidValue() {
	printf("Can't parse value to assign: [%.*s%s]\n", originIdx, origin,
			originIdx == sizeof(origin) ? "..." : "");
	tokenIdx = 0;
	originIdx = 0;
	state = START;
}
