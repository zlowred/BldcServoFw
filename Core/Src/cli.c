/*
 * cli.c
 *
 *  Created on: Sep 21, 2022
 *      Author: matveev
 */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "cli.h"
#include "eeprom_emul.h"
#include "cmsis_os.h"
#include "acs711.h"
#include "tmc4671.h"
#include "drv8320h.h"
#include "microrl.h"

typedef struct PathEntry {
	const char *const name;
} PathEntry;

static const PathEntry PATH_DEV = { "/dev" };
static const PathEntry PATH_DEV_FOC = { "/dev/foc" };
static const PathEntry PATH_DEV_FOC_CONF = { "/dev/foc/conf" };
static const PathEntry PATH_DEV_FOC_STATE = { "/dev/foc/state" };
static const PathEntry PATH_DEV_DRV_SW = { "/dev/drv-sw" };
static const PathEntry PATH_DEV_DRV_SW_CONF = { "/dev/drv-sw/conf" };
static const PathEntry PATH_DEV_DRV_SW_STATE = { "/dev/drv-sw/state" };
static const PathEntry PATH_DEV_DRV_HW = { "/dev/drv-hw" };
static const PathEntry PATH_DEV_DRV_HW_CONF = { "/dev/drv-hw/conf" };
static const PathEntry PATH_DEV_DRV_HW_STATE = { "/dev/drv-hw/state" };
static const PathEntry PATH_DEV_SPI = { "/dev/spi" };
static const PathEntry PATH_DEV_SPI_CONF = { "/dev/spi/conf" };
static const PathEntry PATH_DEV_SPI_STATE = { "/dev/spi/state" };
static const PathEntry PATH_DEV_GPIO = { "/dev/gpio" };
static const PathEntry PATH_DEV_GPIO_CONF = { "/dev/gpio/conf" };
static const PathEntry PATH_DEV_GPIO_STATE = { "/dev/gpio/state" };

static const PathEntry *const PATHS[] = { &PATH_DEV, &PATH_DEV_FOC,
		&PATH_DEV_FOC_CONF, &PATH_DEV_FOC_STATE, &PATH_DEV_DRV_SW,
		&PATH_DEV_DRV_SW_CONF, &PATH_DEV_DRV_SW_STATE, &PATH_DEV_DRV_HW,
		&PATH_DEV_DRV_HW_CONF, &PATH_DEV_DRV_HW_STATE, &PATH_DEV_SPI,
		&PATH_DEV_SPI_CONF, &PATH_DEV_SPI_STATE, &PATH_DEV_GPIO,
		&PATH_DEV_GPIO_CONF, &PATH_DEV_GPIO_STATE };

#define CMD_DFU 	"dfu"
#define CMD_TEST 	"test"
#define CMD_CD 		"cd"

const char *const commands[] = { CMD_CD, CMD_DFU, CMD_TEST };

#define SUBCMD_TEST_CRC 	"crc"
#define SUBCMD_TEST_FLASH	"flash"

const char *const testSubcommands[] = { SUBCMD_TEST_CRC, SUBCMD_TEST_FLASH };

#define SUBCMD_TEST_FLASH_READ	"read"
#define SUBCMD_TEST_FLASH_WRITE	"write"

const char *const testFlashSubcommands[] = { SUBCMD_TEST_FLASH_READ,
SUBCMD_TEST_FLASH_WRITE };

char *completions[16];

extern volatile uint32_t isErasing;

void doCrcTest();
void doReadFlash();
void doWriteFlash();
void doInit();
void doResetDrv();

microrl_t rl;
microrl_t *prl = &rl;
const PathEntry *current = NULL;

const PathEntry* find(const char *path) {
	for (int i = 0; i < sizeof(PATHS) / sizeof(PathEntry*); i++) {
		if (strcmp(PATHS[i]->name, path) == 0) {
			return PATHS[i];
		}
	}
	return NULL;
}

int level(const char *path) {
	int i = 0;
	while (*path != 0) {
		if (*path == '/') {
			i++;
		}
		path++;
	}
	return i;
}

static char fullPath[64];
const char* getFullPath(const char *path) {
	if (path[0] == '/') {
		strcpy(fullPath, path);
	} else {
		sprintf(fullPath, "%s/%s", current == NULL ? "" : current->name, path);
	}
	return fullPath;
}

int addCompletion(int index, const char *value) {
	if (index == sizeof(completions) / sizeof(char*) - 3) {
		return index;
	}
	completions[index] = (char*) value;
	return index + 1;
}

void printCli(const char *str) {
	printf(str);
}

void initCli() {
	if (PATHS[0] != NULL) {
		microrl_init(prl, printCli);
		microrl_set_execute_callback(prl, executeCli);
		microrl_set_complete_callback(prl, completeCli);
		microrl_set_prompt_callback(prl, promptCli);
	}
}

int executeCli(int argc, const char *const*argv) {
	if (argc > 0) {
		const char *cmd = argv[0];
		if (strcmp(cmd, CMD_DFU) == 0) {
			if (argc > 1) {
				printCli("Unexpected 'dfu' argument\r\n");
				return 1;
			}
			printCli("Entering DFU mode\n");
			startDfu();
			return 0;
		} else if (strcmp(cmd, CMD_TEST) == 0) {
			if (argc < 2) {
				printCli("Missing 'test' argument\r\n");
				return 1;
			}
			const char *subCmd = argv[1];
			if (strcmp(subCmd, SUBCMD_TEST_CRC) == 0) {
				if (argc > 2) {
					printCli("Unexpected 'test crc' argument\r\n");
					return 1;
				}
				doCrcTest();
				return 0;
			} else if (strcmp(subCmd, SUBCMD_TEST_FLASH) == 0) {
				if (argc < 3) {
					printCli("Missing 'test flash' argument\r\n");
					return 1;
				}
				const char *flashCmd = argv[2];
				if (strcmp(flashCmd, SUBCMD_TEST_FLASH_READ) == 0) {
					if (argc > 3) {
						printCli("Unexpected 'test flash read' argument\r\n");
						return 1;
					}
					doReadFlash();
					return 0;
				} else if (strcmp(flashCmd, SUBCMD_TEST_FLASH_WRITE) == 0) {
					if (argc > 3) {
						printCli("Unexpected 'test flash write' argument\r\n");
						return 1;
					}
					doWriteFlash();
					return 0;
				} else {
					printCli("Bad 'test flash' argument\r\n");
					return 1;
				}
			} else {
				printCli("Bad 'test' argument\r\n");
				return 1;
			}
		} else if (strcmp(cmd, CMD_CD) == 0) {
			if (argc < 2) {
				printCli("Missing 'cd' argument\r\n");
				return 1;
			}
			if (argc > 2) {
				printCli("Unexpected extra 'cd' argument\r\n");
				return 1;
			}
			const char *subCmd = argv[1];
			if (strcmp(subCmd, "/") == 0) {
				current = NULL;
				return 0;
			}
			const char *fullPath = getFullPath(subCmd);
			const PathEntry *e = find(fullPath);
			if (!e) {
				printCli("Path not found\r\n");
				return 1;
			}
			current = e;
			return 0;
		} else {
			printCli("Bad command\r\n");
			return 1;
		}
	}
	return 0;
}

char** completeCli(int argc, const char *const*argv) {
	completions[0] = NULL;
	int completionIdx = 0;

	if (argc == 0) {
		for (;
				completionIdx < sizeof(commands) / sizeof(char*)
						&& completionIdx
								< sizeof(completions) / sizeof(char*) - 3;
				completionIdx++) {
			addCompletion(completionIdx, commands[completionIdx]);
		}
	} else if (argc == 1) {
		const char *cmd = argv[0];
		for (int i = 0; i < sizeof(commands) / sizeof(char*); i++) {
			if (strstr(commands[i], cmd) == commands[i]) {
				completionIdx = addCompletion(completionIdx, commands[i]);
			}
		}
	} else if (argc >= 2) {
		const char *cmd = argv[0];
		const char *subCmd = argv[1];

		if (strcmp(cmd, CMD_TEST) == 0) {
			if (argc == 2) {
				for (int i = 0; i < sizeof(testSubcommands) / sizeof(char*);
						i++) {
					if (strstr(testSubcommands[i], subCmd)
							== testSubcommands[i]) {
						completionIdx = addCompletion(completionIdx,
								testSubcommands[i]);
					}

				}
			} else if (strcmp(subCmd, SUBCMD_TEST_FLASH) == 0 && argc == 3) {
				const char *testFlashSubCmd = argv[2];
				for (int i = 0;
						i < sizeof(testFlashSubcommands) / sizeof(char*); i++) {
					if (strstr(testFlashSubcommands[i], testFlashSubCmd)
							== testFlashSubcommands[i]) {
						completionIdx = addCompletion(completionIdx,
								testFlashSubcommands[i]);
					}
				}
			}
		} else if (strcmp(cmd, CMD_CD) == 0 && argc == 2) {
			const char *fullPath = getFullPath(subCmd);
			int prefixLen;
			if (subCmd[0] == '/') {
				prefixLen = 0;
			} else {
				prefixLen = (current == NULL) ? 0 : (strlen(current->name) + 1);
			}

			int currentLevel = level(fullPath);
			int matchCount = 0;
			bool fullMatch = false;

			for (int i = 0; i < sizeof(PATHS) / sizeof(PathEntry*); i++) {
				if (strcmp(PATHS[i]->name, fullPath) == 0) {
					fullMatch = true;
				}
				if (strstr(PATHS[i]->name, fullPath) == PATHS[i]->name) {
					if (level(PATHS[i]->name) == currentLevel
							|| (fullMatch
									&& level(PATHS[i]->name) == currentLevel + 1)) {
						completionIdx = addCompletion(completionIdx,
								&PATHS[i]->name[prefixLen]);
					}
					matchCount++;
				}
			}
			if (matchCount > 1) {
				completionIdx = addCompletion(completionIdx, completions[0]);
			}

		}
	}

	completions[completionIdx] = NULL;
	return (char**) completions;
}

char* promptCli() {
	return current == NULL ? "/" : (char*) current->name;
}

void processCli(char c) {
	microrl_insert_char(prl, c);
	fflush(stdout);
}

void doCrcTest() {
	printf("CRC test\r\n");
	uint32_t crcdata[] = { 0x12345678, 0x87654321 };
	uint32_t crc = calculateCrc((uint8_t*) crcdata, 2);
	printf("crc1: %08lx\r\n", crc);
	crc = calculateCrc((uint8_t*) crcdata, 8);
	printf("crc2: %08lx\r\n", crc);
	crcdata[1] = 0x12345678;
	crcdata[0] = 0x87654321;
	crc = calculateCrc((uint8_t*) crcdata, 2);
	printf("crc3: %08lx\r\n", crc);
	// https://crccalc.com/?crc=7856&method=crc32&datatype=hex&outtype=0
	uint8_t crcdata2[] = { 0x78, 0x56 };
	crc = calculateCrc(crcdata2, 2);
	printf("crc4: %08lx\r\n", crc);
}

void doReadFlash() {
	uint32_t flashData;
	EE_ReadVariable32bits(/* idx = */1, &flashData);
	printf("Read: 0x%08lX\r\n", flashData);
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
	printf("Written: 0x%08lX\r\n", flashData);
}

void doInit() {
	initDrv8320h();
	initTmc4671();
	resetDrv8320hFault();
}

void doResetDrv() {
	resetDrv8320hFault();
}
