/*
 * interface.c
 *
 *  Created on: 20 Sep 2022
 *      Author: matveev
 */


#include <string.h>
#include <stdio.h>

#include "main.h"
#include "interface.h"

uint8_t handleCommand(uint8_t *data, uint16_t length, Interface interface) {
	static UsbTxData usbData;

	if (length >= 64 && data[length - 64] == '$') {
		uint32_t actualCrc = calculateCrc(&data[length - 64], 60);
		uint32_t expectedCrc = *((uint32_t *)&data[length - 4]);
		sprintf((char *)usbData.data, "E: %08lx A: %08lx\n", expectedCrc, actualCrc);
		usbData.size = 24;
		transmitUsb(&usbData);
		if (actualCrc == expectedCrc) {
			return 1;
		}
	}

	return 0;
}
