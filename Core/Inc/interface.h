/*
 * interface.h
 *
 *  Created on: 20 Sep 2022
 *      Author: matveev
 */

#ifndef INC_INTERFACE_H_
#define INC_INTERFACE_H_

#include <stdint.h>

uint8_t handleCommand(uint8_t *data, uint16_t length, Interface interface);

#endif /* INC_INTERFACE_H_ */
