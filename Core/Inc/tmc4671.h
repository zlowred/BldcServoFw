/*
 * tmc4671.h
 *
 *  Created on: 4 Sep 2022
 *      Author: matveev
 */

#ifndef SRC_TMC4671_H_
#define SRC_TMC4671_H_

#include <stdint.h>

typedef enum {
	TMC4671_DISABLED,
	TMC4671_ENABLED,
} Tmc4671State;

void setTmc4671State(Tmc4671State state);
Tmc4671State getTmc4671State();

void initTmc4671();

uint8_t hasTmc4671Failure();

#endif /* SRC_TMC4671_H_ */
