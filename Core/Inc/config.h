/*
 * config.h
 *
 *  Created on: 10 Sep 2022
 *      Author: matveev
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include <stdint.h>

typedef struct {
	uint8_t addr;
	uint8_t nextAddr;
	uint8_t unused[2];
} __attribute__((packed)) Config;

#define CTASTR2(pre,post) pre ## post
#define CTASTR(pre,post) CTASTR2(pre,post)
#define STATIC_ASSERT(cond,msg) \
    typedef struct { int CTASTR(static_assertion_failed_,msg) : !!(cond); } \
        CTASTR(static_assertion_failed_,__COUNTER__)

STATIC_ASSERT(!(sizeof(Config) % 4), config_size_must_be_multiple_of_4);

#endif /* INC_CONFIG_H_ */
