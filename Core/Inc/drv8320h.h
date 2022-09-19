/*
 * drv8321h.h
 *
 *  Created on: 4 Sep 2022
 *      Author: matveev
 */

#ifndef SRC_DRV8320H_H_
#define SRC_DRV8320H_H_

#include <stdint.h>

typedef enum {
	PWM_6X,
	PWM_3X,
	PWM_1X,
	PWM_INDEPENDENT,
} Drv8320hMode;

/* Peak source current value. Sink current is 2 * source current. */
typedef enum {
	IDRV_10MA,
	IDRV_30MA,
	IDRV_60MA,
	IDRV_120MA,
	IDRV_260MA,
	IDRV_570MA,
	IDRV_1000MA,
} Drv8320hDriveCurrent;

typedef enum {
	VDS_60MV,
	VDS_130MV,
	VDS_260MV,
	VDS_600MV,
	VDS_1130MV,
	VDS_1880MV,
	VDS_DISABLED,
} Drv8320hVdsOvercurrent;

#define DefaultDrv8320hMode PWM_6X
#define DefaultDrv8320hDriveCurrent IDRV_10MA
#define DefaultDrv8320hVdsOvercurrent VDS_260MV

void initDrv8320h();
uint8_t hasDrv8320hFailure();
void setDrv8320hMode(Drv8320hMode mode);
void setDrv8320hIDrive(Drv8320hDriveCurrent current);
void setDrv8320hVdsOvercurrent(Drv8320hVdsOvercurrent tripVoltage);
void resetDrv8320hFault();

#endif /* SRC_DRV8320H_H_ */
