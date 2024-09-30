/*
 * tof.h
 *
 *  Created on: Sep 29, 2024
 *      Author: yjh
 */

#ifndef INC_TOF_H_
#define INC_TOF_H_

#include "main.h"

#include "vl53l8cx_api.h"
#include "platform.h"
#include "vl53l8cx_plugin_motion_indicator.h"
#include "vl53l8cx_plugin_detection_thresholds.h"
#include "vl53l8cx_plugin_xtalk.h"

#include "common.h"

#define RANGING_FREQUENCY	2
#define INTERGRATION_TIME	10

typedef struct _TOF
{
	VL53L8CX_Configuration config;
	VL53L8CX_Motion_Configuration motionConfig;
	VL53L8CX_ResultsData results;
	int status;
	u8 isAlive;
} TOF;

typedef void (*Init)();
typedef void (*GetDistance)();

typedef struct _TOFInstance
{
	Init Init;
	GetDistance GetDistance;
} TOFInstance;


extern const TOFInstance tofInstance;

#endif /* INC_TOF_H_ */
