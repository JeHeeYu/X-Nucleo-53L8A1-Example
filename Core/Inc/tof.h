/*
 * tof.h
 *
 *  Created on: Sep 29, 2024
 *      Author: yjh
 */

#ifndef INC_TOF_H_
#define INC_TOF_H_

#include "main.h"
#include "math.h"

#include "vl53l8cx_api.h"
#include "platform.h"
#include "vl53l8cx_plugin_motion_indicator.h"
#include "vl53l8cx_plugin_detection_thresholds.h"
#include "vl53l8cx_plugin_xtalk.h"
#include "common.h"

#define RANGING_FREQUENCY 2
#define INTERGRATION_TIME 10
#define CALIBRATION_SAMPLING_COUNT 10
#define MOVING_AVERAGE_WINDOW 5
#define DISTANCE_THRESHOLD_PERCENT 20

typedef struct _PixcelDatas {
    u16 maxDistance;
    u16 minDistance;
    u16 averageDistance;
} PixcelDatas;

typedef struct _MovingAverage {
    i16 samples[MOVING_AVERAGE_WINDOW];
    u8 index;
    i32 sum;
} MovingAverage;

typedef struct _TOF {
    VL53L8CX_Configuration config;
    VL53L8CX_Motion_Configuration motionConfig;
    VL53L8CX_ResultsData results;
    int status;
    u8 isAlive;
    PixcelDatas pixcelData[64];
    MovingAverage movingAverage[64];
    i16 maxDistance[64];
    i16 minDistance[64];
} TOF;

typedef void (*Init)();
typedef void (*PollTOF)();

typedef struct _TOFInstance {
    Init Init;
    PollTOF PollTOF;
} TOFInstance;

extern const TOFInstance tofInstance;

#endif /* INC_TOF_H_ */
