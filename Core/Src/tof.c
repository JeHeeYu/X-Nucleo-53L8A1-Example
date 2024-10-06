/*
 * tof.c
 *
 *  Created on: Sep 29, 2024
 *      Author: yjh
 */

#include "tof.h"

static void TOFInit();
static i16* GetTOFDistance();
static void TOFCalibration();
static void PollTOFProcess();

TOF tof =
{
    .config = {{ 0 }},
    .motionConfig = { 0 },
    .results = { 0 },
    .status = 0,
    .isAlive = 0,
};

static void TOFInit()
{
    tof.config.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;

    VL53L8CX_Reset_Sensor(&(tof.config.platform));

    tof.status = vl53l8cx_is_alive(&tof.config, &tof.isAlive);

    if(!tof.isAlive || tof.status) {
        printf("TOF not detected at requested address\n\r");
        return;
    }

    tof.status = vl53l8cx_init(&tof.config);

    if(tof.status) {
        printf("VL53L8CX ULD loading failed status : %u\n\r", tof.status);
        return;
    }

    printf("VL53L8CX ULD loading success\n\r");

    tof.status = vl53l8cx_motion_indicator_init(&tof.config, &tof.motionConfig, VL53L8CX_RESOLUTION_8X8);

    if(tof.status) {
        printf("motion indicator failed status : %u\n\r", tof.status);
    }

    tof.status = vl53l8cx_set_resolution(&tof.config, VL53L8CX_RESOLUTION_8X8);
    tof.status = vl53l8cx_set_ranging_mode(&tof.config, VL53L8CX_RANGING_MODE_AUTONOMOUS);
    tof.status = vl53l8cx_set_ranging_frequency_hz(&tof.config, RANGING_FREQUENCY);
    tof.status = vl53l8cx_set_integration_time_ms(&tof.config, INTERGRATION_TIME);

    for (u8 i = 0; i < 64; i++) {
        tof.movingAverage[i].index = 0;
        tof.movingAverage[i].sum = 0;
        for (u8 j = 0; j < MOVING_AVERAGE_WINDOW; j++) {
            tof.movingAverage[i].samples[j] = 0;
        }
        tof.maxDistance[i] = 0;
        tof.minDistance[i] = 0xFFFF;
    }

    TOFCalibration();
}

i16* GetTOFDistance()
{
    static i16 distance[VL53L8CX_RESOLUTION_8X8 * VL53L8CX_NB_TARGET_PER_ZONE];

    tof.status = vl53l8cx_start_ranging(&tof.config);

    static u8 isReady = 0;

    while (1) {
        tof.status = vl53l8cx_check_data_ready(&tof.config, &isReady);

        if (isReady) {
            tof.status = vl53l8cx_get_ranging_data(&tof.config, &tof.results);

            if (tof.status == VL53L8CX_STATUS_OK) {
                printf("==== 8x8 Ranging Data ====\n\r");

                for (u8 row = 0; row < 8; row++) {
                    for (u8 col = 0; col < 8; col++) {
                        int zoneIndex = row * 8 + col;
                        distance[VL53L8CX_NB_TARGET_PER_ZONE * zoneIndex] = tof.results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * zoneIndex];

                        u8 currentIndex = tof.movingAverage[zoneIndex].index;
                        tof.movingAverage[zoneIndex].sum -= tof.movingAverage[zoneIndex].samples[currentIndex];
                        tof.movingAverage[zoneIndex].samples[currentIndex] = distance[VL53L8CX_NB_TARGET_PER_ZONE * zoneIndex];
                        tof.movingAverage[zoneIndex].sum += tof.movingAverage[zoneIndex].samples[currentIndex];
                        tof.movingAverage[zoneIndex].index = (currentIndex + 1) % MOVING_AVERAGE_WINDOW;

                        i16 avgDistance = tof.movingAverage[zoneIndex].sum / MOVING_AVERAGE_WINDOW;

                        if (avgDistance > tof.maxDistance[zoneIndex]) {
                            tof.maxDistance[zoneIndex] = avgDistance;
                        }
                        if (avgDistance < tof.minDistance[zoneIndex]) {
                            tof.minDistance[zoneIndex] = avgDistance;
                        }

                        printf("%4d mm (Avg: %4d mm) ", distance[VL53L8CX_NB_TARGET_PER_ZONE * zoneIndex], avgDistance);
                    }
                    printf("\n\r");
                }

                printf("==========================\n\r");
                break;
            }
            else {
                printf("Error retrieving ranging data: %u\n\r", tof.status);
            }
        }

        VL53L8CX_WaitMs(&(tof.config.platform), 5);
    }

    tof.status = vl53l8cx_stop_ranging(&tof.config);

    return distance;
}

static void PollTOFProcess()
{
    GetTOFDistance();

    for (u8 j = 0; j < 64; j++) {
        i16 avgDistance = tof.movingAverage[j].sum / MOVING_AVERAGE_WINDOW;
        i16 maxDist = tof.maxDistance[j];
        i16 minDist = tof.minDistance[j];
        i16 threshold = (maxDist - minDist) * DISTANCE_THRESHOLD_PERCENT / 100;

        if (avgDistance < (tof.pixcelData[j].averageDistance - threshold) ||
            avgDistance > (tof.pixcelData[j].averageDistance + threshold)) {
            printf("Pixel %u: Noise detected, skipping value\n", j);
            continue;
        }

        if (avgDistance >= maxDist - threshold) {
            printf("Pixel %u: No  detected (Distance: %d mm)\n", j, avgDistance);
        }
        else if (avgDistance < minDist + threshold) {
            printf("Pixel %u: detected (Distance: %d mm)\n", j, avgDistance);
        }
        else {
            printf("Pixel %u: detected (Distance: %d mm)\n", j, avgDistance);
        }
    }
}

static void TOFCalibration()
{
    printf("Starting TOF calibration...\n\r");

    for (u8 j = 0; j < 64; j++) {
        tof.pixcelData[j].maxDistance = 0;
        tof.pixcelData[j].minDistance = 0xFFFF;
        tof.pixcelData[j].averageDistance = 0;
    }

    for (u8 i = 0; i < CALIBRATION_SAMPLING_COUNT; i++) {
        i16* distances = GetTOFDistance();

        for (u8 j = 0; j < 64; j++) {
            u16 currentDistance = distances[j];

            if (currentDistance > tof.pixcelData[j].maxDistance) {
                tof.pixcelData[j].maxDistance = currentDistance;
            }

            if (currentDistance < tof.pixcelData[j].minDistance) {
                tof.pixcelData[j].minDistance = currentDistance;
            }

            tof.pixcelData[j].averageDistance += currentDistance;
        }
    }

    for (u8 j = 0; j < 64; j++) {
        tof.pixcelData[j].averageDistance /= CALIBRATION_SAMPLING_COUNT;
    }

    printf("Calibration results\n");
    for (u8 j = 0; j < 64; j++) {
        printf("Pixel %u - Max: %u mm, Min: %u mm, Avg: %u mm\n",
               j, tof.pixcelData[j].maxDistance, tof.pixcelData[j].minDistance, tof.pixcelData[j].averageDistance);
    }
}

const TOFInstance tofInstance =
{
    TOFInit,
    PollTOFProcess,
};
