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

	//check VL53L8CX sensor connected
	tof.status = vl53l8cx_is_alive(&tof.config, &tof.isAlive);

	if(!tof.isAlive || tof.status) {
		printf("TOF not detected at requested address\n\r");
		return;
	}

	// initialize VL53L8CX sensor
	tof.status = vl53l8cx_init(&tof.config);

	if(tof.status) {
		printf("VL53L8CX ULD loading failed status : %u\n\r", tof.status);
		return;
	}

	printf("VL53L8CX ULD loading success\n\r");

	// motion indicator with resolution 8x8
	tof.status = vl53l8cx_motion_indicator_init(&tof.config, &tof.motionConfig, VL53L8CX_RESOLUTION_8X8);

	if(tof.status) {
		printf("motion indicator failed status : %u\n\r", tof.status);
	}

	tof.status = vl53l8cx_set_resolution(&tof.config, VL53L8CX_RESOLUTION_8X8);
	tof.status = vl53l8cx_set_ranging_mode(&tof.config, VL53L8CX_RANGING_MODE_AUTONOMOUS);
	tof.status = vl53l8cx_set_ranging_frequency_hz(&tof.config, RANGING_FREQUENCY);
	tof.status = vl53l8cx_set_integration_time_ms(&tof.config, INTERGRATION_TIME);

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

                        printf("%4d mm ", distance[VL53L8CX_NB_TARGET_PER_ZONE * zoneIndex]);
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

static void TOFCalibration()
{
    u16 reflectancPercent = 50;
    u8 samples = 10;
    u16 distance = 600;

    printf("Starting TOF calibration...\n\r");

    tof.status = vl53l8cx_calibrate_xtalk(&tof.config, reflectancPercent, samples, distance);

    if (tof.status != VL53L8CX_STATUS_OK) {
        printf("TOF calibration failed with status : %u\n\r", tof.status);
    }
    else {
        printf("TOF calibration completed successfully\n\r");

        u8 xtalkData[VL53L8CX_XTALK_BUFFER_SIZE];

        tof.status = vl53l8cx_get_caldata_xtalk(&tof.config, xtalkData);

        if (tof.status != VL53L8CX_STATUS_OK) {
            printf("Failed to retrieve Xtalk calibration data : %u\n\r", tof.status);
        }
        else {
            printf("Xtalk calibration data retrieved successfully\n\r");
        }

        tof.status = vl53l8cx_set_caldata_xtalk(&tof.config, xtalkData);

        for (u8 j = 0; j < 64; j++) {
            tof.pixcelData[j].maxDistance = 0;
            tof.pixcelData[j].minDistance = 0xFFFF;
            tof.pixcelData[j].averageDistance = 0;
        }

        for (u8 i = 0; i < CALIBRATION_SAMPLING_COUNT; i++) {
            i16* distances = GetTOFDistance();

            for (u8 j = 0; j < VL53L8CX_RESOLUTION_8X8; j++) {
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

        for (u8 j = 0; j < VL53L8CX_RESOLUTION_8X8; j++) {
            tof.pixcelData[j].averageDistance /= CALIBRATION_SAMPLING_COUNT;
        }

        printf("Calibration results\n");
        for (u8 j = 0; j < 64; j++) {
            printf("Pixel %u - Max: %u mm, Min: %u mm, Avg: %u mm\n",
                   j, tof.pixcelData[j].maxDistance, tof.pixcelData[j].minDistance, tof.pixcelData[j].averageDistance);
        }
    }
}

const TOFInstance tofInstance =
{
    TOFInit,
	GetTOFDistance,
};
