/*
 * tof.c
 *
 *  Created on: Sep 29, 2024
 *      Author: yjh
 */

#include "tof.h"

//static void TOFInit();

TOF tof =
{
	.config = {{ 0 }},
	.results = { 0 },
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
}

const TOFInstance tofInstance =
{
    TOFInit,
};
