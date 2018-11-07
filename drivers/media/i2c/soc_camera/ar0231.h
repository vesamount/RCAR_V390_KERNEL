/*
 * ON Semiconductor AR0231 sensor camera wizard 1928x1208@30/BGGR/BT601
 *
 * Copyright (C) 2018 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

//#define AR0231_DISPLAY_PATTERN_FIXED
//#define AR0231_DISPLAY_PATTERN_COLOR_BAR

#define AR0231_MAX_WIDTH	1928
#define AR0231_MAX_HEIGHT	1208

#define AR0231_DELAY		0xffff

#define AR0231_SENSOR_WIDTH	1928
#define AR0231_SENSOR_HEIGHT	1208

#define AR0231_X_START		((AR0231_SENSOR_WIDTH - AR0231_MAX_WIDTH) / 2)
#define AR0231_Y_START		((AR0231_SENSOR_HEIGHT - AR0231_MAX_HEIGHT) / 2)
#define AR0231_X_END		(AR0231_X_START + AR0231_MAX_WIDTH - 1)
#define AR0231_Y_END		(AR0231_Y_START + AR0231_MAX_HEIGHT - 1)

struct ar0231_reg {
	u16	reg;
	u16	val;
};

#include "ar0231_rev4.h"
//#include "ar0231_rev6.h"
