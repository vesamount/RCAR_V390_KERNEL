/*
 * ON Semiconductor AR0143 sensor camera wizard 1344x968@30/BGGR/BT601/RAW12
 *
 * Copyright (C) 2018 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

//#define AR0143_DISPLAY_PATTERN_FIXED
//#define AR0143_DISPLAY_PATTERN_COLOR_BAR

//#define AR0143_EMBEDDED_LINE

#define AR0143_MAX_WIDTH	1344
#define AR0143_MAX_HEIGHT	968

#define AR0143_DELAY		0xffff

#define AR0143_SENSOR_WIDTH	1344
#define AR0143_SENSOR_HEIGHT	968

#define AR0143_X_START		((AR0143_SENSOR_WIDTH - AR0143_MAX_WIDTH) / 2)
#define AR0143_Y_START		((AR0143_SENSOR_HEIGHT - AR0143_MAX_HEIGHT) / 2)
#define AR0143_X_END		(AR0143_X_START + AR0143_MAX_WIDTH - 1)
#define AR0143_Y_END		(AR0143_Y_START + AR0143_MAX_HEIGHT + 1) /* must be +1 and not -1 or 2 lines missed - bug in imager? */

struct ar0143_reg {
	u16	reg;
	u16	val;
};

#include "ar0143_custom.h"
#include "ar0143_rev1.h"
