/*
 * ON Semiconductor AR0147 sensor camera wizard 1344x968@30/BGGR/BT601/RAW12
 *
 * Copyright (C) 2019 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

//#define AR0147_DISPLAY_PATTERN_FIXED
//#define AR0147_DISPLAY_PATTERN_COLOR_BAR

//#define AR0147_EMBEDDED_LINE

#define AR0147_MAX_WIDTH	1344
#define AR0147_MAX_HEIGHT	968

#define AR0147_DELAY		0xffff

#define AR0147_SENSOR_WIDTH	1344
#define AR0147_SENSOR_HEIGHT	968

#define AR0147_X_START		((AR0147_SENSOR_WIDTH - AR0147_MAX_WIDTH) / 2)
#define AR0147_Y_START		((AR0147_SENSOR_HEIGHT - AR0147_MAX_HEIGHT) / 2)
#define AR0147_X_END		(AR0147_X_START + AR0147_MAX_WIDTH - 1)
#define AR0147_Y_END		(AR0147_Y_START + AR0147_MAX_HEIGHT - 1)

struct ar0147_reg {
	u16	reg;
	u16	val;
};

#include "ar0147_rev1.h"
#include "ar0147_rev2.h"
