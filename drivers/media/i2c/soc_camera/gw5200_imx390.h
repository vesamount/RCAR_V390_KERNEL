/*
 * GEO Semiconductor GW5200-IMX390 sensor camera wizard 1920x1080@30/UYVY/BT601/8bit
 *
 * Copyright (C) 2019 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define GW5200_MAX_WIDTH	1920
#define GW5200_MAX_HEIGHT	1080

#define GW5200_DELAY		0xffff

struct gw5200_reg {
	u16	reg;
	u16	val;
};

static const struct gw5200_reg gw5200_regs_wizard[] = {
/* enable FSIN */
{GW5200_DELAY, 100},
};
