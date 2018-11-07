/*
 * ON Semiconductor gw4200-ar014x sensor camera wizard 1280x720@30/UYVY/BT601/8bit
 *
 * Copyright (C) 2018 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define GW4200_MAX_WIDTH	1280
#define GW4200_MAX_HEIGHT	960

#define GW4200_DELAY		0xffff

struct gw4200_reg {
	u16	reg;
	u16	val;
};

static const struct gw4200_reg gw4200_regs_wizard[] = {
/* enable FSIN */
{0xc88c, 0x0303},
{0xfc00, 0x2800},
{0x0040, 0x8100},
{GW4200_DELAY, 100},
};
