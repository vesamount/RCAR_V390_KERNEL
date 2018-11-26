/*
 * ON Semiconductor isx016 sensor camera wizard 1280x800@30/UYVY/BT601/8bit
 *
 * Copyright (C) 2018 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define ISX016_MAX_WIDTH	1280
#define ISX016_MAX_HEIGHT	800

#define ISX016_DELAY		0xffff

struct isx016_reg {
	u16	reg;
	u16	val;
};

static const struct isx016_reg isx016_regs_wizard[] = {
#if 0
/* enable FSIN */
#endif
{ISX016_DELAY, 100},
};
