/*
 * ON Semiconductor AR0220 sensor camera wizard 1820x940@44/RCCB/MIPI
 *
 * Copyright (C) 2017 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

//#define AR0220_DISPLAY_PATTERN_FIXED
//#define AR0220_DISPLAY_PATTERN_COLOR_BAR

#define AR0220_MAX_WIDTH	3648 // (1820*2=3640) <- must be multiple of 16 - requred by R-CAR VIN
#define AR0220_MAX_HEIGHT	944

#define AR0220_DELAY		0xffff

struct ar0220_reg {
	u16	reg;
	u16	val;
};

static const struct ar0220_reg ar0220_regs_wizard[] = {
{0x301A, 0x0018}, // RESET_REGISTER
{AR0220_DELAY, 500}, // Wait 500ms
{0x3070, 0x0000}, //  1: Solid color test pattern,
		  //  2: Full color bar test pattern,
		  //  3: Fade to grey color bar test pattern,
		  //256: Walking 1 test pattern (12 bit)
{0x3072, 0x0123}, // R
{0x3074, 0x0456}, // G(GR row)
{0x3076, 0x0abc}, // B
{0x3078, 0x0def}, // G(GB row)
#ifdef AR0220_DISPLAY_PATTERN_FIXED
{0x3070, 0x0001},
#endif
#ifdef AR0220_DISPLAY_PATTERN_COLOR_BAR
{0x3070, 0x0002},
#endif
{AR0220_DELAY, 100}, // Wait 100ms
};
