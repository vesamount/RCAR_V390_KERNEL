/*
 * MAXIM max9288 GMSL driver
 *
 * Copyright (C) 2019 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/videodev2.h>

#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "max9286.h"

#define MAXIM_I2C_I2C_SPEED_837KHZ	(0x7 << 2) /* 837kbps */
#define MAXIM_I2C_I2C_SPEED_533KHZ	(0x6 << 2) /* 533kbps */
#define MAXIM_I2C_I2C_SPEED_339KHZ	(0x5 << 2) /* 339 kbps */
#define MAXIM_I2C_I2C_SPEED_173KHZ	(0x4 << 2) /* 174kbps */
#define MAXIM_I2C_I2C_SPEED_105KHZ	(0x3 << 2) /* 105 kbps */
#define MAXIM_I2C_I2C_SPEED_085KHZ	(0x2 << 2) /* 84.7 kbps */
#define MAXIM_I2C_I2C_SPEED_028KHZ	(0x1 << 2) /* 28.3 kbps */
#define MAXIM_I2C_I2C_SPEED		MAXIM_I2C_I2C_SPEED_339KHZ

struct max9288_priv {
	struct v4l2_subdev	sd;
	struct fwnode_handle	*sd_fwnode;
	int			des_addr;
	int			lanes;
	int			csi_rate;
	int			pclk;
	char			pclk_rising_edge;
	int			gpio_resetb;
	int			active_low_resetb;
	int			him;
	int			hsync;
	int			vsync;
	int			timeout;
	int			poc_delay;
	int			bws;
	int			dbl;
	int			dt;
	int			hsgen;
	int			hts;
	int			vts;
	int			hts_delay;
	struct i2c_client	*client;
	int			max9271_addr;
	int			ser_id;
	struct gpio_desc	*poc_gpio; /* PoC power supply */
};

static int conf_link;
module_param(conf_link, int, 0644);
MODULE_PARM_DESC(conf_link, " Force configuration link. Used only if robust firmware flashing required (f.e. recovery)");

static int poc_trig;
module_param(poc_trig, int, 0644);
MODULE_PARM_DESC(poc_trig, " Use PoC triggering during reverse channel setup. Useful on systems with dedicated PoC and unstable ser-des lock");

static int him = 0;
module_param(him, int, 0644);
MODULE_PARM_DESC(him, " Use High-Immunity mode (default: leagacy mode)");

static int hsync;
module_param(hsync, int, 0644);
MODULE_PARM_DESC(hsync, " HSYNC invertion (default: 0 - not inverted)");

static int vsync = 1;
module_param(vsync, int, 0644);
MODULE_PARM_DESC(vsync, " VSYNC invertion (default: 1 - inverted)");

static int gpio_resetb;
module_param(gpio_resetb, int, 0644);
MODULE_PARM_DESC(gpio_resetb, " Serializer GPIO reset (default: 0 - not used)");

static int active_low_resetb;
module_param(active_low_resetb, int, 0644);
MODULE_PARM_DESC(active_low_resetb, " Serializer GPIO reset level (default: 0 - active high)");

static int timeout_n = 100;
module_param(timeout_n, int, 0644);
MODULE_PARM_DESC(timeout_n, " Timeout of link detection (default: 100 retries)");

static int poc_delay = 50;
module_param(poc_delay, int, 0644);
MODULE_PARM_DESC(poc_delay, " Delay in ms after POC enable (default: 50 ms)");

static int bws = 0;
module_param(bws, int, 0644);
MODULE_PARM_DESC(bws, " BWS mode (default: 0 - 24-bit gmsl packets)");

static int dbl = 1;
module_param(dbl, int, 0644);
MODULE_PARM_DESC(dbl, " DBL mode (default: 1 - DBL mode enabled)");

static int dt = 3;
module_param(dt, int, 0644);
MODULE_PARM_DESC(dt, " DataType (default: 3 - YUV8), 0 - RGB888, 5 - RAW8, 6 - RAW10, 7 - RAW12, 8 - RAW14");

static int hsgen;
module_param(hsgen, int, 0644);
MODULE_PARM_DESC(hsgen, " Enable HS embedded generator (default: 0 - disabled)");

static int pclk = 100;
module_param(pclk, int, 0644);
MODULE_PARM_DESC(pclk, " PCLK rate (default: 100MHz)");

enum {
	RGB888_DT = 0,
	RGB565_DT,
	RGB666_DT,
	YUV8_DT, /* default */
	YUV10_DT,
	RAW8_DT,
	RAW10_DT,
	RAW12_DT,
	RAW14_DT,
};

static int dt2bpp [9] = {
	24,	/* RGB888 */
	16,	/* RGB565 */
	18,	/* RGB666 */
	8,	/* YUV8 - default */
	10,	/* YUV10 */
	8,	/* RAW8/RAW16 */
	10,	/* RAW10 */
	12,	/* RAW12 */
	14,	/* RAW14 */
};

static char* ser_name(int id)
{
	switch (id) {
	case MAX9271_ID:
		return "MAX9271";
	case MAX96705_ID:
		return "MAX96705";
	case MAX96707_ID:
		return "MAX96707";
	default:
		return "unknown";
	}
}

static void max9288_preinit(struct i2c_client *client, int addr)
{

	struct max9288_priv *priv = i2c_get_clientdata(client);

	client->addr = addr;			/* MAX9288-CAMx I2C */
	reg8_write(client, 0x04, 0x00);		/* disable reverse control */
	reg8_write(client, 0x16, (priv->him ? 0x80 : 0x00) |
				 0x5a);		/* high-immunity/legacy mode */
}

static void max9288_sensor_reset(struct i2c_client *client, int addr, int reset_on)
{
	struct max9288_priv *priv = i2c_get_clientdata(client);

	if (priv->gpio_resetb < 1 || priv->gpio_resetb > 5)
		return;

	/* sensor reset/unreset */
	client->addr = addr;					/* MAX9271-CAMx I2C */
	reg8_write(client, 0x0f, (0xfe & ~BIT(priv->gpio_resetb)) | /* set GPIOn value to reset/unreset */
		   ((priv->active_low_resetb ? BIT(priv->gpio_resetb) : 0) ^ reset_on));
	reg8_write(client, 0x0e, 0x42 | BIT(priv->gpio_resetb)); /* set GPIOn direction output */
}

static int max9288_reverse_channel_setup(struct i2c_client *client)
{
	struct max9288_priv *priv = i2c_get_clientdata(client);
	u8 val = 0, lock_sts = 0;
	int timeout = priv->timeout;
	char timeout_str[40];
	int ret = 0;

	/* Reverse channel enable */
	client->addr = priv->des_addr;				/* MAX9288-CAMx I2C */
	reg8_write(client, 0x1c, 0xa2 | MAXIM_I2C_I2C_SPEED);	/* enable artificial ACKs, I2C speed set */
	usleep_range(2000, 2500);				/* wait 2ms after any change of reverse channel settings */
	reg8_write(client, 0x04, 0x03);				/* enable reverse control */
	usleep_range(2000, 2500);				/* wait 2ms after any change of reverse channel settings */

	for (;;) {
		if (priv->him) {
			/* HIM mode setup */
			client->addr = 0x40;			/* MAX9271-CAMx I2C */
			reg8_write(client, 0x4d, 0xc0);
			usleep_range(2000, 2500);		/* wait 2ms after any change of reverse channel settings */
			reg8_write(client, 0x04, 0x43);		/* wake-up, enable reverse_control/conf_link */
			usleep_range(2000, 2500);		/* wait 2ms after any change of reverse channel settings */
			if (priv->bws) {
				reg8_write(client, 0x07, (priv->pclk_rising_edge ? 0 : 0x10) |
							 (priv->dbl ? 0x80 : 0) |
							 (priv->bws ? 0x20 : 0)); /* RAW/YUV, PCLK edge, HS/VS disabled enabled, DBL mode, BWS 24/32-bit */
				usleep_range(2000, 2500);	/* wait 2ms after any change of reverse channel settings */
			}
		} else {
			/* Legacy mode setup */
			client->addr = priv->des_addr;		/* MAX9288-CAMx I2C */
			reg8_write(client, 0x13, 0x00);
			reg8_write(client, 0x11, 0x42);		/* enable custom reverse channel & first pulse length */
			reg8_write(client, 0x0a, 0x0f);		/* first pulse length rise time changed from 300ns to 200ns, amplitude 100mV */
			usleep_range(2000, 2500);		/* wait 2ms after any change of reverse channel settings */

			client->addr = 0x40;			/* MAX9271-CAMx I2C */
			reg8_write(client, 0x04, 0x43);		/* wake-up, enable reverse_control/conf_link */
			usleep_range(2000, 2500);		/* wait 2ms after any change of reverse channel settings */
			reg8_write(client, 0x08, 0x01);		/* reverse channel receiver high threshold enable */
			usleep_range(2000, 2500);		/* wait 2ms after any change of reverse channel settings */
			if (priv->bws) {
				reg8_write(client, 0x07, (priv->pclk_rising_edge ? 0 : 0x10) |
							 (priv->dbl ? 0x80 : 0) |
							 (priv->bws ? 0x20 : 0)); /* RAW/YUV, PCLK edge, HS/VS encoding disabled, DBL mode, BWS 24/32-bit */
				usleep_range(2000, 2500);	/* wait 2ms after any change of reverse channel settings */
			}
			reg8_write(client, 0x97, 0x5f);		/* enable reverse control channel programming (MAX96705-MAX96711 only) */
			usleep_range(2000, 2500);		/* wait 2ms after any change of reverse channel settings */

			client->addr = priv->des_addr;		/* MAX9288-CAMx I2C */
			reg8_write(client, 0x0a, 0x0c);		/* first pulse length rise time changed from 300ns to 200ns, amplitude 100mV */
			reg8_write(client, 0x13, 0x20);		/* reverse channel increase amplitude 170mV to compensate high threshold enabled */
			usleep_range(2000, 2500);		/* wait 2ms after any change of reverse channel settings */
		}

		client->addr = 0x40;				/* MAX9271-CAMx I2C */
		reg8_read(client, 0x1e, &val);			/* read max9271 ID */
		if (val == MAX9271_ID || val == MAX96705_ID || val == MAX96707_ID || --timeout == 0) {
			priv->ser_id = val;
			break;
		}

		/* Check if already initialized (after reboot/reset ?) */
		client->addr = priv->max9271_addr;		/* MAX9271-CAMx I2C */
		reg8_read(client, 0x1e, &val);			/* read max9271 ID */
		if (val == MAX9271_ID || val == MAX96705_ID || val == MAX96707_ID) {
			priv->ser_id = val;
			reg8_write(client, 0x04, 0x43);		/* enable reverse_control/conf_link */
			usleep_range(2000, 2500);		/* wait 2ms after any change of reverse channel settings */
			ret = -EADDRINUSE;
			break;
		}

		if (poc_trig) {
			if (!IS_ERR(priv->poc_gpio) && (timeout % poc_trig == 0)) {
				gpiod_direction_output(priv->poc_gpio, 0); /* POC power off */
				mdelay(200);
				gpiod_direction_output(priv->poc_gpio, 1); /* POC power on */
				mdelay(priv->poc_delay);
			}
		}
	}

	max9288_sensor_reset(client, client->addr, 1);	/* sensor reset */

	client->addr = priv->des_addr;			/* MAX9288-CAMx I2C */
	reg8_read(client, 0x04, &lock_sts);		/* LOCK status */

	if (!timeout) {
		ret = -ETIMEDOUT;
		goto out;
	}

out:
	sprintf(timeout_str, "retries=%d lock_sts=%d", priv->timeout - timeout, !!(lock_sts & 0x80));
	dev_info(&client->dev, "link %s %sat 0x%x %s %s\n", ser_name(priv->ser_id),
			       ret == -EADDRINUSE ? "already " : "", priv->max9271_addr,
			       ret == -ETIMEDOUT ? "not found: timeout GMSL link establish" : "",
			       priv->timeout - timeout ? timeout_str : "");

	return ret;
}

static void max9288_initial_setup(struct i2c_client *client)
{
	struct max9288_priv *priv = i2c_get_clientdata(client);

	/* Initial setup */
	client->addr = priv->des_addr;				/* MAX9288-CAMx I2C */
	reg8_write(client, 0x09, 0x40);				/* Automatic pixel count enable */
	reg8_write(client, 0x15, 0x70);				/* Enable HV and DE tracking by register 0x69 */
	reg8_write(client, 0x60, (priv->dbl ? 0x20 : 0) |
				 (priv->dt & 0xf));		/* VC=0, DBL mode, DataType */
	reg8_write(client, 0x65, 0x47 | ((priv->lanes - 1) << 4)); /* setup CSI lanes, DE input is HS */

	reg8_write(client, 0x08, 0x20);				/* use D18/19 for HS/VS */
	reg8_write(client, 0x14, (priv->vsync ? 0x80 : 0) | (priv->hsync ? 0x40 : 0)); /* setup HS/VS inversion */
	reg8_write(client, 0x64, 0x0c);				/* Drive HSTRAIL state for 120ns after the last payload bit */
}

static void max9288_gmsl_link_setup(struct i2c_client *client)
{
	struct max9288_priv *priv = i2c_get_clientdata(client);

	/* GMSL setup */
	client->addr = 0x40;					/* MAX9271-CAMx I2C */
	reg8_write(client, 0x0d, 0x22 | MAXIM_I2C_I2C_SPEED);	/* disable artificial ACK, I2C speed set */
	reg8_write(client, 0x07, (priv->pclk_rising_edge ? 0 : 0x10) |
				 (priv->dbl ? 0x80 : 0) |
				 (priv->bws ? 0x20 : 0));	/* RAW/YUV, PCLK edge, HS/VS encoding disabled, DBL mode, BWS 24/32-bit */
	usleep_range(2000, 2500);				/* wait 2ms */
	reg8_write(client, 0x02, 0xff);				/* spread spectrum +-4%, pclk range automatic, Gbps automatic  */
	usleep_range(2000, 2500);				/* wait 2ms */

	if (priv->ser_id == MAX96705_ID || priv->ser_id == MAX96707_ID) {
		switch (priv->dt) {
		case YUV8_DT:
			/* setup crossbar for YUV8/RAW8: reverse DVP bus */
			reg8_write(client, 0x20, 3);
			reg8_write(client, 0x21, 4);
			reg8_write(client, 0x22, 5);
			reg8_write(client, 0x23, 6);
			reg8_write(client, 0x24, 7);
			reg8_write(client, 0x25, 0x40);
			reg8_write(client, 0x26, 0x40);
			if (priv->ser_id == MAX96705_ID) {
				reg8_write(client, 0x27, 14); /* HS: D14->D18  */
				reg8_write(client, 0x28, 15); /* VS: D15->D19 */
			}
			if (priv->ser_id == MAX96707_ID) {
				reg8_write(client, 0x27, 14); /* HS: D14->D18, this is a virtual NC pin, hence it is D14 at HS */
				reg8_write(client, 0x28, 13); /* VS: D13->D19 */
			}
			reg8_write(client, 0x29, 0x40);
			reg8_write(client, 0x2A, 0x40);

			/* this is second byte if DBL=1 */
			reg8_write(client, 0x30, 0x10 + 0);
			reg8_write(client, 0x31, 0x10 + 1);
			reg8_write(client, 0x32, 0x10 + 2);
			reg8_write(client, 0x33, 0x10 + 3);
			reg8_write(client, 0x34, 0x10 + 4);
			reg8_write(client, 0x35, 0x10 + 5);
			reg8_write(client, 0x36, 0x10 + 6);
			reg8_write(client, 0x37, 0x10 + 7);
			reg8_write(client, 0x38, 0);
			reg8_write(client, 0x39, 1);
			reg8_write(client, 0x3A, 2);

			reg8_write(client, 0x67, 0xC4); /* DBL_ALIGN_TO = 100b */

			break;
		case RAW12_DT:
#if 0	/* Not supported yet */
			/* setup crossbar for RAW12: reverse DVP bus */
			reg8_write(client, 0x20, 11);
			reg8_write(client, 0x21, 10);
			reg8_write(client, 0x22, 9);
			reg8_write(client, 0x23, 8);
			reg8_write(client, 0x24, 7);
			reg8_write(client, 0x25, 6);
			reg8_write(client, 0x26, 5);
			reg8_write(client, 0x27, 4);
			reg8_write(client, 0x28, 3);
			reg8_write(client, 0x29, 2);
			reg8_write(client, 0x2a, 1);
			reg8_write(client, 0x2b, 0);

			/* this is second byte if DBL=1 */
			reg8_write(client, 0x30, 27);
			reg8_write(client, 0x31, 26);
			reg8_write(client, 0x32, 25);
			reg8_write(client, 0x33, 24);
			reg8_write(client, 0x34, 23);
			reg8_write(client, 0x35, 22);
			reg8_write(client, 0x36, 21);
			reg8_write(client, 0x37, 20);
			reg8_write(client, 0x38, 19);
			reg8_write(client, 0x39, 18);
			reg8_write(client, 0x3a, 17);
			reg8_write(client, 0x3b, 16);

			if (!priv->bws && priv->dbl)
				dev_err(&client->dev, " BWS must be 27/32-bit for RAW12 in DBL mode\n");
#endif
			break;
		}

		if (priv->hsgen) {
			/* HS/VS pins map */
			reg8_write(client, 0x3f, 0x10);			/* HS (NC) */
			reg8_write(client, 0x41, 0x10);			/* DE (NC) */
			if (priv->ser_id == MAX96705_ID)
				reg8_write(client, 0x40, 15);		/* VS (DIN13) */
			if (priv->ser_id == MAX96707_ID)
				reg8_write(client, 0x40, 13);		/* VS (DIN13) */
#if 0
			/* following must come from imager */
#define SENSOR_WIDTH	(1280*2)
#define HTS		(1288*2)
#define VTS		960
#define HTS_DELAY	0x9
			reg8_write(client, 0x4e, HTS_DELAY >> 16);	/* HS delay */
			reg8_write(client, 0x4f, (HTS_DELAY >> 8) & 0xff);
			reg8_write(client, 0x50, HTS_DELAY & 0xff);
			reg8_write(client, 0x54, SENSOR_WIDTH >> 8);	/* HS high period */
			reg8_write(client, 0x55, SENSOR_WIDTH & 0xff);
			reg8_write(client, 0x56, (HTS - SENSOR_WIDTH) >> 8); /* HS low period */
			reg8_write(client, 0x57, (HTS - SENSOR_WIDTH) & 0xff);
			reg8_write(client, 0x58, VTS >> 8);		/* HS count */
			reg8_write(client, 0x59, VTS & 0xff );
#endif
			reg8_write(client, 0x43, 0x15);			/* enable HS generator */
		}
	}

	client->addr = priv->des_addr;				/* MAX9288-CAMx I2C */
	reg8_write(client, 0x1c, 0x22 | MAXIM_I2C_I2C_SPEED);	/* disable artificial ACK, I2C speed set */
	usleep_range(2000, 2500);				/* wait 2ms */

	/* I2C translator setup */
	client->addr = 0x40;					/* MAX9271-CAMx I2C */
//	reg8_write(client, 0x09, maxim_map[2][idx] << 1);	/* SENSOR I2C translated - must be set by sensor driver */
//	reg8_write(client, 0x0A, 0x30 << 1);			/* SENSOR I2C native - must be set by sensor driver */
	reg8_write(client, 0x0B, BROADCAST << 1);		/* broadcast I2C */
	reg8_write(client, 0x0C, priv->max9271_addr << 1);	/* MAX9271-CAMx I2C new */
	/* I2C addresse change */
	reg8_write(client, 0x01, priv->des_addr << 1);		/* MAX9288 I2C */
	reg8_write(client, 0x00, priv->max9271_addr << 1);	/* MAX9271-CAM0 I2C new */
	usleep_range(2000, 2500);				/* wait 2ms */
	/* put MAX9271 in configuration link state  */
	client->addr = priv->max9271_addr;			/* MAX9271-CAMx I2C new */
	reg8_write(client, 0x04, 0x43);				/* enable reverse_control/conf_link */
	usleep_range(2000, 2500);				/* wait 2ms */
#ifdef MAXIM_DUMP
	client->addr = priv->des_addr;				/* MAX9288-CAMx I2C */
	maxim_max927x_dump_regs(client);
	client->addr = priv->max9271_addr;			/* MAX9271-CAMx I2C new */
	maxim_max927x_dump_regs(client);
#endif
}

static int max9288_initialize(struct i2c_client *client)
{
	struct max9288_priv *priv = i2c_get_clientdata(client);

	dev_info(&client->dev, "LANES=%d, PCLK edge=%s\n",
			       priv->lanes, priv->pclk_rising_edge ? "rising" : "falling");

	max9288_preinit(client, priv->des_addr);
	max9288_initial_setup(client);

	if (!IS_ERR(priv->poc_gpio)) {
		gpiod_direction_output(priv->poc_gpio, 1); /* POC power on */
		mdelay(priv->poc_delay);
	}

	max9288_reverse_channel_setup(client);
	max9288_gmsl_link_setup(client);

	client->addr = priv->des_addr;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int max9288_g_register(struct v4l2_subdev *sd,
				      struct v4l2_dbg_register *reg)
{
	struct max9288_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;
	int ret;
	u8 val = 0;

	ret = reg8_read(client, (u8)reg->reg, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = sizeof(u8);

	return 0;
}

static int max9288_s_register(struct v4l2_subdev *sd,
				      const struct v4l2_dbg_register *reg)
{
	struct max9288_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;

	return reg8_write(client, (u8)reg->reg, (u8)reg->val);
}
#endif

static int max9288_s_power(struct v4l2_subdev *sd, int on)
{
	struct max9288_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;

	client->addr = priv->max9271_addr;			/* MAX9271-CAMx I2C new */
	reg8_write(client, 0x04, on ? (conf_link ? 0x43 : 0x83) : 0x43); /* enable serial_link or conf_link */
	usleep_range(2000, 2500);				/* wait 2ms after changing reverse_control */
	client->addr = priv->des_addr;				/* MAX9288-CAMx I2C */

	return 0;
}

static struct v4l2_subdev_core_ops max9288_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= max9288_g_register,
	.s_register		= max9288_s_register,
#endif
	.s_power		= max9288_s_power,
};

static struct v4l2_subdev_ops max9288_subdev_ops = {
	.core	= &max9288_subdev_core_ops,
};

static int max9288_parse_dt(struct i2c_client *client)
{
	struct max9288_priv *priv = i2c_get_clientdata(client);
	struct device_node *np = client->dev.of_node;
	struct device_node *endpoint = NULL;
	struct property *prop;
	int err, pwen;
	int sensor_delay, gpio0 = 1, gpio1 = 1;
	u8 val = 0;
	char poc_name[10];

	if (of_property_read_u32(np, "maxim,lanes", &priv->lanes))
		priv->lanes = 4;

	pwen = of_get_gpio(np, 0);
	if (pwen > 0) {
		err = gpio_request_one(pwen, GPIOF_OUT_INIT_HIGH, dev_name(&client->dev));
		if (err)
			dev_err(&client->dev, "cannot request PWEN gpio %d: %d\n", pwen, err);
	}

	mdelay(250);

	sprintf(poc_name, "POC%d", 0);
	priv->poc_gpio = devm_gpiod_get_optional(&client->dev, kstrdup(poc_name, GFP_KERNEL), 0);

	reg8_read(client, 0x1e, &val);				/* read max9288 ID */
	if (val != MAX9288_ID) {
		prop = of_find_property(np, "reg", NULL);
		if (prop)
			of_remove_property(np, prop);
		return -ENODEV;
	}

	if (!of_property_read_u32(np, "maxim,gpio0", &gpio0) ||
	    !of_property_read_u32(np, "maxim,gpio1", &gpio1))
		reg8_write(client, 0x06, (gpio1 << 3) | (gpio0 << 1));

	if (of_property_read_u32(np, "maxim,resetb-gpio", &priv->gpio_resetb)) {
		priv->gpio_resetb = -1;
	} else {
		if (of_property_read_bool(np, "maxim,resetb-active-high"))
			priv->active_low_resetb = 0;
		else
			priv->active_low_resetb = 1;
	}

	if (!of_property_read_u32(np, "maxim,sensor_delay", &sensor_delay))
		mdelay(sensor_delay);
	priv->pclk_rising_edge = true;
	if (of_property_read_bool(np, "maxim,pclk-falling-edge"))
		priv->pclk_rising_edge = false;
	if (of_property_read_u32(np, "maxim,timeout", &priv->timeout))
		priv->timeout = 100;
	if (of_property_read_u32(np, "maxim,him", &priv->him))
		priv->him = 0;
	if (of_property_read_u32(np, "maxim,hsync", &priv->hsync))
		priv->hsync = 0;
	if (of_property_read_u32(np, "maxim,vsync", &priv->vsync))
		priv->vsync = 1;
	if (of_property_read_u32(np, "maxim,poc-delay", &priv->poc_delay))
		priv->poc_delay = 50;
	if (of_property_read_u32(np, "maxim,bws", &priv->bws))
		priv->bws = 0;
	if (of_property_read_u32(np, "maxim,dbl", &priv->dbl))
		priv->dbl = 1;
	if (of_property_read_u32(np, "maxim,dt", &priv->dt))
		priv->dt = 3;
	if (of_property_read_u32(np, "maxim,hsgen", &priv->hsgen))
		priv->hsgen = 0;
	if (of_property_read_u32(np, "maxim,pclk", &priv->pclk))
		priv->pclk = pclk;

	/* module params override dts */
	if (him)
		priv->him = him;
	if (hsync)
		priv->hsync = hsync;
	if (!vsync)
		priv->vsync = vsync;
	if (gpio_resetb)
		priv->gpio_resetb = gpio_resetb;
	if (active_low_resetb)
		priv->active_low_resetb = active_low_resetb;
	if (timeout_n)
		priv->timeout = timeout_n;
	if (poc_delay)
		priv->poc_delay = poc_delay;
	if (bws)
		priv->bws = bws;
	if (!dbl)
		priv->dbl = dbl;
	if (dt != 3)
		priv->dt = dt;
	if (hsgen)
		priv->hsgen = hsgen;
	if (pclk != 100)
		priv->pclk = pclk;

	endpoint = of_graph_get_next_endpoint(np, endpoint);
	if (endpoint) {
		if (of_property_read_u32(endpoint, "max9271-addr", &priv->max9271_addr)) {
			of_node_put(endpoint);
			dev_err(&client->dev, "max9271-addr not set\n");
			return -EINVAL;
		}

		priv->sd_fwnode = of_fwnode_handle(endpoint);
	}

	of_node_put(endpoint);
	return 0;
}

static void max9288_setup_remote_endpoint(struct i2c_client *client)
{
	struct max9288_priv *priv = i2c_get_clientdata(client);
	struct device_node *np = client->dev.of_node;
	struct device_node *endpoint = NULL, *rendpoint = NULL;
	int i;
	struct property *csi_rate_prop, *dvp_order_prop;

	for (i = 0; ; i++) {
		endpoint = of_graph_get_next_endpoint(np, endpoint);
		if (!endpoint)
			break;

		rendpoint = of_parse_phandle(endpoint, "remote-endpoint", 0);
		if (!rendpoint)
			continue;

		csi_rate_prop = of_find_property(endpoint, "csi-rate", NULL);
		if (csi_rate_prop) {
			/* CSI2_RATE = PCLK*bpp/lanes */
			priv->csi_rate = cpu_to_be32(priv->pclk * dt2bpp[priv->dt] / priv->lanes);
			csi_rate_prop->value = &priv->csi_rate;
			of_update_property(rendpoint, csi_rate_prop);
		}

		dvp_order_prop = of_find_property(endpoint, "dvp-order", NULL);
		if (dvp_order_prop)
			of_update_property(rendpoint, dvp_order_prop);
	}

	of_node_put(endpoint);
}

static int max9288_probe(struct i2c_client *client,
				 const struct i2c_device_id *did)
{
	struct max9288_priv *priv;
	int err;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->des_addr = client->addr;
	priv->client = client;

	err = max9288_parse_dt(client);
	if (err)
		goto out;

	err = max9288_initialize(client);
	if (err < 0)
		goto out;

	max9288_setup_remote_endpoint(client);

	v4l2_subdev_init(&priv->sd, &max9288_subdev_ops);
	priv->sd.owner = client->dev.driver->owner;
	priv->sd.dev = &client->dev;
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.fwnode = priv->sd_fwnode;

	snprintf(priv->sd.name, V4L2_SUBDEV_NAME_SIZE, "%s %d-%04x",
		 client->dev.driver->name, i2c_adapter_id(client->adapter),
		 client->addr);

	err = v4l2_async_register_subdev(&priv->sd);
	if (err < 0)
		goto out;
out:
	return err;
}

static int max9288_remove(struct i2c_client *client)
{
	struct max9288_priv *priv = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(&priv->sd);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

static const struct of_device_id max9288_dt_ids[] = {
	{ .compatible = "maxim,max9288" },
	{},
};
MODULE_DEVICE_TABLE(of, max9288_dt_ids);

static const struct i2c_device_id max9288_id[] = {
	{ "max9288", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max9288_id);

static struct i2c_driver max9288_i2c_driver = {
	.driver	= {
		.name		= "max9288",
		.of_match_table	= of_match_ptr(max9288_dt_ids),
	},
	.probe		= max9288_probe,
	.remove		= max9288_remove,
	.id_table	= max9288_id,
};

module_i2c_driver(max9288_i2c_driver);

MODULE_DESCRIPTION("GMSL driver for MAX9288");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
