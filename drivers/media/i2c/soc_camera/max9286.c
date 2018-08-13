/*
 * MAXIM max9286 GMSL driver
 *
 * Copyright (C) 2015-2018 Cogent Embedded, Inc.
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

#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
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

struct max9286_priv {
	struct v4l2_subdev	sd[4];
	struct fwnode_handle	*sd_fwnode[4];
	int			des_addr;
	int			des_quirk_addr; /* second MAX9286 on the same I2C bus */
	int			links;
	int			links_mask;
	int			lanes;
	int			csi_rate;
	const char		*fsync_mode;
	int			fsync_period;
	char			pclk_rising_edge;
	int			gpio_resetb;
	int			active_low_resetb;
	int			him;
	int			hsync;
	int			vsync;
	int			timeout;
	int			poc_delay;
	atomic_t		use_count;
	u32			csi2_outord;
	struct i2c_client	*client;
	int			max9271_addr_map[4];
	int			ser_id;
	struct gpio_desc	*poc_gpio[4]; /* PoC power supply */
};

static char fsync_mode_default[20] = "manual"; /* manual, automatic, semi-automatic, external */

static int conf_link;
module_param(conf_link, int, 0644);
MODULE_PARM_DESC(conf_link, " Force configuration link. Used only if robust firmware flashing required (f.e. recovery)");

static int poc_trig;
module_param(poc_trig, int, 0644);
MODULE_PARM_DESC(poc_trig, " Use PoC triggering during reverse channel setup. Useful on systems with dedicated PoC and unstable ser-des lock");

static int him;
module_param(him, int, 0644);
MODULE_PARM_DESC(him, " Use High-Immunity mode (default: leagacy mode)");

static int fsync_period;
module_param(fsync_period, int, 0644);
MODULE_PARM_DESC(fsync_period, " Frame sync period (default: 3.2MHz)");

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

static int poc_delay;
module_param(poc_delay, int, 0644);
MODULE_PARM_DESC(poc_delay, " Delay in ms after POC enable (default: 0 ms)");

static char* ser_name(int id)
{
	switch (id) {
	case MAX9271_ID:
		return "MAX9271";
	case MAX96705_ID:
		return "MAX96705";
	default:
		return "unknown";
	}
}

static void max9286_preinit(struct i2c_client *client, int addr)
{
	struct max9286_priv *priv = i2c_get_clientdata(client);

	client->addr = addr;			/* MAX9286-CAMx I2C */
	reg8_write(client, 0x0a, 0x00);		/* disable reverse control for all cams */
	reg8_write(client, 0x00, 0x00);		/* disable all GMSL links [0:3] */
	usleep_range(2000, 2500);		/* wait 2ms after any change of reverse channel settings */
	reg8_write(client, 0x1c, priv->him ? 0xf4 : 0x04); /* high-immunity or legacy mode */
}

static void max9286_sensor_reset(struct i2c_client *client, int addr, int reset_on)
{
	struct max9286_priv *priv = i2c_get_clientdata(client);

	if (priv->gpio_resetb < 1 || priv->gpio_resetb > 5)
		return;

	/* sensor reset/unreset */
	client->addr = addr;					/* MAX9271-CAMx I2C */
	reg8_write(client, 0x0f, (0xfe & ~BIT(priv->gpio_resetb)) | /* set GPIOn value to reset/unreset */
		   ((priv->active_low_resetb ? BIT(priv->gpio_resetb) : 0) ^ reset_on));
	reg8_write(client, 0x0e, 0x42 | BIT(priv->gpio_resetb)); /* set GPIOn direction output */
}

static void max9286_postinit(struct i2c_client *client, int addr)
{
	struct max9286_priv *priv = i2c_get_clientdata(client);
	int idx;

	for (idx = 0; idx < priv->links; idx++) {
		client->addr = priv->des_addr;			/* MAX9286 I2C */
		reg8_write(client, 0x0a, 0x11 << idx);		/* enable reverse/forward control for CAMx */

		client->addr = priv->max9271_addr_map[idx];	/* MAX9271-CAMx I2C */
		max9286_sensor_reset(client, client->addr, 0); /* sensor unreset */
	}

	client->addr = addr;					/* MAX9286 I2C */
	reg8_write(client, 0x0a, 0x00);				/* disable reverse control for all cams */
	reg8_write(client, 0x00, 0xe0 | priv->links_mask);	/* enable GMSL link for CAMs */
	reg8_write(client, 0x0b, priv->csi2_outord);		/* CSI2 output order */
	reg8_write(client, 0x15, 0x9b);				/* enable CSI output, VC is set accordingly to Link number, BIT7 magic must be set */
	reg8_write(client, 0x1b, priv->links_mask);		/* enable equalizer for CAMs */
	usleep_range(5000, 5500);				/* wait 2ms after any change of reverse channel settings */

	if (strcmp(priv->fsync_mode, "manual") == 0) {
		reg8_write(client, 0x01, 0x00);			/* manual: FRAMESYNC set manually via [0x06:0x08] regs */
	} else if (strcmp(priv->fsync_mode, "automatic") == 0) {
		reg8_write(client, 0x01, 0x02);			/* automatic: FRAMESYNC taken from the slowest Link */
	} else if (strcmp(priv->fsync_mode, "semi-automatic") == 0) {
		reg8_write(client, 0x01, 0x01);			/* semi-automatic: FRAMESYNC taken from the slowest Link */
	} else if (strcmp(priv->fsync_mode, "external") == 0) {
		reg8_write(client, 0x01, 0xc0);			/* ECU (aka MCU) based FrameSync using GPI-to-GPO */
	}
}

static int max9286_reverse_channel_setup(struct i2c_client *client, int idx)
{
	struct max9286_priv *priv = i2c_get_clientdata(client);
	u8 val = 0;
	int timeout = priv->timeout;
	char timeout_str[10];
	int ret = 0;

	/* Reverse channel enable */
	client->addr = priv->des_addr;				/* MAX9286-CAMx I2C */
	reg8_write(client, 0x3f, 0x4f);				/* enable custom reverse channel & first pulse length */
	reg8_write(client, 0x34, 0xa2 | MAXIM_I2C_I2C_SPEED);	/* enable artificial ACKs, I2C speed set */
	usleep_range(2000, 2500);				/* wait 2ms after any change of reverse channel settings */
	reg8_write(client, 0x00, 0xe0 | BIT(idx));		/* enable GMSL link for CAMx */
	reg8_write(client, 0x0a, 0x11 << idx);			/* enable reverse control for CAMx */
	usleep_range(2000, 2500);				/* wait 2ms after any change of reverse channel settings */

	for (;;) {
		client->addr = priv->des_addr;			/* MAX9286-CAMx I2C */
		reg8_write(client, 0x3b, 0x1e);			/* first pulse length rise time changed from 300ns to 200ns, amplitude 100mV */
		usleep_range(2000, 2500);			/* wait 2ms after any change of reverse channel settings */

		client->addr = 0x40;				/* MAX9271-CAMx I2C */
		reg8_write(client, 0x04, 0x43);			/* wake-up, enable reverse_control/conf_link */
		usleep_range(2000, 2500);			/* wait 2ms after any change of reverse channel settings */
		reg8_write(client, 0x08, 0x01);			/* reverse channel receiver high threshold enable */
		reg8_write(client, 0x97, priv->him ? 0xaf : 0x5f); /* enable reverse control channel programming (MAX96705-MAX96711 only) */
		usleep_range(2000, 2500);			/* wait 2ms after any change of reverse channel settings */

		client->addr = priv->des_addr;			/* MAX9286-CAMx I2C */
		reg8_write(client, 0x3b, 0x19);			/* reverse channel increase amplitude 170mV to compensate high threshold enabled */
		usleep_range(2000, 2500);			/* wait 2ms after any change of reverse channel settings */

		client->addr = 0x40;				/* MAX9271-CAMx I2C */
		reg8_read(client, 0x1e, &val);			/* read max9271 ID */
		if (val == MAX9271_ID || val == MAX96705_ID || --timeout == 0) {
			priv->ser_id = val;
			break;
		}

		/* Check if already initialized (after reboot/reset ?) */
		client->addr = priv->max9271_addr_map[idx];	/* MAX9271-CAMx I2C */
		reg8_read(client, 0x1e, &val);			/* read max9271 ID */
		if (val == MAX9271_ID || val == MAX96705_ID) {
			priv->ser_id = val;
			reg8_write(client, 0x04, 0x43);		/* enable reverse_control/conf_link */
			usleep_range(2000, 2500);		/* wait 2ms after any change of reverse channel settings */
			ret = -EADDRINUSE;
			break;
		}

		if (timeout == priv->timeout / 2 && poc_trig) {
			if (!IS_ERR(priv->poc_gpio[idx])) {
				gpiod_direction_output(priv->poc_gpio[idx], 0); /* POC power off */
				mdelay(200);
				gpiod_direction_output(priv->poc_gpio[idx], 1); /* POC power on */
				mdelay(priv->poc_delay);
			}
		}
	}

	max9286_sensor_reset(client, client->addr, 1);	/* sensor reset */

	if (!timeout) {
		ret = -ETIMEDOUT;
		goto out;
	}

	priv->links_mask |= BIT(idx);
	priv->csi2_outord &= ~(0x3 << (idx * 2));
	priv->csi2_outord |= ((hweight8(priv->links_mask) - 1) << (idx * 2));

out:
	sprintf(timeout_str, "retries=%d", priv->timeout - timeout);
	dev_info(&client->dev, "link%d %s %sat 0x%x %s %s\n", idx, ser_name(priv->ser_id),
			       ret == -EADDRINUSE ? "already " : "", priv->max9271_addr_map[idx],
			       ret == -ETIMEDOUT ? "not found: timeout GMSL link establish" : "",
			       priv->timeout - timeout? timeout_str : "");

	return ret;
}

static void max9286_initial_setup(struct i2c_client *client)
{
	struct max9286_priv *priv = i2c_get_clientdata(client);

	/* Initial setup */
	client->addr = priv->des_addr;				/* MAX9286-CAMx I2C */
	reg8_write(client, 0x15, 0x13);				/* disable CSI output, VC is set accordingly to Link number */
	reg8_write(client, 0x69, 0x0f);				/* mask CSI forwarding from all links */
	switch (priv->lanes) {
	case 1:
		reg8_write(client, 0x12, 0x33);			/* enable CSI-2 Lane D0, DBL mode, YUV422 8-bit*/
		break;
	case 2:
		reg8_write(client, 0x12, 0x73);			/* enable CSI-2 Lanes D0,D1, DBL mode, YUV422 8-bit*/
		break;
	case 3:
		reg8_write(client, 0x12, 0xd3);			/* enable CSI-2 Lanes D0-D2, DBL mode, YUV422 8-bit*/
		break;
	case 4:
		reg8_write(client, 0x12, 0xf3);			/* enable CSI-2 Lanes D0-D3, DBL mode, YUV422 8-bit*/
		break;
	default:
		dev_err(&client->dev, "CSI2 lanes number is invalid (%d)\n", priv->lanes);
	}

	/* Start GMSL initialization with FSYNC disabled. This is required for some odd LVDS cameras */
	reg8_write(client, 0x01, 0xc0);				/* ECU (aka MCU) based FrameSync using GPI-to-GPO */
	reg8_write(client, 0x06, priv->fsync_period & 0xff);
	reg8_write(client, 0x07, (priv->fsync_period >> 8) & 0xff);
	reg8_write(client, 0x08, priv->fsync_period >> 16);

	reg8_write(client, 0x63, 0);				/* disable overlap window */
	reg8_write(client, 0x64, 0);
	reg8_write(client, 0x0c, 0x91 | (priv->vsync ? BIT(3) : 0) | (priv->hsync ? BIT(2) : 0)); /* enable HS/VS encoding, use D14/15 for HS/VS, invert HS/VS */
	reg8_write(client, 0x19, 0x0c);				/* Drive HSTRAIL state for 120ns after the last payload bit */
}

static void max9286_gmsl_link_setup(struct i2c_client *client, int idx)
{
	struct max9286_priv *priv = i2c_get_clientdata(client);

	/* GMSL setup */
	client->addr = 0x40;					/* MAX9271-CAMx I2C */
	reg8_write(client, 0x0d, 0x22 | MAXIM_I2C_I2C_SPEED);	/* disable artificial ACK, I2C speed set */
	reg8_write(client, 0x07, 0x84 | (priv->pclk_rising_edge ? 0 : 0x10)); /* RAW/YUV, PCLK edge, HS/VS encoding enabled */
	usleep_range(2000, 2500);				/* wait 2ms */
	reg8_write(client, 0x02, 0xff);				/* spread spectrum +-4%, pclk range automatic, Gbps automatic  */
	usleep_range(2000, 2500);				/* wait 2ms */

	if (priv->ser_id == MAX96705_ID) {
		/* setup crossbar in DBL mode: reverse DVP bus */
		reg8_write(client, 0x20, 0x07);
		reg8_write(client, 0x21, 0x06);
		reg8_write(client, 0x22, 0x05);
		reg8_write(client, 0x23, 0x04);
		reg8_write(client, 0x24, 0x03);
		reg8_write(client, 0x25, 0x02);
		reg8_write(client, 0x26, 0x01);
		reg8_write(client, 0x27, 0x00);

		reg8_write(client, 0x30, 0x17);
		reg8_write(client, 0x31, 0x16);
		reg8_write(client, 0x32, 0x15);
		reg8_write(client, 0x33, 0x14);
		reg8_write(client, 0x34, 0x13);
		reg8_write(client, 0x35, 0x12);
		reg8_write(client, 0x36, 0x11);
		reg8_write(client, 0x37, 0x10);
	}

	client->addr = priv->des_addr;				/* MAX9286-CAMx I2C */
	reg8_write(client, 0x34, 0x22 | MAXIM_I2C_I2C_SPEED);	/* disable artificial ACK, I2C speed set */
	usleep_range(2000, 2500);				/* wait 2ms */

	/* I2C translator setup */
	client->addr = 0x40;					/* MAX9271-CAMx I2C */
//	reg8_write(client, 0x09, maxim_map[2][idx] << 1);	/* SENSOR I2C translated - must be set by sensor driver */
//	reg8_write(client, 0x0A, 0x30 << 1);			/* SENSOR I2C native - must be set by sensor driver */
	reg8_write(client, 0x0B, BROADCAST << 1);		/* broadcast I2C */
	reg8_write(client, 0x0C, priv->max9271_addr_map[idx] << 1); /* MAX9271-CAMx I2C new */
	/* I2C addresse change */
	reg8_write(client, 0x01, priv->des_addr << 1);		/* MAX9286 I2C */
	reg8_write(client, 0x00, priv->max9271_addr_map[idx] << 1); /* MAX9271-CAM0 I2C new */
	usleep_range(2000, 2500);				/* wait 2ms */
	/* put MAX9271 in configuration link state  */
	client->addr = priv->max9271_addr_map[idx];		/* MAX9271-CAMx I2C new */
	reg8_write(client, 0x04, 0x43);				/* enable reverse_control/conf_link */
	usleep_range(2000, 2500);				/* wait 2ms */
#ifdef MAXIM_DUMP
	client->addr = priv->des_addr;				/* MAX9286-CAMx I2C */
	maxim_max927x_dump_regs(client);
	client->addr = priv->max9271_addr_map[idx];		/* MAX9271-CAMx I2C new */
	maxim_max927x_dump_regs(client);
#endif
}

static int max9286_initialize(struct i2c_client *client)
{
	struct max9286_priv *priv = i2c_get_clientdata(client);
	int idx, ret;

	dev_info(&client->dev, "LINKs=%d, LANES=%d, FSYNC mode=%s, FSYNC period=%d, PCLK edge=%s\n",
			       priv->links, priv->lanes, priv->fsync_mode, priv->fsync_period,
			       priv->pclk_rising_edge ? "rising" : "falling");

	if (priv->des_quirk_addr)
		max9286_preinit(client, priv->des_quirk_addr);

	max9286_preinit(client, priv->des_addr);
	max9286_initial_setup(client);

	for (idx = 0; idx < priv->links; idx++) {
		if (!IS_ERR(priv->poc_gpio[idx])) {
			gpiod_direction_output(priv->poc_gpio[idx], 1); /* POC power on */
			mdelay(priv->poc_delay);
		}

		ret = max9286_reverse_channel_setup(client, idx);
		if (ret)
			continue;
		max9286_gmsl_link_setup(client, idx);
	}

	max9286_postinit(client, priv->des_addr);

	client->addr = priv->des_addr;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int max9286_g_register(struct v4l2_subdev *sd,
				      struct v4l2_dbg_register *reg)
{
	struct max9286_priv *priv = v4l2_get_subdevdata(sd);
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

static int max9286_s_register(struct v4l2_subdev *sd,
				      const struct v4l2_dbg_register *reg)
{
	struct max9286_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;

	return reg8_write(client, (u8)reg->reg, (u8)reg->val);
}
#endif

static int max9286_s_power(struct v4l2_subdev *sd, int on)
{
	struct max9286_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;

	if (on) {
		if (atomic_inc_return(&priv->use_count) == 1)
			reg8_write(client, 0x69, priv->links_mask ^ 0x0f); /* unmask CSI forwarding from detected links */
	} else {
		if (atomic_dec_return(&priv->use_count) == 0)
			reg8_write(client, 0x69, 0x0f);		/* mask CSI forwarding from all links */
	}

	return 0;
}

static int max9286_registered_async(struct v4l2_subdev *sd)
{
	struct max9286_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;
	int idx, tmp_addr;

	/* switch to GMSL serial_link for streaming video */
	tmp_addr = client->addr;
	idx = sd->grp_id;

	client->addr = priv->des_addr;				/* MAX9286 I2C */
	reg8_write(client, 0x0a, 0x11 << idx);			/* enable reverse/forward control for CAMx */

	client->addr = priv->max9271_addr_map[idx];		/* MAX9271-CAMx */
	reg8_write(client, 0x04, conf_link ? 0x43 : 0x83);	/* enable serial_link */
	usleep_range(2000, 2500);				/* wait 2ms after changing reverse_control */

	client->addr = priv->des_addr;				/* MAX9286 I2C */
	reg8_write(client, 0x0a, (priv->links_mask << 4) | priv->links_mask); /* enable reverse/forward control for all CAMs */

	client->addr = tmp_addr;

	return 0;
}

static struct v4l2_subdev_core_ops max9286_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= max9286_g_register,
	.s_register		= max9286_s_register,
#endif
	.s_power		= max9286_s_power,
	.registered_async	= max9286_registered_async,
};

static struct v4l2_subdev_ops max9286_subdev_ops = {
	.core	= &max9286_subdev_core_ops,
};

static int max9286_parse_dt(struct i2c_client *client)
{
	struct max9286_priv *priv = i2c_get_clientdata(client);
	struct device_node *np = client->dev.of_node;
	struct device_node *endpoint = NULL;
	struct property *prop;
	int err, pwen, i;
	int sensor_delay, gpio0 = 1, gpio1 = 1;
	u8 val = 0;
	char poc_name[10];

	if (of_property_read_u32(np, "maxim,links", &priv->links))
		priv->links = 4;

	if (of_property_read_u32(np, "maxim,lanes", &priv->lanes))
		priv->lanes = 4;

	pwen = of_get_gpio(np, 0);
	if (pwen > 0) {
		err = gpio_request_one(pwen, GPIOF_OUT_INIT_HIGH, dev_name(&client->dev));
		if (err)
			dev_err(&client->dev, "cannot request PWEN gpio %d: %d\n", pwen, err);
	}

	mdelay(250);

	for (i = 0; i < 4; i++) {
		sprintf(poc_name, "POC%d", i);
		priv->poc_gpio[i] = devm_gpiod_get_optional(&client->dev, poc_name, 0);
	}

	reg8_read(client, 0x1e, &val);				/* read max9286 ID */
	if (val != MAX9286_ID) {
		prop = of_find_property(np, "reg", NULL);
		if (prop)
			of_remove_property(np, prop);
		return -ENODEV;
	}

	if (!of_property_read_u32(np, "maxim,gpio0", &gpio0) ||
	    !of_property_read_u32(np, "maxim,gpio1", &gpio1))
		reg8_write(client, 0x0f, 0x08 | (gpio1 << 1) | gpio0);

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
	if (of_property_read_string(np, "maxim,fsync-mode", &priv->fsync_mode))
		priv->fsync_mode = fsync_mode_default;
	if (of_property_read_u32(np, "maxim,fsync-period", &priv->fsync_period))
		priv->fsync_period = 3200000;			/* 96MHz/30fps */
	priv->pclk_rising_edge = true;
	if (of_property_read_bool(np, "maxim,pclk-falling-edge"))
		priv->pclk_rising_edge = false;
	if (of_property_read_u32(np, "maxim,timeout", &priv->timeout))
		priv->timeout = 100;
	if (of_property_read_u32(np, "maxim,i2c-quirk", &priv->des_quirk_addr))
		priv->des_quirk_addr = 0;
	if (of_property_read_u32(np, "maxim,him", &priv->him))
		priv->him = 0;
	if (of_property_read_u32(np, "maxim,hsync", &priv->hsync))
		priv->hsync = 0;
	if (of_property_read_u32(np, "maxim,vsync", &priv->vsync))
		priv->vsync = 1;
	if (of_property_read_u32(np, "maxim,poc-delay", &priv->poc_delay))
		priv->poc_delay = 50;

	/* module params override dts */
	if (him)
		priv->him = him;
	if (fsync_period) {
		priv->fsync_period = fsync_period;
		priv->fsync_mode = fsync_mode_default;
	}
	if (hsync)
		priv->hsync = hsync;
	if (!vsync)
		priv->vsync = vsync;
	if (gpio_resetb)
		priv->gpio_resetb = gpio_resetb;
	if (active_low_resetb)
		priv->active_low_resetb = active_low_resetb;
	if (poc_delay)
		priv->poc_delay = poc_delay;

	for (i = 0; i < priv->links; i++) {
		endpoint = of_graph_get_next_endpoint(np, endpoint);
		if (!endpoint)
			break;

		if (of_property_read_u32(endpoint, "max9271-addr", &priv->max9271_addr_map[i])) {
			of_node_put(endpoint);
			dev_err(&client->dev, "max9271-addr not set\n");
			return -EINVAL;
		}

		priv->sd_fwnode[i] = of_fwnode_handle(endpoint);
	}

	of_node_put(endpoint);
	return 0;
}

static void max9286_setup_remote_endpoint(struct i2c_client *client)
{
	struct max9286_priv *priv = i2c_get_clientdata(client);
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
			/* CSI2_RATE = PCLK*sizeof(YUV8)*links/lanes */
			priv->csi_rate = cpu_to_be32(100 * 8 * hweight8(priv->links_mask) / priv->lanes);
			csi_rate_prop->value = &priv->csi_rate;
			of_update_property(rendpoint, csi_rate_prop);
		}

		dvp_order_prop = of_find_property(endpoint, "dvp-order", NULL);
		if (dvp_order_prop)
			of_update_property(rendpoint, dvp_order_prop);
	}

	of_node_put(endpoint);
}

static int max9286_probe(struct i2c_client *client,
				 const struct i2c_device_id *did)
{
	struct max9286_priv *priv;
	int err, i;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->des_addr = client->addr;
	priv->client = client;
	atomic_set(&priv->use_count, 0);
	priv->csi2_outord = 0xff;

	err = max9286_parse_dt(client);
	if (err)
		goto out;

	err = max9286_initialize(client);
	if (err < 0)
		goto out;

	max9286_setup_remote_endpoint(client);

	for (i = 0; i < 4; i++) {
		v4l2_subdev_init(&priv->sd[i], &max9286_subdev_ops);
		priv->sd[i].owner = client->dev.driver->owner;
		priv->sd[i].dev = &client->dev;
		priv->sd[i].grp_id = i;
		v4l2_set_subdevdata(&priv->sd[i], priv);
		priv->sd[i].fwnode = priv->sd_fwnode[i];

		snprintf(priv->sd[i].name, V4L2_SUBDEV_NAME_SIZE, "%s.%d %d-%04x",
			 client->dev.driver->name, i, i2c_adapter_id(client->adapter),
			 client->addr);

		err = v4l2_async_register_subdev(&priv->sd[i]);
		if (err < 0)
			goto out;
	}
out:
	return err;
}

static int max9286_remove(struct i2c_client *client)
{
	struct max9286_priv *priv = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < 4; i++) {
		v4l2_async_unregister_subdev(&priv->sd[i]);
		v4l2_device_unregister_subdev(&priv->sd[i]);
	}

	return 0;
}

static const struct of_device_id max9286_dt_ids[] = {
	{ .compatible = "maxim,max9286" },
	{},
};
MODULE_DEVICE_TABLE(of, max9286_dt_ids);

static const struct i2c_device_id max9286_id[] = {
	{ "max9286", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max9286_id);

static struct i2c_driver max9286_i2c_driver = {
	.driver	= {
		.name		= "max9286",
		.of_match_table	= of_match_ptr(max9286_dt_ids),
	},
	.probe		= max9286_probe,
	.remove		= max9286_remove,
	.id_table	= max9286_id,
};

module_i2c_driver(max9286_i2c_driver);

MODULE_DESCRIPTION("GMSL driver for MAX9286");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
