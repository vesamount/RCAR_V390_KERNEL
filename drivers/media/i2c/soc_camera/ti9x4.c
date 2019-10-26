 /*
 * TI DS90UB954/960/964 FPDLinkIII driver
 *
 * Copyright (C) 2017-2018 Cogent Embedded, Inc.
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

#include "ti9x4.h"

struct ti9x4_priv {
	struct v4l2_subdev	sd[4];
	struct fwnode_handle	*sd_fwnode[4];
	int			des_addr;
	int			links;
	int			lanes;
	int			csi_rate;
	const char		*forwarding_mode;
	int			is_coax;
	int			dvp_bus;
	int			hsync;
	int			vsync;
	int			poc_delay;
	atomic_t		use_count;
	struct i2c_client	*client;
	int			ti9x3_addr_map[4];
	char			chip_id[6];
	int			ser_id;
	struct gpio_desc	*poc_gpio[4]; /* PoC power supply */
};

static int ser_id;
module_param(ser_id, int, 0644);
MODULE_PARM_DESC(ser_id, "  Serializer ID (default: TI913)");

static int is_stp;
module_param(is_stp, int, 0644);
MODULE_PARM_DESC(is_stp, "  STP cable (default: Coax cable)");

static int dvp_bus = 8;
module_param(dvp_bus, int, 0644);
MODULE_PARM_DESC(dvp_bus, "  DVP/CSI over FPDLink (default: DVP 8-bit)");

static int hsync;
module_param(hsync, int, 0644);
MODULE_PARM_DESC(hsync, " HSYNC invertion (default: 0 - not inverted)");

static int vsync = 1;
module_param(vsync, int, 0644);
MODULE_PARM_DESC(vsync, " VSYNC invertion (default: 1 - inverted)");

static int poc_delay;
module_param(poc_delay, int, 0644);
MODULE_PARM_DESC(poc_delay, " Delay in ms after POC enable (default: 0 ms)");

#ifdef TI954_SILICON_ERRATA
static int indirect_write(struct i2c_client *client, unsigned int page, u8 reg, u8 val)
{
	if (page > 7)
		return -EINVAL;

	reg8_write(client, 0xb0, page << 2);
	reg8_write(client, 0xb1, reg);
	reg8_write(client, 0xb2, val);

	return 0;
}

static int indirect_read(struct i2c_client *client, unsigned int page, u8 reg, u8 *val)
{
	if (page > 7)
		return -EINVAL;

	reg8_write(client, 0xb0, page << 2);
	reg8_write(client, 0xb1, reg);
	reg8_read(client, 0xb2, val);

	return 0;
}
#endif

static void ti9x4_read_chipid(struct i2c_client *client)
{
	struct ti9x4_priv *priv = i2c_get_clientdata(client);

	/* Chip ID */
	reg8_read(client, 0xf1, &priv->chip_id[0]);
	reg8_read(client, 0xf2, &priv->chip_id[1]);
	reg8_read(client, 0xf3, &priv->chip_id[2]);
	reg8_read(client, 0xf4, &priv->chip_id[3]);
	reg8_read(client, 0xf5, &priv->chip_id[4]);
	priv->chip_id[5] = '\0';
}

static void ti9x4_initial_setup(struct i2c_client *client)
{
	struct ti9x4_priv *priv = i2c_get_clientdata(client);

	/* Initial setup */
	client->addr = priv->des_addr;				/* TI9x4 I2C */
	reg8_write(client, 0x08, 0x1c);				/* I2C glitch filter depth */
	reg8_write(client, 0x0a, 0x79);				/* I2C high pulse width */
	reg8_write(client, 0x0b, 0x79);				/* I2C low pulse width */
	reg8_write(client, 0x0d, 0xb9);				/* VDDIO 3.3V */
	switch (priv->csi_rate) {
	case 1600: /* REFCLK = 25MHZ */
	case 1450: /* REFCLK = 22.5MHZ */
		reg8_write(client, 0x1f, 0x00);			/* CSI rate 1.5/1.6Gbps */
		break;
	case 800: /* REFCLK = 25MHZ */
	case 700: /* REFCLK = 22.5MHZ */
		reg8_write(client, 0x1f, 0x02);			/* CSI rate 700/800Mbps */
		break;
	case 400: /* REFCLK = 25MHZ */
		reg8_write(client, 0x1f, 0x03);			/* CSI rate 400Mbps */
		break;
	default:
		dev_err(&client->dev, "unsupported CSI rate %d\n", priv->csi_rate);
	}

	if (strcmp(priv->forwarding_mode, "round-robin") == 0) {
		reg8_write(client, 0x21, 0x01);			/* Round Robin forwarding enable */
	} else if (strcmp(priv->forwarding_mode, "synchronized") == 0) {
		reg8_write(client, 0x21, 0x44);			/* Basic Syncronized forwarding enable (FrameSync must be enabled!!) */
	}

	reg8_write(client, 0x32, 0x01);				/* Select TX (CSI) port 0 */
	reg8_write(client, 0x33, ((priv->lanes - 1) ^ 0x3) << 4); /* disable CSI output, set CSI lane count, non-continuous CSI mode */
	reg8_write(client, 0x20, 0xf0);				/* disable port forwarding */
#if 0
	/* FrameSync setup for REFCLK=25MHz,   FPS=30: period_counts=1/2/FPS*25MHz  =1/2/30*25Mhz  =416666 -> FS_TIME=416666 */
	/* FrameSync setup for REFCLK=22.5MHz, FPS=30: period_counts=1/2/FPS*22.5Mhz=1/2/30*22.5Mhz=375000 -> FS_TIME=375000 */
// #define FS_TIME (priv->csi_rate == 1450 ? 376000 : 417666)
 #define FS_TIME (priv->csi_rate == 1450 ? 385000 : 428000) // FPS=29.2 (new vendor's firmware AWB restriction?)
	reg8_write(client, 0x1a, FS_TIME >> 16);		/* FrameSync time 24bit */
	reg8_write(client, 0x1b, (FS_TIME >> 8) & 0xff);
	reg8_write(client, 0x1c, FS_TIME & 0xff);
	reg8_write(client, 0x18, 0x43);				/* Enable FrameSync, 50/50 mode, Frame clock from 25MHz */
#else
	/* FrameSync setup for REFCLK=25MHz,   FPS=30: period_counts=1/FPS/12mks=1/30/12e-6=2777 -> HI=2, LO=2775 */
	/* FrameSync setup for REFCLK=22.5MHz, FPS=30: period_counts=1/FPS/13.333mks=1/30/13.333e-6=2500 -> HI=2, LO=2498 */
 #define FS_TIME (priv->csi_rate == 1450 ? (2498+15) : (2775+15))
	reg8_write(client, 0x19, 2 >> 8);			/* FrameSync high time MSB */
	reg8_write(client, 0x1a, 2 & 0xff);			/* FrameSync high time LSB */
	reg8_write(client, 0x1b, FS_TIME >> 8);			/* FrameSync low time MSB */
	reg8_write(client, 0x1c, FS_TIME & 0xff);		/* FrameSync low time LSB */
	reg8_write(client, 0x18, 0x01);				/* Enable FrameSync, HI/LO mode, Frame clock from port0 */
#endif
}

//#define SENSOR_ID 0x30  // ov10635
//#define SENSOR_ID 0x24  // ov490

static void ti9x4_fpdlink3_setup(struct i2c_client *client, int idx)
{
	struct ti9x4_priv *priv = i2c_get_clientdata(client);
	u8 port_config = 0x78;
	u8 port_config2 = 0;

	/* FPDLinkIII setup */
	client->addr = priv->des_addr;				/* TI9x4 I2C */
	reg8_write(client, 0x4c, (idx << 4) | (1 << idx));	/* Select RX port number */
	usleep_range(2000, 2500);				/* wait 2ms */

	switch (priv->ser_id) {
	case TI913_ID:
		reg8_write(client, 0x58, 0x58);			/* Back channel: Freq=2.5Mbps */
		break;
	case TI953_ID:
		reg8_write(client, 0x58, 0x5e);			/* Back channel: Freq=50Mbps */
		break;
	default:
		break;
	}

	reg8_write(client, 0x5c, priv->ti9x3_addr_map[idx] << 1); /* TI9X3 I2C addr */
//	reg8_write(client, 0x5d, SENSOR_ID << 1);		/* SENSOR I2C native - must be set by sensor driver */
//	reg8_write(client, 0x65, (0x60 + idx) << 1);		/* SENSOR I2C translated - must be set by sensor driver */

	if (priv->is_coax)
		port_config |= 0x04;				/* Coax */
	else
		port_config |= 0x00;				/* STP */

	switch (priv->dvp_bus) {
	case 8:
		port_config2 |= 0x80;				/* RAW10 as 8-bit prosessing using upper bits */
		/* fall through */
	case 10:
		port_config |= 0x03;				/* DVP over FPDLink (TI913 compatible) RAW10/RAW8 */
		break;
	case 12:
		port_config |= 0x02;				/* DVP over FPDLink (TI913 compatible) RAW12 */
		break;
	default:
		port_config |= 0x00;				/* CSI over FPDLink (TI953 compatible) */
	}

	if (priv->vsync)
		port_config2 |= 0x01;				/* VSYNC acive low */
	if (priv->hsync)
		port_config2 |= 0x02;				/* HSYNC acive low */

	reg8_write(client, 0x6d, port_config);
	reg8_write(client, 0x7c, port_config2);
	reg8_write(client, 0x70, (idx << 6) | 0x1e);		/* CSI data type: yuv422 8-bit, assign VC */
	reg8_write(client, 0x71, (idx << 6) | 0x2a);		/* CSI data type: RAW8, assign VC */
	reg8_write(client, 0xbc, 0x00);				/* Setup minimal time between FV and LV to 3 PCLKs */
	reg8_write(client, 0x6e, 0x88);				/* Sensor reset: backchannel GPIO0/GPIO1 set low */
}

static int ti9x4_initialize(struct i2c_client *client)
{
	struct ti9x4_priv *priv = i2c_get_clientdata(client);
	int idx;

	dev_info(&client->dev, "LINKs=%d, LANES=%d, FORWARDING=%s, CABLE=%s, ID=%s\n",
			       priv->links, priv->lanes, priv->forwarding_mode, priv->is_coax ? "coax" : "stp", priv->chip_id);

	ti9x4_initial_setup(client);

	for (idx = 0; idx < priv->links; idx++) {
		if (!IS_ERR(priv->poc_gpio[idx])) {
			gpiod_direction_output(priv->poc_gpio[idx], 1); /* POC power on */
			mdelay(priv->poc_delay);
		}

		ti9x4_fpdlink3_setup(client, idx);
	}

	client->addr = priv->des_addr;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ti9x4_g_register(struct v4l2_subdev *sd,
				      struct v4l2_dbg_register *reg)
{
	struct ti9x4_priv *priv = v4l2_get_subdevdata(sd);
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

static int ti9x4_s_register(struct v4l2_subdev *sd,
				      const struct v4l2_dbg_register *reg)
{
	struct ti9x4_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;

	return reg8_write(client, (u8)reg->reg, (u8)reg->val);
}
#endif

static int ti9x4_s_power(struct v4l2_subdev *sd, int on)
{
	struct ti9x4_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;

	if (on) {
		if (atomic_inc_return(&priv->use_count) == 1)
			reg8_write(client, 0x20, 0x00);		/* enable port forwarding to CSI */
	} else {
		if (atomic_dec_return(&priv->use_count) == 0)
			reg8_write(client, 0x20, 0xf0);		/* disable port forwarding to CSI */
	}

	return 0;
}

static int ti9x4_registered_async(struct v4l2_subdev *sd)
{
	struct ti9x4_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;

	reg8_write(client, 0x33, ((priv->lanes - 1) ^ 0x3) << 4 | 0x1); /* enable CSI output, set CSI lane count, non-continuous CSI mode */

	return 0;
}

static struct v4l2_subdev_core_ops ti9x4_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= ti9x4_g_register,
	.s_register		= ti9x4_s_register,
#endif
	.s_power		= ti9x4_s_power,
	.registered_async	= ti9x4_registered_async,
};

static struct v4l2_subdev_ops ti9x4_subdev_ops = {
	.core	= &ti9x4_subdev_core_ops,
};

static int ti9x4_parse_dt(struct i2c_client *client)
{
	struct ti9x4_priv *priv = i2c_get_clientdata(client);
	struct device_node *np = client->dev.of_node;
	struct device_node *endpoint = NULL, *rendpoint = NULL;
	struct property *prop;
	int err, pwen, i;
	int sensor_delay;
	char forwarding_mode_default[20] = "round-robin"; /* round-robin, synchronized */
	struct property *csi_rate_prop, *dvp_order_prop;
	u8 val = 0;
	char poc_name[10];

	if (of_property_read_u32(np, "ti,links", &priv->links))
		priv->links = 4;

	if (of_property_read_u32(np, "ti,lanes", &priv->lanes))
		priv->lanes = 4;

	pwen = of_get_gpio(np, 0);
	if (pwen > 0) {
		err = devm_gpio_request_one(&client->dev, pwen, GPIOF_OUT_INIT_LOW, dev_name(&client->dev));
		if (err)
			dev_err(&client->dev, "cannot request PWEN gpio %d: %d\n", pwen, err);
		else
			mdelay(250);
	}

	for (i = 0; i < 4; i++) {
		sprintf(poc_name, "POC%d", i);
		priv->poc_gpio[i] = devm_gpiod_get_optional(&client->dev, poc_name, 0);
	}

	reg8_read(client, 0x00, &val);				/* read TI9x4 I2C address */
	if (val != (priv->des_addr << 1)) {
		prop = of_find_property(np, "reg", NULL);
		if (prop)
			of_remove_property(np, prop);
		return -ENODEV;
	}

	ti9x4_read_chipid(client);

#ifdef TI954_SILICON_ERRATA
	indirect_write(client, 7, 0x15, 0x30);
	if (pwen > 0)
		gpio_set_value(pwen, 1);
	usleep_range(5000, 5500);				/* wait 5ms */
	indirect_write(client, 7, 0x15, 0);
#endif
	if (!of_property_read_u32(np, "ti,sensor_delay", &sensor_delay))
		mdelay(sensor_delay);
	if (of_property_read_string(np, "ti,forwarding-mode", &priv->forwarding_mode))
		priv->forwarding_mode = forwarding_mode_default;
	if (of_property_read_bool(np, "ti,stp"))
		priv->is_coax = 0;
	else
		priv->is_coax = 1;
	if (of_property_read_u32(np, "ti,dvp_bus", &priv->dvp_bus))
		priv->dvp_bus = 8;
	if (of_property_read_u32(np, "ti,hsync", &priv->hsync))
		priv->vsync = 0;
	if (of_property_read_u32(np, "ti,vsync", &priv->vsync))
		priv->vsync = 1;
	if (of_property_read_u32(np, "ti,ser_id", &priv->ser_id))
		priv->ser_id = TI913_ID;
	if (of_property_read_u32(np, "ti,poc-delay", &priv->poc_delay))
		priv->poc_delay = 50;

	/* module params override dts */
	if (is_stp)
		priv->is_coax = 0;
	if (dvp_bus != 8)
		priv->dvp_bus = dvp_bus;
	if (hsync)
		priv->hsync = hsync;
	if (!vsync)
		priv->vsync = vsync;
	if (ser_id)
		priv->ser_id = ser_id;
	if (poc_delay)
		priv->poc_delay = poc_delay;

	for (i = 0; ; i++) {
		endpoint = of_graph_get_next_endpoint(np, endpoint);
		if (!endpoint)
			break;

		of_node_put(endpoint);

		if (i < priv->links) {
			if (of_property_read_u32(endpoint, "ti9x3-addr", &priv->ti9x3_addr_map[i])) {
				dev_err(&client->dev, "ti9x3-addr not set\n");
				return -EINVAL;
			}
			priv->sd_fwnode[i] = of_fwnode_handle(endpoint);
		}

		rendpoint = of_parse_phandle(endpoint, "remote-endpoint", 0);
		if (!rendpoint)
			continue;

		csi_rate_prop = of_find_property(endpoint, "csi-rate", NULL);
		if (csi_rate_prop) {
			of_property_read_u32(endpoint, "csi-rate", &priv->csi_rate);
			of_update_property(rendpoint, csi_rate_prop);
		}

		dvp_order_prop = of_find_property(endpoint, "dvp-order", NULL);
		if (dvp_order_prop)
			of_update_property(rendpoint, dvp_order_prop);
	}

	return 0;
}

static int ti9x4_probe(struct i2c_client *client,
			     const struct i2c_device_id *did)
{
	struct ti9x4_priv *priv;
	int err, i;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->des_addr = client->addr;
	priv->client = client;
	atomic_set(&priv->use_count, 0);

	err = ti9x4_parse_dt(client);
	if (err)
		goto out;

	err = ti9x4_initialize(client);
	if (err < 0)
		goto out;

	for (i = 0; i < priv->links; i++) {
		v4l2_subdev_init(&priv->sd[i], &ti9x4_subdev_ops);
		priv->sd[i].owner = client->dev.driver->owner;
		priv->sd[i].dev = &client->dev;
		priv->sd[i].grp_id = i;
		v4l2_set_subdevdata(&priv->sd[i], priv);
		priv->sd[i].fwnode = priv->sd_fwnode[i];

		snprintf(priv->sd[i].name, V4L2_SUBDEV_NAME_SIZE, "%s %d-%04x",
			 client->dev.driver->name, i2c_adapter_id(client->adapter),
			 client->addr);

		err = v4l2_async_register_subdev(&priv->sd[i]);
		if (err < 0)
			goto out;
	}

out:
	return err;
}

static int ti9x4_remove(struct i2c_client *client)
{
	struct ti9x4_priv *priv = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < priv->links; i++) {
		v4l2_async_unregister_subdev(&priv->sd[i]);
		v4l2_device_unregister_subdev(&priv->sd[i]);
	}

	return 0;
}

static const struct of_device_id ti9x4_dt_ids[] = {
	{ .compatible = "ti,ti9x4" },
	{},
};
MODULE_DEVICE_TABLE(of, ti9x4_dt_ids);

static const struct i2c_device_id ti9x4_id[] = {
	{ "ti9x4", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ti9x4_id);

static struct i2c_driver ti9x4_i2c_driver = {
	.driver	= {
		.name		= "ti9x4",
		.of_match_table	= of_match_ptr(ti9x4_dt_ids),
	},
	.probe		= ti9x4_probe,
	.remove		= ti9x4_remove,
	.id_table	= ti9x4_id,
};

module_i2c_driver(ti9x4_i2c_driver);

MODULE_DESCRIPTION("FPDLinkIII driver for DS90UB9x4");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
