/*
 * GEO Semiconductor GW4200-AR014X sensor camera driver
 *
 * Copyright (C) 2018 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/videodev2.h>

#include <media/soc_camera.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>

#include "gw4200_ar014x.h"

#define GW4200_I2C_ADDR		0x6d

#define GW4200_PID		0x00
#define GW4200_VERSION_REG	0xda

#define GW4200_MEDIA_BUS_FMT	MEDIA_BUS_FMT_YUYV8_2X8

static void gw4200_otp_id_read(struct i2c_client *client);

struct gw4200_priv {
	struct v4l2_subdev		sd;
	struct v4l2_ctrl_handler	hdl;
	struct media_pad		pad;
	struct v4l2_rect		rect;
	int				init_complete;
	u8				id[6];
	int				exposure;
	int				gain;
	int				autogain;
	/* serializers */
	int				max9286_addr;
	int				max9271_addr;
	int				port;
	int				gpio_resetb;
	int				gpio_fsin;
};

static inline struct gw4200_priv *to_gw4200(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct gw4200_priv, sd);
}

static void gw4200_s_port(struct i2c_client *client, int fwd_en)
{
	struct gw4200_priv *priv = to_gw4200(client);
	int tmp_addr;

	if (priv->max9286_addr) {
		tmp_addr = client->addr;
		client->addr = priv->max9286_addr;				/* Deserializer I2C address */
		reg8_write(client, 0x0a, fwd_en ? 0x11 << priv->port : 0);	/* Enable/disable reverse/forward control for this port */
		usleep_range(5000, 5500);					/* wait 5ms */
		client->addr = tmp_addr;
	};
}

#if 0
static int gw4200_set_regs(struct i2c_client *client,
			  const struct gw4200_reg *regs, int nr_regs)
{
	int i;

	for (i = 0; i < nr_regs; i++) {
		if (regs[i].reg == GW4200_DELAY) {
			mdelay(regs[i].val);
			continue;
		}

		reg16_write16(client, regs[i].reg, regs[i].val);
	}

	return 0;
}
#endif

static u16 gw4200_ar014x_read(struct i2c_client *client, u16 addr)
{
	u16 reg_val = 0;

	reg16_write16(client, 0x0040, 0x8d00);
	usleep_range(100, 150); /* wait 100 us */
	reg16_write16(client, 0xfc00, addr);
	reg16_write16(client, 0xfc02, 0x0200); /* 2 bytes */
	reg16_write16(client, 0x0040, 0x8d05);
	usleep_range(100, 150); /* wait 100 us */
	reg16_write16(client, 0x0040, 0x8d08);
	usleep_range(100, 150); /* wait 100 us */
	reg16_read16(client, 0xfc00, &reg_val);
	reg16_write16(client, 0x0040, 0x8d02);
	usleep_range(100, 150); /* wait 100 us */

	return reg_val;
}

static void gw4200_ar014x_write(struct i2c_client *client, u16 addr, u16 val)
{
	reg16_write16(client, 0x0040, 0x8d00);
	usleep_range(100, 150); /* wait 100 us */
	reg16_write16(client, 0xfc00, addr);
	reg16_write16(client, 0xfc02, 0x0200 | (val >> 8)); /* 2 bytes */
	reg16_write16(client, 0xfc04, (val & 0xff) << 8);
	reg16_write16(client, 0x0040, 0x8d06);
	usleep_range(100, 150); /* wait 100 us */
	reg16_write16(client, 0x0040, 0x8d08);
	usleep_range(100, 150); /* wait 100 us */
	reg16_write16(client, 0x0040, 0x8d02);
	usleep_range(100, 150); /* wait 100 us */
}

static int gw4200_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int gw4200_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gw4200_priv *priv = to_gw4200(client);

	if (format->pad)
		return -EINVAL;

	mf->width = priv->rect.width;
	mf->height = priv->rect.height;
	mf->code = GW4200_MEDIA_BUS_FMT;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int gw4200_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;

	mf->code = GW4200_MEDIA_BUS_FMT;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		cfg->try_fmt = *mf;

	return 0;
}

static int gw4200_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = GW4200_MEDIA_BUS_FMT;

	return 0;
}

static int gw4200_get_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gw4200_priv *priv = to_gw4200(client);

	gw4200_otp_id_read(client);

	memcpy(edid->edid, priv->id, 6);

	edid->edid[6] = 0xff;
	edid->edid[7] = client->addr;
	edid->edid[8] = GW4200_VERSION_REG >> 8;
	edid->edid[9] = GW4200_VERSION_REG & 0xff;

	return 0;
}

static int gw4200_set_selection(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_selection *sel)
{
	struct v4l2_rect *rect = &sel->r;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gw4200_priv *priv = to_gw4200(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE ||
	    sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	rect->left = ALIGN(rect->left, 2);
	rect->top = ALIGN(rect->top, 2);
	rect->width = ALIGN(rect->width, 2);
	rect->height = ALIGN(rect->height, 2);

	if ((rect->left + rect->width > GW4200_MAX_WIDTH) ||
	    (rect->top + rect->height > GW4200_MAX_HEIGHT))
		*rect = priv->rect;

	priv->rect.left = rect->left;
	priv->rect.top = rect->top;
	priv->rect.width = rect->width;
	priv->rect.height = rect->height;

	return 0;
}

static int gw4200_get_selection(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_selection *sel)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gw4200_priv *priv = to_gw4200(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = GW4200_MAX_WIDTH;
		sel->r.height = GW4200_MAX_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = GW4200_MAX_WIDTH;
		sel->r.height = GW4200_MAX_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP:
		sel->r = priv->rect;
		return 0;
	default:
		return -EINVAL;
	}
}

static int gw4200_g_mbus_config(struct v4l2_subdev *sd,
			       struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->type = V4L2_MBUS_CSI2;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int gw4200_g_register(struct v4l2_subdev *sd,
			    struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 val = 0;

	ret = reg16_read16(client, (u16)reg->reg, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = sizeof(u16);

	return 0;
}

static int gw4200_s_register(struct v4l2_subdev *sd,
			    const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return reg16_write16(client, (u16)reg->reg, (u16)reg->val);
}
#endif

static struct v4l2_subdev_core_ops gw4200_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = gw4200_g_register,
	.s_register = gw4200_s_register,
#endif
};

static int gw4200_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gw4200_priv *priv = to_gw4200(client);
	int ret = -EINVAL;

	if (!priv->init_complete)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
	case V4L2_CID_HUE:
	case V4L2_CID_GAMMA:
	case V4L2_CID_SHARPNESS:
	case V4L2_CID_AUTOGAIN:
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops gw4200_ctrl_ops = {
	.s_ctrl = gw4200_s_ctrl,
};

static struct v4l2_subdev_video_ops gw4200_video_ops = {
	.s_stream	= gw4200_s_stream,
	.g_mbus_config	= gw4200_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops gw4200_subdev_pad_ops = {
	.get_edid	= gw4200_get_edid,
	.enum_mbus_code	= gw4200_enum_mbus_code,
	.get_selection	= gw4200_get_selection,
	.set_selection	= gw4200_set_selection,
	.get_fmt	= gw4200_get_fmt,
	.set_fmt	= gw4200_set_fmt,
};

static struct v4l2_subdev_ops gw4200_subdev_ops = {
	.core	= &gw4200_core_ops,
	.video	= &gw4200_video_ops,
	.pad	= &gw4200_subdev_pad_ops,
};

static void gw4200_otp_id_read(struct i2c_client *client)
{
	struct gw4200_priv *priv = to_gw4200(client);
	int i;

	/* read camera id from ar014x OTP memory */
	gw4200_ar014x_write(client, 0x3054, 0x400);
	gw4200_ar014x_write(client, 0x304a, 0x110);
	usleep_range(25000, 25500); /* wait 25 ms */

	for (i = 0; i < 6; i += 2) {
		/* first 4 bytes are equal on all ar014x */
		priv->id[i]     = gw4200_ar014x_read(client, 0x3800 + i + 4) >> 8;
		priv->id[i + 1] = gw4200_ar014x_read(client, 0x3800 + i + 4) & 0xff;
	}
}

static ssize_t gw4200_otp_id_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gw4200_priv *priv = to_gw4200(client);

	gw4200_otp_id_read(client);

	return snprintf(buf, 32, "%02x:%02x:%02x:%02x:%02x:%02x\n",
			priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);
}

static DEVICE_ATTR(otp_id_gw4200, S_IRUGO, gw4200_otp_id_show, NULL);

static int gw4200_initialize(struct i2c_client *client)
{
	struct gw4200_priv *priv = to_gw4200(client);
	u8 pid = 0;
	int ret = 0;

	gw4200_s_port(client, 1);

	/* check and show model ID */
	reg8_read(client, GW4200_PID, &pid);

	if (pid != GW4200_VERSION_REG) {
		dev_err(&client->dev, "Product ID error %x\n", pid);
		ret = -ENODEV;
		goto err;
	}

#if 0
	/* Program wizard registers */
	gw4200_set_regs(client, gw4200_regs_wizard, ARRAY_SIZE(gw4200_regs_wizard));
	/* Read OTP IDs */
	gw4200_otp_id_read(client);
#endif

	dev_info(&client->dev, "gw4200 PID %x, res %dx%d, OTP_ID %02x:%02x:%02x:%02x:%02x:%02x\n",
		 pid, GW4200_MAX_WIDTH, GW4200_MAX_HEIGHT, priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);
err:
	gw4200_s_port(client, 0);

	return ret;
}

static int gw4200_parse_dt(struct device_node *np, struct gw4200_priv *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(&priv->sd);
	int i;
	struct device_node *endpoint = NULL, *rendpoint = NULL;
	int tmp_addr = 0;

	for (i = 0; ; i++) {
		endpoint = of_graph_get_next_endpoint(np, endpoint);
		if (!endpoint)
			break;

		rendpoint = of_parse_phandle(endpoint, "remote-endpoint", 0);
		if (!rendpoint)
			continue;

		if (!of_property_read_u32(rendpoint, "max9271-addr", &priv->max9271_addr) &&
		    !of_property_read_u32(rendpoint->parent->parent, "reg", &priv->max9286_addr) &&
		    !kstrtouint(strrchr(rendpoint->full_name, '@') + 1, 0, &priv->port))
			break;
	}

	of_node_put(endpoint);

	if (!priv->max9286_addr) {
		dev_err(&client->dev, "deserializer does not present for GW4200\n");
		return -EINVAL;
	}

	gw4200_s_port(client, 1);

	/* setup I2C translator address */
	tmp_addr = client->addr;
	if (priv->max9286_addr) {
		client->addr = priv->max9271_addr;			/* Serializer I2C address */

		reg8_write(client, 0x09, tmp_addr << 1);		/* Sensor translated I2C address */
		reg8_write(client, 0x0A, GW4200_I2C_ADDR << 1);		/* Sensor native I2C address */
		usleep_range(2000, 2500);				/* wait 2ms */
	};
	client->addr = tmp_addr;

	mdelay(10);

	return 0;
}

static int gw4200_probe(struct i2c_client *client,
		       const struct i2c_device_id *did)
{
	struct gw4200_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&priv->sd, client, &gw4200_subdev_ops);
	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	priv->exposure = 0x100;
	priv->gain = 0x100;
	priv->autogain = 1;
	v4l2_ctrl_handler_init(&priv->hdl, 4);
	v4l2_ctrl_new_std(&priv->hdl, &gw4200_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, 0, 16, 1, 7);
	v4l2_ctrl_new_std(&priv->hdl, &gw4200_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 16, 1, 7);
	v4l2_ctrl_new_std(&priv->hdl, &gw4200_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 7, 1, 2);
	v4l2_ctrl_new_std(&priv->hdl, &gw4200_ctrl_ops,
			  V4L2_CID_HUE, 0, 23, 1, 12);
	v4l2_ctrl_new_std(&priv->hdl, &gw4200_ctrl_ops,
			  V4L2_CID_GAMMA, -128, 128, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &gw4200_ctrl_ops,
			  V4L2_CID_SHARPNESS, 0, 10, 1, 3);
	v4l2_ctrl_new_std(&priv->hdl, &gw4200_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, priv->autogain);
	v4l2_ctrl_new_std(&priv->hdl, &gw4200_ctrl_ops,
			  V4L2_CID_GAIN, 0, 0xffff, 1, priv->gain);
	v4l2_ctrl_new_std(&priv->hdl, &gw4200_ctrl_ops,
			  V4L2_CID_EXPOSURE, 0, 0xffff, 1, priv->exposure);
	v4l2_ctrl_new_std(&priv->hdl, &gw4200_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 1);
	v4l2_ctrl_new_std(&priv->hdl, &gw4200_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	priv->sd.ctrl_handler = &priv->hdl;

	ret = priv->hdl.error;
	if (ret)
		goto cleanup;

	v4l2_ctrl_handler_setup(&priv->hdl);

	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->sd.entity.flags |= MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&priv->sd.entity, 1, &priv->pad);
	if (ret < 0)
		goto cleanup;

	ret = gw4200_parse_dt(client->dev.of_node, priv);
	if (ret)
		goto cleanup;

	ret = gw4200_initialize(client);
	if (ret < 0)
		goto cleanup;

	priv->rect.left = 0;
	priv->rect.top = 0;
	priv->rect.width = GW4200_MAX_WIDTH;
	priv->rect.height = GW4200_MAX_HEIGHT;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret)
		goto cleanup;

	if (device_create_file(&client->dev, &dev_attr_otp_id_gw4200) != 0) {
		dev_err(&client->dev, "sysfs otp_id entry creation failed\n");
		goto cleanup;
	}

	priv->init_complete = 1;

	return 0;

cleanup:
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);
#ifdef CONFIG_SOC_CAMERA_GW4200
	v4l_err(client, "failed to probe @ 0x%02x (%s)\n",
		client->addr, client->adapter->name);
#endif
	return ret;
}

static int gw4200_remove(struct i2c_client *client)
{
	struct gw4200_priv *priv = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_otp_id_gw4200);
	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

#ifdef CONFIG_SOC_CAMERA_GW4200
static const struct i2c_device_id gw4200_id[] = {
	{ "gw4200", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, gw4200_id);

static const struct of_device_id gw4200_of_ids[] = {
	{ .compatible = "geosemi,gw4200", },
	{ }
};
MODULE_DEVICE_TABLE(of, gw4200_of_ids);

static struct i2c_driver gw4200_i2c_driver = {
	.driver	= {
		.name		= "gw4200",
		.of_match_table	= gw4200_of_ids,
	},
	.probe		= gw4200_probe,
	.remove		= gw4200_remove,
	.id_table	= gw4200_id,
};

module_i2c_driver(gw4200_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for GW4200");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
#endif
