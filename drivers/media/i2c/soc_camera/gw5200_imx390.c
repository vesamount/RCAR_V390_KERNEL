/*
 * GEO Semiconductor GW5200-IMX390 sensor camera driver
 *
 * Copyright (C) 2019 Cogent Embedded, Inc.
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

#include "gw5200_imx390.h"

static const int gw5200_i2c_addr[] = {0x18};

#define GW5200_PID		0x00
#define GW5200_VERSION_REG	0x30

#define GW5200_MEDIA_BUS_FMT	MEDIA_BUS_FMT_YUYV8_2X8

static void gw5200_otp_id_read(struct i2c_client *client);

struct gw5200_priv {
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
	int				ti9x4_addr;
	int				ti9x3_addr;
	int				port;
	int				gpio_resetb;
	int				gpio_fsin;
};

static inline struct gw5200_priv *to_gw5200(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct gw5200_priv, sd);
}

static void gw5200_s_port(struct i2c_client *client, int fwd_en)
{
	struct gw5200_priv *priv = to_gw5200(client);
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
static int gw5200_set_regs(struct i2c_client *client,
			  const struct gw5200_reg *regs, int nr_regs)
{
	int i;

	for (i = 0; i < nr_regs; i++) {
		if (regs[i].reg == GW5200_DELAY) {
			mdelay(regs[i].val);
			continue;
		}

		reg16_write16(client, regs[i].reg, regs[i].val);
	}

	return 0;
}
#endif

static int gw5200_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int gw5200_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gw5200_priv *priv = to_gw5200(client);

	if (format->pad)
		return -EINVAL;

	mf->width = priv->rect.width;
	mf->height = priv->rect.height;
	mf->code = GW5200_MEDIA_BUS_FMT;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int gw5200_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;

	mf->code = GW5200_MEDIA_BUS_FMT;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		cfg->try_fmt = *mf;

	return 0;
}

static int gw5200_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = GW5200_MEDIA_BUS_FMT;

	return 0;
}

static int gw5200_get_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gw5200_priv *priv = to_gw5200(client);

	gw5200_otp_id_read(client);

	memcpy(edid->edid, priv->id, 6);

	edid->edid[6] = 0xff;
	edid->edid[7] = client->addr;
	edid->edid[8] = GW5200_VERSION_REG >> 8;
	edid->edid[9] = GW5200_VERSION_REG & 0xff;

	return 0;
}

static int gw5200_set_selection(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_selection *sel)
{
	struct v4l2_rect *rect = &sel->r;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gw5200_priv *priv = to_gw5200(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE ||
	    sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	rect->left = ALIGN(rect->left, 2);
	rect->top = ALIGN(rect->top, 2);
	rect->width = ALIGN(rect->width, 2);
	rect->height = ALIGN(rect->height, 2);

	if ((rect->left + rect->width > GW5200_MAX_WIDTH) ||
	    (rect->top + rect->height > GW5200_MAX_HEIGHT))
		*rect = priv->rect;

	priv->rect.left = rect->left;
	priv->rect.top = rect->top;
	priv->rect.width = rect->width;
	priv->rect.height = rect->height;

	return 0;
}

static int gw5200_get_selection(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_selection *sel)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gw5200_priv *priv = to_gw5200(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = GW5200_MAX_WIDTH;
		sel->r.height = GW5200_MAX_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = GW5200_MAX_WIDTH;
		sel->r.height = GW5200_MAX_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP:
		sel->r = priv->rect;
		return 0;
	default:
		return -EINVAL;
	}
}

static int gw5200_g_mbus_config(struct v4l2_subdev *sd,
			       struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->type = V4L2_MBUS_CSI2;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int gw5200_g_register(struct v4l2_subdev *sd,
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

static int gw5200_s_register(struct v4l2_subdev *sd,
			    const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return reg16_write16(client, (u16)reg->reg, (u16)reg->val);
}
#endif

static struct v4l2_subdev_core_ops gw5200_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = gw5200_g_register,
	.s_register = gw5200_s_register,
#endif
};

static int gw5200_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gw5200_priv *priv = to_gw5200(client);
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

static const struct v4l2_ctrl_ops gw5200_ctrl_ops = {
	.s_ctrl = gw5200_s_ctrl,
};

static struct v4l2_subdev_video_ops gw5200_video_ops = {
	.s_stream	= gw5200_s_stream,
	.g_mbus_config	= gw5200_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops gw5200_subdev_pad_ops = {
	.get_edid	= gw5200_get_edid,
	.enum_mbus_code	= gw5200_enum_mbus_code,
	.get_selection	= gw5200_get_selection,
	.set_selection	= gw5200_set_selection,
	.get_fmt	= gw5200_get_fmt,
	.set_fmt	= gw5200_set_fmt,
};

static struct v4l2_subdev_ops gw5200_subdev_ops = {
	.core	= &gw5200_core_ops,
	.video	= &gw5200_video_ops,
	.pad	= &gw5200_subdev_pad_ops,
};

static void gw5200_otp_id_read(struct i2c_client *client)
{
}

static ssize_t gw5200_otp_id_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gw5200_priv *priv = to_gw5200(client);

	gw5200_otp_id_read(client);

	return snprintf(buf, 32, "%02x:%02x:%02x:%02x:%02x:%02x\n",
			priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);
}

static DEVICE_ATTR(otp_id_gw5200, S_IRUGO, gw5200_otp_id_show, NULL);

static int gw5200_initialize(struct i2c_client *client)
{
	struct gw5200_priv *priv = to_gw5200(client);
	u8 pid = 0;
	int ret = 0;
	int tmp_addr;
	int i;

	for (i = 0; i < ARRAY_SIZE(gw5200_i2c_addr); i++) {
		tmp_addr = client->addr;
		if (priv->ti9x4_addr) {
			client->addr = priv->ti9x4_addr;		/* Deserializer I2C address */
			reg8_write(client, 0x5d, gw5200_i2c_addr[i] << 1); /* Sensor native I2C address */
			usleep_range(2000, 2500);			/* wait 2ms */
		}
		if (priv->max9286_addr) {
			client->addr = priv->max9271_addr;		/* Serializer I2C address */
			reg8_write(client, 0x0a, gw5200_i2c_addr[i] << 1); /* Sensor native I2C address */
			usleep_range(2000, 2500);			/* wait 2ms */
		};
		client->addr = tmp_addr;

		/* check model ID */
		reg8_read(client, GW5200_PID, &pid);

		if (pid == GW5200_VERSION_REG)
			break;
	}

	if (pid != GW5200_VERSION_REG) {
		dev_dbg(&client->dev, "Product ID error %x\n", pid);
		ret = -ENODEV;
		goto err;
	}
#if 0
	/* Program wizard registers */
	gw5200_set_regs(client, gw5200_regs_wizard, ARRAY_SIZE(gw5200_regs_wizard));
	/* Read OTP IDs */
	gw5200_otp_id_read(client);
#endif
	dev_info(&client->dev, "gw5200 PID %x, res %dx%d, OTP_ID %02x:%02x:%02x:%02x:%02x:%02x\n",
		 pid, GW5200_MAX_WIDTH, GW5200_MAX_HEIGHT, priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);
err:
	gw5200_s_port(client, 0);
	return ret;
}

static int gw5200_parse_dt(struct device_node *np, struct gw5200_priv *priv)
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

		if (!of_property_read_u32(rendpoint, "ti9x3-addr", &priv->ti9x3_addr) &&
		    !of_property_match_string(rendpoint->parent->parent, "compatible", "ti,ti9x4") &&
		    !of_property_read_u32(rendpoint->parent->parent, "reg", &priv->ti9x4_addr) &&
		    !kstrtouint(strrchr(rendpoint->full_name, '@') + 1, 0, &priv->port))
			break;
	}

	of_node_put(endpoint);

	if (!priv->ti9x4_addr && !priv->max9286_addr) {
		dev_dbg(&client->dev, "deserializer does not present\n");
		return -EINVAL;
	}

	gw5200_s_port(client, 1);

	/* setup I2C translator address */
	tmp_addr = client->addr;
	if (priv->max9286_addr) {
		client->addr = priv->max9271_addr;			/* Serializer I2C address */

		reg8_write(client, 0x09, tmp_addr << 1);		/* Sensor translated I2C address */
		usleep_range(2000, 2500);				/* wait 2ms */
	};
	if (priv->ti9x4_addr) {
		client->addr = priv->ti9x4_addr;			/* Deserializer I2C address */

		reg8_write(client, 0x4c, (priv->port << 4) | (1 << priv->port)); /* Select RX port number */
		usleep_range(2000, 2500);				/* wait 2ms */
		reg8_write(client, 0x65, tmp_addr << 1);		/* Sensor translated I2C address */
	}
	client->addr = tmp_addr;

	mdelay(10);

	return 0;
}

static int gw5200_probe(struct i2c_client *client,
		       const struct i2c_device_id *did)
{
	struct gw5200_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&priv->sd, client, &gw5200_subdev_ops);
	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	priv->exposure = 0x100;
	priv->gain = 0x100;
	priv->autogain = 1;
	v4l2_ctrl_handler_init(&priv->hdl, 4);
	v4l2_ctrl_new_std(&priv->hdl, &gw5200_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, 0, 16, 1, 7);
	v4l2_ctrl_new_std(&priv->hdl, &gw5200_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 16, 1, 7);
	v4l2_ctrl_new_std(&priv->hdl, &gw5200_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 7, 1, 2);
	v4l2_ctrl_new_std(&priv->hdl, &gw5200_ctrl_ops,
			  V4L2_CID_HUE, 0, 23, 1, 12);
	v4l2_ctrl_new_std(&priv->hdl, &gw5200_ctrl_ops,
			  V4L2_CID_GAMMA, -128, 128, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &gw5200_ctrl_ops,
			  V4L2_CID_SHARPNESS, 0, 10, 1, 3);
	v4l2_ctrl_new_std(&priv->hdl, &gw5200_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, priv->autogain);
	v4l2_ctrl_new_std(&priv->hdl, &gw5200_ctrl_ops,
			  V4L2_CID_GAIN, 0, 0xffff, 1, priv->gain);
	v4l2_ctrl_new_std(&priv->hdl, &gw5200_ctrl_ops,
			  V4L2_CID_EXPOSURE, 0, 0xffff, 1, priv->exposure);
	v4l2_ctrl_new_std(&priv->hdl, &gw5200_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 1);
	v4l2_ctrl_new_std(&priv->hdl, &gw5200_ctrl_ops,
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

	ret = gw5200_parse_dt(client->dev.of_node, priv);
	if (ret)
		goto cleanup;

	ret = gw5200_initialize(client);
	if (ret < 0)
		goto cleanup;

	priv->rect.left = 0;
	priv->rect.top = 0;
	priv->rect.width = GW5200_MAX_WIDTH;
	priv->rect.height = GW5200_MAX_HEIGHT;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret)
		goto cleanup;

	if (device_create_file(&client->dev, &dev_attr_otp_id_gw5200) != 0) {
		dev_err(&client->dev, "sysfs otp_id entry creation failed\n");
		goto cleanup;
	}

	priv->init_complete = 1;

	return 0;

cleanup:
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);
#ifdef CONFIG_SOC_CAMERA_GW5200
	v4l_err(client, "failed to probe @ 0x%02x (%s)\n",
		client->addr, client->adapter->name);
#endif
	return ret;
}

static int gw5200_remove(struct i2c_client *client)
{
	struct gw5200_priv *priv = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_otp_id_gw5200);
	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

#ifdef CONFIG_SOC_CAMERA_GW5200
static const struct i2c_device_id gw5200_id[] = {
	{ "gw5200", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, gw5200_id);

static const struct of_device_id gw5200_of_ids[] = {
	{ .compatible = "geosemi,gw5200", },
	{ }
};
MODULE_DEVICE_TABLE(of, gw5200_of_ids);

static struct i2c_driver gw5200_i2c_driver = {
	.driver	= {
		.name		= "gw5200",
		.of_match_table	= gw5200_of_ids,
	},
	.probe		= gw5200_probe,
	.remove		= gw5200_remove,
	.id_table	= gw5200_id,
};

module_i2c_driver(gw5200_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for GW5200");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
#endif
