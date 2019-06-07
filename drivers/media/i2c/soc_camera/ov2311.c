/*
 * OmniVision ov2311 sensor camera driver
 *
 * Copyright (C) 2015-2019 Cogent Embedded, Inc.
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

#include "max9286.h"
#include "ov2311.h"

#define OV2311_I2C_ADDR			0x60

#define OV2311_PID			0x300a
#define OV2311_VER			0x300b
#define OV2311_REV			0x300c
#define OV2311_VERSION_REG		0x2311

#define OV2311_MEDIA_BUS_FMT		MEDIA_BUS_FMT_Y8_1X8

struct ov2311_priv {
	struct v4l2_subdev		sd;
	struct v4l2_ctrl_handler	hdl;
	struct media_pad		pad;
	struct v4l2_rect		rect;
	int				subsampling;
	int				fps_denominator;
	int				init_complete;
	u8				id[6];
	int				dvp_order;
	/* serializers */
	int				max9286_addr;
	int				max9271_addr;
	int				ti9x4_addr;
	int				ti9x3_addr;
	int				port;
	int				gpio_resetb;
	int				gpio_fsin;
};

static inline struct ov2311_priv *to_ov2311(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov2311_priv, sd);
}

static inline struct v4l2_subdev *ov2311_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov2311_priv, hdl)->sd;
}

static void ov2311_s_port(struct i2c_client *client, int fwd_en)
{
	struct ov2311_priv *priv = to_ov2311(client);
	int tmp_addr;

	if (priv->max9286_addr) {
		tmp_addr = client->addr;
		client->addr = priv->max9286_addr;				/* Deserializer I2C address */
		reg8_write(client, 0x0a, fwd_en ? 0x11 << priv->port : 0);	/* Enable/disable reverse/forward control for this port */
		usleep_range(5000, 5500);					/* wait 5ms */
		client->addr = tmp_addr;
	};
}

static int ov2311_set_regs(struct i2c_client *client,
			    const struct ov2311_reg *regs, int nr_regs)
{
	int i;

	for (i = 0; i < nr_regs; i++) {
		if (regs[i].reg == OV2311_DELAY) {
			mdelay(regs[i].val);
			continue;
		}

		if (reg16_write(client, regs[i].reg, regs[i].val)) {
			usleep_range(100, 150); /* wait 100ns */
			reg16_write(client, regs[i].reg, regs[i].val);
		}
	}

	return 0;
}

static int ov2311_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int ov2311_set_window(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2311_priv *priv = to_ov2311(client);

	dev_dbg(&client->dev, "L=%d T=%d %dx%d\n", priv->rect.left, priv->rect.top, priv->rect.width, priv->rect.height);
#if 0
	/* setup resolution */
	reg16_write(client, 0x3808, priv->rect.width >> 8);
	reg16_write(client, 0x3809, priv->rect.width & 0xff);
	reg16_write(client, 0x380a, priv->rect.height >> 8);
	reg16_write(client, 0x380b, priv->rect.height & 0xff);

	/* horiz isp windowstart */
	reg16_write(client, 0x3810, priv->rect.left >> 8);
	reg16_write(client, 0x3811, priv->rect.left & 0xff);
	reg16_write(client, 0x3812, priv->rect.top >> 8);
	reg16_write(client, 0x3813, priv->rect.top & 0xff);
#endif
	return 0;
};

static int ov2311_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2311_priv *priv = to_ov2311(client);

	if (format->pad)
		return -EINVAL;

	mf->width = priv->rect.width;
	mf->height = priv->rect.height;
	mf->code = OV2311_MEDIA_BUS_FMT;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int ov2311_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;

	mf->code = OV2311_MEDIA_BUS_FMT;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		cfg->try_fmt = *mf;

	return 0;
}

static int ov2311_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = OV2311_MEDIA_BUS_FMT;

	return 0;
}

static int ov2311_get_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2311_priv *priv = to_ov2311(client);

	memcpy(edid->edid, priv->id, 6);

	edid->edid[6] = 0xff;
	edid->edid[7] = client->addr;
	edid->edid[8] = OV2311_VERSION_REG >> 8;
	edid->edid[9] = OV2311_VERSION_REG & 0xff;

	return 0;
}

static int ov2311_set_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_selection *sel)
{
	struct v4l2_rect *rect = &sel->r;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2311_priv *priv = to_ov2311(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE ||
	    sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	rect->left = ALIGN(rect->left, 2);
	rect->top = ALIGN(rect->top, 2);
	rect->width = ALIGN(rect->width, 2);
	rect->height = ALIGN(rect->height, 2);

	if ((rect->left + rect->width > OV2311_MAX_WIDTH) ||
	    (rect->top + rect->height > OV2311_MAX_HEIGHT))
		*rect = priv->rect;

	priv->rect.left = rect->left;
	priv->rect.top = rect->top;
	priv->rect.width = rect->width;
	priv->rect.height = rect->height;

	ov2311_set_window(sd);

	return 0;
}

static int ov2311_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_selection *sel)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2311_priv *priv = to_ov2311(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = OV2311_MAX_WIDTH;
		sel->r.height = OV2311_MAX_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = OV2311_MAX_WIDTH;
		sel->r.height = OV2311_MAX_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP:
		sel->r = priv->rect;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ov2311_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->type = V4L2_MBUS_CSI2;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov2311_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val = 0;

	ret = reg16_read(client, (u16)reg->reg, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = sizeof(u16);

	return 0;
}

static int ov2311_s_register(struct v4l2_subdev *sd,
			      const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return reg16_write(client, (u16)reg->reg, (u8)reg->val);
}
#endif

static struct v4l2_subdev_core_ops ov2311_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov2311_g_register,
	.s_register = ov2311_s_register,
#endif
};

static int ov2311_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ov2311_to_sd(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2311_priv *priv = to_ov2311(client);
	int ret = 0;
	u8 val = 0;

	if (!priv->init_complete)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
	case V4L2_CID_HUE:
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_GAIN:
		reg16_write(client, 0x350A, ctrl->val / 0x3ff); // COARSE: 4.10 format
		reg16_write(client, 0x350B, (ctrl->val % 0x3ff) >> 2); // FINE: 4.10 format
		reg16_write(client, 0x350C, (ctrl->val % 0x3ff) << 6); // FINE: 4.10 format
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		reg16_write(client, 0x3508, ctrl->val / 0xf); // COARSE: 5.4 format
		reg16_write(client, 0x3509, (ctrl->val % 0xf) << 4); // FINE: 5.4 format
		break;
	case V4L2_CID_EXPOSURE:
		reg16_write(client, 0x3501, ctrl->val >> 8);
		reg16_write(client, 0x3502, ctrl->val & 0xff);
		break;
	case V4L2_CID_HFLIP:
		reg16_read(client, 0x3821, &val);
		val &= ~0x04;
		val |= (ctrl->val ? 0x04 : 0);
		reg16_write(client, 0x3821, val);
		break;
	case V4L2_CID_VFLIP:
		reg16_read(client, 0x3820, &val);
		val &= ~0x44;
		val |= (ctrl->val ? 0x44 : 0);
		reg16_write(client, 0x3820, val);
		break;
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		ret = 0;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops ov2311_ctrl_ops = {
	.s_ctrl = ov2311_s_ctrl,
};

static struct v4l2_subdev_video_ops ov2311_video_ops = {
	.s_stream	= ov2311_s_stream,
	.g_mbus_config	= ov2311_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops ov2311_subdev_pad_ops = {
	.get_edid	= ov2311_get_edid,
	.enum_mbus_code	= ov2311_enum_mbus_code,
	.get_selection	= ov2311_get_selection,
	.set_selection	= ov2311_set_selection,
	.get_fmt	= ov2311_get_fmt,
	.set_fmt	= ov2311_set_fmt,
};

static struct v4l2_subdev_ops ov2311_subdev_ops = {
	.core	= &ov2311_core_ops,
	.video	= &ov2311_video_ops,
	.pad	= &ov2311_subdev_pad_ops,
};

static void ov2311_otp_id_read(struct i2c_client *client)
{
	struct ov2311_priv *priv = to_ov2311(client);
	int i;

	reg16_write(client, 0x100, 1);
	reg16_write(client, 0x3d81, 1);
	usleep_range(25000, 25500); /* wait 25 ms */

	for (i = 0; i < 6; i++) {
		/* first 6 bytes are equal on all ov2311 */
		reg16_read(client, 0x7000 + i + 6, &priv->id[i]);
	}

	reg16_write(client, 0x100, 0);
}

static ssize_t ov2311_otp_id_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2311_priv *priv = to_ov2311(client);

	return snprintf(buf, 32, "%02x:%02x:%02x:%02x:%02x:%02x\n",
			priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);
}

static DEVICE_ATTR(otp_id_ov2311, S_IRUGO, ov2311_otp_id_show, NULL);

static int ov2311_initialize(struct i2c_client *client)
{
	struct ov2311_priv *priv = to_ov2311(client);
	u16 pid;
	u8 val = 0, rev = 0;
	int ret = 0;
	int tmp_addr = 0;

	ov2311_s_port(client, 1);

	/* check and show product ID and manufacturer ID */
	reg16_read(client, OV2311_PID, &val);
	pid = val;
	reg16_read(client, OV2311_VER, &val);
	pid = (pid << 8) | val;

	if (pid != OV2311_VERSION_REG) {
		dev_dbg(&client->dev, "Product ID error %x\n", pid);
		ret = -ENODEV;
		goto out;
	}

	tmp_addr = client->addr;
	if (priv->ti9x4_addr) {
		client->addr = priv->ti9x3_addr;			/* Serializer I2C address */
		reg8_write(client, 0x02, 0x13);				/* MIPI 2-lanes */
	}
	client->addr = tmp_addr;

	/* check revision */
	reg16_read(client, OV2311_REV, &rev);
	/* Read OTP IDs */
	ov2311_otp_id_read(client);
	/* Program wizard registers */
	ov2311_set_regs(client, ov2311_regs_wizard_r1c, ARRAY_SIZE(ov2311_regs_wizard_r1c));

	dev_info(&client->dev, "ov2311 PID %x (rev %x), res %dx%d, OTP_ID %02x:%02x:%02x:%02x:%02x:%02x\n",
		 pid, rev, OV2311_MAX_WIDTH, OV2311_MAX_HEIGHT, priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);
out:
	ov2311_s_port(client, 0);

	return ret;
}

static int ov2311_parse_dt(struct device_node *np, struct ov2311_priv *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(&priv->sd);
	int i;
	struct device_node *endpoint = NULL, *rendpoint = NULL;
	int tmp_addr = 0;

	for (i = 0; ; i++) {
		endpoint = of_graph_get_next_endpoint(np, endpoint);
		if (!endpoint)
			break;

		of_node_put(endpoint);

		of_property_read_u32(endpoint, "dvp-order", &priv->dvp_order);

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

	if (!priv->max9286_addr && !priv->ti9x4_addr) {
		dev_err(&client->dev, "deserializer does not present for OV2311\n");
		return -EINVAL;
	}

	ov2311_s_port(client, 1);

	/* setup I2C translator address */
	tmp_addr = client->addr;
	if (priv->max9286_addr) {
		client->addr = priv->max9271_addr;			/* Serializer I2C address */

		reg8_write(client, 0x09, tmp_addr << 1);		/* Sensor translated I2C address */
		reg8_write(client, 0x0A, OV2311_I2C_ADDR << 1);		/* Sensor native I2C address */
		usleep_range(2000, 2500);				/* wait 2ms */
	};

	if (priv->ti9x4_addr) {
		client->addr = priv->ti9x4_addr;			/* Deserializer I2C address */

		reg8_write(client, 0x4c, (priv->port << 4) | (1 << priv->port)); /* Select RX port number */
		usleep_range(2000, 2500);				/* wait 2ms */
		reg8_write(client, 0x65, tmp_addr << 1);		/* Sensor translated I2C address */
		reg8_write(client, 0x5d, OV2311_I2C_ADDR << 1);		/* Sensor native I2C address */
	}
	client->addr = tmp_addr;

	return 0;
}

static int ov2311_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct ov2311_priv *priv;
	struct v4l2_ctrl *ctrl;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&priv->sd, client, &ov2311_subdev_ops);
	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	priv->rect.left = 0;
	priv->rect.top = 0;
	priv->rect.width = OV2311_MAX_WIDTH;
	priv->rect.height = OV2311_MAX_HEIGHT;
	priv->fps_denominator = 30;

	v4l2_ctrl_handler_init(&priv->hdl, 4);
	v4l2_ctrl_new_std(&priv->hdl, &ov2311_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, 0, 0xff, 1, 0x30);
	v4l2_ctrl_new_std(&priv->hdl, &ov2311_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 4, 1, 2);
	v4l2_ctrl_new_std(&priv->hdl, &ov2311_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 0xff, 1, 0xff);
	v4l2_ctrl_new_std(&priv->hdl, &ov2311_ctrl_ops,
			  V4L2_CID_HUE, 0, 255, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov2311_ctrl_ops,
			  V4L2_CID_GAMMA, 0, 0xffff, 1, 0x233);
	v4l2_ctrl_new_std(&priv->hdl, &ov2311_ctrl_ops,
			  V4L2_CID_GAIN, 0, 0x3ff*4, 1, 0x3ff);
	v4l2_ctrl_new_std(&priv->hdl, &ov2311_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 0, 0xf*5, 1, 0xf);
	v4l2_ctrl_new_std(&priv->hdl, &ov2311_ctrl_ops,
			  V4L2_CID_EXPOSURE, 0, 0x580, 1, 0x57c);
	v4l2_ctrl_new_std(&priv->hdl, &ov2311_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov2311_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	ctrl = v4l2_ctrl_new_std(&priv->hdl, &ov2311_ctrl_ops,
			  V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, 1, 32, 1, 9);
	if (ctrl)
		ctrl->flags &= ~V4L2_CTRL_FLAG_READ_ONLY;
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

	ret = ov2311_parse_dt(client->dev.of_node, priv);
	if (ret)
		goto cleanup;

	ret = ov2311_initialize(client);
	if (ret < 0)
		goto cleanup;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret)
		goto cleanup;

	if (device_create_file(&client->dev, &dev_attr_otp_id_ov2311) != 0) {
		dev_err(&client->dev, "sysfs otp_id entry creation failed\n");
		goto cleanup;
	}

	priv->init_complete = 1;

	return 0;

cleanup:
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);
#ifdef CONFIG_SOC_CAMERA_OV2311
	v4l_err(client, "failed to probe @ 0x%02x (%s)\n",
		client->addr, client->adapter->name);
#endif
	return ret;
}

static int ov2311_remove(struct i2c_client *client)
{
	struct ov2311_priv *priv = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_otp_id_ov2311);
	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

#ifdef CONFIG_SOC_CAMERA_OV2311
static const struct i2c_device_id ov2311_id[] = {
	{ "ov2311", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov2311_id);

static const struct of_device_id ov2311_of_ids[] = {
	{ .compatible = "ovti,ov2311", },
	{ }
};
MODULE_DEVICE_TABLE(of, ov2311_of_ids);

static struct i2c_driver ov2311_i2c_driver = {
	.driver	= {
		.name		= "ov2311",
		.of_match_table	= ov2311_of_ids,
	},
	.probe		= ov2311_probe,
	.remove		= ov2311_remove,
	.id_table	= ov2311_id,
};

module_i2c_driver(ov2311_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for OV2311");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
#endif
