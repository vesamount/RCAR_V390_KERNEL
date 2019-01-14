/*
 * OmniVision ov10640 sensor camera driver
 *
 * Copyright (C) 2015-2017 Cogent Embedded, Inc.
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
#include "ov10640.h"

#define OV10640_I2C_ADDR		0x30

#define OV10640_PID			0x300a
#define OV10640_VER			0x300b
#define OV10640_VERSION_REG		0xa640

#define OV10640_MEDIA_BUS_FMT		MEDIA_BUS_FMT_SBGGR12_1X12

struct ov10640_priv {
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

static inline struct ov10640_priv *to_ov10640(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov10640_priv, sd);
}

static inline struct v4l2_subdev *ov10640_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov10640_priv, hdl)->sd;
}

static void ov10640_s_port(struct i2c_client *client, int fwd_en)
{
	struct ov10640_priv *priv = to_ov10640(client);
	int tmp_addr;

	if (priv->max9286_addr) {
		tmp_addr = client->addr;
		client->addr = priv->max9286_addr;				/* Deserializer I2C address */
		reg8_write(client, 0x0a, fwd_en ? 0x11 << priv->port : 0);	/* Enable/disable reverse/forward control for this port */
		usleep_range(5000, 5500);					/* wait 5ms */
		client->addr = tmp_addr;
	};
}

static int ov10640_set_regs(struct i2c_client *client,
			    const struct ov10640_reg *regs, int nr_regs)
{
	int i;

	for (i = 0; i < nr_regs; i++) {
		if (regs[i].reg == OV10640_DELAY) {
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

static int ov10640_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int ov10640_set_window(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10640_priv *priv = to_ov10640(client);

	dev_dbg(&client->dev, "L=%d T=%d %dx%d\n", priv->rect.left, priv->rect.top, priv->rect.width, priv->rect.height);

	/* horiz crop start (reverse) */
	reg16_write(client, 0x3074, (OV10640_MAX_WIDTH - priv->rect.width - priv->rect.left) >> 8);
	reg16_write(client, 0x3075, (OV10640_MAX_WIDTH - priv->rect.width - priv->rect.left) & 0xff);
	/* horiz crop end (reverse) */
	reg16_write(client, 0x3078, (OV10640_MAX_WIDTH - priv->rect.left - 1) >> 8);
	reg16_write(client, 0x3079, (OV10640_MAX_WIDTH - priv->rect.left - 1) & 0xff);
	/* vert crop start */
	reg16_write(client, 0x3076, priv->rect.top >> 8);
	reg16_write(client, 0x3077, priv->rect.top & 0xff);
	/* vert crop end */
	reg16_write(client, 0x307a, (priv->rect.top + priv->rect.height + 1) >> 8);
	reg16_write(client, 0x307b, (priv->rect.top + priv->rect.height + 1) & 0xff);
	/* horiz output */
	reg16_write(client, 0x307c, priv->rect.width >> 8);
	reg16_write(client, 0x307d, priv->rect.width & 0xff);
	/* vert output */
	reg16_write(client, 0x307e, priv->rect.height >> 8);
	reg16_write(client, 0x307f, priv->rect.height & 0xff);

	return 0;
};

static int ov10640_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10640_priv *priv = to_ov10640(client);

	if (format->pad)
		return -EINVAL;

	mf->width = priv->rect.width;
	mf->height = priv->rect.height;
	mf->code = OV10640_MEDIA_BUS_FMT;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int ov10640_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;

	mf->code = OV10640_MEDIA_BUS_FMT;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		cfg->try_fmt = *mf;

	return 0;
}

static int ov10640_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = OV10640_MEDIA_BUS_FMT;

	return 0;
}

static int ov10640_get_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10640_priv *priv = to_ov10640(client);

	memcpy(edid->edid, priv->id, 6);

	edid->edid[6] = 0xff;
	edid->edid[7] = client->addr;
	edid->edid[8] = OV10640_VERSION_REG >> 8;
	edid->edid[9] = OV10640_VERSION_REG & 0xff;

	return 0;
}

static int ov10640_set_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_selection *sel)
{
	struct v4l2_rect *rect = &sel->r;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10640_priv *priv = to_ov10640(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE ||
	    sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	rect->left = ALIGN(rect->left, 2);
	rect->top = ALIGN(rect->top, 2);
	rect->width = ALIGN(rect->width, 2);
	rect->height = ALIGN(rect->height, 2);

	if ((rect->left + rect->width > OV10640_MAX_WIDTH) ||
	    (rect->top + rect->height > OV10640_MAX_HEIGHT))
		*rect = priv->rect;

	priv->rect.left = rect->left;
	priv->rect.top = rect->top;
	priv->rect.width = rect->width;
	priv->rect.height = rect->height;

	ov10640_set_window(sd);

	return 0;
}

static int ov10640_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_selection *sel)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10640_priv *priv = to_ov10640(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = OV10640_MAX_WIDTH;
		sel->r.height = OV10640_MAX_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = OV10640_MAX_WIDTH;
		sel->r.height = OV10640_MAX_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP:
		sel->r = priv->rect;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ov10640_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->type = V4L2_MBUS_CSI2;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov10640_g_register(struct v4l2_subdev *sd,
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

static int ov10640_s_register(struct v4l2_subdev *sd,
			      const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return reg16_write(client, (u16)reg->reg, (u8)reg->val);
}
#endif

static struct v4l2_subdev_core_ops ov10640_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov10640_g_register,
	.s_register = ov10640_s_register,
#endif
};

static int ov10640_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ov10640_to_sd(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10640_priv *priv = to_ov10640(client);
	int ret = -EINVAL;
	u8 val = 0;
	u16 val16 = 0;

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
		reg16_write(client, 0x30EC, ctrl->val);		// L
		reg16_write(client, 0x30EE, ctrl->val);		// S
		reg16_write(client, 0x30F0, ctrl->val);		// VS
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		reg16_read(client, 0x30EB, &val);
		val &= ~(0x3f << 0);		// VS, S, L - Gauss curve
		val |= ((ctrl->val / 2) << 0);	// L
		val |= (ctrl->val << 2);	// S
		val |= ((ctrl->val / 2) << 4);	// VS
		reg16_write(client, 0x30EB, val);
		break;
	case V4L2_CID_EXPOSURE:
		val16 = 0xfff - ctrl->val;

		reg16_write(client, 0x30E6, val16 >> 8);	// L
		reg16_write(client, 0x30E7, val16 & 0xff);	// L

		reg16_write(client, 0x30E8, val16 >> 8);	// S
		reg16_write(client, 0x30E9, val16 & 0xff);	// S

//		reg16_write(client, 0x30EA, val >> 8);	// VS - fractional ...
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		reg16_read(client, 0x30FA, &val);
		val &= ~(0x1 << 6);
		val |= (ctrl->val << 6);
		reg16_write(client, 0x30FA, val);
		break;
	case V4L2_CID_HFLIP:
		reg16_read(client, 0x3128, &val);
		val &= ~(0x1 << 0);
		val |= (ctrl->val << 0);
		reg16_write(client, 0x3128, val);

		reg16_read(client, 0x3291, &val);
		val &= ~(0x1 << 1);
		val |= (ctrl->val << 1);
		reg16_write(client, 0x3291, val);

		reg16_read(client, 0x3090, &val);
		val &= ~(0x1 << 2);
		val |= (ctrl->val << 2);
		ret = reg16_write(client, 0x3090, val);
		break;
	case V4L2_CID_VFLIP:
		reg16_read(client, 0x3128, &val);
		val &= ~(0x1 << 1);
		val |= (ctrl->val << 1);
		reg16_write(client, 0x3128, val);

		reg16_read(client, 0x3291, &val);
		val &= ~(0x1 << 2);
		val |= (ctrl->val << 2);
		reg16_write(client, 0x3291, val);

		reg16_read(client, 0x3090, &val);
		val &= ~(0x1 << 3);
		val |= (ctrl->val << 3);
		ret = reg16_write(client, 0x3090, val);
		break;
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		ret = 0;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops ov10640_ctrl_ops = {
	.s_ctrl = ov10640_s_ctrl,
};

static struct v4l2_subdev_video_ops ov10640_video_ops = {
	.s_stream	= ov10640_s_stream,
	.g_mbus_config	= ov10640_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops ov10640_subdev_pad_ops = {
	.get_edid	= ov10640_get_edid,
	.enum_mbus_code	= ov10640_enum_mbus_code,
	.get_selection	= ov10640_get_selection,
	.set_selection	= ov10640_set_selection,
	.get_fmt	= ov10640_get_fmt,
	.set_fmt	= ov10640_set_fmt,
};

static struct v4l2_subdev_ops ov10640_subdev_ops = {
	.core	= &ov10640_core_ops,
	.video	= &ov10640_video_ops,
	.pad	= &ov10640_subdev_pad_ops,
};

static void ov10640_otp_id_read(struct i2c_client *client)
{
	struct ov10640_priv *priv = to_ov10640(client);
	int i;
	int otp_bank0_allzero = 1;

	reg16_write(client, 0x349C, 1);
	usleep_range(25000, 25500); /* wait 25 ms */

	for (i = 0; i < 6; i++) {
		/* first 6 bytes are equal on all ov10640 */
		reg16_read(client, 0x349e + i + 6, &priv->id[i]);
		if (priv->id[i])
			otp_bank0_allzero = 0;
	}

	if (otp_bank0_allzero) {
		reg16_write(client, 0x3495, 0x41); /* bank#1 */
		reg16_write(client, 0x349C, 1);
		usleep_range(25000, 25500); /* wait 25 ms */

		for (i = 0; i < 6; i++)
			reg16_read(client, 0x34ae + i, &priv->id[i]);
	}
}

static ssize_t ov10640_otp_id_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10640_priv *priv = to_ov10640(client);

	return snprintf(buf, 32, "%02x:%02x:%02x:%02x:%02x:%02x\n",
			priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);
}

static DEVICE_ATTR(otp_id_ov10640, S_IRUGO, ov10640_otp_id_show, NULL);

static int ov10640_initialize(struct i2c_client *client)
{
	struct ov10640_priv *priv = to_ov10640(client);
	u16 pid;
	u8 val = 0;
	int ret = 0;

	ov10640_s_port(client, 1);

	/* check and show product ID and manufacturer ID */
	reg16_read(client, OV10640_PID, &val);
	pid = val;
	reg16_read(client, OV10640_VER, &val);
	pid = (pid << 8) | val;

	if (pid != OV10640_VERSION_REG) {
		dev_err(&client->dev, "Product ID error %x\n", pid);
		ret = -ENODEV;
		goto out;
	}

	/* Read OTP IDs */
	ov10640_otp_id_read(client);
	/* Program wizard registers */
	ov10640_set_regs(client, ov10640_regs_wizard_r1e, ARRAY_SIZE(ov10640_regs_wizard_r1e));

	dev_info(&client->dev, "ov10640 PID %x, res %dx%d, OTP_ID %02x:%02x:%02x:%02x:%02x:%02x\n",
		 pid, OV10640_MAX_WIDTH, OV10640_MAX_HEIGHT, priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);
out:
	ov10640_s_port(client, 0);

	return ret;
}

static int ov10640_parse_dt(struct device_node *np, struct ov10640_priv *priv)
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
		dev_err(&client->dev, "deserializer does not present for OV10640\n");
		return -EINVAL;
	}

	ov10640_s_port(client, 1);

	/* setup I2C translator address */
	tmp_addr = client->addr;
	if (priv->max9286_addr) {
		client->addr = priv->max9271_addr;			/* Serializer I2C address */

		reg8_write(client, 0x09, tmp_addr << 1);		/* Sensor translated I2C address */
		reg8_write(client, 0x0A, OV10640_I2C_ADDR << 1);	/* Sensor native I2C address */
		usleep_range(2000, 2500);				/* wait 2ms */
	};

	if (priv->ti9x4_addr) {
		client->addr = priv->ti9x4_addr;			/* Deserializer I2C address */

		reg8_write(client, 0x4c, (priv->port << 4) | (1 << priv->port)); /* Select RX port number */
		usleep_range(2000, 2500);				/* wait 2ms */
		reg8_write(client, 0x65, tmp_addr << 1);		/* Sensor translated I2C address */
		reg8_write(client, 0x5d, OV10640_I2C_ADDR << 1);	/* Sensor native I2C address */

//		reg8_write(client, 0x6e, 0x9a);				/* GPIO0 - fsin, GPIO1 - resetb */
//		udelay(100);
	}
	client->addr = tmp_addr;

	return 0;
}

static int ov10640_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct ov10640_priv *priv;
	struct v4l2_ctrl *ctrl;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&priv->sd, client, &ov10640_subdev_ops);
	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	priv->rect.left = 0;
	priv->rect.top = 0;
	priv->rect.width = OV10640_MAX_WIDTH;
	priv->rect.height = OV10640_MAX_HEIGHT;
	priv->fps_denominator = 30;

	v4l2_ctrl_handler_init(&priv->hdl, 4);
	v4l2_ctrl_new_std(&priv->hdl, &ov10640_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, 0, 0xff, 1, 0x30);
	v4l2_ctrl_new_std(&priv->hdl, &ov10640_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 4, 1, 2);
	v4l2_ctrl_new_std(&priv->hdl, &ov10640_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 0xff, 1, 0xff);
	v4l2_ctrl_new_std(&priv->hdl, &ov10640_ctrl_ops,
			  V4L2_CID_HUE, 0, 255, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov10640_ctrl_ops,
			  V4L2_CID_GAMMA, 0, 0xffff, 1, 0x233);
	v4l2_ctrl_new_std(&priv->hdl, &ov10640_ctrl_ops,
			  V4L2_CID_GAIN, 0, 0x3f, 1, 0x1);
	v4l2_ctrl_new_std(&priv->hdl, &ov10640_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 0, 3, 1, 1);
	v4l2_ctrl_new_std(&priv->hdl, &ov10640_ctrl_ops,
			  V4L2_CID_EXPOSURE, 0, 0xfff, 1, 0x448);
	v4l2_ctrl_new_std(&priv->hdl, &ov10640_ctrl_ops,
			  V4L2_CID_EXPOSURE_AUTO, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov10640_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 1);
	v4l2_ctrl_new_std(&priv->hdl, &ov10640_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	ctrl = v4l2_ctrl_new_std(&priv->hdl, &ov10640_ctrl_ops,
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

	ret = ov10640_parse_dt(client->dev.of_node, priv);
	if (ret)
		goto cleanup;

	ret = ov10640_initialize(client);
	if (ret < 0)
		goto cleanup;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret)
		goto cleanup;

	if (device_create_file(&client->dev, &dev_attr_otp_id_ov10640) != 0) {
		dev_err(&client->dev, "sysfs otp_id entry creation failed\n");
		goto cleanup;
	}

	priv->init_complete = 1;

	return 0;

cleanup:
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);
#ifdef CONFIG_SOC_CAMERA_OV10640
	v4l_err(client, "failed to probe @ 0x%02x (%s)\n",
		client->addr, client->adapter->name);
#endif
	return ret;
}

static int ov10640_remove(struct i2c_client *client)
{
	struct ov10640_priv *priv = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_otp_id_ov10640);
	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

#ifdef CONFIG_SOC_CAMERA_OV10640
static const struct i2c_device_id ov10640_id[] = {
	{ "ov10640", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov10640_id);

static const struct of_device_id ov10640_of_ids[] = {
	{ .compatible = "ovti,ov10640", },
	{ }
};
MODULE_DEVICE_TABLE(of, ov10640_of_ids);

static struct i2c_driver ov10640_i2c_driver = {
	.driver	= {
		.name		= "ov10640",
		.of_match_table	= ov10640_of_ids,
	},
	.probe		= ov10640_probe,
	.remove		= ov10640_remove,
	.id_table	= ov10640_id,
};

module_i2c_driver(ov10640_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for OV10640");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
#endif
