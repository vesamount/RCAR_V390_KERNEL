/*
 * Dummy sensor camera driver
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

#define MEDIA_BUS_FORMAT MEDIA_BUS_FMT_YUYV8_2X8

struct dummy_priv {
	struct v4l2_subdev		sd;
	struct v4l2_ctrl_handler	hdl;
	struct media_pad		pad;
	struct v4l2_rect		rect;
	int				max_width;
	int				max_height;
};

static inline struct dummy_priv *to_dummy(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct dummy_priv, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct dummy_priv, hdl)->sd;
}

static int dummy_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int dummy_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct dummy_priv *priv = to_dummy(client);

	if (format->pad)
		return -EINVAL;

	mf->width = priv->rect.width;
	mf->height = priv->rect.height;
	mf->code = MEDIA_BUS_FORMAT;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int dummy_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;

	mf->code = MEDIA_BUS_FORMAT;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		cfg->try_fmt = *mf;

	return 0;
}

static int dummy_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FORMAT;

	return 0;
}

static int dummy_set_selection(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_selection *sel)
{
	struct v4l2_rect *rect = &sel->r;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct dummy_priv *priv = to_dummy(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE ||
	    sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	rect->left = ALIGN(rect->left, 2);
	rect->top = ALIGN(rect->top, 2);
	rect->width = ALIGN(rect->width, 2);
	rect->height = ALIGN(rect->height, 2);

	if ((rect->left + rect->width > priv->max_width) ||
	    (rect->top + rect->height > priv->max_height))
		*rect = priv->rect;

	priv->rect.left = rect->left;
	priv->rect.top = rect->top;
	priv->rect.width = rect->width;
	priv->rect.height = rect->height;

	return 0;
}

static int dummy_get_selection(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_selection *sel)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct dummy_priv *priv = to_dummy(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = priv->max_width;
		sel->r.height = priv->max_height;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = priv->max_width;
		sel->r.height = priv->max_height;
		return 0;
	case V4L2_SEL_TGT_CROP:
		sel->r = priv->rect;
		return 0;
	default:
		return -EINVAL;
	}
}

static int dummy_g_mbus_config(struct v4l2_subdev *sd,
			       struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->type = V4L2_MBUS_CSI2;

	return 0;
}

static int dummy_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return -EINVAL;
}

static const struct v4l2_ctrl_ops dummy_ctrl_ops = {
	.s_ctrl = dummy_s_ctrl,
};

static struct v4l2_subdev_video_ops dummy_video_ops = {
	.s_stream	= dummy_s_stream,
	.g_mbus_config	= dummy_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops dummy_subdev_pad_ops = {
	.enum_mbus_code	= dummy_enum_mbus_code,
	.get_selection	= dummy_get_selection,
	.set_selection	= dummy_set_selection,
	.get_fmt	= dummy_get_fmt,
	.set_fmt	= dummy_set_fmt,
};

static struct v4l2_subdev_ops dummy_subdev_ops = {
	.video	= &dummy_video_ops,
	.pad	= &dummy_subdev_pad_ops,
};

static int dummy_initialize(struct i2c_client *client)
{
	struct dummy_priv *priv = to_dummy(client);

	dev_info(&client->dev, "Dummy sensor res %dx%d\n", priv->max_width, priv->max_height);

	return 0;
}

static int dummy_parse_dt(struct device_node *np, struct dummy_priv *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(&priv->sd);
	int err;

	err = of_property_read_u32(np, "dummy,width", &priv->max_width);
	if (err) {
		dev_err(&client->dev, "dummy,width must be defined\n");
		goto out;
	}

	err = of_property_read_u32(np, "dummy,height", &priv->max_height);
	if (err) {
		dev_err(&client->dev, "dummy,height must be defined\n");
		goto out;
	}

out:
	return err;
}

static int dummy_probe(struct i2c_client *client,
		       const struct i2c_device_id *did)
{
	struct dummy_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&priv->sd, client, &dummy_subdev_ops);
	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	v4l2_ctrl_handler_init(&priv->hdl, 4);
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

	ret = dummy_parse_dt(client->dev.of_node, priv);
	if (ret)
		goto cleanup;

	ret = dummy_initialize(client);
	if (ret < 0)
		goto cleanup;

	priv->rect.left = 0;
	priv->rect.top = 0;
	priv->rect.width = priv->max_width;
	priv->rect.height = priv->max_height;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret)
		goto cleanup;

	return 0;

cleanup:
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);
	v4l_err(client, "failed to probe @ 0x%02x (%s)\n",
		client->addr, client->adapter->name);
	return ret;
}

static int dummy_remove(struct i2c_client *client)
{
	struct dummy_priv *priv = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

static const struct i2c_device_id dummy_id[] = {
	{ "dummy-camera", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, dummy_id);

static const struct of_device_id dummy_of_ids[] = {
	{ .compatible = "dummy-camera", },
	{ }
};
MODULE_DEVICE_TABLE(of, dummy_of_ids);

static struct i2c_driver dummy_i2c_driver = {
	.driver	= {
		.name		= "dummy-camera",
		.of_match_table	= dummy_of_ids,
	},
	.probe		= dummy_probe,
	.remove		= dummy_remove,
	.id_table	= dummy_id,
};
module_i2c_driver(dummy_i2c_driver);

MODULE_DESCRIPTION("Dummy SoC camera driver");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
