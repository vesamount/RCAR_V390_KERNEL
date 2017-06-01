/*
 * V4L2 driver for OmniVision OV5647 cameras.
 *
 * Based on Samsung S5K6AAFX SXGA 1/6" 1.3M CMOS Image Sensor driver
 * Copyright (C) 2011 Sylwester Nawrocki <s.nawrocki@samsung.com>
 *
 * Based on Omnivision OV7670 Camera Driver
 * Copyright (C) 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * Copyright (C) 2016, Synopsys, Inc.
 *
 * Copyright (C) 2017 Cogent Embedded, Inc
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-image-sizes.h>
#include <linux/io.h>

#include "ov5647.h"

static bool debug = true;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

#define OV5647_I2C_ADDR 0x6c
#define SENSOR_NAME "ov5647"

#define OV5647_REG_CHIPID_H	0x300A
#define OV5647_REG_CHIPID_L	0x300B

#define REG_TERM 0xfffe
#define VAL_TERM 0xfe
#define REG_DLY  0xffff

/*define the voltage level of control signal*/
#define CSI_STBY_ON		1
#define CSI_STBY_OFF		0
#define CSI_RST_ON		0
#define CSI_RST_OFF		1
#define CSI_PWR_ON		1
#define CSI_PWR_OFF		0
#define CSI_AF_PWR_ON		1
#define CSI_AF_PWR_OFF		0

#define OV5647_ROW_START		0x01
#define OV5647_ROW_START_MIN		0
#define OV5647_ROW_START_MAX		2004
#define OV5647_ROW_START_DEF		54

#define OV5647_COLUMN_START		0x02
#define OV5647_COLUMN_START_MIN		0
#define OV5647_COLUMN_START_MAX		2750
#define OV5647_COLUMN_START_DEF		16

#define OV5647_WINDOW_HEIGHT		0x03
#define OV5647_WINDOW_HEIGHT_MIN	2
#define OV5647_WINDOW_HEIGHT_MAX	2006
#define OV5647_WINDOW_HEIGHT_DEF	1944

#define OV5647_WINDOW_WIDTH		0x04
#define OV5647_WINDOW_WIDTH_MIN		2
#define OV5647_WINDOW_WIDTH_MAX		2752
#define OV5647_WINDOW_WIDTH_DEF		2592

enum power_seq_cmd {
	CSI_SUBDEV_PWR_OFF = 0x00,
	CSI_SUBDEV_PWR_ON = 0x01,
};

struct sensor_format_struct {
	__u8 *desc;
	u32 mbus_code;
	enum v4l2_colorspace colorspace;
	struct regval_list *regs;
	int regs_size;
	int bpp;
};

struct cfg_array {
	struct regval_list *regs;
	int size;
};

struct sensor_win_size {
	int width;
	int height;
	unsigned int hoffset;
	unsigned int voffset;
	unsigned int hts;
	unsigned int vts;
	unsigned int pclk;
	unsigned int mipi_bps;
	unsigned int fps_fixed;
	unsigned int bin_factor;
	unsigned int intg_min;
	unsigned int intg_max;
	void *regs;
	int regs_size;
	int (*set_size)(struct v4l2_subdev *subdev);
};

struct ov5647 {
	struct device			*dev;
	struct v4l2_subdev		subdev;
	struct media_pad		pad;
	struct mutex			lock;
	struct v4l2_mbus_framefmt	format;
	struct sensor_format_struct	*fmt;
	unsigned int			width;
	unsigned int			height;
	unsigned int			capture_mode;
	int				hue;
	struct v4l2_fract		tpf;
	struct sensor_win_size		*current_wins;
};

static inline struct ov5647 *to_state(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct ov5647, subdev);
}

static struct sensor_format_struct sensor_formats[] = {
	{
		.mbus_code	= OV5647_CODE,
		.colorspace	= V4L2_COLORSPACE_JPEG,
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

static int ov5647_write(struct v4l2_subdev *sd, uint16_t reg, uint8_t val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val};
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ret = i2c_master_send(client, data, 3);
	if (ret < 3) {
		printk( "%s: i2c write error, reg: %x, %d\n",
				__func__, reg, ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int ov5647_read(struct v4l2_subdev *sd, uint16_t reg, uint8_t *val)
{
	int ret;
	unsigned char data_w[2] = { reg >> 8, reg & 0xff };
	struct i2c_client *client = v4l2_get_subdevdata(sd);


	ret = i2c_master_send(client, data_w, 2);

	if (ret < 2) {
		printk("%s: i2c read error, reg: %x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	ret = i2c_master_recv(client, val, 1);

	if (ret < 1) {
		printk("%s: i2c read error, reg: %x\n",
				__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int ov5647_write_array(struct v4l2_subdev *subdev,
				struct regval_list *regs, int array_size)
{
	int i = 0;
	int ret = 0;

	if (!regs)
		return -EINVAL;

	while (i < array_size) {
		if (regs->addr == REG_DLY)
			mdelay(regs->data);
		else
			ret = ov5647_write(subdev, regs->addr, regs->data);

		if (ret == -EIO)
			return ret;

		i++;
		regs++;
	}
	return 0;
}

static void ov5647_set_virtual_channel(struct v4l2_subdev *subdev, int channel)
{
#if 0
	u8 channel_id;

	ov5647_read(subdev, 0x4814, &channel_id);
//	channel_id = 0x1e; //override

	channel_id &= ~(3 << 6);
	channel_id |= (channel << 6);
	printk("0x4814 = 0x%02x\n", channel_id);
	ov5647_write(subdev, 0x4814, channel_id);
	ov5647_write(subdev, 0x4801, 0x8f);
#endif
}

void ov5647_stream_on(struct v4l2_subdev *subdev)
{
	ov5647_write(subdev, 0x4202, 0x00);
	ov5647_write(subdev, 0x300D, 0x00);
}

void ov5647_stream_off(struct v4l2_subdev *subdev)
{
	ov5647_write(subdev, 0x4202, 0x0f);
	ov5647_write(subdev, 0x300D, 0x01);
}

static int sensor_s_sw_stby(struct v4l2_subdev *subdev, int on_off)
{
	int ret;
	unsigned char rdval;

	ret = ov5647_read(subdev, 0x0100, &rdval);
	if (ret != 0)
		return ret;

	if (on_off == CSI_STBY_ON)
		ret = ov5647_write(subdev, 0x0100, rdval&0xfe);
	else
		ret = ov5647_write(subdev, 0x0100, rdval|0x01);

	return ret;
}

static int __sensor_init(struct v4l2_subdev *subdev)
{
	int ret;
	unsigned char rdval;

	ret = ov5647_read(subdev, 0x0100, &rdval);
	if (ret != 0)
		return ret;

	ov5647_write(subdev, 0x4800, 0x25);
	ov5647_stream_off(subdev);

	ov5647_write(subdev, 0x100, 0);
	/* reset */
	ov5647_write(subdev, 0x103, 1);
	ov5647_write(subdev, 0x103, 1);
	ov5647_write(subdev, 0x103, 1);
	mdelay(10);

	ret = ov5647_write_array(subdev, ov5647_recommend_settings,
					ARRAY_SIZE(ov5647_recommend_settings));
#if 1
	ret = ov5647_write_array(subdev, ov5647_snap_settings,
					ARRAY_SIZE(ov5647_snap_settings));
#else
	ret = ov5647_write_array(subdev, ov5647_prev_settings,
					ARRAY_SIZE(ov5647_prev_settings));
#endif
	ov5647_set_virtual_channel(subdev, 0);

	ov5647_write(subdev, 0x0100, 0x01);

	ov5647_write(subdev, 0x04800, 0x04);
	ov5647_stream_on(subdev);
	msleep(30);

	return 0;
}

static int sensor_power(struct v4l2_subdev *subdev, int on)
{
	int ret = 0;
	struct ov5647 *ov5647 = to_state(subdev);

	mutex_lock(&ov5647->lock);

	switch (on) {
	case CSI_SUBDEV_PWR_OFF:
		ret = sensor_s_sw_stby(subdev, CSI_STBY_ON);
		if (ret < 0)
			printk("soft stby failed!\n");
		break;
	case CSI_SUBDEV_PWR_ON:
		ret = __sensor_init(subdev);
		if (ret < 0) {
			v4l2_err(subdev, "Camera not available, check power\n");
			break;
		}
		break;
	default:
		return -EINVAL;
	}

	mutex_unlock(&ov5647->lock);

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int sensor_get_register(struct v4l2_subdev *subdev,
			       struct v4l2_dbg_register *reg)
{
	u8 val = 0;
	int ret;

	ret = ov5647_read(subdev, (u16)reg->reg, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = sizeof(u8);

	return ret;
}

static int sensor_set_register(struct v4l2_subdev *subdev,
			       const struct v4l2_dbg_register *reg)
{
	ov5647_write(subdev, (u16)reg->reg, (u8)reg->val);

	return 0;
}
#endif

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.s_power		= sensor_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= sensor_get_register,
	.s_register		= sensor_set_register,
#endif
};

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	if (enable)
		ov5647_stream_on(sd);
	else
		ov5647_stream_off(sd);

	return 0;
}

static int sensor_enum_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= N_FMTS)
		return -EINVAL;

	code->code = OV5647_CODE;

	return 0;
}

static int sensor_try_fmt_internal(struct v4l2_subdev *subdev,
				   struct v4l2_mbus_framefmt *fmt, 
				   struct sensor_format_struct **ret_fmt,
				   struct sensor_win_size **ret_wsize)
{
	int index;

	for (index = 0; index < N_FMTS; index++)
		if (sensor_formats[index].mbus_code == fmt->code)
			break;

	if (index >= N_FMTS)
		return -EINVAL;

	if (ret_fmt != NULL)
		*ret_fmt = sensor_formats + index;

	fmt->field = V4L2_FIELD_NONE;

	return 0;
}

static int sensor_s_fmt(struct v4l2_subdev *subdev,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *fmt)
{
	int ret;
	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize = NULL;
	struct ov5647 *info = to_state(subdev);

	ret = sensor_try_fmt_internal(subdev, &fmt->format,
					&sensor_fmt, &wsize);
	if (ret)
		return ret;

	info->fmt = sensor_fmt;
	info->width = OV5647_WIDTH;
	info->height = OV5647_HEIGHT;

	return 0;
}

static int sensor_g_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *format)
{
	struct ov5647 *info = to_state(sd);
	struct v4l2_mbus_framefmt *mf = &format->format;

	if (format->pad != 0)
		return -EINVAL;

	mf->width = OV5647_WIDTH;
	mf->height = OV5647_HEIGHT;
	mf->code = OV5647_CODE;
	mf->colorspace = info->fmt->colorspace;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int sensor_s_parm(struct v4l2_subdev *subdev,
			 struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct ov5647 *info = to_state(subdev);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (info->tpf.numerator == 0)
		return -EINVAL;

	info->capture_mode = cp->capturemode;

	return 0;
}

static int sensor_g_parm(struct v4l2_subdev *subdev,
			 struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct ov5647 *info = to_state(subdev);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->capturemode = info->capture_mode;

	return 0;
}

static int sensor_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->type = V4L2_MBUS_CSI2;

	return 0;
}

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code	= sensor_enum_fmt,
	.set_fmt	= sensor_s_fmt,
	.get_fmt	= sensor_g_fmt,
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_stream	= sensor_s_stream,
	.s_parm		= sensor_s_parm,
	.g_parm		= sensor_g_parm,
	.g_mbus_config	= sensor_g_mbus_config,
};

static const struct v4l2_subdev_ops subdev_ops = {
	.core		= &sensor_core_ops,
	.video		= &sensor_video_ops,
	.pad		= &sensor_pad_ops,
};

static int ov5647_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char id_h, id_l;
	int ret;

	ret = sensor_power(sd, 1);
	if (ret < 0)
		return ret;

	ret = ov5647_read(sd, OV5647_REG_CHIPID_H, &id_h);
	if (ret < 0)
		return ret;
	ret = ov5647_read(sd, OV5647_REG_CHIPID_L, &id_l);
	if (ret < 0)
		return ret;

	if ((id_h != 0x56) || (id_l != 0x47)) {
		v4l2_info(sd, "Invalid device ID: %02x%02x\n", id_h, id_l);
		return -ENODEV;
	}

	v4l2_info(sd, "OV5647 detected at address 0x%02x\n", client->addr);

	ret = sensor_power(sd, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int ov5647_registered(struct v4l2_subdev *subdev)
{
	return 0;
}

static int ov5647_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *format =
				v4l2_subdev_get_try_format(subdev, fh->pad, 0);
	struct v4l2_rect *crop =
				v4l2_subdev_get_try_crop(subdev, fh->pad, 0);

	crop->left = 0;
	crop->top = 0;
	crop->width = OV5647_WIDTH;
	crop->height = OV5647_HEIGHT;

	format->code = OV5647_CODE;

	format->width = OV5647_WIDTH;
	format->height = OV5647_HEIGHT;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = sensor_formats[0].colorspace;

	return sensor_power(subdev, 1);
}

static int ov5647_close(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	return sensor_power(subdev, 0);
}

static const struct v4l2_subdev_internal_ops ov5647_subdev_internal_ops = {
	.registered = ov5647_registered,
	.open = ov5647_open,
	.close = ov5647_close,
};

static int ov5647_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ov5647 *sensor;
	int ret = 0;
	struct v4l2_subdev *sd;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (sensor == NULL)
		return -ENOMEM;

	mutex_init(&sensor->lock);
	sensor->dev = dev;
	sensor->fmt = &sensor_formats[0];
	sensor->width = OV5647_WIDTH;
	sensor->height = OV5647_HEIGHT;

	sd = &sensor->subdev;
	v4l2_i2c_subdev_init(sd, client, &subdev_ops);
	sensor->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ret = ov5647_detect(sd);
	if (ret < 0) {
		v4l2_err(sd, "OV5647 not found!\n");
		goto out;
	}

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret < 0)
		return ret;

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0)
		media_entity_cleanup(&sd->entity);

out:
	return ret;
}

static int ov5647_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ov5647 *ov5647 = to_state(subdev);

	v4l2_async_unregister_subdev(&ov5647->subdev);
	media_entity_cleanup(&ov5647->subdev.entity);
	v4l2_device_unregister_subdev(subdev);

	return 0;
}

static const struct i2c_device_id ov5647_id[] = {
	{ "ov5647", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov5647_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov5647_of_match[] = {
	{ .compatible = "ovti,ov5647" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ov5647_of_match);
#endif

static struct i2c_driver ov5647_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ov5647_of_match),
		.owner	= THIS_MODULE,
		.name	= "ov5647",
	},
	.probe		= ov5647_probe,
	.remove		= ov5647_remove,
	.id_table	= ov5647_id,
};
module_i2c_driver(ov5647_driver);

MODULE_AUTHOR("Ramiro Oliveira <roliveir@synopsys.com>");
MODULE_DESCRIPTION("A low-level driver for OmniVision ov5647 sensors");
MODULE_LICENSE("GPL");
