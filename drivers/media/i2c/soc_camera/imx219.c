/*
 * V4L2 driver for Sony IMX219 cameras.
 *
 * Based on Samsung S5K6AAFX SXGA 1/6" 1.3M CMOS Image Sensor driver
 * Copyright (C) 2011 Sylwester Nawrocki <s.nawrocki@samsung.com>
 *
 * Based on Omnivision OV7670 Camera Driver
 * Copyright (C) 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * Based on Omnivision OV5647 image sensor driver
 * Copyright (C) 2016, Synopsys, Inc.
 *
 * Copyright (C) 2017-2018 Cogent Embedded, Inc
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

#define IMX219_REG_CHIPID_H	0x0
#define IMX219_REG_CHIPID_L	0x1

#define REG_DLY  0xffff

#define CSI_STBY_ON		1
#define CSI_STBY_OFF		0

/* #define TEST_PATTERN */

struct regval_list {
	u16 addr;
	u8 data;
};

enum power_seq_cmd {
	CSI_SUBDEV_PWR_OFF = 0x00,
	CSI_SUBDEV_PWR_ON = 0x01,
};

struct sensor_format_struct {
	__u8 *desc;
	u32 mbus_code;
	enum v4l2_colorspace colorspace;
};

struct cfg_array {
	struct regval_list *regs;
	int size;
};

struct sensor_win_size {
	int width;
	int height;
	void *regs;
	int regs_size;
};

struct imx219 {
	struct device			*dev;
	struct v4l2_subdev		subdev;
	struct media_pad		pad;
	struct mutex			lock;
	struct v4l2_mbus_framefmt	format;
	struct sensor_format_struct	*fmt;
	unsigned int			width;
	unsigned int			height;
	unsigned int			capture_mode;
	struct v4l2_fract		tpf;
	struct sensor_win_size		*current_wins;
};

static inline struct imx219 *to_state(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct imx219, subdev);
}

static struct sensor_format_struct sensor_formats[] = {
	{
		.mbus_code	= MEDIA_BUS_FMT_SBGGR8_1X8,
		.colorspace	= V4L2_COLORSPACE_SRGB,
	},
};

/*
Used the following sources for register data:
Copyright (C) 2014, Andrew Chew <achew@nvidia.com>
https://chromium.googlesource.com/chromiumos/third_party/kernel/+/factory-ryu-6486.14.B-chromeos-3.14/drivers/media/i2c/soc_camera/imx219.c

Copyright (C) 2013 Broadcom Corporation
https://android.googlesource.com/kernel/bcm/+/android-bcm-tetra-3.10-lollipop-wear-release/drivers/media/video/imx219.c

Chomoly (looks like Allwinner corporation made this one)
https://github.com/allwinner-zh/linux-3.4-sunxi/blob/master/drivers/media/video/sunxi-vfe/device/imx219.c

https://github.com/rellimmot/Sony-IMX219-Raspberry-Pi-V2-CMOS

Copyright (c) 2017, Raspberry Pi Foundation
Copyright (c) 2017, Dave Stevenson
https//github.com/6by9/raspiraw

Register data was manually tuned and tweaked for use with Renesas
RCar CSI driver.
*/

/* unlock vendor registers */
static struct regval_list sensor_unlock_regs[] = {
	{0x30EB, 0x05},
	{0x30EB, 0x0C},
	{0x300A, 0xFF},
	{0x300B, 0xFF},
	{0x30EB, 0x05},
	{0x30EB, 0x09},
	{REG_DLY, 30},
};

/* CIS tuning */
static struct regval_list cis_tuning_regs[] = {
	/* magic */
	{0x455E, 0x00},
	{0x471E, 0x4B},
	{0x4767, 0x0F},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47B4, 0x14},
	{0x4713, 0x30},
	{0x478B, 0x10},
	{0x478F, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0E},
	{0x479B, 0x0E},
};

static struct regval_list sensor_hxga_regs[] = {
	/* 0x114: 03 = 4 lanes, 01 = 2 lanes */
	{0x0114, 0x01},
	/* manual phy control */
	{0x0128, 0x01},
	/* EXCK_FREQ */
	{0x012A, 0x18},
	{0x012B, 0x00},
	/* gain */
	{0x0157, 0xe8},
	{0x0158, 0x01},
	{0x0159, 0x00},
	/* integration time */
	{0x015a, 0x02},
	{0x015b, 0x31},
	/* frame length */
	{0x0160, 0x0f},
	{0x0161, 0xe0},
	/* line length */
	{0x0162, 0x0f},
	{0x0163, 0xE8},
	/* x addr start */
	{0x0164, 0x00},
	{0x0165, 0x00},
	/* x addr end */
	{0x0166, 0x0C},
	{0x0167, 0xCF},
	/* y addr start */
	{0x0168, 0x00},
	{0x0169, 0x00},
	/* y addr end */
	{0x016A, 0x09},
	{0x016B, 0x9F},
	/* Output X size */
	{0x016C, 0x0C},
	{0x016D, 0xD0},
	/* Output Y size */
	{0x016E, 0x09},
	{0x016F, 0xA0},
	/* pixel increment */
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x00},
	{0x0175, 0x00},
	/* pixell data format */
	{0x018C, 0x08},
	{0x018D, 0x08},
	/* pix clk div, */
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	/* PLL1 */
	{0x0306, 0x00},
	{0x0307, 0x39},
	{0x0309, 0x08},
	{0x030B, 0x01},
	/* PLL2 */
	{0x030C, 0x00},
	{0x030D, 0x72},
};

static struct regval_list sensor_1080p_regs[] = {
	/* 0x114: 03 = 4 lanes, 01 = 2 lanes */
	{0x0114, 0x01},
	/* manual phy control */
	{0x0128, 0x01},
	/* EXCK_FREQ */
	{0x012A, 0x18},
	{0x012B, 0x00},
	/* gain */
	{0x0157, 0xe8},
	{0x0158, 0x01},
	{0x0159, 0x00},
	/* coarse integration time */
	{0x015a, 0x05},
	{0x015b, 0x3f},
	{0x0160, 0x0A},
	{0x0161, 0x2F},
	{0x0162, 0x0D},
	{0x0163, 0x78},
	{0x0164, 0x02},
	{0x0165, 0xA8},
	{0x0166, 0x0A},
	{0x0167, 0x27},
	{0x0168, 0x02},
	{0x0169, 0xB4},
	{0x016A, 0x06},
	{0x016B, 0xEB},
	{0x016C, 0x07},
	{0x016D, 0x80},
	{0x016E, 0x04},
	{0x016F, 0x38},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x00},
	{0x0175, 0x00},
	/* pixell data format */
	{0x018C, 0x08},
	{0x018D, 0x08},
	/* pix clk div, */
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	/* PLL1 */
	{0x0306, 0x00},
	{0x0307, 0x39},
	{0x0309, 0x08},
	{0x030B, 0x01},
	/* PLL2 */
	{0x030C, 0x00},
	{0x030D, 0x72},
};

static struct regval_list sensor_720p_regs[] = {
	/* 0x114: 03 = 4 lanes, 01 = 2 lanes */
	{0x0114, 0x01},
	{0x0128, 0x01},
	/* EXCK_FREQ */
	{0x012A, 0x18},
	{0x012B, 0x00},
	/* gain */
	{0x0157, 0xc8},
	{0x0158, 0x01},
	{0x0159, 0x00},
	/* coarse integration time */
	{0x015a, 0x01},
	{0x015b, 0x7f},
	{0x0160, 0x02},
	{0x0161, 0x39},
	{0x0162, 0x0d},
	{0x0163, 0xe7},
	{0x0164, 0x01},
	{0x0165, 0x68},
	{0x0166, 0x0b},
	{0x0167, 0x67},
	{0x0168, 0x02},
	{0x0169, 0x00},
	{0x016A, 0x07},
	{0x016B, 0x9f},
	{0x016C, 0x05},
	{0x016D, 0x00},
	{0x016E, 0x02},
	{0x016F, 0xd0},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0172, 0x03},
	{0x0174, 0x03},
	{0x0175, 0x03},
	/* pixell data format */
	{0x018C, 0x08},
	{0x018D, 0x08},
	/* pix clk div, */
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	/* PLL1 */
	{0x0306, 0x00},
	{0x0307, 0x39},
	/* div2 */
	{0x0309, 0x08},
	{0x030B, 0x01},
	/* PLL2 */
	{0x030C, 0x00},
	{0x030D, 0x72},
};

static struct regval_list sensor_480p_regs[] = {
	/* 0x114: 03 = 4 lanes, 01 = 2 lanes */
	{0x0114, 0x01},
	{0x0128, 0x01},
	/* EXCK_FREQ */
	{0x012A, 0x18},
	{0x012B, 0x00},
	/* gain */
	{0x0157, 0xe8},
	{0x0158, 0x01},
	{0x0159, 0x00},
	/* coarse integration time */
	{0x015a, 0x01},
	{0x015b, 0x2f},
	{0x0160, 0x02},
	{0x0161, 0x39},
	{0x0162, 0x0d},
	{0x0163, 0xe7},
	/* x start */
	{0x0164, 0x03},
	{0x0165, 0xe8},
	/* x end */
	{0x0166, 0x08},
	{0x0167, 0xe7},
	/* y start */
	{0x0168, 0x02},
	{0x0169, 0xf0},
	/* y end */
	{0x016A, 0x06},
	{0x016B, 0xaf},
	{0x016C, 0x02},
	{0x016D, 0x80},
	{0x016E, 0x01},
	{0x016F, 0xe0},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0172, 0x03},
	{0x0174, 0x03},
	{0x0175, 0x03},
	/* pixell data format */
	{0x018C, 0x08},
	{0x018D, 0x08},
	/* pix clk div, */
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	/* PLL1 */
	{0x0306, 0x00},
	{0x0307, 0x39},
	/* div2 */
	{0x0309, 0x08},
	{0x030B, 0x01},
	/* PLL2 */
	{0x030C, 0x00},
	{0x030D, 0x72},
};

static struct regval_list sensor_240p_regs[] = {
	/* 0x114: 03 = 4 lanes, 01 = 2 lanes */
	{0x0114, 0x01},
	{0x0128, 0x01},
	/* EXCK_FREQ */
	{0x012A, 0x18},
	{0x012B, 0x00},
	/* gain */
	{0x0157, 0xd4},
	{0x0158, 0x01},
	{0x0159, 0x00},
	/* coarse integration time */
	{0x015a, 0x01},
	{0x015b, 0x30},
	/* frame time size */
	{0x0160, 0x02},
	{0x0161, 0x39},
	/* line time size */
	{0x0162, 0x0d},
	{0x0163, 0xE7},
	/* x start */
	{0x0164, 0x03},
	{0x0165, 0xe8},
	/* x end */
	{0x0166, 0x06},
	{0x0167, 0x67},
	/* y start */
	{0x0168, 0x03},
	{0x0169, 0xde},
	/* y end */
	{0x016A, 0x05},
	{0x016B, 0xbe},
	/* x width */
	{0x016C, 0x01},
	{0x016D, 0x40},
	/* x height */
	{0x016E, 0x00},
	{0x016F, 0xf0},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0172, 0x03},
	{0x0174, 0x03},
	{0x0175, 0x03},
	/* pixell data format */
	{0x018C, 0x08},
	{0x018D, 0x08},
	/* pix clk div, */
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	/* PLL1 */
	{0x0306, 0x00},
	{0x0307, 0x39},
	/* div2 */
	{0x0309, 0x08},
	{0x030B, 0x01},
	/* PLL2 */
	{0x030C, 0x00},
	{0x030D, 0x72},
};

#define IMX219_DEFAULT_WIDTH	3296
#define IMX219_DEFAULT_HEIGHT	2464

static struct sensor_win_size sensor_win_sizes[] = {
	{
	/* 3296x2464 */
		.width		= 3296,
		.height		= 2464,
		.regs		= sensor_hxga_regs,
		.regs_size	= ARRAY_SIZE(sensor_hxga_regs),
	},
	/* 1920x1080 */
	{
		.width		= 1920,
		.height		= 1080,
		.regs       = sensor_1080p_regs,
		.regs_size  = ARRAY_SIZE(sensor_1080p_regs),
	},
	/* 1280x720 */
	{
		.width      = 1280,
		.height     = 720,
		.regs       = sensor_720p_regs,
		.regs_size  = ARRAY_SIZE(sensor_720p_regs),
	},
	/* 640x480 */
	{
		.width      = 640,
		.height     = 480,
		.regs       = sensor_480p_regs,
		.regs_size  = ARRAY_SIZE(sensor_480p_regs),
	},
	/* 320x240 */
	{
		.width      = 320,
		.height     = 240,
		.regs       = sensor_240p_regs,
		.regs_size  = ARRAY_SIZE(sensor_240p_regs),
	},
};

#define N_FMTS		ARRAY_SIZE(sensor_formats)
#define N_WIN_SIZES	ARRAY_SIZE(sensor_win_sizes)

static int imx219_write(struct v4l2_subdev *sd, uint16_t reg, uint8_t val)
{
	int ret;
	unsigned char data[3] = {reg >> 8, reg & 0xff, val};
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ret = i2c_master_send(client, data, 3);
	if (ret < 3) {
		v4l2_err(sd, "%s: i2c write error, reg: %x, %d\n",
			 __func__, reg, ret);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static int imx219_read(struct v4l2_subdev *sd, uint16_t reg, uint8_t *val)
{
	int ret;
	unsigned char data_w[2] = { reg >> 8, reg & 0xff };
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ret = i2c_master_send(client, data_w, 2);

	if (ret < 2) {
		v4l2_err(sd, "%s: i2c read error, reg: %x\n",
			 __func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	ret = i2c_master_recv(client, val, 1);

	if (ret < 1) {
		v4l2_err(sd, "%s: i2c read error, reg: %x\n",
			 __func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int imx219_write_array(struct v4l2_subdev *subdev,
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
			ret = imx219_write(subdev, regs->addr, regs->data);

		if (ret == -EIO) {
			v4l2_err(subdev, "register write error at %d, %04x\n",
				 i, regs->addr);
			return ret;
		}

		i++;
		regs++;
	}
	return 0;
}

static int sensor_s_sw_stby(struct v4l2_subdev *subdev, int on_off)
{
	int ret;
	unsigned char rdval;

	ret = imx219_read(subdev, 0x0100, &rdval);
	if (ret != 0)
		return ret;

	if (on_off == CSI_STBY_ON)
		ret = imx219_write(subdev, 0x0100, rdval & 0xfe);
	else
		ret = imx219_write(subdev, 0x0100, rdval | 0x01);

	msleep(30);
	return ret;
}

static int sensor_power(struct v4l2_subdev *subdev, int on)
{
	int ret = 0;
	struct imx219 *imx219 = to_state(subdev);

	mutex_lock(&imx219->lock);

	switch (on) {
	case CSI_SUBDEV_PWR_OFF:
		ret = sensor_s_sw_stby(subdev, CSI_STBY_ON);
		if (ret < 0)
			v4l2_err(subdev, "soft stby failed!\n");
		break;
	case CSI_SUBDEV_PWR_ON:
		ret = sensor_s_sw_stby(subdev, CSI_STBY_OFF);
		if (ret) {
			/* soft reset */
			imx219_write(subdev, 0x103, 1);
			msleep(120);
			ret = sensor_s_sw_stby(subdev, CSI_STBY_OFF);
		}
		if (ret < 0) {
			v4l2_err(subdev,
				 "Camera not available, check power\n");
			break;
		}
		break;
	default:
		return -EINVAL;
	}

	mutex_unlock(&imx219->lock);

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int sensor_get_register(struct v4l2_subdev *subdev,
			       struct v4l2_dbg_register *reg)
{
	u8 val = 0;
	int ret;

	ret = imx219_read(subdev, (u16)reg->reg, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = sizeof(u8);

	return ret;
}

static int sensor_set_register(struct v4l2_subdev *subdev,
			       const struct v4l2_dbg_register *reg)
{
	imx219_write(subdev, (u16)reg->reg, (u8)reg->val);

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

#ifdef DUMP_REGS
static void sensor_dump_regs(struct v4l2_subdev *subdev)
{
	u8 val;

	imx219_read(subdev, 0x18, &val);
	pr_info("FRM_CNT %02x\n", val);
	imx219_read(subdev, 0x19, &val);
	pr_info("PX_ORDER %02x\n", val);
	imx219_read(subdev, 0x1a, &val);
	pr_info("DT_PEDESTAL1 %02x\n", val);
	imx219_read(subdev, 0x1b, &val);
	pr_info("DT_PEDESTAL0 %02x\n", val);
	imx219_read(subdev, 0x104, &val);
	pr_info("corrupted frame status %02x\n", val);
	imx219_read(subdev, 0x111, &val);
	pr_info("CSI_SIG_MODE %02x\n", val);
	imx219_read(subdev, 0x114, &val);
	pr_info("CSI_LANE_MODE %02x\n", val);
	imx219_read(subdev, 0x140, &val);
	pr_info("TEMPERATURE_VAL %02x\n", val);
	imx219_read(subdev, 0x142, &val);
	pr_info("READOUT_V_CNT1 %02x\n", val);
	imx219_read(subdev, 0x143, &val);
	pr_info("READOUT_V_CNT0 %02x\n", val);
	imx219_read(subdev, 0x150, &val);
	pr_info("FRAME_BANK_STATUS %02x\n", val);
	imx219_read(subdev, 0x151, &val);
	pr_info("FRAME_BANK_FRM_COUNT %02x\n", val);
}
#else
#define sensor_dump_regs(x)
#endif
static int sensor_enum_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= N_FMTS)
		return -EINVAL;

	code->code = sensor_formats[code->index].mbus_code;
	return 0;
}

static int sensor_try_fmt_internal(struct v4l2_subdev *subdev,
				   struct v4l2_mbus_framefmt *fmt,
				   struct sensor_format_struct **ret_fmt,
				   struct sensor_win_size **ret_wsize)
{
	int index;
	struct sensor_win_size *wsize;
	struct imx219 *imx219 = to_state(subdev);

	for (index = 0; index < N_FMTS; index++)
		if (sensor_formats[index].mbus_code == fmt->code)
			break;

	if (index >= N_FMTS)
		return -EINVAL;

	if (ret_fmt)
		*ret_fmt = sensor_formats + index;

	fmt->field = V4L2_FIELD_NONE;
	for (wsize = sensor_win_sizes;
		wsize < sensor_win_sizes + N_WIN_SIZES;
		wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)
			break;
	if (wsize >= sensor_win_sizes + N_WIN_SIZES)
		wsize--;
	if (ret_wsize)
		*ret_wsize = wsize;
	fmt->width = wsize->width;
	fmt->height = wsize->height;
	imx219->current_wins = wsize;

	return 0;
}

static int sensor_s_fmt(struct v4l2_subdev *subdev,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *fmt)
{
	int ret;
	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize = NULL;
	struct imx219 *info = to_state(subdev);

	ret = sensor_try_fmt_internal(subdev, &fmt->format,
				      &sensor_fmt, &wsize);
	if (ret)
		return ret;

	info->fmt = sensor_fmt;
	if (wsize->regs) {
		/* putting sensor to sleep */
		imx219_write(subdev, 0x100, 0);
		msleep(30);
		ret = imx219_write_array(subdev, sensor_unlock_regs,
					 ARRAY_SIZE(sensor_hxga_regs));
		if (ret < 0)
			return ret;
		ret = imx219_write_array(subdev,
					 wsize->regs,
					 wsize->regs_size);
		if (ret)
			return ret;
		ret = imx219_write_array(subdev, cis_tuning_regs,
					 ARRAY_SIZE(cis_tuning_regs));
		if (ret)
			return ret;
#ifdef TEST_PATTERN
		ret = imx219_write(subdev, 0x0600, 0x00);
		ret |= imx219_write(subdev, 0x0601, 0x02);
		ret |= imx219_write(subdev, 0x0620, 0x00);
		ret |= imx219_write(subdev, 0x0621, 0x00);
		ret |= imx219_write(subdev, 0x0622, 0x00);
		ret |= imx219_write(subdev, 0x0623, 0x00);
		ret |= imx219_write(subdev, 0x0624,
				    (wsize->width >> 8) & 0xff);
		ret |= imx219_write(subdev, 0x0625,
				    wsize->width & 0xff);
		ret |= imx219_write(subdev, 0x0626,
				    (wsize->height >> 8) & 0xff);
		ret |= imx219_write(subdev, 0x0627,
				    wsize->height & 0xff);
		if (ret) {
			v4l2_err(subdev, "%s: i2c write error\n",
				 __func__);
			return -EIO;
		}
#endif
		/* putting sensor out of sleep */
		imx219_write(subdev, 0x100, 1);
		msleep(30);
		sensor_dump_regs(sd);
	}

	return 0;
}

static int sensor_g_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *format)
{
	struct imx219 *info = to_state(sd);
	struct v4l2_mbus_framefmt *mf = &format->format;

	if (format->pad != 0)
		return -EINVAL;

	mf->width = info->current_wins->width;
	mf->height = info->current_wins->height;
	mf->code = info->fmt->mbus_code;
	mf->colorspace = info->fmt->colorspace;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int sensor_s_parm(struct v4l2_subdev *subdev,
			 struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct imx219 *info = to_state(subdev);

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
	struct imx219 *info = to_state(subdev);

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
	cfg->flags = V4L2_MBUS_CSI2_2_LANE;
	cfg->flags |= V4L2_MBUS_CSI2_CHANNEL_0;
	cfg->flags |= V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->flags |= V4L2_MBUS_MASTER;
	cfg->type = V4L2_MBUS_CSI2;

	return 0;
}

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code	= sensor_enum_fmt,
	.set_fmt	= sensor_s_fmt,
	.get_fmt	= sensor_g_fmt,
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_parm		= sensor_s_parm,
	.g_parm		= sensor_g_parm,
	.g_mbus_config	= sensor_g_mbus_config,
};

static const struct v4l2_subdev_ops subdev_ops = {
	.core		= &sensor_core_ops,
	.video		= &sensor_video_ops,
	.pad		= &sensor_pad_ops,
};

static int imx219_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char id_h, id_l;
	int ret;

	ret = sensor_power(sd, 1);
	if (ret < 0)
		return ret;
	msleep(30);

	ret = imx219_read(sd, IMX219_REG_CHIPID_H, &id_h);
	if (ret < 0)
		return ret;
	ret = imx219_read(sd, IMX219_REG_CHIPID_L, &id_l);
	if (ret < 0)
		return ret;

	if (id_h != 0x02 || id_l != 0x19) {
		v4l2_info(sd, "Invalid device ID: %02x%02x\n", id_h, id_l);
		return -ENODEV;
	}

	v4l2_info(sd, "IMX219 detected at address 0x%02x\n", client->addr);

	ret = sensor_power(sd, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int imx219_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *format =
				v4l2_subdev_get_try_format(subdev, fh->pad, 0);
	struct v4l2_rect *crop =
				v4l2_subdev_get_try_crop(subdev, fh->pad, 0);

	crop->left = 0;
	crop->top = 0;
	crop->width = IMX219_DEFAULT_WIDTH;
	crop->height = IMX219_DEFAULT_HEIGHT;

	format->code = MEDIA_BUS_FMT_SBGGR8_1X8;

	format->width = IMX219_DEFAULT_WIDTH;
	format->height = IMX219_DEFAULT_HEIGHT;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = sensor_formats[0].colorspace;

	return sensor_power(subdev, 1);
}

static int imx219_close(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	return sensor_power(subdev, 0);
}

static const struct v4l2_subdev_internal_ops imx219_subdev_internal_ops = {
	.open = imx219_open,
	.close = imx219_close,
};

static int imx219_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct imx219 *sensor;
	int ret = 0, i;
	struct v4l2_subdev *sd;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	mutex_init(&sensor->lock);
	sensor->dev = dev;
	sensor->fmt = &sensor_formats[0];
	sensor->width = sensor_win_sizes[0].width;
	sensor->height = sensor_win_sizes[0].height;
	sensor->current_wins = &sensor_win_sizes[0];

	sd = &sensor->subdev;
	v4l2_i2c_subdev_init(sd, client, &subdev_ops);
	sensor->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ret = imx219_detect(sd);
	if (ret < 0) {
		v4l2_err(sd, "IMX219 not found!\n");
		goto out;
	}
	/* soft reset sequence */
	for (i = 0; i < 3; i++) {
		imx219_write(sd, 0x103, 1);
		mdelay(30);
	}
	msleep(60);

	/* putting sensor to sleep */
	imx219_write(sd, 0x100, 0);
	msleep(30);
	ret = imx219_write_array(sd, sensor_unlock_regs,
				 ARRAY_SIZE(sensor_hxga_regs));
	if (ret < 0)
		return ret;
	ret = imx219_write_array(sd, sensor_win_sizes[0].regs,
				 sensor_win_sizes[0].regs_size);
	if (ret < 0)
		return ret;
	ret = imx219_write_array(sd, cis_tuning_regs,
				 ARRAY_SIZE(cis_tuning_regs));
	if (ret < 0)
		return ret;
	/* getting sensor out of sleep */
	imx219_write(sd, 0x100, 1);
	msleep(30);
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret < 0)
		return ret;

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0)
		media_entity_cleanup(&sd->entity);

	/* putting sensor back to sleep to save power */
	imx219_write(sd, 0x100, 0);
out:
	return ret;
}

static int imx219_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct imx219 *imx219 = to_state(subdev);

	v4l2_async_unregister_subdev(&imx219->subdev);
	media_entity_cleanup(&imx219->subdev.entity);
	v4l2_device_unregister_subdev(subdev);

	return 0;
}

static const struct i2c_device_id imx219_id[] = {
	{ "imx219", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, imx219_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id imx219_of_match[] = {
	{ .compatible = "sony,imx219" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, imx219_of_match);
#endif

static struct i2c_driver imx219_driver = {
	.driver = {
		.of_match_table = of_match_ptr(imx219_of_match),
		.owner	= THIS_MODULE,
		.name	= "imx219",
	},
	.probe		= imx219_probe,
	.remove		= imx219_remove,
	.id_table	= imx219_id,
};
module_i2c_driver(imx219_driver);

MODULE_AUTHOR("Sergey Lapin <sergey.lapin@cogentembedded.com>");
MODULE_DESCRIPTION("A low-level driver for Sony imx219 sensors");
MODULE_LICENSE("GPL");
