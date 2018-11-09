/*
 * drivers/media/platform/soc_camera/rcar_csi2.c
 *     This file is the driver for the R-Car MIPI CSI-2 unit.
 *
 * Copyright (C) 2015-2016 Renesas Electronics Corporation
 *
 * This file is based on the drivers/media/platform/soc_camera/sh_mobile_csi2.c
 *
 * Driver for the SH-Mobile MIPI CSI-2 unit
 *
 * Copyright (C) 2010, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/module.h>
#include <linux/sys_soc.h>

#include <media/rcar_csi2.h>
#include <media/soc_camera.h>
#include <media/v4l2-async.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#include <media/v4l2-fwnode.h>

#define DRV_NAME "rcar_csi2"
#define VC_MAX_CHANNEL		4

#define RCAR_CSI2_TREF		0x00
#define RCAR_CSI2_SRST		0x04
#define RCAR_CSI2_PHYCNT	0x08
#define RCAR_CSI2_CHKSUM	0x0C
#define RCAR_CSI2_VCDT		0x10

#define RCAR_CSI2_VCDT2			0x14 /* Channel Data Type Select */
#define RCAR_CSI2_FRDT			0x18 /* Frame Data Type Select */
#define RCAR_CSI2_FLD			0x1C /* Field Detection Control */
#define RCAR_CSI2_ASTBY			0x20 /* Automatic standby control */
#define RCAR_CSI2_LNGDT0		0x28
#define RCAR_CSI2_LNGDT1		0x2C
#define RCAR_CSI2_INTEN			0x30
#define RCAR_CSI2_INTCLOSE		0x34
#define RCAR_CSI2_INTSTATE		0x38
#define RCAR_CSI2_INTERRSTATE		0x3C

#define RCAR_CSI2_SHPDAT		0x40
#define RCAR_CSI2_SHPCNT		0x44

#define RCAR_CSI2_LINKCNT		0x48
#define RCAR_CSI2_LSWAP			0x4C
#define RCAR_CSI2_PHTW			0x50
#define RCAR_CSI2_PHTC			0x58
#define RCAR_CSI2_PHYPLL		0x68

#define RCAR_CSI2_PHEERM		0x74
#define RCAR_CSI2_PHCLM			0x78
#define RCAR_CSI2_PHDLM			0x7C

#define RCAR_CSI2_CSI0CLKFCPR		0x254 /* CSI0CLK Frequency Configuration Preset */
/* CSI0CLK frequency configuration bit */
#define CSI0CLKFREQRANGE(n)		((n & 0x3f) << 16)

#define RCAR_CSI2_PHYCNT_SHUTDOWNZ		(1 << 17)
#define RCAR_CSI2_PHYCNT_RSTZ			(1 << 16)
#define RCAR_CSI2_PHYCNT_ENABLECLK		(1 << 4)
#define RCAR_CSI2_PHYCNT_ENABLE_3		(1 << 3)
#define RCAR_CSI2_PHYCNT_ENABLE_2		(1 << 2)
#define RCAR_CSI2_PHYCNT_ENABLE_1		(1 << 1)
#define RCAR_CSI2_PHYCNT_ENABLE_0		(1 << 0)

#define RCAR_CSI2_VCDT_VCDTN_EN			(1 << 15)
#define RCAR_CSI2_VCDT_SEL_VCN			(1 << 8)
#define RCAR_CSI2_VCDT_SEL_DTN_ON		(1 << 6)
#define RCAR_CSI2_VCDT_SEL_DTN			(1 << 0)

#define RCAR_CSI2_LINKCNT_MONITOR_EN		(1 << 31)
#define RCAR_CSI2_LINKCNT_REG_MONI_PACT_EN	(1 << 25)

#define RCAR_CSI2_LSWAP_L3SEL_PLANE0		(0 << 6)
#define RCAR_CSI2_LSWAP_L3SEL_PLANE1		(1 << 6)
#define RCAR_CSI2_LSWAP_L3SEL_PLANE2		(2 << 6)
#define RCAR_CSI2_LSWAP_L3SEL_PLANE3		(3 << 6)

#define RCAR_CSI2_LSWAP_L2SEL_PLANE0		(0 << 4)
#define RCAR_CSI2_LSWAP_L2SEL_PLANE1		(1 << 4)
#define RCAR_CSI2_LSWAP_L2SEL_PLANE2		(2 << 4)
#define RCAR_CSI2_LSWAP_L2SEL_PLANE3		(3 << 4)

#define RCAR_CSI2_LSWAP_L1SEL_PLANE0		(0 << 2)
#define RCAR_CSI2_LSWAP_L1SEL_PLANE1		(1 << 2)
#define RCAR_CSI2_LSWAP_L1SEL_PLANE2		(2 << 2)
#define RCAR_CSI2_LSWAP_L1SEL_PLANE3		(3 << 2)

#define RCAR_CSI2_LSWAP_L0SEL_PLANE0		(0 << 0)
#define RCAR_CSI2_LSWAP_L0SEL_PLANE1		(1 << 0)
#define RCAR_CSI2_LSWAP_L0SEL_PLANE2		(2 << 0)
#define RCAR_CSI2_LSWAP_L0SEL_PLANE3		(3 << 0)

#define RCAR_CSI2_PHTW_DWEN			(1 << 24)
#define RCAR_CSI2_PHTW_CWEN			(1 << 8)

#define RCAR_CSI2_PHTC_TESTCLR			(1 << 0)

/* interrupt status registers */
#define RCAR_CSI2_INTSTATE_EBD_CH1		(1 << 29)
#define RCAR_CSI2_INTSTATE_LESS_THAN_WC		(1 << 28)
#define RCAR_CSI2_INTSTATE_AFIFO_OF		(1 << 27)
#define RCAR_CSI2_INTSTATE_VD4_START		(1 << 26)
#define RCAR_CSI2_INTSTATE_VD4_END		(1 << 25)
#define RCAR_CSI2_INTSTATE_VD3_START		(1 << 24)
#define RCAR_CSI2_INTSTATE_VD3_END		(1 << 23)
#define RCAR_CSI2_INTSTATE_VD2_START		(1 << 22)
#define RCAR_CSI2_INTSTATE_VD2_END		(1 << 21)
#define RCAR_CSI2_INTSTATE_VD1_START		(1 << 20)
#define RCAR_CSI2_INTSTATE_VD1_END		(1 << 19)
#define RCAR_CSI2_INTSTATE_SHP			(1 << 18)
#define RCAR_CSI2_INTSTATE_FSFE			(1 << 17)
#define RCAR_CSI2_INTSTATE_LNP			(1 << 16)
#define RCAR_CSI2_INTSTATE_CRC_ERR		(1 << 15)
#define RCAR_CSI2_INTSTATE_HD_WC_ZERO		(1 << 14)
#define RCAR_CSI2_INTSTATE_FRM_SEQ_ERR1		(1 << 13)
#define RCAR_CSI2_INTSTATE_FRM_SEQ_ERR2		(1 << 12)
#define RCAR_CSI2_INTSTATE_ECC_ERR		(1 << 11)
#define RCAR_CSI2_INTSTATE_ECC_CRCT_ERR		(1 << 10)
#define RCAR_CSI2_INTSTATE_LPDT_START		(1 << 9)
#define RCAR_CSI2_INTSTATE_LPDT_END		(1 << 8)
#define RCAR_CSI2_INTSTATE_ULPS_START		(1 << 7)
#define RCAR_CSI2_INTSTATE_ULPS_END		(1 << 6)
#define RCAR_CSI2_INTSTATE_RESERVED		(1 << 5)
#define RCAR_CSI2_INTSTATE_ERRSOTHS		(1 << 4)
#define RCAR_CSI2_INTSTATE_ERRSOTSYNCCHS	(1 << 3)
#define RCAR_CSI2_INTSTATE_ERRESC		(1 << 2)
#define RCAR_CSI2_INTSTATE_ERRSYNCESC		(1 << 1)
#define RCAR_CSI2_INTSTATE_ERRCONTROL		(1 << 0)

/* monitoring registers of interrupt error status */
#define RCAR_CSI2_INTSTATE_ECC_ERR		(1 << 11)
#define RCAR_CSI2_INTSTATE_ECC_CRCT_ERR		(1 << 10)
#define RCAR_CSI2_INTSTATE_LPDT_START		(1 << 9)
#define RCAR_CSI2_INTSTATE_LPDT_END		(1 << 8)
#define RCAR_CSI2_INTSTATE_ULPS_START		(1 << 7)
#define RCAR_CSI2_INTSTATE_ULPS_END		(1 << 6)
#define RCAR_CSI2_INTSTATE_RESERVED		(1 << 5)
#define RCAR_CSI2_INTSTATE_ERRSOTHS		(1 << 4)
#define RCAR_CSI2_INTSTATE_ERRSOTSYNCCHS	(1 << 3)
#define RCAR_CSI2_INTSTATE_ERRESC		(1 << 2)
#define RCAR_CSI2_INTSTATE_ERRSYNCESC		(1 << 1)
#define RCAR_CSI2_INTSTATE_ERRCONTROL		(1 << 0)

static const struct soc_device_attribute r8a77980[] = {
	{ .soc_id = "r8a77980" },
	{ }
};

static const struct soc_device_attribute r8a77970[] = {
	{ .soc_id = "r8a77970" },
	{ }
};

static const struct soc_device_attribute r8a7795[] = {
	{ .soc_id = "r8a7795", .revision = "ES2.0" },
	{ }
};

enum chip_id {
	RCAR_GEN3,
	RCAR_GEN2,
};

enum decoder_input_interface {
	DECODER_INPUT_INTERFACE_RGB888,
	DECODER_INPUT_INTERFACE_YCBCR422,
	DECODER_INPUT_INTERFACE_NONE,
};

/**
 * struct rcar_csi2_link_config - Describes rcar_csi2 hardware configuration
 * @input_colorspace:		The input colorspace (RGB, YUV444, YUV422)
 */
struct rcar_csi2_link_config {
	enum decoder_input_interface input_interface;
	unsigned char lanes;
	unsigned long vcdt;
	unsigned long vcdt2;
	unsigned int csi_rate;
};

#define INIT_RCAR_CSI2_LINK_CONFIG(m) \
{	\
	m.input_interface = DECODER_INPUT_INTERFACE_NONE; \
	m.lanes = 0; \
}

struct rcar_csi_irq_counter_log {
	unsigned long crc_err;
};

struct rcar_csi2 {
	struct v4l2_subdev		subdev[4];
	unsigned int			irq;
	unsigned long			mipi_flags;
	void __iomem			*base;
	struct platform_device		*pdev;
	struct rcar_csi2_client_config	*client;
	unsigned long			vcdt;
	unsigned long			vcdt2;

	unsigned int			field;
	unsigned int			code;
	unsigned int			lanes;
	unsigned int			csi_rate;
	spinlock_t			lock;
	atomic_t			use_count;
};

static int dump = 0;
module_param(dump, int, 0644);
MODULE_PARM_DESC(dump, " Dump CSI packets (default: disabled)");

#define RCAR_CSI_80MBPS		0
#define RCAR_CSI_90MBPS		1
#define RCAR_CSI_100MBPS	2
#define RCAR_CSI_110MBPS	3
#define RCAR_CSI_120MBPS	4
#define RCAR_CSI_130MBPS	5
#define RCAR_CSI_140MBPS	6
#define RCAR_CSI_150MBPS	7
#define RCAR_CSI_160MBPS	8
#define RCAR_CSI_170MBPS	9
#define RCAR_CSI_180MBPS	10
#define RCAR_CSI_190MBPS	11
#define RCAR_CSI_205MBPS	12
#define RCAR_CSI_220MBPS	13
#define RCAR_CSI_235MBPS	14
#define RCAR_CSI_250MBPS	15
#define RCAR_CSI_275MBPS	16
#define RCAR_CSI_300MBPS	17
#define RCAR_CSI_325MBPS	18
#define RCAR_CSI_350MBPS	19
#define RCAR_CSI_400MBPS	20
#define RCAR_CSI_450MBPS	21
#define RCAR_CSI_500MBPS	22
#define RCAR_CSI_550MBPS	23
#define RCAR_CSI_600MBPS	24
#define RCAR_CSI_650MBPS	25
#define RCAR_CSI_700MBPS	26
#define RCAR_CSI_750MBPS	27
#define RCAR_CSI_800MBPS	28
#define RCAR_CSI_850MBPS	29
#define RCAR_CSI_900MBPS	30
#define RCAR_CSI_950MBPS	31
#define RCAR_CSI_1000MBPS	32
#define RCAR_CSI_1050MBPS	33
#define RCAR_CSI_1100MBPS	34
#define RCAR_CSI_1150MBPS	35
#define RCAR_CSI_1200MBPS	36
#define RCAR_CSI_1250MBPS	37
#define RCAR_CSI_1300MBPS	38
#define RCAR_CSI_1350MBPS	39
#define RCAR_CSI_1400MBPS	40
#define RCAR_CSI_1450MBPS	41
#define RCAR_CSI_1500MBPS	42
#define RCAR_CSI_NUMRATES	43

#define RCAR_CSI2_PHxM0(i)		(0xf0 + i * 0x08)
#define RCAR_CSI2_PHxM1(i)		(0xf4 + i * 0x08)
#define RCAR_CSI2_PHRM(i)		(0x110 + i * 0x04)
#define RCAR_CSI2_PHCM(i)		(0x120 + i * 0x04)
#define RCAR_CSI2_SERCCNT		0x140
#define RCAR_CSI2_SSERCCNT		0x144
#define RCAR_CSI2_ECCCM			0x148
#define RCAR_CSI2_ECECM			0x14c
#define RCAR_CSI2_CRCECM		0x150
#define RCAR_CSI2_LCNT(i)		(0x160 + i * 0x04)
#define RCAR_CSI2_LCNTM(i)		(0x168 + i * 0x04)
#define RCAR_CSI2_FCNTM			0x170
#define RCAR_CSI2_FCNTM2		0x174
#define RCAR_CSI2_VINSM(i)		(0x190 + i * 0x04)
#define RCAR_CSI2_PHM(i)		(0x1C0 + i * 0x04)

#define RCAR_CSI2_INTSTATE_ALL		0x3FFFFCDD

static void rcar_sci2_debug_show(struct rcar_csi2 *priv)
{
	int i;
	u32 reg0, reg1;

	if (!dump)
		return;

	dev_info(&priv->pdev->dev, "Debug registers:\n");
	printk("FCNTM : 0x%08x\n", ioread32(priv->base + RCAR_CSI2_FCNTM));
	printk("FCNTM2: 0x%08x\n", ioread32(priv->base + RCAR_CSI2_FCNTM2));

	for (i = 0; i < 4; i++) {
		reg0 = ioread32(priv->base + RCAR_CSI2_PHxM0(i));
		reg1 = ioread32(priv->base + RCAR_CSI2_PHxM1(i));

		printk("Packet header %d: dt: 0x%02x, vc: %d, wc: %d, cnt: %d\n",
			i,
			reg0 & 0x3F, (reg0 >> 6) & 0x03, (reg0 >> 8) & 0xffff,
			reg1 & 0xffff);
	}
	for (i = 0; i < 3; i++) {
		reg0 = ioread32(priv->base + RCAR_CSI2_PHRM(i));

		printk("Packet header R %d dt: 0x%02x, vc: %d, wc: %d, ecc: 0x%02x\n",
			i,
			reg0 & 0x3F, (reg0 >> 6) & 0x03, (reg0 >> 8) & 0xffff,
			(reg0 >> 24) & 0xff);
	}
	for (i = 0; i < 2; i++) {
		reg0 = ioread32(priv->base + RCAR_CSI2_PHCM(i));

		printk("Packet header C %d: dt: 0x%02x, vc: %d, wc: %d, cal_parity: 0x%02x\n",
			i,
			reg0 & 0x3F, (reg0 >> 6) & 0x03, (reg0 >> 8) & 0xffff,
			(reg0 >> 24) & 0xff);
	}
	for (i = 0; i < 8; i++) {
		reg0 = ioread32(priv->base + RCAR_CSI2_PHM(i));

		printk("Packet header Monitor %d: dt: 0x%02x, vc: %d, wc: %d, ecc: 0x%02x\n",
			i + 1,
			reg0 & 0x3F, (reg0 >> 6) & 0x03, (reg0 >> 8) & 0xffff,
			(reg0 >> 24) & 0xff);
	}
	for (i = 0; i < 3; i++)
		printk("VINSM%d: 0x%08x\n", i, ioread32(priv->base + RCAR_CSI2_VINSM(i)));
	printk("SERCCNT: %d\n",
		ioread32(priv->base + RCAR_CSI2_SERCCNT));
	printk("SSERCCNT: %d\n",
		ioread32(priv->base + RCAR_CSI2_SSERCCNT));
	printk("ECCCM: %d\n",
		ioread32(priv->base + RCAR_CSI2_ECCCM));
	printk("ECECM: %d\n",
		ioread32(priv->base + RCAR_CSI2_ECECM));
	printk("CRCECM: %d\n",
		ioread32(priv->base + RCAR_CSI2_CRCECM));
	for (i = 0; i < 2; i++)
		printk("LCNT%d: 0x%08x\n", i, ioread32(priv->base + RCAR_CSI2_LCNT(i)));
	for (i = 0; i < 2; i++)
		printk("LCNTM%d: 0x%08x\n", i, ioread32(priv->base + RCAR_CSI2_LCNTM(i)));
}

static int rcar_csi2_set_phy_freq(struct rcar_csi2 *priv)
{
	const uint32_t hs_freq_range_v3m[43] = {
		0x00, 0x00, 0x20, 0x40, 0x02,	/* 80M, 90M, 100M, 110M, 120M */
		0x02, 0x22, 0x42, 0x04, 0x04,	/* 130M, 140M, 150M, 160M, 170M */
		0x24, 0x44, 0x44, 0x06, 0x26,	/* 180M, 190M, 205M, 220M, 235M */
		0x46, 0x08, 0x28, 0x0a, 0x2a,	/* 250M, 270M, 300M, 325M, 350M */
		0x4a, 0x4a, 0x4a, 0x4a, 0x4a,	/* 400M, 450M, 500M, 550M, 600M */
		0x10, 0x30, 0x12, 0x32, 0x52,	/* 650M, 700M, 750M, 800M, 950M */
		0x72, 0x14, 0x34, 0x52, 0x74,	/* 900M, 950M, 1000M, 1050M, 1100M */
		0x16, 0x36, 0x56, 0x76, 0x18,	/* 1150M, 1200M, 1250M, 1300M, 1350M */
		0x38, 0x58, 0x78		/* 1400M, 1450M, 1500M */
	};
	const uint32_t hs_freq_range_m3[43] = {
		0x00, 0x10, 0x20, 0x30, 0x01,  /* 0-4   */
		0x11, 0x21, 0x31, 0x02, 0x12,  /* 5-9   */
		0x22, 0x32, 0x03, 0x13, 0x23,  /* 10-14 */
		0x33, 0x04, 0x14, 0x05, 0x15,  /* 15-19 */
		0x25, 0x06, 0x16, 0x07, 0x17,  /* 20-24 */
		0x08, 0x18, 0x09, 0x19, 0x29,  /* 25-29 */
		0x39, 0x0A, 0x1A, 0x2A, 0x3A,  /* 30-34 */
		0x0B, 0x1B, 0x2B, 0x3B, 0x0C,  /* 35-39 */
		0x1C, 0x2C, 0x3C               /* 40-42 */
	};
	const uint32_t hs_freq_range_h3[43] = {
		0x00, 0x10, 0x20, 0x30, 0x01,  /* 0-4   */
		0x11, 0x21, 0x31, 0x02, 0x12,  /* 5-9   */
		0x22, 0x32, 0x03, 0x13, 0x23,  /* 10-14 */
		0x33, 0x04, 0x14, 0x25, 0x35,  /* 15-19 */
		0x05, 0x26, 0x36, 0x37, 0x07,  /* 20-24 */
		0x18, 0x28, 0x39, 0x09, 0x19,  /* 25-29 */
		0x29, 0x3A, 0x0A, 0x1A, 0x2A,  /* 30-34 */
		0x3B, 0x0B, 0x1B, 0x2B, 0x3C,  /* 35-39 */
		0x0C, 0x1C, 0x2C               /* 40-42 */
	};
	const uint32_t csi2_rate_range[43] = {
		80, 90, 100, 110, 120,		/* 0-4   */
		130, 140, 150, 160, 170,	/* 5-9   */
		180, 190, 205, 220, 235,	/* 10-14 */
		250, 275, 300, 325, 350,	/* 15-19 */
		400, 450, 500, 550, 600,	/* 20-24 */
		650, 700, 750, 800, 850,	/* 25-29 */
		900, 950, 1000, 1050, 1100,	/* 30-34 */
		1150, 1200, 1250, 1300, 1350,	/* 35-39 */
		1400, 1450, 1500		/* 40-42 */
	};
	uint32_t bps_per_lane = RCAR_CSI_190MBPS;

	for (bps_per_lane = 0; bps_per_lane < RCAR_CSI_NUMRATES; bps_per_lane++) {
		if (priv->csi_rate <= csi2_rate_range[bps_per_lane])
			break;
	}

	dev_dbg(&priv->pdev->dev, "bps_per_lane (%d)\n", bps_per_lane);

	if (soc_device_match(r8a77970))
		iowrite32((hs_freq_range_v3m[bps_per_lane] << 16) |
				RCAR_CSI2_PHTW_DWEN | RCAR_CSI2_PHTW_CWEN | 0x44,
				priv->base + RCAR_CSI2_PHTW);
	else if (soc_device_match(r8a7795) || soc_device_match(r8a77980))
		iowrite32(hs_freq_range_h3[bps_per_lane] << 16,
				priv->base + RCAR_CSI2_PHYPLL);
	else
		/* h3 ws1.x is similar to m3 */
		iowrite32(hs_freq_range_m3[bps_per_lane] << 16,
				priv->base + RCAR_CSI2_PHYPLL);
	return 0;
}

static irqreturn_t rcar_csi2_irq(int irq, void *data)
{
	struct rcar_csi2 *priv = data;
	u32 int_status;
	unsigned int handled = 0;

	spin_lock(&priv->lock);

	int_status = ioread32(priv->base + RCAR_CSI2_INTSTATE);
	if (!int_status)
		goto done;

	/* ack interrupts */
	iowrite32(int_status, priv->base + RCAR_CSI2_INTSTATE);
	handled = 1;

done:
	spin_unlock(&priv->lock);

	return IRQ_RETVAL(handled);

}

static void rcar_csi2_hwdeinit(struct rcar_csi2 *priv)
{
	iowrite32(0, priv->base + RCAR_CSI2_PHYCNT);

	/* reset CSI2 hardware */
	iowrite32(0x00000001, priv->base + RCAR_CSI2_SRST);
	udelay(5);
	iowrite32(0x00000000, priv->base + RCAR_CSI2_SRST);
}

static int rcar_csi2_hwinit(struct rcar_csi2 *priv)
{
	int ret;
	__u32 tmp = 0x10; /* Enable MIPI CSI clock lane */

	/* Reflect registers immediately */
	iowrite32(0x00000001, priv->base + RCAR_CSI2_TREF);
	/* reset CSI2 hardware */
	iowrite32(0x00000001, priv->base + RCAR_CSI2_SRST);
	udelay(5);
	iowrite32(0x00000000, priv->base + RCAR_CSI2_SRST);

	iowrite32(0x00000000, priv->base + RCAR_CSI2_PHTC);

	/* setting HS reception frequency */
	{
		switch (priv->lanes) {
		case 1:
			/* First field number setting */
			iowrite32(0x0001000f, priv->base + RCAR_CSI2_FLD);
			tmp |= 0x1;
			break;
		case 2:
			/* First field number setting */
			iowrite32(0x0001000f, priv->base + RCAR_CSI2_FLD);
			tmp |= 0x3;
			break;
		case 3:
			/* First field number setting */
			iowrite32(0x0001000f, priv->base + RCAR_CSI2_FLD);
			tmp |= 0x7;
			break;
		case 4:
			/* First field number setting */
			iowrite32(0x0002000f, priv->base + RCAR_CSI2_FLD);
			tmp |= 0xF;
			break;
		default:
			dev_err(&priv->pdev->dev,
				"ERROR: lanes is invalid (%d)\n",
				priv->lanes);
			return -EINVAL;
		}

		if (soc_device_match(r8a7795) || soc_device_match(r8a77980)) {
			/* Set PHY Test Interface Write Register in R-Car H3(ES2.0) */
			iowrite32(0x01cc01e2, priv->base + RCAR_CSI2_PHTW);
			iowrite32(0x010101e3, priv->base + RCAR_CSI2_PHTW);
			iowrite32(0x010101e4, priv->base + RCAR_CSI2_PHTW);
			iowrite32(0x01100104, priv->base + RCAR_CSI2_PHTW);
			iowrite32(0x01030100, priv->base + RCAR_CSI2_PHTW);
			iowrite32(0x01800107, priv->base + RCAR_CSI2_PHTW);
		}

		/* set PHY frequency */
		ret = rcar_csi2_set_phy_freq(priv);
		if (ret < 0)
			return ret;

		/* Set CSI0CLK Frequency Configuration Preset Register
		 * in R-Car H3(ES2.0)
		 */
		if (soc_device_match(r8a7795) || soc_device_match(r8a77980))
			iowrite32(CSI0CLKFREQRANGE(32), priv->base + RCAR_CSI2_CSI0CLKFCPR);

		/* Enable lanes */
		iowrite32(tmp, priv->base + RCAR_CSI2_PHYCNT);

		iowrite32(tmp | RCAR_CSI2_PHYCNT_SHUTDOWNZ,
					priv->base + RCAR_CSI2_PHYCNT);
		iowrite32(tmp | (RCAR_CSI2_PHYCNT_SHUTDOWNZ |
					RCAR_CSI2_PHYCNT_RSTZ),
					priv->base + RCAR_CSI2_PHYCNT);
	}

	iowrite32(0x00000003, priv->base + RCAR_CSI2_CHKSUM);
	iowrite32(priv->vcdt, priv->base + RCAR_CSI2_VCDT);
	iowrite32(priv->vcdt2, priv->base + RCAR_CSI2_VCDT2);
	iowrite32(0x00010000, priv->base + RCAR_CSI2_FRDT);
	udelay(10);
	iowrite32(0x83000000, priv->base + RCAR_CSI2_LINKCNT);
	iowrite32(0x000000e4, priv->base + RCAR_CSI2_LSWAP);

	dev_dbg(&priv->pdev->dev, "CSI2 VCDT:  0x%x\n",
			 ioread32(priv->base + RCAR_CSI2_VCDT));
	dev_dbg(&priv->pdev->dev, "CSI2 VCDT2: 0x%x\n",
			 ioread32(priv->base + RCAR_CSI2_VCDT2));

	/* wait until video decoder power off */
	msleep(10);
	{
		int timeout = 100;

		/* Read the PHY clock lane monitor register (PHCLM). */
		while (!(ioread32(priv->base + RCAR_CSI2_PHCLM) & 0x01)
			&& timeout) {
			timeout--;
		}
		if (timeout == 0)
			dev_err(&priv->pdev->dev,
				"Timeout of reading the PHY clock lane\n");
		else
			dev_dbg(&priv->pdev->dev,
				"Detected the PHY clock lane\n");

		timeout = 100;

		/* Read the PHY data lane monitor register (PHDLM). */
		while (!(ioread32(priv->base + RCAR_CSI2_PHDLM) & 0x01)
			&& timeout) {
			timeout--;
		}
		if (timeout == 0)
			dev_err(&priv->pdev->dev,
				"Timeout of reading the PHY data lane\n");
		else
			dev_dbg(&priv->pdev->dev,
				"Detected the PHY data lane\n");
	}

	return 0;
}

static int rcar_csi2_s_power(struct v4l2_subdev *sd, int on)
{
	struct rcar_csi2 *priv = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (on) {
		if (atomic_inc_return(&priv->use_count) == 1) {
			pm_runtime_get_sync(&priv->pdev->dev);
			ret = rcar_csi2_hwinit(priv);
			if (ret < 0)
				return ret;
		}
	} else {
		if (atomic_dec_return(&priv->use_count) == 0) {
			rcar_sci2_debug_show(priv);
			rcar_csi2_hwdeinit(priv);
			pm_runtime_put_sync(&priv->pdev->dev);
		}
	}

	return ret;
}

static struct v4l2_subdev_core_ops rcar_csi2_subdev_core_ops = {
	.s_power	= rcar_csi2_s_power,
};

static struct v4l2_subdev_ops rcar_csi2_subdev_ops = {
	.core	= &rcar_csi2_subdev_core_ops,
};

#ifdef CONFIG_OF
static const struct of_device_id rcar_csi2_of_table[] = {
	{ .compatible = "renesas,r8a77980-csi2", .data = (void *)RCAR_GEN3 },
	{ .compatible = "renesas,r8a77970-csi2", .data = (void *)RCAR_GEN3 },
	{ .compatible = "renesas,r8a77965-csi2", .data = (void *)RCAR_GEN3 },
	{ .compatible = "renesas,r8a7796-csi2", .data = (void *)RCAR_GEN3 },
	{ .compatible = "renesas,r8a7795-csi2", .data = (void *)RCAR_GEN3 },
	{ },
};
MODULE_DEVICE_TABLE(of, rcar_csi2_of_table);
#endif

static struct platform_device_id rcar_csi2_id_table[] = {
	{ "r8a77980-csi2",  RCAR_GEN3 },
	{ "r8a77970-csi2",  RCAR_GEN3 },
	{ "r8a77965-csi2",  RCAR_GEN3 },
	{ "r8a7796-csi2",  RCAR_GEN3 },
	{ "r8a7795-csi2",  RCAR_GEN3 },
	{},
};
MODULE_DEVICE_TABLE(platform, rcar_csi2_id_table);

static int rcar_csi2_parse_dt(struct device_node *np,
			struct rcar_csi2_link_config *config)
{
	struct v4l2_fwnode_endpoint bus_cfg;
	struct device_node *endpoint;
	struct device_node *vc_np, *vc_ch;
	const char *str;
	char csi_name[9];
	int ret;
	int i, ch;

	/* Parse the endpoint. */
	endpoint = of_graph_get_next_endpoint(np, NULL);
	if (!endpoint)
		return -EINVAL;

	v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint), &bus_cfg);
	ret = of_property_read_u32(endpoint, "csi-rate", &config->csi_rate);
	if (ret < 0) {
		printk(KERN_ERR "csi-rate not set\n");
		return ret;
	}
	of_node_put(endpoint);

	config->lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;

	vc_np = of_get_child_by_name(np, "virtual,channel");

	config->vcdt = 0x81008000;
	config->vcdt2 = 0x83008200;
	for (i = 0; i < VC_MAX_CHANNEL; i++) {
		sprintf(csi_name, "csi2_vc%d", i);

		vc_ch = of_get_child_by_name(vc_np, csi_name);
		if (!vc_ch)
			continue;
		ret = of_property_read_string(vc_ch, "data,type", &str);
		if (ret < 0)
			return ret;
		ret = of_property_read_u32(vc_ch, "receive,vc", &ch);
		if (ret < 0)
			return ret;

		if (i < 2) {
			if (!strcmp(str, "rgb888"))
				config->vcdt |= (0x24 << (i * 16));
			else if (!strcmp(str, "ycbcr422"))
				config->vcdt |= (0x1e << (i * 16));
			else if (!strcmp(str, "raw8"))
				config->vcdt |= (0x2a << (i * 16));
			else
				config->vcdt |= 0;

			config->vcdt |= (ch << (8 + (i * 16)));
			config->vcdt |= (RCAR_CSI2_VCDT_VCDTN_EN << (i * 16)) |
					(RCAR_CSI2_VCDT_SEL_DTN_ON << (i * 16));
		}
		if (i >= 2) {
			int j = (i - 2);

			if (!strcmp(str, "rgb888"))
				config->vcdt2 |= (0x24 << (j * 16));
			else if (!strcmp(str, "ycbcr422"))
				config->vcdt2 |= (0x1e << (j * 16));
			else if (!strcmp(str, "raw8"))
				config->vcdt2 |= (0x2a << (j * 16));
			else
				config->vcdt2 |= 0;

			config->vcdt2 |= (ch << (8 + (j * 16)));
			config->vcdt2 |= (RCAR_CSI2_VCDT_VCDTN_EN << (j * 16)) |
					(RCAR_CSI2_VCDT_SEL_DTN_ON << (j * 16));
		}
	}

	return 0;
}

static int rcar_csi2_probe(struct platform_device *pdev)
{
	struct resource *res;
	unsigned int irq;
	int ret;
	struct rcar_csi2 *priv;
	/* Platform data specify the PHY, lanes, ECC, CRC */
	struct rcar_csi2_pdata *pdata;
	struct rcar_csi2_link_config link_config;
	int i;

	dev_dbg(&pdev->dev, "CSI2 probed.\n");

	INIT_RCAR_CSI2_LINK_CONFIG(link_config);

	if (pdev->dev.of_node) {
		ret = rcar_csi2_parse_dt(pdev->dev.of_node, &link_config);
		if (ret)
			return ret;

		dev_info(&pdev->dev, "Data lanes %d, link freq %d\n", link_config.lanes, link_config.csi_rate);
	} else {
		pdata = pdev->dev.platform_data;
		if (!pdata)
			return -EINVAL;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(struct rcar_csi2), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	/* Interrupt unused so far */
	irq = platform_get_irq(pdev, 0);

	if (!res || (int)irq <= 0) {
		dev_err(&pdev->dev, "Not enough CSI2 platform resources.\n");
		return -ENODEV;
	}

	priv->irq = irq;

	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	ret = devm_request_irq(&pdev->dev, irq, rcar_csi2_irq, IRQF_SHARED,
			       dev_name(&pdev->dev), priv);
	if (ret)
		return ret;

	priv->pdev = pdev;
	priv->lanes = link_config.lanes;
	priv->vcdt = link_config.vcdt;
	priv->vcdt2 = link_config.vcdt2;
	priv->csi_rate = link_config.csi_rate;
	atomic_set(&priv->use_count, 0);

	platform_set_drvdata(pdev, priv);

	for (i= 0; i < 4; i++) {
		priv->subdev[i].owner = THIS_MODULE;
		priv->subdev[i].dev = &pdev->dev;
		v4l2_subdev_init(&priv->subdev[i], &rcar_csi2_subdev_ops);
		v4l2_set_subdevdata(&priv->subdev[i], priv);

		snprintf(priv->subdev[i].name, V4L2_SUBDEV_NAME_SIZE, "rcar_csi2.%s",
			 dev_name(&pdev->dev));

		ret = v4l2_async_register_subdev(&priv->subdev[i]);
		if (ret < 0)
			return ret;
	}

	spin_lock_init(&priv->lock);

	pm_runtime_enable(&pdev->dev);

	dev_dbg(&pdev->dev, "CSI2 probed.\n");

	return 0;
}

static int rcar_csi2_remove(struct platform_device *pdev)
{
	struct rcar_csi2 *priv = platform_get_drvdata(pdev);
	int i;

	for (i= 0; i < 4; i++)
		v4l2_async_unregister_subdev(&priv->subdev[i]);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rcar_csi2_suspend(struct device *dev)
{
	/* Empty function for now */
	return 0;
}

static int rcar_csi2_resume(struct device *dev)
{
	/* Empty function for now */
	return 0;
}

static SIMPLE_DEV_PM_OPS(rcar_csi2_pm_ops,
			rcar_csi2_suspend, rcar_csi2_resume);
#define DEV_PM_OPS (&rcar_csi2_pm_ops)
#else
#define DEV_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver __refdata rcar_csi2_pdrv = {
	.remove	= rcar_csi2_remove,
	.probe	= rcar_csi2_probe,
	.driver	= {
		.name	= DRV_NAME,
		.pm	= DEV_PM_OPS,
		.of_match_table	= of_match_ptr(rcar_csi2_of_table),
	},
	.id_table	= rcar_csi2_id_table,
};

module_platform_driver(rcar_csi2_pdrv);

MODULE_DESCRIPTION("Renesas R-Car MIPI CSI-2 driver");
MODULE_AUTHOR("Koji Matsuoka <koji.matsuoka.xm@renesas.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:rcar-csi2");
