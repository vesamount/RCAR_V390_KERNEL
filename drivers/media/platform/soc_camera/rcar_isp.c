/*
 * SoC-camera host driver for Renesas R-Car ISP unit
 *
 * Copyright (C) 2018 Renesas Electronics Corporation
 * Copyright (C) 2018 Cogent Embedded
 *
 * Based on V4L2 Driver for R-Car VIN interface "rca_vin.c"
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

//#define DEBUG

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/list.h>

#include <media/soc_camera.h>
#include <media/drv-intf/soc_mediabus.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/rcar_csi2.h>

#include "soc_scale_crop.h"

#define DRV_NAME "rcar_isp"

/* Register offsets for R-Car ISP Interface */
/* Basic Control */
#define ISPVCR_REG		0x0000
#define ISPFIFOCTRL_REG		0x0004
#define ISPINPUTSEL0_REG	0x0008
#define ISPSTART_REG		0x0014
#define ISPINT_STATUS_REG	0x0040
#define ISPERR0_STATUS_REG	0x0044
#define ISPERR1_STATUS_REG	0x0048
#define ISPINT_CLEAR_REG	0x0050
#define ISPERR0_CLEAR_REG	0x0054
#define ISPERR1_CLEAR_REG	0x0058
#define ISPINT_ENABLE_REG	0x0060
#define ISPERR0_ENABLE_REG	0x0064
#define ISPERR1_ENABLE_REG	0x0068
#define ISP_PADDING_CTRL_REG	0x00C0

/* Register offsets for R-Car ISP Core */
/* Pipeline Frontend */
#define PIPEFE_GFSMR_REG	0x0020

/* Interrupts */
#define INT_FSM_REG		0x0084
#define INT_FSS_REG		0x0088
#define INT_FSC_REG		0x008C
#define INT_FEM_REG		0x0098
#define INT_FES_REG		0x009C
#define INT_FEC_REG		0x00A0
#define INT_FEL0_REG		0x00A4
#define INT_FEL1_REG		0x00A8
#define INT_STM_REG		0x00B0
#define INT_STS_REG		0x00B4
#define INT_STC_REG		0x00B8

/* Input Ports (Port n uses VC=n) */
#define INP_HCSTART0_REG(n)	(0x010C + (n << 6))
#define INP_HCSIZE0_REG(n)	(0x0110 + (n << 6))
#define INP_HCSTART1_REG(n)	(0x0114 + (n << 6))
#define INP_HCSIZE1_REG(n)	(0x0118 + (n << 6))
#define INP_VCSTART_REG(n)	(0x0120 + (n << 6))
#define INP_VCSIZE_REG(n)	(0x0124 + (n << 6))
#define INP_MR_REG		0x01F0
#define INP_MS_REG		0x01F4

#define INP_MS(n)		(1 << (n << 3))

/* OUT_BUFFERS */
#define OBUF_BV_REG(n)		(0x0C00 + (n << 5))
#define OBUF_BA_REG(n)		(0x0C04 + (n << 5))
#define OBUF_LO_REG(n)		(0x0C08 + (n << 5))
#define OBUF_AW_REG(n)		(0x0C0C + (n << 5))
#define OBUF_AH_REG(n)		(0x0C10 + (n << 5))
#define OBUF_HM_REG(n)		(0x0C14 + (n << 5))
#define OBUF_ST_REG(n)		(0x0C18 + (n << 5))

/* Stream Crossbar */
#define SCB_SELECT_REG(n)	(0x03E0 + (n << 2))

/* Multichannel Frontend */
#define MCFE_FIFOC_REG		0x0400
#define MCFE_SCM_REG		0x0414
#define MCFE_OWL_REG		0x041C
#define MCFE_SLM0_REG		0x0420
#define MCFE_SLM4_REG		0x0424
#define MCFE_SLCFGV0_REG	0x0458
#define MCFE_SLCFGV4_REG	0x045C
#define MCFE_SLINP_REG(n)	(0x04E0 + (n << 2))
#define MCFE_SLOUTBUF1_REG(n)	(0x0500 + (n << 2))
#define MCFE_SLOUTBUF2_REG(n)	(0x0520 + (n << 2))
#define MCFE_INM_REG		0x0540
#define MCFE_INAL_REG		0x0544
#define MCFE_INDW_REG		0x054C
#define MCFE_INSV_REG		0x0550

#define MCFE_SCM_STOPPED	(0 << 0)
#define MCFE_SCM_MANUAL		(1 << 0)
#define MCFE_SCM_1STREAM	(2 << 0)

#define MCFE_SCM_SLOT(n)	(n << 16)

#define MCFE_SLM0_PASSTHR(n)	(1 << (n << 3))

/* Top */
#define TOP_AW_REG		0xE010
#define TOP_AH_REG		0xE014
#define TOP_RGGB_REG		0xE018

#define TOP_RGGB_RGrGbB		(0 << 0)
#define TOP_RGGB_GrRBGb		(1 << 0)
#define TOP_RGGB_GbBRGr		(2 << 0)
#define TOP_RGGB_BGbGrR		(3 << 0)

#define TOP_CFA_RGGB		(0 << 8)

/* Pipeline */
#define PIPE_CHS_REG		0xE020
#define PIPE_FSCHS_REG		0xE02C
#define PIPE_CBM_REG		0xE030
#define PIPE_BP_REG		0xE040

#define PIPE_BP_INF		(1 << 2)
#define PIPE_BP_DEMOSAIC	(1 << 19)
#define PIPE_BP_OUTF		(1 << 20)

/* Input Formatter */
#define INF_MI_REG		0xE140

#define INF_MI_LINEAR		(0 << 0)
#define INF_MI_8BITS		(0 << 16)
#define INF_MI_MSB		(1 << 24)

/* Output Formatter */
#define OUTF_BP_REG		0xE600
#define OUTF_MODE_REG		0xE608

#define OUTF_BP_RGB2YUV		(1 << 9)
#define OUTF_BP_RGB2YUV_SP	(1 << 14)

/* Out format MUX for AXI */
#define AXIOUT_MS_REG		0xE740

#define AXIOUT_MS_YUV_Y12	(4 << 0)
#define AXIOUT_MS_YUV_UV8	(7 << 0)
#define AXIOUT_MS_LUV_UV8	(12 << 0)
#define AXIOUT_MS_RGB888	(15 << 0)
#define AXIOUT_MS_RAW8		(24 << 0)
#define AXIOUT_MS_RAW16		(27 << 0)

#define AXIOUT_MSB_ALIGN	(1 << 7)

enum chip_id {
	RCAR_GEN3,
	RCAR_V3M,
	RCAR_V3H,
};

enum csi2_ch {
	RCAR_CSI_CH_NONE = -1,
	RCAR_CSI40,
	RCAR_CSI41,
};

enum gen3_isp_ch {
	RCAR_ISP_CH_NONE = -1,
	RCAR_ISP_CH_0,
	RCAR_ISP_CH_1,
};

enum virtual_ch {
	RCAR_VIRTUAL_NONE = -1,
	RCAR_VIRTUAL_CH0,
	RCAR_VIRTUAL_CH1,
	RCAR_VIRTUAL_CH2,
	RCAR_VIRTUAL_CH3,
};

enum rcar_isp_state {
	STOPPED,
	RUNNING,
};

struct isp_gen3_virtual_sel {
	enum csi2_ch			csi2_ch;
	enum virtual_ch			vc;
};

struct rcar_isp_async_client {
	struct v4l2_async_subdev	*sensor;
	struct v4l2_async_notifier	notifier;
	struct platform_device		*pdev;
};

struct soc_of_info {
	struct soc_camera_async_subdev	sasd;
	struct rcar_isp_async_client	sasc;
	struct v4l2_async_subdev	*subdev;
};

struct rcar_isp_interface {
	void __iomem			*base;
};

struct rcar_isp_priv {
	struct device			*dev;
	void __iomem			*base;
	struct rcar_isp_interface	interface;
	spinlock_t			lock;
	int				sequence;
	/* State of the ISP module in capturing mode */
	enum rcar_isp_state		state;
	struct soc_camera_host		ici;
	struct list_head		capture;
#define MAX_BUFFER_NUM			1
	struct vb2_v4l2_buffer		*queue_buf[MAX_BUFFER_NUM];
	enum v4l2_field			field;
	unsigned int			vb_count;
	unsigned int			nr_hw_slots;
	enum chip_id			chip;
	unsigned int			max_width;
	unsigned int			max_height;
	enum csi2_ch			csi_ch;
	enum virtual_ch			vc;
	bool				csi_sync;
	bool				deser_sync;

	struct rcar_isp_async_client	*async_client;
	/* Asynchronous CSI2 linking */
	struct v4l2_subdev		*csi2_sd;
	/* Asynchronous Deserializer linking */
	struct v4l2_subdev		*deser_sd;
	/* ISP channel */
	unsigned int			index;
};

struct rcar_isp_buffer {
	struct vb2_v4l2_buffer		vb;
	struct list_head		list;
};

#define to_buf_list(vb2_buffer)	(&container_of(vb2_buffer, struct rcar_isp_buffer, vb)->list)

struct rcar_isp_cam {
	/* ISP offsets within the camera output, before the ISP scaler */
	unsigned int			isp_left;
	unsigned int			isp_top;
	/* Client output, as seen by the ISP */
	unsigned int			width;
	unsigned int			height;
	unsigned int			raw_bpp;
	/* User window from S_FMT */
	unsigned int			out_width;
	unsigned int			out_height;
	unsigned int			out_bpp;
	/*
	 * User window from S_SELECTION / G_SELECTION, produced by client cropping and
	 * scaling, ISP scaling and ISP cropping, mapped back onto the client
	 * input window
	 */
	struct v4l2_rect		subrect;
	/* Camera cropping rectangle */
	struct v4l2_rect		rect;
	const struct soc_mbus_pixelfmt	*extra_fmt;
};

static void rcar_isp_open(struct rcar_isp_priv *priv)
{
	int timeout;
	u32 val;
	int inport = 0;

	/* ISP version */
	val = ioread32(priv->interface.base + ISPVCR_REG);
//	printk("ISP version =0x%x\n", val);
	/* ISP Input CSI FIFO enable */
	iowrite32(0x4, priv->interface.base + ISPFIFOCTRL_REG);
	/* ISP Input select CSI */
	iowrite32(0x800, priv->interface.base + ISPINPUTSEL0_REG);
	/* ISP control start */
	iowrite32(0xffff, priv->interface.base + ISPSTART_REG);

	/* V3H CSI stream filter needs following */
	if (priv->chip == RCAR_V3H) {
		// select CSI2_41 channel for Cogent ECU
//		iowrite32(0x800 | BIT(31), priv->interface.base + ISPINPUTSEL0_REG);

		iowrite32(0x02, priv->interface.base + 0x11b0);
		iowrite32(0x01, priv->interface.base + 0x3000);
		iowrite32(0xac, priv->interface.base + 0x3008);
	}

	/* PipelineIP: immediate update mode */
	iowrite32(0, priv->base + PIPE_CBM_REG);

	/* PipelineFE: reset/freeze all state machines (NOTE: does NOT reset ISP to defaults) */
	iowrite32(1, priv->base + PIPEFE_GFSMR_REG);
	udelay(10);
	/* PipelineFE: unfreeze all state machines */
	iowrite32(0, priv->base + PIPEFE_GFSMR_REG);

	/* INP: safe stop */
	iowrite32(0, priv->base + INP_MR_REG);
	/* INP: poll status */
	for (timeout = 0; timeout < 100; timeout++) {
		val = ioread32(priv->base + INP_MS_REG);
		if (!(val & (INP_MS(inport))))
			break;
		udelay(1000);
	}
	if (timeout >= 100)
		pr_err("%s timeout input port stop\n", __func__);

	/* MCFE: one FIFO on each input/output channel */
	iowrite32(0, priv->base + MCFE_FIFOC_REG);

	/* DMA/FIFO watermarks setup */
	iowrite32(0x70004, priv->base + 0x0590);
	iowrite32(0x70004, priv->base + 0x0598);
	iowrite32(0x70004, priv->base + 0x05a4);
	iowrite32(0x70004, priv->base + 0x05ac);
	iowrite32(0x2800140, priv->base + 0x0404);
	iowrite32(0x2800140, priv->base + 0x0408);
	iowrite32(0x2800140, priv->base + 0x040c);
	iowrite32(0x2800140, priv->base + 0x0410);
	iowrite32(0x7000e, priv->base + 0x05c0);
	iowrite32(0x72f681d9, priv->base + 0x05c4);
	iowrite32(1, priv->base + 0x05c8);
	iowrite32(0x800080, priv->base + 0x05cc);
	iowrite32(0x400080, priv->base + 0x05d0);

	/* Invalidate Slot cfg */
	iowrite32(0, priv->base + MCFE_SLCFGV0_REG);
	iowrite32(0, priv->base + MCFE_SLCFGV4_REG);

	/* Pipeline: bypass all modules */
	iowrite32(0xf93ffffc, priv->base + PIPE_BP_REG);
	/* Pipeline: source select for RAW/VTPG */
	iowrite32(0xe4, priv->base + PIPE_CHS_REG);
	/* Pipeline: source select for frame stitch */
	iowrite32(0xe4, priv->base + PIPE_FSCHS_REG);

	/* PipelineIP: global update mode (during vertical blanking) */
	iowrite32(3, priv->base + PIPE_CBM_REG);
	/* Pipeline: tag# */
	iowrite32(0, priv->base + 0xe060);
	/* Pipeline: context# */
	iowrite32(0, priv->base + 0xe064);

	/* FAULTS_CFG: mask all errors */
	iowrite32(0xffffffff, priv->base + 0x1000);
	iowrite32(0xffffffff, priv->base + 0x1004);
	iowrite32(0xffffffff, priv->base + 0x1008);
	iowrite32(0xffffffff, priv->base + 0x100C);
	iowrite32(0xffffffff, priv->base + 0x1010);
	iowrite32(0xffffffff, priv->base + 0x1014);
	iowrite32(0xffffffff, priv->base + 0x1018);
	iowrite32(0xffffffff, priv->base + 0x101C);
	iowrite32(0xffffffff, priv->base + 0x1020);
	iowrite32(0xffffffff, priv->base + 0x1024);
	iowrite32(0xffffffff, priv->base + 0x1028);
	iowrite32(0xffffffff, priv->base + 0x102C);
	iowrite32(0xffffffff, priv->base + 0x1030);
}

static void rcar_isp_close(struct rcar_isp_priv *priv)
{
	/* Pipeline FE: reset/freeze all state machines (NOTE: does NOT reset ISP to defaults) */
	iowrite32(1, priv->base + PIPEFE_GFSMR_REG);
	/* ISP control stop */
	iowrite32(0, priv->interface.base + 0x14);
}

static void rcar_isp_start(struct rcar_isp_priv *priv)
{
	int timeout;
	u32 val;
	int slot = 0;
	int inport = 0;

	/* MCFE: disable scheduler, set slots IDLE state */
	iowrite32(0xf << 16, priv->base + MCFE_SCM_REG);
	/* INP: safe start */
	iowrite32(0x1, priv->base + INP_MR_REG);
	/* INP: poll status */
	for (timeout = 0; timeout < 100; timeout++) {
		val = ioread32(priv->base + INP_MS_REG);
		if (val & (INP_MS(inport)))
			break;
		udelay(1000);
	}
	if (timeout >= 100)
		pr_err("%s timeout input port start\n", __func__);
	/* MCFE: enable scheduler, start slot#0 */
	iowrite32(MCFE_SCM_1STREAM | MCFE_SCM_SLOT(slot), priv->base + MCFE_SCM_REG);

	for (timeout = 0; timeout < 100; timeout++) {
		val = ioread32(priv->base + MCFE_OWL_REG);
		if (val != 0x3f3f3f3f)
			break;
		udelay(1000);
	}
	if (timeout >= 100)
		pr_err("%s timeout out_buffer start\n", __func__);

	/* disable interrupts */
	iowrite32(0, priv->base + INT_FSM_REG);
	iowrite32(0, priv->base + INT_FEM_REG);
	iowrite32(0xffff, priv->base + INT_FEL0_REG);
	iowrite32(0, priv->base + INT_FEL1_REG);
	iowrite32(0, priv->base + INT_STM_REG);
	/* ack interrupts */
	iowrite32(ioread32(priv->base + INT_FSS_REG), priv->base + INT_FSC_REG);
	iowrite32(ioread32(priv->base + INT_FES_REG), priv->base + INT_FEC_REG);
	iowrite32(ioread32(priv->base + INT_STS_REG), priv->base + INT_STC_REG);
	/* enable frame end interrupt */
	iowrite32(BIT(15), priv->base + INT_FEM_REG);
}

static void rcar_isp_stop(struct rcar_isp_priv *priv)
{
	int timeout;
	u32 val;
	int inport = 0;

	/* disable interrupts */
	iowrite32(0, priv->base + INT_FEM_REG);
	/* ack interrups */
	iowrite32(ioread32(priv->base + INT_FES_REG), priv->base + INT_FEC_REG);

	/* MCFE: disable all slots */
	iowrite32(0, priv->base + MCFE_SLM0_REG);
	iowrite32(0, priv->base + MCFE_SLM4_REG);
	/* MCFE: input mode off */
	iowrite32(0, priv->base + MCFE_INM_REG);
	/* MCFE: stop scheduler */
	iowrite32(0, priv->base + MCFE_SCM_REG);
	/* INP: safe stop */
	iowrite32(0, priv->base + INP_MR_REG);
	/* INP: poll status */
	for (timeout = 0; timeout < 100; timeout++) {
		val = ioread32(priv->base + INP_MS_REG);
		if (!(val & (INP_MS(inport))))
			break;
		udelay(1000);
	}
	if (timeout >= 100)
		pr_err("%s timeout input port stop\n", __func__);
}

static void rcar_isp_setup_inputs(struct rcar_isp_priv *priv, int inport, int raw_bpp)
{
	struct rcar_isp_cam *cam = priv->ici.icd->host_priv;
	int timeout;
	u32 val;

	/* MCFE: input mode off */
	iowrite32(0, priv->base + MCFE_INM_REG);
	/* INP: safe stop */
	iowrite32(0, priv->base + INP_MR_REG);
	/* INP: poll status */
	for (timeout = 0; timeout < 100; timeout++) {
		val = ioread32(priv->base + INP_MS_REG);
		if (!(val & (INP_MS(inport))))
			break;
		udelay(1000);
	}
	if (timeout >= 100)
		pr_err("%s timeout input port stop\n", __func__);

	/* INP: freeze */
	iowrite32(0x80, priv->base + INP_MR_REG);
	udelay(100);

	/* Stream Crossbar for MCFE INP#1 */
	val = (0 << 16); /* virtual channel */
	val |= (0 << 8); /* input video port1 */
	val |= 1; /* select data from one input */
	iowrite32(val, priv->base + SCB_SELECT_REG(inport));

	/* window0 setup */
	iowrite32(0, priv->base + INP_HCSTART0_REG(inport));
	iowrite32(0, priv->base + INP_VCSTART_REG(inport));
	iowrite32(cam->out_width, priv->base + INP_HCSIZE0_REG(inport));
	iowrite32(cam->out_height, priv->base + INP_VCSIZE_REG(inport));
	/* window1 setup */
	iowrite32(0, priv->base + INP_HCSTART1_REG(inport));
	iowrite32(cam->out_width, priv->base + INP_HCSIZE1_REG(inport));

	/* MCFE: Invalidate Input stats */
	iowrite32(0, priv->base + MCFE_INSV_REG);
	/* MCFE: input data bpp */
	val = (raw_bpp << 24) | (raw_bpp << 16) | (raw_bpp << 8) | raw_bpp;
	iowrite32(val, priv->base + MCFE_INDW_REG);
	/* MCFE: input data alignment LSB */
	iowrite32(0, priv->base + MCFE_INAL_REG);
	/* MCFE: input mode setup */
	val = 1 << (inport * 8); /* direct through FIFO */
	iowrite32(val, priv->base + MCFE_INM_REG);

	/* INP: unfreeze, safe stop */
	iowrite32(0, priv->base + INP_MR_REG);
}

static void rcar_isp_setup_mcfe(struct rcar_isp_priv *priv, int slot_mode)
{
	u32 val;
	int slot = 0;
	int slot_input = 0;

	/* Slots cancel and clear errors */
	iowrite32(0x06060606, priv->base + MCFE_SLM0_REG);
	iowrite32(0x00000606, priv->base + MCFE_SLM4_REG);
	/* Invalidate Slot cfg */
	iowrite32(0, priv->base + MCFE_SLCFGV0_REG);
	iowrite32(0, priv->base + MCFE_SLCFGV4_REG);
	/* MCFE: input mode off */
	iowrite32(0, priv->base + MCFE_INM_REG);
	/* MCFE: Invalidate Input stats */
	iowrite32(0, priv->base + MCFE_INSV_REG);
	udelay(100); //100

	/* input streams for this slot */
	val = ioread32(priv->base + MCFE_SLINP_REG(slot));
	val &= ~0xff;
	val |= slot_input;
	iowrite32(val, priv->base + MCFE_SLINP_REG(slot));
	/* AXI out_buffers for this slot */
	val = ioread32(priv->base + MCFE_SLOUTBUF1_REG(slot));
	val &= ~0xff;
	val |= 0; /* AXI buffer1 */
	iowrite32(val, priv->base + MCFE_SLOUTBUF1_REG(slot));
	val = ioread32(priv->base + MCFE_SLOUTBUF2_REG(slot));
	val &= ~0xff;
	val |= 1; /* AXI buffer2 */
	iowrite32(val, priv->base + MCFE_SLOUTBUF2_REG(slot));
	/* set slot mode */
	val = ioread32(priv->base + MCFE_SLM0_REG);
	val &= ~(0xff << (slot * 8));
	val |= (slot_mode << (slot * 8));
	iowrite32(val, priv->base + MCFE_SLM0_REG);
	iowrite32(0, priv->base + MCFE_SLM4_REG);
}

static void rcar_isp_setup_pipeline(struct rcar_isp_priv *priv, int axi_colourspace, int axi_num, int is_yuv)
{
	u32 val;

	/* Input formatter */
#if 0
	iowrite32(INF_MI_LINEAR | INF_MI_8BITS | INF_MI_MSB, priv->base + INF_MI_REG);
#else
	val = ioread32(priv->base + INF_MI_REG);
	val &= ~(0x70007);
	iowrite32(val, priv->base + INF_MI_REG);
#endif
	/* pipe input formatter, demosaic, output formatter */
	val = ioread32(priv->base + PIPE_BP_REG);
	val &= ~(PIPE_BP_DEMOSAIC | PIPE_BP_OUTF | PIPE_BP_INF);
	iowrite32(val, priv->base + PIPE_BP_REG);
	/* pipe output formatter RGB2YUV */
	if (is_yuv)
		iowrite32(~OUTF_BP_RGB2YUV, priv->base + OUTF_BP_REG);

	/* AXI output colourspace */
	val = ioread32(priv->base + AXIOUT_MS_REG);
	val &= ~(0xff << axi_num);
	val |= (axi_colourspace << axi_num);
	iowrite32(val, priv->base + AXIOUT_MS_REG);
}

static void rcar_isp_setup_top(struct rcar_isp_priv *priv, int bayer_pattern)
{
	struct rcar_isp_cam *cam = priv->ici.icd->host_priv;

	/* Top setup */
	iowrite32(cam->out_width, priv->base + TOP_AW_REG);
	iowrite32(cam->out_height, priv->base + TOP_AH_REG);
	iowrite32(TOP_CFA_RGGB | bayer_pattern, priv->base + TOP_RGGB_REG);
}

static void rcar_isp_setup_out_buffer(struct rcar_isp_priv *priv, int slot, dma_addr_t phys_addr_top)
{
	struct soc_camera_device *icd = priv->ici.icd;
	struct rcar_isp_cam *cam = icd->host_priv;

	/* invalidate out_buffer */
	iowrite32(0, priv->base + OBUF_BV_REG(slot));
	/* setup phys addr */
	iowrite32(phys_addr_top, priv->base + OBUF_BA_REG(slot));
	/* setup frame params */
	iowrite32(cam->out_width * (cam->out_bpp / 8), priv->base + OBUF_LO_REG(slot));
	iowrite32(cam->out_width, priv->base + OBUF_AW_REG(slot));
	iowrite32(cam->out_height, priv->base + OBUF_AH_REG(slot));
	iowrite32(cam->out_height, priv->base + OBUF_HM_REG(slot));
	/* empty and validate out_buffer */
	iowrite32(0, priv->base + OBUF_ST_REG(slot));
	iowrite32((cam->out_bpp << 8) | 0x1, priv->base + OBUF_BV_REG(slot));
}

static int rcar_isp_setup(struct rcar_isp_priv *priv)
{
	struct soc_camera_device *icd = priv->ici.icd;
	int bayer_pattern, axi_colourspace, raw_bpp;
	int slot = 0;
	int inport = 0;
	int is_yuv = 0;

	/* input interface */
	switch (icd->current_fmt->code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
		raw_bpp = 8;
		break;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		raw_bpp = 12;
		break;
	case MEDIA_BUS_FMT_SGRBG14_1X14:
		raw_bpp = 14;
		break;
	case MEDIA_BUS_FMT_SRGGB16_1X16:
	case MEDIA_BUS_FMT_SBGGR16_1X16:
	case MEDIA_BUS_FMT_SGRBG16_1X16:
		raw_bpp = 16;
		break;
	default:
		goto e_format;
	}

	switch (icd->current_fmt->code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SRGGB16_1X16:
		bayer_pattern = TOP_RGGB_RGrGbB; /* IMX390, AR0143 */
		break;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SBGGR16_1X16:
		bayer_pattern = TOP_RGGB_BGbGrR; /* OV2775, OX03A */
		break;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGRBG14_1X14:
	case MEDIA_BUS_FMT_SGRBG16_1X16:
		bayer_pattern = TOP_RGGB_GrRBGb; /* AR0132, AR0220 */
		break;
	default:
		goto e_format;
	}

	/* output format */
	switch (icd->current_fmt->host_fmt->fourcc) {
	case V4L2_PIX_FMT_NV16:
		axi_colourspace = AXIOUT_MS_YUV_UV8;
		is_yuv = 1;
		break;
	case V4L2_PIX_FMT_XBGR32:
		axi_colourspace = AXIOUT_MS_RGB888;
		axi_colourspace |= AXIOUT_MSB_ALIGN;
		break;
	case V4L2_PIX_FMT_SBGGR8:
		axi_colourspace = AXIOUT_MS_RAW8;
		break;
	case V4L2_PIX_FMT_SBGGR16:
		axi_colourspace = AXIOUT_MS_RAW16;
		axi_colourspace |= AXIOUT_MSB_ALIGN;
		break;
	default:
		goto e_format;
	}

	rcar_isp_setup_inputs(priv, inport, raw_bpp);
	rcar_isp_setup_mcfe(priv, MCFE_SLM0_PASSTHR(slot));
	rcar_isp_setup_top(priv, bayer_pattern);
	rcar_isp_setup_pipeline(priv, axi_colourspace, 0, is_yuv);

	return 0;

e_format:
	dev_warn(icd->parent, "Invalid fourcc format (0x%x)\n",
		 icd->current_fmt->host_fmt->fourcc);
	return -EINVAL;
}

static int rcar_isp_get_free_hw_slot(struct rcar_isp_priv *priv)
{
	int slot;

	for (slot = 0; slot < priv->nr_hw_slots; slot++)
		if (priv->queue_buf[slot] == NULL)
			return slot;

	return -1;
}

static int rcar_isp_hw_ready(struct rcar_isp_priv *priv)
{
	/* Ensure all HW slots are filled */
	return rcar_isp_get_free_hw_slot(priv) < 0 ? 1 : 0;
}

/* Moves a buffer from the queue to the HW slot */
static int rcar_isp_fill_hw_slot(struct rcar_isp_priv *priv)
{
	struct vb2_v4l2_buffer *vbuf;
	dma_addr_t phys_addr_top;
	int slot;

	if (list_empty(&priv->capture))
		return 0;

	/* Find a free HW slot */
	slot = rcar_isp_get_free_hw_slot(priv);
	if (slot < 0)
		return 0;

	vbuf = &list_entry(priv->capture.next, struct rcar_isp_buffer, list)->vb;
	list_del_init(to_buf_list(vbuf));
	priv->queue_buf[slot] = vbuf;
	phys_addr_top = vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 0);

	if (1) {
		rcar_isp_setup_out_buffer(priv, slot, phys_addr_top);
	} else {
		struct rcar_isp_cam *cam = priv->ici.icd->host_priv;

		rcar_isp_setup_out_buffer(priv, 0, phys_addr_top); // Y
		rcar_isp_setup_out_buffer(priv, 1, phys_addr_top +
					  cam->out_width * cam->out_height); // UV
	}
	return 1;
}

static int rcar_isp_videobuf_setup(struct vb2_queue *vq, unsigned int *count,
				   unsigned int *num_planes,
				   unsigned int sizes[], struct device *alloc_devs[])
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_isp_priv *priv = ici->priv;

	if (!vq->num_buffers)
		priv->sequence = 0;

	if (!*count)
		*count = 2;
	priv->vb_count = *count;

	/* Number of hardware slots */
	priv->nr_hw_slots = MAX_BUFFER_NUM;

	if (*num_planes)
		return sizes[0] < icd->sizeimage ? -EINVAL : 0;

	sizes[0] = icd->sizeimage;
	*num_planes = 1;

	dev_dbg(icd->parent, "count=%d, size=%u\n", *count, sizes[0]);

	return 0;
}

static void rcar_isp_videobuf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_isp_priv *priv = ici->priv;
	unsigned long size;

	size = icd->sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(icd->parent, "Buffer #%d too small (%lu < %lu)\n",
			vb->index, vb2_plane_size(vb, 0), size);
		goto error;
	}

	vb2_set_plane_payload(vb, 0, size);

	dev_dbg(icd->parent, "%s (vb=0x%p) 0x%p %lu\n", __func__,
		vb, vb2_plane_vaddr(vb, 0), vb2_get_plane_payload(vb, 0));

	spin_lock_irq(&priv->lock);

	list_add_tail(to_buf_list(vbuf), &priv->capture);
	rcar_isp_fill_hw_slot(priv);

	/* If we weren't running, and have enough buffers, start capturing! */
	if (priv->state != RUNNING && rcar_isp_hw_ready(priv)) {
		if (rcar_isp_setup(priv)) {
			/* Submit error */
			list_del_init(to_buf_list(vbuf));
			spin_unlock_irq(&priv->lock);
			goto error;
		}
		priv->state = RUNNING;
		rcar_isp_start(priv);
	}

	spin_unlock_irq(&priv->lock);

	return;

error:
	vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
}

static void rcar_isp_stop_streaming(struct vb2_queue *vq)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_isp_priv *priv = ici->priv;
	struct list_head *buf_head, *tmp;
	int i;

	spin_lock_irq(&priv->lock);

	if (priv->state == RUNNING) {
		rcar_isp_stop(priv);
		priv->state = STOPPED;
	}

	for (i = 0; i < MAX_BUFFER_NUM; i++) {
		/* invalidate and empty out_buffer */
		iowrite32(0, priv->base + OBUF_BV_REG(i));
		iowrite32(0, priv->base + OBUF_ST_REG(i));
		if (priv->queue_buf[i]) {
			vb2_buffer_done(&priv->queue_buf[i]->vb2_buf,
					VB2_BUF_STATE_ERROR);
			priv->queue_buf[i] = NULL;
		}
	}

	list_for_each_safe(buf_head, tmp, &priv->capture) {
		vb2_buffer_done(&list_entry(buf_head,
				struct rcar_isp_buffer, list)->vb.vb2_buf,
				VB2_BUF_STATE_ERROR);
		list_del_init(buf_head);
	}

	spin_unlock_irq(&priv->lock);
}

static const struct vb2_ops rcar_isp_vb2_ops = {
	.queue_setup		= rcar_isp_videobuf_setup,
	.buf_queue		= rcar_isp_videobuf_queue,
	.stop_streaming		= rcar_isp_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static irqreturn_t rcar_isp_irq(int irq, void *data)
{
	struct rcar_isp_priv *priv = data;
	u32 int_status;
	int slot;
	unsigned int handled = 0;

	spin_lock(&priv->lock);

	int_status = ioread32(priv->base + INT_FES_REG);
	if (!int_status)
		goto done;

	/* ack interrupts */
	iowrite32(int_status, priv->base + INT_FEC_REG);
	/* enable frame end interrupt */
	iowrite32(BIT(15), priv->base + INT_FEM_REG);
	handled = 1;

	if (priv->state == STOPPED)
		goto done;

	/* out_buffer not filled */
	slot = ioread32(priv->base + MCFE_OWL_REG);
	if (slot == 0x3f3f3f3f)
		goto done;
	slot &= 0xff; // tnis is for 1 camera support

	if (!list_empty(&priv->capture)) {
		priv->queue_buf[slot]->field = priv->field;
		priv->queue_buf[slot]->sequence = priv->sequence++;
		priv->queue_buf[slot]->vb2_buf.timestamp = ktime_get_ns();
		vb2_buffer_done(&priv->queue_buf[slot]->vb2_buf, VB2_BUF_STATE_DONE);
		priv->queue_buf[slot] = NULL;

		rcar_isp_fill_hw_slot(priv);
	} else {
		rcar_isp_stop(priv);
		priv->state = STOPPED;
	}

done:
	spin_unlock(&priv->lock);

	return IRQ_RETVAL(handled);
}

static struct v4l2_subdev *find_csi2(struct rcar_isp_priv *pcdev)
{
	struct v4l2_subdev *sd;
	char name[] = "rcar_csi2";

	v4l2_device_for_each_subdev(sd, &pcdev->ici.v4l2_dev) {
		if (!strncmp(name, sd->name, sizeof(name) - 1)) {
			pcdev->csi2_sd = sd;
			return sd;
		}
	}

	return NULL;
}

static struct v4l2_subdev *find_deser(struct rcar_isp_priv *pcdev)
{
	struct v4l2_subdev *sd;
	char name[] = "max9286";
	char name2[] = "ti9x4";

	v4l2_device_for_each_subdev(sd, &pcdev->ici.v4l2_dev) {
		if (!strncmp(name, sd->name, sizeof(name) - 1)) {
			pcdev->deser_sd = sd;
			return sd;
		}
		if (!strncmp(name2, sd->name, sizeof(name2) - 1)) {
			pcdev->deser_sd = sd;
			return sd;
		}
	}

	return NULL;
}

static int rcar_isp_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_isp_priv *priv = ici->priv;
	int i;

	for (i = 0; i < MAX_BUFFER_NUM; i++)
		priv->queue_buf[i] = NULL;

	pm_runtime_get_sync(ici->v4l2_dev.dev);

	if (priv->chip == RCAR_V3M || priv->chip == RCAR_V3H) {
		struct v4l2_subdev *csi2_sd = find_csi2(priv);
		struct v4l2_subdev *deser_sd = find_deser(priv);
		int ret = 0;

		if (csi2_sd) {
			csi2_sd->grp_id = soc_camera_grp_id(icd);
			v4l2_set_subdev_hostdata(csi2_sd, icd);

			ret = v4l2_subdev_call(csi2_sd, core, s_power, 1);
			priv->csi_sync = true;

			if (ret < 0 && ret != -EINVAL)
				priv->csi_sync = false;

			if (ret < 0 && ret != -ENOIOCTLCMD && ret != -ENODEV)
				return ret;
		}
		if (deser_sd) {
			v4l2_set_subdev_hostdata(deser_sd, icd);

			ret = v4l2_subdev_call(deser_sd, core, s_power, 1);
			priv->deser_sync = true;

			if (ret < 0 && ret != -EINVAL)
				priv->deser_sync = false;

			if (ret < 0 && ret != -ENOIOCTLCMD && ret != -ENODEV)
				return ret;
		}
		/*
		 * -ENODEV is special:
		 * either csi2_sd == NULL or the CSI-2 driver
		 * has not found this soc-camera device among its clients
		 */
		if (csi2_sd && ret == -ENODEV)
			csi2_sd->grp_id = 0;

		dev_dbg(icd->parent,
			"R-Car ISP/CSI-2 driver attached to camera %d\n",
			icd->devnum);

	} else
		dev_dbg(icd->parent, "R-Car ISP driver attached to camera %d\n",
			icd->devnum);

	rcar_isp_open(priv);

	return 0;
}

static void rcar_isp_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_isp_priv *priv = ici->priv;
	struct vb2_v4l2_buffer *vbuf;
	struct v4l2_subdev *csi2_sd = find_csi2(priv);
	struct v4l2_subdev *deser_sd = find_deser(priv);
	int i;

	/* make sure active buffer is cancelled */
	spin_lock_irq(&priv->lock);
	for (i = 0; i < MAX_BUFFER_NUM; i++) {
		vbuf = priv->queue_buf[i];
		if (vbuf) {
			list_del_init(to_buf_list(vbuf));
			vb2_buffer_done(&vbuf->vb2_buf, VB2_BUF_STATE_ERROR);
		}
	}
	spin_unlock_irq(&priv->lock);

	rcar_isp_close(priv);

	pm_runtime_put(ici->v4l2_dev.dev);

	if ((csi2_sd) && (priv->csi_sync))
		v4l2_subdev_call(csi2_sd, core, s_power, 0);
	if ((deser_sd) && (priv->deser_sync))
		v4l2_subdev_call(deser_sd, core, s_power, 0);

	dev_dbg(icd->parent, "R-Car ISP driver detached from camera %d\n",
		icd->devnum);
}

/* rect is guaranteed to not exceed the scaled camera rectangle */
static int rcar_isp_set_rect(struct soc_camera_device *icd)
{
	struct rcar_isp_cam *cam = icd->host_priv;
	unsigned int left_offset, top_offset;
	struct v4l2_rect *cam_subrect = &cam->subrect;

	dev_dbg(icd->parent, "Crop %ux%u@%u:%u\n",
		icd->user_width, icd->user_height, cam->isp_left, cam->isp_top);

	left_offset = cam->isp_left;
	top_offset = cam->isp_top;

	dev_dbg(icd->parent, "Cam %ux%u@%u:%u\n",
		cam->width, cam->height, cam->isp_left, cam->isp_top);
	dev_dbg(icd->parent, "Cam subrect %ux%u@%u:%u\n",
		cam_subrect->width, cam_subrect->height,
		cam_subrect->left, cam_subrect->top);

	return 0;
}

#define ISP_MBUS_FLAGS	(V4L2_MBUS_CSI2_LANES | \
			 V4L2_MBUS_CSI2_CHANNELS | \
			 V4L2_MBUS_CSI2_CONTINUOUS_CLOCK | \
			 V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK)

static int rcar_isp_set_bus_param(struct soc_camera_device *icd)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct v4l2_mbus_config cfg;
	unsigned long common_flags;
	int ret;

	ret = v4l2_subdev_call(sd, video, g_mbus_config, &cfg);
	if (!ret) {
		common_flags = soc_mbus_config_compatible(&cfg, ISP_MBUS_FLAGS);
		if (!common_flags) {
			dev_warn(icd->parent,
				 "MBUS flags incompatible: camera 0x%x, host 0x%x\n",
				 cfg.flags, ISP_MBUS_FLAGS);
			return -EINVAL;
		}
	} else if (ret != -ENOIOCTLCMD) {
		return ret;
	} else {
		common_flags = ISP_MBUS_FLAGS;
	}

	return 0;
}

static int rcar_isp_try_bus_param(struct soc_camera_device *icd,
				  unsigned char buswidth)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct v4l2_mbus_config cfg;
	int ret;

	ret = v4l2_subdev_call(sd, video, g_mbus_config, &cfg);
	if (ret == -ENOIOCTLCMD)
		return 0;
	else if (ret)
		return ret;

	/* check is there common mbus flags */
	ret = soc_mbus_config_compatible(&cfg, ISP_MBUS_FLAGS);
	if (ret)
		return 0;

	dev_warn(icd->parent,
		"MBUS flags incompatible: camera 0x%x, host 0x%x\n",
		 cfg.flags, ISP_MBUS_FLAGS);

	return -EINVAL;
}

static bool rcar_isp_packing_supported(const struct soc_mbus_pixelfmt *fmt)
{
	return fmt->packing == SOC_MBUS_PACKING_NONE ||
		(fmt->bits_per_sample > 8 &&
		fmt->packing == SOC_MBUS_PACKING_EXTEND16);
}

static const struct soc_mbus_pixelfmt rcar_isp_formats[] = {
	{
		.fourcc			= V4L2_PIX_FMT_NV16,
		.name			= "NV16",
		.bits_per_sample	= 16,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
		.layout			= SOC_MBUS_LAYOUT_PACKED,
	},
	{
		.fourcc			= V4L2_PIX_FMT_XBGR32,
		.name			= "RGBX8888",
		.bits_per_sample	= 32,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
		.layout			= SOC_MBUS_LAYOUT_PACKED,
	},
	{
		.fourcc			= V4L2_PIX_FMT_ABGR32,
		.name			= "ARGB8888",
		.bits_per_sample	= 32,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
		.layout			= SOC_MBUS_LAYOUT_PACKED,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SBGGR8,
		.name			= "Bayer 8 BGGR",
		.bits_per_sample	= 8,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
		.layout			= SOC_MBUS_LAYOUT_PACKED,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SBGGR16,
		.name			= "Bayer 16 BGGR",
		.bits_per_sample	= 16,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
		.layout			= SOC_MBUS_LAYOUT_PACKED,
	},
};

static int rcar_isp_get_formats(struct soc_camera_device *icd, unsigned int idx,
				struct soc_camera_format_xlate *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;
	int ret, k, n;
	int formats = 0;
	struct rcar_isp_cam *cam;
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_isp_priv *priv = ici->priv;
	struct v4l2_subdev_mbus_code_enum code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.index = idx,
	};
	const struct soc_mbus_pixelfmt *fmt;

	ret = v4l2_subdev_call(sd, pad, enum_mbus_code, NULL, &code);
	if (ret < 0)
		return 0;

	fmt = soc_mbus_get_fmtdesc(code.code);
	if (!fmt) {
		dev_warn(dev, "unsupported format code #%u: %d\n", idx, code.code);
		return 0;
	}

	ret = rcar_isp_try_bus_param(icd, fmt->bits_per_sample);
	if (ret < 0)
		return 0;

	if (!icd->host_priv) {
		struct v4l2_subdev_format fmt = {
			.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		};
		struct v4l2_mbus_framefmt *mf = &fmt.format;
		struct v4l2_rect rect;
		struct device *dev = icd->parent;
		int shift;

		ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &fmt);
		if (ret < 0)
			return ret;

		/* Cache current client geometry */
		ret = soc_camera_client_g_rect(sd, &rect);
		if (ret == -ENOIOCTLCMD) {
			/* Sensor driver doesn't support cropping */
			rect.left = 0;
			rect.top = 0;
			rect.width = mf->width;
			rect.height = mf->height;
		} else if (ret < 0) {
			return ret;
		}

		/*
		 * If sensor proposes too large format then try smaller ones:
		 * 1280x960, 640x480, 320x240
		 */
		for (shift = 0; shift < 3; shift++) {
			if (mf->width <= priv->max_width &&
			    mf->height <= priv->max_height)
				break;

			mf->width = 1280 >> shift;
			mf->height = 960 >> shift;
			ret = v4l2_device_call_until_err(sd->v4l2_dev,
							 soc_camera_grp_id(icd),
							 pad, set_fmt, NULL,
							 &fmt);
			if (ret < 0)
				return ret;
		}

		if (shift == 3) {
			dev_err(dev,
				"Failed to configure the client below %ux%u\n",
				mf->width, mf->height);
			return -EIO;
		}

		dev_dbg(dev, "camera fmt %ux%u\n", mf->width, mf->height);

		cam = kzalloc(sizeof(*cam), GFP_KERNEL);
		if (!cam)
			return -ENOMEM;
		/*
		 * We are called with current camera crop,
		 * initialise subrect with it
		 */
		cam->rect = rect;
		cam->subrect = rect;
		cam->width = mf->width;
		cam->height = mf->height;
		cam->out_width	= mf->width;
		cam->out_height	= mf->height;

		icd->host_priv = cam;
	} else {
		cam = icd->host_priv;
	}

	/* Beginning of a pass */
	if (!idx)
		cam->extra_fmt = NULL;

	switch (code.code) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SBGGR16_1X16:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SRGGB16_1X16:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGRBG14_1X14:
	case MEDIA_BUS_FMT_SGRBG16_1X16:
		if (cam->extra_fmt)
			break;

		/* Add all our formats that can be generated by ISP */
		cam->extra_fmt = rcar_isp_formats;

		n = ARRAY_SIZE(rcar_isp_formats);
		formats += n;
		for (k = 0; xlate && k < n; k++, xlate++) {
			xlate->host_fmt = &rcar_isp_formats[k];
			xlate->code = code.code;
			dev_dbg(dev, "Providing format %s using code %d\n",
				rcar_isp_formats[k].name, code.code);
		}
		break;
	default:
		if (!rcar_isp_packing_supported(fmt))
			return 0;

		dev_dbg(dev, "Providing format %s in pass-through mode\n",
			fmt->name);
		break;
	}

	/* Generic pass-through */
	formats++;
	if (xlate) {
		xlate->host_fmt = fmt;
		xlate->code = code.code;
		xlate++;
	}

	return formats;
}

static void rcar_isp_put_formats(struct soc_camera_device *icd)
{
	kfree(icd->host_priv);
	icd->host_priv = NULL;
}

static int rcar_isp_set_selection(struct soc_camera_device *icd,
				  struct v4l2_selection *sel)
{
	const struct v4l2_rect *rect = &sel->r;
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_isp_priv *priv = ici->priv;
	struct v4l2_selection cam_sel;
	struct rcar_isp_cam *cam = icd->host_priv;
	struct v4l2_rect *cam_rect = &cam_sel.r;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct v4l2_mbus_framefmt *mf = &fmt.format;
	int ret;

	dev_dbg(dev, "S_SELECTION(%ux%u@%u:%u)\n", rect->width, rect->height,
		rect->left, rect->top);

	/* Apply iterative camera S_SELECTION for new input window. */
	ret = soc_camera_client_s_selection(sd, sel, &cam_sel,
				       &cam->rect, &cam->subrect);
	if (ret < 0)
		return ret;

	dev_dbg(dev, "camera cropped to %ux%u@%u:%u\n",
		cam_rect->width, cam_rect->height,
		cam_rect->left, cam_rect->top);

	/* On success cam_crop contains current camera crop */

	/* Retrieve camera output window */
	ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &fmt);
	if (ret < 0)
		return ret;

	if (mf->width > priv->max_width || mf->height > priv->max_height)
		return -EINVAL;

	/* Cache camera output window */
	cam->width = mf->width;
	cam->height = mf->height;

	cam->isp_left = rect->left;
	cam->isp_top = rect->top;

	/* Use ISP cropping to crop to the new window. */
	ret = rcar_isp_set_rect(icd);
	if (ret < 0)
		return ret;

	dev_dbg(dev, "ISP cropped to %ux%u@%u:%u\n",
		icd->user_width, icd->user_height,
		cam->isp_left, cam->isp_top);

	/* Even if only camera cropping succeeded */
	return ret;
}

static int rcar_isp_get_selection(struct soc_camera_device *icd,
				  struct v4l2_selection *sel)
{
	struct rcar_isp_cam *cam = icd->host_priv;

	sel->r = cam->subrect;

	return 0;
}

static int rcar_isp_set_fmt(struct soc_camera_device *icd,
			    struct v4l2_format *f)
{
	struct rcar_isp_cam *cam = icd->host_priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	struct device *dev = icd->parent;
	__u32 pixfmt = pix->pixelformat;
	const struct soc_camera_format_xlate *xlate;
	unsigned int isp_sub_width = 0, isp_sub_height = 0;
	int ret;
	bool can_scale = false;

	dev_dbg(dev, "S_FMT(pix=0x%x, %ux%u)\n", pixfmt, pix->width, pix->height);

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_warn(dev, "Format %x not found\n", pixfmt);
		return -EINVAL;
	}
	/* Calculate client output geometry */
	soc_camera_calc_client_output(icd, &cam->rect, &cam->subrect, pix, &mf, 12);
	mf.field = pix->field;
	mf.colorspace = pix->colorspace;
	mf.code = xlate->code;

	switch (pixfmt) {
	case V4L2_PIX_FMT_XBGR32:
	case V4L2_PIX_FMT_XRGB32:
		cam->out_bpp = 32;
		break;
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_SBGGR16:
		cam->out_bpp = 16;
		break;
	case V4L2_PIX_FMT_SBGGR8:
		cam->out_bpp = 8;
		break;
	default:
		break;
	}

	dev_dbg(dev, "request camera output %ux%u\n", mf.width, mf.height);

	ret = soc_camera_client_scale(icd, &cam->rect, &cam->subrect,
				      &mf, &isp_sub_width, &isp_sub_height,
				      can_scale, 12);

	/* Done with the camera. Now see if we can improve the result */
	dev_dbg(dev, "Camera %d fmt %ux%u, requested %ux%u\n",
		ret, mf.width, mf.height, pix->width, pix->height);

	if (ret == -ENOIOCTLCMD)
		dev_dbg(dev, "Sensor doesn't support scaling\n");
	else if (ret < 0)
		return ret;

	if (mf.code != xlate->code)
		return -EINVAL;

	/* Prepare ISP crop */
	cam->width = mf.width;
	cam->height = mf.height;

	/* We cannot scale up */
	if (pix->width > isp_sub_width)
		isp_sub_width = pix->width;

	if (pix->height > isp_sub_height)
		isp_sub_height = pix->height;

	pix->colorspace = mf.colorspace;

	if (!can_scale) {
		pix->width = isp_sub_width;
		pix->height = isp_sub_height;
	}

	dev_dbg(dev, "W: %u : %u, H: %u : %u\n",
		isp_sub_width, pix->width, isp_sub_height, pix->height);

	cam->out_width = pix->width;
	cam->out_height = pix->height;

	icd->current_fmt = xlate;

	return 0;
}

static int rcar_isp_try_fmt(struct soc_camera_device *icd,
			    struct v4l2_format *f)
{
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct v4l2_subdev_pad_config pad_cfg;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};
	struct v4l2_mbus_framefmt *mf = &format.format;
	__u32 pixfmt = pix->pixelformat;
	int width, height;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		xlate = icd->current_fmt;
		dev_dbg(icd->parent, "Format %x not found, keeping %x\n",
			pixfmt, xlate->host_fmt->fourcc);
		pixfmt = xlate->host_fmt->fourcc;
		pix->pixelformat = pixfmt;
		pix->colorspace = icd->colorspace;
	}

	width = pix->width;
	height = pix->height;

	/* let soc-camera calculate these values */
	pix->bytesperline = 0;
	pix->sizeimage = 0;

	/* limit to sensor capabilities */
	mf->width = pix->width;
	mf->height = pix->height;
	mf->field = pix->field;
	mf->code = xlate->code;
	mf->colorspace = pix->colorspace;

	ret = v4l2_device_call_until_err(sd->v4l2_dev, soc_camera_grp_id(icd),
					 pad, set_fmt, &pad_cfg, &format);
	if (ret < 0)
		return ret;

	pix->field = mf->field;
	pix->colorspace = mf->colorspace;

	return ret;
}

static unsigned int rcar_isp_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = video_drvdata(file);

	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static int rcar_isp_querycap(struct soc_camera_host *ici,
			     struct v4l2_capability *cap)
{
	strlcpy(cap->card, "R_Car_ISP", sizeof(cap->card));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s%d", DRV_NAME, ici->nr);

	return 0;
}

static int rcar_isp_init_videobuf2(struct vb2_queue *vq,
				   struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);

	vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	vq->drv_priv = icd;
	vq->ops = &rcar_isp_vb2_ops;
	vq->mem_ops = &vb2_dma_contig_memops;
	vq->buf_struct_size = sizeof(struct rcar_isp_buffer);
	vq->timestamp_flags  = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	vq->lock = &ici->host_lock;
	vq->dev = ici->v4l2_dev.dev;

	return vb2_queue_init(vq);
}

static int rcar_isp_get_edid(struct soc_camera_device *icd,
			     struct v4l2_edid *edid)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	int ret;

	ret = v4l2_subdev_call(sd, pad, get_edid, edid);
	if (ret < 0)
		return ret;

	return 0;
}

static struct soc_camera_host_ops rcar_isp_host_ops = {
	.owner		= THIS_MODULE,
	.add		= rcar_isp_add_device,
	.remove		= rcar_isp_remove_device,
	.get_formats	= rcar_isp_get_formats,
	.put_formats	= rcar_isp_put_formats,
	.get_selection	= rcar_isp_get_selection,
	.set_selection	= rcar_isp_set_selection,
	.try_fmt	= rcar_isp_try_fmt,
	.set_fmt	= rcar_isp_set_fmt,
	.poll		= rcar_isp_poll,
	.querycap	= rcar_isp_querycap,
	.set_bus_param	= rcar_isp_set_bus_param,
	.init_videobuf2	= rcar_isp_init_videobuf2,
	.get_edid	= rcar_isp_get_edid,
};

#ifdef CONFIG_OF
static const struct of_device_id rcar_isp_of_table[] = {
	{ .compatible = "renesas,isp-r8a77980", .data = (void *)RCAR_V3H },
	{ .compatible = "renesas,isp-r8a77970", .data = (void *)RCAR_V3M },
	{ .compatible = "renesas,rcar-gen3-isp", .data = (void *)RCAR_GEN3 },
	{ },
};
MODULE_DEVICE_TABLE(of, rcar_isp_of_table);
#endif

#define MAP_MAX_NUM 128
static DECLARE_BITMAP(device_map, MAP_MAX_NUM);
static DEFINE_MUTEX(list_lock);

static int rcar_isp_dyn_pdev(struct soc_camera_desc *sdesc,
			       struct rcar_isp_async_client *sasc)
{
	struct platform_device *pdev;
	int ret, i;

	mutex_lock(&list_lock);
	i = find_first_zero_bit(device_map, MAP_MAX_NUM);
	if (i < MAP_MAX_NUM)
		set_bit(i, device_map);
	mutex_unlock(&list_lock);
	if (i >= MAP_MAX_NUM)
		return -ENOMEM;

	pdev = platform_device_alloc("soc-camera-pdrv", ((2 * i) + 1));
	if (!pdev)
		return -ENOMEM;

	ret = platform_device_add_data(pdev, sdesc, sizeof(*sdesc));
	if (ret < 0) {
		platform_device_put(pdev);
		return ret;
	}

	sasc->pdev = pdev;

	return 0;
}

static int rcar_isp_async_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *sd,
				struct v4l2_async_subdev *asd)
{
	return 0;
}

static void rcar_isp_async_unbind(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *sd,
				  struct v4l2_async_subdev *asd)
{
}

static int rcar_isp_async_probe(struct soc_camera_host *ici,
				struct soc_camera_device *icd)
{
	struct soc_camera_desc *sdesc = to_soc_camera_desc(icd);
	struct soc_camera_host_desc *shd = &sdesc->host_desc;
	struct device *control = NULL;
	int ret;

	ret = v4l2_ctrl_handler_init(&icd->ctrl_handler, 16);
	if (ret < 0)
		return ret;

	if (shd->module_name)
		ret = request_module(shd->module_name);

	ret = shd->add_device(icd);

	control = to_soc_camera_control(icd);
	if (!control || !control->driver || !dev_get_drvdata(control) ||
		!try_module_get(control->driver->owner)) {
		shd->del_device(icd);
		ret = -ENODEV;
	}

	return ret;
}

static int rcar_isp_async_complete(struct v4l2_async_notifier *notifier)
{
	struct rcar_isp_async_client *sasc = container_of(notifier,
					struct rcar_isp_async_client, notifier);
	struct soc_camera_device *icd = platform_get_drvdata(sasc->pdev);

	if (to_soc_camera_control(icd)) {
		struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
		int ret;

		mutex_lock(&list_lock);
		ret = rcar_isp_async_probe(ici, icd);
		mutex_unlock(&list_lock);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static struct soc_camera_device *rcar_isp_add_pdev(struct rcar_isp_async_client *sasc)
{
	struct platform_device *pdev = sasc->pdev;
	int ret;

	ret = platform_device_add(pdev);

	if (ret < 0 || !pdev->dev.driver)
		return NULL;

	return platform_get_drvdata(pdev);
}

static const struct v4l2_async_notifier_operations rcar_isp_notifier_ops = {
	.bound = rcar_isp_async_bound,
	.unbind = rcar_isp_async_unbind,
	.complete = rcar_isp_async_complete,
};

static int rcar_isp_soc_of_bind(struct rcar_isp_priv *priv,
				struct soc_camera_host *ici,
				struct device_node *ep,
				struct device_node *remote)
{
	struct soc_camera_device *icd;
	struct soc_camera_desc sdesc = {.host_desc.bus_id = ici->nr,};
	struct rcar_isp_async_client *sasc;
	struct soc_of_info *info;
	struct i2c_client *client;
	char clk_name[V4L2_SUBDEV_NAME_SIZE];
	int ret;

	/* allocate a new subdev and add match info to it */
	info = devm_kzalloc(ici->v4l2_dev.dev, sizeof(struct soc_of_info),
			    GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->sasd.asd.match.fwnode.fwnode = of_fwnode_handle(remote);
	info->sasd.asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
	info->subdev = &info->sasd.asd;

	/* Or shall this be managed by the soc-camera device? */
	sasc = &info->sasc;

	ret = rcar_isp_dyn_pdev(&sdesc, sasc);
	if (ret < 0)
		goto eallocpdev;

	sasc->sensor = &info->sasd.asd;

	icd = rcar_isp_add_pdev(sasc);
	if (!icd) {
		ret = -ENOMEM;
		goto eaddpdev;
	}

	sasc->notifier.subdevs = &info->subdev;
	sasc->notifier.num_subdevs = 1;
	sasc->notifier.ops = &rcar_isp_notifier_ops;

	priv->async_client = sasc;

	client = of_find_i2c_device_by_node(remote);

	if (client)
		snprintf(clk_name, sizeof(clk_name), "%d-%04x",
			 client->adapter->nr, client->addr);
	else
		snprintf(clk_name, sizeof(clk_name), "of-%s",
			 of_node_full_name(remote));

	ret = v4l2_async_notifier_register(&ici->v4l2_dev, &sasc->notifier);
	if (!ret)
		return 0;

	platform_device_del(sasc->pdev);
eaddpdev:
	platform_device_put(sasc->pdev);
eallocpdev:
	devm_kfree(ici->v4l2_dev.dev, info);
	dev_err(ici->v4l2_dev.dev, "group probe failed: %d\n", ret);

	return ret;
}

static int rcar_isp_probe(struct platform_device *pdev)
{
	const struct of_device_id *match = NULL;
	struct rcar_isp_priv *priv;
	struct v4l2_fwnode_endpoint ep;
	struct device_node *np;
	struct resource *mem;
	int irq, ret;
	const char *str;
	unsigned int i;
	struct device_node *epn = NULL, *ren = NULL;
	struct device_node *csi2_ren = NULL, *max9286_ren = NULL, *ti9x4_ren = NULL;
	bool csi_use = false;
	bool max9286_use = false;
	bool ti9x4_use = false;

	match = of_match_device(of_match_ptr(rcar_isp_of_table), &pdev->dev);

	np = of_graph_get_next_endpoint(pdev->dev.of_node, NULL);
	if (!np) {
		dev_err(&pdev->dev, "could not find endpoint\n");
		return -EINVAL;
	}

	for (i = 0; ; i++) {
		epn = of_graph_get_next_endpoint(pdev->dev.of_node, epn);
		if (!epn)
			break;

		ren = of_graph_get_remote_port(epn);
		if (!ren) {
			dev_notice(&pdev->dev, "no remote for %s\n",
					of_node_full_name(epn));
			continue;
		}

		dev_dbg(&pdev->dev, "node name:%s\n", of_node_full_name(ren->parent));

		if (strcmp(ren->parent->name, "csi2") == 0) {
			csi2_ren = ren;
			csi_use = true;
		}

		if (strcmp(ren->parent->name, "max9286") == 0) {
			max9286_ren = of_parse_phandle(epn, "remote-endpoint", 0);
			max9286_use = true;
		}

		if (strcmp(ren->parent->name, "ti9x4") == 0) {
			ti9x4_ren = of_parse_phandle(epn, "remote-endpoint", 0);
			ti9x4_use = true;
		}

		of_node_put(ren);
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(np), &ep);
	if (ret) {
		dev_err(&pdev->dev, "could not parse endpoint\n");
		return ret;
	}

	of_node_put(np);

	priv = devm_kzalloc(&pdev->dev, sizeof(struct rcar_isp_priv),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* ISP Core */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem == NULL)
		return -EINVAL;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -EINVAL;

	priv->base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	ret = devm_request_irq(&pdev->dev, irq, rcar_isp_irq, IRQF_SHARED,
			       dev_name(&pdev->dev), priv);
	if (ret)
		return ret;

	/* ISP Interface */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (mem == NULL)
		return -EINVAL;

	irq = platform_get_irq(pdev, 1);
	if (irq <= 0)
		return -EINVAL;

	priv->interface.base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);
#if 0
	ret = devm_request_irq(&pdev->dev, irq, rcar_isp_interface_irq, IRQF_SHARED,
			       dev_name(&pdev->dev), priv);
	if (ret)
		return ret;
#endif
	priv->ici.priv = priv;
	priv->ici.v4l2_dev.dev = &pdev->dev;
	priv->ici.drv_name = dev_name(&pdev->dev);
	priv->ici.ops = &rcar_isp_host_ops;
	priv->csi_sync = false;
	priv->deser_sync = false;
	priv->dev = &pdev->dev;

	if (!match) {
		priv->ici.nr = pdev->id;
		priv->chip = pdev->id_entry->driver_data;
	} else {
		priv->ici.nr = of_alias_get_id(pdev->dev.of_node, "isp");
		priv->chip = (enum chip_id)match->data;
	}

	priv->max_width = 4096;
	priv->max_height = 4096;

	if ((priv->chip == RCAR_V3M || priv->chip == RCAR_V3H) &&
	    !of_property_read_string(np, "csi,select", &str)) {
		int vc;

		if (strcmp(dev_name(priv->ici.v4l2_dev.dev),
						"fec00000.isp") == 0)
			priv->index = RCAR_ISP_CH_0;
		else if (strcmp(dev_name(priv->ici.v4l2_dev.dev),
						"fee00000.isp") == 0)
			priv->index = RCAR_ISP_CH_1;
		else
			priv->index = RCAR_ISP_CH_NONE;

		if (strcmp(str, "csi40") == 0)
			priv->csi_ch = RCAR_CSI40;
		else if (strcmp(str, "csi41") == 0)
			priv->csi_ch = RCAR_CSI41;
		else
			priv->csi_ch = RCAR_CSI_CH_NONE;

		ret = of_property_read_u32(np, "virtual,channel", &vc);
		if (ret) {
			dev_err(&pdev->dev,
			"could not parse virtual,channel\n");
			return ret;
		}

		if (vc == 0)
			priv->vc = RCAR_VIRTUAL_CH0;
		else if (vc == 1)
			priv->vc = RCAR_VIRTUAL_CH1;
		else if (vc == 2)
			priv->vc = RCAR_VIRTUAL_CH2;
		else if (vc == 3)
			priv->vc = RCAR_VIRTUAL_CH3;
		else
			priv->vc = RCAR_VIRTUAL_NONE;

		dev_dbg(&pdev->dev, "csi_ch:%d, vc:%d\n",
					priv->csi_ch, priv->vc);
	}

	spin_lock_init(&priv->lock);
	INIT_LIST_HEAD(&priv->capture);

	priv->state = STOPPED;

	pm_suspend_ignore_children(&pdev->dev, true);
	pm_runtime_enable(&pdev->dev);

	ret = soc_camera_host_register(&priv->ici);
	if (ret)
		goto cleanup;

	if (csi_use) {
		ret = rcar_isp_soc_of_bind(priv, &priv->ici, epn, csi2_ren->parent);
		if (ret)
			goto cleanup;
	}

	if (max9286_use) {
		ret = rcar_isp_soc_of_bind(priv, &priv->ici, epn, max9286_ren);
		if (ret)
			goto cleanup;
	}

	if (ti9x4_use) {
		ret = rcar_isp_soc_of_bind(priv, &priv->ici, epn, ti9x4_ren);
		if (ret)
			goto cleanup;
	}

	return 0;

cleanup:
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int rcar_isp_remove(struct platform_device *pdev)
{
	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct rcar_isp_priv *priv = container_of(soc_host,
						  struct rcar_isp_priv, ici);

	platform_device_del(priv->async_client->pdev);
	platform_device_put(priv->async_client->pdev);

	v4l2_async_notifier_unregister(&priv->async_client->notifier);

	soc_camera_host_unregister(soc_host);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rcar_isp_suspend(struct device *dev)
{
	return 0;
}

static int rcar_isp_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(rcar_isp_pm_ops,
			rcar_isp_suspend, rcar_isp_resume);
#define DEV_PM_OPS (&rcar_isp_pm_ops)
#else
#define DEV_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver rcar_isp_driver = {
	.probe		= rcar_isp_probe,
	.remove		= rcar_isp_remove,
	.driver		= {
		.name		= DRV_NAME,
		.pm		= DEV_PM_OPS,
		.of_match_table	= of_match_ptr(rcar_isp_of_table),
	},
};

module_platform_driver(rcar_isp_driver);

MODULE_ALIAS("platform:rcar_isp");
MODULE_AUTHOR("Cogent Embedded Inc. <sources@cogentembedded.com>");
MODULE_DESCRIPTION("Renesas R-Car ISP camera host driver");
MODULE_LICENSE("GPL");
