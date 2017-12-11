/*
 * DMA support for Internal DMAC with SDHI SD/SDIO controller
 *
 * Copyright (C) 2016-17 Renesas Electronics Corporation
 * Copyright (C) 2016-17 Horms Solutions, Simon Horman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/io-64-nonatomic-hi-lo.h>
#include <linux/mfd/tmio.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>
#include <linux/sys_soc.h>

#include "renesas_sdhi.h"
#include "tmio_mmc.h"

#define DM_CM_SEQ_REGSET	0x800
#define DM_CM_SEQ_MODE		0x808
#define DM_CM_SEQ_CTRL		0x810
#define DM_CM_DTRAN_MODE	0x820
#define DM_CM_DTRAN_CTRL	0x828
#define DM_CM_RST		0x830
#define DM_CM_INFO1		0x840
#define DM_CM_INFO1_MASK	0x848
#define DM_CM_INFO2		0x850
#define DM_CM_INFO2_MASK	0x858
#define DM_CM_TUNING_STAT	0x860
#define DM_CM_SEQ_STAT		0x868
#define DM_DTRAN_ADDR		0x880
#define DM_SEQ_CMD		0x8a0
#define DM_SEQ_ARG		0x8a8
#define DM_SEQ_SIZE		0x8b0
#define DM_SEQ_SECCNT		0x8b8
#define DM_SEQ_RSP		0x8c0
#define DM_SEQ_RSP_CHK		0x8c8
#define DM_SEQ_ADDR		0x8d0

/* DM_CM_DTRAN_MODE */
#define DTRAN_MODE_CH_NUM_CH0	0	/* "downstream" = for write commands */
#define DTRAN_MODE_CH_NUM_CH1	BIT(16)	/* "uptream" = for read commands */
#define DTRAN_MODE_BUS_WID_TH	(BIT(5) | BIT(4))
#define DTRAN_MODE_ADDR_MODE	BIT(0)	/* 1 = Increment address */

/* DM_CM_DTRAN_CTRL */
#define DTRAN_CTRL_DM_START	BIT(0)

/* DM_CM_RST */
#define RST_DTRANRST1		BIT(9)
#define RST_DTRANRST0		BIT(8)
#define RST_SEQRST		BIT(0)
#define RST_RESERVED_BITS	GENMASK_ULL(32, 0)

/* DM_CM_INFO1 and DM_CM_INFO1_MASK */
#define INFO1_CLEAR		0
#define INFO1_DTRANEND1_BIT20	BIT(20)
#define INFO1_DTRANEND1_BIT17	BIT(17)
#define INFO1_DTRANEND0		BIT(16)
#define INFO1_SEQSUSPEND	BIT(8)
#define INFO1_SEQEND		BIT(0)

/* DM_CM_INFO2 and DM_CM_INFO2_MASK */
#define INFO2_DTRANERR1		BIT(17)
#define INFO2_DTRANERR0		BIT(16)

/*
 * Specification of this driver:
 * - host->chan_{rx,tx} will be used as a flag of enabling/disabling the dma
 * - Since this SDHI DMAC register set has 16 but 32-bit width, we
 *   need a custom accessor.
 */

static unsigned long global_flags;
/*
 * Workaround for avoiding to use RX DMAC by multiple channels.
 * On R-Car H3 ES1.* and M3-W ES1.0, when multiple SDHI channels use
 * RX DMAC simultaneously, sometimes hundreds of bytes data are not
 * stored into the system memory even if the DMAC interrupt happened.
 * So, this driver then uses one RX DMAC channel only.
 */
#define SDHI_INTERNAL_DMAC_ONE_RX_ONLY	0
#define SDHI_INTERNAL_DMAC_RX_IN_USE	1

/* Definitions for sampling clocks */
static struct renesas_sdhi_scc rcar_gen3_scc_taps[] = {
	{
		.clk_rate = 0,
		.tap = 0x00000300,
	},
};

static const struct renesas_sdhi_of_data of_rcar_gen3_compatible = {
	.tmio_flags	= TMIO_MMC_HAS_IDLE_WAIT | TMIO_MMC_WRPROTECT_DISABLE |
			  TMIO_MMC_CLK_ACTUAL | TMIO_MMC_HAVE_CBSY |
			  TMIO_MMC_MIN_RCAR2,
	.capabilities	= MMC_CAP_SD_HIGHSPEED | MMC_CAP_SDIO_IRQ |
			  MMC_CAP_CMD23,
	.bus_shift	= 2,
	.scc_offset	= 0x1000,
	.taps		= rcar_gen3_scc_taps,
	.taps_num	= ARRAY_SIZE(rcar_gen3_scc_taps),
	/* Gen3 SDHI DMAC can handle 0xffffffff blk count, but seg = 1 */
	.max_blk_count	= 0xffffffff,
	.max_segs	= 1,
};

static const struct of_device_id renesas_sdhi_internal_dmac_of_match[] = {
	{ .compatible = "renesas,sdhi-r8a7795", .data = &of_rcar_gen3_compatible, },
	{ .compatible = "renesas,sdhi-r8a7796", .data = &of_rcar_gen3_compatible, },
	{ .compatible = "renesas,sdhi-r8a77965", .data = &of_rcar_gen3_compatible, },
	{ .compatible = "renesas,sdhi-r8a77990", .data = &of_rcar_gen3_compatible, },
	{ .compatible = "renesas,sdhi-r8a77995", .data = &of_rcar_gen3_compatible, },
	{},
};
MODULE_DEVICE_TABLE(of, renesas_sdhi_internal_dmac_of_match);

static void
renesas_sdhi_internal_dmac_dm_write(struct tmio_mmc_host *host,
				    int addr, u64 val)
{
	writeq(val, host->ctl + addr);
}

static u32
renesas_sdhi_internal_dmac_dm_read(struct tmio_mmc_host *host, int addr)
{
	return readl(host->ctl + addr);
}

static void
renesas_sdhi_internal_dmac_enable_dma(struct tmio_mmc_host *host, bool enable)
{
	if (!host->chan_tx || !host->chan_rx)
		return;

	if (!enable)
		renesas_sdhi_internal_dmac_dm_write(host, DM_CM_INFO1,
						    INFO1_CLEAR);

	if (host->dma->enable) {
		host->dma_irq_mask = host->seq_enabled ?
			0xffffffff : ~(host->dma_tranend1 | INFO1_DTRANEND0);
		host->dma->enable(host, enable);
		renesas_sdhi_internal_dmac_dm_write(host, DM_CM_INFO1_MASK,
						    host->dma_irq_mask);
	}
}

static void
renesas_sdhi_internal_dmac_abort_dma(struct tmio_mmc_host *host) {
	u64 val = RST_DTRANRST1 | RST_DTRANRST0;

	renesas_sdhi_internal_dmac_enable_dma(host, false);

	if (host->seq_enabled)
		val |= RST_SEQRST;

	renesas_sdhi_internal_dmac_dm_write(host, DM_CM_RST,
					    RST_RESERVED_BITS & ~val);
	renesas_sdhi_internal_dmac_dm_write(host, DM_CM_RST,
					    RST_RESERVED_BITS | val);

	if (host->data && host->data->flags & MMC_DATA_READ)
		clear_bit(SDHI_INTERNAL_DMAC_RX_IN_USE, &global_flags);

	renesas_sdhi_internal_dmac_enable_dma(host, true);

	if (host->bounce_sg_mapped) {
		dma_unmap_sg(&host->pdev->dev, &host->bounce_sg, 1,
			     DMA_FROM_DEVICE);
		host->bounce_sg_mapped = false;
	}
}

static void
renesas_sdhi_internal_dmac_dataend_dma(struct tmio_mmc_host *host) {
	tasklet_schedule(&host->dma_complete);
}

static void
renesas_sdhi_internal_dmac_start_dma(struct tmio_mmc_host *host,
				     struct mmc_data *data)
{
	struct scatterlist *sg = host->sg_ptr;
	u32 dtran_mode = DTRAN_MODE_BUS_WID_TH | DTRAN_MODE_ADDR_MODE;

	/* This DMAC cannot handle if sg_len is not 1 */
	WARN_ON(host->sg_len > 1);

	if (!tmio_mmc_pre_dma_transfer(host, data, COOKIE_MAPPED))
		goto force_pio;

	if (data->flags & MMC_DATA_READ) {
		dtran_mode |= DTRAN_MODE_CH_NUM_CH1;
		if (test_bit(SDHI_INTERNAL_DMAC_ONE_RX_ONLY, &global_flags) &&
		    test_and_set_bit(SDHI_INTERNAL_DMAC_RX_IN_USE, &global_flags))
			goto force_pio;
	} else {
		dtran_mode |= DTRAN_MODE_CH_NUM_CH0;
	}

	tmio_mmc_clear_transtate(host);
	renesas_sdhi_internal_dmac_enable_dma(host, true);

	/* set dma parameters */
	renesas_sdhi_internal_dmac_dm_write(host, DM_CM_DTRAN_MODE,
					    dtran_mode);
	renesas_sdhi_internal_dmac_dm_write(host, DM_DTRAN_ADDR,
					    sg_dma_address(sg));

	return;

force_pio:
	host->force_pio = true;
	renesas_sdhi_internal_dmac_enable_dma(host, false);
}

static void renesas_sdhi_internal_dmac_issue_tasklet_fn(unsigned long arg)
{
	struct tmio_mmc_host *host = (struct tmio_mmc_host *)arg;

	tmio_mmc_enable_mmc_irqs(host, TMIO_STAT_DATAEND |
				 TMIO_STAT_DATATIMEOUT);

	/* start the DMAC */
	renesas_sdhi_internal_dmac_dm_write(host, DM_CM_DTRAN_CTRL,
					    DTRAN_CTRL_DM_START);
}

static void renesas_sdhi_internal_dmac_complete_tasklet_fn(unsigned long arg)
{
	struct tmio_mmc_host *host = (struct tmio_mmc_host *)arg;

	spin_lock_irq(&host->lock);

	if (!host->data)
		goto out;

	renesas_sdhi_internal_dmac_enable_dma(host, false);
	if (host->data && host->data->host_cookie == COOKIE_MAPPED) {
		dma_unmap_sg(&host->pdev->dev, host->sg_ptr, host->sg_len,
			     mmc_get_dma_dir(host->data));
		host->data->host_cookie = COOKIE_UNMAPPED;
	}

	if (host->bounce_sg_mapped) {
		dma_unmap_sg(&host->pdev->dev, &host->bounce_sg, 1,
			     DMA_FROM_DEVICE);
		host->bounce_sg_mapped = false;
	}

	if (host->data->flags & MMC_DATA_READ)
		clear_bit(SDHI_INTERNAL_DMAC_RX_IN_USE, &global_flags);

	tmio_mmc_do_data_irq(host);
out:
	spin_unlock_irq(&host->lock);
}

static void
renesas_sdhi_internal_dmac_seq_complete_tasklet_fn(unsigned long arg)
{
	renesas_sdhi_internal_dmac_complete_tasklet_fn(arg);
}

/* DM_CM_SEQ_REGSET bits */
#define DM_CM_SEQ_REGSET_TABLE_NUM	BIT(8)

/* DM_CM_SEQ_CTRL bits */
#define DM_CM_SEQ_CTRL_SEQ_TABLE	BIT(28)
#define DM_CM_SEQ_CTRL_T_NUM		BIT(24)
#define DM_CM_SEQ_CTRL_SEQ_TYPE_SD	BIT(16)
#define DM_CM_SEQ_CTRL_START_NUM(x)	((x) << 12)
#define DM_CM_SEQ_CTRL_END_NUM(x)	((x) << 8)
#define DM_CM_SEQ_CTRL_SEQ_START	BIT(0)

/* DM_SEQ_CMD bits */
#define DM_SEQ_CMD_MULTI		BIT(13)
#define DM_SEQ_CMD_DIO			BIT(12)
#define DM_SEQ_CMD_CMDTYP		BIT(11)
#define DM_SEQ_CMD_RSP_NONE		(BIT(9) | BIT(8))
#define DM_SEQ_CMD_RSP_R1		BIT(10)
#define DM_SEQ_CMD_RSP_R1B		(BIT(10) | BIT(8))
#define DM_SEQ_CMD_RSP_R2		(BIT(10) | BIT(9))
#define DM_SEQ_CMD_RSP_R3		(BIT(10) | BIT(9) | BIT(8))
#define DM_SEQ_CMD_NONAUTOSTP		BIT(7)
#define DM_SEQ_CMD_APP			BIT(6)

#define MAX_CONTEXT_NUM			8

struct tmio_mmc_context {
	u64	seq_cmd;
	u64	seq_arg;
	u64	seq_size;
	u64	seq_seccnt;
	u64	seq_rsp;
	u64	seq_rsp_chk;
	u64	seq_addr;
};

static void
renesas_sdhi_internal_dmac_set_seq_context(struct tmio_mmc_host *host,
					   int ctxt_num,
					   struct tmio_mmc_context *ctxt)
{
	u64 val;

	WARN_ON(ctxt_num >= MAX_CONTEXT_NUM);

	/* set sequencer table/context number */
	if (ctxt_num < 4)
		val = ctxt_num;
	else
		val = DM_CM_SEQ_REGSET_TABLE_NUM | (ctxt_num - 4);
	renesas_sdhi_internal_dmac_dm_write(host, DM_CM_SEQ_REGSET, val);

	/* set command parameter */
	renesas_sdhi_internal_dmac_dm_write(host, DM_SEQ_CMD, ctxt->seq_cmd);
	renesas_sdhi_internal_dmac_dm_write(host, DM_SEQ_ARG, ctxt->seq_arg);
	renesas_sdhi_internal_dmac_dm_write(host, DM_SEQ_SIZE, ctxt->seq_size);
	renesas_sdhi_internal_dmac_dm_write(host, DM_SEQ_SECCNT,
					    ctxt->seq_seccnt);
	renesas_sdhi_internal_dmac_dm_write(host, DM_SEQ_RSP, ctxt->seq_rsp);
	renesas_sdhi_internal_dmac_dm_write(host, DM_SEQ_RSP_CHK,
					    ctxt->seq_rsp_chk);
	renesas_sdhi_internal_dmac_dm_write(host, DM_SEQ_ADDR, ctxt->seq_addr);
}

static int renesas_sdhi_internal_dmac_set_seq_table(struct tmio_mmc_host *host,
						    struct mmc_request *mrq,
						    struct scatterlist *sg,
						     bool ipmmu_on)
{
	struct mmc_card *card = host->mmc->card;
	struct mmc_data *data = mrq->data;
	struct scatterlist *sg_tmp;
	struct tmio_mmc_context ctxt;
	unsigned int blksz, blocks;
	u32 cmd_opcode, cmd_flag, cmd_arg;
	u32 sbc_opcode = 0, sbc_arg = 0;
	int i, ctxt_cnt = 0;

	/* SD_COMBO media not tested */
	cmd_opcode = (mrq->cmd->opcode & 0x3f);
	cmd_flag = DM_SEQ_CMD_CMDTYP;
	if (data->flags & MMC_DATA_READ)
		cmd_flag |= DM_SEQ_CMD_DIO;
	if (mmc_op_multi(mrq->cmd->opcode) ||
	    (cmd_opcode == SD_IO_RW_EXTENDED && mrq->cmd->arg & 0x08000000))
		cmd_flag |= DM_SEQ_CMD_MULTI;
	if (mrq->sbc || cmd_opcode == SD_IO_RW_EXTENDED)
		cmd_flag |= DM_SEQ_CMD_NONAUTOSTP;

	switch (mmc_resp_type(mrq->cmd)) {
	case MMC_RSP_NONE:
		cmd_flag |= DM_SEQ_CMD_RSP_NONE;
		break;
	case MMC_RSP_R1:
	case MMC_RSP_R1 & ~MMC_RSP_CRC:
		cmd_flag |= DM_SEQ_CMD_RSP_R1;
		break;
	case MMC_RSP_R1B:
		cmd_flag |= DM_SEQ_CMD_RSP_R1B;
		break;
	case MMC_RSP_R2:
		cmd_flag |= DM_SEQ_CMD_RSP_R2;
		break;
	case MMC_RSP_R3:
		cmd_flag |= DM_SEQ_CMD_RSP_R3;
		break;
	default:
		pr_debug("Unknown response type %d\n", mmc_resp_type(mrq->cmd));
		return -EINVAL;
	}

	cmd_arg = mrq->cmd->arg;
	if (cmd_opcode == SD_IO_RW_EXTENDED && cmd_arg & 0x08000000) {
		/* SDIO CMD53 block mode */
		cmd_arg &= ~0x1ff;
	}

	if (mrq->sbc) {
		sbc_opcode = (mrq->sbc->opcode & 0x3f) | DM_SEQ_CMD_RSP_R1;
		sbc_arg = mrq->sbc->arg & (MMC_CMD23_ARG_REL_WR |
			  MMC_CMD23_ARG_PACKED | MMC_CMD23_ARG_TAG_REQ);
	}

	blksz = data->blksz;
	if (ipmmu_on) {
		blocks = data->blocks;
		memset(&ctxt, 0, sizeof(ctxt));

		if (sbc_opcode) {
			/* set CMD23 */
			ctxt.seq_cmd = sbc_opcode;
			ctxt.seq_arg = sbc_arg | blocks;
			renesas_sdhi_internal_dmac_set_seq_context
						(host, ctxt_cnt, &ctxt);
			ctxt_cnt++;
		}

		/* set CMD */
		ctxt.seq_cmd = cmd_opcode | cmd_flag;
		ctxt.seq_arg = cmd_arg;
		if (cmd_opcode == SD_IO_RW_EXTENDED && cmd_arg & 0x08000000) {
			/* SDIO CMD53 block mode */
			ctxt.seq_arg |= blocks;
		}
		ctxt.seq_size = blksz;
		ctxt.seq_seccnt = blocks;
		ctxt.seq_addr = sg_dma_address(sg);
		renesas_sdhi_internal_dmac_set_seq_context
						(host, ctxt_cnt, &ctxt);
	} else {
		for_each_sg(sg, sg_tmp, host->sg_len, i) {
			blocks = sg_tmp->length / blksz;
			memset(&ctxt, 0, sizeof(ctxt));

			if (sbc_opcode) {
				/* set CMD23 */
				ctxt.seq_cmd = sbc_opcode;
				ctxt.seq_arg = sbc_arg | blocks;
				if (sbc_arg & MMC_CMD23_ARG_TAG_REQ && card &&
				    card->ext_csd.data_tag_unit_size &&
				    blksz * blocks <
				    card->ext_csd.data_tag_unit_size)
					ctxt.seq_arg &= ~MMC_CMD23_ARG_TAG_REQ;
				renesas_sdhi_internal_dmac_set_seq_context
							(host, ctxt_cnt, &ctxt);
				ctxt_cnt++;
			}

			/* set CMD */
			ctxt.seq_cmd = cmd_opcode | cmd_flag;
			ctxt.seq_arg = cmd_arg;
			if (cmd_opcode == SD_IO_RW_EXTENDED &&
			    cmd_arg & 0x08000000) {
				/* SDIO CMD53 block mode */
				ctxt.seq_arg |= blocks;
			}
			ctxt.seq_size = blksz;
			ctxt.seq_seccnt = blocks;
			ctxt.seq_addr = sg_dma_address(sg_tmp);
			renesas_sdhi_internal_dmac_set_seq_context
						(host, ctxt_cnt, &ctxt);

			if (i < (host->sg_len - 1)) {
				/* increment address */
				if (cmd_opcode == SD_IO_RW_EXTENDED) {
					/*
					 * sg_len should be 1 in SDIO CMD53
					 * byte mode
					 */
					WARN_ON(!(cmd_arg & 0x08000000));
					if (cmd_arg & 0x04000000) {
						/*
						 * SDIO CMD53 address
						 * increment mode
						 */
						cmd_arg +=
							(blocks * blksz) << 9;
					}
				} else {
					/* card uses block-addressing */
					if (card && !(card->state & 0x4))
						cmd_arg += blocks * blksz;
					else
						cmd_arg += blocks;
				}
				ctxt_cnt++;
			}
		}
	}

	if (data->flags & MMC_DATA_READ) {
		/* dummy read */
		if (cmd_opcode == MMC_READ_MULTIPLE_BLOCK && card &&
		    blksz == 512 && data->blocks > 1) {
			memset(&ctxt, 0, sizeof(ctxt));
			if (sbc_opcode) {
				/* set CMD23 */
				ctxt.seq_cmd = sbc_opcode;
				ctxt.seq_arg = sbc_arg | 2;
				if (sbc_arg & MMC_CMD23_ARG_TAG_REQ &&
				    card->ext_csd.data_tag_unit_size &&
				    blksz * 2 <
				      card->ext_csd.data_tag_unit_size)
					ctxt.seq_arg &= ~MMC_CMD23_ARG_TAG_REQ;
				ctxt_cnt++;
				renesas_sdhi_internal_dmac_set_seq_context
							(host, ctxt_cnt, &ctxt);
			}

			/* set CMD18 */
			ctxt.seq_cmd = cmd_opcode | cmd_flag;
			ctxt.seq_arg = mrq->cmd->arg;
			/* card uses block-addressing */
			if (!(card->state & 0x4))
				ctxt.seq_arg += (data->blocks - 2) * 512;
			else
				ctxt.seq_arg += data->blocks - 2;
			ctxt.seq_size = 512;
			ctxt.seq_seccnt = 2;
			ctxt.seq_addr = sg_dma_address(&host->bounce_sg);
			ctxt_cnt++;
			renesas_sdhi_internal_dmac_set_seq_context
							(host, ctxt_cnt, &ctxt);
		} else {
			if (cmd_opcode == SD_SWITCH) {
				/* set SD CMD6 twice  */
				ctxt.seq_addr =
					sg_dma_address(&host->bounce_sg);
			} else if ((card && (mmc_card_sdio(card) ||
				    card->type == MMC_TYPE_SD_COMBO)) ||
				   cmd_opcode == SD_IO_RW_EXTENDED) {
				/*
				 * In case of SDIO/SD_COMBO,
				 * read Common I/O Area 0x0-0x1FF twice.
				 */
				memset(&ctxt, 0, sizeof(ctxt));
				ctxt.seq_cmd = SD_IO_RW_EXTENDED |
					       DM_SEQ_CMD_CMDTYP |
					       DM_SEQ_CMD_DIO |
					       DM_SEQ_CMD_NONAUTOSTP |
					       DM_SEQ_CMD_RSP_R1;
				/*
				 * SD_IO_RW_EXTENDED argument format:
				 * [31] R/W flag -> 0
				 * [30:28] Function number -> 0x0 selects
				 *			      Common I/O Area
				 * [27] Block mode -> 0
				 * [26] Increment address -> 1
				 * [25:9] Regiser address -> 0x0
				 * [8:0] Byte/block count -> 0x0 -> 512Bytes
				 */
				ctxt.seq_arg = 0x04000000;
				ctxt.seq_size = 512;
				ctxt.seq_seccnt = 1;
				ctxt.seq_addr =
					sg_dma_address(&host->bounce_sg);
			} else {
				/* set CMD17 twice */
				memset(&ctxt, 0, sizeof(ctxt));
				ctxt.seq_cmd = MMC_READ_SINGLE_BLOCK |
					       DM_SEQ_CMD_CMDTYP |
					       DM_SEQ_CMD_DIO |
					       DM_SEQ_CMD_RSP_R1;
				if ((cmd_opcode == MMC_READ_SINGLE_BLOCK ||
				     cmd_opcode == MMC_READ_MULTIPLE_BLOCK) &&
				    blksz == 512)
					ctxt.seq_arg = mrq->cmd->arg;
				else
					ctxt.seq_arg = 0;
				ctxt.seq_size = 512;
				ctxt.seq_seccnt = 1;
				ctxt.seq_addr =
					sg_dma_address(&host->bounce_sg);
			}

			for (i = 0; i < 2; i++) {
				ctxt_cnt++;
				renesas_sdhi_internal_dmac_set_seq_context
							(host, ctxt_cnt, &ctxt);
			}
		}
	}

	return ctxt_cnt;
}

void renesas_sdhi_internal_dmac_start_sequencer(struct tmio_mmc_host *host)
{
	struct mmc_card *card = host->mmc->card;
	struct scatterlist *sg = host->sg_ptr;
	struct mmc_host *mmc = host->mmc;
	struct mmc_request *mrq = host->mrq;
	struct mmc_data *data = mrq->data;
	int ret, ctxt_num;
	u64 val;
	bool ipmmu_on = false;

	/* This DMAC cannot handle if sg_len larger than max_segs */
	if (mmc->max_segs == 1 || mmc->max_segs == 3)
		WARN_ON(host->sg_len > mmc->max_segs);
	else
		ipmmu_on = true;

	dev_dbg(&host->pdev->dev, "%s: %d, %x\n", __func__, host->sg_len,
		data->flags);

	if (!card && host->mrq->cmd->opcode == MMC_SEND_TUNING_BLOCK) {
		/*
		 * workaround: if card is NULL,
		 * we can not decide a dummy read command to be added
		 * to the CMD19.
		 */
		goto force_pio;
	}

	ret = tmio_mmc_pre_dma_transfer(host, data, COOKIE_MAPPED);
	if (ret == 0)
		goto force_pio;

	if (ipmmu_on) {
		/*
		 * workaround: if we use IPMMU, sometimes unhandled error
		 * happened
		 */
		switch (host->mrq->cmd->opcode) {
		case MMC_SEND_TUNING_BLOCK_HS200:
		case MMC_SEND_TUNING_BLOCK:
			goto force_pio;
		default:
			break;
		}
	}

	if (data->flags & MMC_DATA_READ && !host->bounce_sg_mapped) {
		if (dma_map_sg(&host->pdev->dev, &host->bounce_sg, 1,
			       DMA_FROM_DEVICE) <= 0) {
			dev_err(&host->pdev->dev, "%s: bounce_sg map failed\n",
				__func__);
			goto force_pio;
		}
		host->bounce_sg_mapped = true;
	}

	renesas_sdhi_internal_dmac_enable_dma(host, true);
	/* set context */
	ctxt_num = renesas_sdhi_internal_dmac_set_seq_table(host, mrq, sg,
							    ipmmu_on);
	if (ctxt_num < 0)
		goto unmap_sg;
	/* set dma mode */
	renesas_sdhi_internal_dmac_dm_write(host, DM_CM_DTRAN_MODE,
					    DTRAN_MODE_BUS_WID_TH);
	/* enable SEQEND irq */
	renesas_sdhi_internal_dmac_dm_write(host, DM_CM_INFO1_MASK,
					    GENMASK_ULL(31, 0) & ~INFO1_SEQEND);

	if (ctxt_num < 4) {
		/* issue table0 commands */
		val = DM_CM_SEQ_CTRL_SEQ_TYPE_SD |
		      DM_CM_SEQ_CTRL_START_NUM(0) |
		      DM_CM_SEQ_CTRL_END_NUM(ctxt_num) |
		      DM_CM_SEQ_CTRL_SEQ_START;
		renesas_sdhi_internal_dmac_dm_write(host, DM_CM_SEQ_CTRL, val);
	} else {
		/* issue table0 commands */
		val = DM_CM_SEQ_CTRL_SEQ_TYPE_SD |
		      DM_CM_SEQ_CTRL_T_NUM |
		      DM_CM_SEQ_CTRL_START_NUM(0) |
		      DM_CM_SEQ_CTRL_END_NUM(3) |
		      DM_CM_SEQ_CTRL_SEQ_START;
		renesas_sdhi_internal_dmac_dm_write(host, DM_CM_SEQ_CTRL, val);
		/* issue table1 commands */
		val = DM_CM_SEQ_CTRL_SEQ_TABLE |
		      DM_CM_SEQ_CTRL_SEQ_TYPE_SD |
		      DM_CM_SEQ_CTRL_T_NUM |
		      DM_CM_SEQ_CTRL_START_NUM(0) |
		      DM_CM_SEQ_CTRL_END_NUM(ctxt_num - 4) |
		      DM_CM_SEQ_CTRL_SEQ_START;
		renesas_sdhi_internal_dmac_dm_write(host, DM_CM_SEQ_CTRL, val);
	}

	return;

unmap_sg:
	if (host->bounce_sg_mapped) {
		dma_unmap_sg(&host->pdev->dev, &host->bounce_sg, 1,
			     DMA_FROM_DEVICE);
		host->bounce_sg_mapped = false;
	}
force_pio:
	host->force_pio = true;
	renesas_sdhi_internal_dmac_enable_dma(host, false);

	return; /* return for PIO */
}

static bool renesas_sdhi_internal_dmac_dma_irq(struct tmio_mmc_host *host)
{
	unsigned int ireg, status;

	status = renesas_sdhi_internal_dmac_dm_read(host, DM_CM_INFO1);
	if (host->seq_enabled && status & INFO1_SEQEND)
		return true;

	ireg = status & ~host->dma_irq_mask;

	if (ireg & INFO1_DTRANEND0) {
		renesas_sdhi_internal_dmac_dm_write(host, DM_CM_INFO1, ireg &
						    ~INFO1_DTRANEND0);
		tmio_mmc_set_transtate(host, TMIO_TRANSTATE_DEND);
		return true;
	}

	if (ireg & host->dma_tranend1) {
		renesas_sdhi_internal_dmac_dm_write(host, DM_CM_INFO1, ireg &
						    ~host->dma_tranend1);
		tmio_mmc_set_transtate(host, TMIO_TRANSTATE_DEND);
		return true;
	}
	return false;
}

static void renesas_sdhi_internal_dmac_seq_irq(struct tmio_mmc_host *host,
					       int status)
{
	struct mmc_data *data;
	struct mmc_command *cmd, *sbc;
	u64 dm_cm_info2;

	spin_lock(&host->lock);
	data = host->data;
	cmd = host->mrq->cmd;
	sbc = host->mrq->sbc;

	dm_cm_info2 = renesas_sdhi_internal_dmac_dm_read(host, DM_CM_INFO2);
	renesas_sdhi_internal_dmac_dm_write(host, DM_CM_INFO1, 0x0);
	renesas_sdhi_internal_dmac_dm_write(host, DM_CM_INFO2, 0x0);

	if (dm_cm_info2) {
		pr_debug("sequencer error, CMD%d SD_INFO2=0x%x\n",
			 cmd->opcode, status >> 16);
		if (status & TMIO_STAT_CMDTIMEOUT) {
			cmd->error = -ETIMEDOUT;
			if (sbc)
				sbc->error = -ETIMEDOUT;
		} else if ((status & TMIO_STAT_CRCFAIL &&
			   cmd->flags & MMC_RSP_CRC) ||
			   status & TMIO_STAT_STOPBIT_ERR ||
			   status & TMIO_STAT_CMD_IDX_ERR) {
			cmd->error = -EILSEQ;
			if (sbc)
				sbc->error = -EILSEQ;
		}

		if (status & TMIO_STAT_DATATIMEOUT)
			data->error = -ETIMEDOUT;
		else if (status & TMIO_STAT_CRCFAIL ||
			 status & TMIO_STAT_STOPBIT_ERR ||
			 status & TMIO_STAT_TXUNDERRUN)
			data->error = -EILSEQ;
	}

	if (host->chan_tx && (data->flags & MMC_DATA_WRITE)) {
		u32 status = sd_ctrl_read16_and_16_as_32(host, CTL_STATUS);
		bool done = false;

		/*
		 * Has all data been written out yet? Testing on SuperH showed,
		 * that in most cases the first interrupt comes already with the
		 * BUSY status bit clear, but on some operations, like mount or
		 * in the beginning of a write / sync / umount, there is one
		 * DATAEND interrupt with the BUSY bit set, in this cases
		 * waiting for one more interrupt fixes the problem.
		 */
		if (host->pdata->flags & TMIO_MMC_HAS_IDLE_WAIT) {
			if (status & TMIO_STAT_SCLKDIVEN)
				done = true;
		} else {
			if (!(status & TMIO_STAT_CMD_BUSY))
				done = true;
		}
		if (!done)
			goto out;
	}
	/* mask sequencer irq */
	renesas_sdhi_internal_dmac_dm_write(host, DM_CM_INFO1_MASK, 0xffffffff);
	tasklet_schedule(&host->seq_complete);

out:
	spin_unlock(&host->lock);
}

static void
renesas_sdhi_internal_dmac_request_dma(struct tmio_mmc_host *host,
				       struct tmio_mmc_data *pdata)
{
	/* Each value is set to non-zero to assume "enabling" each DMA */
	host->chan_rx = host->chan_tx = (void *)0xdeadbeaf;

	host->dma_tranend1 = (host->sdhi_quirks & DTRAEND1_SET_BIT17) ?
		INFO1_DTRANEND1_BIT17 : INFO1_DTRANEND1_BIT20;

	tasklet_init(&host->dma_complete,
		     renesas_sdhi_internal_dmac_complete_tasklet_fn,
		     (unsigned long)host);
	tasklet_init(&host->dma_issue,
		     renesas_sdhi_internal_dmac_issue_tasklet_fn,
		     (unsigned long)host);
	tasklet_init(&host->seq_complete,
		     renesas_sdhi_internal_dmac_seq_complete_tasklet_fn,
		     (unsigned long)host);
	/* alloc bounce_buf for dummy read */
	host->bounce_buf = (u8 *)__get_free_page(GFP_KERNEL | GFP_DMA);
	if (!host->bounce_buf) {
		host->chan_rx = NULL;
		host->chan_tx = NULL;
		return;
	}
	/* setup bounce_sg for dummy read */
	sg_init_one(&host->bounce_sg, host->bounce_buf, 1024);
	host->bounce_sg_mapped = false;
}

static void
renesas_sdhi_internal_dmac_release_dma(struct tmio_mmc_host *host)
{
	/* Each value is set to zero to assume "disabling" each DMA */
	host->chan_rx = host->chan_tx = NULL;

	/* free bounce_buf for dummy read */
	if (host->bounce_buf) {
		if (host->bounce_sg_mapped) {
			dma_unmap_sg(&host->pdev->dev, &host->bounce_sg, 1,
				     DMA_FROM_DEVICE);
			host->bounce_sg_mapped = false;
	}
		free_pages((unsigned long)host->bounce_buf, 0);
		host->bounce_buf = NULL;
	}
}

static const struct tmio_mmc_dma_ops renesas_sdhi_internal_dmac_dma_ops = {
	.start = renesas_sdhi_internal_dmac_start_dma,
	.enable = renesas_sdhi_internal_dmac_enable_dma,
	.request = renesas_sdhi_internal_dmac_request_dma,
	.release = renesas_sdhi_internal_dmac_release_dma,
	.abort = renesas_sdhi_internal_dmac_abort_dma,
	.dataend = renesas_sdhi_internal_dmac_dataend_dma,
	.dma_irq = renesas_sdhi_internal_dmac_dma_irq,
	.seq_start = renesas_sdhi_internal_dmac_start_sequencer,
	.seq_irq = renesas_sdhi_internal_dmac_seq_irq,
};

/*
 * Whitelist of specific R-Car Gen3 SoC ES versions to use this DMAC
 * implementation as others may use a different implementation.
 */
static const struct soc_device_attribute gen3_soc_whitelist[] = {
	/* specific ones */
	{ .soc_id = "r8a7795", .revision = "ES1.*",
	  .data = (void *)BIT(SDHI_INTERNAL_DMAC_ONE_RX_ONLY) },
	{ .soc_id = "r8a7796", .revision = "ES1.0",
	  .data = (void *)BIT(SDHI_INTERNAL_DMAC_ONE_RX_ONLY) },
	/* generic ones */
	{ .soc_id = "r8a7795" },
	{ .soc_id = "r8a7796" },
	{ .soc_id = "r8a77965" },
	{ .soc_id = "r8a77990" },
	{ .soc_id = "r8a77995" },
	{ /* sentinel */ }
};

static int renesas_sdhi_internal_dmac_probe(struct platform_device *pdev)
{
	const struct soc_device_attribute *soc = soc_device_match(gen3_soc_whitelist);

	if (!soc)
		return -ENODEV;

	global_flags |= (unsigned long)soc->data;

#ifndef CONFIG_MMC_SDHI_PIO
	return renesas_sdhi_probe(pdev, &renesas_sdhi_internal_dmac_dma_ops);
#else
	return renesas_sdhi_probe(pdev, NULL);
#endif
}

static const struct dev_pm_ops renesas_sdhi_internal_dmac_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(tmio_mmc_host_runtime_suspend,
			   tmio_mmc_host_runtime_resume,
			   NULL)
};

static struct platform_driver renesas_internal_dmac_sdhi_driver = {
	.driver		= {
		.name	= "renesas_sdhi_internal_dmac",
		.pm	= &renesas_sdhi_internal_dmac_dev_pm_ops,
		.of_match_table = renesas_sdhi_internal_dmac_of_match,
	},
	.probe		= renesas_sdhi_internal_dmac_probe,
	.remove		= renesas_sdhi_remove,
};

module_platform_driver(renesas_internal_dmac_sdhi_driver);

MODULE_DESCRIPTION("Renesas SDHI driver for internal DMAC");
MODULE_AUTHOR("Yoshihiro Shimoda");
MODULE_LICENSE("GPL v2");
