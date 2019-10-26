/*
 * vsp1_dl.h  --  R-Car VSP1 Display List
 *
 * Copyright (C) 2015-2017 Renesas Corporation
 *
 * Contact: Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/refcount.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>
#include <linux/workqueue.h>

#include "vsp1.h"
#include "vsp1_dl.h"
#include "vsp1_drm.h"
#include "vsp1_pipe.h"
#include "vsp1_rwpf.h"

#define VSP1_DL_NUM_ENTRIES		256
#define VSP1_DL_EXT_NUM_ENTRIES		160

#define VSP1_DLH_INT_ENABLE		(1 << 1)
#define VSP1_DLH_AUTO_START		(1 << 0)

struct vsp1_dl_header_list {
	u32 num_bytes;
	u32 addr;
} __attribute__((__packed__));

struct vsp1_dl_header {
	u32 num_lists;
	struct vsp1_dl_header_list lists[8];
	u32 next_header;
	u32 flags;
	/* if (VI6_DL_EXT_CTRL.EXT) */
	u32 zero_bits;
	/* zero_bits:6 + pre_ext_dl_exec:1 + */
	/* post_ext_dl_exec:1 + zero_bits:8 + pre_ext_dl_num_cmd:16 */
	u32 pre_post_num;
	u32 pre_ext_dl_plist;
	/* zero_bits:16 + post_ext_dl_num_cmd:16 */
	u32 post_ext_dl_num_cmd;
	u32 post_ext_dl_p_list;
} __attribute__((__packed__));

struct vsp1_ext_dl_body {
	u32 ext_dl_cmd[2];
	u32 ext_dl_data[2];
} __attribute__((__packed__));

struct vsp1_ext_addr {
	u32 addr;
} __attribute__((__packed__));

struct vsp1_dl_entry {
	u32 addr;
	u32 data;
} __attribute__((__packed__));

/**
 * struct vsp1_dl_body - Display list body
 * @list: entry in the display list list of bodies
 * @free: entry in the pool free body list
 * @pool: pool to which this body belongs
 * @vsp1: the VSP1 device
 * @ext_body: display list extended body
 * @ext_dma: DMA address for extended body
 * @src_dst_addr: display list (Auto-FLD) source/destination address
 * @ext_addr_dma: DMA address for display list (Auto-FLD)
 * @entries: array of entries
 * @dma: DMA address of the entries
 * @size: size of the DMA memory in bytes
 * @num_entries: number of stored entries
 * @max_entries: number of entries available
 */
struct vsp1_dl_body {
	struct list_head list;
	struct list_head free;

	refcount_t refcnt;

	struct vsp1_dl_fragment_pool *pool;
	struct vsp1_device *vsp1;

	struct vsp1_ext_dl_body *ext_body;
	dma_addr_t ext_dma;

	struct vsp1_ext_addr *src_dst_addr;
	dma_addr_t ext_addr_dma;

	struct vsp1_dl_entry *entries;
	dma_addr_t dma;
	size_t size;

	unsigned int num_entries;
	unsigned int max_entries;
};

/**
 * struct vsp1_dl_fragment_pool - display list body/fragment pool
 * @dma: DMA address of the entries
 * @size: size of the full DMA memory pool in bytes
 * @mem: CPU memory pointer for the pool
 * @bodies: Array of DLB structures for the pool
 * @free: List of free DLB entries
 * @lock: Protects the pool and free list
 * @vsp1: the VSP1 device
 */
struct vsp1_dl_fragment_pool {
	/* DMA allocation */
	dma_addr_t dma;
	size_t size;
	void *mem;

	/* Body management */
	struct vsp1_dl_body *bodies;
	struct list_head free;
	spinlock_t lock;

	struct vsp1_device *vsp1;
};

/**
 * struct vsp1_dl_list - Display list
 * @list: entry in the display list manager lists
 * @dlm: the display list manager
 * @header: display list reader
 * @dma: DMA address for the header
 * @body0: first display list body
 * @fragments: list of extra display list bodies
 * @chain: entry in the display list partition chain
 */
struct vsp1_dl_list {
	struct list_head list;
	struct vsp1_dl_manager *dlm;

	struct vsp1_dl_header *header;
	dma_addr_t dma;

	struct vsp1_dl_body *body0;
	struct list_head fragments;

	bool has_chain;
	struct list_head chain;
};

/**
 * struct vsp1_dl_manager - Display List manager
 * @index: index of the related WPF
 * @singleshot: execute the display list in single-shot mode
 * @vsp1: the VSP1 device
 * @lock: protects the free, active, queued, pending and gc_fragments lists
 * @free: array of all free display lists
 * @active: list currently being processed (loaded) by hardware
 * @queued: list queued to the hardware (written to the DL registers)
 * @pending: list waiting to be queued to the hardware
 * @pool: fragment pool for the display list bodies
 */
struct vsp1_dl_manager {
	unsigned int index;
	bool singleshot;
	struct vsp1_device *vsp1;

	spinlock_t lock;
	struct list_head free;
	struct vsp1_dl_list *active;
	struct vsp1_dl_list *queued;
	struct vsp1_dl_list *pending;

	struct vsp1_dl_fragment_pool *pool;
};

/* -----------------------------------------------------------------------------
 * Display List Body Management
 */

/*
 * Fragment pool's reduce the pressure on the iommu TLB by allocating a single
 * large area of DMA memory and allocating it as a pool of fragment bodies
 */
struct vsp1_dl_fragment_pool *
vsp1_dl_fragment_pool_alloc(struct vsp1_device *vsp1, unsigned int qty,
			    unsigned int num_entries, size_t extra_size)
{
	struct vsp1_dl_fragment_pool *pool;
	size_t dlb_size;
	unsigned int i;

	pool = kzalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool)
		return NULL;

	pool->vsp1 = vsp1;

	dlb_size = num_entries * sizeof(struct vsp1_dl_entry) + extra_size;
	pool->size = dlb_size * qty;

	pool->bodies = kcalloc(qty, sizeof(*pool->bodies), GFP_KERNEL);
	if (!pool->bodies) {
		kfree(pool);
		return NULL;
	}

	pool->mem = dma_alloc_wc(vsp1->bus_master, pool->size, &pool->dma,
					    GFP_KERNEL);
	if (!pool->mem) {
		kfree(pool->bodies);
		kfree(pool);
		return NULL;
	}

	spin_lock_init(&pool->lock);
	INIT_LIST_HEAD(&pool->free);

	for (i = 0; i < qty; ++i) {
		struct vsp1_dl_body *dlb = &pool->bodies[i];
		size_t header_offset;
		size_t ex_body_offset;
		size_t ex_addr_offset;

		dlb->pool = pool;
		dlb->max_entries = num_entries;

		dlb->dma = pool->dma + i * dlb_size;
		dlb->entries = pool->mem + i * dlb_size;

		if (!(vsp1->ths_quirks & VSP1_AUTO_FLD_NOT_SUPPORT)) {
			header_offset = dlb->max_entries *
					sizeof(*dlb->entries);
			ex_body_offset = sizeof(struct vsp1_dl_header);
			ex_addr_offset = sizeof(struct vsp1_ext_dl_body);

			dlb->ext_dma = pool->dma + (i * dlb_size) +
					header_offset + ex_body_offset;
			dlb->ext_body = pool->mem + (i * dlb_size) +
					header_offset + ex_body_offset;
			dlb->ext_addr_dma = pool->dma + (i * dlb_size) +
					header_offset + ex_body_offset +
					ex_addr_offset;
			dlb->src_dst_addr = pool->mem + (i * dlb_size) +
					header_offset + ex_body_offset +
					ex_addr_offset;
		}

		list_add_tail(&dlb->free, &pool->free);
	}

	return pool;
}

void vsp1_dl_fragment_pool_free(struct vsp1_dl_fragment_pool *pool)
{
	if (!pool)
		return;

	if (pool->mem)
		dma_free_wc(pool->vsp1->bus_master, pool->size, pool->mem,
			    pool->dma);

	kfree(pool->bodies);
	kfree(pool);
}

struct vsp1_dl_body *vsp1_dl_fragment_get(struct vsp1_dl_fragment_pool *pool)
{
	struct vsp1_dl_body *dlb = NULL;
	unsigned long flags;

	spin_lock_irqsave(&pool->lock, flags);

	if (!list_empty(&pool->free)) {
		dlb = list_first_entry(&pool->free, struct vsp1_dl_body, free);
		list_del(&dlb->free);
		refcount_set(&dlb->refcnt, 1);
	}

	spin_unlock_irqrestore(&pool->lock, flags);

	return dlb;
}

void vsp1_dl_fragment_put(struct vsp1_dl_body *dlb)
{
	unsigned long flags;

	if (!dlb)
		return;

	if (!refcount_dec_and_test(&dlb->refcnt))
		return;

	dlb->num_entries = 0;

	spin_lock_irqsave(&dlb->pool->lock, flags);
	list_add_tail(&dlb->free, &dlb->pool->free);
	spin_unlock_irqrestore(&dlb->pool->lock, flags);
}

/**
 * vsp1_dl_fragment_write - Write a register to a display list fragment
 * @dlb: The fragment
 * @reg: The register address
 * @data: The register value
 *
 * Write the given register and value to the display list fragment. The maximum
 * number of entries that can be written in a fragment is specified when the
 * fragment is allocated by vsp1_dl_fragment_alloc().
 */
void vsp1_dl_fragment_write(struct vsp1_dl_body *dlb, u32 reg, u32 data)
{
	if (unlikely(dlb->num_entries >= dlb->max_entries)) {
		WARN_ONCE(true, "DLB size exceeded (max %u)", dlb->max_entries);
		return;
	}

	dlb->entries[dlb->num_entries].addr = reg;
	dlb->entries[dlb->num_entries].data = data;
	dlb->num_entries++;
}

/* -----------------------------------------------------------------------------
 * Display List Transaction Management
 */

void vsp1_dl_set_addr_auto_fld(struct vsp1_dl_body *dlb,
			       struct vsp1_rwpf *rpf,
			       struct vsp1_rwpf_memory mem)
{
	const struct vsp1_format_info *fmtinfo = rpf->fmtinfo;
	const struct v4l2_rect *crop;
	u32 y_top_index, y_bot_index;
	u32 u_top_index, u_bot_index;
	u32 v_top_index, v_bot_index;
	u32 alpha_index;
	dma_addr_t y_top_addr, y_bot_addr;
	dma_addr_t u_top_addr, u_bot_addr;
	dma_addr_t v_top_addr, v_bot_addr;
	u32 width, stride;

	crop = vsp1_rwpf_get_crop(rpf, rpf->entity.config);
	width = ALIGN(crop->width, 16);
	stride = width * fmtinfo->bpp[0] / 8;

	y_top_index = rpf->entity.index * 8;
	y_bot_index = rpf->entity.index * 8 + 1;
	u_top_index = rpf->entity.index * 8 + 2;
	u_bot_index = rpf->entity.index * 8 + 3;
	v_top_index = rpf->entity.index * 8 + 4;
	v_bot_index = rpf->entity.index * 8 + 5;
	alpha_index = rpf->entity.index * 8 + 6;

	switch (rpf->fmtinfo->fourcc) {
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YVU420M:
		y_top_addr = mem.addr[0];
		y_bot_addr = mem.addr[0] + stride;
		u_top_addr = mem.addr[1];
		u_bot_addr = mem.addr[1] + stride / 2;
		v_top_addr = mem.addr[2];
		v_bot_addr = mem.addr[2] + stride / 2;
		break;

	case V4L2_PIX_FMT_YUV422M:
	case V4L2_PIX_FMT_YVU422M:
		y_top_addr = mem.addr[0];
		y_bot_addr = mem.addr[0] + stride * 2;
		u_top_addr = mem.addr[1];
		u_bot_addr = mem.addr[1] + stride;
		v_top_addr = mem.addr[2];
		v_bot_addr = mem.addr[2] + stride;
		break;

	case V4L2_PIX_FMT_YUV444M:
	case V4L2_PIX_FMT_YVU444M:
		y_top_addr = mem.addr[0];
		y_bot_addr = mem.addr[0] + stride * 3;
		u_top_addr = mem.addr[1];
		u_bot_addr = mem.addr[1] + stride * 3;
		v_top_addr = mem.addr[2];
		v_bot_addr = mem.addr[2] + stride * 3;
		break;

	default:
		y_top_addr = mem.addr[0];
		y_bot_addr = mem.addr[0] + stride;
		u_top_addr = mem.addr[1];
		u_bot_addr = mem.addr[1] + stride;
		v_top_addr = mem.addr[2];
		v_bot_addr = mem.addr[2] + stride;
		break;
	}

	dlb->src_dst_addr[y_top_index].addr = y_top_addr;
	dlb->src_dst_addr[y_bot_index].addr = y_bot_addr;
	dlb->src_dst_addr[u_top_index].addr = u_top_addr;
	dlb->src_dst_addr[u_bot_index].addr = u_bot_addr;
	dlb->src_dst_addr[v_top_index].addr = v_top_addr;
	dlb->src_dst_addr[v_bot_index].addr = v_bot_addr;

	/* ...set alpha-plane address as needed */
	dlb->src_dst_addr[alpha_index].addr = rpf->mem.alpha +
					      crop->top * width + crop->left;
}

static struct vsp1_dl_list *vsp1_dl_list_alloc(struct vsp1_dl_manager *dlm,
					struct vsp1_dl_fragment_pool *pool)
{
	struct vsp1_dl_list *dl;
	size_t header_offset;

	dl = kzalloc(sizeof(*dl), GFP_KERNEL);
	if (!dl)
		return NULL;

	INIT_LIST_HEAD(&dl->fragments);
	dl->dlm = dlm;

	/* Retrieve a body from our DLM body pool */
	dl->body0 = vsp1_dl_fragment_get(pool);
	if (!dl->body0)
		return NULL;

	header_offset = dl->body0->max_entries * sizeof(*dl->body0->entries);

	dl->header = ((void *)dl->body0->entries) + header_offset;
	dl->dma = dl->body0->dma + header_offset;

	memset(dl->header, 0, sizeof(*dl->header));
	dl->header->lists[0].addr = dl->body0->dma;

	return dl;
}

static void vsp1_dl_list_fragments_free(struct vsp1_dl_list *dl)
{
	struct vsp1_dl_body *dlb, *tmp;

	list_for_each_entry_safe(dlb, tmp, &dl->fragments, list) {
		list_del(&dlb->list);
		vsp1_dl_fragment_put(dlb);
	}
}

static void vsp1_dl_list_free(struct vsp1_dl_list *dl)
{
	vsp1_dl_fragment_put(dl->body0);
	vsp1_dl_list_fragments_free(dl);

	kfree(dl);
}

/**
 * vsp1_dl_list_get - Get a free display list
 * @dlm: The display list manager
 *
 * Get a display list from the pool of free lists and return it.
 *
 * This function must be called without the display list manager lock held.
 */
struct vsp1_dl_list *vsp1_dl_list_get(struct vsp1_dl_manager *dlm)
{
	struct vsp1_dl_list *dl = NULL;
	unsigned long flags;

	spin_lock_irqsave(&dlm->lock, flags);

	if (!list_empty(&dlm->free)) {
		dl = list_first_entry(&dlm->free, struct vsp1_dl_list, list);
		list_del(&dl->list);

		/*
		 * The display list chain must be initialised to ensure every
		 * display list can assert list_empty() if it is not in a chain.
		 */
		INIT_LIST_HEAD(&dl->chain);
	}

	spin_unlock_irqrestore(&dlm->lock, flags);

	return dl;
}

/* This function must be called with the display list manager lock held.*/
static void __vsp1_dl_list_put(struct vsp1_dl_list *dl)
{
	struct vsp1_dl_list *dl_child;

	if (!dl)
		return;

	/*
	 * Release any linked display-lists which were chained for a single
	 * hardware operation.
	 */
	if (dl->has_chain) {
		list_for_each_entry(dl_child, &dl->chain, chain)
			__vsp1_dl_list_put(dl_child);
	}

	dl->has_chain = false;

	vsp1_dl_list_fragments_free(dl);

	/* body0 is reused */
	dl->body0->num_entries = 0;

	list_add_tail(&dl->list, &dl->dlm->free);
}

/**
 * vsp1_dl_list_put - Release a display list
 * @dl: The display list
 *
 * Release the display list and return it to the pool of free lists.
 *
 * Passing a NULL pointer to this function is safe, in that case no operation
 * will be performed.
 */
void vsp1_dl_list_put(struct vsp1_dl_list *dl)
{
	unsigned long flags;

	if (!dl)
		return;

	spin_lock_irqsave(&dl->dlm->lock, flags);
	__vsp1_dl_list_put(dl);
	spin_unlock_irqrestore(&dl->dlm->lock, flags);
}

/**
 * vsp1_dl_list_get_body - Obtain the default body for the display list
 * @dl: The display list
 *
 * Obtain a pointer to the internal display list body allowing this to be passed
 * directly to configure operations.
 */
struct vsp1_dl_body *vsp1_dl_list_body(struct vsp1_dl_list *dl)
{
	return dl->body0;
}

/**
 * vsp1_dl_list_add_fragment - Add a fragment to the display list
 * @dl: The display list
 * @dlb: The fragment
 *
 * Add a display list body as a fragment to a display list. Registers contained
 * in fragments are processed after registers contained in the main display
 * list, in the order in which fragments are added.
 *
 * Adding a fragment to a display list passes ownership of the fragment to the
 * list. The caller must not modify the fragment after this call, but can retain
 * a reference to it for future use if necessary, to add to subsequent lists.
 *
 * The reference count of the body is incremented by this attachment, and thus
 * the caller should release it's reference if does not want to cache the body.
 */
int vsp1_dl_list_add_fragment(struct vsp1_dl_list *dl,
			      struct vsp1_dl_body *dlb)
{
	refcount_inc(&dlb->refcnt);

	list_add_tail(&dlb->list, &dl->fragments);
	return 0;
}

/**
 * vsp1_dl_list_add_chain - Add a display list to a chain
 * @head: The head display list
 * @dl: The new display list
 *
 * Add a display list to an existing display list chain. The chained lists
 * will be automatically processed by the hardware without intervention from
 * the CPU. A display list end interrupt will only complete after the last
 * display list in the chain has completed processing.
 *
 * Adding a display list to a chain passes ownership of the display list to
 * the head display list item. The chain is released when the head dl item is
 * put back with __vsp1_dl_list_put().
 */
int vsp1_dl_list_add_chain(struct vsp1_dl_list *head,
			   struct vsp1_dl_list *dl)
{
	head->has_chain = true;
	list_add_tail(&dl->chain, &head->chain);
	return 0;
}

static void vsp1_dl_list_fill_header(struct vsp1_dl_list *dl, bool is_last,
				     unsigned int pipe_index)
{
	struct vsp1_dl_manager *dlm = dl->dlm;
	struct vsp1_dl_header_list *hdr = dl->header->lists;
	struct vsp1_dl_body *dlb;
	unsigned int num_lists = 0;
	struct vsp1_device *vsp1 = dlm->vsp1;
	struct vsp1_drm_pipeline *drm_pipe = &vsp1->drm->pipe[pipe_index];
	struct vsp1_pipeline *pipe = &drm_pipe->pipe;
	unsigned int i, rpf_update = 0;

	/*
	 * Fill the header with the display list bodies addresses and sizes. The
	 * address of the first body has already been filled when the display
	 * list was allocated.
	 */

	hdr->num_bytes = dl->body0->num_entries
		       * sizeof(*dl->header->lists);

	list_for_each_entry(dlb, &dl->fragments, list) {
		num_lists++;
		hdr++;

		hdr->addr = dlb->dma;
		hdr->num_bytes = dlb->num_entries
			       * sizeof(*dl->header->lists);
	}

	dl->header->num_lists = num_lists;

	if (!list_empty(&dl->chain) && !is_last) {
		/*
		 * If this display list's chain is not empty, we are on a list,
		 * and the next item is the display list that we must queue for
		 * automatic processing by the hardware.
		 */
		struct vsp1_dl_list *next = list_next_entry(dl, chain);

		dl->header->next_header = next->dma;
		dl->header->flags = VSP1_DLH_AUTO_START;
	} else if (!dlm->singleshot) {
		/*
		 * if the display list manager works in continuous mode, the VSP
		 * should loop over the display list continuously until
		 * instructed to do otherwise.
		 */
		dl->header->next_header = dl->dma;
		dl->header->flags = VSP1_DLH_INT_ENABLE | VSP1_DLH_AUTO_START;

		for (i = 0; i < vsp1->info->rpf_count; ++i) {
			if (!pipe->inputs[i])
				continue;

			rpf_update |= 0x01 << (16 + i);
		}

		if (!vsp1->info->uapi &&
		    !(dl->dlm->vsp1->ths_quirks & VSP1_AUTO_FLD_NOT_SUPPORT)) {
			/* Set extended display list header */
			/* pre_ext_dl_exec = 1, pre_ext_dl_num_cmd = 1 */
			dl->header->pre_post_num = (1 << 25) | (0x01);
			dl->header->pre_ext_dl_plist = dl->body0->ext_dma;
			dl->header->post_ext_dl_num_cmd = 0;
			dl->header->post_ext_dl_p_list = 0;

			/* Set extended display list (Auto-FLD) */
			/* Set opecode */
			dl->body0->ext_body->ext_dl_cmd[0] = 0x00000003;
			/* RPF[0]-[4] address is updated */
			dl->body0->ext_body->ext_dl_cmd[1] =
						0x00000001 | rpf_update;

			/* Set pointer of source/destination address */
			dl->body0->ext_body->ext_dl_data[0] =
						dl->body0->ext_addr_dma;
			/* Should be set to 0 */
			dl->body0->ext_body->ext_dl_data[1] = 0;
		}
	} else {
		/*
		 * Otherwise, in mem-to-mem mode, we work in single-shot mode
		 * and the next display list must not be started automatically.
		 */
		dl->header->flags = VSP1_DLH_INT_ENABLE;
	}
}

static bool vsp1_dl_list_hw_update_pending(struct vsp1_dl_manager *dlm)
{
	struct vsp1_device *vsp1 = dlm->vsp1;

	if (!dlm->queued)
		return false;

	/*
	 * Check whether the VSP1 has taken the update. The hardware indicates
	 * this by clearing the UPDHDR bit in the CMD register.
	 */
	return !!(vsp1_read(vsp1, VI6_CMD(dlm->index)) & VI6_CMD_UPDHDR);
}

static void vsp1_dl_list_hw_enqueue(struct vsp1_dl_list *dl)
{
	struct vsp1_dl_manager *dlm = dl->dlm;
	struct vsp1_device *vsp1 = dlm->vsp1;

	/*
	 * Program the display list header address. If the hardware is idle
	 * (single-shot mode or first frame in continuous mode) it will then be
	 * started independently. If the hardware is operating, the
	 * VI6_DL_HDR_REF_ADDR register will be updated with the display list
	 * address.
	 */
	vsp1_write(vsp1, VI6_DL_HDR_ADDR(dlm->index), dl->dma);
	if (vsp1->ths_quirks & VSP1_UNDERRUN_WORKAROUND)
		vsp1->dl_addr = dl->dma;
}

static void vsp1_dl_list_commit_continuous(struct vsp1_dl_list *dl)
{
	struct vsp1_dl_manager *dlm = dl->dlm;

	/*
	 * If a previous display list has been queued to the hardware but not
	 * processed yet, the VSP can start processing it at any time. In that
	 * case we can't replace the queued list by the new one, as we could
	 * race with the hardware. We thus mark the update as pending, it will
	 * be queued up to the hardware by the frame end interrupt handler.
	 */
	if (vsp1_dl_list_hw_update_pending(dlm)) {
		__vsp1_dl_list_put(dlm->pending);
		dlm->pending = dl;
		return;
	}

	/*
	 * Pass the new display list to the hardware and mark it as queued. It
	 * will become active when the hardware starts processing it.
	 */
	vsp1_dl_list_hw_enqueue(dl);

	__vsp1_dl_list_put(dlm->queued);
	dlm->queued = dl;
}

static void vsp1_dl_list_commit_singleshot(struct vsp1_dl_list *dl)
{
	struct vsp1_dl_manager *dlm = dl->dlm;

	/*
	 * When working in single-shot mode, the caller guarantees that the
	 * hardware is idle at this point. Just commit the head display list
	 * to hardware. Chained lists will be started automatically.
	 */
	vsp1_dl_list_hw_enqueue(dl);

	dlm->active = dl;
}

void vsp1_dl_list_commit(struct vsp1_dl_list *dl, unsigned int pipe_index)
{
	struct vsp1_dl_manager *dlm = dl->dlm;
	struct vsp1_dl_list *dl_child;
	unsigned long flags;

	/* Fill the header for the head and chained display lists. */
	vsp1_dl_list_fill_header(dl, list_empty(&dl->chain), pipe_index);

	list_for_each_entry(dl_child, &dl->chain, chain) {
		bool last = list_is_last(&dl_child->chain, &dl->chain);

		vsp1_dl_list_fill_header(dl_child, last, pipe_index);
	}

	spin_lock_irqsave(&dlm->lock, flags);

	if (dlm->singleshot)
		vsp1_dl_list_commit_singleshot(dl);
	else
		vsp1_dl_list_commit_continuous(dl);

	spin_unlock_irqrestore(&dlm->lock, flags);
}

/* -----------------------------------------------------------------------------
 * Display List Manager
 */

/**
 * vsp1_dlm_irq_frame_end - Display list handler for the frame end interrupt
 * @dlm: the display list manager
 *
 * Return true if the previous display list has completed at frame end, or false
 * if it has been delayed by one frame because the display list commit raced
 * with the frame end interrupt. The function always returns true in single-shot
 * mode as display list processing is then not continuous and races never occur.
 */
bool vsp1_dlm_irq_frame_end(struct vsp1_dl_manager *dlm, bool interlaced)
{
	struct vsp1_device *vsp1 = dlm->vsp1;
	bool completed = false;

	spin_lock(&dlm->lock);

	/*
	 * The mem-to-mem pipelines work in single-shot mode. No new display
	 * list can be queued, we don't have to do anything.
	 */
	if (dlm->singleshot) {
		__vsp1_dl_list_put(dlm->active);
		dlm->active = NULL;
		completed = true;
		goto done;
	}

	/*
	 * If the commit operation raced with the interrupt and occurred after
	 * the frame end event but before interrupt processing, the hardware
	 * hasn't taken the update into account yet. We have to skip one frame
	 * and retry.
	 */
	if (vsp1_dl_list_hw_update_pending(dlm))
		goto done;

	if (interlaced && ((vsp1_read(vsp1, VI6_STATUS) &
	    VI6_STATUS_FLD_STD(dlm->index)) !=
	    VI6_STATUS_FLD_STD(dlm->index)))
		goto done;
	/*
	 * The device starts processing the queued display list right after the
	 * frame end interrupt. The display list thus becomes active.
	 */
	if (dlm->queued) {
		__vsp1_dl_list_put(dlm->active);
		dlm->active = dlm->queued;
		dlm->queued = NULL;
		completed = true;
	}

	/*
	 * Now that the VSP has started processing the queued display list, we
	 * can queue the pending display list to the hardware if one has been
	 * prepared.
	 */
	if (dlm->pending) {
		vsp1_dl_list_hw_enqueue(dlm->pending);
		dlm->queued = dlm->pending;
		dlm->pending = NULL;
	}

done:
	spin_unlock(&dlm->lock);

	return completed;
}

/* Hardware Setup */
void vsp1_dlm_setup(struct vsp1_device *vsp1, unsigned int pipe_index)
{
	u32 ctrl = (256 << VI6_DL_CTRL_AR_WAIT_SHIFT)
		 | VI6_DL_CTRL_DC2 | VI6_DL_CTRL_DC1 | VI6_DL_CTRL_DC0
		 | VI6_DL_CTRL_DLE;

	if (!vsp1->info->uapi &&
	    !(vsp1->ths_quirks & VSP1_AUTO_FLD_NOT_SUPPORT)) {
		vsp1_write(vsp1, VI6_DL_EXT_CTRL(pipe_index),
			   (0x02 << VI6_DL_EXT_CTRL_POLINT_SHIFT) |
			   VI6_DL_EXT_CTRL_DLPRI | VI6_DL_EXT_CTRL_EXT);
	}
	vsp1_write(vsp1, VI6_DL_CTRL, ctrl);
	vsp1_write(vsp1, VI6_DL_SWAP(pipe_index), VI6_DL_SWAP_LWS |
			 ((pipe_index == 1) ? VI6_DL_SWAP_IND : 0));
}

void vsp1_dlm_reset(struct vsp1_dl_manager *dlm)
{
	unsigned long flags;

	spin_lock_irqsave(&dlm->lock, flags);

	__vsp1_dl_list_put(dlm->active);
	__vsp1_dl_list_put(dlm->queued);
	__vsp1_dl_list_put(dlm->pending);

	spin_unlock_irqrestore(&dlm->lock, flags);

	dlm->active = NULL;
	dlm->queued = NULL;
	dlm->pending = NULL;
}

struct vsp1_dl_manager *vsp1_dlm_create(struct vsp1_device *vsp1,
					unsigned int index,
					unsigned int prealloc)
{
	struct vsp1_dl_manager *dlm;
	size_t header_size;
	unsigned int i;

	dlm = devm_kzalloc(vsp1->dev, sizeof(*dlm), GFP_KERNEL);
	if (!dlm)
		return NULL;

	dlm->index = index;

	dlm->singleshot = vsp1->info->uapi;
	dlm->vsp1 = vsp1;

	spin_lock_init(&dlm->lock);
	INIT_LIST_HEAD(&dlm->free);

	/*
	 * Initialize the display list body and allocate DMA memory for the body
	 * and the header. Both are allocated together to avoid memory
	 * fragmentation, with the header located right after the body in
	 * memory.
	 */
	header_size = ALIGN(sizeof(struct vsp1_dl_header), 8);

	if (!vsp1->info->uapi &&
	    !(vsp1->ths_quirks & VSP1_AUTO_FLD_NOT_SUPPORT)) {
		size_t ex_addr_offset;
		size_t ex_addr_size;

		ex_addr_offset = sizeof(struct vsp1_ext_dl_body);
		ex_addr_size = sizeof(struct vsp1_ext_dl_body)
				* VSP1_DL_EXT_NUM_ENTRIES;
		header_size += (ex_addr_offset + ex_addr_size);
	}

	dlm->pool = vsp1_dl_fragment_pool_alloc(vsp1, prealloc,
					VSP1_DL_NUM_ENTRIES, header_size);
	if (!dlm->pool)
		return NULL;

	for (i = 0; i < prealloc; ++i) {
		struct vsp1_dl_list *dl;

		dl = vsp1_dl_list_alloc(dlm, dlm->pool);
		if (!dl) {
			vsp1_dlm_destroy(dlm);
			return NULL;
		}

		list_add_tail(&dl->list, &dlm->free);
	}

	return dlm;
}

void vsp1_dlm_destroy(struct vsp1_dl_manager *dlm)
{
	struct vsp1_dl_list *dl, *next;

	if (!dlm)
		return;

	list_for_each_entry_safe(dl, next, &dlm->free, list) {
		list_del(&dl->list);
		vsp1_dl_list_free(dl);
	}

	vsp1_dl_fragment_pool_free(dlm->pool);
}
