/*
 * rcar_du_kms.c  --  R-Car Display Unit Mode Setting
 *
 * Copyright (C) 2013-2017 Renesas Electronics Corporation
 *
 * Contact: Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

//#define DEBUG
#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include <linux/dma-buf.h>
#include <linux/of_graph.h>
#include <linux/wait.h>

#include "rcar_du_crtc.h"
#include "rcar_du_drv.h"
#include "rcar_du_encoder.h"
#include "rcar_du_kms.h"
#include "rcar_du_lvdsenc.h"
#include "rcar_du_regs.h"
#include "rcar_du_vsp.h"

/* -----------------------------------------------------------------------------
 * Format helpers
 */

static const struct rcar_du_format_info rcar_du_format_infos[] = {
	{
		.fourcc = DRM_FORMAT_RGB565,
		.bpp = 16,
		.planes = 1,
		.pnmr = PnMR_SPIM_TP | PnMR_DDDF_16BPP,
		.edf = PnDDCR4_EDF_NONE,
	}, {
		.fourcc = DRM_FORMAT_ARGB1555,
		.bpp = 16,
		.planes = 1,
		.pnmr = PnMR_SPIM_ALP | PnMR_DDDF_ARGB,
		.edf = PnDDCR4_EDF_NONE,
	}, {
		.fourcc = DRM_FORMAT_XRGB1555,
		.bpp = 16,
		.planes = 1,
		.pnmr = PnMR_SPIM_ALP | PnMR_DDDF_ARGB,
		.edf = PnDDCR4_EDF_NONE,
	}, {
		.fourcc = DRM_FORMAT_XRGB8888,
		.bpp = 32,
		.planes = 1,
		.pnmr = PnMR_SPIM_TP | PnMR_DDDF_16BPP,
		.edf = PnDDCR4_EDF_RGB888,
	}, {
		.fourcc = DRM_FORMAT_ARGB8888,
		.bpp = 32,
		.planes = 1,
		.pnmr = PnMR_SPIM_ALP | PnMR_DDDF_16BPP,
		.edf = PnDDCR4_EDF_ARGB8888,
	}, {
		.fourcc = DRM_FORMAT_UYVY,
		.bpp = 16,
		.planes = 1,
		.pnmr = PnMR_SPIM_TP_OFF | PnMR_DDDF_YC,
		.edf = PnDDCR4_EDF_NONE,
	}, {
		.fourcc = DRM_FORMAT_YUYV,
		.bpp = 16,
		.planes = 1,
		.pnmr = PnMR_SPIM_TP_OFF | PnMR_DDDF_YC,
		.edf = PnDDCR4_EDF_NONE,
	}, {
		.fourcc = DRM_FORMAT_NV12,
		.bpp = 12,
		.planes = 2,
		.pnmr = PnMR_SPIM_TP_OFF | PnMR_DDDF_YC,
		.edf = PnDDCR4_EDF_NONE,
	}, {
		.fourcc = DRM_FORMAT_NV21,
		.bpp = 12,
		.planes = 2,
		.pnmr = PnMR_SPIM_TP_OFF | PnMR_DDDF_YC,
		.edf = PnDDCR4_EDF_NONE,
	}, {
		.fourcc = DRM_FORMAT_NV16,
		.bpp = 16,
		.planes = 2,
		.pnmr = PnMR_SPIM_TP_OFF | PnMR_DDDF_YC,
		.edf = PnDDCR4_EDF_NONE,
	}, {
		.fourcc = DRM_FORMAT_R8,
		.bpp = 8,
		.planes = 1,
		.pnmr = PnMR_SPIM_TP_OFF | PnMR_DDDF_8BPP,
		.edf = PnDDCR4_EDF_NONE,
	},
	/*
	 * The following formats are not supported on Gen2 and thus have no
	 * associated .pnmr or .edf settings.
	 */
	{
		.fourcc = DRM_FORMAT_NV61,
		.bpp = 16,
		.planes = 2,
	}, {
		.fourcc = DRM_FORMAT_YUV420,
		.bpp = 12,
		.planes = 3,
	}, {
		.fourcc = DRM_FORMAT_YVU420,
		.bpp = 12,
		.planes = 3,
	}, {
		.fourcc = DRM_FORMAT_YUV422,
		.bpp = 16,
		.planes = 3,
	}, {
		.fourcc = DRM_FORMAT_YVU422,
		.bpp = 16,
		.planes = 3,
	}, {
		.fourcc = DRM_FORMAT_YUV444,
		.bpp = 24,
		.planes = 3,
	}, {
		.fourcc = DRM_FORMAT_YVU444,
		.bpp = 24,
		.planes = 3,
	}, {
		.fourcc = DRM_FORMAT_RGB332,
		.bpp = 8,
		.planes = 1,
	}, {
		.fourcc = DRM_FORMAT_ARGB4444,
		.bpp = 16,
		.planes = 1,
	}, {
		.fourcc = DRM_FORMAT_XRGB4444,
		.bpp = 16,
		.planes = 1,
	}, {
		.fourcc = DRM_FORMAT_BGR888,
		.bpp = 24,
		.planes = 1,
	}, {
		.fourcc = DRM_FORMAT_RGB888,
		.bpp = 24,
		.planes = 1,
	}, {
		.fourcc = DRM_FORMAT_BGRA8888,
		.bpp = 32,
		.planes = 1,
	}, {
		.fourcc = DRM_FORMAT_BGRX8888,
		.bpp = 32,
		.planes = 1,
	}, {
		.fourcc = DRM_FORMAT_YVYU,
		.bpp = 16,
		.planes = 1,
	},
};

const struct rcar_du_format_info *rcar_du_format_info(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rcar_du_format_infos); ++i) {
		if (rcar_du_format_infos[i].fourcc == fourcc)
			return &rcar_du_format_infos[i];
	}

	return NULL;
}

/* -----------------------------------------------------------------------------
 * Frame buffer
 */

struct drm_gem_object *rcar_du_gem_prime_import_sg_table(struct drm_device *dev,
				struct dma_buf_attachment *attach,
				struct sg_table *sgt)
{
	struct rcar_du_device *rcdu = dev->dev_private;
	struct drm_gem_cma_object *cma_obj;
	struct drm_gem_object *gem_obj;
	int ret;

	if (!rcar_du_has(rcdu, RCAR_DU_FEATURE_VSP1_SOURCE))
		return drm_gem_cma_prime_import_sg_table(dev, attach, sgt);

	/* Create a CMA GEM buffer. */
	cma_obj = kzalloc(sizeof(*cma_obj), GFP_KERNEL);
	if (!cma_obj)
		return ERR_PTR(-ENOMEM);
	gem_obj = &cma_obj->base;

	ret = drm_gem_object_init(dev, gem_obj, attach->dmabuf->size);
	if (ret)
		goto error;

	ret = drm_gem_create_mmap_offset(gem_obj);
	if (ret) {
		drm_gem_object_release(gem_obj);
		goto error;
	}

	cma_obj->paddr = sg_dma_address(sgt->sgl);
	cma_obj->sgt = sgt;

	return gem_obj;

error:
	kfree(cma_obj);
	return ERR_PTR(ret);
}

int rcar_du_dumb_create(struct drm_file *file, struct drm_device *dev,
			struct drm_mode_create_dumb *args)
{
	struct rcar_du_device *rcdu = dev->dev_private;
	unsigned int min_pitch = DIV_ROUND_UP(args->width * args->bpp, 8);
	unsigned int align;

	/*
	 * The R8A7779 DU requires a 16 pixels pitch alignment as documented,
	 * but the R8A7790 DU seems to require a 128 bytes pitch alignment.
	 */
	if (rcar_du_needs(rcdu, RCAR_DU_QUIRK_ALIGN_128B))
		align = 128;
	else
		align = 16 * args->bpp / 8;

	args->pitch = roundup(min_pitch, align);

	return drm_gem_cma_dumb_create_internal(file, dev, args);
}

static struct drm_framebuffer *
rcar_du_fb_create(struct drm_device *dev, struct drm_file *file_priv,
		  const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct rcar_du_device *rcdu = dev->dev_private;
	const struct rcar_du_format_info *format;
	unsigned int max_pitch;
	unsigned int align;
	unsigned int bpp;
	unsigned int i;

	format = rcar_du_format_info(mode_cmd->pixel_format);
	if (format == NULL) {
		dev_dbg(dev->dev, "unsupported pixel format %08x\n",
			mode_cmd->pixel_format);
		return ERR_PTR(-EINVAL);
	}

	if (rcar_du_has(rcdu, RCAR_DU_FEATURE_VSP1_SOURCE))
		goto done;

	/*
	 * The pitch and alignment constraints are expressed in pixels on the
	 * hardware side and in bytes in the DRM API.
	 */
	bpp = format->planes == 1 ? format->bpp / 8 : 1;
	max_pitch =  4096 * bpp;

	if (rcar_du_needs(rcdu, RCAR_DU_QUIRK_ALIGN_128B))
		align = 128;
	else
		align = 16 * bpp;

	if (mode_cmd->pitches[0] & (align - 1) ||
	    mode_cmd->pitches[0] >= max_pitch) {
		dev_dbg(dev->dev, "invalid pitch value %u\n",
			mode_cmd->pitches[0]);
		return ERR_PTR(-EINVAL);
	}

	for (i = 1; i < format->planes; ++i) {
		if (mode_cmd->pitches[i] != mode_cmd->pitches[0]) {
			dev_dbg(dev->dev,
				"luma and chroma pitches do not match\n");
			return ERR_PTR(-EINVAL);
		}
	}
done:
	return drm_fb_cma_create(dev, file_priv, mode_cmd);
}

static void rcar_du_output_poll_changed(struct drm_device *dev)
{
	struct rcar_du_device *rcdu = dev->dev_private;

	drm_fbdev_cma_hotplug_event(rcdu->fbdev);
}

/* -----------------------------------------------------------------------------
 * Atomic Check and Update
 */

static int rcar_du_atomic_check(struct drm_device *dev,
				struct drm_atomic_state *state)
{
	struct rcar_du_device *rcdu = dev->dev_private;
	int ret;

	ret = drm_atomic_helper_check_modeset(dev, state);
	if (ret)
		return ret;

	ret = drm_atomic_normalize_zpos(dev, state);
	if (ret)
		return ret;

	ret = drm_atomic_helper_check_planes(dev, state);
	if (ret)
		return ret;

	if (rcar_du_has(rcdu, RCAR_DU_FEATURE_VSP1_SOURCE))
		return 0;

	return rcar_du_atomic_check_planes(dev, state);
}

static void rcar_du_atomic_commit_tail(struct drm_atomic_state *old_state)
{
	struct drm_device *dev = old_state->dev;

	/* Apply the atomic update. */
	drm_atomic_helper_commit_modeset_disables(dev, old_state);
	drm_atomic_helper_commit_planes(dev, old_state,
					DRM_PLANE_COMMIT_ACTIVE_ONLY);
	drm_atomic_helper_commit_modeset_enables(dev, old_state);

	drm_atomic_helper_commit_hw_done(old_state);
	drm_atomic_helper_wait_for_flip_done(dev, old_state);

	drm_atomic_helper_cleanup_planes(dev, old_state);
}

int rcar_du_async_commit(struct drm_device *dev, struct drm_crtc *crtc)
{
	int ret = 0;
	struct drm_atomic_state *state;
	struct drm_crtc_state *crtc_state;
	struct drm_mode_config *config = &dev->mode_config;

	drm_modeset_lock_all(dev);

	state = drm_atomic_state_alloc(dev);
	if (!state) {
		ret = -ENOMEM;
		goto err;
	}

	crtc_state = drm_atomic_helper_crtc_duplicate_state(crtc);
	if (!crtc_state) {
		ret = -ENOMEM;
		goto err;
	}

	state->crtcs->state = crtc_state;
	state->crtcs->old_state = crtc->state;
	state->crtcs->new_state = crtc_state;
	state->crtcs->ptr = crtc;
	crtc_state->state = state;
	crtc_state->active = true;

	state->acquire_ctx = config->acquire_ctx;
	ret = drm_atomic_commit(state);
	drm_atomic_state_put(state);
err:
	drm_modeset_unlock_all(dev);

	return ret;
}

/* -----------------------------------------------------------------------------
 * Initialization
 */

static const struct drm_mode_config_helper_funcs rcar_du_mode_config_helper = {
	.atomic_commit_tail = rcar_du_atomic_commit_tail,
};

static const struct drm_mode_config_funcs rcar_du_mode_config_funcs = {
	.fb_create = rcar_du_fb_create,
	.output_poll_changed = rcar_du_output_poll_changed,
	.atomic_check = rcar_du_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int rcar_du_encoders_init_one(struct rcar_du_device *rcdu,
				     enum rcar_du_output output,
				     struct of_endpoint *ep)
{
	struct device_node *connector = NULL;
	struct device_node *encoder = NULL;
	struct device_node *ep_node = NULL;
	struct device_node *entity_ep_node;
	struct device_node *entity;
	int ret;

	/*
	 * Locate the connected entity and infer its type from the number of
	 * endpoints.
	 */
	entity = of_graph_get_remote_port_parent(ep->local_node);
	if (!entity) {
		dev_dbg(rcdu->dev, "unconnected endpoint %pOF, skipping\n",
			ep->local_node);
		return -ENODEV;
	}

	if (!of_device_is_available(entity)) {
		dev_dbg(rcdu->dev,
			"connected entity %pOF is disabled, skipping\n",
			entity);
		return -ENODEV;
	}

	entity_ep_node = of_graph_get_remote_endpoint(ep->local_node);

	for_each_endpoint_of_node(entity, ep_node) {
		if (ep_node == entity_ep_node)
			continue;

		/*
		 * We've found one endpoint other than the input, this must
		 * be an encoder. Locate the connector.
		 */
		encoder = entity;
		connector = of_graph_get_remote_port_parent(ep_node);
		of_node_put(ep_node);

		if (!connector) {
			dev_warn(rcdu->dev,
				 "no connector for encoder %pOF, skipping\n",
				 encoder);
			of_node_put(entity_ep_node);
			of_node_put(encoder);
			return -ENODEV;
		}

		break;
	}

	of_node_put(entity_ep_node);

	if (!encoder) {
		/*
		 * If no encoder has been found the entity must be the
		 * connector.
		 */
		connector = entity;
	}

	ret = rcar_du_encoder_init(rcdu, output, encoder, connector);
	if (ret && ret != -EPROBE_DEFER)
		dev_warn(rcdu->dev,
			 "failed to initialize encoder %pOF on output %u (%d), skipping\n",
			 encoder, output, ret);

	of_node_put(encoder);
	of_node_put(connector);

	return ret;
}

static int rcar_du_encoders_init(struct rcar_du_device *rcdu)
{
	struct device_node *np = rcdu->dev->of_node;
	struct device_node *ep_node;
	unsigned int num_encoders = 0;

	/*
	 * Iterate over the endpoints and create one encoder for each output
	 * pipeline.
	 */
	for_each_endpoint_of_node(np, ep_node) {
		enum rcar_du_output output;
		struct of_endpoint ep;
		unsigned int i;
		int ret;

		ret = of_graph_parse_endpoint(ep_node, &ep);
		if (ret < 0) {
			of_node_put(ep_node);
			return ret;
		}

		/* Find the output route corresponding to the port number. */
		for (i = 0; i < RCAR_DU_OUTPUT_MAX; ++i) {
			if (rcdu->info->routes[i].possible_crtcs &&
			    rcdu->info->routes[i].port == ep.port) {
				output = i;
				break;
			}
		}

		if (i == RCAR_DU_OUTPUT_MAX) {
			dev_warn(rcdu->dev,
				 "port %u references unexisting output, skipping\n",
				 ep.port);
			continue;
		}

		/* Process the output pipeline. */
		ret = rcar_du_encoders_init_one(rcdu, output, &ep);
		if (ret < 0) {
			if (ret == -EPROBE_DEFER) {
				of_node_put(ep_node);
				return ret;
			}

			continue;
		}

		num_encoders++;
	}

	return num_encoders;
}

static int rcar_du_properties_init(struct rcar_du_device *rcdu)
{
	rcdu->props.alpha =
		drm_property_create_range(rcdu->ddev, 0, "alpha", 0, 255);
	if (rcdu->props.alpha == NULL)
		return -ENOMEM;

	/*
	 * The color key is expressed as an RGB888 triplet stored in a 32-bit
	 * integer in XRGB8888 format. Bit 24 is used as a flag to disable (0)
	 * or enable source color keying (1).
	 */
	rcdu->props.colorkey =
		drm_property_create_range(rcdu->ddev, 0, "colorkey",
					  0, 0x01ffffff);
	if (rcdu->props.colorkey == NULL)
		return -ENOMEM;

	if (rcdu->info->gen == 3) {
		rcdu->props.colorkey_alpha =
			drm_property_create_range(rcdu->ddev, 0,
						  "colorkey_alpha", 0, 255);
		if (!rcdu->props.colorkey_alpha)
			return -ENOMEM;
	}

	rcdu->props.alphaplane =
		drm_property_create(rcdu->ddev, DRM_MODE_PROP_OBJECT, "alphaplane", 1);
	if (rcdu->props.alphaplane == NULL)
		return -ENOMEM;

	rcdu->props.alphaplane->values[0] = DRM_MODE_OBJECT_FB;

	rcdu->props.blend =
		drm_property_create_range(rcdu->ddev, 0, "blend",
					  0, 0xffffffff);
	if (rcdu->props.blend == NULL)
		return -ENOMEM;

	rcdu->props.ckey =
		drm_property_create_range(rcdu->ddev, 0, "ckey",
					  0, 0xffffffff);
	if (rcdu->props.ckey == NULL)
		return -ENOMEM;

	rcdu->props.ckey_set0 =
		drm_property_create_range(rcdu->ddev, 0, "ckey_set0",
					  0, 0xffffffff);
	if (rcdu->props.ckey_set0 == NULL)
		return -ENOMEM;

	rcdu->props.ckey_set1 =
		drm_property_create_range(rcdu->ddev, 0, "ckey_set1",
					  0, 0xffffffff);
	if (rcdu->props.ckey_set1 == NULL)
		return -ENOMEM;

	return 0;
}

static int rcar_du_vsps_init(struct rcar_du_device *rcdu)
{
	const struct device_node *np = rcdu->dev->of_node;
	struct of_phandle_args args;
	struct {
		struct device_node *np;
		unsigned int crtcs_mask;
	} vsps[RCAR_DU_MAX_VSPS] = { { NULL, }, };
	unsigned int vsps_count = 0;
	unsigned int cells;
	unsigned int i;
	int ret;

	/*
	 * First parse the DT vsps property to populate the list of VSPs. Each
	 * entry contains a pointer to the VSP DT node and a bitmask of the
	 * connected DU CRTCs.
	 */
	cells = of_property_count_u32_elems(np, "vsps") / rcdu->num_crtcs - 1;
	if (cells > 1)
		return -EINVAL;

	for (i = 0; i < rcdu->num_crtcs; ++i) {
		unsigned int j;
		unsigned int brs_num = 0;

		ret = of_parse_phandle_with_fixed_args(np, "vsps", cells, i,
						       &args);
		if (ret < 0)
			goto error;

		ret = of_property_read_u32(args.np, "renesas,#brs", &brs_num);
		if (brs_num > 2) {
			dev_err(rcdu->dev, "error: brs number\n");
			goto error;
		}
		/*
		 * Add the VSP to the list or update the corresponding existing
		 * entry if the VSP has already been added.
		 */
		for (j = 0; j < vsps_count; ++j) {
			if (vsps[j].np == args.np)
				break;
		}

		if (j < vsps_count)
			of_node_put(args.np);
		else
			vsps[vsps_count++].np = args.np;

		vsps[j].crtcs_mask |= BIT(i);

		/* Store the VSP pointer and pipe index in the CRTC. */
		rcdu->crtcs[i].vsp = &rcdu->vsps[j];
		rcdu->crtcs[i].vsp_pipe = cells >= 1 ? args.args[0] : 0;

		/* Has VSPDL */
		if (rcdu->crtcs[i].vsp_pipe) {
			if (!ret)
				rcdu->vspdl_fix = true;
			rcdu->brs_num = brs_num;
		}
	}

	/*
	 * Then initialize all the VSPs from the node pointers and CRTCs bitmask
	 * computed previously.
	 */
	for (i = 0; i < vsps_count; ++i) {
		struct rcar_du_vsp *vsp = &rcdu->vsps[i];

		vsp->index = i;
		vsp->dev = rcdu;

		ret = rcar_du_vsp_init(vsp, vsps[i].np, vsps[i].crtcs_mask);
		if (ret < 0)
			goto error;
	}

	return 0;

error:
	for (i = 0; i < ARRAY_SIZE(vsps); ++i)
		of_node_put(vsps[i].np);

	return ret;
}

int rcar_du_modeset_init(struct rcar_du_device *rcdu)
{
	static const unsigned int mmio_offsets[] = {
		DU0_REG_OFFSET, DU2_REG_OFFSET
	};

	struct drm_device *dev = rcdu->ddev;
	struct drm_encoder *encoder;
	struct drm_fbdev_cma *fbdev;
	unsigned int num_encoders;
	unsigned int num_groups;
	unsigned int i;
	int ret;

	drm_mode_config_init(dev);

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_width = 4096;
	dev->mode_config.max_height = 2160;
	dev->mode_config.funcs = &rcar_du_mode_config_funcs;
	dev->mode_config.helper_private = &rcar_du_mode_config_helper;

	rcdu->num_crtcs = rcdu->info->num_crtcs;
	rcdu->vspdl_fix = false;
	rcdu->brs_num = 0;

	ret = rcar_du_properties_init(rcdu);
	if (ret < 0)
		return ret;

	/*
	 * Initialize vertical blanking interrupts handling. Start with vblank
	 * disabled for all CRTCs.
	 */
	ret = drm_vblank_init(dev, (1 << rcdu->info->num_crtcs) - 1);
	if (ret < 0)
		return ret;

	/* Initialize the groups. */
	num_groups = DIV_ROUND_UP(rcdu->num_crtcs, 2);

	for (i = 0; i < num_groups; ++i) {
		struct rcar_du_group *rgrp = &rcdu->groups[i];

		mutex_init(&rgrp->lock);

		rgrp->dev = rcdu;
		rgrp->mmio_offset = mmio_offsets[i];
		rgrp->index = i;
		rgrp->num_crtcs = min(rcdu->num_crtcs - 2 * i, 2U);

		/*
		 * If we have more than one CRTCs in this group pre-associate
		 * the low-order planes with CRTC 0 and the high-order planes
		 * with CRTC 1 to minimize flicker occurring when the
		 * association is changed.
		 */
		rgrp->dptsr_planes = rgrp->num_crtcs > 1
				   ? (rcdu->info->gen >= 3 ? 0x04 : 0xf0)
				   : 0;

		if (!rcar_du_has(rcdu, RCAR_DU_FEATURE_VSP1_SOURCE)) {
			ret = rcar_du_planes_init(rgrp);
			if (ret < 0)
				return ret;
		}
	}

	/* Initialize the compositors. */
	if (rcar_du_has(rcdu, RCAR_DU_FEATURE_VSP1_SOURCE)) {
		ret = rcar_du_vsps_init(rcdu);
		if (ret < 0)
			return ret;
	}

	/* Create the CRTCs. */
	for (i = 0; i < rcdu->num_crtcs; ++i) {
		struct rcar_du_group *rgrp = &rcdu->groups[i / 2];

		ret = rcar_du_crtc_create(rgrp, i);
		if (ret < 0)
			return ret;

		rcar_du_pre_group_set_routing(rcdu->crtcs[i].group,
					      &rcdu->crtcs[i]);
	}

	/* Initialize the encoders. */
	ret = rcar_du_lvdsenc_init(rcdu);
	if (ret < 0)
		return ret;

	ret = rcar_du_encoders_init(rcdu);
	if (ret < 0)
		return ret;

	if (ret == 0) {
		dev_err(rcdu->dev, "error: no encoder could be initialized\n");
		return -EINVAL;
	}

	num_encoders = ret;

	/*
	 * Set the possible CRTCs and possible clones. There's always at least
	 * one way for all encoders to clone each other, set all bits in the
	 * possible clones field.
	 */
	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		struct rcar_du_encoder *renc = to_rcar_encoder(encoder);
		const struct rcar_du_output_routing *route =
			&rcdu->info->routes[renc->output];

		encoder->possible_crtcs = route->possible_crtcs;
		encoder->possible_clones = (1 << num_encoders) - 1;
	}

	drm_mode_config_reset(dev);

	drm_kms_helper_poll_init(dev);

	if (dev->mode_config.num_connector) {
		fbdev = drm_fbdev_cma_init(dev, 32,
					   dev->mode_config.num_connector);
		if (IS_ERR(fbdev))
			return PTR_ERR(fbdev);

		rcdu->fbdev = fbdev;
	} else {
		dev_info(rcdu->dev,
			 "no connector found, disabling fbdev emulation\n");
	}

	return 0;
}
