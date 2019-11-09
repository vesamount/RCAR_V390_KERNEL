/*************************************************************************/ /*
 * VSP2
 *
 * Copyright (C) 2015-2017 Renesas Electronics Corporation
 *
 * License        Dual MIT/GPLv2
 *
 * The contents of this file are subject to the MIT license as set out below.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * Alternatively, the contents of this file may be used under the terms of
 * the GNU General Public License Version 2 ("GPL") in which case the provisions
 * of GPL are applicable instead of those above.
 *
 * If you wish to allow use of your version of this file only under the terms of
 * GPL, and not to allow others to use your version of this file under the terms
 * of the MIT license, indicate your decision by deleting the provisions above
 * and replace them with the notice and other provisions required by GPL as set
 * out in the file called "GPL-COPYING" included in this distribution. If you do
 * not delete the provisions above, a recipient may use your version of this
 * file under the terms of either the MIT license or GPL.
 *
 * This License is also included in this distribution in the file called
 * "MIT-COPYING".
 *
 * EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS
 * OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 * IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 * GPLv2:
 * If you wish to use this file under the terms of GPL, following terms are
 * effective.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */ /*************************************************************************/

#ifndef __VSP2_USER_H__
#define __VSP2_USER_H__

#include <linux/types.h>
#include <linux/videodev2.h>

/*
 * Private IOCTLs
 *
 * VIDIOC_VSP2_LUT_CONFIG - Configure the lookup table
 * VIDIOC_VSP2_CLU_CONFIG - Configure the 3D lookup table
 * VIDIOC_VSP2_HGO_CONFIG - Configure the Histogram Generator -One dimension
 */

#define VIDIOC_VSP2_LUT_CONFIG \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 1, struct vsp2_lut_config)

#define VIDIOC_VSP2_CLU_CONFIG \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 2, struct vsp2_clu_config)

#define VIDIOC_VSP2_HGO_CONFIG \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 3, struct vsp2_hgo_config)

#define VIDIOC_VSP2_HGT_CONFIG \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 4, struct vsp2_hgt_config)

/*
 * Private IOCTL configs
 */

struct vsp2_lut_config {
	void		*addr;	/* Allocate memory size is tbl_num * 8 bytes. */
	unsigned short	tbl_num;	/* 1 to 256 */
	unsigned char	fxa;
};

/*
 * mode = VSP_CLU_MODE_3D      -> tbl_num = 2 to 9826
 * mode = VSP_CLU_MODE_2D      -> tbl_num = 2 to 578
 * mode = VSP_CLU_MODE_3D_AUTO -> tbl_num = 1 to 4913
 * mode = VSP_CLU_MODE_2D_AUTO -> tbl_num = 1 to 289
 */
struct vsp2_clu_config {
	unsigned char	mode;
	void		*addr;	/* Allocate memory size is tbl_num * 8 bytes. */
	unsigned char	fxa;
	unsigned short	tbl_num;	/* 1 to 9826 */
};

struct vsp2_hgo_config {
	void __user	*addr;	/* Allocate memory size is 1088 bytes. */
	unsigned short	width;	/* horizontal size */
	unsigned short	height;	/* vertical size */
	unsigned short	x_offset;
	unsigned short	y_offset;
	unsigned char	binary_mode;
	unsigned char	maxrgb_mode;
	unsigned char	step_mode;
	unsigned long	sampling;	/* sampling module */
};

struct vsp2_hue_area {
	unsigned char lower;
	unsigned char upper;
};

struct vsp2_hgt_config {
	void __user	*addr;	/* Allocate memory size is 800 bytes. */
	unsigned short	width;	/* horizontal size */
	unsigned short	height;	/* vertical size */
	struct vsp2_hue_area area[6];
	unsigned long	sampling;	/* sampling module */
};

/* HGO,HGT sampling module */
/*
 * Use the following value to sampling module in application.
 * #define VSP_SMPPT_SRC1				(0)
 * #define VSP_SMPPT_SRC2				(1)
 * #define VSP_SMPPT_SRC3				(2)
 * #define VSP_SMPPT_SRC4				(3)
 * #define VSP_SMPPT_DRC				(8)
 * #define VSP_SMPPT_SRU				(16)
 * #define VSP_SMPPT_UDS				(17)
 * #define VSP_SMPPT_LUT				(22)
 * #define VSP_SMPPT_BRU				(27)
 * #define VSP_SMPPT_CLU				(29)
 * #define VSP_SMPPT_HST				(30)
 * #define VSP_SMPPT_HSI				(31)
 * #define VSP_SMPPT_SHP				(46)
 */

enum vsp2_ctrl_id {
	VSP2_CID_COMPRESS = V4L2_CID_PRIVATE_BASE,
};

/*--------------------------------------------------------------------------
 * for debug
 *--------------------------------------------------------------------------
 */

#ifdef VSP2_DEBUG

#define VIDIOC_VSP2_DEBUG \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 99, struct vsp2_debug)

struct vsp2_debug {
	unsigned int	command;
	unsigned int	param1;
	unsigned int	param2;
	unsigned int	param3;
	unsigned int	param4;
};
#endif

#endif	/* __VSP2_USER_H__ */
