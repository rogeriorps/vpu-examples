/*
 * Copyright 2004-2013 Freescale Semiconductor, Inc.
 *
 * Copyright (c) 2006, Chips & Media.  All rights reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#undef u32
#undef u16
#undef u8
typedef unsigned long u32;
typedef unsigned short u16;
typedef unsigned char u8;

#define MAX_BUF_NUM	32
#define QUEUE_SIZE	(MAX_BUF_NUM + 1)

struct frame_buf {
	int addrY;
	int addrCb;
	int addrCr;
	int strideY;
	int strideC;
	int mvColBuf;
	vpu_mem_desc desc;
};

struct decode {
	DecHandle handle;
	PhysicalAddress phy_bsbuf_addr;
	PhysicalAddress phy_ps_buf;
	PhysicalAddress phy_slice_buf;
	PhysicalAddress phy_vp8_mbparam_buf;

	int phy_slicebuf_size;
	int phy_vp8_mbparam_size;
	u32 virt_bsbuf_addr;
	int picwidth;
	int picheight;
	int stride;
	int mjpg_fmt;
	int regfbcount;
	int minfbcount;
	int rot_buf_count;
	int extrafb;
	FrameBuffer *fb;
	struct frame_buf **pfbpool;
	struct vpu_display *disp;
	vpu_mem_desc *mvcol_memdesc;
	Rect picCropRect;
	int reorderEnable;
	int tiled2LinearEnable;
	int post_processing;

	DecReportInfo mbInfo;
	DecReportInfo mvInfo;
	DecReportInfo frameBufStat;
	DecReportInfo userData;

	struct cmd_line *cmdl;

	int decoded_field[32];
	int lastPicWidth;
	int lastPicHeight;

	int mjpgLineBufferMode;
	u32 mjpg_wr_ptr;
	u32 mjpg_rd_ptr;
	int mjpg_sc_state; /* start code FSM state */
	int mjpg_eof;
	u8 *mjpg_cached_bsbuf;

	struct frame_buf fbpool[MAX_BUF_NUM];
};

