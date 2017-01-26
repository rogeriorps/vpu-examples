/*
 * Copyright 2004-2014 Freescale Semiconductor, Inc.
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

#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "vpu_test.h"
#include <linux/mxc_v4l2.h>
#include <linux/mxcfb.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <linux/ipu.h>
#include <signal.h>
#include <getopt.h>

sigset_t sigset;
int quitflag;

int vpu_v4l_performance_test;
int vpu_test_dbg_level;

static FILE *fpFrmStatusLogfile = NULL;
static FILE *fpErrMapLogfile = NULL;
static FILE *fpQpLogfile = NULL;
static FILE *fpSliceBndLogfile = NULL;
static FILE *fpMvLogfile = NULL;
static FILE *fpUserDataLogfile = NULL;

static int isInterlacedMPEG4 = 0;

#define FN_FRAME_BUFFER_STATUS "dec_frame_buf_status.log"
#define FN_ERR_MAP_DATA "dec_error_map.log"
#define FN_QP_DATA "dec_qp.log"
#define FN_SB_DATA "dec_slice_bnd.log"
#define FN_MV_DATA "dec_mv.log"
#define FN_USER_DATA "dec_user_data.log"

#define JPG_HEADER_SIZE	     0x200

#ifdef COMBINED_VIDEO_SUPPORT
#define MAX_FRAME_WIDTH 720
#define MAX_FRAME_HEIGHT 576
#endif

static int using_config_file;

struct input_argument {
	int mode;
	pthread_t tid;
	char line[256];
	struct cmd_line cmd;
};

static struct input_argument input_arg[MAX_NUM_INSTANCE];

static int
signal_thread(void *arg)
{
	int sig;

	pthread_sigmask(SIG_BLOCK, &sigset, NULL);

	while (1) {
		sigwait(&sigset, &sig);
		if (sig == SIGINT) {
			warn_msg("Ctrl-C received\n");
		} else {
			warn_msg("Unknown signal. Still exiting\n");
		}
		quitflag = 1;
		break;
	}

	return 0;
}

// -- from framebuffer

int tiled_framebuf_base(FrameBuffer *fb, Uint32 frame_base, int strideY, int height, int mapType)
{
	int align;
	int divX, divY;
	Uint32 lum_top_base, lum_bot_base, chr_top_base, chr_bot_base;
	Uint32 lum_top_20bits, lum_bot_20bits, chr_top_20bits, chr_bot_20bits;
	int luma_top_size, luma_bot_size, chroma_top_size, chroma_bot_size;

	divX = 2;
	divY = 2;

	/*
	 * The buffers is luma top, chroma top, luma bottom and chroma bottom for
	 * tiled map type, and only 20bits for the address description, so we need
	 * to do 4K page align for each buffer.
	 */
	align = SZ_4K;
	if (mapType == TILED_FRAME_MB_RASTER_MAP) {
		/* luma_top_size means the Y size of one frame, chroma_top_size
		 * means the interleaved UV size of one frame in frame tiled map type*/
		luma_top_size = (strideY * height + align - 1) & ~(align - 1);
		chroma_top_size = (strideY / divX * height / divY * 2 + align - 1) & ~(align - 1);
		luma_bot_size = chroma_bot_size = 0;
	} else {
		/* This is FIELD_FRAME_MB_RASTER_MAP case, there are two fields */
		luma_top_size = (strideY * height / 2 + align - 1) & ~(align - 1);
		luma_bot_size = luma_top_size;
		chroma_top_size = (strideY / divX * height / divY + align - 1) & ~(align - 1);
		chroma_bot_size = chroma_top_size;
	}

	lum_top_base = (frame_base + align - 1) & ~(align -1);
	chr_top_base = lum_top_base + luma_top_size;
	if (mapType == TILED_FRAME_MB_RASTER_MAP) {
		lum_bot_base = 0;
		chr_bot_base = 0;
	} else {
		lum_bot_base = chr_top_base + chroma_top_size;
		chr_bot_base = lum_bot_base + luma_bot_size;
	}

	lum_top_20bits = lum_top_base >> 12;
	lum_bot_20bits = lum_bot_base >> 12;
	chr_top_20bits = chr_top_base >> 12;
	chr_bot_20bits = chr_bot_base >> 12;

	/*
	 * In tiled map format the construction of the buffer pointers is as follows:
	 * 20bit = addrY [31:12]: lum_top_20bits
	 * 20bit = addrY [11: 0], addrCb[31:24]: chr_top_20bits
	 * 20bit = addrCb[23: 4]: lum_bot_20bits
	 * 20bit = addrCb[ 3: 0], addrCr[31:16]: chr_bot_20bits
	 */
	fb->bufY = (lum_top_20bits << 12) + (chr_top_20bits >> 8);
	fb->bufCb = (chr_top_20bits << 24) + (lum_bot_20bits << 4) + (chr_bot_20bits >> 16);
	fb->bufCr = chr_bot_20bits << 16;

	return 0;
}

struct frame_buf *tiled_framebuf_alloc(struct frame_buf *fb, int stdMode, int format, int strideY, int height, int mvCol, int mapType)
{
	int err, align;
	int divX, divY;
	Uint32 lum_top_base, lum_bot_base, chr_top_base, chr_bot_base;
	Uint32 lum_top_20bits, lum_bot_20bits, chr_top_20bits, chr_bot_20bits;
	int luma_top_size, luma_bot_size, chroma_top_size, chroma_bot_size;

	if (fb == NULL)
		return NULL;

	divX = (format == MODE420 || format == MODE422) ? 2 : 1;
	divY = (format == MODE420 || format == MODE224) ? 2 : 1;

	memset(&(fb->desc), 0, sizeof(vpu_mem_desc));

	/*
	 * The buffers is luma top, chroma top, luma bottom and chroma bottom for
	 * tiled map type, and only 20bits for the address description, so we need
	 * to do 4K page align for each buffer.
	 */
	align = SZ_4K;
	if (mapType == TILED_FRAME_MB_RASTER_MAP) {
		/* luma_top_size means the Y size of one frame, chroma_top_size
		 * means the interleaved UV size of one frame in frame tiled map type*/
		luma_top_size = (strideY * height + align - 1) & ~(align - 1);
		chroma_top_size = (strideY / divX * height / divY * 2 + align - 1) & ~(align - 1);
		luma_bot_size = chroma_bot_size = 0;
	} else {
		/* This is FIELD_FRAME_MB_RASTER_MAP case, there are two fields */
		luma_top_size = (strideY * height / 2 + align - 1) & ~(align - 1);
		luma_bot_size = luma_top_size;
		chroma_top_size = (strideY / divX * height / divY + align - 1) & ~(align - 1);
		chroma_bot_size = chroma_top_size;
	}
	fb->desc.size = luma_top_size + chroma_top_size + luma_bot_size + chroma_bot_size;
	/* There is possible fb->desc.phy_addr in IOGetPhyMem not 4K page align,
	 * so add more SZ_4K byte here for alignment */
	fb->desc.size += align - 1;

	if (mvCol)
		fb->desc.size += strideY / divX * height / divY;

	err = IOGetPhyMem(&fb->desc);
	if (err) {
		printf("Frame buffer allocation failure\n");
		memset(&(fb->desc), 0, sizeof(vpu_mem_desc));
		return NULL;
	}

	if (IOGetVirtMem(&(fb->desc)) == -1) {
		IOFreePhyMem(&fb->desc);
		memset(&(fb->desc), 0, sizeof(vpu_mem_desc));
		return NULL;
	}

	lum_top_base = (fb->desc.phy_addr + align - 1) & ~(align -1);
	chr_top_base = lum_top_base + luma_top_size;
	if (mapType == TILED_FRAME_MB_RASTER_MAP) {
		lum_bot_base = 0;
		chr_bot_base = 0;
	} else {
		lum_bot_base = chr_top_base + chroma_top_size;
		chr_bot_base = lum_bot_base + luma_bot_size;
	}

	lum_top_20bits = lum_top_base >> 12;
	lum_bot_20bits = lum_bot_base >> 12;
	chr_top_20bits = chr_top_base >> 12;
	chr_bot_20bits = chr_bot_base >> 12;

	/*
	 * In tiled map format the construction of the buffer pointers is as follows:
	 * 20bit = addrY [31:12]: lum_top_20bits
	 * 20bit = addrY [11: 0], addrCb[31:24]: chr_top_20bits
	 * 20bit = addrCb[23: 4]: lum_bot_20bits
	 * 20bit = addrCb[ 3: 0], addrCr[31:16]: chr_bot_20bits
	 */
	fb->addrY = (lum_top_20bits << 12) + (chr_top_20bits >> 8);
	fb->addrCb = (chr_top_20bits << 24) + (lum_bot_20bits << 4) + (chr_bot_20bits >> 16);
	fb->addrCr = chr_bot_20bits << 16;
	fb->strideY = strideY;
	fb->strideC = strideY / divX;
	if (mvCol) {
		if (mapType == TILED_FRAME_MB_RASTER_MAP) {
			fb->mvColBuf = chr_top_base + chroma_top_size;
		} else {
			fb->mvColBuf = chr_bot_base + chroma_bot_size;
		}
	}

	return fb;
}


struct frame_buf *framebuf_alloc(struct frame_buf *fb, int stdMode, int format, int strideY, int height, int mvCol)
{
	int err;
	int divX, divY;

	if (fb == NULL)
		return NULL;

	divX = (format == MODE420 || format == MODE422) ? 2 : 1;
	divY = (format == MODE420 || format == MODE224) ? 2 : 1;

	memset(&(fb->desc), 0, sizeof(vpu_mem_desc));
	fb->desc.size = (strideY * height  + strideY / divX * height / divY * 2);
	if (mvCol)
		fb->desc.size += strideY / divX * height / divY;

	err = IOGetPhyMem(&fb->desc);
	if (err) {
		printf("Frame buffer allocation failure\n");
		memset(&(fb->desc), 0, sizeof(vpu_mem_desc));
		return NULL;
	}

	fb->addrY = fb->desc.phy_addr;
	fb->addrCb = fb->addrY + strideY * height;
	fb->addrCr = fb->addrCb + strideY / divX * height / divY;
	fb->strideY = strideY;
	fb->strideC =  strideY / divX;
	if (mvCol)
		fb->mvColBuf = fb->addrCr + strideY / divX * height / divY;

	if (IOGetVirtMem(&(fb->desc)) == -1) {
		IOFreePhyMem(&fb->desc);
		memset(&(fb->desc), 0, sizeof(vpu_mem_desc));
		return NULL;
	}

	return fb;
}

void framebuf_free(struct frame_buf *fb)
{
	if (fb == NULL)
		return;

	if (fb->desc.virt_uaddr) {
		IOFreeVirtMem(&fb->desc);
	}

	if (fb->desc.phy_addr) {
		IOFreePhyMem(&fb->desc);
	}

	memset(&(fb->desc), 0, sizeof(vpu_mem_desc));
}

// --- from display

static pthread_mutex_t v4l_mutex;
static pthread_cond_t ipu_cond;
static pthread_mutex_t ipu_mutex;
static int ipu_waiting = 0;
static int ipu_running = 0;

int ipu_memory_alloc(int size, int cnt, dma_addr_t paddr[], void * vaddr[], int fd_fb_alloc)
{
	int i, ret = 0;

	for (i=0;i<cnt;i++) {
		/*alloc mem from DMA zone*/
		/*input as request mem size */
		paddr[i] = size;
		if ( ioctl(fd_fb_alloc, FBIO_ALLOC, &(paddr[i])) < 0) {
			printf("Unable alloc mem from /dev/fb0\n");
			close(fd_fb_alloc);
			ret = -1;
			goto done;
		}

		vaddr[i] = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED,
				fd_fb_alloc, paddr[i]);
		if (vaddr[i] == MAP_FAILED) {
			printf("mmap failed!\n");
			ret = -1;
			goto done;
		}
	}
done:
	return ret;
}

static inline void wait_queue()
{
	pthread_mutex_lock(&ipu_mutex);
	ipu_waiting = 1;
	pthread_cond_wait(&ipu_cond, &ipu_mutex);
	pthread_mutex_unlock(&ipu_mutex);
}


void ipu_disp_loop_thread(void *arg)
{
	struct decode *dec = (struct decode *)arg;
	DecHandle handle = dec->handle;
	struct vpu_display *disp = dec->disp;
	int index = -1, disp_clr_index, tmp_idx[3] = {0,0,0}, err, mode;
	pthread_attr_t attr;

	ipu_running = 1;

	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);

	while(1) {
		disp_clr_index = index;
		index = dequeue_buf(&(disp->ipu_q));
		if (index < 0) {
			wait_queue();
			ipu_waiting = 0;
			index = dequeue_buf(&(disp->ipu_q));
			if (index < 0) {
				info_msg("thread is going to finish\n");
				break;
			}
		}

		if (disp->ncount == 0) {
			disp->input.user_def_paddr[0] = disp->ipu_bufs[index].ipu_paddr;
			/* For video de-interlace, Low/Medium motion */
			tmp_idx[0] = index;
		} else if ((disp->deinterlaced == 1) && (disp->input.motion_sel != HIGH_MOTION) && (disp->ncount == 1)) {
			disp->input.user_def_paddr[1] = disp->ipu_bufs[index].ipu_paddr;
			/* For video de-interlace, Low/Medium motion */
			tmp_idx[1] = index;
		} else if ((disp->ncount == 1) || ((disp->deinterlaced == 1) && (disp->input.motion_sel != HIGH_MOTION) &&
						 (disp->ncount == 2))) {
			disp->input.user_def_paddr[disp->ncount] = disp->ipu_bufs[index].ipu_paddr;
			mode = (disp->deinterlaced == 1) ? (OP_STREAM_MODE | TASK_VDI_VF_MODE) : (OP_STREAM_MODE | TASK_PP_MODE);
			err = mxc_ipu_lib_task_init(&(disp->input), NULL, &(disp->output), mode, &(disp->ipu_handle));
			if (err < 0) {
				err_msg("mxc_ipu_lib_task_init failed, err %d\n", err);
				quitflag = 1;
				return;
			}
			/* it only enable ipu task and finish first frame */
			err = mxc_ipu_lib_task_buf_update(&(disp->ipu_handle), 0, 0, 0, NULL, NULL);
			if (err < 0) {
				err_msg("mxc_ipu_lib_task_buf_update failed, err %d\n", err);
				quitflag = 1;
				break;
			}
			/* For video de-interlace, Low/Medium motion */
			tmp_idx[2] = index;
			if ((disp->deinterlaced == 1) && (disp->input.motion_sel != HIGH_MOTION))
				disp_clr_index = tmp_idx[0];
		} else {
			err = mxc_ipu_lib_task_buf_update(&(disp->ipu_handle), disp->ipu_bufs[index].ipu_paddr,
					0, 0, NULL, NULL);
			if (err < 0) {
				err_msg("mxc_ipu_lib_task_buf_update failed, err %d\n", err);
				quitflag = 1;
				break;
			}
			/* For video de-interlace, Low/Medium motion */
			if ((disp->deinterlaced == 1) && (disp->input.motion_sel != HIGH_MOTION)) {
				tmp_idx[0] = tmp_idx[1];
				tmp_idx[1] = tmp_idx[2];
				tmp_idx[2] = index;
				disp_clr_index = tmp_idx[0];
			}
		}

		if ((dec->cmdl->format != STD_MJPG) && (disp_clr_index >= 0) && (!disp->stopping) &&
		    !((disp->deinterlaced == 1) && (disp->input.motion_sel != HIGH_MOTION) && (disp->ncount < 2))) {
			err = vpu_DecClrDispFlag(handle, disp_clr_index);
			if (err) {
				err_msg("vpu_DecClrDispFlag failed Error code %d\n", err);
				quitflag = 1;
				break;
			}
		}
		disp->ncount++;
	}
	mxc_ipu_lib_task_uninit(&(disp->ipu_handle));
	pthread_attr_destroy(&attr);
	info_msg("Disp loop thread exit\n");
	ipu_running = 0;
	return;
}


struct vpu_display *
ipu_display_open(struct decode *dec, int nframes, struct rot rotation, Rect cropRect)
{
	int width = dec->picwidth;
	int height = dec->picheight;
	int left = cropRect.left;
	int top = cropRect.top;
	int right = cropRect.right;
	int bottom = cropRect.bottom;
	int disp_width = dec->cmdl->width;
	int disp_height = dec->cmdl->height;
	int disp_left =  dec->cmdl->loff;
	int disp_top =  dec->cmdl->toff;
	char motion_mode = dec->cmdl->vdi_motion;
	int err = 0, i;
	struct vpu_display *disp;
	struct mxcfb_gbl_alpha alpha;
	struct fb_var_screeninfo fb_var;

	disp = (struct vpu_display *)calloc(1, sizeof(struct vpu_display));
	if (disp == NULL) {
		err_msg("falied to allocate vpu_display\n");
		return NULL;
	}

	/* set alpha */
#ifdef BUILD_FOR_ANDROID
	disp->fd = open("/dev/graphics/fb0", O_RDWR, 0);
#else
	disp->fd = open("/dev/fb0", O_RDWR, 0);
#endif
	if (disp->fd < 0) {
		err_msg("unable to open fb0\n");
		free(disp);
		return NULL;
	}
	alpha.alpha = 0;
	alpha.enable = 1;
	if (ioctl(disp->fd, MXCFB_SET_GBL_ALPHA, &alpha) < 0) {
		err_msg("set alpha blending failed\n");
		close(disp->fd);
		free(disp);
		return NULL;
	}
	if ( ioctl(disp->fd, FBIOGET_VSCREENINFO, &fb_var) < 0) {
		err_msg("Get FB var info failed!\n");
		close(disp->fd);
		free(disp);
		return NULL;
	}
	if (!disp_width || !disp_height) {
		disp_width = fb_var.xres;
		disp_height = fb_var.yres;
	}

	if (rotation.rot_en) {
		if (rotation.rot_angle == 90 || rotation.rot_angle == 270) {
			i = width;
			width = height;
			height = i;
		}
		dprintf(3, "VPU rot: width = %d; height = %d\n", width, height);
	}

	/* allocate buffers, use an extra buf for init buf */
	disp->nframes = nframes;
	disp->frame_size = width*height*3/2;
	for (i=0;i<nframes;i++) {
		err = ipu_memory_alloc(disp->frame_size, 1, &(disp->ipu_bufs[i].ipu_paddr),
				&(disp->ipu_bufs[i].ipu_vaddr), disp->fd);
		if ( err < 0) {
			err_msg("ipu_memory_alloc failed\n");
			free(disp);
			return NULL;
		}
	}

	memset(&(disp->ipu_handle), 0, sizeof(ipu_lib_handle_t));
	memset(&(disp->input), 0, sizeof(ipu_lib_input_param_t));
	memset(&(disp->output), 0, sizeof(ipu_lib_output_param_t));

        disp->input.width = width;
        disp->input.height = height;
	disp->input.input_crop_win.pos.x = left;
	disp->input.input_crop_win.pos.y = top;
	disp->input.input_crop_win.win_w = right - left;
	disp->input.input_crop_win.win_h = bottom - top;

	/* Set VDI motion algorithm. */
	if (motion_mode) {
		if (motion_mode == 'h') {
			disp->input.motion_sel = HIGH_MOTION;
		} else if (motion_mode == 'l') {
			disp->input.motion_sel = LOW_MOTION;
		} else if (motion_mode == 'm') {
			disp->input.motion_sel = MED_MOTION;
		} else {
			disp->input.motion_sel = MED_MOTION;
			info_msg("%c unknown motion mode, medium, the default is used\n", motion_mode);
		}
	}

	if (dec->cmdl->chromaInterleave == 0)
		disp->input.fmt = V4L2_PIX_FMT_YUV420;
	else
		disp->input.fmt = V4L2_PIX_FMT_NV12;

	disp->output.width = disp_width;
	disp->output.height = disp_height;
	disp->output.fmt = V4L2_PIX_FMT_UYVY;
	if (rotation.ext_rot_en && (rotation.rot_angle != 0)) {
		if (rotation.rot_angle == 90)
			disp->output.rot = IPU_ROTATE_90_RIGHT;
		else if (rotation.rot_angle == 180)
			disp->output.rot = IPU_ROTATE_180;
		else if (rotation.rot_angle == 270)
			disp->output.rot = IPU_ROTATE_90_LEFT;
	}
	disp->output.fb_disp.pos.x = disp_left;
	disp->output.fb_disp.pos.y = disp_top;
	disp->output.show_to_fb = 1;
	disp->output.fb_disp.fb_num = 2;

	info_msg("Display to %d %d, top offset %d, left offset %d\n",
			disp_width, disp_height, disp_top, disp_left);

	disp->ipu_q.tail = disp->ipu_q.head = 0;
	disp->stopping = 0;

	dec->disp = disp;
	pthread_mutex_init(&ipu_mutex, NULL);
	pthread_cond_init(&ipu_cond, NULL);

	/* start disp loop thread */
	pthread_create(&(disp->ipu_disp_loop_thread), NULL, (void *)ipu_disp_loop_thread, (void *)dec);

	return disp;
}


static int
calculate_ratio(int width, int height, int maxwidth, int maxheight)
{
	int i, tmp, ratio, compare;

	i = ratio = 1;
	if (width >= height) {
		tmp = width;
		compare = maxwidth;
	} else {
		tmp = height;
		compare = maxheight;
	}

	if (width <= maxwidth && height <= maxheight) {
		ratio = 1;
	} else {
		while (tmp > compare) {
			ratio = (1 << i);
			tmp /= ratio;
			i++;
		}
	}

	return ratio;
}


void v4l_free_bufs(int n, struct vpu_display *disp)
{
	int i;
	struct v4l_buf *buf;
	struct v4l_specific_data *v4l_rsd = (struct v4l_specific_data *)disp->render_specific_data;

	for (i = 0; i < n; i++) {
		if (v4l_rsd->buffers[i] != NULL) {
			buf = v4l_rsd->buffers[i];
			if (buf->start > 0)
				munmap(buf->start, buf->length);

			free(buf);
			v4l_rsd->buffers[i] = NULL;
		}
	}
	free(v4l_rsd);
}

/* The thread for display in performance test with v4l */
void v4l_disp_loop_thread(void *arg)
{
	struct decode *dec = (struct decode *)arg;
	struct vpu_display *disp = dec->disp;
	pthread_attr_t attr;
	struct timespec ts;
	int error_status = 0, ret;
	struct v4l2_buffer buffer;

	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);

	while (!error_status && !quitflag) {
		/* Use timed wait here */
		do {
			clock_gettime(CLOCK_REALTIME, &ts);
			ts.tv_nsec +=100000000; // 100ms
			if (ts.tv_nsec >= 1000000000)
			{
				ts.tv_sec += ts.tv_nsec / 1000000000;
				ts.tv_nsec %= 1000000000;
			}
		} while ((sem_timedwait(&disp->avaiable_dequeue_frame,
			 &ts) != 0) && !quitflag);

		if (quitflag)
			break;

		buffer.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		buffer.memory = V4L2_MEMORY_MMAP;
		ret = ioctl(disp->fd, VIDIOC_DQBUF, &buffer);
		if (ret < 0) {
			err_msg("VIDIOC_DQBUF failed\n");
			error_status = 1;
		}
		queue_buf(&(disp->released_q), buffer.index);

		pthread_mutex_lock(&v4l_mutex);
		disp->queued_count--;
		disp->dequeued_count++;
		pthread_mutex_unlock(&v4l_mutex);
		sem_post(&disp->avaiable_decoding_frame);
	}
	pthread_attr_destroy(&attr);
	return;
}

struct vpu_display *
v4l_display_open(struct decode *dec, int nframes, struct rot rotation, Rect cropRect)
{
	int width = dec->picwidth;
	int height = dec->picheight;
	int left = cropRect.left;
	int top = cropRect.top;
	int right = cropRect.right;
	int bottom = cropRect.bottom;
	int disp_width = dec->cmdl->width;
	int disp_height = dec->cmdl->height;
	int disp_left =  dec->cmdl->loff;
	int disp_top =  dec->cmdl->toff;
	int fd = -1, err = 0, out = 0, i = 0;
	char v4l_device[80], node[8];
	struct v4l2_cropcap cropcap = {0};
	struct v4l2_crop crop = {0};
	struct v4l2_framebuffer fb = {0};
	struct v4l2_format fmt = {0};
	struct v4l2_requestbuffers reqbuf = {0};
	struct v4l2_mxc_offset off = {0};
	struct v4l2_rect icrop = {0};
	struct vpu_display *disp;
	struct v4l_specific_data *v4l_rsd;
	int fd_fb;
	char *tv_mode, *test_mode;
	char motion_mode = dec->cmdl->vdi_motion;
	struct mxcfb_gbl_alpha alpha;

	int ratio = 1;

	if (cpu_is_mx27()) {
		out = 0;
	} else {
		out = 3;
#ifdef BUILD_FOR_ANDROID
		fd_fb = open("/dev/graphics/fb0", O_RDWR, 0);
#else
		fd_fb = open("/dev/fb0", O_RDWR, 0);
#endif
		if (fd_fb < 0) {
			err_msg("unable to open fb0\n");
			return NULL;
		}
		alpha.alpha = 0;
		alpha.enable = 1;
		if (ioctl(fd_fb, MXCFB_SET_GBL_ALPHA, &alpha) < 0) {
			err_msg("set alpha blending failed\n");
			return NULL;
		}
		close(fd_fb);
	}
	dprintf(3, "rot_en:%d; rot_angle:%d; ext_rot_en:%d\n", rotation.rot_en,
			rotation.rot_angle, rotation.ext_rot_en);

	tv_mode = getenv("VPU_TV_MODE");

	if (tv_mode) {
		err = system("/bin/echo 1 > /sys/class/graphics/fb1/blank");
		if (!strcmp(tv_mode, "NTSC")) {
			err = system("/bin/echo U:720x480i-60 > /sys/class/graphics/fb1/mode");
			out = 5;
		} else if (!strcmp(tv_mode, "PAL")) {
			err = system("/bin/echo U:720x576i-50 > /sys/class/graphics/fb1/mode");
			out = 5;
		} else if (!strcmp(tv_mode, "720P")) {
			err = system("/bin/echo U:1280x720p-60 > /sys/class/graphics/fb1/mode");
			out = 5;
		} else {
			out = 3;
			warn_msg("VPU_TV_MODE should be set to NTSC, PAL, or 720P.\n"
				 "\tDefault display is LCD if not set this environment "
				 "or set wrong string.\n");
		}
		err = system("/bin/echo 0 > /sys/class/graphics/fb1/blank");
		if (err == -1) {
			warn_msg("set tv mode error\n");
		}
		/* make sure tvout init done */
		sleep(2);
	}

	if (rotation.rot_en) {
		if (rotation.rot_angle == 90 || rotation.rot_angle == 270) {
			i = width;
			width = height;
			height = i;
		}
		dprintf(3, "VPU rot: width = %d; height = %d\n", width, height);
	}

	disp = (struct vpu_display *)calloc(1, sizeof(struct vpu_display));
       	if (disp == NULL) {
		err_msg("falied to allocate vpu_display\n");
		return NULL;
	}

	if (!dec->cmdl->video_node) {
		if (cpu_is_mx6x())
			dec->cmdl->video_node = 17; /* fg for mx6x */
		else
			dec->cmdl->video_node = 16;
	}
	sprintf(node, "%d", dec->cmdl->video_node);
	strcpy(v4l_device, "/dev/video");
	strcat(v4l_device, node);

	fd = open(v4l_device, O_RDWR, 0);
	if (fd < 0) {
		err_msg("unable to open %s\n", v4l_device);
		goto err;
	}

	info_msg("v4l output to %s\n", v4l_device);

	if (!cpu_is_mx6x()) {
		err = ioctl(fd, VIDIOC_S_OUTPUT, &out);
		if (err < 0) {
			err_msg("VIDIOC_S_OUTPUT failed\n");
			goto err;
		}
	}

	cropcap.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	err = ioctl(fd, VIDIOC_CROPCAP, &cropcap);
	if (err < 0) {
		err_msg("VIDIOC_CROPCAP failed\n");
		goto err;
	}
	dprintf(1, "cropcap.bounds.width = %d\n\tcropcap.bound.height = %d\n\t" \
		"cropcap.defrect.width = %d\n\tcropcap.defrect.height = %d\n",
		cropcap.bounds.width, cropcap.bounds.height,
		cropcap.defrect.width, cropcap.defrect.height);

	if (rotation.ext_rot_en == 0) {
		ratio = calculate_ratio(width, height, cropcap.bounds.width,
				cropcap.bounds.height);
		dprintf(3, "VPU rot: ratio = %d\n", ratio);
	}

	crop.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	crop.c.top = disp_top;
	crop.c.left = disp_left;
	crop.c.width = width / ratio;
	crop.c.height = height / ratio;

	if ((disp_width != 0) && (disp_height!= 0 )) {
		crop.c.width = disp_width;
		crop.c.height = disp_height;
	} else if (!cpu_is_mx27()) {
		crop.c.width = cropcap.bounds.width;
		crop.c.height = cropcap.bounds.height;
	}

	info_msg("Display to %d %d, top offset %d, left offset %d\n",
			crop.c.width, crop.c.height, disp_top, disp_left);

	dprintf(1, "crop.c.width/height: %d/%d\n", crop.c.width, crop.c.height);

	err = ioctl(fd, VIDIOC_S_CROP, &crop);
	if (err < 0) {
		err_msg("VIDIOC_S_CROP failed\n");
		goto err;
	}

	/* Set VDI motion algorithm. */
	if (motion_mode) {
		struct v4l2_control ctrl;
		ctrl.id = V4L2_CID_MXC_MOTION;
		if (motion_mode == 'h') {
			ctrl.value = HIGH_MOTION;
		} else if (motion_mode == 'l') {
			ctrl.value = LOW_MOTION;
		} else if (motion_mode == 'm') {
			ctrl.value = MED_MOTION;
		} else {
			ctrl.value = MED_MOTION;
			info_msg("%c unknown motion mode, medium, the default is used\n",motion_mode);
		}
		err = ioctl(fd, VIDIOC_S_CTRL, &ctrl);
		if (err < 0) {
			err_msg("VIDIOC_S_CTRL failed\n");
			goto err;
		}
	}

	if (cpu_is_mx6x()) {
		/* Set rotation via new V4L2 interface on 2.6.38 kernel */
		struct v4l2_control ctrl;

		ctrl.id = V4L2_CID_ROTATE;
		if (rotation.ext_rot_en)
			ctrl.value = rotation.rot_angle;
		else
			ctrl.value = 0;
		err = ioctl(fd, VIDIOC_S_CTRL, &ctrl);
		if (err < 0) {
			err_msg("VIDIOC_S_CTRL failed\n");
			goto err;
		}
	} else if (rotation.ext_rot_en && (rotation.rot_angle != 0)) {
		/* Set rotation via V4L2 i/f */
		struct v4l2_control ctrl;
		ctrl.id = V4L2_CID_PRIVATE_BASE;
		if (rotation.rot_angle == 90)
			ctrl.value = V4L2_MXC_ROTATE_90_RIGHT;
		else if (rotation.rot_angle == 180)
			ctrl.value = V4L2_MXC_ROTATE_180;
		else if (rotation.rot_angle == 270)
			ctrl.value = V4L2_MXC_ROTATE_90_LEFT;
		err = ioctl(fd, VIDIOC_S_CTRL, &ctrl);
		if (err < 0) {
			err_msg("VIDIOC_S_CTRL failed\n");
			goto err;
		}
	} else {
		struct v4l2_control ctrl;
		ctrl.id = V4L2_CID_PRIVATE_BASE;
		ctrl.value = 0;
		err = ioctl(fd, VIDIOC_S_CTRL, &ctrl);
		if (err < 0) {
			err_msg("VIDIOC_S_CTRL failed\n");
			goto err;
		}
	}

	if (cpu_is_mx27()) {
		fb.capability = V4L2_FBUF_CAP_EXTERNOVERLAY;
		fb.flags = V4L2_FBUF_FLAG_PRIMARY;

		err = ioctl(fd, VIDIOC_S_FBUF, &fb);
		if (err < 0) {
			err_msg("VIDIOC_S_FBUF failed\n");
			goto err;
		}
	}

	fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

	/*
	 * Just consider one case:
	 * (top,left) = (0,0)
	 */
	if (top || left) {
		err_msg("This case is not covered in this demo for simplicity:\n"
			"croping rectangle (top, left) != (0, 0); "
			"top/left = %d/%d\n", top, left);
		goto err;
	} else if (right || bottom) {
		if (cpu_is_mx6x()) {
			/* This is aligned with new V4L interface on 2.6.38 kernel */
			fmt.fmt.pix.width = width;
			fmt.fmt.pix.height = height;
			icrop.left = left;
			icrop.top = top;
			icrop.width = right - left;
			icrop.height = bottom - top;
			fmt.fmt.pix.priv =  (unsigned long)&icrop;
		} else {
			fmt.fmt.pix.width = right - left;
			fmt.fmt.pix.height = bottom - top;
			fmt.fmt.pix.bytesperline = width;
			off.u_offset = width * height;
			off.v_offset = off.u_offset + width * height / 4;
			fmt.fmt.pix.priv = (unsigned long) &off;
			fmt.fmt.pix.sizeimage = width * height * 3 / 2;
		}
	} else {
		fmt.fmt.pix.width = width;
		fmt.fmt.pix.height = height;
		fmt.fmt.pix.bytesperline = width;
	}
	dprintf(1, "fmt.fmt.pix.width = %d\n\tfmt.fmt.pix.height = %d\n",
				fmt.fmt.pix.width, fmt.fmt.pix.height);

	fmt.fmt.pix.field = V4L2_FIELD_ANY;

	if (dec->cmdl->mapType == LINEAR_FRAME_MAP) {
		if (dec->cmdl->chromaInterleave == 0) {
			if (dec->mjpg_fmt == MODE420)
				fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
			else if (dec->mjpg_fmt == MODE422)
				fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV422P;
			else {
				err_msg("Display cannot support this MJPG format\n");
				goto err;
			}
		} else {
			if (dec->mjpg_fmt == MODE420) {
				info_msg("Display: NV12\n");
				fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
			}
			else if (dec->mjpg_fmt == MODE422) {
				info_msg("Display: NV16\n");
				fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV16;
			}
			else {
				err_msg("Display cannot support this MJPG format\n");
				goto err;
			}
		}
	}
	else if (dec->cmdl->mapType == TILED_FRAME_MB_RASTER_MAP) {
		fmt.fmt.pix.pixelformat = IPU_PIX_FMT_TILED_NV12;
	}
	else if (dec->cmdl->mapType == TILED_FIELD_MB_RASTER_MAP) {
		fmt.fmt.pix.pixelformat = IPU_PIX_FMT_TILED_NV12F;
	}
	else {
		err_msg("Display cannot support mapType = %d\n", dec->cmdl->mapType);
		goto err;
	}
	err = ioctl(fd, VIDIOC_S_FMT, &fmt);
	if (err < 0) {
		err_msg("VIDIOC_S_FMT failed\n");
		goto err;
	}

	err = ioctl(fd, VIDIOC_G_FMT, &fmt);
	if (err < 0) {
		err_msg("VIDIOC_G_FMT failed\n");
		goto err;
	}

	reqbuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	reqbuf.memory = V4L2_MEMORY_MMAP;
	reqbuf.count = nframes;

	err = ioctl(fd, VIDIOC_REQBUFS, &reqbuf);
	if (err < 0) {
		err_msg("VIDIOC_REQBUFS failed\n");
		goto err;
	}

	if (reqbuf.count < nframes) {
		err_msg("VIDIOC_REQBUFS: not enough buffers\n");
		goto err;
	}

	v4l_rsd = calloc(1, sizeof(struct v4l_specific_data));
	disp->render_specific_data = (void *)v4l_rsd;

	for (i = 0; i < nframes; i++) {
		struct v4l2_buffer buffer = {0};
		struct v4l_buf *buf;

		buf = calloc(1, sizeof(struct v4l_buf));
		if (buf == NULL) {
			v4l_free_bufs(i, disp);
			goto err;
		}

		v4l_rsd->buffers[i] = buf;

		buffer.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		buffer.memory = V4L2_MEMORY_MMAP;
		buffer.index = i;

		err = ioctl(fd, VIDIOC_QUERYBUF, &buffer);
		if (err < 0) {
			err_msg("VIDIOC_QUERYBUF: not enough buffers\n");
			v4l_free_bufs(i, disp);
			goto err;
		}
		buf->start = mmap(NULL, buffer.length, PROT_READ | PROT_WRITE,
				MAP_SHARED, fd, buffer.m.offset);

		if (cpu_is_mx6x()) {
			/*
			 * Workaround for new V4L interface change, this change
			 * will be removed after V4L driver is updated for this.
			 * Need to call QUERYBUF ioctl again after mmap.
			 */
			err = ioctl(fd, VIDIOC_QUERYBUF, &buffer);
			if (err < 0) {
				err_msg("VIDIOC_QUERYBUF: not enough buffers\n");
				v4l_free_bufs(i, disp);
				goto err;
			}
		}
		buf->offset = buffer.m.offset;
		buf->length = buffer.length;
		dprintf(3, "V4L2buf phy addr: %08x, size = %d\n",
				    (unsigned int)buf->offset, buf->length);

		if (buf->start == MAP_FAILED) {
			err_msg("mmap failed\n");
			v4l_free_bufs(i, disp);
			goto err;
		}

	}

	disp->fd = fd;
	disp->nframes = nframes;

	/*
	 * Use environment VIDEO_PERFORMANCE_TEST to select different mode.
	 * When doing performance test, video decoding and display are in different
	 * threads and default display fps is controlled by cmd. Display will
	 * show the frame immediately if user doesn't input fps with -a option.
	 * This is different from normal unit test.
	 */
	test_mode = getenv("VIDEO_PERFORMANCE_TEST");
	if (test_mode && !strcmp(test_mode, "1") && (dec->cmdl->dst_scheme == PATH_V4L2))
		vpu_v4l_performance_test = 1;

	if (vpu_v4l_performance_test) {
		dec->disp = disp;
		disp->released_q.tail = disp->released_q.head = 0;
		sem_init(&disp->avaiable_decoding_frame, 0,
			    dec->regfbcount - dec->minfbcount);
		sem_init(&disp->avaiable_dequeue_frame, 0, 0);
		pthread_mutex_init(&v4l_mutex, NULL);
		/* start v4l disp loop thread */
		pthread_create(&(disp->disp_loop_thread), NULL,
				    (void *)v4l_disp_loop_thread, (void *)dec);
	}

	return disp;
err:
	close(fd);
	free(disp);
	return NULL;
}

void v4l_display_close(struct vpu_display *disp)
{
	int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

	if (disp) {
		if (vpu_v4l_performance_test) {
			quitflag = 1;
			pthread_join(disp->disp_loop_thread, NULL);
			sem_destroy(&disp->avaiable_decoding_frame);
			sem_destroy(&disp->avaiable_dequeue_frame);
		}

		ioctl(disp->fd, VIDIOC_STREAMOFF, &type);
		v4l_free_bufs(disp->nframes, disp);
		close(disp->fd);
		free(disp);
	}
}

void ipu_memory_free(int size, int cnt, dma_addr_t paddr[], void * vaddr[], int fd_fb_alloc)
{
	int i;

	for (i=0;i<cnt;i++) {
		if (vaddr[i])
			munmap(vaddr[i], size);
		if (paddr[i])
			ioctl(fd_fb_alloc, FBIO_FREE, &(paddr[i]));
	}
}

static inline void wakeup_queue()
{
	pthread_cond_signal(&ipu_cond);
}

void ipu_display_close(struct vpu_display *disp)
{
	int i;

	disp->stopping = 1;
	disp->deinterlaced = 0;
	while(ipu_running && ((queue_size(&(disp->ipu_q)) > 0) || !ipu_waiting)) usleep(10000);
	if (ipu_running) {
		wakeup_queue();
		info_msg("Join disp loop thread\n");
		pthread_join(disp->ipu_disp_loop_thread, NULL);
	}
	pthread_mutex_destroy(&ipu_mutex);
	pthread_cond_destroy(&ipu_cond);
	for (i=0;i<disp->nframes;i++)
		ipu_memory_free(disp->frame_size, 1, &(disp->ipu_bufs[i].ipu_paddr),
				&(disp->ipu_bufs[i].ipu_vaddr), disp->fd);
	close(disp->fd);
	free(disp);
}

int ipu_put_data(struct vpu_display *disp, int index, int field, int fps)
{
	/*TODO: ipu lib dose not support fps control yet*/

	disp->ipu_bufs[index].field = field;
	if (field == V4L2_FIELD_TOP || field == V4L2_FIELD_BOTTOM ||
	    field == V4L2_FIELD_INTERLACED_TB ||
	    field == V4L2_FIELD_INTERLACED_BT)
		disp->deinterlaced = 1;
	queue_buf(&(disp->ipu_q), index);
	wakeup_queue();

	return 0;
}

int v4l_get_buf(struct decode *dec)
{
	int index = -1;
	struct timespec ts;
	struct vpu_display *disp;

	disp = dec->disp;
	/* Block here to wait avaiable_decoding_frame */
	if (vpu_v4l_performance_test) {
		do {
			clock_gettime(CLOCK_REALTIME, &ts);
			ts.tv_nsec +=100000000; // 100ms
			if (ts.tv_nsec >= 1000000000)
			{
				ts.tv_sec += ts.tv_nsec / 1000000000;
				ts.tv_nsec %= 1000000000;
			}
		} while ((sem_timedwait(&disp->avaiable_decoding_frame,
			    &ts) != 0) && !quitflag);
	}

	if (disp->dequeued_count > 0) {
		index = dequeue_buf(&(disp->released_q));
		pthread_mutex_lock(&v4l_mutex);
		disp->dequeued_count--;
		pthread_mutex_unlock(&v4l_mutex);
	}
	return index;
}



int v4l_put_data(struct decode *dec, int index, int field, int fps)
{
	struct timeval tv;
	int err, type, threshold;
	struct v4l2_format fmt = {0};
	struct vpu_display *disp;
	struct v4l_specific_data *v4l_rsd;

	disp = dec->disp;
	v4l_rsd = (struct v4l_specific_data *)disp->render_specific_data;

	v4l_rsd->buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	v4l_rsd->buf.memory = V4L2_MEMORY_MMAP;

	/* query buffer info */
	v4l_rsd->buf.index = index;
	err = ioctl(disp->fd, VIDIOC_QUERYBUF, &v4l_rsd->buf);
	if (err < 0) {
		err_msg("VIDIOC_QUERYBUF failed\n");
		goto err;
	}

	if (disp->ncount == 0) {
		gettimeofday(&tv, 0);
		v4l_rsd->buf.timestamp.tv_sec = tv.tv_sec;
		v4l_rsd->buf.timestamp.tv_usec = tv.tv_usec;

		disp->sec = tv.tv_sec;
		disp->usec = tv.tv_usec;
	}

	if (disp->ncount > 0) {
		if (fps != 0) {
			disp->usec += (1000000 / fps);
			if (disp->usec >= 1000000) {
				disp->sec += 1;
				disp->usec -= 1000000;
			}

			v4l_rsd->buf.timestamp.tv_sec = disp->sec;
			v4l_rsd->buf.timestamp.tv_usec = disp->usec;
		} else {
			gettimeofday(&tv, 0);
			v4l_rsd->buf.timestamp.tv_sec = tv.tv_sec;
			v4l_rsd->buf.timestamp.tv_usec = tv.tv_usec;
		}
	}

	v4l_rsd->buf.index = index;
	v4l_rsd->buf.field = field;

	err = ioctl(disp->fd, VIDIOC_QBUF, &v4l_rsd->buf);
	if (err < 0) {
		err_msg("VIDIOC_QBUF failed\n");
		goto err;
	}

	if (vpu_v4l_performance_test) {
		/* Use mutex to protect queued_count in multi-threads */
		pthread_mutex_lock(&v4l_mutex);
		disp->queued_count++;
		pthread_mutex_unlock(&v4l_mutex);
	} else
		disp->queued_count++;

	if (disp->ncount == 1) {
		if ((v4l_rsd->buf.field == V4L2_FIELD_TOP) ||
		    (v4l_rsd->buf.field == V4L2_FIELD_BOTTOM) ||
		    (v4l_rsd->buf.field == V4L2_FIELD_INTERLACED_TB) ||
		    (v4l_rsd->buf.field == V4L2_FIELD_INTERLACED_BT)) {
			/* For interlace feature */
			fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			err = ioctl(disp->fd, VIDIOC_G_FMT, &fmt);
			if (err < 0) {
				err_msg("VIDIOC_G_FMT failed\n");
				goto err;
			}
			if ((v4l_rsd->buf.field == V4L2_FIELD_TOP) ||
			    (v4l_rsd->buf.field == V4L2_FIELD_BOTTOM))
				fmt.fmt.pix.field = V4L2_FIELD_ALTERNATE;
			else
				fmt.fmt.pix.field = field;
			fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			err = ioctl(disp->fd, VIDIOC_S_FMT, &fmt);
			if (err < 0) {
				err_msg("VIDIOC_S_FMT failed\n");
				goto err;
			}
		}
		type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		err = ioctl(disp->fd, VIDIOC_STREAMON, &type);
		if (err < 0) {
			err_msg("VIDIOC_STREAMON failed\n");
			goto err;
		}
	}

	disp->ncount++;

	if (dec->post_processing)
		threshold = dec->rot_buf_count - 1;
	else
		threshold = dec->regfbcount - dec->minfbcount;
	if (disp->queued_count > threshold) {
		if (vpu_v4l_performance_test) {
			sem_post(&disp->avaiable_dequeue_frame);
		} else {
			v4l_rsd->buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			v4l_rsd->buf.memory = V4L2_MEMORY_MMAP;
			err = ioctl(disp->fd, VIDIOC_DQBUF, &v4l_rsd->buf);
			if (err < 0) {
				err_msg("VIDIOC_DQBUF failed\n");
				goto err;
			}
			disp->queued_count--;
		}
	}
	else
		v4l_rsd->buf.index = -1;

	return 0;

err:
	return -1;
}


//


// ---from vpu utils

/* Our custom header */
struct nethdr {
	int seqno;
	int iframe;
	int len;
};

int	/* write n bytes to a file descriptor */
fwriten(int fd, void *vptr, size_t n)
{
	int nleft;
	int nwrite;
	char  *ptr;

	ptr = vptr;
	nleft = n;
	while (nleft > 0) {
		if ( (nwrite = write(fd, ptr, nleft)) <= 0) {
			perror("fwrite: ");
			return (-1);			/* error */
		}

		nleft -= nwrite;
		ptr   += nwrite;
	}

	return (n);
} /* end fwriten */

int	/* Read n bytes from a file descriptor */
freadn(int fd, void *vptr, size_t n)
{
	int nleft = 0;
	int nread = 0;
	char  *ptr;

	ptr = vptr;
	nleft = n;
	while (nleft > 0) {
		if ( (nread = read(fd, ptr, nleft)) <= 0) {
			if (nread == 0)
				return (n - nleft);

			perror("read");
			return (-1);			/* error EINTR XXX */
		}

		nleft -= nread;
		ptr   += nread;
	}

	return (n - nleft);
}

/* Receive data from udp socket */
static int
udp_recv(struct cmd_line *cmd, int sd, char *buf, int n)
{
	int nleft, nread, nactual, nremain, ntotal = 0;
	char *ptr;
	struct nethdr *net_h;
	int hdrlen = sizeof(struct nethdr);
	fd_set rfds;
	struct timeval tv;

	if (cmd->nlen) {
		/* No more data to be received */
		if (cmd->nlen == -1) {
			return 0;
		}

		/* There was some data pending from the previous recvfrom.
		 * copy that data into the buffer
		 */
		if (cmd->nlen > n) {
			memcpy(buf, (cmd->nbuf + cmd->noffset), n);
			cmd->nlen -= n;
			cmd->noffset += n;
			return n;
		}

		memcpy(buf, (cmd->nbuf + cmd->noffset), cmd->nlen);
		ptr = buf + cmd->nlen;
		nleft = n - cmd->nlen;
		ntotal = cmd->nlen;
		cmd->nlen = 0;
	} else {
		ptr = buf;
		nleft = n;
	}

	while (nleft > 0) {
		tv.tv_sec = 0;
		tv.tv_usec = 3000;

		FD_ZERO(&rfds);
		FD_SET(sd, &rfds);

		nread = select(sd + 1, &rfds, NULL, NULL, &tv);
		if (nread < 0) {
			perror("select");
			return -1;
		}

		/* timeout */
		if (nread == 0) {
			if (quitflag) {
				n = ntotal;
				break;
			}

			/* wait for complete buffer to be full */
			if (cmd->complete) {
				continue;
			}

			if (ntotal == 0) {
			       return -EAGAIN;
			}

			n = ntotal;
			break;
		}

		nread = recvfrom(sd, cmd->nbuf, DEFAULT_PKT_SIZE, 0,
					NULL, NULL);
		if (nread < 0) {
			perror("recvfrom");
			return -1;
		}

		/* get our custom header */
		net_h = (struct nethdr *)cmd->nbuf;
		dprintf(4, "RX: seqno %d, neth seqno %d, iframe %d, len %d\n",
				cmd->seq_no, net_h->seqno, net_h->iframe, net_h->len);
		if (net_h->len == 0) {
			/* zero length data means no more data will be
			 * received */
			cmd->nlen = -1;
			return (n - nleft);
		}

		nactual = (nread - hdrlen);
		if (net_h->len != nactual) {
			warn_msg("length mismatch\n");
		}

		if (cmd->seq_no++ != net_h->seqno) {
			/* read till we get an I frame */
			if (net_h->iframe == 1) {
				cmd->seq_no = net_h->seqno + 1;
			} else {
				continue;
			}
		}

		/* check if there is space in user buffer to copy all the
		 * received data
		 */
		if (nactual > nleft) {
			nremain = nleft;
			cmd->nlen = nactual - nleft;
			cmd->noffset = (nleft + hdrlen);
		} else {
			nremain = nactual;
		}

		memcpy(ptr, (cmd->nbuf + hdrlen), nremain);
		ntotal += nremain;
		nleft -= nremain;
		ptr += nremain;
	}

	return (n);
}


int
vpu_read(struct cmd_line *cmd, char *buf, int n)
{
	int fd = cmd->src_fd;
	printf("\n file descriptor 0x%x\n", fd);
	if (cmd->src_scheme == PATH_NET) {
		return udp_recv(cmd, fd, buf, n);
	}

#ifdef _FSL_VTS_
    if ( NULL != g_pfnVTSProbe )
    {
        DUT_STREAM_READ sFileRead;
        sFileRead.hBitstream = fd;
        sFileRead.pBitstreamBuf = buf;
        sFileRead.iLength = n;
        return g_pfnVTSProbe( E_READ_BITSTREAM, &sFileRead );
    }
    else
    {
        return 0;
    }
#else
	return freadn(fd, buf, n);
#endif
}

// --------

void SaveFrameBufStat(u8 *frmStatusBuf, int size, int DecNum)
{

	int i;

	if (fpFrmStatusLogfile == NULL) {
		fpFrmStatusLogfile = fopen(FN_FRAME_BUFFER_STATUS, "w+");
	}

	fprintf(fpFrmStatusLogfile, "FRAME [%1d]\n", DecNum);

	for (i = 0; i < size; i++) {
		fprintf(fpFrmStatusLogfile, "[%d] %d ", i*2, ((frmStatusBuf[i]>>4)&0x0F));
		fprintf(fpFrmStatusLogfile, "[%d] %d ", (i*2)+1, (frmStatusBuf[i]&0x0F));
	}
	fprintf(fpFrmStatusLogfile, "\n");
	fflush(fpFrmStatusLogfile);
}


void SaveMB_Para(u8 *mbParaBuf, int size, int DecNum)
{

	int i;

	if (DecNum == 1)
		DecNum = DecNum;

	if (fpErrMapLogfile == NULL)
		fpErrMapLogfile = fopen(FN_ERR_MAP_DATA, "w+");
	if (fpQpLogfile == NULL)
		fpQpLogfile = fopen(FN_QP_DATA, "w+");
	if (fpSliceBndLogfile == NULL)
		fpSliceBndLogfile = fopen(FN_SB_DATA, "w+");

	fprintf(fpQpLogfile, "FRAME [%1d]\n", DecNum);
	fprintf(fpSliceBndLogfile, "FRAME [%1d]\n", DecNum);
	fprintf(fpErrMapLogfile, "FRAME [%1d]\n", DecNum);


	for (i = 0; i < size; i++) {
		fprintf(fpQpLogfile, "MbAddr[%4d]: MbQs[%2d]\n", i, mbParaBuf[i]&0x3F);
		fprintf(fpSliceBndLogfile, "MbAddr[%4d]: Slice Boundary Flag[%1d]\n", i, (mbParaBuf[i]>>6)&1);
		fprintf(fpErrMapLogfile, "MbAddr[%4d]: ErrMap[%1d]\n", i, (mbParaBuf[i]>>7)&1);
	}

	fflush(fpQpLogfile);
	fflush(fpSliceBndLogfile);
	fflush(fpErrMapLogfile);
}

void SaveMvPara(u8 *mvParaBuf, int size, int mvNumPerMb, int mbNumX, int DecNum)
{

	int i, j;
	short mvX, mvY;
	u8 *mvDatabuf;

	if (fpMvLogfile == 0) {
		fpMvLogfile = fopen(FN_MV_DATA, "w+");
	}

	fprintf(fpMvLogfile, "FRAME [%1d]\n", DecNum);

	for (i = 0; i < size; i++) {
		for (j = 0; j < mvNumPerMb; j++) {
			mvDatabuf = (mvParaBuf + (i*mvNumPerMb + j)*4);
			mvX = (short)((mvDatabuf[0]<<8) | (mvDatabuf[1]<<0));
			mvY = (short)((mvDatabuf[2]<<8) | (mvDatabuf[3]<<0));

			if (!(mvX & 0x8000)){
				/* Intra MB */
				mvX = 0;
				mvY = 0;

				if (j == 0 )
					fprintf(fpMvLogfile, "MbAddr[%4d:For ]: Avail[0] Mv[%5d:%5d]\n", i, mvX, mvY);
				if (j == 1)
					fprintf(fpMvLogfile, "MbAddr[%4d:Back]: Avail[0] Mv[%5d:%5d]\n", i, mvX, mvY);

			} else {

				if(mvX & 0x2000) {/* Signed extension */
					mvX = mvX | 0xC000;
				} else {
					mvX = mvX & 0x1FFF;
				}

				if(mvY & 0x2000) {
					mvY = mvY | 0xC000;
				} else {
					mvY = mvY & 0x1FFF;
				}

				if (j == 0 )
					fprintf(fpMvLogfile, "MbAddr[%4d:For ]: Avail[1] Mv[%5d:%5d]\n", i, mvX, mvY);
				if (j == 1)
					fprintf(fpMvLogfile, "MbAddr[%4d:Back]: Avail[1] Mv[%5d:%5d]\n", i, mvX, mvY);
			}
		}
	}
	fflush(fpMvLogfile);
}

void SaveUserData(u8 *userDataBuf) {
	int i, UserDataType, UserDataSize, userDataNum, TotalSize;
	u8 *tmpBuf;

	if(fpUserDataLogfile == 0) {
		fpUserDataLogfile = fopen(FN_USER_DATA, "w+");
	}

	tmpBuf = userDataBuf;
	userDataNum = (short)((tmpBuf[0]<<8) | (tmpBuf[1]<<0));
	TotalSize = (short)((tmpBuf[2]<<8) | (tmpBuf[3]<<0));

	tmpBuf = userDataBuf + 8;

	for(i=0; i<userDataNum; i++) {
		UserDataType = (short)((tmpBuf[0]<<8) | (tmpBuf[1]<<0));
		UserDataSize = (short)((tmpBuf[2]<<8) | (tmpBuf[3]<<0));
		fprintf(fpUserDataLogfile, "\n[Idx Type Size] : [%4d %4d %4d]",i, UserDataType, UserDataSize);

		tmpBuf += 8;
	}
	fprintf(fpUserDataLogfile, "\n");

	tmpBuf = userDataBuf + USER_DATA_INFO_OFFSET;
	for(i=0; i<TotalSize; i++) {
		fprintf(fpUserDataLogfile, "%02x", tmpBuf[i]);
		if ((i&7) == 7)
			fprintf(fpUserDataLogfile, "\n");
	}
	fprintf(fpUserDataLogfile, "\n");
	fflush(fpUserDataLogfile);

}

/*
 * Fill the bitstream ring buffer
 */
int dec_fill_bsbuffer(DecHandle handle, struct cmd_line *cmd,
		u32 bs_va_startaddr, u32 bs_va_endaddr,
		u32 bs_pa_startaddr, int defaultsize,
		int *eos, int *fill_end_bs)
{
	RetCode ret;
	PhysicalAddress pa_read_ptr, pa_write_ptr;
	u32 target_addr, space;
	int size;
	int nread, room;
	*eos = 0;



	ret = vpu_DecGetBitstreamBuffer(handle, &pa_read_ptr, &pa_write_ptr,
					&space);


	if (ret != RETCODE_SUCCESS) {
		err_msg("vpu_DecGetBitstreamBuffer failed\n");
		printf("vpu_DecGetBitstreamBuffer failed\n");
		return -1;
	}


	/* Decoder bitstream buffer is full */
	if (space <= 0) {
		warn_msg("space %lu <= 0\n", space);
		return 0;
	}

	if (defaultsize > 0) {
		if (space < defaultsize)
			return 0;

		size = defaultsize;
	} else {
		size = ((space >> 9) << 9);
	}

	if (size == 0) {
		warn_msg("size == 0, space %lu\n", space);
		return 0;
	}



	/* Fill the bitstream buffer */
	target_addr = bs_va_startaddr + (pa_write_ptr - bs_pa_startaddr);
	if ( (target_addr + size) > bs_va_endaddr) {

		room = bs_va_endaddr - target_addr;
		nread = vpu_read(cmd, (void *)target_addr, room);

		if (nread <= 0) {
			/* EOF or error */
			if (nread < 0) {
				if (nread == -EAGAIN)
					return 0;

				err_msg("nread %d < 0\n", nread);
				printf("\n10\n");
				return -1;
			}

			*eos = 1;
		} else {

			/* unable to fill the requested size, so back off! */
			if (nread != room)
				goto update;

			/* read the remaining */
			space = nread;
			nread = vpu_read(cmd, (void *)bs_va_startaddr,
					(size - room));
			if (nread <= 0) {
				/* EOF or error */
				if (nread < 0) {
					if (nread == -EAGAIN)
						return 0;

					err_msg("nread %d < 0\n", nread);
					printf("\n11\n");
					return -1;
				}

				*eos = 1;
			}

			nread += space;
		}
	} else {
	printf("\n26\n");
		nread = vpu_read(cmd, (void *)target_addr, size);
		printf("\n27\n");
		if (nread <= 0) {
			/* EOF or error */
			if (nread < 0) {
				if (nread == -EAGAIN)
					return 0;

				err_msg("nread %d < 0\n", nread);
				printf("\n12\n");
				return -1;
			}

			*eos = 1;
		}
	}

update:
	if (*eos == 0) {
		ret = vpu_DecUpdateBitstreamBuffer(handle, nread);
		if (ret != RETCODE_SUCCESS) {
			err_msg("vpu_DecUpdateBitstreamBuffer failed\n");
			printf("\n12.5\n");
			return -1;
		}
		*fill_end_bs = 0;
	} else {
		if (!*fill_end_bs) {
			ret = vpu_DecUpdateBitstreamBuffer(handle,
					STREAM_END_SIZE);
			if (ret != RETCODE_SUCCESS) {
				err_msg("vpu_DecUpdateBitstreamBuffer failed"
								"\n");
				printf("\n13\n");
				return -1;
			}
			*fill_end_bs = 1;
		}

	}

	printf("\n14\n");
	return nread;
}

/*
 * This function is to convert framebuffer from interleaved Cb/Cr mode
 * to non-interleaved Cb/Cr mode.
 *
 * Note: This function does _NOT_ really store this framebuffer into file.
 */
static void
saveNV12ImageHelper(u8 *pYuv, struct decode *dec, u8 *buf)
{
	int Y, Cb;
	u8 *Y1, *Cb1, *Cr1;
	int img_size;
	int y, x;
	u8 *tmp;
	int height = dec->picheight;
	int stride = dec->stride;

	if (!pYuv || !buf) {
		err_msg("pYuv or buf should not be NULL.\n");
		return;
	}

	img_size = stride * height;

	Y = (int)buf;
	Cb = Y + img_size;

	Y1 = pYuv;
	Cb1 = Y1 + img_size;
	Cr1 = Cb1 + (img_size >> 2);

	memcpy(Y1, (u8 *)Y, img_size);

	for (y = 0; y < (dec->picheight / 2); y++) {
		tmp = (u8*)(Cb + dec->picwidth * y);
		for (x = 0; x < dec->picwidth; x += 2) {
			*Cb1++ = tmp[x];
			*Cr1++ = tmp[x + 1];
		}
	}
}

/*
 * This function is to store the framebuffer with Cropping size.
 *
 * Note: The output of picture width or height from VPU is always
 * 4-bit aligned. For example, the Cropping information in one
 * bit stream is crop.left/crop.top/crop.right/crop.bottom = 0/0/320/136
 * VPU output is picWidth = 320, picHeight = (136 + 15) & ~15 = 144;
 * whereas, this function will use crop information to save the framebuffer
 * with picWidth = 320, picHeight = 136. Kindly remind that all this calculation
 * is under the case without Rotation or Mirror.
 */
static void
saveCropYuvImageHelper(struct decode *dec, u8 *buf, Rect cropRect)
{
	u8 *pCropYuv;
	int cropWidth, cropHeight;
	int i;

	int width = dec->picwidth;
	int height = dec->picheight;
	int rot_en = dec->cmdl->rot_en;
	int rot_angle = dec->cmdl->rot_angle;


	if (!buf) {
		err_msg("buffer point should not be NULL.\n");
		return;
	}

	if (rot_en && (rot_angle == 90 || rot_angle == 270)) {
		i = width;
		width = height;
		height = i;
	}

	dprintf(3, "left/top/right/bottom = %lu/%lu/%lu/%lu\n", cropRect.left,
			cropRect.top, cropRect.right, cropRect.bottom);

	pCropYuv = buf;
	cropWidth = cropRect.right - cropRect.left;
	cropHeight = cropRect.bottom - cropRect.top;

	pCropYuv += width * cropRect.top;
	pCropYuv += cropRect.left;


	for (i = 0; i < cropHeight; i++) {
		fwriten(dec->cmdl->dst_fd, pCropYuv, cropWidth);
		pCropYuv += width;
	}

	if (dec->cmdl->format == STD_MJPG && dec->mjpg_fmt == MODE400)
		return;

	cropWidth /= 2;
	cropHeight /= 2;
	pCropYuv = buf + (width * height);
	pCropYuv += (width / 2) * (cropRect.top / 2);
	pCropYuv += cropRect.left / 2;


	for (i = 0; i < cropHeight; i++) {
		fwriten(dec->cmdl->dst_fd, pCropYuv, cropWidth);
		pCropYuv += width / 2;
	}


	pCropYuv = buf + (width * height) * 5 / 4;
	pCropYuv += (width / 2) * (cropRect.top / 2);
	pCropYuv += cropRect.left / 2;


	for (i = 0; i < cropHeight; i++) {
		fwriten(dec->cmdl->dst_fd, pCropYuv, cropWidth);
		pCropYuv += width / 2;
	}

}

/*
 *YUV image copy from on-board memory of tiled YUV to host buffer
 */
int SaveTiledYuvImageHelper(struct decode *dec, int yuvFp,
                              int picWidth, int picHeight, int index)
{
	int frameSize, pix_addr, offset;
	int y, x, nY, nCb, j;
	Uint8 *puc, *pYuv = 0, *dstAddrCb, *dstAddrCr;
	Uint32 addrY, addrCb, addrCr;
	Uint8 temp_buf[8];
	struct frame_buf *pfb = NULL;

	frameSize = picWidth * picHeight * 3 / 2;
	pfb = dec->pfbpool[index];

	pYuv = malloc(frameSize);
	if (!pYuv) {
		err_msg("Fail to allocate memory\n");
		return -1;
	}

	if (dec->cmdl->rot_en && (dec->cmdl->rot_angle == 90 || dec->cmdl->rot_angle == 270)) {
                j = picWidth;
                picWidth = picHeight;
                picHeight = j;
        }

	puc = pYuv;
	nY = picHeight;
	nCb = picHeight / 2;
	addrY = pfb->addrY;
	addrCb = pfb->addrCb;
	addrCr = pfb->addrCr;
	offset = pfb->desc.virt_uaddr - pfb->desc.phy_addr;

	for (y = 0; y < nY; y++) {
		for (x = 0; x < picWidth; x += 8) {
			pix_addr = vpu_GetXY2AXIAddr(dec->handle, 0, y, x, picWidth,
                                        addrY, addrCb, addrCr);
			pix_addr += offset;
			memcpy(puc + y * picWidth + x, (Uint8 *)pix_addr, 8);
		}
	}

	dstAddrCb = (Uint8 *)(puc + picWidth * nY);
	dstAddrCr = (Uint8 *)(puc + picWidth * nY * 5 / 4);

	for (y = 0; y < nCb ; y++) {
		for (x = 0; x < picWidth; x += 8) {
			pix_addr = vpu_GetXY2AXIAddr(dec->handle, 2, y, x, picWidth,
					addrY, addrCb, addrCr);
			pix_addr += offset;
			memcpy(temp_buf, (Uint8 *)pix_addr, 8);
			for (j = 0; j < 4; j++) {
				*dstAddrCb++ = *(temp_buf + j * 2);
				*dstAddrCr++ = *(temp_buf + j * 2 + 1);
			}
		}
	}

	fwriten(yuvFp, (u8 *)pYuv, frameSize);
	free(pYuv);

	return 0;
}
/*
 * This function is to swap the cropping left/top/right/bottom
 * when there's cropping information, under rotation case.
 *
 * Note: If there's no cropping information in bit stream, just
 *	set rotCrop as no cropping info. And hence, the calling
 *	function after this will handle this case.
 */
static void
swapCropRect(struct decode *dec, Rect *rotCrop)
{
	int mode = 0;
	int rot_en = dec->cmdl->rot_en;
	int rotAngle = dec->cmdl->rot_angle;
	int framebufWidth = dec->picwidth;
	int framebufHeight = dec->picheight;
	int mirDir = dec->cmdl->mirror;
	int crop;

	if (!rot_en)
		err_msg("VPU Rotation disabled. No need to call this func.\n");

	Rect *norCrop = &(dec->picCropRect);
	dprintf(3, "left/top/right/bottom = %lu/%lu/%lu/%lu\n", norCrop->left,
			norCrop->top, norCrop->right, norCrop->bottom);

	/*
	 * If no cropping info in bitstream, we just set rotCrop as it is.
	 */
	crop = norCrop->left | norCrop->top | norCrop->right | norCrop->bottom;
	if (!crop) {
		memcpy(rotCrop, norCrop, sizeof(*norCrop));
		return;
	}

	switch (rotAngle) {
		case 0:
			switch (mirDir) {
				case MIRDIR_NONE:
					mode = 0;
					break;
				case MIRDIR_VER:
					mode = 5;
					break;
				case MIRDIR_HOR:
					mode = 4;
					break;
				case MIRDIR_HOR_VER:
					mode = 2;
					break;
			}
			break;
		/*
		 * Remember? VPU sets the rotation angle by counter-clockwise.
		 * We convert it to clockwise, in order to make it consistent
		 * with V4L2 rotation angle strategy. (refer to decoder_start)
		 *
		 * Note: if you wanna leave VPU rotation angle as it originally
		 *	is, bear in mind you need exchange the codes for
		 *	"case 90" and "case 270".
		 */
		case 90:
			switch (mirDir) {
				case MIRDIR_NONE:
					mode = 3;
					break;
				case MIRDIR_VER:
					mode = 7;
					break;
				case MIRDIR_HOR:
					mode = 6;
					break;
				case MIRDIR_HOR_VER:
					mode = 1;
					break;
			}
			break;
		case 180:
			switch (mirDir) {
				case MIRDIR_NONE:
					mode = 2;
					break;
				case MIRDIR_VER:
					mode = 4;
					break;
				case MIRDIR_HOR:
					mode = 5;
					break;
				case MIRDIR_HOR_VER:
					mode = 0;
					break;
			}
			break;
		/*
		 * Ditto. As the rot angle 90.
		 */
		case 270:
			switch (mirDir) {
				case MIRDIR_NONE:
					mode = 1;
					break;
				case MIRDIR_VER:
					mode = 6;
					break;
				case MIRDIR_HOR:
					mode = 7;
					break;
				case MIRDIR_HOR_VER:
					mode = 3;
					break;
			}
			break;
	}

	switch (mode) {
		case 0:
			rotCrop->bottom = norCrop->bottom;
			rotCrop->left = norCrop->left;
			rotCrop->right = norCrop->right;
			rotCrop->top = norCrop->top;
			break;
		case 1:
			rotCrop->bottom = framebufWidth - norCrop->left;
			rotCrop->left = norCrop->top;
			rotCrop->right = norCrop->bottom;
			rotCrop->top = framebufWidth - norCrop->right;
			break;
		case 2:
			rotCrop->bottom = framebufHeight - norCrop->top;
			rotCrop->left = framebufWidth - norCrop->right;
			rotCrop->right = framebufWidth - norCrop->left;
			rotCrop->top = framebufHeight - norCrop->bottom;
			break;
		case 3:
			rotCrop->bottom = norCrop->right;
			rotCrop->left = framebufHeight - norCrop->bottom;
			rotCrop->right = framebufHeight - norCrop->top;
			rotCrop->top = norCrop->left;
			break;
		case 4:
			rotCrop->bottom = norCrop->bottom;
			rotCrop->left = framebufWidth - norCrop->right;
			rotCrop->right = framebufWidth - norCrop->left;
			rotCrop->top = norCrop->top;
			break;
		case 5:
			rotCrop->bottom = framebufHeight - norCrop->top;
			rotCrop->left = norCrop->left;
			rotCrop->right = norCrop->right;
			rotCrop->top = framebufHeight - norCrop->bottom;
			break;
		case 6:
			rotCrop->bottom = norCrop->right;
			rotCrop->left = norCrop->top;
			rotCrop->right = norCrop->bottom;
			rotCrop->top = norCrop->left;
			break;
		case 7:
			rotCrop->bottom = framebufWidth - norCrop->left;
			rotCrop->left = framebufHeight - norCrop->bottom;
			rotCrop->right = framebufHeight - norCrop->top;
			rotCrop->top = framebufWidth - norCrop->right;
			break;
	}

	return;
}

int decoder_start(struct decode *dec)
{
	DecHandle handle = dec->handle;
	DecOutputInfo outinfo = {0};
	DecParam decparam = {0};
	int rot_en = dec->cmdl->rot_en, rot_stride, fwidth, fheight;
	int rot_angle = dec->cmdl->rot_angle;
	int deblock_en = dec->cmdl->deblock_en;
	int dering_en = dec->cmdl->dering_en;
	FrameBuffer *deblock_fb = NULL;
	FrameBuffer *fb = dec->fb;
	struct frame_buf **pfbpool = dec->pfbpool;
	struct frame_buf *pfb = NULL;
	struct vpu_display *disp = dec->disp;
	struct v4l_specific_data *v4l_rsd;
	int err = 0, eos = 0, fill_end_bs = 0, decodefinish = 0;
	struct timeval tdec_begin,tdec_end, total_start, total_end;
	RetCode ret;
	int sec, usec, loop_id;
	u32 img_size;
	double tdec_time = 0, frame_id = 0, total_time=0;
	int decIndex = 0;
	int rotid = 0, dblkid = 0, mirror;
	int count = dec->cmdl->count;
	int totalNumofErrMbs = 0;
	int disp_clr_index = -1, actual_display_index = -1, field = V4L2_FIELD_NONE;
	int is_waited_int = 0;
	int tiled2LinearEnable = dec->tiled2LinearEnable;
	char *delay_ms, *endptr;
	int mjpgReadChunk = 0;
	int index = -1;

	disp = dec->disp;
//	v4l_rsd = (struct v4l_specific_data *)disp->render_specific_data;

	/* deblock_en is zero on none mx27 since it is cleared in decode_open() function. */
	if (rot_en || dering_en || tiled2LinearEnable) {
		rotid = dec->regfbcount;
		if (deblock_en) {
			dblkid = dec->regfbcount + dec->rot_buf_count;
		}
	} else if (deblock_en) {
		dblkid = dec->regfbcount;
	}
	printf("ponto 40\n");
	decparam.dispReorderBuf = 0;
	decparam.skipframeMode = 0;
	decparam.skipframeNum = 0;
	/*
	 * once iframeSearchEnable is enabled, prescanEnable, prescanMode
	 * and skipframeMode options are ignored.
	 */
	decparam.iframeSearchEnable = 0;

	fwidth = ((dec->picwidth + 15) & ~15);
	fheight = ((dec->picheight + 15) & ~15);

	if (rot_en || dering_en || tiled2LinearEnable || (dec->cmdl->format == STD_MJPG)) {
		/*
		 * VPU is setting the rotation angle by counter-clockwise.
		 * We convert it to clockwise, which is consistent with V4L2
		 * rotation angle strategy.
		 */
		if (rot_en) {
			if (rot_angle == 90 || rot_angle == 270)
				rot_angle = 360 - rot_angle;
		} else
			rot_angle = 0;

		vpu_DecGiveCommand(handle, SET_ROTATION_ANGLE,
					&rot_angle);

		mirror = dec->cmdl->mirror;
		vpu_DecGiveCommand(handle, SET_MIRROR_DIRECTION,
					&mirror);

		if (rot_en)
			rot_stride = (rot_angle == 90 || rot_angle == 270) ?
					fheight : fwidth;
		else
			rot_stride = fwidth;
		vpu_DecGiveCommand(handle, SET_ROTATOR_STRIDE, &rot_stride);
	}
	printf("ponto 41\n");
	if (deblock_en) {
		deblock_fb = &fb[dblkid];
	}

	if ((dec->cmdl->dst_scheme == PATH_V4L2) || (dec->cmdl->dst_scheme == PATH_IPU)) {
		img_size = dec->stride * dec->picheight;
	} else {
		img_size = dec->picwidth * dec->picheight * 3 / 2;
		if (deblock_en) {
			pfb = pfbpool[dblkid];
			deblock_fb->bufY = pfb->addrY;
			deblock_fb->bufCb = pfb->addrCb;
			deblock_fb->bufCr = pfb->addrCr;
		}
	}
	printf("ponto 42\n");
	gettimeofday(&total_start, NULL);

	while (1) {

		if (rot_en || dering_en || tiled2LinearEnable || (dec->cmdl->format == STD_MJPG)) {
			vpu_DecGiveCommand(handle, SET_ROTATOR_OUTPUT,
						(void *)&fb[rotid]);
			if (frame_id == 0) {
				if (rot_en) {
					vpu_DecGiveCommand(handle,
							ENABLE_ROTATION, 0);
					vpu_DecGiveCommand(handle,
							ENABLE_MIRRORING,0);
				}
				if (dering_en) {
					vpu_DecGiveCommand(handle,
							ENABLE_DERING, 0);
				}
			}
		}
		printf("ponto 43\n");
		if (deblock_en) {
			ret = vpu_DecGiveCommand(handle, DEC_SET_DEBLOCK_OUTPUT,
						(void *)deblock_fb);
			if (ret != RETCODE_SUCCESS) {
				err_msg("Failed to set deblocking output\n");
				return -1;
			}
		}

		if (dec->mbInfo.enable == 1) {
			ret = vpu_DecGiveCommand(handle, DEC_SET_REPORT_MBINFO, &dec->mbInfo);
			if (ret != RETCODE_SUCCESS) {
				err_msg("Failed to set MbInfo report, ret %d\n", ret);
				return -1;
			}
		}
		if (dec->mvInfo.enable == 1) {
			ret = vpu_DecGiveCommand(handle,DEC_SET_REPORT_MVINFO, &dec->mvInfo);
			if (ret != RETCODE_SUCCESS) {
				err_msg("Failed to set MvInfo report, ret %d\n", ret);
				return -1;
			}
		}
		if (dec->frameBufStat.enable == 1) {
			ret = vpu_DecGiveCommand(handle,DEC_SET_REPORT_BUFSTAT, &dec->frameBufStat);
			if (ret != RETCODE_SUCCESS) {
				err_msg("Failed to set frame status report, ret %d\n", ret);
				return -1;
			}
		}
		if (dec->userData.enable == 1) {
			ret = vpu_DecGiveCommand(handle,DEC_SET_REPORT_USERDATA, &dec->userData);
			if (ret != RETCODE_SUCCESS) {
				err_msg("Failed to set user data report, ret %d\n", ret);
				return -1;
			}
		}
		printf("ponto 44\n");
		gettimeofday(&tdec_begin, NULL);
		ret = vpu_DecStartOneFrame(handle, &decparam);
		printf("ponto 45\n");

		if (ret != RETCODE_SUCCESS) {
			err_msg("DecStartOneFrame failed, ret=%d\n", ret);
			return -1;
		}

		is_waited_int = 0;
		loop_id = 0;
		while (vpu_IsBusy()) {

			/*
			 * Suppose vpu is hang if one frame cannot be decoded in 5s,
			 * then do vpu software reset.
			 * Please take care of this for network case since vpu
			 * interrupt also cannot be received if no enough data.
			 */
			if (loop_id == 50) {
				err = vpu_SWReset(handle, 0);
				return -1;
			}

			if (vpu_WaitForInt(100) == 0)
				is_waited_int = 1;
			loop_id ++;
		}
		printf("ponto 46\n");
		if (!is_waited_int)
			vpu_WaitForInt(100);

		gettimeofday(&tdec_end, NULL);
		sec = tdec_end.tv_sec - tdec_begin.tv_sec;
		usec = tdec_end.tv_usec - tdec_begin.tv_usec;

		if (usec < 0) {
			sec--;
			usec = usec + 1000000;
		}
		printf("ponto 47\n");
		tdec_time += (sec * 1000000) + usec;

		ret = vpu_DecGetOutputInfo(handle, &outinfo);
		printf("ponto 48\n");
		dprintf(3, "frame_id %d, decidx %d, disidx %d, rotid %d, decodingSuccess 0x%x\n",
				(int)frame_id, outinfo.indexFrameDecoded, outinfo.indexFrameDisplay,
				rotid, outinfo.decodingSuccess);
		if (ret != RETCODE_SUCCESS) {
			err_msg("vpu_DecGetOutputInfo failed Err code is %d\n"
				"\tframe_id = %d\n", ret, (int)frame_id);
			return -1;
		}

		if (outinfo.decodingSuccess == 0) {
			warn_msg("Incomplete finish of decoding process.\n"
				"\tframe_id = %d\n", (int)frame_id);
			if ((outinfo.indexFrameDecoded >= 0) && (outinfo.numOfErrMBs)) {
				if (cpu_is_mx6x() && dec->cmdl->format == STD_MJPG)
					info_msg("Error restart idx:%d, MCU x:%d, y:%d, in Frame : %d\n",
							(outinfo.numOfErrMBs & 0x0F000000) >> 24,
							(outinfo.numOfErrMBs & 0x00FFF000) >> 12,
							(outinfo.numOfErrMBs & 0x00000FFF),
							decIndex);
			}
			if (quitflag)
				break;
			else
				continue;
		}

		if (cpu_is_mx6x() && (outinfo.decodingSuccess & 0x10)) {
			warn_msg("vpu needs more bitstream in rollback mode\n"
				"\tframe_id = %d\n", (int)frame_id);

			err = dec_fill_bsbuffer(handle,  dec->cmdl, dec->virt_bsbuf_addr,
					(dec->virt_bsbuf_addr + STREAM_BUF_SIZE),
					dec->phy_bsbuf_addr, 0, &eos, &fill_end_bs);
			if (err < 0) {
				err_msg("dec_fill_bsbuffer failed 4\n");
				return -1;
			}

			if (quitflag)
				break;
			else
				continue;
		}
		printf("ponto 49\n");
		if (cpu_is_mx6x() && (outinfo.decodingSuccess & 0x100000))
			warn_msg("sequence parameters have been changed\n");

		if (outinfo.notSufficientPsBuffer) {
			err_msg("PS Buffer overflow\n");
			return -1;
		}

		if (outinfo.notSufficientSliceBuffer) {
			err_msg("Slice Buffer overflow\n");
			return -1;
		}
		printf("ponto 50\n");
		/* Frame Buffer Status */
		if (outinfo.indexFrameDecoded >= 0 &&
			outinfo.frameBufStat.enable &&
			outinfo.frameBufStat.size) {
			SaveFrameBufStat(outinfo.frameBufStat.addr,
				   outinfo.frameBufStat.size, decIndex);
		}

		/* Mb Param */
		if (outinfo.indexFrameDecoded >= 0 && outinfo.mbInfo.enable &&
			outinfo.mbInfo.size) {
			/* skip picture */
			if (!(dec->cmdl->format == STD_VC1 &&
				(outinfo.picType >>3 == 4)))
				SaveMB_Para(outinfo.mbInfo.addr,
				        outinfo.mbInfo.size, decIndex);
		}

		/* Motion Vector */
		if (outinfo.indexFrameDecoded >= 0 && outinfo.mvInfo.enable &&
			outinfo.mvInfo.size) {
			int mbNumX = dec->picwidth / 16;
			SaveMvPara(outinfo.mvInfo.addr, outinfo.mvInfo.size,
			   outinfo.mvInfo.mvNumPerMb, mbNumX, decIndex);
		}

		/* User Data */
		if (outinfo.indexFrameDecoded >= 0 &&
			outinfo.userData.enable && outinfo.userData.size) {
			if (outinfo.userData.userDataBufFull)
				warn_msg("User Data Buffer is Full\n");
			SaveUserData(dec->userData.addr);
		}
		printf("ponto 51\n");
		if (outinfo.indexFrameDecoded >= 0) {
			printf("ponto 52\n");
			field = V4L2_FIELD_NONE;

			if (dec->cmdl->format == STD_VC1) {
				if (outinfo.pictureStructure == 2)
					info_msg("dec_idx %d : FRAME_INTERLACE\n", decIndex);
				else if (outinfo.pictureStructure == 3) {
					if (outinfo.topFieldFirst)
						field = V4L2_FIELD_INTERLACED_TB;
					else
						field = V4L2_FIELD_INTERLACED_BT;
				}
				if (outinfo.vc1_repeatFrame)
					info_msg("dec_idx %d : VC1 RPTFRM [%1d]\n", decIndex, outinfo.vc1_repeatFrame);
			} else if ((dec->cmdl->format == STD_AVC) ||
				   (dec->cmdl->format == STD_MPEG4)) {
				if ((outinfo.interlacedFrame) ||
				    ((dec->cmdl->format == STD_MPEG4) && isInterlacedMPEG4)) {
					if (outinfo.topFieldFirst)
						field = V4L2_FIELD_INTERLACED_TB;
					else
						field = V4L2_FIELD_INTERLACED_BT;
					dprintf(3, "Top Field First flag: %d, dec_idx %d\n",
						  outinfo.topFieldFirst, decIndex);
				}
			} else if (dec->cmdl->format == STD_MPEG2) {
				if ((outinfo.interlacedFrame)||(!outinfo.progressiveFrame)) {
					if (outinfo.pictureStructure == 1)
						field = V4L2_FIELD_INTERLACED_BT;
					else if (outinfo.pictureStructure == 2)
						field = V4L2_FIELD_INTERLACED_TB;
					else if (outinfo.pictureStructure == 3) {
						if (outinfo.topFieldFirst)
							field = V4L2_FIELD_INTERLACED_TB;
						else
							field = V4L2_FIELD_INTERLACED_BT;
					}
				}
				if (outinfo.repeatFirstField)
					info_msg("frame_idx %d : Repeat First Field\n", decIndex);
			} else if ((dec->cmdl->format != STD_MPEG4) &&
				   (dec->cmdl->format != STD_H263) &&
				   (dec->cmdl->format != STD_RV)){
				if (outinfo.interlacedFrame) {
					if (outinfo.pictureStructure == 1)
						field = V4L2_FIELD_TOP;
					else if (outinfo.pictureStructure == 2)
						field = V4L2_FIELD_BOTTOM;
					else if (outinfo.pictureStructure == 3) {
						if (outinfo.topFieldFirst)
							field = V4L2_FIELD_INTERLACED_TB;
						else
							field = V4L2_FIELD_INTERLACED_BT;
					}
				}
				if (outinfo.repeatFirstField)
					info_msg("frame_idx %d : Repeat First Field\n", decIndex);
			}
			dec->decoded_field[outinfo.indexFrameDecoded]= field;
			printf("ponto 53\n");
		}

		if (outinfo.indexFrameDisplay == -1)
			decodefinish = 1;
		else if ((outinfo.indexFrameDisplay > dec->regfbcount) &&
			 (outinfo.prescanresult != 0) && !cpu_is_mx6x())
			decodefinish = 1;
		printf("ponto 54\n");
		if (decodefinish && (!(rot_en || dering_en || tiled2LinearEnable)))
			break;


		if(outinfo.indexFrameDecoded >= 0) {
			/* We MUST be careful of sequence param change (resolution change, etc)
			 * Different frame buffer number or resolution may require vpu_DecClose
			 * and vpu_DecOpen again to reallocate sufficient resources.
			 * If you already allocate enough frame buffers of max resolution
			 * in the beginning, you may not need vpu_DecClose, etc. But sequence
			 * headers must be ahead of their pictures to signal param change.
			 */
			if ((outinfo.decPicWidth != dec->lastPicWidth)
					||(outinfo.decPicHeight != dec->lastPicHeight)) {
				warn_msg("resolution changed from %dx%d to %dx%d\n",
						dec->lastPicWidth, dec->lastPicHeight,
						outinfo.decPicWidth, outinfo.decPicHeight);
				dec->lastPicWidth = outinfo.decPicWidth;
				dec->lastPicHeight = outinfo.decPicHeight;
			}

			if (outinfo.numOfErrMBs) {
				totalNumofErrMbs += outinfo.numOfErrMBs;
				info_msg("Num of Error Mbs : %d, in Frame : %d \n",
						outinfo.numOfErrMBs, decIndex);
			}
		}
		printf("ponto 55\n");
		if(outinfo.indexFrameDecoded >= 0)
			decIndex++;

		/* BIT don't have picture to be displayed */
		if ((outinfo.indexFrameDisplay == -3) ||
				(outinfo.indexFrameDisplay == -2)) {
			dprintf(3, "VPU doesn't have picture to be displayed.\n"
				"\toutinfo.indexFrameDisplay = %d\n",
						outinfo.indexFrameDisplay);

			if (!vpu_v4l_performance_test && (dec->cmdl->dst_scheme != PATH_IPU)) {
				if (dec->cmdl->format != STD_MJPG && disp_clr_index >= 0) {
					err = vpu_DecClrDispFlag(handle, disp_clr_index);
					if (err)
						err_msg("vpu_DecClrDispFlag failed Error code"
								" %d\n", err);
				}
				disp_clr_index = outinfo.indexFrameDisplay;
			}
			continue;
		}
		printf("ponto 56\n");
		if (rot_en || dering_en || tiled2LinearEnable || (dec->cmdl->format == STD_MJPG)) {
			/* delay one more frame for PP */
			if ((dec->cmdl->format != STD_MJPG) && (disp_clr_index < 0)) {
				disp_clr_index = outinfo.indexFrameDisplay;
				continue;
			}
			actual_display_index = rotid;
		}
		else
			actual_display_index = outinfo.indexFrameDisplay;

		if ((dec->cmdl->dst_scheme == PATH_V4L2) || (dec->cmdl->dst_scheme == PATH_IPU)

		   ) {
			 v4l_rsd = (struct v4l_specific_data *)disp->render_specific_data;
			if (deblock_en) {
				deblock_fb->bufY =
					v4l_rsd->buffers[v4l_rsd->buf.index]->offset;
				deblock_fb->bufCb = deblock_fb->bufY + img_size;
				deblock_fb->bufCr = deblock_fb->bufCb +
							(img_size >> 2);
			}

				if (dec->cmdl->dst_scheme == PATH_V4L2)
					err = v4l_put_data(dec, actual_display_index, dec->decoded_field[actual_display_index], dec->cmdl->fps);
				else if (dec->cmdl->dst_scheme == PATH_IPU)
					err = ipu_put_data(disp, actual_display_index, dec->decoded_field[actual_display_index], dec->cmdl->fps);

			if (err)
				return -1;

			if (dec->cmdl->dst_scheme == PATH_V4L2

			   ) {
				if (dec->cmdl->dst_scheme == PATH_V4L2) {
					if (!vpu_v4l_performance_test)
						index = v4l_rsd->buf.index;
					else
						index = v4l_get_buf(dec);
				}


				if (dec->cmdl->format != STD_MJPG && disp_clr_index >= 0) {
					err = vpu_DecClrDispFlag(handle, disp_clr_index);
					if (err)
						err_msg("vpu_DecClrDispFlag failed Error code"
								" %d\n", err);
				}

				if (dec->cmdl->format == STD_MJPG) {
					rotid++;
					rotid %= dec->regfbcount;
				} else if (rot_en || dering_en || tiled2LinearEnable) {
					disp_clr_index = outinfo.indexFrameDisplay;
					if (index != -1)
						rotid = index; /* de-queued buffer as next rotation buffer */
					else {
						rotid++;
						rotid = (rotid - dec->regfbcount) % dec->extrafb;
						rotid += dec->regfbcount;
					}
				} else
					disp_clr_index = index;
			}
		} else {
			if (rot_en) {
				Rect rotCrop;
				swapCropRect(dec, &rotCrop);
			} 

			if (dec->cmdl->format != STD_MJPG && disp_clr_index >= 0) {
				err = vpu_DecClrDispFlag(handle,disp_clr_index);
				if (err)
					err_msg("vpu_DecClrDispFlag failed Error code"
						" %d\n", err);
			}
			disp_clr_index = outinfo.indexFrameDisplay;
		}
		printf("ponto 57\n");

		frame_id++;
		if ((count != 0) && (frame_id >= count))
			break;

		if (dec->cmdl->src_scheme == PATH_NET) {
			err = dec_fill_bsbuffer(handle,	dec->cmdl,
				      dec->virt_bsbuf_addr,
				      (dec->virt_bsbuf_addr + STREAM_BUF_SIZE),
				      dec->phy_bsbuf_addr, STREAM_FILL_SIZE,
				      &eos, &fill_end_bs);
			if (err < 0) {
				err_msg("dec_fill_bsbuffer failed 6\n");
				return -1;
			}
		}

		delay_ms = getenv("VPU_DECODER_DELAY_MS");
		if (delay_ms && strtol(delay_ms, &endptr, 10))
			usleep(strtol(delay_ms,&endptr, 10) * 1000);

		if (decodefinish)
			break;
	}

	if (totalNumofErrMbs) {
		info_msg("Total Num of Error MBs : %d\n", totalNumofErrMbs);
	}

	gettimeofday(&total_end, NULL);
	sec = total_end.tv_sec - total_start.tv_sec;
	usec = total_end.tv_usec - total_start.tv_usec;
	if (usec < 0) {
		sec--;
		usec = usec + 1000000;
	}
	total_time = (sec * 1000000) + usec;

	info_msg("%d frames took %d microseconds\n", (int)frame_id,
						(int)total_time);
	info_msg("dec fps = %.2f\n", (frame_id / (tdec_time / 1000000)));
	info_msg("total fps= %.2f \n",(frame_id / (total_time / 1000000)));
	return 0;
}

void
decoder_free_framebuffer(struct decode *dec)
{
	int i, totalfb;
	vpu_mem_desc *mvcol_md = dec->mvcol_memdesc;
	int deblock_en = dec->cmdl->deblock_en;

	totalfb = dec->regfbcount + dec->extrafb;

	if ((dec->cmdl->dst_scheme == PATH_V4L2) || (dec->cmdl->dst_scheme == PATH_IPU)

	   ) {
		if (dec->disp) {
			if (dec->cmdl->dst_scheme == PATH_V4L2)
				v4l_display_close(dec->disp);
			else if (dec->cmdl->dst_scheme == PATH_IPU)
				ipu_display_close(dec->disp);

			dec->disp = NULL;
		}

		if (mvcol_md) {
			for (i = 0; i < totalfb; i++) {
				if (mvcol_md[i].phy_addr)
					IOFreePhyMem(&mvcol_md[i]);
			}
			if (dec->mvcol_memdesc) {
				free(dec->mvcol_memdesc);
				dec->mvcol_memdesc = NULL;
			}
		}
	}

	// deblock_en is zero on none mx27 since it is cleared in decode_open() function. 
	if (((dec->cmdl->dst_scheme != PATH_V4L2) && (dec->cmdl->dst_scheme != PATH_IPU)) ||
			(((dec->cmdl->dst_scheme == PATH_V4L2) || (dec->cmdl->dst_scheme == PATH_IPU))
			 && deblock_en)) {
		for (i = 0; i < totalfb; i++) {
			if (dec->pfbpool)
				framebuf_free(dec->pfbpool[i]);
		}
	}

	if (fpFrmStatusLogfile) {
		fclose(fpFrmStatusLogfile);
		fpFrmStatusLogfile = NULL;
	}
	if (fpErrMapLogfile) {
		fclose(fpErrMapLogfile);
		fpErrMapLogfile = NULL;
	}
	if (fpQpLogfile) {
		fclose(fpQpLogfile);
		fpQpLogfile = NULL;
	}
	if (fpSliceBndLogfile) {
		fclose(fpSliceBndLogfile);
		fpSliceBndLogfile = NULL;
	}
	if (fpMvLogfile) {
		fclose(fpMvLogfile);
		fpMvLogfile = NULL;
	}
	if (fpUserDataLogfile) {
		fclose(fpUserDataLogfile);
		fpUserDataLogfile = NULL;
	}

	if (dec->fb) {
		free(dec->fb);
		dec->fb = NULL;
	}
	if (dec->pfbpool) {
		free(dec->pfbpool);
		dec->pfbpool = NULL;
	}

	if (dec->frameBufStat.addr) {
		free(dec->frameBufStat.addr);
	}

	if (dec->mbInfo.addr) {
		free(dec->mbInfo.addr);
	}

	if (dec->mvInfo.addr) {
		free(dec->mvInfo.addr);
	}

	if (dec->userData.addr) {
		free(dec->userData.addr);
	}

	return;
}

int
decoder_allocate_framebuffer(struct decode *dec)
{
	DecBufInfo bufinfo;
	int i, regfbcount = dec->regfbcount, totalfb, img_size, mvCol;
       	int dst_scheme = dec->cmdl->dst_scheme, rot_en = dec->cmdl->rot_en;
	int deblock_en = dec->cmdl->deblock_en;
	int dering_en = dec->cmdl->dering_en;
	int tiled2LinearEnable =  dec->tiled2LinearEnable;
	struct rot rotation = {0};
	RetCode ret;
	DecHandle handle = dec->handle;
	FrameBuffer *fb;
	struct frame_buf **pfbpool;
	struct vpu_display *disp = NULL;
	struct v4l_specific_data *v4l_rsd = NULL;

	int stride, divX, divY;
	vpu_mem_desc *mvcol_md = NULL;
	Rect rotCrop;
	int delay = -1;

	if (rot_en || dering_en || tiled2LinearEnable) {
		//
		// At least 1 extra fb for rotation(or dering) is needed, two extrafb
		// are allocated for rotation if path is V4L,then we can delay 1 frame
		// de-queue from v4l queue to improve performance.
		//
		dec->rot_buf_count = ((dec->cmdl->dst_scheme == PATH_V4L2) ||
				(dec->cmdl->dst_scheme == PATH_IPU)) ? 2 : 1;
		//
		// Need more buffers for V4L2 performance mode
		// otherwise, buffer will be queued twice
		//
		dec->rot_buf_count = dec->regfbcount - dec->minfbcount + 1;
		dec->extrafb += dec->rot_buf_count;
		dec->post_processing = 1;
	}

	//
	// 1 extra fb for deblocking on MX32, no need extrafb for blocking on MX37 and MX51
	// dec->cmdl->deblock_en has been cleared to zero after set it to oparam.mp4DeblkEnable
	// in decoder_open() function on MX37 and MX51.
	//
	if (deblock_en) {
		dec->extrafb++;
	}

	totalfb = regfbcount + dec->extrafb;
	dprintf(4, "regfb %d, extrafb %d\n", regfbcount, dec->extrafb);

	fb = dec->fb = calloc(totalfb, sizeof(FrameBuffer));
	if (fb == NULL) {
		err_msg("Failed to allocate fb\n");
		return -1;
	}

	pfbpool = dec->pfbpool = calloc(totalfb, sizeof(struct frame_buf *));
	if (pfbpool == NULL) {
		err_msg("Failed to allocate pfbpool\n");
		free(dec->fb);
		dec->fb = NULL;
		return -1;
	}

	if (dec->cmdl->format == STD_MJPG)
		mvCol = 0;
	else
		mvCol = 1;

	if (((dst_scheme != PATH_V4L2) && (dst_scheme != PATH_IPU)) ||
			(((dst_scheme == PATH_V4L2) || (dst_scheme == PATH_IPU)) && deblock_en)) {
		if (dec->cmdl->mapType == LINEAR_FRAME_MAP) {
			// All buffers are linear 
			for (i = 0; i < totalfb; i++) {
				pfbpool[i] = framebuf_alloc(&dec->fbpool[i], dec->cmdl->format, dec->mjpg_fmt,
						    dec->stride, dec->picheight, mvCol);
				if (pfbpool[i] == NULL)
					goto err;
			}
                } else {
			// decoded buffers are tiled 
			for (i = 0; i < regfbcount; i++) {
				pfbpool[i] = tiled_framebuf_alloc(&dec->fbpool[i], dec->cmdl->format, dec->mjpg_fmt,
						dec->stride, dec->picheight, mvCol, dec->cmdl->mapType);
				if (pfbpool[i] == NULL)
					goto err;
			}

			for (i = regfbcount; i < totalfb; i++) {
				// if tiled2LinearEnable == 1, post processing buffer is linear,
				// otherwise, the buffer is tiled
				if (dec->tiled2LinearEnable)
					pfbpool[i] = framebuf_alloc(&dec->fbpool[i], dec->cmdl->format, dec->mjpg_fmt,
							dec->stride, dec->picheight, mvCol);
				else
					pfbpool[i] = tiled_framebuf_alloc(&dec->fbpool[i], dec->cmdl->format, dec->mjpg_fmt,
						    dec->stride, dec->picheight, mvCol, dec->cmdl->mapType);
				if (pfbpool[i] == NULL)
					goto err1;
			}
		 }

		for (i = 0; i < totalfb; i++) {
			fb[i].myIndex = i;
			fb[i].bufY = pfbpool[i]->addrY;
			fb[i].bufCb = pfbpool[i]->addrCb;
			fb[i].bufCr = pfbpool[i]->addrCr;
			if (!cpu_is_mx27()) {
				fb[i].bufMvCol = pfbpool[i]->mvColBuf;
			}
		}
	}

	printf("dst_scheme: %x\n", dst_scheme);

	if ((dst_scheme == PATH_V4L2) || (dst_scheme == PATH_IPU)

	   ) {
		rotation.rot_en = dec->cmdl->rot_en;
		rotation.rot_angle = dec->cmdl->rot_angle;

		if (dec->cmdl->ext_rot_en) {
			rotation.rot_en = 0;
			rotation.ext_rot_en = 1;
		}
		if (rotation.rot_en) {
			swapCropRect(dec, &rotCrop);
			if (dst_scheme == PATH_V4L2)
				disp = v4l_display_open(dec, totalfb, rotation, rotCrop);
			else if (dst_scheme == PATH_IPU)
				disp = ipu_display_open(dec, totalfb, rotation, rotCrop);

		} else
			if (dst_scheme == PATH_V4L2)
				disp = v4l_display_open(dec, totalfb, rotation, dec->picCropRect);
			else if (dst_scheme == PATH_IPU)
				disp = ipu_display_open(dec, totalfb, rotation, dec->picCropRect);


		if (disp == NULL) {
			goto err;
		}


		//Not set fps when doing performance test default//
		if ((dec->cmdl->fps == 0) && !vpu_v4l_performance_test && (dst_scheme == PATH_V4L2))
			dec->cmdl->fps = 30;


		info_msg("Display fps will be %d\n", dec->cmdl->fps);

		if (dst_scheme == PATH_V4L2)
			v4l_rsd = (struct v4l_specific_data *)disp->render_specific_data;

		printf("ponto 30\n");
		divX = (dec->mjpg_fmt == MODE420 || dec->mjpg_fmt == MODE422) ? 2 : 1;
		divY = (dec->mjpg_fmt == MODE420 || dec->mjpg_fmt == MODE224) ? 2 : 1;
		printf("ponto 31\n");
		if (deblock_en == 0) {
			img_size = dec->stride * dec->picheight;

			if (!cpu_is_mx27()) {
				mvcol_md = dec->mvcol_memdesc =
					calloc(totalfb, sizeof(vpu_mem_desc));
			}
			printf("ponto 32\n");
			for (i = 0; i < totalfb; i++) {
				fb[i].myIndex = i;

				if (dec->cmdl->mapType == LINEAR_FRAME_MAP) {
					if (dst_scheme == PATH_V4L2)
						fb[i].bufY = v4l_rsd->buffers[i]->offset;
					else if (dst_scheme == PATH_IPU)
						fb[i].bufY = disp->ipu_bufs[i].ipu_paddr;

					fb[i].bufCb = fb[i].bufY + img_size;
					fb[i].bufCr = fb[i].bufCb + (img_size / divX / divY);
				}
				else if ((dec->cmdl->mapType == TILED_FRAME_MB_RASTER_MAP)
						||(dec->cmdl->mapType == TILED_FIELD_MB_RASTER_MAP)){
					if (dst_scheme == PATH_V4L2)
						tiled_framebuf_base(&fb[i], v4l_rsd->buffers[i]->offset,
							dec->stride, dec->picheight, dec->cmdl->mapType);
					else if (dst_scheme == PATH_IPU) {
						fb[i].bufY = disp->ipu_bufs[i].ipu_paddr;
						fb[i].bufCb = fb[i].bufY + img_size;
						fb[i].bufCr = fb[i].bufCb + (img_size / divX / divY);
					}

				} else {
					err_msg("undefined mapType = %d\n", dec->cmdl->mapType);
					goto err1;
				}
				printf("ponto 33\n");
				// allocate MvCol buffer here
				if (!cpu_is_mx27()) {
					memset(&mvcol_md[i], 0,
							sizeof(vpu_mem_desc));
					mvcol_md[i].size = img_size / divX / divY;
					ret = IOGetPhyMem(&mvcol_md[i]);
					if (ret) {
						err_msg("buf alloc failed\n");
						goto err1;
					}
					fb[i].bufMvCol = mvcol_md[i].phy_addr;
				}
			}
		}
	}
	printf("ponto 34\n");

	stride = ((dec->stride + 15) & ~15);

	if (dec->cmdl->format == STD_AVC) {
		bufinfo.avcSliceBufInfo.bufferBase = dec->phy_slice_buf;
		bufinfo.avcSliceBufInfo.bufferSize = dec->phy_slicebuf_size;
	} else if (dec->cmdl->format == STD_VP8) {
		bufinfo.vp8MbDataBufInfo.bufferBase = dec->phy_vp8_mbparam_buf;
		bufinfo.vp8MbDataBufInfo.bufferSize = dec->phy_vp8_mbparam_size;
	}

	// User needs to fill max suported macro block value of frame as following
	bufinfo.maxDecFrmInfo.maxMbX = dec->stride / 16;
	bufinfo.maxDecFrmInfo.maxMbY = dec->picheight / 16;
	bufinfo.maxDecFrmInfo.maxMbNum = dec->stride * dec->picheight / 256;
	printf("ponto 35\n");
	// For H.264, we can overwrite initial delay calculated from syntax.
	// * delay can be 0,1,... (in unit of frames)
	// * Set to -1 or do not call this command if you don't want to overwrite it.
	// * Take care not to set initial delay lower than reorder depth of the clip,
	// * otherwise, display will be out of order. 
	vpu_DecGiveCommand(handle, DEC_SET_FRAME_DELAY, &delay);
	printf("ponto 36\n");
	ret = vpu_DecRegisterFrameBuffer(handle, fb, dec->regfbcount, stride, &bufinfo);
	if (ret != RETCODE_SUCCESS) {
		err_msg("Register frame buffer failed, ret=%d\n", ret);
		goto err1;
	}
	printf("ponto 37\n");
	dec->disp = disp;
	return 0;

err1:
	if (dst_scheme == PATH_V4L2) {
		v4l_display_close(disp);
		dec->disp = NULL;
	} else if (dst_scheme == PATH_IPU) {
		ipu_display_close(disp);
		dec->disp = NULL;
	}

err:
	if (((dst_scheme != PATH_V4L2) && (dst_scheme != PATH_IPU))||
	   ((dst_scheme == PATH_V4L2) && deblock_en )) {
		for (i = 0; i < totalfb; i++) {
			framebuf_free(pfbpool[i]);
		}
	}

	if (fpFrmStatusLogfile) {
		fclose(fpFrmStatusLogfile);
		fpFrmStatusLogfile = NULL;
	}
	if (fpErrMapLogfile) {
		fclose(fpErrMapLogfile);
		fpErrMapLogfile = NULL;
	}
	if (fpQpLogfile) {
		fclose(fpQpLogfile);
		fpQpLogfile = NULL;
	}
	if (fpSliceBndLogfile) {
		fclose(fpSliceBndLogfile);
		fpSliceBndLogfile = NULL;
	}
	if (fpMvLogfile) {
		fclose(fpMvLogfile);
		fpMvLogfile = NULL;
	}
	if (fpUserDataLogfile) {
		fclose(fpUserDataLogfile);
		fpUserDataLogfile = NULL;
	}

	free(dec->fb);
	free(dec->pfbpool);
	dec->fb = NULL;
	dec->pfbpool = NULL;

	return -1;

}

int
decoder_parse(struct decode *dec)
{
	DecInitialInfo initinfo = {0};
	DecHandle handle = dec->handle;
	int align, profile, level, extended_fbcount;
	RetCode ret;
	char *count;
	int origPicWidth, origPicHeight;

	/*
	 * If userData report is enabled, buffer and comamnd need to be set
 	 * before DecGetInitialInfo for MJPG.
	 */
	if (dec->userData.enable) {
		dec->userData.size = SIZE_USER_BUF;
		dec->userData.addr = malloc(SIZE_USER_BUF);
		if (!dec->userData.addr)
			err_msg("malloc_error\n");
	}

	if(!cpu_is_mx6x() && dec->cmdl->format == STD_MJPG) {
		ret = vpu_DecGiveCommand(handle,DEC_SET_REPORT_USERDATA, &dec->userData);
		if (ret != RETCODE_SUCCESS) {
			err_msg("Failed to set user data report, ret %d\n", ret);
			return -1;
		}
	}

	/* Parse bitstream and get width/height/framerate etc */
	vpu_DecSetEscSeqInit(handle, 1);
	ret = vpu_DecGetInitialInfo(handle, &initinfo);
	vpu_DecSetEscSeqInit(handle, 0);
	if (ret != RETCODE_SUCCESS) {
		err_msg("vpu_DecGetInitialInfo failed, ret:%d, errorcode:%ld\n",
		         ret, initinfo.errorcode);
		return -1;
	}

	if (initinfo.streamInfoObtained) {
		switch (dec->cmdl->format) {
		case STD_AVC:
			info_msg("H.264 Profile: %d Level: %d Interlace: %d\n",
				initinfo.profile, initinfo.level, initinfo.interlace);

			if (initinfo.aspectRateInfo) {
				int aspect_ratio_idc;
				int sar_width, sar_height;

				if ((initinfo.aspectRateInfo >> 16) == 0) {
					aspect_ratio_idc = (initinfo.aspectRateInfo & 0xFF);
					info_msg("aspect_ratio_idc: %d\n", aspect_ratio_idc);
				} else {
					sar_width = (initinfo.aspectRateInfo >> 16) & 0xFFFF;
					sar_height = (initinfo.aspectRateInfo & 0xFFFF);
					info_msg("sar_width: %d, sar_height: %d\n",
						sar_width, sar_height);
				}
			} else {
				info_msg("Aspect Ratio is not present.\n");
			}

			break;
		case STD_VC1:
			if (initinfo.profile == 0)
				info_msg("VC1 Profile: Simple\n");
			else if (initinfo.profile == 1)
				info_msg("VC1 Profile: Main\n");
			else if (initinfo.profile == 2)
				info_msg("VC1 Profile: Advanced\n");

			info_msg("Level: %d Interlace: %d Progressive Segmented Frame: %d\n",
				initinfo.level, initinfo.interlace, initinfo.vc1_psf);

			if (initinfo.aspectRateInfo)
				info_msg("Aspect Ratio [X, Y]:[%3d, %3d]\n",
						(initinfo.aspectRateInfo >> 8) & 0xff,
						(initinfo.aspectRateInfo) & 0xff);
			else
				info_msg("Aspect Ratio is not present.\n");

			break;
		case STD_MPEG2:
			info_msg("Mpeg2 Profile: %d Level: %d Progressive Sequence Flag: %d\n",
				initinfo.profile, initinfo.level, !initinfo.interlace);
			/*
			 * Profile: 3'b101: Simple, 3'b100: Main, 3'b011: SNR Scalable,
			 * 3'b10: Spatially Scalable, 3'b001: High
			 * Level: 4'b1010: Low, 4'b1000: Main, 4'b0110: High 1440, 4'b0100: High
			 */
			if (initinfo.aspectRateInfo)
				info_msg("Aspect Ratio Table index: %d\n", initinfo.aspectRateInfo);
			else
				info_msg("Aspect Ratio is not present.\n");
			break;

		case STD_MPEG4:
			if (initinfo.level & 0x80) { /* VOS Header */
				initinfo.level &= 0x7F;
				if (initinfo.level == 8 && initinfo.profile == 0) {
					level = 0; profile = 0;	 /* Simple, Level_L0 */
				} else {
					switch (initinfo.profile) {
					case 0xB:
						profile = 1; /* advanced coding efficiency object */
						break;
					case 0xF:
						if ((initinfo.level & 8) == 0)
							profile = 2; /* advanced simple object */
						else
							profile = 5; /* reserved */
						break;
					case 0x0:
						profile = 0;
						break;    /* Simple Profile */
					default:
						profile = 5;
						break;
					}
					level = initinfo.level;
				}
			} else { /* VOL Header only */
				level = 7;  /* reserved */
				switch (initinfo.profile) {
				case 0x1:
					profile = 0;  /* simple object */
					break;
				case 0xC:
					profile = 1;  /* advanced coding efficiency object */
					break;
				case 0x11:
					profile = 2;  /* advanced simple object */
					break;
				default:
					profile = 5;  /* reserved */
					break;
				}
			}
			isInterlacedMPEG4 = initinfo.interlace;

			info_msg("Mpeg4 Profile: %d Level: %d Interlaced: %d\n",
				profile, level, initinfo.interlace);
			/*
			 * Profile: 8'b00000000: SP, 8'b00010001: ASP
			 * Level: 4'b0000: L0, 4'b0001: L1, 4'b0010: L2, 4'b0011: L3, ...
			 * SP: 1/2/3/4a/5/6, ASP: 0/1/2/3/4/5
			 */
			if (initinfo.aspectRateInfo)
				info_msg("Aspect Ratio Table index: %d\n", initinfo.aspectRateInfo);
			else
				info_msg("Aspect Ratio is not present.\n");

			break;

		case STD_RV:
			info_msg("RV Profile: %d Level: %d\n", initinfo.profile, initinfo.level);
			break;

		case STD_H263:
			info_msg("H263 Profile: %d Level: %d\n", initinfo.profile, initinfo.level);
			break;

		case STD_DIV3:
			info_msg("DIV3 Profile: %d Level: %d\n", initinfo.profile, initinfo.level);
			break;

		case STD_MJPG:
			dec->mjpg_fmt = initinfo.mjpg_sourceFormat;
			info_msg("MJPG SourceFormat: %d\n", initinfo.mjpg_sourceFormat);
			break;

		case STD_AVS:
			info_msg("AVS Profile: %d Level: %d\n", initinfo.profile, initinfo.level);
			break;

		case STD_VP8:
			info_msg("VP8 Profile: %d Level: %d\n", initinfo.profile, initinfo.level);
			info_msg("hScaleFactor: %d vScaleFactor:%d\n",
				    initinfo.vp8ScaleInfo.hScaleFactor,
				    initinfo.vp8ScaleInfo.vScaleFactor);
			break;

		default:
			break;
		}
	}

	dec->lastPicWidth = initinfo.picWidth;
	dec->lastPicHeight = initinfo.picHeight;

	if (cpu_is_mx6x())
		info_msg("Decoder: width = %d, height = %d, frameRateRes = %lu, frameRateDiv = %lu, count = %u\n",
			initinfo.picWidth, initinfo.picHeight,
			initinfo.frameRateRes, initinfo.frameRateDiv,
			initinfo.minFrameBufferCount);
	else
		info_msg("Decoder: width = %d, height = %d, fps = %lu, count = %u\n",
			initinfo.picWidth, initinfo.picHeight,
			initinfo.frameRateInfo,
			initinfo.minFrameBufferCount);

#ifdef COMBINED_VIDEO_SUPPORT
	/* Following lines are sample code to support minFrameBuffer counter
	   changed in combined video stream. */
	if (dec->cmdl->format == STD_AVC)
		initinfo.minFrameBufferCount = 19;
#endif
	/*
	 * We suggest to add two more buffers than minFrameBufferCount:
	 *
	 * vpu_DecClrDispFlag is used to control framebuffer whether can be
	 * used for decoder again. One framebuffer dequeue from IPU is delayed
	 * for performance improvement and one framebuffer is delayed for
	 * display flag clear.
	 *
	 * Performance is better when more buffers are used if IPU performance
	 * is bottleneck.
	 *
	 * Two more buffers may be needed for interlace stream from IPU DVI view
	 */
	dec->minfbcount = initinfo.minFrameBufferCount;
	count = getenv("VPU_EXTENDED_BUFFER_COUNT");
	if (count)
		extended_fbcount = atoi(count);
	else
		extended_fbcount = 2;

	if (initinfo.interlace)
		dec->regfbcount = dec->minfbcount + extended_fbcount + 2;
	else
		dec->regfbcount = dec->minfbcount + extended_fbcount;
	dprintf(4, "minfb %d, extfb %d\n", dec->minfbcount, extended_fbcount);

	if (cpu_is_mx6x() && (dec->cmdl->format == STD_MJPG)) {
		origPicWidth = initinfo.picWidth >> dec->mjpegScaleDownRatioWidth;
		origPicHeight = initinfo.picHeight >> dec->mjpegScaleDownRatioHeight;
	} else {
		origPicWidth = initinfo.picWidth;
		origPicHeight = initinfo.picHeight;
	}

	dec->picwidth = ((origPicWidth + 15) & ~15);

	align = 16;
	if ((dec->cmdl->format == STD_MPEG2 ||
	    dec->cmdl->format == STD_VC1 ||
	    dec->cmdl->format == STD_AVC ||
	    dec->cmdl->format == STD_VP8) && initinfo.interlace == 1)
		align = 32;

	dec->picheight = ((origPicHeight + align - 1) & ~(align - 1));

#ifdef COMBINED_VIDEO_SUPPORT
	/* Following lines are sample code to support resolution change
	   from small to large in combined video stream. MAX_FRAME_WIDTH
           and MAX_FRAME_HEIGHT must be defined per user requirement. */
	if (dec->picwidth < MAX_FRAME_WIDTH)
		dec->picwidth = MAX_FRAME_WIDTH;
	if (dec->picheight < MAX_FRAME_HEIGHT)
		dec->picheight =  MAX_FRAME_HEIGHT;
#endif

	if ((dec->picwidth == 0) || (dec->picheight == 0))
		return -1;

	/*
	 * Information about H.264 decoder picture cropping rectangle which
	 * presents the offset of top-left point and bottom-right point from
	 * the origin of frame buffer.
	 *
	 * By using these four offset values, host application can easily
	 * detect the position of target output window. When display cropping
	 * is off, the cropping window size will be 0.
	 *
	 * This structure for cropping rectangles is only valid for H.264
	 * decoder case.
	 */

	/* Add non-h264 crop support, assume left=top=0 */
	if ((dec->picwidth > origPicWidth ||
		dec->picheight > origPicHeight) &&
		(!initinfo.picCropRect.left &&
		!initinfo.picCropRect.top &&
		!initinfo.picCropRect.right &&
		!initinfo.picCropRect.bottom)) {
		initinfo.picCropRect.left = 0;
		initinfo.picCropRect.top = 0;
		initinfo.picCropRect.right = origPicWidth;
		initinfo.picCropRect.bottom = origPicHeight;
	}

	info_msg("CROP left/top/right/bottom %lu %lu %lu %lu\n",
					initinfo.picCropRect.left,
					initinfo.picCropRect.top,
					initinfo.picCropRect.right,
					initinfo.picCropRect.bottom);

	memcpy(&(dec->picCropRect), &(initinfo.picCropRect),
					sizeof(initinfo.picCropRect));

	/* worstSliceSize is in kilo-byte unit */
	dec->phy_slicebuf_size = initinfo.worstSliceSize * 1024;
	dec->stride = dec->picwidth;

	/* Allocate memory for frame status, Mb and Mv report */
	if (dec->frameBufStat.enable) {
		dec->frameBufStat.addr = malloc(initinfo.reportBufSize.frameBufStatBufSize);
		if (!dec->frameBufStat.addr)
			err_msg("malloc_error\n");
	}
	if (dec->mbInfo.enable) {
		dec->mbInfo.addr = malloc(initinfo.reportBufSize.mbInfoBufSize);
		if (!dec->mbInfo.addr)
			err_msg("malloc_error\n");
	}
	if (dec->mvInfo.enable) {
		dec->mvInfo.addr = malloc(initinfo.reportBufSize.mvInfoBufSize);
		if (!dec->mvInfo.addr)
			err_msg("malloc_error\n");
	}

	return 0;
}

int
decoder_open(struct decode *dec)
{
	RetCode ret;
	DecHandle handle = {0};
	DecOpenParam oparam = {0};

	if (dec->cmdl->mapType == LINEAR_FRAME_MAP)
		dec->tiled2LinearEnable = 0;
	else
		/* CbCr interleave must be enabled for tiled map */
		dec->cmdl->chromaInterleave = 1;

	oparam.bitstreamFormat = dec->cmdl->format;
	oparam.bitstreamBuffer = dec->phy_bsbuf_addr;
	oparam.bitstreamBufferSize = STREAM_BUF_SIZE;
	oparam.pBitStream = (Uint8 *)dec->virt_bsbuf_addr;
	oparam.reorderEnable = dec->reorderEnable;
	oparam.mp4DeblkEnable = dec->cmdl->deblock_en;
	oparam.chromaInterleave = dec->cmdl->chromaInterleave;
	oparam.mp4Class = dec->cmdl->mp4_h264Class;

	oparam.avcExtension = dec->cmdl->mp4_h264Class;
	oparam.mjpg_thumbNailDecEnable = 0;
	oparam.mapType = dec->cmdl->mapType;
	oparam.tiled2LinearEnable = dec->tiled2LinearEnable;
	oparam.bitstreamMode = dec->cmdl->bs_mode;
	oparam.jpgLineBufferMode = dec->mjpgLineBufferMode;

	/*
	 * mp4 deblocking filtering is optional out-loop filtering for image
	 * quality. In other words, mpeg4 deblocking is post processing.
	 * So, host application need to allocate new frame buffer.
	 * - On MX27, VPU doesn't support mpeg4 out loop deblocking filtering.
	 * - On MX5X, VPU control the buffer internally and return one
	 *   more buffer after vpu_DecGetInitialInfo().
	 * - On MX32, host application need to set frame buffer externally via
	 *   the command DEC_SET_DEBLOCK_OUTPUT.
	 */

	oparam.psSaveBuffer = dec->phy_ps_buf;
	oparam.psSaveBufferSize = PS_SAVE_SIZE;

	ret = vpu_DecOpen(&handle, &oparam);
	if (ret != RETCODE_SUCCESS) {
		err_msg("vpu_DecOpen failed, ret:%d\n", ret);
		return -1;
	}

	dec->handle = handle;
	return 0;
}

void decoder_close(struct decode *dec)
{
	RetCode ret;

	ret = vpu_DecClose(dec->handle);
	if (ret == RETCODE_FRAME_NOT_COMPLETE) {
		vpu_SWReset(dec->handle, 0);
		ret = vpu_DecClose(dec->handle);
		if (ret != RETCODE_SUCCESS)
			err_msg("vpu_DecClose failed\n");
	}
}

int
decode_test()
{
	struct cmd_line *cmdl;
	struct cmd_line cmdl_input;
	vpu_mem_desc mem_desc = {0};
	vpu_mem_desc ps_mem_desc = {0};
	vpu_mem_desc slice_mem_desc = {0};
	vpu_mem_desc vp8_mbparam_mem_desc = {0};
	struct decode *dec;
	int ret, eos = 0, fill_end_bs = 0, fillsize = 0;
	int using_config_file = 0;

	memset(&cmdl_input, 0, sizeof(cmdl_input));

	cmdl = &cmdl_input;
	cmdl_input.input[0]="a";
	cmdl_input.input[1]="k";
	cmdl_input.input[2]="i";
	cmdl_input.input[3]="y";
	cmdl_input.input[4]="o";
	cmdl_input.input[5]=".";
	cmdl_input.input[6]="m";
	cmdl_input.input[7]="p";
	cmdl_input.input[8]="4";
	cmdl_input.chromaInterleave = 0;
	cmdl_input.bs_mode = 1;
	cmdl_input.mapType = 0;
	cmdl_input.src_fd = open("akiyo.mp4", O_RDONLY, 0);
	//cmdl->output=
	cmdl_input.format=0;
	//cmdl->mp4Class=
	//cmdl->chromaInterleave=
	//cmdl->rotation=
	//cmdl->ipu_rot=
	//cmdl->count=
	//cmdl->deblock=
	//cmdl->dering=
	//cmdl->mirror=
	//cmdl->width=
	//cmdl->height=
	//cmdl->bitrate=
	//cmdl->gop=

//#ifndef COMMON_INIT
	vpu_versioninfo ver;
	ret = vpu_Init(NULL);
	if (ret) {
		err_msg("VPU Init Failure.\n");
		return -1;
	}

	ret = vpu_GetVersionInfo(&ver);
	if (ret) {
		err_msg("Cannot get version info, err:%d\n", ret);
		vpu_UnInit();
		return -1;
	}

	info_msg("VPU firmware version: %d.%d.%d_r%d\n", ver.fw_major, ver.fw_minor,
						ver.fw_release, ver.fw_code);
	info_msg("VPU library version: %d.%d.%d\n", ver.lib_major, ver.lib_minor,
						ver.lib_release);
//#endif

	vpu_v4l_performance_test = 0;
	
	dec = (struct decode *)calloc(1, sizeof(struct decode));
	if (dec == NULL) {
		err_msg("Failed to allocate decode structure\n");
		ret = -1;
		goto err;
	}
	mem_desc.size = STREAM_BUF_SIZE;
	ret = IOGetPhyMem(&mem_desc);
	if (ret) {
		err_msg("Unable to obtain physical mem\n");
		goto err;
	}

	if (IOGetVirtMem(&mem_desc) == -1) {
		err_msg("Unable to obtain virtual mem\n");
		ret = -1;
		goto err;
	}
	dec->phy_bsbuf_addr = mem_desc.phy_addr;
	dec->virt_bsbuf_addr = mem_desc.virt_uaddr;
	dec->reorderEnable = 1;
	dec->tiled2LinearEnable = 0;

	dec->userData.enable = 0;
	dec->mbInfo.enable = 0;
	dec->mvInfo.enable = 0;
	dec->frameBufStat.enable = 0;
	dec->mjpgLineBufferMode = 0;
	dec->mjpegScaleDownRatioWidth = 0;  /* 0,1,2,3 */
	dec->mjpegScaleDownRatioHeight = 0; /* 0,1,2,3 */

	dec->cmdl = cmdl;

	/* open decoder */
	ret = decoder_open(dec);
	if (ret) {
		printf("\nerror\n");	
		goto err;

	}
	cmdl->complete = 1;
	if (dec->cmdl->src_scheme == PATH_NET)
		fillsize = 1024;

	if (cpu_is_mx6x() && (dec->cmdl->format == STD_MJPG) && dec->mjpgLineBufferMode) {
		if (ret < 0)
			goto err1;
		else if (ret == 0) {
			err_msg("no pic in the clip\n");
			ret = -1;
			goto err1;
		}
	} else {
		ret = dec_fill_bsbuffer(dec->handle, cmdl,
				dec->virt_bsbuf_addr,
				(dec->virt_bsbuf_addr + STREAM_BUF_SIZE),
				dec->phy_bsbuf_addr, fillsize, &eos, &fill_end_bs);

		if (fill_end_bs)
			err_msg("Update 0 before seqinit, fill_end_bs=%d\n", fill_end_bs);

		if (ret < 0) {
			err_msg("dec_fill_bsbuffer failed 7\n");
			goto err1;
		}
	}
	cmdl->complete = 0;

	/* parse the bitstream */
	ret = decoder_parse(dec);
	if (ret) {
		printf("decoder parse failed\n");
		goto err1;
	}
	/* allocate slice buf */
	if (cmdl->format == STD_AVC) {
		slice_mem_desc.size = dec->phy_slicebuf_size;
		ret = IOGetPhyMem(&slice_mem_desc);
		if (ret) {
			err_msg("Unable to obtain physical slice save mem\n");
			goto err1;
		}
		dec->phy_slice_buf = slice_mem_desc.phy_addr;
	}

	if (cmdl->format == STD_VP8) {
		vp8_mbparam_mem_desc.size = 68 * (dec->picwidth * dec->picheight / 256);
		ret = IOGetPhyMem(&vp8_mbparam_mem_desc);
		if (ret) {
			err_msg("Unable to obtain physical vp8 mbparam mem\n");
			goto err1;
		}
		dec->phy_vp8_mbparam_buf = vp8_mbparam_mem_desc.phy_addr;
	}

	/* allocate frame buffers */
	ret = decoder_allocate_framebuffer(dec);
	if (ret)
		goto err1;
	printf("ponto 38\n");
	/* start decoding */
	ret = decoder_start(dec);
	printf("ponto 39\n");
err1:
	decoder_close(dec);
	/* free the frame buffers */
	decoder_free_framebuffer(dec);
err:
	if (cmdl->format == STD_AVC) {
		IOFreePhyMem(&slice_mem_desc);
		IOFreePhyMem(&ps_mem_desc);
	}

	if (cmdl->format == STD_VP8)
		IOFreePhyMem(&vp8_mbparam_mem_desc);

	if (dec->mjpg_cached_bsbuf)
		free(dec->mjpg_cached_bsbuf);
	IOFreeVirtMem(&mem_desc);
	IOFreePhyMem(&mem_desc);
	if (dec)
		free(dec);
#ifndef COMMON_INIT
	vpu_UnInit();
#endif
	return ret;
}

static struct input_argument input_arg[MAX_NUM_INSTANCE];
static int instance;
static int using_config_file = 0;

/* Encode or Decode or Loopback */
static char *mainopts = "HE:D:L:T:C:";

static char *options = "i:o:x:n:p:r:f:c:w:h:g:b:d:e:m:u:t:s:l:j:k:a:v:y:q:";

int
open_files(struct cmd_line *cmd)
{
	if (cmd->src_scheme == PATH_FILE) {
#ifdef _FSL_VTS_
    if ( NULL != g_pfnVTSProbe )
    {
        DUT_STREAM_OPEN sFileOpen;
        sFileOpen.strBitstream = g_strInStream;
        sFileOpen.strMode = "rb";
        cmd->src_fd = NULL;
        cmd->src_fd = g_pfnVTSProbe( E_OPEN_BITSTREAM, &sFileOpen );
		if (NULL == cmd->src_fd ) {
			perror("file open");
			return -1;
		}
    }
#else
		cmd->src_fd = open(cmd->input, O_RDONLY, 0);
		if (cmd->src_fd < 0) {
			perror("file open");
			return -1;
		}
		info_msg("Input file \"%s\" opened.\n", cmd->input);
#endif
	} 

	if (cmd->dst_scheme == PATH_FILE) {
#ifndef _FSL_VTS_
		cmd->dst_fd = open(cmd->output, O_CREAT | O_RDWR | O_TRUNC,
					S_IRWXU | S_IRWXG | S_IRWXO);
		if (cmd->dst_fd < 0) {
			perror("file open");

			if (cmd->src_scheme == PATH_FILE)
				close(cmd->src_fd);

			return -1;
		}
		info_msg("Output file \"%s\" opened.\n", cmd->output);
#endif
	}

	return 0;
}

void
close_files(struct cmd_line *cmd)
{
#ifdef _FSL_VTS_
    if ( NULL != g_pfnVTSProbe )
    {
        g_pfnVTSProbe( E_CLOSE_BITSTREAM, &cmd->src_fd );
        cmd->src_fd = NULL;
    }
#else
	if ((cmd->src_fd > 0)) {
		close(cmd->src_fd);
		cmd->src_fd = -1;
	}
#endif

#ifndef _FSL_VTS_
	if ((cmd->dst_fd > 0)) {
		close(cmd->dst_fd);
		cmd->dst_fd = -1;
	}
#endif

	if (cmd->nbuf) {
		free(cmd->nbuf);
		cmd->nbuf = 0;
	}
}

int
check_params(struct cmd_line *cmd, int op)
{
	switch (cmd->format) {
	case STD_MPEG4:
		info_msg("Format: STD_MPEG4\n");
		switch (cmd->mp4_h264Class) {
		case 0:
			info_msg("MPEG4 class: MPEG4\n");
			break;
		case 1:
			info_msg("MPEG4 class: DIVX5.0 or higher\n");
			break;
		case 2:
			info_msg("MPEG4 class: XVID\n");
			break;
		case 5:
			info_msg("MPEG4 class: DIVX4.0\n");
			break;
		default:
			err_msg("Unsupported MPEG4 Class!\n");
			break;
		}
		break;
	case STD_H263:
		info_msg("Format: STD_H263\n");
		break;
	case STD_AVC:
		info_msg("Format: STD_AVC\n");
		switch (cmd->mp4_h264Class) {
		case 0:
			info_msg("AVC\n");
			break;
		case 1:
			info_msg("MVC\n");
			break;
		default:
			err_msg("Unsupported H264 type\n");
			break;
		}
		break;
	case STD_VC1:
		info_msg("Format: STD_VC1\n");
		break;
	case STD_MPEG2:
		info_msg("Format: STD_MPEG2\n");
		break;
	case STD_DIV3:
		info_msg("Format: STD_DIV3\n");
		break;
	case STD_RV:
		info_msg("Format: STD_RV\n");
		break;
	case STD_MJPG:
		info_msg("Format: STD_MJPG\n");
		break;
	case STD_AVS:
		info_msg("Format: STD_AVS\n");
		break;
	case STD_VP8:
		info_msg("Format: STD_VP8\n");
		break;
	default:
		err_msg("Unsupported Format!\n");
		break;
	}

	if (cmd->port == 0) {
		cmd->port = DEFAULT_PORT;
	}

	if (cmd->src_scheme != PATH_FILE && op == DECODE) {
		cmd->src_scheme = PATH_NET;
	}

	if (cmd->src_scheme == PATH_FILE && op == ENCODE) {
		if (cmd->width == 0 || cmd->height == 0) {
			warn_msg("Enter width and height for YUV file\n");
			return -1;
		}
	}

	if (cmd->src_scheme == PATH_V4L2 && op == ENCODE) {
		if (cmd->width == 0)
			cmd->width = 176; /* default */

		if (cmd->height == 0)
			cmd->height = 144;

		if (cmd->width % 16 != 0) {
			cmd->width -= cmd->width % 16;
			warn_msg("width not divisible by 16, adjusted %d\n",
					cmd->width);
		}

		if (cmd->height % 8 != 0) {
			cmd->height -= cmd->height % 8;
			warn_msg("height not divisible by 8, adjusted %d\n",
					cmd->height);
		}

	}

	if (cmd->dst_scheme != PATH_FILE && op == ENCODE) {
		if (cmd->dst_scheme != PATH_NET) {
			warn_msg("No output file specified, using default\n");
			cmd->dst_scheme = PATH_FILE;

			if (cmd->format == STD_MPEG4) {
				strncpy(cmd->output, "enc.mpeg4", 16);
			} else if (cmd->format == STD_H263) {
				strncpy(cmd->output, "enc.263", 16);
			} else {
				strncpy(cmd->output, "enc.264", 16);
			}
		}
	}

	if (cmd->rot_en) {
		if (cmd->rot_angle != 0 && cmd->rot_angle != 90 &&
			cmd->rot_angle != 180 && cmd->rot_angle != 270) {
			warn_msg("Invalid rotation angle. No rotation!\n");
			cmd->rot_en = 0;
			cmd->rot_angle = 0;
		}
	}

	if (cmd->mirror < MIRDIR_NONE || cmd->mirror > MIRDIR_HOR_VER) {
		warn_msg("Invalid mirror angle. Using 0\n");
		cmd->mirror = 0;
	}

	if (!(cmd->format == STD_MPEG4 || cmd->format == STD_H263 ||
	    cmd->format == STD_MPEG2 || cmd->format == STD_DIV3) &&
	    cmd->deblock_en) {
		warn_msg("Deblocking only for MPEG4 and MPEG2. Disabled!\n");
		cmd->deblock_en = 0;
	}

	return 0;
}


int
parse_main_args(int argc, char *argv[])
{
	int status = 0, opt;

	do {
		opt = getopt(argc, argv, mainopts);
		switch (opt)
		{
		case 'D':
			input_arg[instance].mode = DECODE;
			strncpy(input_arg[instance].line, argv[0], 26);
			strncat(input_arg[instance].line, " ", 2);
			strncat(input_arg[instance].line, optarg, 200);
			instance++;
			break;
		case 'E':
			input_arg[instance].mode = ENCODE;
			strncpy(input_arg[instance].line, argv[0], 26);
			strncat(input_arg[instance].line, " ", 2);
			strncat(input_arg[instance].line, optarg, 200);
			instance++;
			break;
		case 'L':
			input_arg[instance].mode = LOOPBACK;
			strncpy(input_arg[instance].line, argv[0], 26);
			strncat(input_arg[instance].line, " ", 2);
			strncat(input_arg[instance].line, optarg, 200);
			instance++;
			break;
                case 'T':
                        input_arg[instance].mode = TRANSCODE;
                        strncpy(input_arg[instance].line, argv[0], 26);
                        strncat(input_arg[instance].line, " ", 2);
                        strncat(input_arg[instance].line, optarg, 200);
                        instance++;
                        break;
		case 'C':
			if (instance > 0) {
			 	warn_msg("-C option not selected because of"
							"other options\n");
				break;
			}

	//		if (parse_config_file(optarg) == 0) {
	//			using_config_file = 1;
	//		}

			break;
		case -1:
			break;
		case 'H':
		default:
			status = -1;
			break;
		}
	} while ((opt != -1) && (status == 0) && (instance < MAX_NUM_INSTANCE));

	optind = 1;
	return status;
}


int
parse_args(int argc, char *argv[], int i)
{
	int status = 0, opt, val;

	input_arg[i].mode = DECODE;

	input_arg[i].cmd.chromaInterleave = 1;
	if (cpu_is_mx6x())
		input_arg[i].cmd.bs_mode = 1;

	do {
		opt = getopt(argc, argv, options);
		switch (opt)
		{
		case 'i':
			strncpy(input_arg[i].cmd.input, optarg, MAX_PATH);
			input_arg[i].cmd.src_scheme = PATH_FILE;
			break;
		case 'o':
			if (input_arg[i].cmd.dst_scheme == PATH_NET) {
				warn_msg("-o ignored because of -n\n");
				break;
			}
			strncpy(input_arg[i].cmd.output, optarg, MAX_PATH);
			input_arg[i].cmd.dst_scheme = PATH_FILE;
			break;
		case 'x':
			val = atoi(optarg);
			if ((input_arg[i].mode == ENCODE) || (input_arg[i].mode == LOOPBACK))
				input_arg[i].cmd.video_node_capture = val;
			else {
				if (val == 1) {
					input_arg[i].cmd.dst_scheme = PATH_IPU;
					info_msg("Display through IPU LIB\n");
					if (cpu_is_mx6x())
						warn_msg("IPU lib is OBSOLETE, please try other renderer\n");
#ifdef BUILD_FOR_ANDROID
				} else if (val == 2) {
					input_arg[i].cmd.dst_scheme = PATH_G2D;
					info_msg("Display through G2D\n");
#endif
				} else {
					input_arg[i].cmd.dst_scheme = PATH_V4L2;
					info_msg("Display through V4L2\n");
					input_arg[i].cmd.video_node = val;
				}
				if (cpu_is_mx27() &&
					(input_arg[i].cmd.dst_scheme == PATH_IPU)) {
					input_arg[i].cmd.dst_scheme = PATH_V4L2;
					warn_msg("ipu lib disp only support in ipuv3\n");
				}
			}
			break;
		case 'n':
			if (input_arg[i].mode == ENCODE) {
				/* contains the ip address */
				strncpy(input_arg[i].cmd.output, optarg, 64);
				input_arg[i].cmd.dst_scheme = PATH_NET;
			} else {
				warn_msg("-n option used only for encode\n");
			}
			break;
		case 'p':
			input_arg[i].cmd.port = atoi(optarg);
			break;
		case 'r':
			input_arg[i].cmd.rot_angle = atoi(optarg);
			if (input_arg[i].cmd.rot_angle)
				input_arg[i].cmd.rot_en = 1;
			break;
		case 'u':
			input_arg[i].cmd.ext_rot_en = atoi(optarg);
			/* ipu/gpu rotation will override vpu rotation */
			if (input_arg[i].cmd.ext_rot_en)
				input_arg[i].cmd.rot_en = 0;
			break;
		case 'f':
			input_arg[i].cmd.format = atoi(optarg);
			break;
		case 'c':
			input_arg[i].cmd.count = atoi(optarg);
			break;
		case 'v':
			input_arg[i].cmd.vdi_motion = optarg[0];
			break;
		case 'w':
			input_arg[i].cmd.width = atoi(optarg);
			break;
		case 'h':
			input_arg[i].cmd.height = atoi(optarg);
			break;
		case 'j':
			input_arg[i].cmd.loff = atoi(optarg);
			break;
		case 'k':
			input_arg[i].cmd.toff = atoi(optarg);
			break;
		case 'g':
			input_arg[i].cmd.gop = atoi(optarg);
			break;
		case 's':
			if (cpu_is_mx6x())
				input_arg[i].cmd.bs_mode = atoi(optarg);
			else
				input_arg[i].cmd.prescan = atoi(optarg);
			break;
		case 'b':
			input_arg[i].cmd.bitrate = atoi(optarg);
			break;
		case 'd':
			input_arg[i].cmd.deblock_en = atoi(optarg);
			break;
		case 'e':
			input_arg[i].cmd.dering_en = atoi(optarg);
			break;
		case 'm':
			input_arg[i].cmd.mirror = atoi(optarg);
			if (input_arg[i].cmd.mirror)
				input_arg[i].cmd.rot_en = 1;
			break;
		case 't':
			input_arg[i].cmd.chromaInterleave = atoi(optarg);
			break;
		case 'l':
			input_arg[i].cmd.mp4_h264Class = atoi(optarg);
			break;
		case 'a':
			input_arg[i].cmd.fps = atoi(optarg);
			break;
		case 'y':
			input_arg[i].cmd.mapType = atoi(optarg);
			break;
		case 'q':
			input_arg[i].cmd.quantParam = atoi(optarg);
			break;
		case -1:
			break;
		default:
			status = -1;
			break;
		}
	} while ((opt != -1) && (status == 0));

	//add
	input_arg[i].cmd.dst_scheme = PATH_V4L2;

	optind = 1;
	return status;
}

static char*
skip(char *ptr)
{
	switch (*ptr) {
	case    '\0':
	case    ' ':
	case    '\t':
	case    '\n':
		break;
	case    '\"':
		ptr++;
		while ((*ptr != '\"') && (*ptr != '\0') && (*ptr != '\n')) {
			ptr++;
		}
		if (*ptr != '\0') {
			*ptr++ = '\0';
		}
		break;
	default :
		while ((*ptr != ' ') && (*ptr != '\t')
			&& (*ptr != '\0') && (*ptr != '\n')) {
			ptr++;
		}
		if (*ptr != '\0') {
			*ptr++ = '\0';
		}
		break;
	}

	while ((*ptr == ' ') || (*ptr == '\t') || (*ptr == '\n')) {
		ptr++;
	}

	return (ptr);
}

void
get_arg(char *buf, int *argc, char *argv[])
{
	char *ptr;
	*argc = 0;

	while ( (*buf == ' ') || (*buf == '\t'))
		buf++;

	for (ptr = buf; *argc < 32; (*argc)++) {
		if (!*ptr)
			break;
		argv[*argc] = ptr + (*ptr == '\"');
		ptr = skip(ptr);
	}

	argv[*argc] = NULL;
}

int main(int argc, char *argv[])

{
	int err, nargc, i, ret;
	char *pargv[32] = {0}, *dbg_env;
	pthread_t sigtid;
#ifdef COMMON_INIT
	vpu_versioninfo ver;
#endif
	int ret_thr;
	static int instance;
	instance = 1;

	char *soc_list[] = {"i.MX6Q", "i.MX6QP", "i.MX6DL", " "};

	//ret = soc_version_check(soc_list);
	//if (ret == 0) {
//		printf("mxc_vpu_test.out not supported on current soc\n");
//		return 0;
//	}
	printf("Ponto 1\n");
	ret = 0;

#ifndef COMMON_INIT
	srand((unsigned)time(0)); /* init seed of rand() */
#endif

	dbg_env=getenv("VPU_TEST_DBG");
	if (dbg_env)
		vpu_test_dbg_level = atoi(dbg_env);
	else
		vpu_test_dbg_level = 0;

	err = parse_main_args(argc, argv);
	if (err) {
		goto usage;
	}
	printf("Ponto 3\n");
	if (!instance) {
		goto usage;
	}
	printf("Ponto 2\n");
	info_msg("VPU test program built on %s %s\n", __DATE__, __TIME__);
#ifndef _FSL_VTS_
	sigemptyset(&sigset);
	sigaddset(&sigset, SIGINT);
	pthread_sigmask(SIG_BLOCK, &sigset, NULL);
	pthread_create(&sigtid, NULL, (void *)&signal_thread, NULL);
#endif

#ifdef COMMON_INIT
	err = vpu_Init(NULL);
	if (err) {
		err_msg("VPU Init Failure.\n");
		return -1;
	}
	else {
		printf("VPU init ok\n");
	}

	err = vpu_GetVersionInfo(&ver);
	if (err) {
		err_msg("Cannot get version info, err:%d\n", err);
		vpu_UnInit();
		return -1;
	}

	info_msg("VPU firmware version: %d.%d.%d_r%d\n", ver.fw_major, ver.fw_minor,
						ver.fw_release, ver.fw_code);
	info_msg("VPU library version: %d.%d.%d\n", ver.lib_major, ver.lib_minor,
						ver.lib_release);
#else
	// just to enable cpu_is_xx() to be used in command line parsing
	err = vpu_Init(NULL);
	if (err) {
		err_msg("VPU Init Failure.\n");
		return -1;
	}	else {
		printf("VPU init ok 2\n");
	}

	vpu_UnInit();

#endif

	if (instance > 1) {
		for (i = 0; i < instance; i++) {
#ifndef COMMON_INIT
			/* sleep roughly a frame interval to test multi-thread race
			   especially vpu_Init/vpu_UnInit */
			usleep((int)(rand()%ONE_FRAME_INTERV));
#endif
			if (using_config_file == 0) {
				get_arg(input_arg[i].line, &nargc, pargv);
				err = parse_args(nargc, pargv, i);
				if (err) {
					vpu_UnInit();
					goto usage;
				}
			}

			if (check_params(&input_arg[i].cmd,
						input_arg[i].mode) == 0) {
				if (open_files(&input_arg[i].cmd) == 0) {
					if (input_arg[i].mode == DECODE) {
					     pthread_create(&input_arg[i].tid,
						   NULL,
						   (void *)&decode_test,
						   (void *)&input_arg[i].cmd);
					} else if (input_arg[i].mode ==
							ENCODE) {
					     pthread_create(&input_arg[i].tid,
						   NULL,
						   (void *)&decode_test, //(void *)&encode_test,
						   (void *)&input_arg[i].cmd);
					}
				}
			}

		}
	} else {
	//	if (using_config_file == 0) {
			get_arg(input_arg[0].line, &nargc, pargv);
			err = parse_args(nargc, pargv, 0);
			if (err) {
				vpu_UnInit();
				goto usage;
			}
	//	}

		if (check_params(&input_arg[0].cmd, input_arg[0].mode) == 0) {
			if (open_files(&input_arg[0].cmd) == 0) {
				if (input_arg[0].mode == DECODE) {
					ret = decode_test(&input_arg[0].cmd);
				} 
				/*	else if (input_arg[0].mode == ENCODE) {
					ret = encode_test(&input_arg[0].cmd);
                                } else if (input_arg[0].mode == TRANSCODE) {
                                        ret = transcode_test(&input_arg[0].cmd);
				} */

				close_files(&input_arg[0].cmd);
			} else {
				ret = -1;
			}
		} else {
			ret = -1;
		}

		//if (input_arg[0].mode == LOOPBACK) {
		//	encdec_test(&input_arg[0].cmd);
		//}
	}

/*	if (instance > 1) {
		for (i = 0; i < instance; i++) {
			if (input_arg[i].tid != 0) {
				pthread_join(input_arg[i].tid, (void *)&ret_thr);
				if (ret_thr)
					ret = -1;
				close_files(&input_arg[i].cmd);
			}
		}
	} */

#ifdef COMMON_INIT
	vpu_UnInit();
#endif
	return ret;

usage:
	//info_msg("\n%s", usage);
	return -1;

}
