/*
 * Copyright (C) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * Unicam camera host exports
 */

#ifndef BCM_UNICAM_CPI_H
#define BCM_UNICAM_CPI_H

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/regmap.h>

#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/soc_camera.h>
#include <media/drv-intf/soc_mediabus.h>
#include <linux/media-bus-format.h>

enum buffer_mode {
	BUFFER_SINGLE,
	BUFFER_DOUBLE,
	BUFFER_TRIGGER,
};

enum host_mode {
	CSI2,
	DVP = 1,
	CSI1CCP2,
	INVALID,
};

struct buffer_desc {
	u32 start;
	u32 size;
	u32 ls;
};

struct unicam_cpi_generic {
	bool devbusy;
	enum host_mode mode;
	bool db_en;
	bool trigger;
};

struct unicam_camera_dev {
	struct soc_camera_device *icd;
	struct soc_camera_host soc_host;
	struct soc_camera_desc *pdata;
	/* generic driver related */
	unsigned int irq;
	struct device *dev;
	/* h/w specific */
	void __iomem *base;
	void __iomem *asiu_pad_ctrl;
	struct clk *clk;
	unsigned int clk_freq;
	struct clk *asiu_clk;
	/* data structure needed to support streaming */
	spinlock_t lock;
	struct vb2_v4l2_buffer *active;
	struct list_head video_buffer_list;
	int curr;
	atomic_t streaming;
	u32 skip_frames;
	struct completion stop;
	atomic_t stopping;
	enum buffer_mode b_mode;
	struct v4l2_selection sel;
	u32 panic_count;
	atomic_t cam_triggered;
	struct v4l2_format active_fmt;
	struct unicam_cpi_generic cam_state;
	int data_width;
	bool camera_mode_continuous;
	bool last_buffer_in_queue;
};

void disable_cpi_interface(struct unicam_camera_dev *dev);
void unicam_camera_cpi_config(struct unicam_camera_dev *dev);
void unicam_camera_reset_standby(struct unicam_camera_dev *dev);

void unicam_cpi_update_addr(struct unicam_camera_dev *dev,
			struct buffer_desc *im0, struct buffer_desc *im1,
			struct buffer_desc *dat0, struct buffer_desc *dat1);

void unicam_cpi_trigger_cap(struct unicam_camera_dev *dev);
unsigned int unicam_cpi_get_rx_stat(struct unicam_camera_dev *dev, int ack);

/* ISTA register ... only FS, FE and LCI are allowed */
int unicam_cpi_get_int_stat(struct unicam_camera_dev *dev, int rx_stat,
		int ack);

extern void unicam_reg_dump(struct unicam_camera_dev *dev);

#endif
