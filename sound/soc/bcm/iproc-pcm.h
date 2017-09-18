/*
 * Copyright (C) 2014-2015 Broadcom Corporation
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
#ifndef __IPROC_PCM_H__
#define __IPROC_PCM_H__

struct iproc_pcm_dma_info {
	void __iomem *audio;
	int portnum;
	struct snd_pcm_substream *substream;
};

struct iproc_rb_info {
	struct device *dev;
	void __iomem *audio;

	int irq_num;

	int num_playback;
	struct iproc_pcm_dma_info *rb_state_play;

	int num_capture;
	struct iproc_pcm_dma_info *rb_state_cap;
};

extern int iproc_pcm_platform_register(struct device *dev,
					struct iproc_rb_info *rb_info);
extern int iproc_pcm_platform_unregister(struct device *dev);

#endif
