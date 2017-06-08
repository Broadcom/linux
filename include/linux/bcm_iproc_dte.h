/*
 * Copyright 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 (GPLv2) for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 (GPLv2) along with this source code.
 */

#ifndef _BCM_IPROC_DTE_H_
#define _BCM_IPROC_DTE_H_

#include <linux/cdev.h>
#include <linux/kfifo.h>
#include <linux/timer.h>
#include <linux/wait.h>

struct dte_user_info {
	struct file *fp; /* File opened by user */
	bool internal_user;
	wait_queue_head_t ts_wait_queue;
};

struct dte_client_mapping {
	u32 client_index;
	u32 lts_index;
	u32 reg_offset;
	u32 shift;
	char *name;
	u32 div_status;
};

struct bcm_dte {
	struct platform_device *pdev;
	struct cdev dte_cdev;
	struct class *dte_class;
	struct kfifo *recv_fifo;
	int irq;
	dev_t devt;
	uint32_t fifoof;
	uint32_t fifouf;
	uint32_t *kfifoof;
	uint32_t src_ena;
	spinlock_t lock;
	struct mutex en_ts_lock;
	struct mutex mutex;
	struct timespec ts_ref;
	uint32_t timestamp_overflow_last;
	void __iomem *audioeav;
	uint32_t irq_interval_ns;
	uint32_t usr_cnt; /* tracks num of users */
	struct dte_user_info *user;
	struct timer_list fifo_timer;
	struct timer_list ovf_timer;
	unsigned int num_of_clients;
	struct dte_client_mapping *dte_cli;
	uint32_t nco_susp_val;

	int (*enable_ts)(struct bcm_dte *dte,
		unsigned int client, bool enable,
		uint32_t divider, struct file *fp);
	int (*get_ts)(struct bcm_dte *dte,
		unsigned int client, struct timespec *ts);

	int (*nco_set_time)(struct bcm_dte *dte, struct timespec *ts);
	int (*nco_get_time)(struct bcm_dte *dte, struct timespec *ts);
	int (*nco_adj_time)(struct bcm_dte *dte, int64_t delta);
	int (*nco_adj_freq)(struct bcm_dte *dte, int32_t ppb);
	int (*nco_get_freq_adj)(struct bcm_dte *dte, int32_t *ppb);
};

#endif
