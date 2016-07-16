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

#ifndef _BCM_IPROC_UAPI_DTE_H_
#define _BCM_IPROC_UAPI_DTE_H_

#include <linux/ioctl.h>

struct dte_client_data {
	unsigned int client;
	unsigned int enable;
	unsigned int divider;
};

struct dte_timestamp {
	unsigned int client;
	struct timespec ts;
};

#define DTE_IOCTL_BASE          0xFC
#define DTE_IO(nr)              _IO(DTE_IOCTL_BASE, nr)
#define DTE_IOR(nr, type)       _IOR(DTE_IOCTL_BASE, nr, type)
#define DTE_IOW(nr, type)       _IOW(DTE_IOCTL_BASE, nr, type)
#define DTE_IOWR(nr, type)      _IOWR(DTE_IOCTL_BASE, nr, type)

#define DTE_IOCTL_GET_TIMESTAMP     DTE_IOWR(0x01, struct dte_timestamp)
#define DTE_IOCTL_SET_TIME          DTE_IOW(0x02, struct timespec)
#define DTE_IOCTL_GET_TIME          DTE_IOR(0x03, struct timespec)
#define DTE_IOCTL_ADJ_TIME          DTE_IOW(0x04, int64_t)
#define DTE_IOCTL_ADJ_FREQ          DTE_IOW(0x05, int32_t)
#define DTE_IOCTL_ENABLE_CLIENT_TS  DTE_IOW(0x06, struct dte_client_data)
#define DTE_IOCTL_DISPLAY_DRV_INF   DTE_IO(0x07)

#endif
