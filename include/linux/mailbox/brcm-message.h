/*
 *  Copyright (C) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Common header for Broadcom mailbox messages which is shared across
 * Broadcom SoCs and Broadcom mailbox client drivers.
 */

#ifndef _LINUX_BRCM_MESSAGE_H_
#define _LINUX_BRCM_MESSAGE_H_

#include <linux/scatterlist.h>

enum brcm_message_types {
	BRCM_MESSAGE_UNKNOWN = 0,
	BRCM_MESSAGE_SG,
	BRCM_MESSAGE_SPU,
	BRCM_MESSAGE_MAX,
};

struct brcm_message {
	int type;
	union {
		struct {
			struct scatterlist *src;
			struct scatterlist *dst;
		} sg;
	};
	void *ctx;
	int error;
};

#endif /* _LINUX_BRCM_MESSAGE_H_ */
