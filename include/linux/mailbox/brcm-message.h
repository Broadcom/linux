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

enum brcm_message_type {
	BRCM_MESSAGE_UNKNOWN = 0,
	BRCM_MESSAGE_SG,
	BRCM_MESSAGE_SPU,
	BRCM_MESSAGE_SBA,
	BRCM_MESSAGE_MAX,
};

struct brcm_sba_command {
	u64 cmd;
#define BRCM_SBA_CMD_TYPE_A		(1 << 0)
#define BRCM_SBA_CMD_TYPE_B		(1 << 1)
#define BRCM_SBA_CMD_TYPE_C		(1 << 2)
#define BRCM_SBA_CMD_HAS_RESP		(1 << 3)
#define BRCM_SBA_CMD_HAS_OUTPUT		(1 << 4)
	u64 flags;
	dma_addr_t input;
	size_t input_len;
	dma_addr_t resp;
	size_t resp_len;
	dma_addr_t output;
	size_t output_len;
};

struct brcm_message {
	enum brcm_message_type type;
	union {
		struct {
			struct scatterlist *src;
			struct scatterlist *dst;
		} sg;
		struct {
			struct brcm_sba_command *cmds;
			unsigned int cmds_count;
		} sba;
	};
	void *ctx;
	int error;
};

#endif /* _LINUX_BRCM_MESSAGE_H_ */
