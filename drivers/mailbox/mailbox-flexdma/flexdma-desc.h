/* Broadcom FlexDMA Mailbox Driver
 *
 * Copyright (C) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * FlexDMA descriptor library
 */

#ifndef __FLEXDMA_DESC_H__
#define __FLEXDMA_DESC_H__

#include <linux/types.h>
#include <linux/device.h>
#include <linux/mailbox/brcm-message.h>

extern u64 flexdma_read_desc(void *desc_ptr);

extern void flexdma_write_desc(void *desc_ptr, u64 desc);

extern u32 flexdma_cmpl_desc_to_reqid(u64 cmpl_desc);

extern int flexdma_cmpl_desc_to_error(u64 cmpl_desc);

extern bool flexdma_is_next_table_desc(void *desc_ptr);

extern u64 flexdma_next_table_desc(u32 toggle, dma_addr_t next_addr);

extern u64 flexdma_null_desc(u32 toogle);

extern u32 flexdma_estimate_header_desc_count(u32 nhcnt);

extern bool flexdma_sanity_check(struct brcm_message *msg);

extern u32 flexdma_estimate_nonheader_desc_count(struct brcm_message *msg);

extern int flexdma_dma_map(struct device *dev, struct brcm_message *msg);

extern void flexdma_dma_unmap(struct device *dev, struct brcm_message *msg);

extern void *flexdma_write_descs(struct brcm_message *msg, u32 nhcnt,
				 u32 reqid, void *desc_ptr, u32 toggle,
				 void *start_desc, void *end_desc);

#endif /* __FLEXDMA_DESC_H__ */
