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

#include <asm/byteorder.h>
#include <linux/dma-mapping.h>
#include <linux/printk.h>

#include "flexdma-desc.h"

/* Completion descriptor format */
#define CMPL_OPAQUE_SHIFT			0
#define CMPL_OPAQUE_MASK			0xffff
#define CMPL_ENGINE_STATUS_SHIFT		16
#define CMPL_ENGINE_STATUS_MASK			0xffff
#define CMPL_DME_STATUS_SHIFT			32
#define CMPL_DME_STATUS_MASK			0xffff
#define CMPL_RM_STATUS_SHIFT			48
#define CMPL_RM_STATUS_MASK			0xffff

/* Completion DME status code */
#define DME_STATUS_MEM_COR_ERR			BIT(0)
#define DME_STATUS_MEM_UCOR_ERR			BIT(1)
#define DME_STATUS_FIFO_UNDERFLOW		BIT(2)
#define DME_STATUS_FIFO_OVERFLOW		BIT(3)
#define DME_STATUS_RRESP_ERR			BIT(4)
#define DME_STATUS_BRESP_ERR			BIT(5)
#define DME_STATUS_ERROR_MASK			(DME_STATUS_MEM_COR_ERR | \
						 DME_STATUS_MEM_UCOR_ERR | \
						 DME_STATUS_FIFO_UNDERFLOW | \
						 DME_STATUS_FIFO_OVERFLOW | \
						 DME_STATUS_RRESP_ERR | \
						 DME_STATUS_BRESP_ERR)

/* Completion RM status code */
#define RM_STATUS_CODE_SHIFT			0
#define RM_STATUS_CODE_MASK			0x3ff
#define RM_STATUS_CODE_GOOD			0x0
#define RM_STATUS_CODE_AE_TIMEOUT		0x3ff

/* General descriptor format */
#define DESC_TYPE_SHIFT				60
#define DESC_TYPE_MASK				0xf
#define DESC_PAYLOAD_SHIFT			0
#define DESC_PAYLOAD_MASK			0x0fffffffffffffff

/* Null descriptor format  */
#define NULL_TYPE				0
#define NULL_TOGGLE_SHIFT			58
#define NULL_TOGGLE_MASK			0x1

/* Header descriptor format */
#define HEADER_TYPE				1
#define HEADER_TOGGLE_SHIFT			58
#define HEADER_TOGGLE_MASK			0x1
#define HEADER_ENDPKT_SHIFT			57
#define HEADER_ENDPKT_MASK			0x1
#define HEADER_STARTPKT_SHIFT			56
#define HEADER_STARTPKT_MASK			0x1
#define HEADER_BDCOUNT_SHIFT			36
#define HEADER_BDCOUNT_MASK			0x1f
#define HEADER_FLAGS_SHIFT			16
#define HEADER_FLAGS_MASK			0xffff
#define HEADER_OPAQUE_SHIFT			0
#define HEADER_OPAQUE_MASK			0xffff

/* Source (SRC) descriptor format */
#define SRC_TYPE				2
#define SRC_LENGTH_SHIFT			44
#define SRC_LENGTH_MASK				0xffff
#define SRC_ADDR_SHIFT				0
#define SRC_ADDR_MASK				0x00000fffffffffff

/* Destination (DST) descriptor format */
#define DST_TYPE				3
#define DST_LENGTH_SHIFT			44
#define DST_LENGTH_MASK				0xffff
#define DST_ADDR_SHIFT				0
#define DST_ADDR_MASK				0x00000fffffffffff

/* Immediate (IMM) descriptor format */
#define IMM_TYPE				4
#define IMM_DATA_SHIFT				0
#define IMM_DATA_MASK				0x0fffffffffffffff

/* Next pointer (NPTR) descriptor format */
#define NPTR_TYPE				5
#define NPTR_TOGGLE_SHIFT			58
#define NPTR_TOGGLE_MASK			0x1
#define NPTR_ADDR_SHIFT				0
#define NPTR_ADDR_MASK				0x00000fffffffffff

/* Mega source (MSRC) descriptor format */
#define MSRC_TYPE				6
#define MSRC_LENGTH_SHIFT			44
#define MSRC_LENGTH_MASK			0xffff
#define MSRC_ADDR_SHIFT				0
#define MSRC_ADDR_MASK				0x00000fffffffffff

/* Mega destination (MDST) descriptor format */
#define MDST_TYPE				7
#define MDST_LENGTH_SHIFT			44
#define MDST_LENGTH_MASK			0xffff
#define MDST_ADDR_SHIFT				0
#define MDST_ADDR_MASK				0x00000fffffffffff

/* Source with tlast (SRCT) descriptor format */
#define SRCT_TYPE				8
#define SRCT_LENGTH_SHIFT			44
#define SRCT_LENGTH_MASK			0xffff
#define SRCT_ADDR_SHIFT				0
#define SRCT_ADDR_MASK				0x00000fffffffffff

/* Destination with tlast (DSTT) descriptor format */
#define DSTT_TYPE				9
#define DSTT_LENGTH_SHIFT			44
#define DSTT_LENGTH_MASK			0xffff
#define DSTT_ADDR_SHIFT				0
#define DSTT_ADDR_MASK				0x00000fffffffffff

/* Immediate with tlast (IMMT) descriptor format */
#define IMMT_TYPE				10
#define IMMT_DATA_SHIFT				0
#define IMMT_DATA_MASK				0x0fffffffffffffff

/* Descriptor helper macros */
#define DESC_DEC(_d, _s, _m)			(((_d) >> (_s)) & (_m))
#define DESC_ENC(_d, _v, _s, _m)		\
			do { \
				(_d) &= ~((u64)(_m) << (_s)); \
				(_d) |= (((u64)(_v) & (_m)) << (_s)); \
			} while (0)

u64 flexdma_read_desc(void *desc_ptr)
{
	return le64_to_cpu(*((u64 *)desc_ptr));
}

void flexdma_write_desc(void *desc_ptr, u64 desc)
{
	*((u64 *)desc_ptr) = cpu_to_le64(desc);
}

u32 flexdma_cmpl_desc_to_reqid(u64 cmpl_desc)
{
	return (u32)(cmpl_desc & CMPL_OPAQUE_MASK);
}

int flexdma_cmpl_desc_to_error(u64 cmpl_desc)
{
	u32 status;

	status = DESC_DEC(cmpl_desc, CMPL_DME_STATUS_SHIFT,
			  CMPL_DME_STATUS_MASK);
	if (status & DME_STATUS_ERROR_MASK)
		return -EIO;

	status = DESC_DEC(cmpl_desc, CMPL_RM_STATUS_SHIFT,
			  CMPL_RM_STATUS_MASK);
	status &= RM_STATUS_CODE_MASK;
	if (status == RM_STATUS_CODE_AE_TIMEOUT)
		return -ETIMEDOUT;

	return 0;
}

bool flexdma_is_next_table_desc(void *desc_ptr)
{
	u64 desc = flexdma_read_desc(desc_ptr);
	u32 type = DESC_DEC(desc, DESC_TYPE_SHIFT, DESC_TYPE_MASK);

	return (type == NPTR_TYPE) ? true : false;
}

u64 flexdma_next_table_desc(u32 toggle, dma_addr_t next_addr)
{
	u64 desc = 0;

	DESC_ENC(desc, NPTR_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, toggle, NPTR_TOGGLE_SHIFT, NPTR_TOGGLE_MASK);
	DESC_ENC(desc, next_addr, NPTR_ADDR_SHIFT, NPTR_ADDR_MASK);

	return desc;
}

u64 flexdma_null_desc(u32 toggle)
{
	u64 desc = 0;

	DESC_ENC(desc, NULL_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, toggle, NULL_TOGGLE_SHIFT, NULL_TOGGLE_MASK);

	return desc;
}

static void flexdma_flip_header_toogle(void *desc_ptr)
{
	u64 desc = flexdma_read_desc(desc_ptr);

	if (desc & ((u64)0x1 << HEADER_TOGGLE_SHIFT))
		desc &= ~((u64)0x1 << HEADER_TOGGLE_SHIFT);
	else
		desc |= ((u64)0x1 << HEADER_TOGGLE_SHIFT);

	flexdma_write_desc(desc_ptr, desc);
}

static void flexdma_enqueue_desc(u64 desc, void **desc_ptr, u32 *toggle,
				 void *start_desc, void *end_desc)
{
	flexdma_write_desc(*desc_ptr, desc);

	/* Point to next descriptor */
	*desc_ptr += sizeof(desc);
	if (*desc_ptr == end_desc)
		*desc_ptr = start_desc;

	/* Skip next pointer descriptors */
	while (flexdma_is_next_table_desc(*desc_ptr)) {
		*toggle = (*toggle) ? 0 : 1;
		*desc_ptr += sizeof(desc);
		if (*desc_ptr == end_desc)
			*desc_ptr = start_desc;
	}
}

static u64 flexdma_header_desc(u32 toggle, u32 startpkt, u32 endpkt,
			       u32 bdcount, u32 flags, u32 opaque)
{
	u64 desc = 0;

	DESC_ENC(desc, HEADER_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, toggle, HEADER_TOGGLE_SHIFT, HEADER_TOGGLE_MASK);
	DESC_ENC(desc, startpkt, HEADER_STARTPKT_SHIFT, HEADER_STARTPKT_MASK);
	DESC_ENC(desc, endpkt, HEADER_ENDPKT_SHIFT, HEADER_ENDPKT_MASK);
	DESC_ENC(desc, bdcount, HEADER_BDCOUNT_SHIFT, HEADER_BDCOUNT_MASK);
	DESC_ENC(desc, flags, HEADER_FLAGS_SHIFT, HEADER_FLAGS_MASK);
	DESC_ENC(desc, opaque, HEADER_OPAQUE_SHIFT, HEADER_OPAQUE_MASK);

	return desc;
}

static u64 flexdma_src_desc(dma_addr_t addr, unsigned int length)
{
	u64 desc = 0;

	DESC_ENC(desc, SRC_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, length, SRC_LENGTH_SHIFT, SRC_LENGTH_MASK);
	DESC_ENC(desc, addr, SRC_ADDR_SHIFT, SRC_ADDR_MASK);

	return desc;
}

static u64 flexdma_msrc_desc(dma_addr_t addr, unsigned int length_div_16)
{
	u64 desc = 0;

	DESC_ENC(desc, MSRC_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, length_div_16, MSRC_LENGTH_SHIFT, MSRC_LENGTH_MASK);
	DESC_ENC(desc, addr, MSRC_ADDR_SHIFT, MSRC_ADDR_MASK);

	return desc;
}

static u64 flexdma_dst_desc(dma_addr_t addr, unsigned int length)
{
	u64 desc = 0;

	DESC_ENC(desc, DST_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, length, DST_LENGTH_SHIFT, DST_LENGTH_MASK);
	DESC_ENC(desc, addr, DST_ADDR_SHIFT, DST_ADDR_MASK);

	return desc;
}

static u64 flexdma_mdst_desc(dma_addr_t addr, unsigned int length_div_16)
{
	u64 desc = 0;

	DESC_ENC(desc, MDST_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, length_div_16, MDST_LENGTH_SHIFT, MDST_LENGTH_MASK);
	DESC_ENC(desc, addr, MDST_ADDR_SHIFT, MDST_ADDR_MASK);

	return desc;
}

static u64 flexdma_imm_desc(u64 data)
{
	u64 desc = 0;

	DESC_ENC(desc, IMM_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, data, IMM_DATA_SHIFT, IMM_DATA_MASK);

	return desc;
}

static u64 flexdma_srct_desc(dma_addr_t addr, unsigned int length)
{
	u64 desc = 0;

	DESC_ENC(desc, SRCT_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, length, SRCT_LENGTH_SHIFT, SRCT_LENGTH_MASK);
	DESC_ENC(desc, addr, SRCT_ADDR_SHIFT, SRCT_ADDR_MASK);

	return desc;
}

static u64 flexdma_dstt_desc(dma_addr_t addr, unsigned int length)
{
	u64 desc = 0;

	DESC_ENC(desc, DSTT_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, length, DSTT_LENGTH_SHIFT, DSTT_LENGTH_MASK);
	DESC_ENC(desc, addr, DSTT_ADDR_SHIFT, DSTT_ADDR_MASK);

	return desc;
}

static u64 flexdma_immt_desc(u64 data)
{
	u64 desc = 0;

	DESC_ENC(desc, IMMT_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, data, IMMT_DATA_SHIFT, IMMT_DATA_MASK);

	return desc;
}

static bool flexdma_sg_sanity_check(struct brcm_message *msg)
{
	struct scatterlist *sg;

	/*
	 * TODO: Use multiple headers for longer scatterlist
	 * using startpkt and endpkt fields.
	 */

	if (!msg->sg.src || !msg->sg.dst)
		return false;
	if ((sg_nents(msg->sg.src) + sg_nents(msg->sg.dst)) > 31)
		return false;
	for (sg = msg->sg.src; sg; sg = sg_next(sg)) {
		if (sg->length & 0xf) {
			if (sg->length > SRC_LENGTH_MASK)
				return false;
		} else {
			if (sg->length > (MSRC_LENGTH_MASK * 16))
				return false;
		}
	}
	for (sg = msg->sg.dst; sg; sg = sg_next(sg)) {
		if (sg->length & 0xf) {
			if (sg->length > DST_LENGTH_MASK)
				return false;
		} else {
			if (sg->length > (MDST_LENGTH_MASK * 16))
				return false;
		}
	}

	return true;
}

static u32 flexdma_sg_estimate_desc_count(struct brcm_message *msg)
{
	/*
	 * TODO: Use multiple headers for longer scatterlist
	 * using startpkt and endpkt fields.
	 */

	/*
	 * Total descriptors = 1x header descriptor +
	 *			source descriptor(s) +
	 *			destination descriptor(s) +
	 *			1x null descriptor
	 */
	return sg_nents(msg->sg.src) + sg_nents(msg->sg.dst) + 2;
}

static int flexdma_sg_dma_map(struct device *dev, struct brcm_message *msg)
{
	int rc;

	rc = dma_map_sg(dev, msg->sg.src, sg_nents(msg->sg.src),
			DMA_TO_DEVICE);
	if (rc < 0)
		return rc;

	rc = dma_map_sg(dev, msg->sg.dst, sg_nents(msg->sg.dst),
			DMA_FROM_DEVICE);
	if (rc < 0) {
		dma_unmap_sg(dev, msg->sg.src, sg_nents(msg->sg.src),
			     DMA_TO_DEVICE);
		return rc;
	}

	return 0;
}

static void flexdma_sg_dma_unmap(struct device *dev, struct brcm_message *msg)
{
	dma_unmap_sg(dev, msg->sg.dst, sg_nents(msg->sg.dst), DMA_FROM_DEVICE);
	dma_unmap_sg(dev, msg->sg.src, sg_nents(msg->sg.src), DMA_TO_DEVICE);
}

static void *flexdma_sg_write_descs(struct brcm_message *msg,
				    u32 reqid, void *desc_ptr, u32 toggle,
				    void *start_desc, void *end_desc)
{
	u64 desc;
	void *orig_desc_ptr = desc_ptr;
	struct scatterlist *sg;

	/*
	 * TODO: Use multiple headers for longer scatterlist
	 * using startpkt and endpkt fields.
	 */

	/* Header descriptor */
	desc = flexdma_header_desc(!toggle, 0x1, 0x1,
				   flexdma_sg_estimate_desc_count(msg) - 2,
				   0x0, reqid);
	flexdma_enqueue_desc(desc, &desc_ptr, &toggle, start_desc, end_desc);

	/* Source descriptor(s) */
	for (sg = msg->sg.src; sg; sg = sg_next(sg)) {
		if (sg_dma_len(sg) & 0xf)
			desc = flexdma_src_desc(sg_dma_address(sg),
						sg_dma_len(sg));
		else
			desc = flexdma_msrc_desc(sg_dma_address(sg),
						 sg_dma_len(sg)/16);
		flexdma_enqueue_desc(desc, &desc_ptr, &toggle,
				     start_desc, end_desc);
	}

	/* Destination descriptor(s) */
	for (sg = msg->sg.dst; sg; sg = sg_next(sg)) {
		if (sg_dma_len(sg) & 0xf)
			desc = flexdma_dst_desc(sg_dma_address(sg),
						sg_dma_len(sg));
		else
			desc = flexdma_mdst_desc(sg_dma_address(sg),
						 sg_dma_len(sg)/16);
		flexdma_enqueue_desc(desc, &desc_ptr, &toggle,
				     start_desc, end_desc);
	}

	/* Null descriptor with invalid toggle bit */
	flexdma_write_desc(desc_ptr, flexdma_null_desc(!toggle));

	/* Flip toggle bit in header */
	flexdma_flip_header_toogle(orig_desc_ptr);

	return desc_ptr;
}

static bool flexdma_sba_sanity_check(struct brcm_message *msg)
{
	u32 i;

	/*
	 * TODO: Use multiple headers for large number of
	 * source and/or destinetion buffers using startpkt
	 * and endpkt fields.
	 */

	if (!msg->sba.cmds || !msg->sba.cmds_count)
		return false;

	for (i = 0; i < msg->sba.cmds_count; i++) {
		if ((msg->sba.cmds[i].flags & BRCM_SBA_CMD_TYPE_B) &&
		    (msg->sba.cmds[i].input_len > SRCT_LENGTH_MASK))
			return false;
		if ((msg->sba.cmds[i].flags & BRCM_SBA_CMD_TYPE_C) &&
		    (msg->sba.cmds[i].input_len > SRCT_LENGTH_MASK))
			return false;
		if ((msg->sba.cmds[i].flags & BRCM_SBA_CMD_HAS_RESP) &&
		    (msg->sba.cmds[i].resp_len > DSTT_LENGTH_MASK))
			return false;
		if ((msg->sba.cmds[i].flags & BRCM_SBA_CMD_HAS_OUTPUT) &&
		    (msg->sba.cmds[i].output_len > DSTT_LENGTH_MASK))
			return false;
	}

	return true;
}

static u32 flexdma_sba_estimate_desc_count(struct brcm_message *msg)
{
	u32 i, cnt;

	/*
	 * TODO: Use multiple headers for large number of
	 * source and/or destinetion buffers using startpkt
	 * and endpkt fields.
	 */

	cnt = 0;
	for (i = 0; i < msg->sba.cmds_count; i++) {
		cnt++;

		if ((msg->sba.cmds[i].flags & BRCM_SBA_CMD_TYPE_B) ||
		    (msg->sba.cmds[i].flags & BRCM_SBA_CMD_TYPE_C))
			cnt++;

		if (msg->sba.cmds[i].flags & BRCM_SBA_CMD_HAS_RESP)
			cnt++;

		if (msg->sba.cmds[i].flags & BRCM_SBA_CMD_HAS_OUTPUT)
			cnt++;
	}

	/* +2 for header at start and null descriptor at end */
	return cnt + 2;
}

static void *flexdma_sba_write_descs(struct brcm_message *msg,
				     u32 reqid, void *desc_ptr, u32 toggle,
				     void *start_desc, void *end_desc)
{
	u64 desc;
	unsigned int i;
	struct brcm_sba_command *c;
	void *orig_desc_ptr = desc_ptr;

	/*
	 * TODO: Use multiple headers for large number of
	 * source and/or destinetion buffers using startpkt
	 * and endpkt fields.
	 */

	/* Header descriptor */
	desc = flexdma_header_desc(!toggle, 0x1, 0x1,
				   flexdma_sba_estimate_desc_count(msg) - 2,
				   0x0, reqid);
	flexdma_enqueue_desc(desc, &desc_ptr, &toggle, start_desc, end_desc);

	/* Convert SBA commands into descriptors */
	for (i = 0; i < msg->sba.cmds_count; i++) {
		c = &msg->sba.cmds[i];

		if ((c->flags & BRCM_SBA_CMD_HAS_RESP) &&
		    (c->flags & BRCM_SBA_CMD_HAS_OUTPUT)) {
			/* Destination response descriptor */
			desc = flexdma_dst_desc(c->resp, c->resp_len);
			flexdma_enqueue_desc(desc, &desc_ptr, &toggle,
					     start_desc, end_desc);
		} else if (c->flags & BRCM_SBA_CMD_HAS_RESP) {
			/* Destination response with tlast descriptor */
			desc = flexdma_dstt_desc(c->resp, c->resp_len);
			flexdma_enqueue_desc(desc, &desc_ptr, &toggle,
					     start_desc, end_desc);
		}

		if (c->flags & BRCM_SBA_CMD_HAS_OUTPUT) {
			/* Destination with tlast descriptor */
			desc = flexdma_dstt_desc(c->output, c->output_len);
			flexdma_enqueue_desc(desc, &desc_ptr, &toggle,
					     start_desc, end_desc);
		}

		if (c->flags & BRCM_SBA_CMD_TYPE_B) {
			/* Command as immediate descriptor */
			desc = flexdma_imm_desc(c->cmd);
			flexdma_enqueue_desc(desc, &desc_ptr, &toggle,
					     start_desc, end_desc);
		} else {
			/* Command as immediate descriptor with tlast */
			desc = flexdma_immt_desc(c->cmd);
			flexdma_enqueue_desc(desc, &desc_ptr, &toggle,
					     start_desc, end_desc);
		}

		if ((c->flags & BRCM_SBA_CMD_TYPE_B) ||
		    (c->flags & BRCM_SBA_CMD_TYPE_C)) {
			/* Source with tlast descriptor */
			desc = flexdma_srct_desc(c->input, c->input_len);
			flexdma_enqueue_desc(desc, &desc_ptr, &toggle,
					     start_desc, end_desc);
		}
	}

	/* Null descriptor with invalid toggle bit */
	flexdma_write_desc(desc_ptr, flexdma_null_desc(!toggle));

	/* Flip toggle bit in header */
	flexdma_flip_header_toogle(orig_desc_ptr);

	return desc_ptr;
}

bool flexdma_sanity_check(struct brcm_message *msg)
{
	if (!msg)
		return false;

	switch (msg->type) {
	case BRCM_MESSAGE_SG:
		return flexdma_sg_sanity_check(msg);
	case BRCM_MESSAGE_SBA:
		return flexdma_sba_sanity_check(msg);
	default:
		return false;
	};
}

u32 flexdma_estimate_desc_count(struct brcm_message *msg)
{
	if (!msg)
		return 0;

	switch (msg->type) {
	case BRCM_MESSAGE_SG:
		return flexdma_sg_estimate_desc_count(msg);
	case BRCM_MESSAGE_SBA:
		return flexdma_sba_estimate_desc_count(msg);
	default:
		return 0;
	};
}

int flexdma_dma_map(struct device *dev, struct brcm_message *msg)
{
	if (!dev || !msg)
		return -EINVAL;

	switch (msg->type) {
	case BRCM_MESSAGE_SG:
		return flexdma_sg_dma_map(dev, msg);
	default:
		break;
	};

	return 0;
}

void flexdma_dma_unmap(struct device *dev, struct brcm_message *msg)
{
	if (!dev || !msg)
		return;

	switch (msg->type) {
	case BRCM_MESSAGE_SG:
		flexdma_sg_dma_unmap(dev, msg);
		break;
	default:
		break;
	};
}

void *flexdma_write_descs(struct brcm_message *msg,
			  u32 reqid, void *desc_ptr, u32 toggle,
			  void *start_desc, void *end_desc)
{
	if (!msg || !desc_ptr || !start_desc || !end_desc)
		return ERR_PTR(-ENOTSUPP);

	if ((desc_ptr < start_desc) || (end_desc <= desc_ptr))
		return ERR_PTR(-ERANGE);

	switch (msg->type) {
	case BRCM_MESSAGE_SG:
		return flexdma_sg_write_descs(msg, reqid,
					      desc_ptr, toggle,
					      start_desc, end_desc);
	case BRCM_MESSAGE_SBA:
		return flexdma_sba_write_descs(msg, reqid,
					       desc_ptr, toggle,
					       start_desc, end_desc);
	default:
		return ERR_PTR(-ENOTSUPP);
	};
}
