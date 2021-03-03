// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 Broadcom.
 */

/*
 * Broadcom FS5 ring manager Mailbox Driver
 *
 * Each Broadcom FS5 offload engine is implemented as an
 * extension to Broadcom FS5 ring manager. The FS5 ring
 * manager provides a set of rings which can be used to submit
 * work to a FS5 offload engines.
 *
 * This driver creates a mailbox controller using a set of FS5RM
 * rings where each mailbox channel represents a separate FS5RM ring.
 */

#include <asm/barrier.h>
#include <asm/byteorder.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox/brcm-message.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

/* ====== FS5RM register defines ===== */

/* FS5RM configuration */
#define RING_REGS_SIZE					0x10000
#define RING_DESC_SIZE					8
#define RING_DESC_INDEX(offset)				\
			((offset) / RING_DESC_SIZE)
#define RING_DESC_OFFSET(index)				\
			((index) * RING_DESC_SIZE)
#define RING_MAX_REQ_COUNT				1024
#define RING_BD_ALIGN_ORDER				12
#define RING_BD_ALIGN_CHECK(addr)			\
			(!((addr) & ((BIT(RING_BD_ALIGN_ORDER)) - 1)))
#define RING_BD_DESC_PER_REQ				32
#define RING_BD_DESC_COUNT				\
			(RING_MAX_REQ_COUNT * RING_BD_DESC_PER_REQ)
#define RING_BD_SIZE					\
			(RING_BD_DESC_COUNT * RING_DESC_SIZE)
#define RING_CMPL_ALIGN_ORDER				13
#define RING_CMPL_DESC_COUNT				RING_MAX_REQ_COUNT
#define RING_CMPL_SIZE					\
			(RING_CMPL_DESC_COUNT * RING_DESC_SIZE)
#define RING_VER_MAGIC					0x76303032
#define MSI_COUNT_THRESHOLD				0x1
#define MSI_TIMER_VAL					0x2710

/* Per-Ring register offsets */
#define RING_VER					0x000
#define RING_BD_START_ADDRESS_LSB			0x004
#define RING_BD_READ_PTR				0x008
#define RING_BD_WRITE_PTR				0x00c
#define RING_BD_READ_PTR_DDR_LS				0x010
#define RING_BD_READ_PTR_DDR_MS				0x014
#define RING_CMPL_START_ADDR_LSB			0x018
#define RING_CMPL_WRITE_PTR				0x01c
#define RING_NUM_REQ_RECV_LS				0x020
#define RING_NUM_REQ_RECV_MS				0x024
#define RING_NUM_REQ_TRANS_LS				0x028
#define RING_NUM_REQ_TRANS_MS				0x02c
#define RING_NUM_REQ_OUTSTAND				0x030
#define RING_CONTROL					0x034
#define RING_FLUSH_DONE					0x038
#define RING_MSI_ADDR_LS				0x03c
#define RING_MSI_ADDR_MS				0x040
#define RING_MSI_CONTROL				0x048
#define RING_BD_READ_PTR_DDR_CONTROL			0x04c
#define RING_MSI_DATA_VALUE				0x064
#define RING_BD_START_ADDRESS_MSB			0x078
#define RING_CMPL_START_ADDR_MSB			0x07c
#define RING_DOORBELL_BD_WRITE_COUNT			0x074

/* Register RING_BD_START_ADDR fields */
#define BD_LAST_UPDATE_HW_SHIFT				28
#define BD_LAST_UPDATE_HW_MASK				0x1

/* Register RING_CONTROL fields */
#define CONTROL_MASK_DISABLE_CONTROL			12
#define CONTROL_FLUSH_SHIFT				5
#define CONTROL_ACTIVE_SHIFT				4
#define CONTROL_RATE_ADAPT_MASK				0xf
#define CONTROL_RATE_DYNAMIC				0x0
#define CONTROL_RATE_FAST				0x8
#define CONTROL_RATE_MEDIUM				0x9
#define CONTROL_RATE_SLOW				0xa
#define CONTROL_RATE_IDLE				0xb

/* Register RING_FLUSH_DONE fields */
#define FLUSH_DONE_MASK					0x1

/* Register RING_MSI_CONTROL fields */
#define MSI_TIMER_VAL_SHIFT				16
#define MSI_TIMER_VAL_MASK				0xffff
#define MSI_ENABLE_SHIFT				15
#define MSI_ENABLE_MASK					0x1
#define MSI_COUNT_SHIFT					0
#define MSI_COUNT_MASK					0x3ff

/* Register RING_BD_READ_PTR_DDR_CONTROL fields */
#define BD_READ_PTR_DDR_TIMER_VAL_SHIFT			16
#define BD_READ_PTR_DDR_TIMER_VAL_MASK			0xffff
#define BD_READ_PTR_DDR_ENABLE_SHIFT			15
#define BD_READ_PTR_DDR_ENABLE_MASK			0x1

/* ====== FS5RM ring descriptor defines ===== */

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

/* Header descriptor format */
#define HEADER_TYPE				1
#define HEADER_ENDPKT_SHIFT			57
#define HEADER_ENDPKT_MASK			0x1
#define HEADER_STARTPKT_SHIFT			56
#define HEADER_STARTPKT_MASK			0x1
#define HEADER_BDCOUNT_SHIFT			36
#define HEADER_BDCOUNT_MASK			0x1f
#define HEADER_BDCOUNT_MAX			HEADER_BDCOUNT_MASK
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

/* ====== FS5RM data structures ===== */

struct fs5rm_ring {
	/* Unprotected members */
	unsigned int ring_idx;
	struct fs5rm_mbox *mbox;
	void __iomem *regs;
	bool irq_requested;
	unsigned int irq;
	cpumask_t irq_aff_hint;
	struct brcm_message *requests[RING_MAX_REQ_COUNT];
	void *bd_base;
	dma_addr_t bd_dma_base;
	u32 bd_write_offset;
	void *cmpl_base;
	dma_addr_t cmpl_dma_base;
	/* Protected members */
	spinlock_t lock;
	DECLARE_BITMAP(requests_bmap, RING_MAX_REQ_COUNT);
	u32 cmpl_read_offset;
};

struct fs5rm_mbox {
	struct device *dev;
	void __iomem *regs;
	u32 num_rings;
	struct fs5rm_ring *rings;
	struct dma_pool *bd_pool;
	struct dma_pool *cmpl_pool;
	struct mbox_controller controller;
};

/* ====== FS5RM ring descriptor helper routines ===== */

static u64 build_desc(u64 val, u32 shift, u64 mask)
{
	return((val & mask) << shift);
}

static u64 fs5rm_read_desc(void *desc_ptr)
{
	return le64_to_cpu(*((u64 *)desc_ptr));
}

static void fs5rm_write_desc(void *desc_ptr, u64 desc)
{
	*((u64 *)desc_ptr) = cpu_to_le64(desc);
}

static u32 fs5rm_cmpl_desc_to_reqid(u64 cmpl_desc)
{
	return (u32)(cmpl_desc & CMPL_OPAQUE_MASK);
}

static int fs5rm_cmpl_desc_to_error(u64 cmpl_desc)
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

static bool fs5rm_is_next_table_desc(void *desc_ptr)
{
	u64 desc = fs5rm_read_desc(desc_ptr);
	u32 type = DESC_DEC(desc, DESC_TYPE_SHIFT, DESC_TYPE_MASK);

	return (type == NPTR_TYPE) ? true : false;
}

static u64 fs5rm_next_table_desc(dma_addr_t next_addr)
{
	return (build_desc(NPTR_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK) |
		build_desc(next_addr, NPTR_ADDR_SHIFT, NPTR_ADDR_MASK));
}

static u64 fs5rm_null_desc(void)
{
	return build_desc(NULL_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
}

static u64 fs5rm_header_desc(u32 spkt, u32 epkt,
			     u32 bdcount, u32 flags, u32 opaque)
{
	return (build_desc(HEADER_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK) |
		build_desc(spkt, HEADER_STARTPKT_SHIFT, HEADER_STARTPKT_MASK) |
		build_desc(epkt, HEADER_ENDPKT_SHIFT, HEADER_ENDPKT_MASK) |
		build_desc(bdcount, HEADER_BDCOUNT_SHIFT, HEADER_BDCOUNT_MASK) |
		build_desc(flags, HEADER_FLAGS_SHIFT, HEADER_FLAGS_MASK) |
		build_desc(opaque, HEADER_OPAQUE_SHIFT, HEADER_OPAQUE_MASK));
}

static void fs5rm_enqueue_desc(u32 nhpos, u32 nhcnt, u32 reqid, bool *nxt_pg,
			       u64 desc, void **desc_ptr, void *start_desc,
			       void *end_desc)
{
	u64 d;
	u32 nhavail, _startpkt, _endpkt, _bdcount;

	/*
	 * Each request or packet start with a HEADER descriptor followed
	 * by one or more non-HEADER descriptors (SRC, SRCT, MSRC, DST,
	 * DSTT, MDST, IMM, and IMMT). The number of non-HEADER descriptors
	 * following a HEADER descriptor is represented by BDCOUNT field
	 * of HEADER descriptor. The max value of BDCOUNT field is 31 which
	 * means we can only have 31 non-HEADER descriptors following one
	 * HEADER descriptor.
	 *
	 * In general use, number of non-HEADER descriptors can easily go
	 * beyond 31. To tackle this situation, we have packet (or request)
	 * extension bits (STARTPKT and ENDPKT) in the HEADER descriptor.
	 *
	 * To use packet extension, the first HEADER descriptor of request
	 * (or packet) will have STARTPKT=1 and ENDPKT=0. The intermediate
	 * HEADER descriptors will have STARTPKT=0 and ENDPKT=0. The last
	 * HEADER descriptor will have STARTPKT=0 and ENDPKT=1.
	 */

	if ((nhpos % HEADER_BDCOUNT_MAX == 0) && (nhcnt - nhpos)) {
		/* Prepare the header descriptor */
		nhavail = (nhcnt - nhpos);
		_startpkt = (nhpos == 0) ? 0x1 : 0x0;
		_endpkt = (nhavail <= HEADER_BDCOUNT_MAX) ? 0x1 : 0x0;
		_bdcount = (nhavail <= HEADER_BDCOUNT_MAX) ?
				nhavail : HEADER_BDCOUNT_MAX;
		if (nhavail <= HEADER_BDCOUNT_MAX)
			_bdcount = nhavail;
		else
			_bdcount = HEADER_BDCOUNT_MAX;
		d = fs5rm_header_desc(_startpkt, _endpkt,
				      _bdcount, 0x0, reqid);

		/* Write header descriptor */
		fs5rm_write_desc(*desc_ptr, d);

		/* Point to next descriptor */
		*desc_ptr += sizeof(desc);
		if (*desc_ptr == end_desc)
			*desc_ptr = start_desc;

		/* Skip next pointer descriptors */
		while (fs5rm_is_next_table_desc(*desc_ptr)) {
			*nxt_pg = true;
			*desc_ptr += sizeof(desc);
			if (*desc_ptr == end_desc)
				*desc_ptr = start_desc;
		}
	}

	/* Write desired descriptor */
	fs5rm_write_desc(*desc_ptr, desc);

	/* Point to next descriptor */
	*desc_ptr += sizeof(desc);
	if (*desc_ptr == end_desc)
		*desc_ptr = start_desc;

	/* Skip next pointer descriptors */
	while (fs5rm_is_next_table_desc(*desc_ptr)) {
		*nxt_pg = true;
		*desc_ptr += sizeof(desc);
		if (*desc_ptr == end_desc)
			*desc_ptr = start_desc;
	}
}

static u64 fs5rm_src_desc(dma_addr_t addr, unsigned int len)
{
	return (build_desc(SRC_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK) |
		build_desc(len, SRC_LENGTH_SHIFT, SRC_LENGTH_MASK) |
		build_desc(addr, SRC_ADDR_SHIFT, SRC_ADDR_MASK));
}

static u64 fs5rm_msrc_desc(dma_addr_t addr, unsigned int len_div_16)
{
	return (build_desc(MSRC_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK) |
		build_desc(len_div_16, MSRC_LENGTH_SHIFT, MSRC_LENGTH_MASK) |
		build_desc(addr, MSRC_ADDR_SHIFT, MSRC_ADDR_MASK));
}

static u64 fs5rm_dst_desc(dma_addr_t addr, unsigned int len)
{
	return (build_desc(DST_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK) |
		build_desc(len, DST_LENGTH_SHIFT, DST_LENGTH_MASK) |
		build_desc(addr, DST_ADDR_SHIFT, DST_ADDR_MASK));
}

static u64 fs5rm_mdst_desc(dma_addr_t addr, unsigned int len_div_16)
{
	return (build_desc(MDST_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK) |
		build_desc(len_div_16, MDST_LENGTH_SHIFT, MDST_LENGTH_MASK) |
		build_desc(addr, MDST_ADDR_SHIFT, MDST_ADDR_MASK));
}

static bool fs5rm_spu_sanity_check(struct brcm_message *msg)
{
	struct scatterlist *sg;

	if (!msg->spu.src || !msg->spu.dst)
		return false;
	for (sg = msg->spu.src; sg; sg = sg_next(sg)) {
		if (sg->length & 0xf) {
			if (sg->length > SRC_LENGTH_MASK)
				return false;
		} else {
			if (sg->length > (MSRC_LENGTH_MASK * 16))
				return false;
		}
	}
	for (sg = msg->spu.dst; sg; sg = sg_next(sg)) {
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

static int fs5rm_dma_map(struct device *dev,
			 struct brcm_message *msg,
			 int *rx_ent, int *tx_ent)
{
	int rc;

	rc = dma_map_sg(dev, msg->spu.src, sg_nents(msg->spu.src),
			DMA_TO_DEVICE);
	if (rc < 0)
		return rc;
	*rx_ent = rc;

	rc = dma_map_sg(dev, msg->spu.dst, sg_nents(msg->spu.dst),
			DMA_FROM_DEVICE);
	if (rc < 0) {
		dma_unmap_sg(dev, msg->spu.src, sg_nents(msg->spu.src),
			     DMA_TO_DEVICE);
		return rc;
	}
	*tx_ent = rc;

	return 0;
}

static void fs5rm_spu_dma_unmap(struct device *dev, struct brcm_message *msg)
{
	dma_unmap_sg(dev, msg->spu.dst, sg_nents(msg->spu.dst),
		     DMA_FROM_DEVICE);
	dma_unmap_sg(dev, msg->spu.src, sg_nents(msg->spu.src),
		     DMA_TO_DEVICE);
}

static void *fs5rm_spu_write_descs(struct fs5rm_ring *ring,
				   struct brcm_message *msg, u32 nhcnt,
				   u32 reqid, void *desc_ptr,
				   void *start_desc, void *end_desc)
{
	u64 d;
	u32 nhpos = 0;
	struct scatterlist *src_sg = msg->spu.src, *dst_sg = msg->spu.dst;
	bool is_nxt_pg = false;

	while (src_sg) {
		if (!sg_dma_len(src_sg)) {
			src_sg = sg_next(src_sg);
			continue;
		}
		if (sg_dma_len(src_sg) & 0xf)
			d = fs5rm_src_desc(sg_dma_address(src_sg),
					   sg_dma_len(src_sg));
		else
			d = fs5rm_msrc_desc(sg_dma_address(src_sg),
					    sg_dma_len(src_sg) / 16);
		fs5rm_enqueue_desc(nhpos, nhcnt, reqid, &is_nxt_pg,
				   d, &desc_ptr, start_desc,
				   end_desc);
		nhpos++;
		src_sg = sg_next(src_sg);
	}

	while (dst_sg) {
		if (!sg_dma_len(dst_sg)) {
			dst_sg = sg_next(dst_sg);
			continue;
		}
		if (sg_dma_len(dst_sg) & 0xf)
			d = fs5rm_dst_desc(sg_dma_address(dst_sg),
					   sg_dma_len(dst_sg));
		else
			d = fs5rm_mdst_desc(sg_dma_address(dst_sg),
					    sg_dma_len(dst_sg) / 16);
		fs5rm_enqueue_desc(nhpos, nhcnt, reqid, &is_nxt_pg,
				   d, &desc_ptr, start_desc,
				   end_desc);
		nhpos++;
		dst_sg = sg_next(dst_sg);
	}

	/* Ensure that descriptors have been written to memory */
	wmb();

	if (is_nxt_pg)
		writel_relaxed(nhcnt + 2, ring->regs +
			       RING_DOORBELL_BD_WRITE_COUNT);
	else
		writel_relaxed(nhcnt + 1, ring->regs +
			       RING_DOORBELL_BD_WRITE_COUNT);

	return desc_ptr;
}

static bool fs5rm_sanity_check(struct brcm_message *msg)
{
	if (!msg)
		return false;

	switch (msg->type) {
	case BRCM_MESSAGE_SPU:
		return fs5rm_spu_sanity_check(msg);
	default:
		return false;
	};
}

static void fs5rm_dma_unmap(struct device *dev, struct brcm_message *msg)
{
	if (!dev || !msg)
		return;

	fs5rm_spu_dma_unmap(dev, msg);
}

static void *fs5rm_write_descs(struct fs5rm_ring *ring,
			       struct brcm_message *msg,
			       u32 nhcnt, u32 reqid, void *desc_ptr,
			       void *start_desc, void *end_desc)
{
	if (!msg || !desc_ptr || !start_desc || !end_desc)
		return ERR_PTR(-EINVAL);

	if ((desc_ptr < start_desc) || (end_desc <= desc_ptr))
		return ERR_PTR(-ERANGE);

	return fs5rm_spu_write_descs(ring, msg, nhcnt, reqid,
				     desc_ptr, start_desc, end_desc);
}

/* ====== FS5RM driver helper routines ===== */

static int fs5rm_new_request(struct fs5rm_ring *ring,
			     struct brcm_message *batch_msg,
			     struct brcm_message *msg)
{
	void *next;
	unsigned long flags;
	u32  count, nhcnt;
	bool exit_cleanup = false;
	int ret = 0, reqid;
	int rx_ent, tx_ent;

	/* Do sanity check on message */
	if (!fs5rm_sanity_check(msg))
		return -EINVAL;
	msg->error = 0;

	spin_lock_irqsave(&ring->lock, flags);
	reqid = bitmap_find_free_region(ring->requests_bmap,
					RING_MAX_REQ_COUNT, 0);
	spin_unlock_irqrestore(&ring->lock, flags);
	/* Check if resources are available */
	if (unlikely(reqid < 0))
		return -EBUSY;

	ring->requests[reqid] = msg;

	/* Do DMA mappings for the message */
	ret = fs5rm_dma_map(ring->mbox->dev, msg, &rx_ent, &tx_ent);
	if (ret < 0) {
		ring->requests[reqid] = NULL;
		spin_lock_irqsave(&ring->lock, flags);
		bitmap_release_region(ring->requests_bmap, reqid, 0);
		spin_unlock_irqrestore(&ring->lock, flags);
		return ret;
	}

	/*
	 * Number required descriptors = number of non-header descriptors +
	 *				 number of header descriptors
	 */
	nhcnt = rx_ent + tx_ent;
	count = DIV_ROUND_UP(nhcnt, HEADER_BDCOUNT_MAX) + nhcnt + 1;

	/* Write descriptors to ring */
	next = fs5rm_write_descs(ring, msg, nhcnt, reqid,
				 ring->bd_base + ring->bd_write_offset,
				 ring->bd_base, ring->bd_base + RING_BD_SIZE);
	if (IS_ERR(next)) {
		ret = PTR_ERR(next);
		exit_cleanup = true;
		goto exit;
	}

	/* Save ring BD write offset */
	ring->bd_write_offset = (u32)(next - ring->bd_base);

exit:
	/* Update error status in message */
	msg->error = ret;

	/* Cleanup if we failed */
	if (exit_cleanup) {
		fs5rm_dma_unmap(ring->mbox->dev, msg);
		ring->requests[reqid] = NULL;
		spin_lock_irqsave(&ring->lock, flags);
		bitmap_release_region(ring->requests_bmap, reqid, 0);
		spin_unlock_irqrestore(&ring->lock, flags);
	}

	return ret;
}

static int fs5rm_process_completions(struct fs5rm_ring *ring)
{
	u64 desc;
	int err, count = 0;
	unsigned long flags;
	struct brcm_message *msg = NULL;
	u32 reqid, cmpl_read_offset, cmpl_write_offset;
	struct mbox_chan *chan = &ring->mbox->controller.chans[ring->ring_idx];

	spin_lock_irqsave(&ring->lock, flags);

	/*
	 * Get current completion read and write offset
	 *
	 * Note: We should read completion write pointer atleast once
	 * after we get a MSI interrupt because HW maintains internal
	 * MSI status which will allow next MSI interrupt only after
	 * completion write pointer is read.
	 */
	cmpl_write_offset = readl_relaxed(ring->regs + RING_CMPL_WRITE_PTR);
	cmpl_write_offset *= RING_DESC_SIZE;
	cmpl_read_offset = ring->cmpl_read_offset;
	ring->cmpl_read_offset = cmpl_write_offset;

	spin_unlock_irqrestore(&ring->lock, flags);

	/* Ensure read/write h/w pointers are loaded */
	rmb();

	/* For each completed request notify mailbox clients */
	reqid = 0;
	while (cmpl_read_offset != cmpl_write_offset) {
		/* Dequeue next completion descriptor */
		desc = *((u64 *)(ring->cmpl_base + cmpl_read_offset));

		/* Next read offset */
		cmpl_read_offset += RING_DESC_SIZE;
		if (cmpl_read_offset == RING_CMPL_SIZE)
			cmpl_read_offset = 0;

		/* Decode error from completion descriptor */
		err = fs5rm_cmpl_desc_to_error(desc);
		if (err < 0) {
			dev_warn(ring->mbox->dev,
				 "ring%d cmpl error\n",
				 ring->ring_idx);
		}

		/* Determine request id from completion descriptor */
		reqid = fs5rm_cmpl_desc_to_reqid(desc);

		/* Determine message pointer based on reqid */
		msg = ring->requests[reqid];
		if (!msg) {
			dev_err(ring->mbox->dev,
				"HW BUG? bad message rcvd on ring: %d\n",
				ring->ring_idx);
			return -ENOMSG;
		}

		/* Release reqid for recycling */
		ring->requests[reqid] = NULL;
		spin_lock_irqsave(&ring->lock, flags);
		bitmap_release_region(ring->requests_bmap, reqid, 0);
		spin_unlock_irqrestore(&ring->lock, flags);

		/* Unmap DMA mappings */
		fs5rm_dma_unmap(ring->mbox->dev, msg);

		/* Give-back message to mailbox client */
		msg->error = err;
		mbox_chan_received_data(chan, msg);

		/* Increment number of completions processed */
		count++;
	}

	return count;
}

/* ====== FS5RM interrupt handler ===== */

static irqreturn_t fs5rm_irq_event(int irq, void *dev_id)
{
	struct fs5rm_ring *ring = (struct fs5rm_ring *)dev_id;
	u32 cmpl_write_offset, cmpl_read_offset;
	unsigned long flags;

	spin_lock_irqsave(&ring->lock, flags);
	cmpl_write_offset = readl_relaxed(ring->regs + RING_CMPL_WRITE_PTR);
	cmpl_read_offset = ring->cmpl_read_offset;
	spin_unlock_irqrestore(&ring->lock, flags);

	cmpl_write_offset *= RING_DESC_SIZE;

	/*
	 * Don't schedule irq thread if there is no data to process.
	 * It means peek_data() has already processed all data.
	 */
	if (cmpl_write_offset == cmpl_read_offset)
		return IRQ_HANDLED;
	else
		return IRQ_WAKE_THREAD;
}

static irqreturn_t fs5rm_irq_thread(int irq, void *dev_id)
{
	fs5rm_process_completions(dev_id);

	return IRQ_HANDLED;
}

/* ====== FS5RM mailbox callbacks ===== */

static int fs5rm_send_data(struct mbox_chan *chan, void *data)
{
	int i, rc;
	struct fs5rm_ring *ring = chan->con_priv;
	struct brcm_message *msg = data;

	if (msg->type == BRCM_MESSAGE_BATCH) {
		for (i = msg->batch.msgs_queued;
		     i < msg->batch.msgs_count; i++) {
			rc = fs5rm_new_request(ring, msg,
					       &msg->batch.msgs[i]);
			if (rc) {
				msg->error = rc;
				return rc;
			}
			msg->batch.msgs_queued++;
		}
		return 0;
	}

	return fs5rm_new_request(ring, NULL, data);
}

static bool fs5rm_peek_data(struct mbox_chan *chan)
{
	int cnt = fs5rm_process_completions(chan->con_priv);

	return (cnt > 0) ? true : false;
}

static int fs5rm_startup(struct mbox_chan *chan)
{
	u64 d;
	u32 val, off;
	int ret = 0;
	dma_addr_t next_addr;
	struct fs5rm_ring *ring = chan->con_priv;
	u32 bd_high, bd_low;
	u32 cmpl_high, cmpl_low;

	/* Disable/inactivate ring */
	writel_relaxed(0x0, ring->regs + RING_CONTROL);

	/* Allocate BD memory */
	ring->bd_base = dma_pool_alloc(ring->mbox->bd_pool,
				       GFP_KERNEL, &ring->bd_dma_base);
	if (!ring->bd_base) {
		dev_err(ring->mbox->dev,
			"can't allocate BD memory for ring%d\n",
			ring->ring_idx);
		ret = -ENOMEM;
		goto fail;
	}

	/* Configure next table pointer entries in BD memory */
	for (off = 0; off < RING_BD_SIZE; off += RING_DESC_SIZE) {
		next_addr = off + RING_DESC_SIZE;
		if (next_addr == RING_BD_SIZE)
			next_addr = 0;
		next_addr += ring->bd_dma_base;
		if (RING_BD_ALIGN_CHECK(next_addr))
			d = fs5rm_next_table_desc(next_addr);
		else
			d = fs5rm_null_desc();
		fs5rm_write_desc(ring->bd_base + off, d);
	}

	/* Allocate completion memory */
	ring->cmpl_base = dma_pool_zalloc(ring->mbox->cmpl_pool,
					  GFP_KERNEL, &ring->cmpl_dma_base);
	if (!ring->cmpl_base) {
		dev_err(ring->mbox->dev,
			"can't allocate completion memory for ring%d\n",
			ring->ring_idx);
		ret = -ENOMEM;
		goto fail_free_bd_memory;
	}

	/* Request IRQ */
	if (ring->irq == UINT_MAX) {
		dev_err(ring->mbox->dev,
			"ring%d IRQ not available\n", ring->ring_idx);
		ret = -ENODEV;
		goto fail_free_cmpl_memory;
	}
	ret = request_threaded_irq(ring->irq,
				   fs5rm_irq_event,
				   fs5rm_irq_thread,
				   0, dev_name(ring->mbox->dev), ring);
	if (ret) {
		dev_err(ring->mbox->dev,
			"failed to request ring%d IRQ\n", ring->ring_idx);
		goto fail_free_cmpl_memory;
	}
	ring->irq_requested = true;

	/* Set IRQ affinity hint */
	ring->irq_aff_hint = CPU_MASK_NONE;
	val = ring->mbox->num_rings;
	val = (num_online_cpus() < val) ? val / num_online_cpus() : 1;
	cpumask_set_cpu((ring->ring_idx / val) % num_online_cpus(),
			&ring->irq_aff_hint);
	ret = irq_set_affinity_hint(ring->irq, &ring->irq_aff_hint);
	if (ret) {
		dev_err(ring->mbox->dev,
			"failed to set IRQ affinity hint for ring%d\n",
			ring->ring_idx);
		goto fail_free_irq;
	}

	/* Program BD start address */
	bd_high = upper_32_bits(ring->bd_dma_base);
	bd_low = lower_32_bits(ring->bd_dma_base);
	writel_relaxed(bd_low, ring->regs + RING_BD_START_ADDRESS_LSB);
	writel_relaxed(bd_high, ring->regs + RING_BD_START_ADDRESS_MSB);

	/* BD write pointer will be same as HW write pointer */
	ring->bd_write_offset = 0;

	/* Program completion start address */
	cmpl_high = upper_32_bits(ring->cmpl_dma_base);
	cmpl_low = lower_32_bits(ring->cmpl_dma_base);
	writel_relaxed(cmpl_low, ring->regs + RING_CMPL_START_ADDR_LSB);
	writel_relaxed(cmpl_high, ring->regs + RING_CMPL_START_ADDR_MSB);

	/* Completion read pointer will be same as HW write pointer */
	ring->cmpl_read_offset = 0;
	ring->cmpl_read_offset *= RING_DESC_SIZE;

	/* Read ring Tx, Rx, and Outstanding counts to clear */
	readl_relaxed(ring->regs + RING_NUM_REQ_RECV_LS);
	readl_relaxed(ring->regs + RING_NUM_REQ_RECV_MS);
	readl_relaxed(ring->regs + RING_NUM_REQ_TRANS_LS);
	readl_relaxed(ring->regs + RING_NUM_REQ_TRANS_MS);
	readl_relaxed(ring->regs + RING_NUM_REQ_OUTSTAND);

	/* Configure RING_MSI_CONTROL */
	val = 0;
	val |= (MSI_TIMER_VAL << MSI_TIMER_VAL_SHIFT);
	val |= BIT(MSI_ENABLE_SHIFT);
	val |= (MSI_COUNT_THRESHOLD << MSI_COUNT_SHIFT);
	writel_relaxed(val, ring->regs + RING_MSI_CONTROL);

	/* Enable/activate ring */
	val = BIT(CONTROL_ACTIVE_SHIFT);
	writel_relaxed(val, ring->regs + RING_CONTROL);

	return 0;

fail_free_irq:
	free_irq(ring->irq, ring);
	ring->irq_requested = false;
fail_free_cmpl_memory:
	dma_pool_free(ring->mbox->cmpl_pool,
		      ring->cmpl_base, ring->cmpl_dma_base);
	ring->cmpl_base = NULL;
fail_free_bd_memory:
	dma_pool_free(ring->mbox->bd_pool,
		      ring->bd_base, ring->bd_dma_base);
	ring->bd_base = NULL;
fail:
	return ret;
}

static void fs5rm_shutdown(struct mbox_chan *chan)
{
	u32 reqid;
	unsigned int timeout;
	struct brcm_message *msg;
	struct fs5rm_ring *ring = chan->con_priv;

	/* Disable/inactivate ring */
	writel_relaxed(0x0, ring->regs + RING_CONTROL);

	/* Set ring flush state */
	timeout = 1000; /* timeout of 1s */
	writel_relaxed(BIT(CONTROL_FLUSH_SHIFT),
		       ring->regs + RING_CONTROL);
	do {
		if (readl_relaxed(ring->regs + RING_FLUSH_DONE) &
		    FLUSH_DONE_MASK)
			break;
		mdelay(1);
	} while (--timeout);
	if (!timeout)
		dev_err(ring->mbox->dev,
			"ring%d flush state timedout\n", ring->ring_idx);

	/* Clear ring flush state */
	timeout = 1000; /* timeout of 1s */
	writel_relaxed(0x0, ring->regs + RING_CONTROL);
	do {
		if (!(readl_relaxed(ring->regs + RING_FLUSH_DONE) &
		      FLUSH_DONE_MASK))
			break;
		mdelay(1);
	} while (--timeout);
	if (!timeout)
		dev_err(ring->mbox->dev,
			"clear ring%d flush state timedout\n", ring->ring_idx);

	/* Abort all in-flight requests */
	for (reqid = 0; reqid < RING_MAX_REQ_COUNT; reqid++) {
		msg = ring->requests[reqid];
		if (!msg)
			continue;

		/* Release reqid for recycling */
		ring->requests[reqid] = NULL;

		/* Unmap DMA mappings */
		fs5rm_dma_unmap(ring->mbox->dev, msg);

		/* Give-back message to mailbox client */
		msg->error = -EIO;
		mbox_chan_received_data(chan, msg);
	}

	/* Clear requests bitmap */
	bitmap_zero(ring->requests_bmap, RING_MAX_REQ_COUNT);

	/* Release IRQ */
	if (ring->irq_requested) {
		irq_set_affinity_hint(ring->irq, NULL);
		free_irq(ring->irq, ring);
		ring->irq_requested = false;
	}

	/* Free-up completion descriptor ring */
	if (ring->cmpl_base) {
		dma_pool_free(ring->mbox->cmpl_pool,
			      ring->cmpl_base, ring->cmpl_dma_base);
		ring->cmpl_base = NULL;
	}

	/* Free-up BD descriptor ring */
	if (ring->bd_base) {
		dma_pool_free(ring->mbox->bd_pool,
			      ring->bd_base, ring->bd_dma_base);
		ring->bd_base = NULL;
	}
}

static const struct mbox_chan_ops fs5rm_mbox_chan_ops = {
	.send_data	= fs5rm_send_data,
	.startup	= fs5rm_startup,
	.shutdown	= fs5rm_shutdown,
	.peek_data	= fs5rm_peek_data,
};

static struct mbox_chan *fs5rm_mbox_of_xlate(struct mbox_controller *cntlr,
					     const struct of_phandle_args *pa)
{
	int ind;

	if (pa->args_count < 1)
		return ERR_PTR(-EINVAL);

	ind = pa->args[0];

	if (ind >= cntlr->num_chans)
		return ERR_PTR(-EINVAL);

	return &cntlr->chans[ind];
}

/* ====== FS5RM platform driver ===== */

static void fs5rm_mbox_msi_write(struct msi_desc *desc, struct msi_msg *msg)
{
	struct device *dev = msi_desc_to_dev(desc);
	struct fs5rm_mbox *mbox = dev_get_drvdata(dev);
	struct fs5rm_ring *ring = &mbox->rings[desc->platform.msi_index];

	/* Configure per-Ring MSI registers */
	writel_relaxed(msg->address_lo, ring->regs + RING_MSI_ADDR_LS);
	writel_relaxed(msg->address_hi, ring->regs + RING_MSI_ADDR_MS);
	writel_relaxed(msg->data, ring->regs + RING_MSI_DATA_VALUE);
}

static int fs5rm_mbox_probe(struct platform_device *pdev)
{
	int index, ret = 0;
	void __iomem *regs;
	void __iomem *regs_end;
	struct msi_desc *desc;
	struct resource *iomem;
	struct fs5rm_ring *ring;
	struct fs5rm_mbox *mbox;
	struct device *dev = &pdev->dev;

	/* Allocate driver mailbox struct */
	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;
	mbox->dev = dev;
	platform_set_drvdata(pdev, mbox);

	/* Get resource for registers */
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	/* Map registers of all rings */
	mbox->regs = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(mbox->regs)) {
		ret = PTR_ERR(mbox->regs);
		dev_err(&pdev->dev, "Failed to remap mailbox regs: %d\n", ret);
		goto fail;
	}
	regs_end = mbox->regs + resource_size(iomem);

	/* Scan and count available rings */
	mbox->num_rings = 0;
	for (regs = mbox->regs; regs < regs_end; regs += RING_REGS_SIZE) {
		if (readl_relaxed(regs + RING_VER) == RING_VER_MAGIC)
			mbox->num_rings++;
	}
	if (!mbox->num_rings) {
		ret = -ENODEV;
		goto fail;
	}

	/* Allocate driver ring structs */
	ring = devm_kcalloc(dev, mbox->num_rings, sizeof(*ring), GFP_KERNEL);
	if (!ring) {
		ret = -ENOMEM;
		goto fail;
	}
	mbox->rings = ring;

	/* Initialize members of driver ring structs */
	regs = mbox->regs;
	for (index = 0; index < mbox->num_rings; index++) {
		ring = &mbox->rings[index];
		ring->ring_idx = index;
		ring->mbox = mbox;
		while ((regs < regs_end) &&
		       (readl_relaxed(regs + RING_VER) != RING_VER_MAGIC))
			regs += RING_REGS_SIZE;
		if (regs_end <= regs) {
			ret = -ENODEV;
			goto fail;
		}
		ring->regs = regs;
		regs += RING_REGS_SIZE;
		ring->irq = UINT_MAX;
		ring->irq_requested = false;
		memset(ring->requests, 0, sizeof(ring->requests));
		spin_lock_init(&ring->lock);
		bitmap_zero(ring->requests_bmap, RING_MAX_REQ_COUNT);
	}

	/* FS5RM is capable of 40-bit physical addresses only */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (ret) {
		dev_warn(dev, "No suitable DMA available\n");
		goto fail;
	}

	/* Create DMA pool for ring BD memory */
	mbox->bd_pool = dma_pool_create("bd", dev, RING_BD_SIZE,
					BIT(RING_BD_ALIGN_ORDER), 0);
	if (!mbox->bd_pool) {
		ret = -ENOMEM;
		goto fail;
	}

	/* Create DMA pool for ring completion memory */
	mbox->cmpl_pool = dma_pool_create("cmpl", dev, RING_CMPL_SIZE,
					  BIT(RING_CMPL_ALIGN_ORDER), 0);
	if (!mbox->cmpl_pool) {
		ret = -ENOMEM;
		goto fail_destroy_bd_pool;
	}

	/* Allocate platform MSIs for each ring */
	ret = platform_msi_domain_alloc_irqs(dev, mbox->num_rings,
					     fs5rm_mbox_msi_write);
	if (ret)
		goto fail_destroy_cmpl_pool;

	/* Save alloced IRQ numbers for each ring */
	for_each_msi_entry(desc, dev) {
		ring = &mbox->rings[desc->platform.msi_index];
		ring->irq = desc->irq;
	}

	/* Initialize mailbox controller */
	mbox->controller.txdone_irq = false;
	mbox->controller.txdone_poll = false;
	mbox->controller.ops = &fs5rm_mbox_chan_ops;
	mbox->controller.dev = dev;
	mbox->controller.num_chans = mbox->num_rings;
	mbox->controller.of_xlate = fs5rm_mbox_of_xlate;
	mbox->controller.chans = devm_kcalloc(dev, mbox->num_rings,
					      sizeof(*mbox->controller.chans),
					      GFP_KERNEL);
	if (!mbox->controller.chans) {
		ret = -ENOMEM;
		goto fail_free_msis;
	}
	for (index = 0; index < mbox->num_rings; index++)
		mbox->controller.chans[index].con_priv = &mbox->rings[index];

	/* Register mailbox controller */
	ret = devm_mbox_controller_register(dev, &mbox->controller);
	if (ret)
		goto fail_free_msis;

	dev_info(dev, "registered fs5rm mailbox with %d channels\n",
		 mbox->controller.num_chans);

	return 0;

fail_free_msis:
	platform_msi_domain_free_irqs(dev);
fail_destroy_cmpl_pool:
	dma_pool_destroy(mbox->cmpl_pool);
fail_destroy_bd_pool:
	dma_pool_destroy(mbox->bd_pool);
fail:
	devm_kfree(dev, mbox);
	return ret;
}

static int fs5rm_mbox_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fs5rm_mbox *mbox = platform_get_drvdata(pdev);
	struct dma_pool *bd_pool = mbox->bd_pool;
	struct dma_pool *cmpl_pool = mbox->cmpl_pool;

	devm_kfree(dev, mbox);

	platform_msi_domain_free_irqs(dev);

	dma_pool_destroy(cmpl_pool);
	dma_pool_destroy(bd_pool);

	return 0;
}

static const struct of_device_id fs5rm_mbox_of_match[] = {
	{
		.compatible = "brcm,iproc-fs5rm-mbox",
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, fs5rm_mbox_of_match);

static struct platform_driver fs5rm_mbox_driver = {
	.driver = {
		.name = "brcm-fs5rm-mbox",
		.of_match_table = fs5rm_mbox_of_match,
	},
	.probe		= fs5rm_mbox_probe,
	.remove		= fs5rm_mbox_remove,
};
module_platform_driver(fs5rm_mbox_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom FS5RM mailbox driver");
MODULE_LICENSE("GPL v2");
