/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2018-2019 Broadcom.
 */

#ifndef BCM_VK_MSG_H
#define BCM_VK_MSG_H

#include <uapi/linux/misc/bcm_vk.h>
#include "bcm_vk_sg.h"

/* Single message queue control structure */
struct bcm_vk_msgq {
	uint16_t type;	/* queue type */
	uint16_t num;	/* queue number */
	uint32_t start;	/* offset in BAR1 where the queue memory starts */
	volatile uint32_t rd_idx; /* read idx */
	volatile uint32_t wr_idx; /* write idx */
	uint32_t size;	/*
			 * size, which is in number of 16byte blocks,
			 * to align with the message data structure.
			 */
	uint32_t nxt;	/*
			 * nxt offset to the next msg queue struct.
			 * This is to provide flexibity for alignment purposes.
			 */
};

#define VK_MSGQ_MAX_NR 4 /* Maximum number of message queues */

/*
 * some useful message queue macros
 */

/* vk_msg_blk is 16 bytes fixed */
#define VK_MSGQ_BLK_SIZE   (sizeof(struct vk_msg_blk))
/* shift for fast division of basic msg blk size */
#define VK_MSGQ_BLK_SZ_SHIFT 4

#define VK_MSGQ_EMPTY(q) (q->rd_idx == q->wr_idx)

#define VK_MSGQ_SIZE_MASK(q) (q->size - 1)

#define VK_MSGQ_INC(q, idx, inc) (((idx) + (inc)) & VK_MSGQ_SIZE_MASK(q))

#define VK_MSGQ_FULL(q) (VK_MSGQ_INC(q, q->wr_idx, 1) == q->rd_idx)

#define VK_MSGQ_OFFSET(q, idx) (q->start + VK_MSGQ_BLK_SIZE * (idx))

#define VK_MSGQ_BLK_ADDR(base, q, idx) \
	 (volatile struct vk_msg_blk *)(base + VK_MSGQ_OFFSET(q, idx))

#define VK_MSGQ_OCCUPIED(q) ((q->wr_idx - q->rd_idx) & VK_MSGQ_SIZE_MASK(q))

#define VK_MSGQ_AVAIL_SPACE(q) (q->size - VK_MSGQ_OCCUPIED(q) - 1)

/* context per session opening of sysfs */
struct bcm_vk_ctx {
	struct list_head node; /* use for linkage in Hash Table */
	uint idx;
	bool in_use;
	struct task_struct *ppid;
	uint32_t hash_idx;
	struct miscdevice *miscdev;
};

/* pid hash table entry */
struct bcm_vk_ht_entry {
	struct list_head head;
};

#define VK_DMA_MAX_ADDRS 4 /* Max 4 DMA Addresses */
/* structure for house keeping a single work entry */
struct bcm_vk_wkent {

	struct list_head node; /* for linking purpose */
	struct bcm_vk_ctx *ctx;

	/* Store up to 4 dma pointers */
	struct bcm_vk_dma dma[VK_DMA_MAX_ADDRS];

	uint32_t vk2h_blks; /* response */
	struct vk_msg_blk *vk2h_msg;

	/*
	 * put the h2vk_msg at the end so that we could simply append h2vk msg
	 * to the end of the allocated block
	 */
	uint32_t usr_msg_id;
	uint32_t h2vk_blks;
	uint32_t seq_num;
	struct vk_msg_blk h2vk_msg[0];
};

/* control channel structure for either h2vk or vk2h communication */
struct bcm_vk_msg_chan {
	uint32_t q_nr;
	struct mutex msgq_mutex;
	/* pointing to BAR locations */
	struct bcm_vk_msgq *msgq[VK_MSGQ_MAX_NR];

	spinlock_t pendq_lock;
	/* for temporary storing pending items, one for each queue */
	struct list_head pendq[VK_MSGQ_MAX_NR];

};

/* TO_DO: some of the following defines may need to be adjusted */
#define VK_CMPT_CTX_MAX		(32 * 5)

/* hash table defines to store the opened FDs */
#define VK_PID_HT_SHIFT_BIT	7 /* 128 */
#define VK_PID_HT_SZ		(1 << VK_PID_HT_SHIFT_BIT)

/* The following are offsets of DDR info provided by the vk card */
#define VK_BAR0_SEG_SIZE	(4 * SZ_1K) /* segment size for BAR0 */

/* shutdown types supported */
#define VK_SHUTDOWN_PID		1
#define VK_SHUTDOWN_GRACEFUL	2

/*
 * first door bell reg, ie for queue = 0.  Only need the first one, as
 * we will use the queue number to derive the others
 */
#define VK_BAR0_REGSEG_DB_BASE		0x484
#define VK_BAR0_REGSEG_DB_REG_GAP	8 /*
					   * DB register gap,
					   * DB1 at 0x48c and DB2 at 0x494
					   */

/* reset register and specific values */
#define VK_BAR0_RESET_DB_NUM		3
#define VK_BAR0_RESET_DB_SOFT		0xFFFFFFFF
#define VK_BAR0_RESET_DB_HARD		0xFFFFFFFD

/* BAR1 message q definition */

/* indicate if msgq ctrl in BAR1 is populated */
#define VK_BAR1_MSGQ_DEF_RDY		0x60c0
/* ready marker value for the above location, normal boot2 */
#define VK_BAR1_MSGQ_RDY_MARKER		0xbeefcafe
/* ready marker value for the above location, normal boot2 */
#define VK_BAR1_DIAG_RDY_MARKER		0xdeadcafe
/* number of msgqs in BAR1 */
#define VK_BAR1_MSGQ_NR			0x60c4
/* BAR1 queue control structure offset */
#define VK_BAR1_MSGQ_CTRL_OFF		0x60c8
/* BAR1 ucode and boot1 version tag */
#define VK_BAR1_UCODE_VER_TAG		0x6170
#define VK_BAR1_BOOT1_VER_TAG		0x61b0
#define VK_BAR1_VER_TAG_SIZE		64
/* BAR1 SOTP AUTH and REVID info */
#define VK_BAR1_DAUTH_BASE_ADDR		0x6200
#define VK_BAR1_DAUTH_STORE_SIZE	0x48
#define VK_BAR1_DAUTH_VALID_SIZE	0x8
#define VK_BAR1_DAUTH_MAX		4
#define VK_BAR1_DAUTH_STORE_ADDR(x) \
		(VK_BAR1_DAUTH_BASE_ADDR + \
		 (x) * (VK_BAR1_DAUTH_STORE_SIZE + VK_BAR1_DAUTH_VALID_SIZE))
#define VK_BAR1_DAUTH_VALID_ADDR(x) \
		(VK_BAR1_DAUTH_STORE_ADDR(x) + VK_BAR1_DAUTH_STORE_SIZE)

#define VK_BAR1_SOTP_REVID_BASE_ADDR	0x6340
#define VK_BAR1_SOTP_REVID_SIZE		0x10
#define VK_BAR1_SOTP_REVID_MAX		2
#define VK_BAR1_SOTP_REVID_ADDR(x) \
		(VK_BAR1_SOTP_REVID_BASE_ADDR + (x) * VK_BAR1_SOTP_REVID_SIZE)

/* Scratch memory allocated on host for VK */
#define VK_BAR1_SCRATCH_OFF_LO		0x61f0
#define VK_BAR1_SCRATCH_OFF_HI		(VK_BAR1_SCRATCH_OFF_LO + 4)
#define VK_BAR1_SCRATCH_SZ_ADDR		(VK_BAR1_SCRATCH_OFF_LO + 8)
#define VK_BAR1_SCRATCH_DEF_NR_PAGES	32

#endif
