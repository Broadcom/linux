/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2018-2020 Broadcom.
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

	uint32_t rd_idx; /* read idx */
	uint32_t wr_idx; /* write idx */

	uint32_t size;	/*
			 * size, which is in number of 16byte blocks,
			 * to align with the message data structure.
			 */
	uint32_t nxt;	/*
			 * nxt offset to the next msg queue struct.
			 * This is to provide flexibity for alignment purposes.
			 */
};

/*
 * Structure to record static info from the msgq sync.  We keep local copy
 * for some of these variables for both performance + checking purpose.
 */
struct bcm_vk_sync_qinfo {
	void __iomem *q_start;
	uint32_t q_size;
	uint32_t q_mask;
	uint32_t q_low;
};

#define VK_MSGQ_MAX_NR 4 /* Maximum number of message queues */

/* vk_msg_blk is 16 bytes fixed */
#define VK_MSGQ_BLK_SIZE   (sizeof(struct vk_msg_blk))
/* shift for fast division of basic msg blk size */
#define VK_MSGQ_BLK_SZ_SHIFT 4

/* use msg_id 0 for any simplex host2vk communication */
#define VK_SIMPLEX_MSG_ID 0

/* context per session opening of sysfs */
struct bcm_vk_ctx {
	struct list_head node; /* use for linkage in Hash Table */
	uint idx;
	bool in_use;
	pid_t pid;
	uint32_t hash_idx;
	struct miscdevice *miscdev;
	atomic_t pend_cnt; /* number of items pending to be read from host */
	wait_queue_head_t rd_wq;
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

	uint32_t to_h_blks; /* response */
	struct vk_msg_blk *to_h_msg;

	/*
	 * put the to_v_msg at the end so that we could simply append to_v msg
	 * to the end of the allocated block
	 */
	uint32_t usr_msg_id;
	uint32_t to_v_blks;
	uint32_t seq_num;
	struct vk_msg_blk to_v_msg[0];
};

/* queue stats counters */
struct bcm_vk_qs_cnts {
	uint32_t cnt; /* general counter, used to limit output */
	uint32_t acc_sum;
	uint32_t max_occ; /* max during a sampling period */
	uint32_t max_abs; /* the abs max since reset */
};

/* stats structure */
struct bcm_vk_qstats {
	uint32_t q_num;
	struct bcm_vk_qs_cnts qcnts;
};

/* control channel structure for either to_v or to_h communication */
struct bcm_vk_msg_chan {
	uint32_t q_nr;
	/* Mutex to access msgq */
	struct mutex msgq_mutex;
	/* pointing to BAR locations */
	struct bcm_vk_msgq __iomem *msgq[VK_MSGQ_MAX_NR];
	/* Spinlock to access pending queue */
	spinlock_t pendq_lock;
	/* for temporary storing pending items, one for each queue */
	struct list_head pendq[VK_MSGQ_MAX_NR];
	/* static queue info from the sync */
	struct bcm_vk_sync_qinfo sync_qinfo[VK_MSGQ_MAX_NR];
#if defined(CONFIG_BCM_VK_QSTATS)
	/* qstats */
	struct bcm_vk_qstats qstats[VK_MSGQ_MAX_NR];
#endif
};

/* total number of supported ctx, 32 ctx each for 5 components */
#define VK_CMPT_CTX_MAX		(32 * 5)

/* hash table defines to store the opened FDs */
#define VK_PID_HT_SHIFT_BIT	7 /* 128 */
#define VK_PID_HT_SZ		BIT(VK_PID_HT_SHIFT_BIT)

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
#define VK_BAR0_RESET_DB_SOFT		0xffffffff
#define VK_BAR0_RESET_DB_HARD		0xfffffffd
#define VK_BAR0_RESET_RAMPDUMP		0xa0000000

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
/* Memory to hold the DMA buffer memory address allocated for boot2 download */
#define VK_BAR1_DMA_BUF_OFF_HI		0x61e0
#define VK_BAR1_DMA_BUF_OFF_LO		(VK_BAR1_DMA_BUF_OFF_HI + 4)
#define VK_BAR1_DMA_BUF_SZ		(VK_BAR1_DMA_BUF_OFF_HI + 8)
/* Scratch memory allocated on host for VK */
#define VK_BAR1_SCRATCH_OFF_HI		0x61f0
#define VK_BAR1_SCRATCH_OFF_LO		(VK_BAR1_SCRATCH_OFF_HI + 4)
#define VK_BAR1_SCRATCH_SZ_ADDR		(VK_BAR1_SCRATCH_OFF_HI + 8)
#define VK_BAR1_SCRATCH_DEF_NR_PAGES	32
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

#endif
