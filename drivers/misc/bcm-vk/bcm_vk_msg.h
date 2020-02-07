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

/*
 * some useful message queue macros
 */

/* vk_msg_blk is 16 bytes fixed */
#define VK_MSGQ_BLK_SIZE   (sizeof(struct vk_msg_blk))
/* shift for fast division of basic msg blk size */
#define VK_MSGQ_BLK_SZ_SHIFT 4

#define VK_MSGQ_EMPTY(_msgq) ((_msgq)->rd_idx == (_msgq)->wr_idx)

#define VK_MSGQ_SIZE_MASK(_qinfo) ((_qinfo)->q_mask)

#define VK_MSGQ_INC(_qinfo, _idx, _inc) \
	(((_idx) + (_inc)) & VK_MSGQ_SIZE_MASK(_qinfo))

#define VK_MSGQ_BLK_ADDR(_qinfo, _idx) \
	(volatile struct vk_msg_blk *)((_qinfo)->q_start + \
				       (VK_MSGQ_BLK_SIZE * (_idx)))

#define VK_MSGQ_OCCUPIED(_msgq, _qinfo) \
	(((_msgq)->wr_idx - (_msgq)->rd_idx) & VK_MSGQ_SIZE_MASK(_qinfo))

#define VK_MSGQ_AVAIL_SPACE(_msgq, _qinfo) \
	((_qinfo)->q_size - VK_MSGQ_OCCUPIED(_msgq, _qinfo) - 1)

/* context per session opening of sysfs */
struct bcm_vk_ctx {
	struct list_head node; /* use for linkage in Hash Table */
	uint idx;
	bool in_use;
	struct task_struct *ppid;
	uint32_t hash_idx;
	struct miscdevice *miscdev;
	int pend_cnt; /* number of items pending to be read from host */
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

/* control channel structure for either h2vk or vk2h communication */
struct bcm_vk_msg_chan {
	uint32_t q_nr;
	struct mutex msgq_mutex;
	/* pointing to BAR locations */
	struct bcm_vk_msgq *msgq[VK_MSGQ_MAX_NR];

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
/* Memory to hold the DMA buffer memory address allocated for boot2 download */
#define VK_BAR1_DMA_BUF_OFF_HI		0x61e0
#define VK_BAR1_DMA_BUF_OFF_LO		(VK_BAR1_DMA_BUF_OFF_HI + 4)
#define VK_BAR1_DMA_BUF_SZ		(VK_BAR1_DMA_BUF_OFF_HI + 8)
/* Scratch memory allocated on host for VK */
#define VK_BAR1_SCRATCH_OFF_LO		0x61f0
#define VK_BAR1_SCRATCH_OFF_HI		(VK_BAR1_SCRATCH_OFF_LO + 4)
#define VK_BAR1_SCRATCH_SZ_ADDR		(VK_BAR1_SCRATCH_OFF_LO + 8)
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
