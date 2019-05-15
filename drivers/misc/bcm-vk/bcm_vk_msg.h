/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright(c) 2018 Broadcom
 */

#ifndef __BCM_VK_MSG_H
#define __BCM_VK_MSG_H

#include <uapi/linux/misc/bcm_vk.h>
#include "bcm_vk_sg.h"

/* This is a single message queue control structure */
struct bcm_vk_msgq {
	uint16_t q_type;	  /* queue type */
	uint16_t q_num;		  /* q_num */
	uint32_t q_start_loc;	  /* offset in BAR1 where the queue
				   * memory starts
				   */
	volatile uint32_t rd_idx; /* read idx */
	volatile uint32_t wr_idx; /* write idx */
	uint32_t size;		  /* size, which is in number of 16byte
				   * blocks, to align with the message
				   * data structure.
				   */
	uint32_t next_off;	  /* offset to the next msg queue struct.
				   * This is to provide flexibity for
				   * alignment purpose
				   */
};

/*
 * some useful message queue macros
 */
#define VK_MSGQ_BLK_SIZE   (sizeof(struct vk_msg_blk))

#define VK_MSGQ_EMPTY(_p)						\
	 (_p->rd_idx == _p->wr_idx)

#define VK_MSGQ_SIZE_MASK(_p)						\
	 (_p->size - 1)

#define VK_MSGQ_INC(_p, idx, inc)					\
	 (((idx) + (inc)) & VK_MSGQ_SIZE_MASK(_p))

#define VK_MSGQ_FULL(_p)						\
	 (VK_MSGQ_INC(_p, _p->wr_idx, 1) == _p->rd_idx)

#define VK_MSGQ_OFFSET(_p, idx)						\
	 (_p->q_start_loc + VK_MSGQ_BLK_SIZE * (idx))

#define VK_MSGQ_BLK_ADDR(base, _p, idx)					\
	 (volatile struct vk_msg_blk *)(base + VK_MSGQ_OFFSET(_p, idx))

#define VK_MSGQ_OCCUPIED(_p)						\
	 ((_p->wr_idx - _p->rd_idx) & VK_MSGQ_SIZE_MASK(_p))

#define VK_MSGQ_AVAIL_SPACE(_p)						\
	 (_p->size - VK_MSGQ_OCCUPIED(_p) - 1)

#define VK_MSGQ_MAX_NR (4)  /* we should not need more than 4 for now */

#define VK_MSG_ID_OVERFLOW (0xFFFF)  /* msg_id overflow value */

/* context per session opening of sysfs */
struct bcm_vk_ctx {
	struct list_head list_node; /* use for linkage in HT */
	uint idx;
	bool in_use;
	struct task_struct *p_pid;
	uint32_t hash_idx;
	struct miscdevice *p_miscdev;
};

/* pid hash table entry */
struct bcm_vk_ht_entry {
	struct list_head fd_head;
};

#define VK_DMA_MAX_ADDRS 4 /* Max 4 DMA Addresses */
/* structure for house keeping a single work entry */
struct bcm_vk_wkent {

	struct list_head list_node; /* for linking purpose */
	struct bcm_vk_ctx *p_ctx;

	/*
	 * Store up to 4 dma pointers
	 */
	struct bcm_vk_dma dma[VK_DMA_MAX_ADDRS];

	uint32_t vk2h_blks;	    /* response */
	struct vk_msg_blk *p_vk2h_msg;

	/*
	 * put the h2vk_msg at the end so that we could simply append h2vk msg
	 * to the end of the allocated block
	 */
	uint32_t usr_msg_id;
	uint32_t h2vk_blks;
	struct vk_msg_blk p_h2vk_msg[0];
};

/* control channel structure for either h2vk or vk2h communication */
struct bcm_vk_msg_chan {
	uint32_t q_nr;
	struct mutex msgq_mutex;
	/* pointing to BAR locations */
	struct bcm_vk_msgq *msgq[VK_MSGQ_MAX_NR];

	spinlock_t pendq_lock;
	/* for temporary storing pending items, one for each queue */
	struct list_head pendq_head[VK_MSGQ_MAX_NR];

};

/* TO_DO: some of the following defines may need to be adjusted */
#define VK_CMPT_CTX_MAX		    (32 * 5)

/* hash table defines to store the opened FDs */
#define VK_PID_HT_SHIFT_BIT         7 /* 128 */
#define VK_PID_HT_SZ                (1 << VK_PID_HT_SHIFT_BIT)

/* The following are offsets of DDR info provided by the vk card */
#define VK_BAR0_SEG_SIZE	    (4 * SZ_1K)	/* segment size for BAR0 */

/* shutdown types supported */
#define VK_SHUTDOWN_PID		    1
#define VK_SHUTDOWN_GRACEFUL	    2

/*
 * first door bell reg, ie for queue = 0.  Only need the first one, as
 * we will use the queue number to derive the others
 */
#define VK_BAR0_REGSEG_DB_BASE		0x484
#define VK_BAR0_REGSEG_DB_REG_GAP	8 /* DB register gap,
					   * DB1 at 0x48c and DB2 at 0x494
					   */

/* reset register and specici values */
#define VK_BAR0_RESET_DB_NUM		3
#define VK_BAR0_RESET_DB_VAL		0xFFFFFFFF

/* BAR1 message q definition */

/* indicate if msgq ctrl in BAR1 is populated */
#define VK_BAR1_MSGQ_DEF_RDY		0x60c0
/* ready marker value for the above location */
#define VK_BAR1_MSGQ_RDY_MARKER		0xbeefcafe
/* number of msgqs in BAR1 */
#define VK_BAR1_MSGQ_NR			0x60c4
/* BAR1 queue control structure offset */
#define VK_BAR1_MSGQ_CTRL_OFF		0x60c8
/* Scratch memory allocated on host for VK */
#define VK_BAR1_SCRATCH_OFF             0x61f0

#endif
