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
#ifndef PDC_H
#define PDC_H

#include <linux/scatterlist.h>
#include <linux/mailbox_controller.h>

#define PDC_SUCCESS  0

#define RING_ENTRY_SIZE   sizeof(struct dma64dd)

/* # entries in PDC dma ring */
#define PDC_RING_ENTRIES  128
#define PDC_RING_SIZE    (PDC_RING_ENTRIES * RING_ENTRY_SIZE)
/* Rings are 8k aligned */
#define RING_ALIGN_ORDER  13
#define RING_ALIGN (1 << RING_ALIGN_ORDER)

#define RX_BUF_ALIGN_ORDER  2
#define RX_BUF_ALIGN (1 << RX_BUF_ALIGN_ORDER)

/* descriptor bumping macros */
#define XXD(x, max_mask)              ((x) & (max_mask))
#define TXD(x, max_mask)              XXD((x), (max_mask))
#define RXD(x, max_mask)              XXD((x), (max_mask))
#define NEXTTXD(i, max_mask)          TXD((i) + 1, (max_mask))
#define PREVTXD(i, max_mask)          TXD((i) - 1, (max_mask))
#define NEXTRXD(i, max_mask)          RXD((i) + 1, (max_mask))
#define PREVRXD(i, max_mask)          RXD((i) - 1, (max_mask))
#define NTXDACTIVE(h, t, max_mask)    TXD((t) - (h), (max_mask))
#define NRXDACTIVE(h, t, max_mask)    RXD((t) - (h), (max_mask))


struct pdc_dma_map {
	void *ctx;          /* opaque context associated with frame */
};

struct hw_init_parms {
	dma_addr_t hw_pbase;         /* base physical addr of PDC regs */
	void *hw_vbase;              /* base virtual addr of PDC regs */

	u32  ring_entries;           /* number of ring entries */
};

/* dma descriptor */
struct dma64dd {
	u32 ctrl1;      /* misc control bits */
	u32 ctrl2;      /* buffer count and address extension */
	u32 addrlow;    /* memory address of the date buffer, bits 31:0 */
	u32 addrhigh;   /* memory address of the date buffer, bits 63:32 */
};

/* dma registers per channel(xmt or rcv) */
struct dma64_regs {
	u32  control;   /* enable, et al */
	u32  ptr;       /* last descriptor posted to chip */
	u32  addrlow;   /* descriptor ring base address low 32-bits */
	u32  addrhigh;  /* descriptor ring base address bits 63:32 */
	u32  status0;   /* last rx descriptor written by hw */
	u32  status1;   /* driver does not use */
};

/* cpp contortions to concatenate w/arg prescan */
#ifndef PAD
#define _PADLINE(line)  pad ## line
#define _XSTR(line)     _PADLINE(line)
#define PAD             _XSTR(__LINE__)
#endif  /* PAD */

/* dma registers. matches hw layout. */
struct dma64 {
	struct dma64_regs dmaxmt;  /* dma tx */
	u32          PAD[2];
	struct dma64_regs dmarcv;  /* dma rx */
	u32          PAD[2];
};

/* PDC registers */
struct pdc_regs {
	u32  devcontrol;             /* 0x000 */
	u32  devstatus;              /* 0x004 */
	u32  PAD;
	u32  biststatus;             /* 0x00c */
	u32  PAD[4];
	u32  intstatus;              /* 0x020 */
	u32  intmask;                /* 0x024 */
	u32  gptimer;                /* 0x028 */

	u32  PAD;
	u32  intrcvlazy_0;           /* 0x030 */
	u32  intrcvlazy_1;           /* 0x034 */
	u32  intrcvlazy_2;           /* 0x038 */
	u32  intrcvlazy_3;           /* 0x03c */

	u32  PAD[48];
	u32  removed_intrecvlazy;    /* 0x100 */
	u32  flowctlthresh;          /* 0x104 */
	u32  wrrthresh;              /* 0x108 */
	u32  gmac_idle_cnt_thresh;   /* 0x10c */

	u32  PAD[4];
	u32  ifioaccessaddr;         /* 0x120 */
	u32  ifioaccessbyte;         /* 0x124 */
	u32  ifioaccessdata;         /* 0x128 */

	u32  PAD[21];
	u32  phyaccess;              /* 0x180 */
	u32  PAD;
	u32  phycontrol;             /* 0x188 */
	u32  txqctl;                 /* 0x18c */
	u32  rxqctl;                 /* 0x190 */
	u32  gpioselect;             /* 0x194 */
	u32  gpio_output_en;         /* 0x198 */
	u32  PAD;                    /* 0x19c */
	u32  txq_rxq_mem_ctl;        /* 0x1a0 */
	u32  memory_ecc_status;      /* 0x1a4 */
	u32  serdes_ctl;             /* 0x1a8 */
	u32  serdes_status0;         /* 0x1ac */
	u32  serdes_status1;         /* 0x1b0 */
	u32  PAD[11];                /* 0x1b4-1dc */
	u32  clk_ctl_st;             /* 0x1e0 */
	u32  hw_war;                 /* 0x1e4 */
	u32  pwrctl;                 /* 0x1e8 */
	u32  PAD[5];

#define PDC_NUM_DMA_RINGS   4
	struct dma64 dmaregs[PDC_NUM_DMA_RINGS];  /* 0x0200 - 0x2fc */

	/* more registers follow, but we don't use them */
};

/* structure for allocating/freeing DMA rings */
struct pdc_ring_alloc {
	dma_addr_t  dmabase; /* DMA address of start of ring */
	void	   *vbase;   /* base kernel virtual address of ring */
	u32	    size;    /* ring allocation size in bytes */
};

/* PDC state structure */
struct pdc_state {

	/* synchronize access to this PDC state structure */
	spinlock_t pdc_lock;

	/* Index of the PDC whose state is in this structure instance */
	u8 pdc_idx;

	/* Platform device for this PDC instance */
	struct platform_device *pdev;

	/* Each PDC instance has a mailbox controller. PDC receives request
	 * messages through mailboxes, and sends response messages through the
	 * mailbox framework.
	 */
	struct mbox_controller mbc;

	unsigned int pdc_irq;

	/* Last interrupt status read from PDC device. Saved in interrupt
	 * handler so the handler can clear the interrupt in the device,
	 * and the interrupt thread called later can know which interrupt
	 * bits are active.
	 */
	unsigned long intstatus;

	/* Number of bytes of receive status prior to each rx frame */
	u32 rx_status_len;
	/* Whether a BCM header is prepended to each frame */
	bool use_bcm_hdr;
	/* Sum of length of BCM header and rx status header */
	u32 pdc_resp_hdr_len;

	/* The base virtual address of DMA hw registers */
	void __iomem *pdc_reg_vbase;

	/* Pool for allocation of DMA rings */
	struct dma_pool *ring_pool;

	/* Pool for allocation of metadata buffers for response messages */
	struct dma_pool *rx_buf_pool;

	/* The base virtual address of DMA tx/rx descriptor rings. Corresponding
	 * DMA address and size of ring allocation.
	 */
	struct pdc_ring_alloc tx_ring_alloc;
	struct pdc_ring_alloc rx_ring_alloc;

	struct pdc_regs *regs;    /* start of PDC registers */

	struct dma64_regs *txregs_64; /* dma tx engine registers */
	struct dma64_regs *rxregs_64; /* dma rx engine registers */

	/* Arrays of PDC_RING_ENTRIES descriptors */
	struct dma64dd   *txd_64;  /* tx descriptor ring */
	struct dma64dd   *rxd_64;  /* rx descriptor ring */

	/* descriptor ring sizes */
	u32      ntxd;       /* # tx descriptors */
	u32      nrxd;       /* # rx descriptors */
	u32      nrxpost;    /* # rx buffers to keep posted */
	u32      ntxpost;    /* max number of tx buffers that can be posted */

	/* tx ring management variables */
	/* Index of next tx descriptor to reclaim. That is, the descriptor
	 * index of the oldest tx buffer for which the host has yet to process
	 * the corresponding response.
	 */
	u32  txin;

	/* Index of the first receive descriptor for the sequence of
	 * message fragments currently under construction. Used to build up
	 * the rxin_numd count for a message. Updated to rxout when the host
	 * starts a new sequence of rx buffers for a new message.
	 */
	u32  tx_msg_start;

	/* Index of next tx descriptor to post. */
	u32  txout;

	/* Number of tx descriptors associated with the message that starts
	 * at this tx descriptor index.
	 */
	u32      txin_numd[PDC_RING_ENTRIES];

	/* rx ring management variables */
	/* Index of next rx descriptor to reclaim. This is the index of
	 * the next descriptor whose data has yet to be processed by the host.
	 */
	u32  rxin;

	/* Index of the first receive descriptor for the sequence of
	 * message fragments currently under construction. Used to build up
	 * the rxin_numd count for a message. Updated to rxout when the host
	 * starts a new sequence of rx buffers for a new message.
	 */
	u32  rx_msg_start;

	/* Saved value of current hardware rx descriptor index.
	 * The last rx buffer written by the hw is the index previous to
	 * this one.
	 */
	u32  last_rx_curr;

	/* Index of next rx descriptor to post. */
	u32  rxout;

	/* opaque context associated with frame that starts at each
	 * rx ring index.
	 */
	void *rxp_ctx[PDC_RING_ENTRIES];

	/* Scatterlists used to form request and reply frames beginning at a
	 * given ring index. Retained in order to unmap each sg after reply
	 * is processed
	 */
	struct scatterlist *src_sg[PDC_RING_ENTRIES];
	struct scatterlist *dst_sg[PDC_RING_ENTRIES];

	/* Number of rx descriptors associated with the message that starts
	 * at this descriptor index. Not set for every index. For example,
	 * if descriptor index i points to a scatterlist with 4 entries, then
	 * the next three descriptor indexes don't have a value set.
	 */
	u32  rxin_numd[PDC_RING_ENTRIES];

	void *resp_hdr[PDC_RING_ENTRIES];

	struct dentry *debugfs_stats;  /* debug FS stats file for this PDC */

	/* counters */
	u32  pdc_requests;    /* number of request messages submitted */
	u32  pdc_replies;     /* number of reply messages received */
	u32  txnobuf;         /* count of tx ring full */
	u32  rxnobuf;         /* count of rx ring full */
	u32  rx_oflow;        /* count of rx overflows */
	u32  no_rx_init;      /* missing rx init calls */
};
#endif
