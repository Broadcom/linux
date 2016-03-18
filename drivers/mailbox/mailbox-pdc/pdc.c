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

/*
 * Broadcom PDC Mailbox Driver
 * The PDC driver manages a DMA interface to one or more hardware offload
 * engines. For example, the PDC driver works with both SPU-M and SPU2
 * cryptographic offload hardware. In some chips the PDC is referred to as MDE.
 *
 * The PDC driver registers with the Linux mailbox framework a mailbox
 * controller, once for each PDC instance. Ring 0 for each PDC is registered as
 * a mailbox channel. The PDC driver uses interrupts to determine when DMA
 * transfers to and from an offload engine are complete. The PDC driver uses
 * threaded IRQs so that response messages are handled outside of interrupt
 * context.
 *
 * The PDC driver allows multiple messages to be pending in the descriptor
 * rings. The tx_msg_start descriptor index indicates where the last message
 * starts. The txin_numd value at this index indicates how many descriptor
 * indexes make up the message. Similar state is kept on the receive side. When
 * an rx interrupt indicates a response is ready, the PDC driver processes numd
 * descriptors from the tx and rx ring, thus processing one response at a time.
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox/brcm-message.h>
#include <linux/scatterlist.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>

#include "pdc.h"
#include "pdc_debug.h"

/* Length of BCM header at start of SPU msg, in bytes */
#define BCM_HDR_LEN  8

/*
 * Interrupt mask and status definitions. Enable interrupts for tx and rx on
 * ring 0
 */
#define PDC_XMTINTEN_0 (1 << 24)
#define PDC_RCVINTEN_0 (1 << 16)
#define PDC_INTMASK  (PDC_XMTINTEN_0 | PDC_RCVINTEN_0)
#define PDC_LAZY_FRAMECOUNT  1
#define PDC_LAZY_TIMEOUT     10000
#define PDC_LAZY_INT  (PDC_LAZY_TIMEOUT | (PDC_LAZY_FRAMECOUNT << 24))
#define PDC_INTMASK_OFFSET   0x24
#define PDC_INTSTATUS_OFFSET 0x20
#define PDC_RCVLAZY0_OFFSET  0x30

/*
 * For SPU2, configure MDE_CKSUM_CONTROL to write 17 bytes of metadata
 * before frame
 */
#define PDC_SPU2_RESP_HDR_LEN  17
#define PDC_CKSUM_CTRL (1 << 27)
#define PDC_CKSUM_CTRL_OFFSET 0x400

#define PDC_SPUM_RESP_HDR_LEN  32

/*
 * Sets the following bits for write to transmit control reg:
 *  0    - XmtEn - enable activity on the tx channel
 * 11    - PtyChkDisable - parity check is disabled
 * 20:18 - BurstLen = 3 -> 2^7 = 128 byte data reads from memory
 */
#define PDC_TX_CTL              0x000C0801

/*
 * Sets the following bits for write to receive control reg:
 * 0     - RcvEn - enable activity on the rx channel
 * 7:1   - RcvOffset - size in bytes of status region at start of rx frame buf
 * 9     - SepRxHdrDescEn - place start of new frames only in descriptors
 *                          that have StartOfFrame set
 * 10    - OflowContinue - on rx FIFO overflow, clear rx fifo, discard all
 *                         remaining bytes in current frame, report error
 *                         in rx frame status for current frame
 * 11    - PtyChkDisable - parity check is disabled
 * 20:18 - BurstLen = 3 -> 2^7 = 128 byte data reads from memory
 */
#define PDC_RX_CTL              0x000C0E01

#define CRYPTO_D64_RS0_CD_MASK   ((PDC_RING_ENTRIES * RING_ENTRY_SIZE) - 1)

/* descriptor flags */
#define D64_CTRL1_EOT   ((u32) 1 << 28)	/* end of descriptor table */
#define D64_CTRL1_IOC   ((u32) 1 << 29)	/* interrupt on complete */
#define D64_CTRL1_EOF   ((u32) 1 << 30)	/* end of frame */
#define D64_CTRL1_SOF   ((u32) 1 << 31)	/* start of frame */

#define RX_STATUS_OVERFLOW       0x00800000
#define RX_STATUS_LEN            0x0000FFFF

#define PDC_TXREGS_OFFSET  0x200
#define PDC_RXREGS_OFFSET  0x220

/* PDC driver reserves ringset 0 on each SPU for its own use */
#define PDC_RINGSET  0

/* Maximum size buffer the DMA engine can handle */
#define PDC_DMA_BUF_MAX 16384

/* Global variables */

struct pdc_globals {
	/* Actual number of SPUs in hardware, as reported by device tree */
	u32 num_spu;
};

static struct pdc_globals pdcg;

/**
 * pdc_build_rxd() - Build DMA descriptor to receive SPU result.
 * @pdcs:      PDC state for SPU that will generate result
 * @dma_addr:  DMA address of buffer that descriptor is being built for
 * @buf_len:   Length of the receive buffer, in bytes
 * @flags:     Flags to be stored in descriptor
 */
static inline void
pdc_build_rxd(struct pdc_state *pdcs, dma_addr_t dma_addr,
	      u32 buf_len, u32 flags)
{
	struct device *dev = &pdcs->pdev->dev;

	dev_dbg(dev,
		"Writing rx descriptor for PDC %u at index %u with length %u. flags %#x\n",
		pdcs->pdc_idx, pdcs->rxout, buf_len, flags);

	iowrite32(lower_32_bits(dma_addr),
		  (void *)&pdcs->rxd_64[pdcs->rxout].addrlow);
	iowrite32(upper_32_bits(dma_addr),
		  (void *)&pdcs->rxd_64[pdcs->rxout].addrhigh);
	iowrite32(flags, (void *)&pdcs->rxd_64[pdcs->rxout].ctrl1);
	iowrite32(buf_len, (void *)&pdcs->rxd_64[pdcs->rxout].ctrl2);
	/* bump ring index and return */
	pdcs->rxout = NEXTRXD(pdcs->rxout, pdcs->nrxpost);
}

/**
 * pdc_build_txd() - Build a DMA descriptor to transmit a SPU request to
 * hardware.
 * @pdcs:        PDC state for the SPU that will process this request
 * @dma_addr:    DMA address of packet to be transmitted
 * @buf_len:     Length of tx buffer, in bytes
 * @flags:       Flags to be stored in descriptor
 */
static inline void
pdc_build_txd(struct pdc_state *pdcs, dma_addr_t dma_addr, u32 buf_len,
	      u32 flags)
{
	struct device *dev = &pdcs->pdev->dev;

	dev_dbg(dev,
		"Writing tx descriptor for PDC %u at index %u with length %u, flags %#x\n",
		pdcs->pdc_idx, pdcs->txout, buf_len, flags);

	iowrite32(lower_32_bits(dma_addr),
		  (void *)&pdcs->txd_64[pdcs->txout].addrlow);
	iowrite32(upper_32_bits(dma_addr),
		  (void *)&pdcs->txd_64[pdcs->txout].addrhigh);
	iowrite32(flags, (void *)&pdcs->txd_64[pdcs->txout].ctrl1);
	iowrite32(buf_len, (void *)&pdcs->txd_64[pdcs->txout].ctrl2);

	/* bump ring index and return */
	pdcs->txout = NEXTTXD(pdcs->txout, pdcs->ntxpost);
}

/**
 * pdc_receive() - Receive a response message from a given SPU.
 * @pdcs:    PDC state for the SPU to receive from
 * @mssg:    mailbox message to be returned to client
 *
 * When the return code indicates success, the response message is available in
 * the receive buffers provided prior to submission of the request.
 *
 * Input:
 *   pdcs - PDC state structure for the SPU to be polled
 *   mssg - mailbox message to be returned to client. This function sets the
 *	    context pointer on the message to help the client associate the
 *	    response with a request.
 *
 * Return:  PDC_SUCCESS if one or more receive descriptors was processed
 *          -EAGAIN indicates that no response message is available
 *          -EIO an error occurred
 */
static int
pdc_receive(struct pdc_state *pdcs, struct brcm_message *mssg)
{
	struct device *dev = &pdcs->pdev->dev;
	u32 len, rx_status;
	u32 num_frags;
	int i;
	u8 *resp_hdr;    /* virtual addr of start of resp message DMA header */
	u32 frags_rdy;   /* number of fragments ready to read */
	u32 rx_idx;      /* ring index of start of receive frame */
	dma_addr_t resp_hdr_daddr;

	spin_lock(&pdcs->pdc_lock);

	/*
	 * return if a complete response message is not yet ready.
	 * rxin_numd[rxin] is the number of fragments in the next msg
	 * to read.
	 */
	frags_rdy = NRXDACTIVE(pdcs->rxin, pdcs->last_rx_curr, pdcs->nrxpost);
	if ((frags_rdy == 0) || (frags_rdy < pdcs->rxin_numd[pdcs->rxin])) {
		/* See if the hw has written more fragments than we know */
		pdcs->last_rx_curr =
		    (ioread32((void *)&pdcs->rxregs_64->status0) &
		     CRYPTO_D64_RS0_CD_MASK) / RING_ENTRY_SIZE;
		frags_rdy = NRXDACTIVE(pdcs->rxin, pdcs->last_rx_curr,
				       pdcs->nrxpost);
		if ((frags_rdy == 0) ||
		    (frags_rdy < pdcs->rxin_numd[pdcs->rxin])) {
			/* No response ready */
			spin_unlock(&pdcs->pdc_lock);
			return -EAGAIN;
		}
		rmb();		/* barrier after register read */
	}

	num_frags = pdcs->txin_numd[pdcs->txin];
	dma_unmap_sg(dev, pdcs->src_sg[pdcs->txin],
		     sg_nents(pdcs->src_sg[pdcs->txin]), DMA_TO_DEVICE);

	for (i = 0; i < num_frags; i++)
		pdcs->txin = NEXTTXD(pdcs->txin, pdcs->ntxpost);

	dev_dbg(dev, "PDC %u reclaimed %d tx descriptors",
		pdcs->pdc_idx, num_frags);

	rx_idx = pdcs->rxin;
	num_frags = pdcs->rxin_numd[rx_idx];
	/* Return opaque context with result */
	mssg->ctx = pdcs->rxp_ctx[rx_idx];
	pdcs->rxp_ctx[rx_idx] = NULL;
	resp_hdr = pdcs->resp_hdr[rx_idx];
	resp_hdr_daddr = pdcs->rxd_64[rx_idx].addrhigh;
	resp_hdr_daddr = resp_hdr_daddr << 32;
	resp_hdr_daddr += pdcs->rxd_64[rx_idx].addrlow;
	dma_unmap_sg(dev, pdcs->dst_sg[rx_idx],
		     sg_nents(pdcs->dst_sg[rx_idx]), DMA_FROM_DEVICE);

	for (i = 0; i < num_frags; i++)
		pdcs->rxin = NEXTRXD(pdcs->rxin, pdcs->nrxpost);

	spin_unlock(&pdcs->pdc_lock);

	dev_dbg(dev, "PDC %u reclaimed %d rx descriptors",
		pdcs->pdc_idx, num_frags);

	dev_dbg(dev,
		"PDC %u txin %u, txout %u, rxin %u, rxout %u, last_rx_curr %u\n",
		pdcs->pdc_idx, pdcs->txin, pdcs->txout, pdcs->rxin,
		pdcs->rxout, pdcs->last_rx_curr);

	if (pdcs->pdc_resp_hdr_len == PDC_SPUM_RESP_HDR_LEN) {
		/*
		 * For SPU-M, get length of response msg and rx overflow status.
		 */
		rx_status = *((u32 *) resp_hdr);
		len = rx_status & RX_STATUS_LEN;
		dev_dbg(dev,
			"SPU response length %u bytes", len);
		if (unlikely(((rx_status & RX_STATUS_OVERFLOW) || (!len)))) {
			if (rx_status & RX_STATUS_OVERFLOW) {
				dev_err_ratelimited(dev,
						    "crypto receive overflow");
				pdcs->rx_oflow++;
			} else {
				dev_info_ratelimited(dev, "crypto rx len = 0");
			}
			return -EIO;
		}
	}

	dma_pool_free(pdcs->rx_buf_pool, resp_hdr, resp_hdr_daddr);

	pdcs->pdc_replies++;
	/* if we read one or more rx descriptors, claim success */
	if (num_frags > 0)
		return PDC_SUCCESS;
	else
		return -EIO;
}

/**
 * pdc_tx_list_sg_add() - Add the buffers in a scatterlist to the transmit
 * descriptors for a given SPU. The scatterlist buffers contain the data for a
 * SPU request message.
 * @spu_idx:   The index of the SPU to submit the request to, [0, max_spu)
 * @sg:        Scatterlist whose buffers contain part of the SPU request
 *
 * If a scatterlist buffer is larger than PDC_DMA_BUF_MAX, multiple descriptors
 * are written for that buffer, each <= PDC_DMA_BUF_MAX byte in length.
 *
 * Return: PDC_SUCCESS if successful
 *         < 0 otherwise
 */
static int pdc_tx_list_sg_add(struct pdc_state *pdcs, struct scatterlist *sg)
{
	u32 flags = 0;
	u32 eot;
	u32 tx_avail;

	/* Num descriptors needed. Conservatively assume we need a descriptor
	 * for every entry in sg.
	 */
	u32 num_desc;
	u32 desc_w = 0;	/* Number of tx descriptors written */
	u32 bufcnt;	/* Number of bytes of buffer pointed to by descriptor */
	dma_addr_t databufptr;	/* DMA address to put in descriptor */

	num_desc = (u32) sg_nents(sg);

	/* check whether enough tx descriptors are available */
	tx_avail = pdcs->ntxpost - NTXDACTIVE(pdcs->txin, pdcs->txout,
					      pdcs->ntxpost);
	if (unlikely(num_desc > tx_avail)) {
		pdcs->txnobuf++;
		return -ENOSPC;
	}

	/* build tx descriptors */
	if (pdcs->tx_msg_start == pdcs->txout) {
		/* Start of frame */
		pdcs->txin_numd[pdcs->tx_msg_start] = 0;
		pdcs->src_sg[pdcs->txout] = sg;
		flags = D64_CTRL1_SOF;
	}

	while (sg) {
		if (unlikely(pdcs->txout == (pdcs->ntxd - 1)))
			eot = D64_CTRL1_EOT;
		else
			eot = 0;

		/* PDC DMA cannot accept a buffer larger than 16384 bytes. */
		bufcnt = sg_dma_len(sg);
		databufptr = sg_dma_address(sg);
		while (bufcnt > PDC_DMA_BUF_MAX) {
			pdc_build_txd(pdcs, databufptr, PDC_DMA_BUF_MAX,
				      flags | eot);
			desc_w++;
			bufcnt -= PDC_DMA_BUF_MAX;
			databufptr += PDC_DMA_BUF_MAX;
			if (unlikely(pdcs->txout == (pdcs->ntxd - 1)))
				eot = D64_CTRL1_EOT;
			else
				eot = 0;
		}
		sg = sg_next(sg);
		if (sg == NULL)
			/* Writing last descriptor for frame */
			flags |= (D64_CTRL1_EOF | D64_CTRL1_IOC);
		pdc_build_txd(pdcs, databufptr, bufcnt, flags | eot);
		desc_w++;
		/* Clear start of frame after first descriptor */
		flags &= ~D64_CTRL1_SOF;
	}
	pdcs->txin_numd[pdcs->tx_msg_start] += desc_w;

	return PDC_SUCCESS;
}

/**
 * pdc_tx_list_final() - Initiate DMA transfer of last frame written to tx
 * ring.
 * @pdcs:  PDC state for SPU to process the request
 *
 * Sets the index of the last descriptor written in both the rx and tx ring.
 *
 * Return: PDC_SUCCESS
 */
static int pdc_tx_list_final(struct pdc_state *pdcs)
{
	/*
	 * write barrier to ensure all register writes are complete
	 * before chip starts to process new request
	 */
	wmb();
	iowrite32(pdcs->rxout << 4, (void *)&pdcs->rxregs_64->ptr);
	iowrite32(pdcs->txout << 4, (void *)&pdcs->txregs_64->ptr);
	pdcs->pdc_requests++;

	return PDC_SUCCESS;
}

/**
 * pdc_rx_list_init() - Start a new receive descriptor list for a given PDC.
 * @pdcs:   PDC state for SPU handling request
 * @dst_sg: scatterlist providing rx buffers for response to be returned to
 *	    mailbox client
 * @ctx:    Opaque context for this request
 *
 * Posts a single receive descriptor to hold the metadata that precedes a
 * response. For example, with SPU-M, the metadata is a 32-byte DMA header and
 * an 8-byte BCM header. Moves the msg_start descriptor indexes for both tx and
 * rx to indicate the start of a new message.
 *
 * Return:  PDC_SUCCESS if successful
 *          < 0 if an error (e.g., rx ring is full)
 */
static int pdc_rx_list_init(struct pdc_state *pdcs, struct scatterlist *dst_sg,
			    void *ctx)
{
	u32 flags = 0;
	u32 rx_avail;
	u32 rx_pkt_cnt = 1;	/* Adding a single rx buffer */
	dma_addr_t daddr;
	void *vaddr;

	rx_avail = pdcs->nrxpost - NRXDACTIVE(pdcs->rxin, pdcs->rxout,
					      pdcs->nrxpost);
	if (unlikely(rx_pkt_cnt > rx_avail)) {
		pdcs->rxnobuf++;
		return -ENOSPC;
	}

	/* allocate a buffer for the dma rx status */
	vaddr = dma_pool_zalloc(pdcs->rx_buf_pool, GFP_ATOMIC, &daddr);
	if (!vaddr)
		return -ENOMEM;

	/*
	 * Update msg_start indexes for both tx and rx to indicate the start
	 * of a new sequence of descriptor indexes that contain the fragments
	 * of the same message.
	 */
	pdcs->rx_msg_start = pdcs->rxout;
	pdcs->tx_msg_start = pdcs->txout;

	/* This is always the first descriptor in the receive sequence */
	flags = D64_CTRL1_SOF;
	pdcs->rxin_numd[pdcs->rx_msg_start] = 1;

	if (unlikely(pdcs->rxout == (pdcs->nrxd - 1)))
		flags |= D64_CTRL1_EOT;

	pdcs->rxp_ctx[pdcs->rxout] = ctx;
	pdcs->dst_sg[pdcs->rxout] = dst_sg;
	pdcs->resp_hdr[pdcs->rxout] = vaddr;
	pdc_build_rxd(pdcs, daddr, pdcs->pdc_resp_hdr_len, flags);
	return PDC_SUCCESS;
}

/**
 * pdc_rx_list_sg_add() - Add the buffers in a scatterlist to the receive
 * descriptors for a given SPU. The caller must have already DMA mapped the
 * scatterlist.
 * @spu_idx:    Indicates which SPU the buffers are for
 * @sg:         Scatterlist whose buffers are added to the receive ring
 *
 * If a receive buffer in the scatterlist is larger than PDC_DMA_BUF_MAX,
 * multiple receive descriptors are written, each with a buffer <=
 * PDC_DMA_BUF_MAX.
 *
 * Return: PDC_SUCCESS if successful
 *         < 0 otherwise (e.g., receive ring is full)
 */
static int pdc_rx_list_sg_add(struct pdc_state *pdcs, struct scatterlist *sg)
{
	u32 flags = 0;
	u32 rx_avail;

	/* Num descriptors needed. Conservatively assume we need a descriptor
	 * for every entry from our starting point in the scatterlist.
	 */
	u32 num_desc;
	u32 desc_w = 0;	/* Number of tx descriptors written */
	u32 bufcnt;	/* Number of bytes of buffer pointed to by descriptor */
	dma_addr_t databufptr;	/* DMA address to put in descriptor */

	num_desc = (u32) sg_nents(sg);

	rx_avail = pdcs->nrxpost - NRXDACTIVE(pdcs->rxin, pdcs->rxout,
					      pdcs->nrxpost);
	if (unlikely(num_desc > rx_avail)) {
		pdcs->rxnobuf++;
		return -ENOSPC;
	}

	while (sg) {
		if (unlikely(pdcs->rxout == (pdcs->nrxd - 1)))
			flags = D64_CTRL1_EOT;
		else
			flags = 0;

		/*
		 * DMA cannot accept a buffer larger than 16384 bytes. For
		 * larger buffers, write multiple descriptors.
		 */
		bufcnt = sg_dma_len(sg);
		databufptr = sg_dma_address(sg);
		while (bufcnt > PDC_DMA_BUF_MAX) {
			pdc_build_rxd(pdcs, databufptr, PDC_DMA_BUF_MAX, flags);
			desc_w++;
			bufcnt -= PDC_DMA_BUF_MAX;
			databufptr += PDC_DMA_BUF_MAX;
			if (unlikely(pdcs->rxout == (pdcs->nrxd - 1)))
				flags = D64_CTRL1_EOT;
			else
				flags = 0;
		}
		pdc_build_rxd(pdcs, databufptr, bufcnt, flags);
		desc_w++;
		sg = sg_next(sg);
	}
	pdcs->rxin_numd[pdcs->rx_msg_start] += desc_w;

	return PDC_SUCCESS;
}

/**
 * pdc_irq_handler() - Interrupt handler called in interrupt context.
 * @irq:      Interrupt number that has fired
 * @cookie:   PDC state for DMA engine that generated the interrupt
 *
 * We have to clear the device interrupt status flags here. So cache the
 * status for later use in the thread function. Other than that, just return
 * WAKE_THREAD to invoke the thread function.
 *
 * Return: IRQ_WAKE_THREAD if interrupt is ours
 *         IRQ_NONE otherwise
 */
static irqreturn_t pdc_irq_handler(int irq, void *cookie)
{
	struct pdc_state *pdcs = cookie;
	u32 intstatus = ioread32(pdcs->pdc_reg_vbase + PDC_INTSTATUS_OFFSET);

	if (intstatus & PDC_XMTINTEN_0)
		set_bit(PDC_XMTINTEN_0, &pdcs->intstatus);
	if (intstatus & PDC_RCVINTEN_0)
		set_bit(PDC_RCVINTEN_0, &pdcs->intstatus);

	/* Clear interrupt flags in device */
	iowrite32(intstatus, pdcs->pdc_reg_vbase + PDC_INTSTATUS_OFFSET);

	/* Wakeup IRQ thread */
	if (pdcs && (irq == pdcs->pdc_irq) && (intstatus & PDC_INTMASK))
		return IRQ_WAKE_THREAD;

	return IRQ_NONE;
}

/**
 * pdc_irq_thread() - Function invoked on deferred thread when a DMA tx has
 * completed or data is available to receive.
 * @irq:    Interrupt number
 * @cookie: PDC state for PDC that generated the interrupt
 *
 * On DMA tx complete, notify the mailbox client. On DMA rx complete, process
 * as many SPU response messages as are available and send each to the mailbox
 * client.
 *
 * Return: IRQ_HANDLED if we recognized and handled the interrupt
 *         IRQ_NONE otherwise
 */
static irqreturn_t pdc_irq_thread(int irq, void *cookie)
{
	struct pdc_state *pdcs = cookie;
	struct mbox_controller *mbc;
	struct mbox_chan *chan;
	bool tx_int;
	bool rx_int;
	int rx_status;
	struct brcm_message mssg;

	tx_int = test_and_clear_bit(PDC_XMTINTEN_0, &pdcs->intstatus);
	rx_int = test_and_clear_bit(PDC_RCVINTEN_0, &pdcs->intstatus);

	if (pdcs && (tx_int || rx_int)) {
		dev_dbg(&pdcs->pdev->dev,
			"%s() got irq %d with tx_int %s, rx_int %s",
			__func__, irq,
			tx_int ? "set" : "clear", rx_int ? "set" : "clear");

		mbc = &pdcs->mbc;
		chan = &mbc->chans[PDC_RINGSET];

		if (tx_int) {
			dev_dbg(&pdcs->pdev->dev, "%s(): tx done", __func__);
			/* only one frame in flight at a time */
			mbox_chan_txdone(chan, PDC_SUCCESS);
		}
		if (rx_int) {
			while (1) {
				/* Could be many frames ready */
				memset(&mssg, 0, sizeof(mssg));
				mssg.type = BRCM_MESSAGE_SPU;
				rx_status = pdc_receive(pdcs, &mssg);
				if (rx_status >= 0) {
					dev_dbg(&pdcs->pdev->dev,
						"%s(): invoking client rx cb",
						__func__);
					mbox_chan_received_data(chan, &mssg);
				} else {
					dev_dbg(&pdcs->pdev->dev,
						"%s(): no SPU response available",
						__func__);
					break;
				}
			}
		}
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

/*
 * Allocate DMA rings and initialize constant fields of descriptors in one
 * ringset.
 */
static int pdc_ring_init(struct pdc_state *pdcs, int ringset)
{
	int i;
	int err = PDC_SUCCESS;
	struct dma64 *dma_reg;
	struct device *dev = &pdcs->pdev->dev;
	struct pdc_ring_alloc tx;
	struct pdc_ring_alloc rx;

	/* Allocate tx ring */
	tx.vbase = dma_pool_zalloc(pdcs->ring_pool, GFP_KERNEL, &tx.dmabase);
	if (!tx.vbase) {
		err = -ENOMEM;
		goto done;
	}

	/* Allocate rx ring */
	rx.vbase = dma_pool_zalloc(pdcs->ring_pool, GFP_KERNEL, &rx.dmabase);
	if (!rx.vbase) {
		err = -ENOMEM;
		goto fail_dealloc;
	}

	dev_dbg(dev, " - base DMA addr of tx ring      %#llx", tx.dmabase);
	dev_dbg(dev, " - base virtual addr of tx ring  %p", tx.vbase);
	dev_dbg(dev, " - base DMA addr of rx ring      %#llx", rx.dmabase);
	dev_dbg(dev, " - base virtual addr of rx ring  %p", rx.vbase);

	/* lock after ring allocation to avoid scheduling while atomic */
	spin_lock(&pdcs->pdc_lock);

	memcpy(&pdcs->tx_ring_alloc, &tx, sizeof(tx));
	memcpy(&pdcs->rx_ring_alloc, &rx, sizeof(rx));

	pdcs->rxin = 0;
	pdcs->rx_msg_start = 0;
	pdcs->last_rx_curr = 0;
	pdcs->rxout = 0;
	pdcs->txin = 0;
	pdcs->tx_msg_start = 0;
	pdcs->txout = 0;

	/* Set descriptor array base addresses */
	pdcs->txd_64 = (struct dma64dd *)pdcs->tx_ring_alloc.vbase;
	pdcs->rxd_64 = (struct dma64dd *)pdcs->rx_ring_alloc.vbase;

	/* Tell device the base DMA address of each ring */
	dma_reg = &pdcs->regs->dmaregs[ringset];
	iowrite32(lower_32_bits(pdcs->tx_ring_alloc.dmabase),
		  (void *)&dma_reg->dmaxmt.addrlow);
	iowrite32(upper_32_bits(pdcs->tx_ring_alloc.dmabase),
		  (void *)&dma_reg->dmaxmt.addrhigh);

	iowrite32(lower_32_bits(pdcs->rx_ring_alloc.dmabase),
		  (void *)&dma_reg->dmarcv.addrlow);
	iowrite32(upper_32_bits(pdcs->rx_ring_alloc.dmabase),
		  (void *)&dma_reg->dmarcv.addrhigh);

	/* Initialize descriptors */
	for (i = 0; i < PDC_RING_ENTRIES; i++) {
		/* Every tx descriptor can be used for start of frame. */
		if (i != pdcs->ntxpost) {
			iowrite32(D64_CTRL1_SOF | D64_CTRL1_EOF,
				  (void *)&pdcs->txd_64[i].ctrl1);
		} else {
			/* Last descriptor in ringset. Set End of Table. */
			iowrite32(D64_CTRL1_SOF | D64_CTRL1_EOF |
				  D64_CTRL1_EOT,
				  (void *)&pdcs->txd_64[i].ctrl1);
		}

		/* Every rx descriptor can be used for start of frame */
		if (i != pdcs->nrxpost) {
			iowrite32(D64_CTRL1_SOF,
				  (void *)&pdcs->rxd_64[i].ctrl1);
		} else {
			/* Last descriptor in ringset. Set End of Table. */
			iowrite32(D64_CTRL1_SOF | D64_CTRL1_EOT,
				  (void *)&pdcs->rxd_64[i].ctrl1);
		}
	}
	spin_unlock(&pdcs->pdc_lock);
	return PDC_SUCCESS;

fail_dealloc:
	dma_pool_free(pdcs->ring_pool, tx.vbase, tx.dmabase);
done:
	return err;
}

static void pdc_ring_free(struct pdc_state *pdcs)
{
	if (pdcs->tx_ring_alloc.vbase) {
		dma_pool_free(pdcs->ring_pool, pdcs->tx_ring_alloc.vbase,
			      pdcs->tx_ring_alloc.dmabase);
		pdcs->tx_ring_alloc.vbase = NULL;
	}

	if (pdcs->rx_ring_alloc.vbase) {
		dma_pool_free(pdcs->ring_pool, pdcs->rx_ring_alloc.vbase,
			      pdcs->rx_ring_alloc.dmabase);
		pdcs->rx_ring_alloc.vbase = NULL;
	}
}

/**
 * pdc_send_data() - mailbox send_data function
 * @chan:	The mailbox channel on which the data is sent. The channel
 *              corresponds to a DMA ringset.
 * @data:	The mailbox message to be sent. The message must be a
 *              brcm_message structure.
 *
 * This function is registered as the send_data function for the mailbox
 * controller. From the destination scatterlist in the mailbox message, it
 * creates a sequence of receive descriptors in the rx ring. From the source
 * scatterlist, it creates a sequence of transmit descriptors in the tx ring.
 * After creating the descriptors, it writes the rx ptr and tx ptr registers to
 * initiate the DMA transfer.
 *
 * This function does the DMA map and unmap of the src and dst scatterlists in
 * the mailbox message.
 *
 * Return: 0 if successful
 *	   -ENOTSUPP if the mailbox message is a type this driver does not
 *			support
 *         < 0 if an error
 */
static int pdc_send_data(struct mbox_chan *chan, void *data)
{
	struct pdc_state *pdcs = chan->con_priv;
	struct device *dev = &pdcs->pdev->dev;
	struct brcm_message *mssg = data;
	int err = PDC_SUCCESS;
	int src_nent;
	int dst_nent;
	int nent;

	if (mssg->type != BRCM_MESSAGE_SPU)
		return -ENOTSUPP;

	src_nent = sg_nents(mssg->spu.src);
	if (src_nent) {
		nent = dma_map_sg(dev, mssg->spu.src, src_nent, DMA_TO_DEVICE);
		if (nent == 0)
			return -EIO;
	}

	dst_nent = sg_nents(mssg->spu.dst);
	if (dst_nent) {
		nent = dma_map_sg(dev, mssg->spu.dst, dst_nent,
				  DMA_FROM_DEVICE);
		if (nent == 0) {
			dma_unmap_sg(dev, mssg->spu.src, src_nent,
				     DMA_TO_DEVICE);
			return -EIO;
		}
	}

	spin_lock(&pdcs->pdc_lock);

	/* Create rx descriptors to SPU catch response */
	err = pdc_rx_list_init(pdcs, mssg->spu.dst, mssg->ctx);
	err |= pdc_rx_list_sg_add(pdcs, mssg->spu.dst);

	/* Create tx descriptors to submit SPU request */
	err |= pdc_tx_list_sg_add(pdcs, mssg->spu.src);
	err |= pdc_tx_list_final(pdcs);	/* initiate transfer */

	spin_unlock(&pdcs->pdc_lock);

	if (err)
		dev_err(&pdcs->pdev->dev,
			"%s failed with error %d", __func__, err);

	return err;
}

static int pdc_startup(struct mbox_chan *chan)
{
	return pdc_ring_init(chan->con_priv, PDC_RINGSET);
}

static void pdc_shutdown(struct mbox_chan *chan)
{
	struct pdc_state *pdcs = chan->con_priv;

	if (pdcs)
		dev_dbg(&pdcs->pdev->dev,
			"Shutdown mailbox channel for PDC %u", pdcs->pdc_idx);

	pdc_ring_free(pdcs);
}

/**
 * pdc_hw_init() - Use the given initialization parameters to initialize the
 * state for one of the PDCs.
 * @dev:   device structure for PDC
 * @pdcs:  state of the PDC
 * @parms: parameter values to set
 */
static
void pdc_hw_init(struct device *dev, struct pdc_state *pdcs,
		 struct hw_init_parms *parms)
{
	struct dma64 *dma_reg;

	int ringset = PDC_RINGSET;

	dev_dbg(dev, "PDC %u initial values:", pdcs->pdc_idx);
	dev_dbg(dev, "state structure:                   %p",
		pdcs);
	dev_dbg(dev, " - base physical addr of hw regs   %#llx",
		parms->hw_pbase);
	dev_dbg(dev, " - base virtual addr of hw regs    %p",
		parms->hw_vbase);

	/* initialize data structures */
	pdcs->regs = (struct pdc_regs *)parms->hw_vbase;
	pdcs->txregs_64 = (struct dma64_regs *)
	    (void *)(((u8 *) parms->hw_vbase) +
		     PDC_TXREGS_OFFSET + (sizeof(struct dma64) * ringset));
	pdcs->rxregs_64 = (struct dma64_regs *)
	    (void *)(((u8 *) parms->hw_vbase) +
		     PDC_RXREGS_OFFSET + (sizeof(struct dma64) * ringset));

	pdcs->ntxd = parms->ring_entries;
	pdcs->nrxd = parms->ring_entries;
	pdcs->ntxpost = parms->ring_entries - 1;
	pdcs->nrxpost = parms->ring_entries - 1;
	pdcs->regs->intmask = 0;

	dma_reg = &pdcs->regs->dmaregs[ringset];
	iowrite32(0, (void *)&dma_reg->dmaxmt.control);
	iowrite32(0, (void *)&dma_reg->dmaxmt.ptr);
	iowrite32(0, (void *)&dma_reg->dmarcv.control);
	iowrite32(0, (void *)&dma_reg->dmarcv.ptr);

	iowrite32(PDC_TX_CTL, (void *)&dma_reg->dmaxmt.control);

	iowrite32(PDC_RX_CTL + (pdcs->rx_status_len << 1),
		  (void *)&dma_reg->dmarcv.control);

	if (pdcs->pdc_resp_hdr_len == PDC_SPU2_RESP_HDR_LEN)
		iowrite32(PDC_CKSUM_CTRL,
			  pdcs->pdc_reg_vbase + PDC_CKSUM_CTRL_OFFSET);
}

/**
 * pdc_rx_buf_pool_create() - Pool of receive buffers used to catch the metadata
 * header returned with each response message.
 * @dev:  device structure
 * @pdcs: PDC state structure
 *
 * The metadata is not returned to the mailbox client. So the PDC driver
 * manages these buffers.
 *
 * Return: PDC_SUCCESS
 *         -ENOMEM if pool creation fails
 */
static int pdc_rx_buf_pool_create(struct device *dev, struct pdc_state *pdcs)
{
	pdcs->pdc_resp_hdr_len = pdcs->rx_status_len;
	if (pdcs->use_bcm_hdr)
		pdcs->pdc_resp_hdr_len += BCM_HDR_LEN;

	pdcs->rx_buf_pool = dma_pool_create("pdc rx bufs", dev,
					    pdcs->pdc_resp_hdr_len,
					    RX_BUF_ALIGN_ORDER, 0);
	if (pdcs->rx_buf_pool == NULL)
		return -ENOMEM;

	return PDC_SUCCESS;
}

/**
 * pdc_interrupts_init() - Initialize the interrupt configuration for a PDC and
 * specify a threaded IRQ handler for deferred handling of interrupts outside of
 * interrupt context.
 * @pdev:   platform device
 * @pdcs:   PDC state
 *
 * Set the interrupt mask for transmit and receive done.
 * Set the lazy interrupt frame count to generate an interrupt for just one pkt.
 *
 * Return:  PDC_SUCCESS
 *          <0 if threaded irq request fails
 */
static int pdc_interrupts_init(struct platform_device *pdev,
			       struct pdc_state *pdcs)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn = pdev->dev.of_node;
	int err;

	pdcs->intstatus = 0;

	/* interrupt configuration */
	iowrite32(PDC_INTMASK, pdcs->pdc_reg_vbase + PDC_INTMASK_OFFSET);
	iowrite32(PDC_LAZY_INT, pdcs->pdc_reg_vbase + PDC_RCVLAZY0_OFFSET);

	/* read irq from device tree */
	pdcs->pdc_irq = irq_of_parse_and_map(dn, 0);
	dev_dbg(dev, "pdc device %s irq %u for pdcs %p",
		dev_name(dev), pdcs->pdc_irq, pdcs);
	err = devm_request_threaded_irq(dev, pdcs->pdc_irq,
					pdc_irq_handler,
					pdc_irq_thread, 0, dev_name(dev), pdcs);
	if (err) {
		dev_err(dev, "threaded tx IRQ %u request failed with err %d\n",
			pdcs->pdc_irq, err);
		return err;
	}
	return PDC_SUCCESS;
}

static const struct mbox_chan_ops pdc_mbox_chan_ops = {
	.send_data = pdc_send_data,
	.startup = pdc_startup,
	.shutdown = pdc_shutdown
};

/**
 * pdc_mb_init() - Initialize the mailbox controller.
 * @pdcs:  PDC state
 *
 * Each PDC is a mailbox controller. Each ringset is a mailbox channel. Kernel
 * driver only uses one ringset and thus one mb channel. PDC uses the transmit
 * complete interrupt to determine when a mailbox message has successfully been
 * transmitted.
 *
 * Return: 0 on success
 *         < 0 if there is an allocation or registration failure
 */
static int pdc_mb_init(struct pdc_state *pdcs)
{
	struct device *dev = &pdcs->pdev->dev;
	struct mbox_controller *mbc;
	int chan_index;
	int err;

	mbc = &pdcs->mbc;
	mbc->dev = dev;
	mbc->ops = &pdc_mbox_chan_ops;
	mbc->num_chans = 1;
	mbc->chans = devm_kcalloc(dev, mbc->num_chans, sizeof(*mbc->chans),
				  GFP_KERNEL);
	if (mbc->chans == NULL)
		return -ENOMEM;

	mbc->txdone_irq = true;
	mbc->txdone_poll = false;
	for (chan_index = 0; chan_index < mbc->num_chans; chan_index++)
		mbc->chans[chan_index].con_priv = pdcs;

	/* Register mailbox controller */
	err = mbox_controller_register(mbc);
	if (err) {
		dev_crit(dev,
			 "Failed to register PDC mailbox controller. Error %d.",
			 err);
		return err;
	}
	return 0;
}

/**
 * pdc_dt_read() - Read application-specific data from device tree.
 * @pdev:  Platform device
 * @pdcs:  PDC state
 *
 * Reads the number of bytes of receive status that precede each received frame.
 * Reads whether transmit and received frames should be preceded by an 8-byte
 * BCM header.
 *
 * Return: 0 if successful
 *         -ENODEV if device not available
 */
static int pdc_dt_read(struct platform_device *pdev, struct pdc_state *pdcs)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn = pdev->dev.of_node;
	int err;

	err = of_property_read_u32(dn, "brcm,rx_status_len",
				   &pdcs->rx_status_len);
	if (err < 0)
		dev_err(dev,
			"%s failed to get DMA receive status length from device tree",
			__func__);

	pdcs->use_bcm_hdr = of_property_read_bool(dn, "brcm,use_bcm_hdr");

	return 0;
}

/**
 * pdc_probe() - Probe function for PDC driver.
 * @pdev:   PDC platform device
 *
 * Reserve and map register regions defined in device tree.
 * Allocate and initialize tx and rx DMA rings.
 * Initialize a mailbox controller for each PDC.
 *
 * Return: 0 if successful
 *         < 0 if an error
 */
static int pdc_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device *dev = &pdev->dev;
	struct resource *pdc_regs;
	struct pdc_state *pdcs;
	struct hw_init_parms hw_parms;	/* params for initializing spu-dma */

	pr_info("pdc_probe\n");

	/* PDC state for one SPU */
	pdcs = devm_kzalloc(dev, sizeof(*pdcs), GFP_KERNEL);
	if (pdcs == NULL) {
		err = -ENOMEM;
		goto cleanup;
	}

	spin_lock_init(&pdcs->pdc_lock);
	pdcs->pdev = pdev;
	platform_set_drvdata(pdev, pdcs);
	pdcs->pdc_idx = pdcg.num_spu;
	pdcg.num_spu++;

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (err) {
		dev_warn(dev, "PDC device cannot perform DMA. Error %d.", err);
		goto cleanup;
	}

	/* Create DMA pool for tx ring */
	pdcs->ring_pool = dma_pool_create("pdc rings", dev, PDC_RING_SIZE,
					  RING_ALIGN, 0);
	if (!pdcs->ring_pool) {
		err = -ENOMEM;
		goto cleanup;
	}

	err = pdc_dt_read(pdev, pdcs);
	if (err)
		goto cleanup_ring_pool;

	pdc_regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (pdc_regs == NULL) {
		err = -ENODEV;
		goto cleanup_ring_pool;
	}
	dev_dbg(dev, "PDC register region res.start = %#llx, res.end = %#llx",
		pdc_regs->start, pdc_regs->end);

	pdcs->pdc_reg_vbase = devm_ioremap_resource(&pdev->dev, pdc_regs);
	if (IS_ERR(pdcs->pdc_reg_vbase)) {
		err = PTR_ERR(pdcs->pdc_reg_vbase);
		dev_err(&pdev->dev, "Failed to map registers: %d\n", err);
		goto cleanup_ring_pool;
	}

	/* create rx buffer pool after dt read to know how big buffers are */
	err = pdc_rx_buf_pool_create(dev, pdcs);
	if (err)
		goto cleanup_ring_pool;

	hw_parms.hw_pbase = (dma_addr_t) (pdc_regs->start);
	hw_parms.hw_vbase = pdcs->pdc_reg_vbase;

	hw_parms.ring_entries = PDC_RING_ENTRIES;

	pdc_hw_init(dev, pdcs, &hw_parms);

	err = pdc_interrupts_init(pdev, pdcs);
	if (err)
		goto cleanup_buf_pool;

	/* Initialize mailbox controller */
	err = pdc_mb_init(pdcs);
	if (err)
		goto cleanup_buf_pool;

	pdcs->debugfs_stats = NULL;
	pdc_setup_debugfs(pdcs);

	dev_dbg(dev, "pdc_probe() successful");
	return PDC_SUCCESS;

cleanup_buf_pool:
	dma_pool_destroy(pdcs->rx_buf_pool);

cleanup_ring_pool:
	dma_pool_destroy(pdcs->ring_pool);

cleanup:
	return err;
}

static int pdc_remove(struct platform_device *pdev)
{
	struct pdc_state *pdcs = platform_get_drvdata(pdev);

	pdc_free_debugfs_stats(pdcs);
	pdc_free_debugfs();

	mbox_controller_unregister(&pdcs->mbc);

	dma_pool_destroy(pdcs->rx_buf_pool);
	dma_pool_destroy(pdcs->ring_pool);
	return 0;
}

static const struct of_device_id pdc_mbox_of_match[] = {
	{.compatible = "brcm,pdc-mbox"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pdc_mbox_of_match);

static struct platform_driver pdc_mbox_driver = {
	.probe = pdc_probe,
	.remove = pdc_remove,
	.driver = {
		   .name = "brcm-pdc-mbox",
		   .of_match_table = of_match_ptr(pdc_mbox_of_match),
		   },
};
module_platform_driver(pdc_mbox_driver);

MODULE_AUTHOR("Rob Rice <rob.rice@broadcom.com>");
MODULE_DESCRIPTION("Broadcom PDC mailbox driver");
MODULE_LICENSE("GPL v2");
