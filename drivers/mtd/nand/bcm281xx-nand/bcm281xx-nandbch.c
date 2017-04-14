/*
 * Copyright (C) 2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/nand.h>

#include "bcm281xx-nandbch.h"
#include "bcm281xx-nandregs.h"

#define NANDBCH_BCH_POLL_MAX_COUNT	100000

/* timeout values are in micro seconds */
#define NANDBCH_TIMEOUT_RESET		2000	/* reset timeout (> tRST) */
#define NANDBCH_TIMEOUT_READ		500	/* read timeout (> tR) */
#define NANDBCH_TIMEOUT_WRITE		10000	/* program timeout (> tPROG) */
/* block erase timeout (> tBERS) */
#define NANDBCH_TIMEOUT_ERASE		50000
/* DMA transfer timeout (> page size * max(tRC,tWC) ) */
#define NANDBCH_TIMEOUT_DMA		10000
#define NANDBCH_TIMEOUT_FEATURE		200	/* feature timeout (> tFEAT) */
#define NANDBCH_TIMEOUT_STATUS_GET	200
#define NANDBCH_TIMEOUT_ID_GET		200

#define NANDBCH_IRQ_ALL(bank) \
	(NAND_IRQSTATUS_BCH_ERR_RDY_MASK | \
	 NAND_IRQSTATUS_BCH_WR_ECC_VALID_MASK | \
	 NAND_IRQSTATUS_BCH_RD_ECC_VALID_MASK | \
	 NAND_IRQSTATUS_DMA_CMPL_IRQ_MASK | \
	 NAND_IRQSTATUS_DMA_ERROR_ENABLE_MASK | \
	 (0x1 << (NAND_IRQSTATUS_BANK_CMPL_IRQ_SHIFT + (bank))) | \
	 (0x1 << (NAND_IRQSTATUS_BANK_ERR_IRQ_SHIFT + (bank))) | \
	 (0x1 << (NAND_IRQSTATUS_RB_IRQ_SHIFT + (bank))))

#define NANDBCH_IRQ_ERR(bank) \
	(NAND_IRQSTATUS_DMA_ERROR_ENABLE_MASK | \
	 (0x1 << (NAND_IRQSTATUS_BANK_ERR_IRQ_SHIFT + (bank))))

/* Interrupt status masks indicating the completion of various operation */
#define NANDBCH_IRQ_BANK_RB(bank) \
	((0x1 << (NAND_IRQSTATUS_BANK_CMPL_IRQ_SHIFT + (bank))) | \
	 (0x1 << (NAND_IRQSTATUS_RB_IRQ_SHIFT + (bank))))

#define NANDBCH_IRQ_BANK_DMA(bank) \
	((0x1 << (NAND_IRQSTATUS_BANK_CMPL_IRQ_SHIFT + (bank))) | \
	 NAND_IRQSTATUS_DMA_CMPL_IRQ_MASK)

#define NANDBCH_IRQ_BANK(bank) \
	(0x1 << (NAND_IRQSTATUS_BANK_CMPL_IRQ_SHIFT + (bank)))

#define NANDBCH_IRQ_RB(bank) \
	(0x1 << (NAND_IRQSTATUS_RB_IRQ_SHIFT + (bank)))

/* bad block marker */
#define NAND_BADBLOCK_MARKER_POS	0x0
#define NAND_BADBLOCK_MARKER_GOOD	0xff
#define NAND_BADBLOCK_MARKER_BAD	0x0

/* Configuration table version */
#define NANDBCH_CFG_VER			0
/* Configuration data ECC definitions */
#define NANDBCH_CFG_ECC_T		40
#define NANDBCH_CFG_ECC_N		4656
#define NANDBCH_CFG_ECC_K		4096
#define NANDBCH_CFG_ECC_SIZE		72
#define NANDBCH_CFG_ECC_BYTES		70

/* global variables */
/* DMA buffers and descriptor alignment */
#define NAND_DMA_ALIGN			4
static uint8_t nand_id_buf[NANDBCH_ID_BUF_SIZE]
__aligned(NAND_DMA_ALIGN);
static uint8_t nand_aux_buf[NANDBCH_AUX_MAX_SIZE]
__aligned(NAND_DMA_ALIGN);

/* NAND controller registers */
#define NC_R0		0x0
#define NC_R1		0x1
#define NC_R2		0x2
#define NC_R3		0x3
#define NC_R4		0x4
#define NC_R5		0x5
#define NC_R6		0x6
#define NC_R7		0x7
#define NC_R8		0x8
#define NC_R9		0x9
#define NC_RA		0xa
#define NC_RB		0xb
#define NC_RC		0xc
#define NC_RD		0xd
#define NC_RE		0xe
#define NC_RF		0xf

/* NAND controller instructions */
#define NCI_WC		0x00	/* write command */
#define NCI_WA		0x04	/* write address */
#define NCI_WD		0x08	/* write data */
#define NCI_RD		0x0c	/* sample/read data */
#define NCI_RS		0x10	/* read status */
#define NCI_WT		0x20	/* wait event/timeout */
#define NCI_ADD		0x40
#define NCI_SUB		0x60
#define NCI_AND		0x80
#define NCI_OR		0xa0
#define NCI_SH		0xc0
#define NCI_MV		0xd0
#define NCI_BR		0xe0
#define NCI_END		0x1c

/* NAND controller instruction with 10 and 13 bits for operands */
#define NCI10OP(cmd, op)	(0x10000 | ((cmd & 0xfc) << 8)|((op) & 0x3ff))
#define NCI13OP(cmd, op)	(0x10000 | ((cmd & 0xe0) << 8)|((op) & 0x1fff))

/* NAND controller assembly instructions */
#define CA_END			NCI10OP(NCI_END, 0x3FF)
#define CA_WCI(opcode)		NCI10OP(NCI_WC, (opcode & 0xff))
#define CA_WAI(cycles)		NCI10OP(NCI_WA, (cycles & 0xff))
#define CA_WDI(bytes)		NCI10OP(NCI_WD, (bytes & 0x1ff))
#define CA_WA(reg)		NCI10OP(NCI_WA, (0x200 | (reg & 0xf)))
#define CA_WD(reg)		NCI10OP(NCI_WD, (0x200 | (reg & 0xf)))
#define CA_RD(reg)		NCI10OP(NCI_RD, (0x200 | (reg & 0xf)))
#define CA_RS			NCI10OP(NCI_RS, 0x1)
#define CA_MV(reg1, reg2)	NCI10OP(NCI_MV, ((reg1 & 0xf) << 5) | \
					(reg2 & 0xf))
#define CA_ANDI(reg, val)	NCI13OP(NCI_AND, ((reg & 0xf) << 9) | \
					(val & 0xff))
#define CA_BRA(offset)		NCI13OP(NCI_BR, \
					((offset < 0) ? \
					 (0x100 | (-(offset) & 0xff)) : \
					 (offset & 0xff)))
#define CA_WTS(evt, ticks)	NCI13OP(NCI_WT, ((evt & 0x7) << 10) | \
					(ticks & 0x1ff))

/* NAND controller events */
#define uEVT_RB			0
#define uEVT_DMA_START		1
#define uEVT_WR_DONE		2
#define uEVT_RD_DONE		3
#define uEVT_DMA_DONE		4
#define uEVT_TOUT		7

/* NAND controller timeouts: using steps of ~320 ns */
#define uTIMEOUT_TWB		2
#define uTIMEOUT_TCCS		1
#define uTIMEOUT_TWHR		1
#define uTIMEOUT_TADL		1

/* NAND ONFI timing */
#define NDCONF1_tCH(c)	(min((c), 15) << 20)
#define NDCONF1_tCS(c)	(min((c), 15) << 16)
#define NDCONF1_tWH(c)	(min((c), 15) << 12)
#define NDCONF1_tWP(c)	(min((c), 15) << 8)
#define NDCONF1_tRH(c)	(min((c), 15) << 4)
#define NDCONF1_tRP(c)	(min((c), 15) << 0)

#define NDCONF2_tRHZ(c)	(min((c), 15) << 12)
#define NDCONF2_tCEA(c)	(min((c), 15) << 8)
#define NDCONF2_tIR(c)	(min((c), 15) << 0)
/* convert nano-seconds to nand flash controller clock cycles */
#define ns2cycle(ns, clk)	(int)((ns) * (clk / 1000000) / 1000)

/* NAND controller commands */
enum nand_ctrl_cmd {
	NC_CMD_RESET = 0,
	NC_CMD_ID_GET,
	NC_CMD_PARAM_READ_PRE,
	NC_CMD_GET_FEATURE_PRE,
	NC_CMD_SET_FEATURE,
	NC_CMD_READ_PRE,
	NC_CMD_READ,
	NC_CMD_READ_RANDOM,
	NC_CMD_READ_ECC,
	NC_CMD_READ_RANDOM_ECC,
	NC_CMD_STATUS_GET,
	NC_CMD_BLOCK_ERASE,
	NC_CMD_WRITE_PRE,
	NC_CMD_WRITE_RANDOM,
	NC_CMD_WRITE,
	NC_CMD_WRITE_COMPLETE,
	NC_CMD_MAX
};
struct nand_ctrl_cmds {
	enum nand_ctrl_cmd cmd;		/* nand controller command */
	uint32_t cmd_seq[20];		/* nand controller command sequences */
	uint32_t num_seq;		/* no of command sequences */
};

static const struct nand_ctrl_cmds nc_cmds[] = {
	{
		.cmd = NC_CMD_RESET,
		.cmd_seq = {
			CA_WCI(NAND_CMD_RESET),
			CA_WTS(uEVT_TOUT, uTIMEOUT_TWB),
			CA_END
		},
		.num_seq = 3
	},
	{
		.cmd = NC_CMD_ID_GET,
		.cmd_seq = {
			CA_WTS(uEVT_DMA_START, 0), /* wait for DMA start */
			CA_MV(NC_R2, NC_RB),	/* load address (0x0 or 0x20)*/
			CA_WCI(NAND_CMD_READID), /* send command 0x90 */
			CA_WAI(0x1),		/* one address cycle */
			CA_WTS(uEVT_TOUT, uTIMEOUT_TWHR), /* wait tWHR */
			CA_RD(NC_R4),		/* execute [R4] read cycles */
			CA_BRA(-7),	/* jump to wait for DMA start event */
			CA_END
		},
		.num_seq = 8
	},
	{
		.cmd = NC_CMD_PARAM_READ_PRE,
		.cmd_seq = {
			CA_ANDI(NC_RB, 0x0),	/* zero the address */
			CA_WCI(NAND_CMD_PARAM),	/* send command 0xec */
			CA_WAI(0x1),		/* one address cycle */
			CA_WTS(uEVT_TOUT, uTIMEOUT_TWB),	/* wait tWB */
			CA_END
		},
		.num_seq = 5
	},
	{
		.cmd = NC_CMD_GET_FEATURE_PRE,
		.cmd_seq = {
			CA_MV(NC_R2, NC_RB),	/* load feature address */
			CA_WCI(NAND_CMD_GET_FEATURES),	/* send command 0xee */
			CA_WAI(0x1),			/* one address cycle */
			CA_WTS(uEVT_TOUT, uTIMEOUT_TWB),   /* wait tWB */
			CA_END
		},
		.num_seq = 5
	},
		{
		.cmd = NC_CMD_SET_FEATURE,
		.cmd_seq = {
			CA_WTS(uEVT_DMA_START, 0), /* wait for DMA start event*/
			CA_MV(NC_R2, NC_RB),	/* load feature address */
			CA_WCI(NAND_CMD_SET_FEATURES), /* send command 0xef */
			CA_WAI(0x1),			/* one address cycle */
			CA_WTS(uEVT_TOUT, uTIMEOUT_TADL), /* wait tADL */
			CA_WDI(0x4),			/* write 4 bytes */
			CA_WTS(uEVT_TOUT, uTIMEOUT_TWB),	/* wait tWB */
			CA_BRA(-8),	/* jump to wait for DMA start event */
			CA_END
		},
		.num_seq = 9
	},
	{
		.cmd = NC_CMD_READ_PRE,
		.cmd_seq = {
			CA_MV(NC_R5, NC_RB), /*load 2 bytes of column address*/
			CA_MV(NC_R2, NC_RC), /*load first 2 bytes of address*/
			CA_MV(NC_R3, NC_RD), /*load 3rd byte of address */
			CA_WCI(NAND_CMD_READ0), /* send command 0x00 */
			CA_WA(NC_R6),		/* 4 or 5 address cycles */
			CA_WCI(NAND_CMD_READSTART), /* send command 0x30 */
			CA_WTS(uEVT_TOUT, uTIMEOUT_TWB), /* wait tWB */
			CA_END
		},
		.num_seq = 8
	},
	{
		.cmd = NC_CMD_READ,
		.cmd_seq = {
			CA_WTS(uEVT_DMA_START, 0), /* wait for DMA start event*/
			CA_RD(NC_R4),		/* execute [R4] read cycles */
			CA_BRA(-3),	/* jump to wait for DMA start event */
			CA_END
		},
		.num_seq = 4
	},
	{
		.cmd = NC_CMD_READ_RANDOM,
		.cmd_seq = {
			CA_MV(NC_R5, NC_RB), /*load 2 bytes of column address*/
			CA_WTS(uEVT_DMA_START, 0), /* wait for DMA start event*/
			CA_WCI(NAND_CMD_RNDOUT), /* start random output */
			CA_WAI(0x2),		/* wait 2 address cycles */
			CA_WCI(NAND_CMD_RNDOUTSTART),
			CA_WTS(uEVT_TOUT, uTIMEOUT_TCCS), /* wait tCCS */
			CA_RD(NC_R4),		/* execute [R4] read cycles */
			CA_BRA(-7),	/* jump to wait for DMA start event */
			CA_END
		},
		.num_seq = 9
	},
	{
		.cmd = NC_CMD_READ_ECC,
		.cmd_seq = {
			CA_WTS(uEVT_DMA_START, 0), /* wait for DMA start event*/
			CA_RD(NC_R4), /* execute [R4] read cycles to read data*/
			CA_MV(NC_R7, NC_RB), /* load 2 bytes of ECC offset */
			CA_WCI(NAND_CMD_RNDOUT), /* start random output */
			CA_WAI(0x2),		/* 2 address cycles */
			CA_WCI(NAND_CMD_RNDOUTSTART),
			CA_WTS(uEVT_TOUT, uTIMEOUT_TCCS), /* wait tCCS */
			CA_RD(NC_R6),	/* execute [R6] read cycles to read */
			CA_BRA(-9),	/* jump to wait for DMA start event */
			CA_END
		},
		.num_seq = 10
	},
	{
		.cmd = NC_CMD_READ_RANDOM_ECC,
		.cmd_seq = {
			CA_WTS(uEVT_DMA_START, 0), /* wait for DMA start event*/
			CA_MV(NC_R5, NC_RB),	/* load 2 bytes of data offset*/
			CA_WCI(NAND_CMD_RNDOUT),   /* start random output */
			CA_WAI(0x2),		/* 2 address cycles */
			CA_WCI(NAND_CMD_RNDOUTSTART),
			CA_WTS(uEVT_TOUT, uTIMEOUT_TCCS), /* wait tCCS */
			CA_RD(NC_R4), /* execute [R4] read cycles to read data*/
			CA_MV(NC_R7, NC_RB),	/* load 2 bytes of ECC offset */
			CA_WCI(NAND_CMD_RNDOUT),	/* start random output*/
			CA_WAI(0x2),		/* 2 address cycles */
			CA_WCI(NAND_CMD_RNDOUTSTART),
			CA_WTS(uEVT_TOUT, uTIMEOUT_TCCS), /* wait tCCS */
			CA_RD(NC_R6),	/* execute [R6] read cycles to read */
			CA_BRA(-14),	/* jump to wait for DMA start event */
			CA_END
		},
		.num_seq = 15
	},
	{
		.cmd = NC_CMD_STATUS_GET,
		.cmd_seq = {
			CA_WCI(NAND_CMD_STATUS), /* send command 0x70 (status)*/
			CA_WTS(uEVT_TOUT, uTIMEOUT_TWHR), /* Wait tWHR */
			CA_RS,				/* read status */
			CA_MV(NC_RA, NC_RF),		/* move status to RF */
			CA_END
		},
		.num_seq = 5
	},
	{
		.cmd = NC_CMD_BLOCK_ERASE,
		.cmd_seq = {
			CA_MV(NC_R2, NC_RB), /* load first 2 bytes of address */
			CA_MV(NC_R3, NC_RC), /* load 3rd byte of address */
			CA_WCI(NAND_CMD_ERASE1), /* send command 0x60 (block) */
			CA_WA(NC_R6),		/* 2 or 3 address cycles */
			CA_WCI(NAND_CMD_ERASE2), /* confirm the erase ops */
			CA_WTS(uEVT_TOUT, uTIMEOUT_TWB),	/* wait tWB */
			CA_END
		},
		.num_seq = 7
	},
	{
		.cmd = NC_CMD_WRITE_PRE,
		.cmd_seq = {
			CA_MV(NC_R5, NC_RB),	/* load 2 bytes of column */
			CA_MV(NC_R2, NC_RC), /* load first 2 bytes of address */
			CA_MV(NC_R3, NC_RD), /* load 3rd byte of address */
			CA_WCI(NAND_CMD_SEQIN),		/* send command 0x80 */
			CA_WA(NC_R6),		/* 4 or 5 address cycles */
			CA_WTS(uEVT_TOUT, uTIMEOUT_TADL), /* wait tADL */
			CA_WTS(uEVT_DMA_START, 0), /* wait for DMA start event*/
			CA_WD(NC_R4),		/* execute [R4] write cycles */
			CA_BRA(-3),	/* jump to wait for DMA start event */
			CA_END
		},
		.num_seq = 10
	},
	{
		.cmd = NC_CMD_WRITE_RANDOM,
		.cmd_seq = {
			CA_MV(NC_R5, NC_RB), /* load 2 bytes of column address*/
			CA_WCI(NAND_CMD_RNDIN), /* send command 0x85 */
			CA_WAI(0x2),		/* 2 address cycles */
			CA_WTS(uEVT_TOUT, uTIMEOUT_TCCS),  /* wait tCCS */
			CA_WTS(uEVT_DMA_START, 0), /* wait for DMA start event*/
			CA_WD(NC_R4),		/* execute [R4] write cycles */
			CA_BRA(-3),	/* jump to wait for DMA start event */
			CA_END
		},
		.num_seq = 8
	},
	{
		.cmd = NC_CMD_WRITE,
		.cmd_seq = {
			CA_WTS(uEVT_DMA_START, 0), /* wait for DMA start event*/
			CA_WD(NC_R4),	/* execute [R4] write cycles */
			CA_BRA(-3),	/* jump to wait for DMA start event */
			CA_END
		},
		.num_seq = 4
	},
	{
		.cmd = NC_CMD_WRITE_COMPLETE,
		.cmd_seq = {
			CA_WCI(NAND_CMD_PAGEPROG), /* send command 0x10 */
			CA_WTS(uEVT_TOUT, uTIMEOUT_TWB),	/* wait tWB */
			CA_END
		},
		.num_seq = 3
	},
};

static inline uint32_t nand_readreg(struct ctrl_nand_info *ctrl, uint32_t offs)
{
	return ioread32(ctrl->nand_base + offs);
}

static inline void nand_writereg(struct ctrl_nand_info *ctrl, uint32_t offs,
				 uint32_t val)
{
	iowrite32(val, ctrl->nand_base + offs);
}

/* Configure DMA descriptor entry */
static inline void nc_prd_desc_set(struct nand_prd_entry *entry, void *phys,
				   uint8_t bank, uint32_t size, uint8_t eot)
{
	entry->phys_addr = (uint32_t)phys;
	entry->desc = (eot << 31) | ((bank & 0x7) << 28) | (size & 0x3FFFFF);
}

/*
 * Verify if the loaded micro-code fits into the NAND controller buffer
 */
static inline uint32_t nc_ucode_buf_overflow(struct ctrl_nand_info *ni,
					     uint32_t offset)
{
	return (offset > (uint32_t *)(ni->nand_base+NAND_REG_UCODE_END_OFFSET) -
					(uint32_t *)(ni->nand_base +
						  NAND_REG_UCODE_START_OFFSET));
}

/*
 * Configure DMAINT register
 */
static inline void nc_dmaint_set(struct ctrl_nand_info *ni, uint32_t bank,
				 uint32_t data_size, uint32_t aux_size)
{
	uint32_t aux_type_shift, dmaint;

	aux_type_shift = ((nand_readreg(ni, NAND_CONFIG0_OFFSET) &
				NAND_CONFIG0_AUX_DATA_TYPE_MASK) >>
				NAND_CONFIG0_AUX_DATA_TYPE_SHIFT) + 1;

	dmaint = (data_size >> 4) | NAND_DMAINT0_ENABLE_MASK |
				(bank << 10) | (aux_size >> aux_type_shift);

	nand_writereg(ni, NAND_DMAINT0_OFFSET, dmaint);
	nand_writereg(ni, NAND_DMAINT1_OFFSET, 0x0);
	nand_writereg(ni, NAND_DMAINT2_OFFSET, 0x0);
	nand_writereg(ni, NAND_DMAINT3_OFFSET, 0x0);
}

/*
 * Get the NAND status
 */
static inline uint8_t nc_status_result(struct ctrl_nand_info *ni)
{
	return (nand_readreg(ni, NAND_MRESP_OFFSET) &
					NAND_MRESP_NANDSTATUS_MASK) >>
					NAND_MRESP_NANDSTATUS_SHIFT;
}

/*
 * Reset the NAND controller
 */
static inline void nc_reset(struct ctrl_nand_info *ni)
{
	nand_writereg(ni, NAND_CONTROL_OFFSET, NAND_CONTROL_NORMAL_CONFIG |
						NAND_CONTROL_RESETALL_MASK);
	/* disable and clear interrupts */
	nand_writereg(ni, NAND_IRQCTRL_OFFSET, 0);
	nand_writereg(ni, NAND_IRQSTATUS_OFFSET, ~0x0);
	nand_writereg(ni, NAND_CONTROL_OFFSET, NAND_CONTROL_NORMAL_CONFIG);
	/* disable BCH-ECC */
	nand_writereg(ni, NAND_BCH_CTRL_OFFSET, 0);
}

/*
 * load nand controller command sequences on to the controller buffer at the
 * specified offset and increment offset
 */
static inline uint32_t nc_load_cmd_seq(struct ctrl_nand_info *ni,
				       uint32_t *offset,
				       const uint32_t *cmd_seq,
				       uint32_t num_instr)
{
	uint32_t *ptr;
	uint32_t cmd;

	ptr = (uint32_t *)(ni->nand_base + NAND_REG_UCODE_START_OFFSET) +
		*offset;
	cmd = *offset;
	*offset += num_instr;

	if ((ptr + num_instr) <= (uint32_t *)(ni->nand_base +
				  NAND_REG_UCODE_END_OFFSET)) {
		while (num_instr--)
			*ptr++ = *cmd_seq++;
	}
	return cmd;
}

/* initiate nand controller command sequence */
static inline void nc_exec_cmd(struct ctrl_nand_info *ni, uint32_t bank,
			       uint32_t cmd_offset, uint32_t addr,
			       uint32_t attr0, uint32_t attr1,
			       void *prdbase, uint32_t direction)
{
	uint32_t ctrl;

	ctrl = nand_readreg(ni, NAND_CONTROL_OFFSET);

	/* set the transfer direction */
	if (direction == NAND_DIRECTION_TO_NAND)
		ctrl |= NAND_CONTROL_TRAN_DIR_MASK;
	else
		ctrl &= ~NAND_CONTROL_TRAN_DIR_MASK;

	nand_writereg(ni, NAND_CONTROL_OFFSET, ctrl);
	nand_writereg(ni, NAND_ADDRESS_OFFSET, addr);
	nand_writereg(ni, NAND_ATTRI0_OFFSET, attr0);
	nand_writereg(ni, NAND_ATTRI1_OFFSET, attr1);
	nand_writereg(ni, NAND_BANK_OFFSET, bank);

	/* clear all irq in the status */
	nand_writereg(ni, NAND_IRQSTATUS_OFFSET,
		(NAND_IRQSTATUS_BCH_ERR_RDY_MASK |
		NAND_IRQSTATUS_BCH_WR_ECC_VALID_MASK |
		NAND_IRQSTATUS_BCH_RD_ECC_VALID_MASK |
		NAND_IRQSTATUS_DMA_CMPL_IRQ_MASK |
		NAND_IRQSTATUS_DMA_ERROR_ENABLE_MASK |
		(0x1 << (NAND_IRQSTATUS_BANK_CMPL_IRQ_SHIFT + (bank))) |
		(0x1 << (NAND_IRQSTATUS_BANK_ERR_IRQ_SHIFT + (bank))) |
		(0x1 << (NAND_IRQSTATUS_RB_IRQ_SHIFT + (bank)))));

	nand_writereg(ni, NAND_PRDBASE_OFFSET, (uint32_t)prdbase);

	if (prdbase) {
		/* start DMA */
		nand_writereg(ni, NAND_CONTROL_OFFSET,
			      nand_readreg(ni, NAND_CONTROL_OFFSET) |
			      NAND_CONTROL_DMA_STARTSTOP_MASK);
	} else {
		/* clear DMAINT registers */
		nand_writereg(ni, NAND_DMAINT0_OFFSET, 0x0);
		nand_writereg(ni, NAND_DMAINT1_OFFSET, 0x0);
		nand_writereg(ni, NAND_DMAINT2_OFFSET, 0x0);
		nand_writereg(ni, NAND_DMAINT3_OFFSET, 0x0);
	}

	/* start controller code execution */
	nand_writereg(ni, NAND_COMMAND_OFFSET,
		      NAND_COMMAND_VALID_MASK | cmd_offset);
}

/* set nand controller auxiliary data type field */
static inline uint32_t nc_set_aux_data_type(uint32_t config,
					    uint32_t data_type)
{
	config &= ~NAND_CONFIG0_AUX_DATA_TYPE_MASK;
	return (config | ((data_type << NAND_CONFIG0_AUX_DATA_TYPE_SHIFT)
			  & NAND_CONFIG0_AUX_DATA_TYPE_MASK));
}

/* get nand controller auxiliary data type size in bytes */
static inline uint32_t nc_get_aux_data_type_size(struct ctrl_nand_info *ni)
{
	return 1 << (((nand_readreg(ni, NAND_CONFIG0_OFFSET) &
		      NAND_CONFIG0_AUX_DATA_TYPE_MASK) >>
		      NAND_CONFIG0_AUX_DATA_TYPE_SHIFT) + 1);
}

/* convert onfi timing to capri timing configure */
static void nand_get_sdr_timing(struct ctrl_nand_info *ni,
				const struct nand_sdr_timings *t)
{
	struct clk *clk_peri;
	unsigned long nand_clk;
	uint32_t conf1, conf2;
	uint32_t tCH_min, tCS_min;
	uint32_t tWH_min, tWP_min;
	uint32_t tRH_min, tRP_min;
	uint32_t tRHZ_max, tCEA_max, tIR_min;

	clk_peri = clk_get(ni->dev, "nand_clk");
	nand_clk = clk_get_rate(clk_peri);

	tCH_min = DIV_ROUND_UP(t->tCH_min, 1000);
	tCS_min = DIV_ROUND_UP(t->tCS_min, 1000);
	tWH_min = DIV_ROUND_UP(t->tWH_min, 1000);
	tWP_min = DIV_ROUND_UP(t->tWP_min, 1000);
	tRH_min = DIV_ROUND_UP(t->tREH_min, 1000);
	tRP_min = DIV_ROUND_UP(t->tRP_min, 1000);
	tRHZ_max = DIV_ROUND_UP(t->tRHZ_max, 1000);
	tCEA_max = DIV_ROUND_UP(t->tCEA_max, 1000);
	tIR_min = DIV_ROUND_UP(t->tIR_min, 1000);

	conf1 = NDCONF1_tCH(ns2cycle(tCH_min, nand_clk)) |
		NDCONF1_tCS(ns2cycle(tCS_min, nand_clk)) |
		NDCONF1_tWH(ns2cycle(tWH_min, nand_clk)) |
		NDCONF1_tWP(ns2cycle(tWP_min, nand_clk)) |
		NDCONF1_tRH(ns2cycle(tRH_min, nand_clk)) |
		NDCONF1_tRP(ns2cycle(tRP_min, nand_clk));

	conf2 = NDCONF2_tRHZ(ns2cycle(tRHZ_max, nand_clk)) |
		NDCONF2_tCEA(ns2cycle(tCEA_max, nand_clk)) |
		NDCONF2_tIR(ns2cycle(tIR_min, nand_clk));

	ni->timing.conf1 = conf1;
	ni->timing.conf2 = conf2;
}

/* get nand sdr_timing from onfi according to the time mode */
static int nand_init_timings_onfi(struct ctrl_nand_info *ni)
{
	const struct nand_sdr_timings *timings;

	timings = onfi_async_timing_mode_to_sdr_timings(ni->timing.mode);
	if (IS_ERR(timings))
		return PTR_ERR(timings);

	nand_get_sdr_timing(ni, timings);

	return 0;
}

/* configure BCH ECC parameters */
static inline void nc_bch_config(struct ctrl_nand_info *ni,
				 uint32_t bus, uint32_t n,
				 uint32_t k, uint32_t t)
{
	/* set bus width and disable read/write bch ecc calculation */
	nand_writereg(ni, NAND_BCH_CTRL_OFFSET,
		      NAND_BCH_CTRL_BUS_WIDTH_MASK &
		      ((bus >> 4) << NAND_BCH_CTRL_BUS_WIDTH_SHIFT));

	nand_writereg(ni, NAND_BCH_NK_OFFSET,
		      (NAND_BCH_NK_K_MASK & (k << NAND_BCH_NK_K_SHIFT)) |
		      (NAND_BCH_NK_N_MASK & (n << NAND_BCH_NK_N_SHIFT)));

	nand_writereg(ni, NAND_BCH_T_OFFSET,
		      NAND_BCH_T_T_MASK & (t << NAND_BCH_T_T_SHIFT));
}

/* enable bch ecc computation for read */
static inline void nc_bch_read_enable(struct ctrl_nand_info *ni)
{
	uint32_t bch_ctrl;

	bch_ctrl = nand_readreg(ni, NAND_BCH_CTRL_OFFSET);
	nand_writereg(ni, NAND_BCH_CTRL_OFFSET,
		      bch_ctrl | NAND_BCH_CTRL_ECC_RD_EN_MASK);
}

/* enable bch ecc computation for write */
static inline void nc_bch_write_enable(struct ctrl_nand_info *ni)
{
	uint32_t bch_ctrl;

	bch_ctrl = nand_readreg(ni, NAND_BCH_CTRL_OFFSET);
	nand_writereg(ni, NAND_BCH_CTRL_OFFSET,
		      bch_ctrl | NAND_BCH_CTRL_ECC_WR_EN_MASK);
}

/* disable bch ecc BCH ECC computation (for both read and write) */
static inline void nc_bch_disable(struct ctrl_nand_info *ni)
{
	uint32_t bch_ctrl;

	bch_ctrl = nand_readreg(ni, NAND_BCH_CTRL_OFFSET);
	nand_writereg(ni, NAND_BCH_CTRL_OFFSET,
		      bch_ctrl &
		      ~(NAND_BCH_CTRL_ECC_RD_EN_MASK |
			NAND_BCH_CTRL_ECC_WR_EN_MASK));
}

/*
 * Returns the number of correctable errors or:
 *    -1 for uncorrectable error
 *    0 for no error
 */
static inline int32_t nc_bch_err_count(struct ctrl_nand_info *ni)
{
	uint32_t bch_status;

	bch_status = nand_readreg(ni, NAND_BCH_STATUS_OFFSET);

	/* check if uncorrectable error was detected */
	if (bch_status & NAND_BCH_STATUS_UNCORR_ERR_MASK)
		return -1;

	/* check if correctable error was detected */
	if (bch_status & NAND_BCH_STATUS_CORR_ERR_MASK)
		return (bch_status & NAND_BCH_STATUS_NB_CORR_ERR_MASK) >>
			NAND_BCH_STATUS_NB_CORR_ERR_SHIFT;
	return 0;
}

/*
 * Retrieve all bad bit locations and fix
 * Note: The len parameter is used to limit the corrections to the data
 * portion only (i.e. not correct bad bits in ECC bytes when this is not
 * needed and/or ECC bytes have been transferred to a different location)
 */
static inline void nc_bch_err_fix(struct ctrl_nand_info *ni, int32_t err_count,
				  uint8_t *buf, uint32_t len)
{
	uint32_t bit, byte;

	while (err_count > 0) {
		err_count--;
		bit = (nand_readreg(ni, NAND_BCH_ERR_LOC_OFFSET) &
		       NAND_BCH_ERR_LOC_ERR_LOC_MASK) >>
			NAND_BCH_ERR_LOC_ERR_LOC_SHIFT;
		byte = bit >> 3;
		if (byte < len)
			buf[byte] ^= 0x1 << (bit & 0x7);
	}
}

/* copy the calculated bch ecc bytes from controller to specified destination */
static inline void nc_bch_ecc_copy(struct ctrl_nand_info *ni, uint8_t *dest,
				   uint32_t len)
{
	uint32_t buf[18];
	uint32_t w;
	uint32_t *w_src, *w_dest, *w_end;
	uint8_t  *b_src, *b_end;

	w = (len + 3) >> 2;
	w_src = (uint32_t *)(NAND_BCH_WR_ECC0_OFFSET);
	w_dest = buf;
	w_end = buf + w;
	while (w_dest < w_end) {
		*w_dest = nand_readreg(ni, (uint32_t)w_src);
		w_dest++;
		w_src++;
	}

	b_end = (uint8_t *)buf - 1;
	b_src = (uint8_t *)buf + len - 1;
	while (b_end < b_src)
		*dest++ = *b_src--;
}

/*
 * Get DMA active status
 */
static inline uint32_t nc_dma_active(struct ctrl_nand_info *ni)
{
	return (nand_readreg(ni, NAND_STATUS_OFFSET) &
		NAND_STATUS_DMA_STATUS_MASK) >> NAND_STATUS_DMA_STATUS_SHIFT;
}

#define nandbch_reset_device              nandbch_reset

static uint32_t shift_of(uint32_t i)
{
	uint32_t s = 0;

	i >>= 1;
	while (i) {
		s++;
		i >>= 1;
	}
	return s;
}

/*
 * Wait for the specified interrupts.
 * Returns 1 for timeout, 0 for success
 */
static uint32_t wait_for_ucode_completion(struct ctrl_nand_info *ni,
					  uint32_t bank, uint32_t irq_mask,
					  int32_t usec)
{
	int32_t delay_usec = 1;	/* polling period */
	uint32_t ret = 0;

	while (usec > 0) {
		if (irq_mask & NAND_IRQSTATUS_DMA_CMPL_IRQ_MASK) {
			if (((irq_mask & nand_readreg(ni,
			    NAND_IRQSTATUS_OFFSET)) == irq_mask)
			    && (!nc_dma_active(ni)))
				break;
		} else {
			if ((irq_mask & nand_readreg(ni,
			    NAND_IRQSTATUS_OFFSET)) == irq_mask)
				break;
		}
		udelay(delay_usec);
		usec -= delay_usec;
	}

	if (usec <= 0) {
		ret = ((irq_mask & nand_readreg(ni,
			NAND_IRQSTATUS_OFFSET)) != irq_mask) ||
			((irq_mask & NAND_IRQSTATUS_DMA_CMPL_IRQ_MASK) ?
			nc_dma_active(ni) : 0);
	}

	if (irq_mask & NAND_IRQSTATUS_DMA_CMPL_IRQ_MASK)
		nand_writereg(ni, NAND_CONTROL_OFFSET,
			      nand_readreg(ni, NAND_CONTROL_OFFSET) &
			      ~NAND_CONTROL_DMA_STARTSTOP_MASK);

	/* clear irqs */
	if (ret == 0)
		nand_writereg(ni, NAND_IRQSTATUS_OFFSET,
			      irq_mask | NANDBCH_IRQ_ERR(bank));

	return ret;
}

static inline void bch_config_aux(struct ctrl_nand_info *ni)
{
	nc_bch_config(ni, ni->geometry.bus_width, ni->aux.ecc_n,
		      ni->aux.ecc_k, ni->aux.ecc_t);
}

static inline void bch_config_data(struct ctrl_nand_info *ni)
{
	nc_bch_config(ni, ni->geometry.bus_width, ni->sector.ecc_n,
		      ni->sector.ecc_k, ni->sector.ecc_t);
}

static inline int wait_ucode_irq_rb(struct ctrl_nand_info *ni, uint32_t bank,
				    int32_t usec)
{
	return wait_for_ucode_completion(ni, bank, NANDBCH_IRQ_RB(bank), usec);
}

static inline int wait_ucode_bank_dma(struct ctrl_nand_info *ni, uint32_t bank,
				      int32_t usec)
{
	return wait_for_ucode_completion(ni, bank,
					 NANDBCH_IRQ_BANK_DMA(bank), usec);
}

static inline int wait_ucode_bank_rb(struct ctrl_nand_info *ni, uint32_t bank,
				     int32_t usec)
{
	return wait_for_ucode_completion(ni, bank, NANDBCH_IRQ_BANK_RB(bank),
					 usec);
}

static inline int wait_ucode_bank(struct ctrl_nand_info *ni, uint32_t bank,
				  int32_t usec)
{
	return wait_for_ucode_completion(ni, bank, NANDBCH_IRQ_BANK(bank),
					 usec);
}

static int cmd_completion_err(struct ctrl_nand_info *ni, uint32_t bank,
			      const char *func, uint32_t cmd)
{
	uint32_t irqstatus = nand_readreg(ni, NAND_IRQSTATUS_OFFSET);
	int ret = -ETIME;

	dev_dbg(ni->dev, "Completion error: %s cmd %d, bank %d irqstatus 0x%x\n",
	 func, cmd, bank, irqstatus);

	if (irqstatus & (0x1 << (NAND_IRQSTATUS_BANK_ERR_IRQ_SHIFT + bank)))
		ret = -ENOEXEC;

	if (irqstatus & NAND_IRQSTATUS_DMA_ERROR_ENABLE_MASK)
		ret = -EIO;

	nand_writereg(ni, NAND_IRQSTATUS_OFFSET, NANDBCH_IRQ_ALL(bank));

	return ret;
}

/**
 * nandbch_reset(ni, bank, buf) - Send reset command to the nand device
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 */
int nandbch_reset(struct ctrl_nand_info *ni, uint8_t bank)
{
	/* disable ECC */
	nc_bch_disable(ni);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_RESET], 0,	/* addr */
			     0,	/* attr0 */
			     0,	/* attr1 */
			     NULL,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_rb(ni, bank, NANDBCH_TIMEOUT_RESET))
		return cmd_completion_err(ni, bank, __func__, NC_CMD_RESET);

	return 0;
}

static int nand_bch_err_count(struct ctrl_nand_info *ni, int32_t *err_count)
{
	uint32_t cnt = NANDBCH_BCH_POLL_MAX_COUNT;

	while (cnt) {
		if ((nand_readreg(ni, NAND_IRQSTATUS_OFFSET) &
		     NAND_IRQSTATUS_BCH_RD_ECC_VALID_MASK) >>
		     NAND_IRQSTATUS_BCH_RD_ECC_VALID_SHIFT)
			break;
		cnt--;
	}

	if (cnt == 0) {
		dev_dbg(ni->dev, "Read ECC valid IRQ timeout\n");
		return -ETIME;
	}

	*err_count = nc_bch_err_count(ni);

	nand_writereg(ni, NAND_IRQSTATUS_OFFSET,
		      NAND_IRQSTATUS_BCH_RD_ECC_VALID_MASK);

	return 0;
}

static uint32_t nand_bch_err_fix(struct ctrl_nand_info *ni, int32_t err_count,
				 uint8_t *buf, uint32_t len)
{
	uint32_t cnt = NANDBCH_BCH_POLL_MAX_COUNT;

	while (cnt) {
		if ((nand_readreg(ni, NAND_IRQSTATUS_OFFSET) &
		     NAND_IRQSTATUS_BCH_ERR_RDY_MASK) >>
		     NAND_IRQSTATUS_BCH_ERR_RDY_SHIFT)
			break;
		cnt--;
	}

	if (cnt == 0) {
		dev_dbg(ni->dev, "ECC error ready IRQ timeout\n");
		return -ETIME;
	}

	nc_bch_err_fix(ni, err_count, buf, len);

	nand_writereg(ni, NAND_IRQSTATUS_OFFSET,
		      NAND_IRQSTATUS_BCH_ERR_RDY_MASK);

	return 0;
}

static uint32_t nand_bch_ecc_copy(struct ctrl_nand_info *ni, uint8_t *buf,
				  uint32_t len, uint32_t total_len)
{
	uint32_t cnt = NANDBCH_BCH_POLL_MAX_COUNT;
	uint8_t *p;

	while (cnt) {
		if ((nand_readreg(ni, NAND_IRQSTATUS_OFFSET) &
		     NAND_IRQSTATUS_BCH_WR_ECC_VALID_MASK) >>
		    NAND_IRQSTATUS_BCH_WR_ECC_VALID_SHIFT)
			break;
		cnt--;
	}

	if (cnt == 0) {
		dev_dbg(ni->dev, "Write ECC valid IRQ timeout\n");
		return -ETIME;
	}

	nc_bch_ecc_copy(ni, buf, len);

	/* zero the padding bytes */
	for (p = buf + len; p < buf + total_len; p++)
		*p = 0;

	nand_writereg(ni, NAND_IRQSTATUS_OFFSET,
		      NAND_IRQSTATUS_BCH_WR_ECC_VALID_MASK);

	return 0;
}

/**
 * CRC16 verification
 *
 * Argunents:
 *    datap    pointer to byte stream
 *    len      byte stream length
 *    crc16p   pointer to CRC16
 * Return:
 *    1     crc correct
 *    0     crc incorrect
 */
static uint32_t crc16_valid(uint8_t *datap, uint32_t len, uint8_t *crc16p)
{
	/* Bit by bit algorithm without augmented zero bytes */
	const uint32_t crcinit = 0x4F4E; /* initial CRC value in the shift reg*/
	const int32_t order = 16;	/* Order of the CRC-16 */
	const uint32_t polynom = 0x8005;	/* Polynomial */

	uint32_t i, j, c, bit;
	uint32_t crc;
	uint32_t crcmask, crchighbit;

	crc = crcinit;	/* Initialize the shift register with 0x4F4E */
	crcmask = ((((uint32_t) 1 << (order - 1)) - 1) << 1) | 1;
	crchighbit = (uint32_t) 1 << (order - 1);

	/*
	 * process byte stream, one byte at a time,
	 * bits processed from MSB to LSB
	 */
	for (i = 0; i < len; i++) {
		c = (uint32_t) (*datap);
		datap++;
		for (j = 0x80; j; j >>= 1) {
			bit = crc & crchighbit;
			crc <<= 1;
			if (c & j)
				bit ^= crchighbit;
			if (bit)
				crc ^= polynom;
		}
		crc &= crcmask;
	}

	c = ((uint32_t) (*(crc16p + 1)) << 8) | (uint32_t) (*crc16p);

	return (c == crc);
}

/**
 * Send ONFI read parameter page command to device
 * @ni:    [in] nand info structure
 * @bank:  [in] bank number
 */
static int nand_param_read_pre(struct ctrl_nand_info *ni, uint8_t bank)
{
	/* disable ECC */
	nc_bch_disable(ni);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_PARAM_READ_PRE],
			     0,	/* addr */
			     0,	/* attr0 */
			     0,	/* attr1 */
			     NULL,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_rb(ni, bank, NANDBCH_TIMEOUT_READ)) {
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_PARAM_READ_PRE);
	}

	return 0;
}

/* Setup descriptor */
static inline void setup_desc(struct ctrl_nand_info *ni, dma_addr_t addr,
			      uint8_t bank, uint32_t size)
{
	nc_prd_desc_set(&ni->desc.desc[0], (void *)addr, bank, size, 1);
}

/* Setup descriptor pair */
static inline void setup_desc2(struct ctrl_nand_info *ni, dma_addr_t addr,
			       uint8_t bank1, uint32_t size, dma_addr_t addr2,
			       uint8_t bank2, uint32_t size2)
{
	nc_prd_desc_set(&ni->desc.desc[0], (void *)addr, bank1, size, 0);
	nc_prd_desc_set(&ni->desc.desc[1], (void *)addr2, bank2, size2, 1);
}

enum for_dma { for_device, for_cpu };
/* Cache maintenance for descriptor */
static inline void cache_desc(enum for_dma owner, struct ctrl_nand_info *ni)
{
	if (owner == for_device)
		dma_sync_single_for_device(ni->dev, ni->desc.phys_addr,
					   sizeof(ni->desc.desc),
					   DMA_TO_DEVICE);
	else
		dma_sync_single_for_cpu(ni->dev, ni->desc.phys_addr,
					sizeof(ni->desc.desc), DMA_TO_DEVICE);
}

/* Cache maintenance for buffer */
static inline void cache_buf(enum for_dma owner, struct ctrl_nand_info *ni,
			     dma_addr_t addr, uint32_t size,
			     enum dma_data_direction dir)
{
	if (owner == for_device)
		dma_sync_single_for_device(ni->dev, addr, size, dir);
	else
		dma_sync_single_for_cpu(ni->dev, addr, size, dir);

}

/* Setup descriptor and do cache maintenance on it */
static inline void cache_desc_for_device(struct ctrl_nand_info *ni,
					 dma_addr_t addr,
					 uint8_t bank,
					 uint32_t size)
{
	setup_desc(ni, addr, bank, size);
	cache_desc(for_device, ni);
}

/* Setup descriptor pair and do cache maintenance on it */
static inline void cache_desc2_for_device(struct ctrl_nand_info *ni,
					  dma_addr_t addr,
					  uint8_t bank1,
					  uint32_t size,
					  dma_addr_t addr2,
					  uint8_t bank2,
					  uint32_t size2)
{
	setup_desc2(ni, addr, bank1, size, addr2, bank2, size2);
	cache_desc(for_device, ni);
}

/* Setup descriptor pair and do cache maintenance on it and on the buffer */
static inline void cache_desc2_buf_for_device(struct ctrl_nand_info *ni,
					      dma_addr_t addr,
					      uint8_t bank1,
					      uint32_t size,
					      dma_addr_t addr2,
					      uint8_t bank2,
					      uint32_t size2,
					      enum dma_data_direction dir)
{

	cache_desc2_for_device(ni, addr, bank1, size, addr2, bank2, size2);

	/* First buffer is the main data buffer */
	cache_buf(for_device, ni, addr, size, dir);
}

/* Setup descriptor and do cache maintenance on it and on the buffer */
static inline void cache_desc_buf_for_device(
	struct ctrl_nand_info *ni,
	dma_addr_t addr,
	uint8_t bank,
	uint32_t size,
	enum dma_data_direction dir)
{
	cache_desc_for_device(ni, addr, bank, size);
	cache_buf(for_device, ni, addr, size, dir);
}

/* Do cache maintenance on the descriptor and the buffer */
static inline void cache_desc_buf_for_cpu(struct ctrl_nand_info *ni,
					  dma_addr_t addr,
					  uint8_t bank,
					  uint32_t size,
					  enum dma_data_direction dir)
{
	(void)bank;	/* Keep prototypes the same for ease of use */

	cache_desc(for_cpu, ni);
	cache_buf(for_cpu, ni, addr, size, dir);
}

/**
 * get nand id
 * @ni:   [in] nand info structure
 * @bank: [in] bank number
 * @addr: [in] address (0x0 or 0x20)
 * @len:  [in] length (8 or 4)
 * @buf:  [out] buffer to get the id
 */
int nandbch_id_get(struct ctrl_nand_info *ni, uint8_t bank, uint8_t addr,
		   uint8_t len, uint8_t *buf)
{
	dma_addr_t bufaddr = ni->buf.phys_addr;
	uint32_t size = len;
	enum dma_data_direction dir = DMA_FROM_DEVICE;

	nc_dmaint_set(ni, bank, 0, len);
	cache_desc_buf_for_device(ni, bufaddr, bank, size, dir);

	/* disable ECC */
	nc_bch_disable(ni);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_ID_GET],
			     addr,	/* addr */
			     len,	/* attr0 */
			     0,	/* attr1 */
			     (void *)ni->desc.phys_addr,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_ID_GET)) {
		return cmd_completion_err(ni, bank, __func__,
					  NC_CMD_ID_GET);
	}
	cache_desc_buf_for_cpu(ni, bufaddr, bank, size, dir);
	memcpy(buf, ni->buf.buf, len);
	return 0;
}
/**
 * Read random from parameter page
 * @ni:      [in] nand info structure
 * @bank:    [in] bank number
 * @offset:  [in] byte offset
 * @size:    [in] data size
 * @buf:     [out] buffer
 */
static int nand_param_read_random(struct ctrl_nand_info *ni, uint8_t bank,
				  uint32_t offset, uint32_t size, uint8_t *buf)
{
	dma_addr_t addr = ni->buf.phys_addr;
	enum dma_data_direction dir = DMA_FROM_DEVICE;

	/* round up size to multiple of 512 bytes (HW limitation: HWCAPRI-385)*/
	size = ((size + 511) / 512) * 512;
	nc_dmaint_set(ni, bank, size, 0);
	cache_desc_buf_for_device(ni, addr, bank, size, dir);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset
			     [NC_CMD_READ_RANDOM], 0,	/* addr */
			     (offset << 16) | size,	/* attr0 */
			     0,	/* attr1 */
			     (void *)ni->desc.phys_addr,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
		cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_READ_RANDOM);
	}
	cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
	memcpy(buf, ni->buf.buf, size);
	return 0;
}

/**
 * Read ONFI parameter page (x 2)
 * @ni:    [in] nand info structure
 * @bank:  [in] bank number
 * @buf:   [out] buffer
 */
static uint32_t nand_param_read(struct ctrl_nand_info *ni, uint8_t bank,
				uint8_t *buf)
{
	int bufsize = 512;
	dma_addr_t addr = ni->buf.phys_addr;
	uint32_t size = bufsize;
	enum dma_data_direction dir = DMA_FROM_DEVICE;

	nc_dmaint_set(ni, bank, bufsize, 0);
	cache_desc_buf_for_device(ni, addr, bank, size, dir);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_READ],
			     0,	/* addr */
			     bufsize,	/* attr0 */
			     0,	/* attr1 */
			     (void *)ni->desc.phys_addr,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
		cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
		return cmd_completion_err(ni, bank, __func__, NC_CMD_READ);
	}
	cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
	memcpy(buf, ni->buf.buf, bufsize);
	return 0;
}

/**
 * configures the geometry based on the ONFI parameter page
 */
static int nand_config_onfi(struct ctrl_nand_info *ni)
{
	static uint32_t buf[1024];
	uint8_t *ext_param = (uint8_t *) buf;
	uint8_t *param_page = NULL;
	uint32_t i, j;
	uint32_t page_size;
	uint32_t pages_per_block;
	uint32_t blocks_per_lun;
	uint32_t lun_per_chip_enable;
	uint32_t valid_param_page;
	uint32_t param_page_cpy;
	uint32_t ext_param_offset;
	uint32_t ext_param_size;
	uint32_t ecc_section_offset;
	uint32_t timing_mode;
	uint8_t ecc;
	int ret;

	ret = nandbch_id_get(ni, 0, NANDBCH_ONFI_ID_ADDR,
				    NANDBCH_ONFI_ID_SIZE,
				    nand_id_buf);
	if (ret)
		return ret;

	if ((nand_id_buf[0] != 'O') ||
	    (nand_id_buf[1] != 'N') ||
	    (nand_id_buf[2] != 'F') ||
	    (nand_id_buf[3] != 'I'))
		return -ENODEV;
	ret = nand_param_read_pre(ni, 0);
	if (ret)
		return ret;

	i = 0;
	valid_param_page = 0;
	while (!valid_param_page && (i < 4)) {
		i++;
		/* due to hardware limitation fetch get 512 bytes in one read*/
		ret = nand_param_read(ni, 0, ni->param_buf.buf);
		if (ret)
			return ret;

		for (j = 0; j < 2; j++) {
			param_page = ni->param_buf.buf + 256 * j;

			/* validate ONFI parameter page signature */
			if ((param_page[0] != 'O') ||
			    (param_page[1] != 'N') ||
			    (param_page[2] != 'F') ||
			    (param_page[3] != 'I'))
				continue;

			/* validated CRC */
			if (crc16_valid(param_page, 254, param_page + 254)) {
				valid_param_page = 1;
				break;
			}
		}
	}

	/* return if no valid parameter page has been found */
	if (valid_param_page == 0)
		return -ENODEV;

	/* configure geometry parameters */
	page_size = get_unaligned_le32(&param_page[80]);
	pages_per_block = get_unaligned_le32(&param_page[92]);
	blocks_per_lun = get_unaligned_le32(&param_page[96]);
	lun_per_chip_enable = param_page[100];
	timing_mode = get_unaligned_le16(&param_page[129]);
	ni->geometry.bus_width = (param_page[6] & 0x1) ? 16 : 8;
	if (param_page[101] == 0x22)
		ni->geometry.addr_cycles = 4;
	else if (param_page[101] == 0x23)
		ni->geometry.addr_cycles = 5;
	else
		ni->geometry.addr_cycles = 0;

	ni->geometry.page_shift = shift_of(page_size);
	ni->geometry.block_shift = shift_of(pages_per_block) +
					ni->geometry.page_shift;
	ni->geometry.bank_shift = shift_of(blocks_per_lun) +
					shift_of(lun_per_chip_enable) +
					ni->geometry.block_shift;
	ni->geometry.oob_size = get_unaligned_le16(&param_page[84]);

	ni->onfi.feature = get_unaligned_le16(&param_page[6]);
	ni->onfi.opt_cmd = get_unaligned_le16(&param_page[8]);

	if ((ni->flags & NAND_FLAG_TIMING) == 0) {

		/* set timing mode to the maximum supported by the device */
		ni->timing.mode = 5;
		while (ni->timing.mode > 0) {
			if (timing_mode & (1UL << ni->timing.mode))
				break;
			ni->timing.mode--;
		}
		ni->timing.select = NANDBCH_TIMING_SELECT_TMODE_ONLY;

		if (ni->onfi.opt_cmd & NANDBCH_OPT_CMD_GET_SET_FEATURE)
			ni->timing.select =
				NANDBCH_TIMING_SELECT_TMODE_SET_FEATURE;

		ni->flags |= NAND_FLAG_TIMING;
	}
	if ((ni->flags & NAND_FLAG_ECC_CONFIG) == 0) {
		/* configure ECC */
		ecc = param_page[112];
		if (ecc != 0xff) {
			ni->sector.data_size = 512;
			ni->sector.ecc_t = ecc;
		} else {
			/* extended parameter page not supported */
			if ((ni->onfi.feature &
			     NANDBCH_FEATURE_EXT_PARAM_PAGE) == 0)
				return -EINVAL;

			param_page_cpy = param_page[14];
			ext_param_offset = 256 * param_page_cpy;
			ext_param_size =
				get_unaligned_le16(&param_page[12]) << 4;

			if (ext_param_size > sizeof(buf))
				return -EINVAL;

			for (i = 0; i < param_page_cpy; i++) {
				ret = nand_param_read_random(ni, 0,
							ext_param_offset,
							ext_param_size,
							ext_param);
				if (ret)
					return ret;

				ext_param_offset += ext_param_size;

				/* validate extended parameter page signature */
				if ((ext_param[2] != 'E') ||
				    (ext_param[3] != 'P') ||
				    (ext_param[4] != 'P') ||
				    (ext_param[5] != 'S'))
					continue;

				/* validated CRC */
				if (crc16_valid(ext_param + 2,
						ext_param_size - 2,
						ext_param)) {
					ecc_section_offset = 32;
					if (ext_param[16] != 2) {
						ecc_section_offset +=
							(ext_param[17] << 4);
					if (ext_param[18] != 2)
						return -EINVAL;
					}

					ni->sector.ecc_t =
						ext_param[ecc_section_offset];
					ni->sector.data_size = 0x1 <<
						ext_param[ecc_section_offset+1];
					break;
				}
			}
			/* return if no valid extended parameter page
			 * has been found
			 */
			if (i == param_page_cpy)
				return -EINVAL;
		}
	}

	return 0;
}

/**
 * Reads one page from nand into buf with BCH-ECC correction
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @page:   [in] page number
 * @buf:    [out] target buffer
 * @stats:  [out] ECC stats (number of bad bits per sector)
 * @ecc_error:   [out] Uncorrectable BCH ECC errors
 *
 * Note: If the caller wants ECC correction stats it needs to provide a pointer
 * to a nandbch_eccstats structure. The number of bits corrected
 * by ECC in every sector will be returned in the corresponding array element.
 * If stats are not needed caller should provide a NULL pointer argument.
 */
static int nandbch_page_read_ecc(struct ctrl_nand_info *ni, uint8_t bank,
				 uint32_t page, uint8_t *buf,
				 struct nandbch_eccstats *stats)
{
	uint32_t sector;
	uint32_t sect_offs;
	uint32_t ecc_offs;
	int32_t err_count;
	dma_addr_t sect_buf;
	uint32_t cmd;
	int ret;

	/* disable ECC */
	nc_bch_disable(ni);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_READ_PRE],
			     page,	/* addr */
			     0,	/* attr0 */
			     ni->geometry.addr_cycles,	/* attr1 */
			     NULL,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_rb(ni, bank, NANDBCH_TIMEOUT_READ)) {
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_READ_PRE);
	}

	/* Configure ECC for main data */
	bch_config_data(ni);

	for (sector = 0; sector < NANDBCH_SECTORS(ni); sector++) {
		dma_addr_t addr;
		uint32_t size = NANDBCH_SECTOR_SIZE(ni);
		dma_addr_t addr2 = ni->ecc_buf.phys_addr;
		uint32_t size2 = ni->sector.ecc_size;
		enum dma_data_direction dir = DMA_FROM_DEVICE;

		sect_offs = sector * NANDBCH_SECTOR_SIZE(ni);
		sect_buf = ni->buf.phys_addr + sect_offs;
		ecc_offs =
			NANDBCH_PAGE_SIZE(ni) + ni->aux.data_size +
			ni->aux.ecc_size + sector * ni->sector.ecc_size;
		addr = sect_buf;
		nc_dmaint_set(ni, bank, size, size2);
		cache_desc2_buf_for_device(ni, addr, bank, size, addr2, bank,
					   size2, dir);

		/* Enable ECC for read */
		nc_bch_read_enable(ni);

		cmd = (sector ==
			 0) ? NC_CMD_READ_ECC : NC_CMD_READ_RANDOM_ECC;
		/* Execute ucode */
		nc_exec_cmd(ni, bank, ni->uc_offset[cmd],
				     0,	/* addr */
				     (sect_offs << 16) |
					NANDBCH_SECTOR_SIZE(ni),
				     (ecc_offs << 16) |
					ni->sector.ecc_size,
				     (void *)ni->desc.phys_addr,
				     NAND_DIRECTION_FROM_NAND);

		/* Wait for completion interrupts */
		if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
			cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
			return cmd_completion_err(ni, bank, __func__, cmd);
		}
		cache_desc_buf_for_cpu(ni, addr, bank, size, dir);

		/* Disable ECC */
		nc_bch_disable(ni);

		/* Check for BCH ECC errors */
		ret = nand_bch_err_count(ni, &err_count);
		if (ret)
			return ret;

		/* Uncorrectable BCH ECC errors */
		if (err_count == -1)
			return -EPERM;

		/* Fix all correctable BCH ECC errors */
		if (err_count > 0) {
			ret = nand_bch_err_fix(ni,
					err_count,
					(uint8_t *)(ni->buf.buf + sect_offs),
					NANDBCH_SECTOR_SIZE(ni));
			if (ret)
				return ret;
		}

		/* Set the ECC stats for the sector */
		if (stats)
			stats->errs[sector] = err_count;
	}

	if (stats)
		stats->len = NANDBCH_SECTORS(ni);

	memcpy(buf, ni->buf.buf, NANDBCH_SECTOR_SIZE(ni) * NANDBCH_SECTORS(ni));
	return 0;
}

/**
 * Reads one page from nand into buf witout ECC corection
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @page:   [in] page number
 * @buf:    [out] target buffer
 */
static int nandbch_page_read_raw(struct ctrl_nand_info *ni, uint8_t bank,
				 uint32_t page, uint8_t *buf)
{
	dma_addr_t addr = ni->buf.phys_addr;
	uint32_t size = NANDBCH_PAGE_SIZE(ni);
	enum dma_data_direction dir = DMA_FROM_DEVICE;

	dev_dbg(ni->dev, "bank%u page 0x%x buf=%p\n", bank, page, buf);

	/* disable ECC */
	nc_bch_disable(ni);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_READ_PRE],
			     page,	/* addr */
			     0,	/* attr0 */
			     ni->geometry.addr_cycles,	/* attr1 */
			     NULL,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_rb(ni, bank, NANDBCH_TIMEOUT_READ)) {
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_READ_PRE);
	}
	nc_dmaint_set(ni, bank, size, 0);
	cache_desc_buf_for_device(ni, addr, bank, size, dir);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_READ], 0,	/* addr */
			     NANDBCH_PAGE_SIZE(ni),	/* attr0 */
			     0,	/* attr1 */
			     (void *)ni->desc.phys_addr,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
		cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
		return cmd_completion_err(ni, bank, __func__, NC_CMD_READ);
	}
	cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
	memcpy(buf, ni->buf.buf, NANDBCH_PAGE_SIZE(ni));
	return 0;
}

/**
 * Reads the auxiliary data from oob with BCH-ECC correction
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @page:   [in] page number
 * @buf:    [out] target buffer
 * @stats:  [out] ECC stats (number of bad bits)
 *
 * Note: If ECC correction stats are not needed caller should provide a NULL
 * pointer argument.
 */
uint32_t nandbch_aux_read_ecc(struct ctrl_nand_info *ni, uint8_t bank,
			      uint32_t page, uint8_t *buf, uint8_t *stats)
{
	int32_t err_count;
	int ret;
	dma_addr_t addr = ni->buf.phys_addr;
	uint32_t size = ni->aux.data_size;
	dma_addr_t addr2 = ni->ecc_buf.phys_addr;
	uint32_t size2 = ni->aux.ecc_size;
	enum dma_data_direction dir = DMA_FROM_DEVICE;

	dev_dbg(ni->dev, "bank%u page 0x%x buf=%p\n", bank, page, buf);

	/* disable ECC */
	nc_bch_disable(ni);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_READ_PRE],
			     page,	/* addr */
			     NANDBCH_PAGE_SIZE(ni) << 16,	/* attr0 */
			     ni->geometry.addr_cycles,	/* attr1 */
			     NULL,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_rb(ni, bank, NANDBCH_TIMEOUT_READ)) {
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_READ_PRE);
	}
	nc_dmaint_set(ni, bank, 0, size + size2);
	cache_desc2_buf_for_device(ni, addr, bank, size, addr2, bank, size2,
				   dir);

	/* Configure ECC for auxiliary data */
	bch_config_aux(ni);

	/* Enable ECC for read */
	nc_bch_read_enable(ni);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_READ], 0,	/* addr */
			     ni->aux.data_size +
				ni->aux.ecc_size,	/* attr0 */
			     0,	/* attr1 */
			     (void *)ni->desc.phys_addr,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
		cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
		return cmd_completion_err(ni, bank, __func__, NC_CMD_READ);
	}
	cache_desc_buf_for_cpu(ni, addr, bank, size, dir);

	/* disable ECC */
	nc_bch_disable(ni);

	/* check for BCH ECC errors */
	ret = nand_bch_err_count(ni, &err_count);
	if (ret)
		return ret;

	/* uncorrectable BCH ECC errors */
	if (err_count == -1)
		return -EPERM;

	/* fix all correctable BCH ECC errors */
	if (err_count > 0) {
		ret = nand_bch_err_fix(ni, err_count, (uint8_t *) ni->buf.buf,
				      ni->aux.data_size);
		if (ret)
			return ret;
	}

	/* set the ECC stats */
	if (stats)
		*stats = err_count;

	memcpy(buf, ni->buf.buf, ni->aux.data_size);
	return 0;
}

/**
 * Reads the auxiliary data from oob without ECC correction
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @page:   [in] page number
 * @buf:    [out] target buffer
 */
static int nandbch_aux_read_raw(struct ctrl_nand_info *ni, uint8_t bank,
				uint32_t page, uint8_t *buf)
{
	dma_addr_t addr = ni->buf.phys_addr;
	uint32_t size = ni->aux.data_size;
	enum dma_data_direction dir = DMA_FROM_DEVICE;

	/* disable ECC */
	nc_bch_disable(ni);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_READ_PRE],
			     page,	/* addr */
			     NANDBCH_PAGE_SIZE(ni) << 16,	/* attr0 */
			     ni->geometry.addr_cycles,	/* attr1 */
			     NULL,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_rb(ni, bank, NANDBCH_TIMEOUT_READ)) {
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_READ_PRE);
	}
	nc_dmaint_set(ni, bank, 0, size);
	cache_desc_buf_for_device(ni, addr, bank, size, dir);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_READ], 0,	/* addr */
			     ni->aux.data_size,	/* attr0 */
			     0,	/* attr1 */
			     (void *)ni->desc.phys_addr,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
		cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
		return cmd_completion_err(ni, bank, __func__, NC_CMD_READ);
	}
	cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
	memcpy(buf, ni->buf.buf, ni->aux.data_size);
	return 0;
}

/**
 * Reads the auxiliary data from oob
 * @ni:      [in] nand info structure
 * @bank:    [in] bank number
 * @page:    [in] page number
 * @buf:     [out] target buffer
 * @stats:   [out] ECC stats (number of bad bits)
 *
 * Note: If ECC correction stats are not needed caller should provide a NULL
 * pointer argument.
 */
int nandbch_aux_read(struct ctrl_nand_info *ni, uint8_t bank, uint32_t page,
		     uint8_t *buf, uint8_t *stats)
{
	int ret;

	dev_dbg(ni->dev, "bank%u page 0x%x buf=%p\n", bank, page, buf);

	if ((ni->flags & NAND_FLAG_ECC)
	    && ni->aux.ecc_t) {
		ret = nandbch_aux_read_ecc(ni, bank, page, buf, stats);
		if (ret == -EPERM) {
			/* Allow reading unprogrammed aux data */
			dev_dbg(ni->dev, "%s: ret=%d uncorrectable ecc, reading raw aux data instead\n",
				     __func__, ret);
			return nandbch_aux_read_raw(ni, bank, page, buf);
		}
		if (*stats > 40) {
			/* Allow reading unprogrammed aux data */
			dev_dbg(ni->dev, "%s: ret = %d auxerrs = %d, reading raw aux data instead\n",
				     __func__, ret, *stats);
			*stats = 0;
			return nandbch_aux_read_raw(ni, bank, page, buf);
		}
		return ret;
	}
	return nandbch_aux_read_raw(ni, bank, page, buf);
}


/**
 * Gets the status for an individual bank
 * @ni:     [in] nand info structure
 * @bank:   [in] bank to get status
 * @status: [out] nand status
 */
int nandbch_status_get(struct ctrl_nand_info *ni, uint8_t bank,
		       uint8_t *status)
{
	nc_bch_disable(ni);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_STATUS_GET],
			     0,	/* addr */
			     0,	/* attr0 */
			     0,	/* attr1 */
			     NULL,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank(ni, bank, NANDBCH_TIMEOUT_STATUS_GET)) {
		*status = ~0;
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_STATUS_GET);
	}

	*status = nc_status_result(ni);

	dev_dbg(ni->dev, "bank%u status=0x%x\n", bank, *status);

	return 0;
}

/**
 * Writes page from buf to the nand device, without ECC
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @page:   [in] page number
 * @buf:    [in] source buffer
 */
static int nandbch_page_write_raw(struct ctrl_nand_info *ni, uint8_t bank,
				  uint32_t page, uint8_t *buf)
{
	int ret;
	uint8_t status;
	dma_addr_t addr = ni->buf.phys_addr;
	uint32_t size = NANDBCH_PAGE_SIZE(ni);
	enum dma_data_direction dir = DMA_TO_DEVICE;

	dev_dbg(ni->dev, "bank%u page 0x%x, buf=%p\n", bank, page, buf);
	memcpy(ni->buf.buf, buf, NANDBCH_PAGE_SIZE(ni));

	/* disable ECC */
	nc_bch_disable(ni);
	nc_dmaint_set(ni, bank, size, 0);
	cache_desc_buf_for_device(ni, addr, bank, size, dir);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_WRITE_PRE],
			     page,	/* addr */
			     NANDBCH_PAGE_SIZE(ni),	/* attr0 */
			     ni->geometry.addr_cycles,	/* attr1 */
			     (void *)ni->desc.phys_addr,	/* prdbase */
			     NAND_DIRECTION_TO_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
		cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_WRITE_PRE);
	}
	cache_desc_buf_for_cpu(ni, addr, bank, size, dir);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_WRITE_COMPLETE],
			     0,	/* addr */
			     0,	/* attr0 */
			     0,	/* attr1 */
			     NULL,	/* prdbase */
			     NAND_DIRECTION_TO_NAND);
	/* Wait for completion interrupts */
	if (wait_ucode_bank_rb(ni, bank, NANDBCH_TIMEOUT_WRITE)) {
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_WRITE_COMPLETE);
	}

	do {
		ret = nandbch_status_get(ni, bank, &status);
		if (ret)
			return ret;
	} while (!(status & NAND_STATUS_READY));

	return ((status & NAND_STATUS_FAIL_N1) ? NAND_STATUS_FAIL : 0);
}

/**
 * Erase block ignoring bad block marks
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @block:  [in] block number
 */
int nandbch_block_force_erase(struct ctrl_nand_info *ni, uint8_t bank,
			      uint32_t block)
{
	int ret;
	uint8_t status;

	dev_dbg(ni->dev, "bank%u block=0x%x\n", bank, block);

	/* disable ECC */
	nc_bch_disable(ni);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_BLOCK_ERASE],
			     block << (NANDBCH_BLOCK_SHIFT(ni) -
				NANDBCH_PAGE_SHIFT(ni)),	/* addr */
			     0,	/* attr0 */
			     ni->geometry.addr_cycles - 2,	/* attr1 */
			     NULL,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_rb(ni, bank, NANDBCH_TIMEOUT_ERASE)) {
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_BLOCK_ERASE);
	}

	do {
		ret = nandbch_status_get(ni, bank, &status);
		if (ret)
			return ret;
	} while (!(status & NAND_STATUS_READY));

	return ((status & NAND_STATUS_FAIL_N1) ? NAND_STATUS_FAIL : 0);

}


/**
 * erase a block
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @block:  [in] block number
 */
int nandbch_block_erase(struct ctrl_nand_info *ni, uint8_t bank,
			uint32_t block)
{
	int ret;
	bool isbad;

	dev_dbg(ni->dev, "bank%u block=0x%x\n", bank, block);

	ret = nandbch_block_isbad(ni, bank, block, &isbad);
	if (ret)
		return ret;
	if (isbad)
		return -EPERM;

	return nandbch_block_force_erase(ni, bank, block);
}

/**
 * Writes the auxiliary data to oob without ECC
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @page:   [in] page number
 * @buf:    [in] source buffer
 */
int nandbch_aux_write_raw(struct ctrl_nand_info *ni, uint8_t bank,
			  uint32_t page, uint8_t *buf)
{
	int ret;
	uint8_t status;
	dma_addr_t addr = ni->buf.phys_addr;
	uint32_t size = ni->aux.data_size;
	enum dma_data_direction dir = DMA_TO_DEVICE;

	memcpy(ni->buf.buf, buf, ni->aux.data_size);

	dev_dbg(ni->dev, "bank%u page 0x%x, buf=%p\n", bank, page, buf);

	/* disable ECC */
	nc_bch_disable(ni);
	nc_dmaint_set(ni, bank, 0, size);
	cache_desc_buf_for_device(ni, addr, bank, size, dir);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_WRITE_PRE],
			     page,	/* addr */
			     (NANDBCH_PAGE_SIZE(ni) << 16) |
				ni->aux.data_size,	/* attr0 */
			     ni->geometry.addr_cycles,	/* attr1 */
			     (void *)ni->desc.phys_addr,	/* prdbase */
			     NAND_DIRECTION_TO_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
		cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_WRITE_PRE);
	}
	cache_desc_buf_for_cpu(ni, addr, bank, size, dir);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_WRITE_COMPLETE],
			     0,	/* addr */
			     0,	/* attr0 */
			     0,	/* attr1 */
			     NULL,	/* prdbase */
			     NAND_DIRECTION_TO_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_rb(ni, bank, NANDBCH_TIMEOUT_WRITE)) {
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_WRITE_COMPLETE);
	}

	do {
		ret = nandbch_status_get(ni, bank, &status);
		if (ret)
			return ret;
	} while (!(status & NAND_STATUS_READY));

	return ((status & NAND_STATUS_FAIL_N1) ? NAND_STATUS_FAIL : 0);
}


/**
 * Erase block and write the bad block marker in oob
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @block:  [in] block number
 */
#define	TOTAL_PAGES_TO_MARK	3
int nandbch_block_markbad(struct ctrl_nand_info *ni, uint8_t bank,
			  uint32_t block)
{
	bool isbad;
	uint32_t i, page;
	uint32_t off[TOTAL_PAGES_TO_MARK];
	int ret;

	dev_dbg(ni->dev, "bank%u block=0x%x\n", bank, block);

	/* erase block first */
	nandbch_block_force_erase(ni, bank, block);

	/* mark first second and last page of the block */
	off[0] = 0;
	off[1] = 1;
	off[2] = (1 << (ni->geometry.block_shift -
						ni->geometry.page_shift)) - 1;

	for (i = 0; i < ni->aux.data_size; i++)
		nand_aux_buf[i] = 0xff;

	nand_aux_buf[NAND_BADBLOCK_MARKER_POS] = NAND_BADBLOCK_MARKER_BAD;

	page = block << (ni->geometry.block_shift - ni->geometry.page_shift);

	for (i = 0; i < TOTAL_PAGES_TO_MARK; i++)
		nandbch_aux_write_raw(ni, bank, page + off[i], nand_aux_buf);

	ret = nandbch_block_isbad(ni, bank, block, &isbad);
	if (ret || !isbad)
		return -EINVAL;

	return 0;
}

/**
 * Writes page from buf to the nand device with BCH_ECC
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @page:   [in] page number
 * @buf:    [in] source buffer
 */
int nandbch_page_write_ecc(struct ctrl_nand_info *ni, uint8_t bank,
			   uint32_t page, uint8_t *buf)
{
	uint32_t cmd;
	uint32_t sector;
	uint32_t sect_offs;
	uint32_t ecc_offs;
	dma_addr_t sect_buf;
	uint8_t status;
	int ret;
	dma_addr_t addr0 = ni->buf.phys_addr;
	uint32_t size0 = NANDBCH_PAGE_SIZE(ni);
	enum dma_data_direction dir = DMA_TO_DEVICE;

	memcpy(ni->buf.buf, buf, NANDBCH_PAGE_SIZE(ni));
	dev_dbg(ni->dev, "bank%u page 0x%x, buf=%p\n", bank, page, buf);

	/* disable ECC */
	nc_bch_disable(ni);

	/* Configure ECC for main data */
	bch_config_data(ni);

	/* Perform cache management for page buffer */
	cache_buf(for_device, ni, addr0, size0, dir);
	for (sector = 0; sector < NANDBCH_SECTORS(ni); sector++) {
		dma_addr_t addr;
		uint32_t size = NANDBCH_SECTOR_SIZE(ni);
		enum dma_data_direction dir = DMA_TO_DEVICE;

		sect_buf = ni->buf.phys_addr + sector * NANDBCH_SECTOR_SIZE(ni);
		sect_offs = sector * NANDBCH_SECTOR_SIZE(ni);
		ecc_offs =
			NANDBCH_PAGE_SIZE(ni) + ni->aux.data_size +
			ni->aux.ecc_size + sector * ni->sector.ecc_size;
		addr = sect_buf;
		nc_dmaint_set(ni, bank, size, 0);
		cache_desc_for_device(ni, addr, bank, size);

		/* Enable ECC for write */
		nc_bch_write_enable(ni);
		cmd =
			(sector ==
			 0) ? NC_CMD_WRITE_PRE : NC_CMD_WRITE_RANDOM;

		/* Execute ucode */
		nc_exec_cmd(ni, bank, ni->uc_offset[cmd], page,	/* addr */
				     (sect_offs << 16) |
					NANDBCH_SECTOR_SIZE(ni), /* attr0 */
				     ni->geometry.addr_cycles,	/* attr1 */
				     (void *)ni->desc.phys_addr, /* prdbase */
				     NAND_DIRECTION_TO_NAND);

		/* Wait for completion interrupts */
		if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
			cache_desc(for_cpu, ni);
			return cmd_completion_err(ni, bank, __func__, cmd);
		}
		cache_desc(for_cpu, ni);

		/* disable ECC */
		nc_bch_disable(ni);

		/* retrieve ECC bytes */
		ret = nand_bch_ecc_copy(ni, ni->ecc_buf.buf,
					ni->sector.ecc_bytes,
					ni->sector.ecc_size);
		if (ret)
			return ret;

		addr = ni->ecc_buf.phys_addr;
		size = ni->sector.ecc_size;
		dir = DMA_TO_DEVICE;
		nc_dmaint_set(ni, bank, 0, size);
		cache_desc_buf_for_device(ni, addr, bank, size, dir);

		/* Execute ucode */
		nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_WRITE_RANDOM],
				     0,	/* addr */
				     (ecc_offs << 16) |
					ni->sector.ecc_size,	/* attr0 */
				     0,	/* attr1 */
				     (void *)ni->desc.phys_addr, /* prdbase */
				     NAND_DIRECTION_TO_NAND);

		/* Wait for completion interrupts */
		if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
			cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
			return cmd_completion_err(ni, bank, __func__,
						    NC_CMD_WRITE_RANDOM);
		}
		cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
	}			/* for each sector of the page */
	cache_buf(for_cpu, ni, addr0, size0, dir);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_WRITE_COMPLETE],
			     0,	/* addr */
			     0,	/* attr0 */
			     0,	/* attr1 */
			     NULL,	/* prdbase */
			     NAND_DIRECTION_TO_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_rb(ni, bank, NANDBCH_TIMEOUT_WRITE)) {
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_WRITE_COMPLETE);
	}

	do {
		ret = nandbch_status_get(ni, bank, &status);
		if (ret)
			return ret;
	} while (!(status & NAND_STATUS_READY));

	return ((status & NAND_STATUS_FAIL_N1) ? NAND_STATUS_FAIL : 0);

}


/**
 * Writes auxiliary data from buf to the nand device with BCH_ECC
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @page:   [in] page number
 * @buf:    [in] source buffer
 */
static int nandbch_aux_write_ecc(struct ctrl_nand_info *ni, uint8_t bank,
				 uint32_t page, uint8_t *buf)
{
	uint32_t aux_offs;
	int ret;
	uint8_t status;
	dma_addr_t addr = ni->buf.phys_addr;
	uint32_t size = ni->aux.data_size;
	enum dma_data_direction dir = DMA_TO_DEVICE;

	memcpy(ni->buf.buf, buf, size);

	dev_dbg(ni->dev, "bank%u page 0x%x, buf=%p\n", bank, page, buf);

	/* disable ECC */
	nc_bch_disable(ni);

	aux_offs = NANDBCH_PAGE_SIZE(ni);
	nc_dmaint_set(ni, bank, 0, size);
	cache_desc_buf_for_device(ni, addr, bank, size, dir);

	/* Configure ECC for auxiliary data */
	bch_config_aux(ni);

	/* Enable ECC for write */
	nc_bch_write_enable(ni);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_WRITE_PRE],
			page,	/* addr */
			(aux_offs << 16) | ni->aux.data_size,	/* attr0 */
			ni->geometry.addr_cycles,	/* attr1 */
			(void *)ni->desc.phys_addr,	/* prdbase */
			NAND_DIRECTION_TO_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
		cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_WRITE_PRE);
	}
	cache_desc_buf_for_cpu(ni, addr, bank, size, dir);

	/* disable ECC */
	nc_bch_disable(ni);

	/* retrieve ECC bytes */
	ret = nand_bch_ecc_copy(ni, ni->ecc_buf.buf, ni->aux.ecc_bytes,
			       ni->aux.ecc_size);
	if (ret)
		return ret;

	addr = ni->ecc_buf.phys_addr;
	size = ni->aux.ecc_size;
	dir = DMA_TO_DEVICE;
	nc_dmaint_set(ni, bank, 0, size);
	cache_desc_buf_for_device(ni, addr, bank, size, dir);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_WRITE], 0,	/* addr */
			     ni->aux.ecc_size,	/* attr0 */
			     0,	/* attr1 */
			     (void *)ni->desc.phys_addr,	/* prdbase */
			     NAND_DIRECTION_TO_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
		cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
		return cmd_completion_err(ni, bank, __func__, NC_CMD_WRITE);
	}
	cache_desc_buf_for_cpu(ni, addr, bank, size, dir);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_WRITE_COMPLETE],
			     0,	/* addr */
			     0,	/* attr0 */
			     0,	/* attr1 */
			     NULL,	/* prdbase */
			     NAND_DIRECTION_TO_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_rb(ni, bank, NANDBCH_TIMEOUT_WRITE)) {
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_WRITE_COMPLETE);
	}

	do {
		ret = nandbch_status_get(ni, bank, &status);
		if (ret)
			return ret;
	} while (!(status & NAND_STATUS_READY));

	return ((status & NAND_STATUS_FAIL_N1) ? NAND_STATUS_FAIL : 0);
}

/**
 * get ONFI feature
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @feature:[in] feature address
 * @buf:    [out] buffer to get the feature value (4 bytes)
 */
int nandbch_feature_get(struct ctrl_nand_info *ni, uint8_t bank,
			uint8_t feature, uint8_t *buf)
{
	dma_addr_t addr = ni->buf.phys_addr;
	uint32_t size = NANDBCH_FEATURE_SIZE;
	enum dma_data_direction dir = DMA_FROM_DEVICE;

	dev_dbg(ni->dev, "bank%u feature 0x%x, buf=%p\n", bank, feature,
	       buf);

	/* disable ECC */
	nc_bch_disable(ni);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_GET_FEATURE_PRE],
			     feature,	/* addr */
			     0,	/* attr0 */
			     0,	/* attr1 */
			     0,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_rb(ni, bank, NANDBCH_TIMEOUT_FEATURE)) {
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_GET_FEATURE_PRE);
	}
	nc_dmaint_set(ni, bank, 0, size);
	cache_desc_buf_for_device(ni, addr, bank, size, dir);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_READ], 0,	/* addr */
			     NANDBCH_FEATURE_SIZE,	/* attr0 */
			     0,	/* attr1 */
			     (void *)ni->desc.phys_addr,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
		cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
		return cmd_completion_err(ni, bank, __func__, NC_CMD_READ);
	}
	cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
	memcpy(buf, ni->buf.buf, NANDBCH_FEATURE_SIZE);
	return 0;
}

/**
 * set ONFI feature
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @feature:[in] feature address
 * @buf:    [out] buffer to get the feature value (4 bytes)
 */
static int nandbch_feature_set(struct ctrl_nand_info *ni, uint8_t bank,
			       uint8_t feature, uint8_t *buf)
{
	dma_addr_t addr = ni->buf.phys_addr;
	uint32_t size = NANDBCH_FEATURE_SIZE;
	enum dma_data_direction dir = DMA_TO_DEVICE;

	memcpy(ni->buf.buf, buf, NANDBCH_FEATURE_SIZE);
	nc_dmaint_set(ni, bank, 0, size);
	cache_desc_buf_for_device(ni, addr, bank, size, dir);

	dev_dbg(ni->dev, "bank%u feature 0x%x, buf=%p\n", bank, feature,
	       buf);

	/* disable ECC */
	nc_bch_disable(ni);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_GET_FEATURE_PRE],
			     feature,	/* addr */
			     0,	/* attr0 */
			     0,	/* attr1 */
			     (void *)ni->desc.phys_addr,	/* prdbase */
			     NAND_DIRECTION_TO_NAND);

	/* Wait for completion interrupts */
	if (wait_for_ucode_completion(ni, bank,
				      (NAND_IRQSTATUS_DMA_CMPL_IRQ_MASK |
				       NANDBCH_IRQ_BANK_RB(bank)),
				      NANDBCH_TIMEOUT_FEATURE)) {
		cache_desc_buf_for_device(ni, addr, bank, size, dir);
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_GET_FEATURE_PRE);
	}
	cache_desc_buf_for_device(ni, addr, bank, size, dir);
	return 0;
}

/**
 * Reads used oob bytes (aux data, aux data ECC bytes and main data ECC bytes)
 * from nand into buf without ECC correction
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @page:   [in] page number
 * @buf:    [out] target buffer
 */
int nandbch_oob_read(struct ctrl_nand_info *ni, uint8_t bank,
		     uint32_t page, uint8_t *buf)
{
	dma_addr_t addr = ni->buf.phys_addr;
	uint32_t size =
		ni->aux.data_size + ni->aux.ecc_size +
		NANDBCH_SECTORS(ni) * ni->sector.ecc_size;
	enum dma_data_direction dir = DMA_FROM_DEVICE;

	uint32_t len, chunk, aux_type_size;

	cache_buf(for_device, ni, addr, size, dir);

	/* disable ECC */
	nc_bch_disable(ni);

	/* Execute ucode */
	nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_READ_PRE],
			     page,	/* addr */
			     NANDBCH_PAGE_SIZE(ni) << 16,	/* attr0 */
			     ni->geometry.addr_cycles,	/* attr1 */
			     NULL,	/* prdbase */
			     NAND_DIRECTION_FROM_NAND);

	/* Wait for completion interrupts */
	if (wait_ucode_bank_rb(ni, bank, NANDBCH_TIMEOUT_READ)) {
		return cmd_completion_err(ni, bank, __func__,
					    NC_CMD_WRITE_COMPLETE);
	}

	/*
	 * Read OOB bytes in several chunks because of the size limitation
	 * imposed by DMAINT register (HWCAPRI-385)
	 */

	/* round up size to multiple of auxiliary data type */
	aux_type_size = nc_get_aux_data_type_size(ni);
	size = ((size + aux_type_size - 1) / aux_type_size) * aux_type_size;
	chunk = 0x10 * aux_type_size;
	while (size) {
		if (size > chunk)
			len = chunk;
		else
			len = size;

		/* Setup DMA descriptor */
		nc_dmaint_set(ni, bank, 0, len);
		cache_desc_for_device(ni, addr, bank, size);
		size -= len;
		buf += len;

		/* Execute ucode */
		nc_exec_cmd(ni, bank, ni->uc_offset
				     [NC_CMD_READ], 0,	/* addr */
				     len,	/* attr0 */
				     0,	/* attr1 */
				     (void *)ni->desc.phys_addr, /* prdbase */
				     NAND_DIRECTION_FROM_NAND);

		/* Wait for completion interrupts */
		if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
			return cmd_completion_err(ni, bank, __func__,
						  NC_CMD_READ);
		}
	}

	cache_buf(for_cpu, ni, addr, size, dir);
	memcpy(buf, ni->buf.buf, size);
	return 0;
}

/**
 * checks if block is marked bad
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @block:  [in] block number
 * @is_bad: [out] true if block is marked bad, false if good
 */
int nandbch_block_isbad(struct ctrl_nand_info *ni, uint8_t bank,
			uint32_t block, bool *is_bad)
{
	uint32_t i, page;
	uint32_t off[3];
	int ret;

	/* check first second and last page of the block */
	off[0] = 0;
	off[1] = 1;
	off[2] = (1 << (ni->geometry.block_shift -
			 ni->geometry.page_shift)) - 1;

	*is_bad = false;
	page = block << (ni->geometry.block_shift - ni->geometry.page_shift);

	for (i = 0; i < 3; i++) {

		ret = nandbch_aux_read_raw(ni, bank, page + off[i],
					  nand_aux_buf);

		if (ret) {
			*is_bad = true;
			break;
		}

		if (nand_aux_buf[NAND_BADBLOCK_MARKER_POS] !=
		    NAND_BADBLOCK_MARKER_GOOD) {
			*is_bad = true;
			break;
		}
	}
	if (*is_bad) {
		dev_dbg(ni->dev, "bank%u block 0x%x isbad=%d\n", bank, block,
			     *is_bad);
	}

	return ret;
}


/**
 * Reads one page from nand into buf
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @page:   [in] page number
 * @buf:    [out] target buffer
 * @stats:  [out] ECC stats (number of bad bits per sector)
 *
 * Note: If the caller wants ECC correction stats it needs to provide a pointer
 * to a nandbch_eccstats structure. The number of bits corrected
 * by ECC in every sector will be returned in the corresponding array element.
 * If stats are not needed caller should provide a NULL pointer argument.
 */
int nandbch_page_read(struct ctrl_nand_info *ni, uint8_t bank, uint32_t page,
		      uint8_t *buf, struct nandbch_eccstats *stats)
{
	int ret;
	int dataerrs;
	int i;

	if (ni->flags & NAND_FLAG_ECC) {
		ret = nandbch_page_read_ecc(ni, bank, page, buf, stats);
		if (ret == -EPERM)
			/* Allow reading unprogrammed pages */
			return nandbch_page_read_raw(ni, bank, page, buf);

		dataerrs = 0;
		for (i = 0; i < stats->len; i++)
			dataerrs += stats->errs[i];

		if (dataerrs > 40) {
			/* Allow reading unprogrammed pages */
			/* Clear bogus stats by setting returned length to 0 */
			stats->len = 0;
			dev_dbg(ni->dev, "ret = %d dataerrs=%u, reading raw pages instead\n",
				     ret, dataerrs);
			return nandbch_page_read_raw(ni, bank, page, buf);
		}
		return ret;
	}
	return nandbch_page_read_raw(ni, bank, page, buf);
}

/**
 * nand_config(ni, bank, addr_cycles, page, incr, iter) - Configure nand
 * interface based on parameters stored on the NAND flash
 * @ni:     [out] nand info structure
 * @bank:   [in] bank number
 * @addr_cycles:   [in] address cycles
 * @page:   [in/out] page number
 * @incr:   [in] page increment
 * @iter:   [in] mamimum number of iterations
 *
 * Attempts to retrieve NAND configuration parameters from the specified bank
 * and page. If unsuccessful, it repeats the attempt up to the maximum number
 * specified in the <iter> argument adding <incr> to the page number at each
 * attempt. When it finds a valid config page, it updates the page number.
 *
 * The configuration page has the following information:
 * (LSB at lower offset)
 *
 * Bytes        Content
 * 0-3          "NAND" signature
 * 4-7          Version number
 * 8-11         Page size
 * 12-15        ECC offset
 * 16-17        Sector size
 * 18           BCH ECC correction requirement (bits/sector)
 *
 * Additional parameters not used by BOOTROM:
 *
 * Bytes        Content
 * 19           Auxiliary data BCH ECC correction requirement
 * 20-23        Block size
 * 24-27        Bank size (in MB)
 * 28           Timing config selection
 *                      0x00    default (ONFI timing mode 0)
 *                      0x01    use ONFI timing mode from byte 29 and send
 *                              ONFI SET_FEATURE command to device to change
 *                              timing mode
 *                      0x02    use ONFI timing mode from byte 29 only
 *                      0x03    use NAND controller timing config register
 *                              values
 * 29           ONFI timing mode [0..5]
 * 30-45        NAND controller timing config register values
 * 46-47        OOB size
 * 48-511       Reserved
 */
int nandbch_config(struct ctrl_nand_info *ni, uint8_t bank,
		   uint8_t addr_cycles, uint32_t *page, uint32_t incr,
		   uint32_t iter)
{
	int ret;
	uint32_t i, p;
	uint32_t uc_offset;
	uint32_t page_size, sect_size, ecc_offs;
	int32_t err_count;
	uint8_t ecc_t;
	uint32_t block_size, bank_size, aux_type_size, aux_ecc_bytes, oob_size;
	uint8_t aux_ecc_t;

	dev_dbg(ni->dev, "%s: bank%u cycles %u page 0x%x incr %u, iter %u\n",
	       __func__, bank, addr_cycles, *page, incr, iter);

	if ((ni == NULL) || (page == NULL))
		return -EFAULT;

	if ((addr_cycles != 4) && (addr_cycles != 5))
		return -EFAULT;

	ni->geometry.bus_width = 8;
	ni->geometry.addr_cycles = addr_cycles;

	/* reset the NAND controller and set normal config (DMA mode; ECC off)*/
	nc_reset(ni);

	/* setup bus width to 8 bit and aux data type to 4 byte */
	nand_writereg(ni, NAND_CONFIG0_OFFSET,
			(((NAND_CONFIG0_AUX_DATA_TYPE_4B <<
			       NAND_CONFIG0_AUX_DATA_TYPE_SHIFT) &
		NAND_CONFIG0_AUX_DATA_TYPE_MASK) |
	       ((NAND_CONFIG0_DATA_WIDTH_8 << NAND_CONFIG0_DATA_WIDTH_SHIFT) &
		NAND_CONFIG0_DATA_WIDTH_MASK)));

	/* configure ONFI timing mode 0 */
	ni->timing.mode = 0;
	nand_init_timings_onfi(ni);
	nand_writereg(ni, NAND_CONFIG1_OFFSET, ni->timing.conf1);
	nand_writereg(ni, NAND_CONFIG2_OFFSET, ni->timing.conf2);

	/* load ucode programs */
	uc_offset = 0;

	for (i = 0; i < NC_CMD_MAX; i++) {
		ni->uc_offset[nc_cmds[i].cmd] =
			nc_load_cmd_seq(ni, &uc_offset, nc_cmds[i].cmd_seq,
					     nc_cmds[i].num_seq);
	}

	/* check for ucode memory overflow */
	if (nc_ucode_buf_overflow(ni, uc_offset))
		return -ENOMEM;

	ret = nandbch_reset_device(ni, bank);
	if (ret)
		return ret;

	/* Configure ECC to maximum supported */
	nc_bch_config(ni, 8, NANDBCH_CFG_ECC_N, NANDBCH_CFG_ECC_K,
			     NANDBCH_CFG_ECC_T);

	/* Search for configuration page */
	p = *page;
	for (i = 0; i < iter; i++, p += incr) {
		dma_addr_t addr = ni->param_buf.phys_addr;
		uint32_t size = 512;
		dma_addr_t addr2 = ni->ecc_buf.phys_addr;
		uint32_t size2 = NANDBCH_CFG_ECC_SIZE;
		enum dma_data_direction dir = DMA_FROM_DEVICE;

		/* disable ECC */
		nc_bch_disable(ni);

		/* Execute ucode */
		nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_READ_PRE],
				     p, /* addr */
				     0,	/* attr0 */
				     ni->geometry.addr_cycles, /* attr1 */
				     NULL,	/* prdbase */
				     NAND_DIRECTION_FROM_NAND);

		/* Wait for completion interrupts */
		if (wait_ucode_bank_rb(ni, bank, NANDBCH_TIMEOUT_READ)) {
			return cmd_completion_err(ni, bank, __func__,
						    NC_CMD_READ_PRE);
		}
		nc_dmaint_set(ni, bank, size, size2);
		cache_desc2_buf_for_device(ni, addr, bank, size, addr2, bank,
					   size2, dir);

		/* Enable ECC for read */
		nc_bch_read_enable(ni);

		/* Execute ucode */
		nc_exec_cmd(ni, bank, ni->uc_offset[NC_CMD_READ], 0, /* addr */
				     512 + NANDBCH_CFG_ECC_SIZE, /* attr0 */
				     0,	/* attr1 */
				     (void *)ni->desc.phys_addr, /* prdbase */
				     NAND_DIRECTION_FROM_NAND);

		/* Wait for completion interrupts */
		if (wait_ucode_bank_dma(ni, bank, NANDBCH_TIMEOUT_DMA)) {
			cache_desc_buf_for_cpu(ni, addr, bank, size, dir);
			return cmd_completion_err(ni, bank, __func__,
						    NC_CMD_READ);
		}
		cache_desc_buf_for_cpu(ni, addr, bank, size, dir);

		/* disable ECC */
		nc_bch_disable(ni);

		/* Check for BCH ECC errors */
		ret = nand_bch_err_count(ni, &err_count);
		if (ret)
			return ret;

		/* Uncorrectable BCH ECC errors */
		if (err_count == -1) {
			/* not a configuration page */
			dev_dbg(ni->dev, "NAND table not found: page %d\n", p);
			continue;
		}

		/* Fix all correctable BCH ECC errors */
		if (err_count > 0) {
			ret = nand_bch_err_fix(ni, err_count, ni->param_buf.buf,
					      512);
			if (ret)
				return ret;
		}
		if ((ni->param_buf.buf[0] == 'N')
		    && (ni->param_buf.buf[1] == 'A')
		    && (ni->param_buf.buf[2] == 'N')
		    && (ni->param_buf.buf[3] == 'D')) {

			/* configuration page found */
			*page = p;
			break;
		}
	}

	if (i == iter) {
		/* config data not found */
		return -ENODEV;
	}

	/* use configuration parameters */
	page_size = get_unaligned_le32(&ni->param_buf.buf[8]);
	ecc_offs = get_unaligned_le32(&ni->param_buf.buf[12]);
	sect_size = get_unaligned_le16(&ni->param_buf.buf[16]);
	ecc_t = ni->param_buf.buf[18];

	ni->geometry.page_shift = shift_of(page_size);
	ni->sector.data_size = sect_size;
	ni->sector.ecc_t = ecc_t;
	ni->aux.data_size = ecc_offs - page_size;
	ni->aux.ecc_t = 0;

	aux_ecc_t = ni->param_buf.buf[19];
	block_size = get_unaligned_le32(&ni->param_buf.buf[20]);
	bank_size = get_unaligned_le32(&ni->param_buf.buf[24]);
	oob_size = get_unaligned_le16(&ni->param_buf.buf[46]);
	if ((block_size > 0) && (block_size != 0xffffffff))
		ni->geometry.block_shift = shift_of(block_size);

	if ((bank_size > 0) && (bank_size != 0xffffffff))
		ni->geometry.bank_shift = shift_of(bank_size) + 20;

	if ((oob_size > 0) && (oob_size != 0xFFFF))
		ni->geometry.oob_size = oob_size;

	if (ni->param_buf.buf[28] != 0xff) {
		ni->timing.select = ni->param_buf.buf[28];
		switch (ni->timing.select) {
		case NANDBCH_TIMING_SELECT_DEFAULT:
			/* nothing to do */
			break;
		case NANDBCH_TIMING_SELECT_TMODE_SET_FEATURE:
		case NANDBCH_TIMING_SELECT_TMODE_ONLY:
			ni->flags |= NAND_FLAG_TIMING;
			ni->timing.mode = ni->param_buf.buf[29];
			break;
		case NANDBCH_TIMING_SELECT_REG_CAPRI:
			ni->timing.conf1 =
				get_unaligned_le32(&ni->param_buf.buf[30]);
			ni->timing.conf2 =
				get_unaligned_le32(&ni->param_buf.buf[34]);
			break;
		default:
			dev_dbg(ni->dev, "NAND config: ignoring invalid timing selection %d\n",
				  ni->timing.select);
			ni->timing.select =
				NANDBCH_TIMING_SELECT_DEFAULT;
		}
	}
	if ((aux_ecc_t > 0) && (aux_ecc_t != 0xff)) {

		/*
		 * The aux data size in DMAINT register has only 5 bits.
		 * By default, the aux type is set to 2 bytes, so max aux
		 * data size or ecc size with this config is 31*2
		 */
		aux_type_size = 2;

		/* Check the main data ECC size */
		if ((14 * ecc_t + 7) / 8 > 62) {
			/* set auxiliary data type to 4 bytes */
			aux_type_size = 4;
		}

		/* Calculate the AUX data ECC bytes */
		aux_ecc_bytes = (14 * aux_ecc_t + 7) / 8;

		/* round up aux ecc bytes to multiple of aux data type size */
		aux_ecc_bytes =
			((aux_ecc_bytes + aux_type_size -
			  1) / aux_type_size) * aux_type_size;
		if ((ni->aux.data_size % aux_type_size == 0)
		    && (ni->aux.data_size > aux_ecc_bytes)) {
			ni->aux.ecc_t = aux_ecc_t;
			ni->aux.data_size -= aux_ecc_bytes;
		} else {
			dev_dbg(ni->dev, "NAND config: ignoring invalid auxiliary data ecc configuration\n");
		}
	}
	dev_dbg(ni->dev, "NAND table found: page %d addr 0x%08X\n",
			     p, p << ni->geometry.page_shift);

	return 0;
}

/**
 * Initialize NAND interface
 * @ni:     [in/out] nand info structure
 * @flags:  [in] options
 *
 * By default it configures parameters in struct ctrl_nand_info based on ONFI
 * parameter page information.
 * Default values for the aux_size is 2 bytes / 512 bytes of main data and no
 * ECC correction is enabled for auxiliary data in the OOB.
 * Calling nandbch_init with the NAND_FLAG_GEOMETRY flag will force the use of
 * the parameters passed in the geometry structure (minimum the bus_width and
 * page_shift need to be set for NAND read only access).
 * Calling nandbch_init with the NAND_FLAG_ECC_CONFIG flag will force the use of
 * the ECC parameters passed in the ecc structure (sector data_size and ecc_t
 * need to be set).
 * Calling nandbch_init with the NAND_FLAG_AUX_CONFIG flag will force the use of
 * the auxiliary data parameters passed in the aux structure (aux data_size and
 * ecc_t need to be set).
 * Calling nandbch_init with the NAND_FLAG_TIMING flag will force the use of the
 * timing parameters passed in the timing structure.
 */
int nandbch_init(struct ctrl_nand_info *ni, uint8_t flags)
{
	uint32_t uc_offset;
	uint32_t bank;
	int ret;
	uint32_t i;
	uint32_t aux_type_size;
	uint32_t b;
	uint32_t buf;
	uint8_t *feat = (uint8_t *) &buf;

	if (ni == NULL)
		return -EFAULT;

	ni->flags = flags;

	/*
	 * reset the NAND controller
	 * and set normal config (DMA mode; ECC off)
	 */
	nc_reset(ni);

	/* setup bus width to 8 bit for ID stage */
	nand_writereg(ni, NAND_CONFIG0_OFFSET, NAND_CONFIG0_NORMAL);

	/* configure ONFI timing mode 0 for ID stage */
	ni->timing.mode = 0;
	nand_init_timings_onfi(ni);
	nand_writereg(ni, NAND_CONFIG1_OFFSET, ni->timing.conf1);
	nand_writereg(ni, NAND_CONFIG2_OFFSET, ni->timing.conf2);

	/* load ucode programs */
	uc_offset = 0;

	for (i = 0; i < NC_CMD_MAX; i++) {
		ni->uc_offset[nc_cmds[i].cmd] =
			nc_load_cmd_seq(ni, &uc_offset, nc_cmds[i].cmd_seq,
					     nc_cmds[i].num_seq);
	}

	/* check for ucode memory overflow */
	if (nc_ucode_buf_overflow(ni, uc_offset))
		return -ENOMEM;

	/* discover bank and configuration */
	for (bank = 0; bank < NANDBCH_BANKS_MAX; bank++) {
		ret = nandbch_reset_device(ni, bank);
		if (ret)
			break;
		ret = nandbch_id_get(ni, bank, NANDBCH_JEDEC_ID_ADDR,
					    NANDBCH_JEDEC_ID_SIZE,
					    nand_id_buf);
		if (ret)
			break;

		/* save ID of the first bank --- they all MUST be the same */
		if (bank == 0) {
			for (i = 0; i < NANDBCH_JEDEC_ID_SIZE; i++)
				ni->id[i] = nand_id_buf[i];

		} else {
			/* all NAND chips must be the same */
			for (i = 0; i < NANDBCH_JEDEC_ID_SIZE; i++) {
				if (ni->id[i] != nand_id_buf[i])
					return -EINVAL;
			}
		}
	}

	if (bank == 0)
		return ret;

	ni->geometry.banks = bank;

	if ((ni->flags & NAND_FLAG_GEOMETRY) == 0) {
		/* Configure NAND geometry for ONFI devices */
		ret = nand_config_onfi(ni);
		if (ret)
			return ret;
	}
	/* Configure timing parameters */
	if (ni->flags & NAND_FLAG_TIMING) {
		switch (ni->timing.select) {
		case NANDBCH_TIMING_SELECT_DEFAULT:
			/* nothing to do */
		break;
		case NANDBCH_TIMING_SELECT_TMODE_SET_FEATURE:
			/*
			 * send SET_FEATURE command to
			 * change timing mode on the NAND device
			 */
			feat[0] = ni->timing.mode;
			feat[1] = 0;
			feat[2] = 0;
			feat[3] = 0;
			for (b = 0; b < ni->geometry.banks; b++) {
				ret = nandbch_feature_set(ni, b, 0x01, feat);
				if (ret) {
					dev_dbg(ni->dev, "Fail to set ONFI timing mode %d on bank %d\n",
						    ni->timing.mode, b);
					return ret;
				}
			}
		/* fall through */
		case NANDBCH_TIMING_SELECT_TMODE_ONLY:
			nand_init_timings_onfi(ni);
			nand_writereg(ni, NAND_CONFIG1_OFFSET,
				      ni->timing.conf1);
			nand_writereg(ni, NAND_CONFIG2_OFFSET,
				      ni->timing.conf2);
		break;
		case NANDBCH_TIMING_SELECT_REG_CAPRI:
			/* set timing parameters */
			nand_writereg(ni, NAND_CONFIG1_OFFSET,
				      ni->timing.conf1);
			nand_writereg(ni, NAND_CONFIG2_OFFSET,
				      ni->timing.conf2);
		break;
		default:
			dev_dbg(ni->dev, "NAND init: ignore invalid timing select %d\n",
			       ni->timing.select);
			ni->timing.select = 0;
		}
	}
	/* configure timing mode */
	if (ni->geometry.bus_width == 16)
		return -EPERM; /* 16 bit NAND not supported */

	if ((ni->geometry.addr_cycles != 4) &&
	    (ni->geometry.addr_cycles != 5))
		/* only 4 and 5 address cycles supported */
		return -EPERM;

	if (ni->sector.ecc_t == 0)
		/* disable ECC if not required */
		ni->flags &= ~NAND_FLAG_ECC;

	if ((ni->flags & NAND_FLAG_ECC) == 0) {
		/* ECC disabled */
		ni->sector.data_size = 0;
		ni->sector.ecc_bytes = 0;
		ni->sector.ecc_size = 0;
		ni->sector.ecc_t = 0;
		ni->sector.ecc_n = 0;
		ni->sector.ecc_k = 0;
	} else {
		if (((ni->sector.data_size != 512) &&
		     (ni->sector.data_size != 1024)) ||
		    (ni->sector.ecc_t > 40))
			/* Unsupported BCH ECC configuration */
			return -EINVAL;

		/*
		 * calculate the other ecc parameters from
		 * sector_size and ecc_t
		 */
		ni->sector.ecc_bytes = (14 * ni->sector.ecc_t + 7) / 8;

		/*
		 * The aux data size in DMAINT register has only 5 bits.
		 * By default the aux type set to 2 bytes, so max aux data
		 * size with this config is 31*2
		 */
		if (ni->sector.ecc_bytes > 62) {
			/* set auxiliary data type to 4 bytes */
			nand_writereg(ni, NAND_CONFIG0_OFFSET,
				      nc_set_aux_data_type(
					nand_readreg(ni, NAND_CONFIG0_OFFSET),
					NAND_CONFIG0_AUX_DATA_TYPE_4B));
		}

		/* round up ecc bytes to multiple of aux data type size */
		aux_type_size = nc_get_aux_data_type_size(ni);
		ni->sector.ecc_size = ((ni->sector.ecc_bytes + aux_type_size
					-1) / aux_type_size) * aux_type_size;

		ni->sector.ecc_n = 8 * (ni->sector.data_size +
					ni->sector.ecc_bytes);
		ni->sector.ecc_k = ni->sector.ecc_n - 14 * ni->sector.ecc_t;
	}

	/* AUX data configuration */
	if ((ni->flags & NAND_FLAG_AUX_CONFIG) == 0) {
		/*
		 * default aux size: 2 bytes for every 512 byte of data
		 * and ECC disabled
		 */
		ni->aux.data_size = 2 * (NANDBCH_PAGE_SIZE(ni) >> 9);
		ni->aux.ecc_bytes = 0;
		ni->aux.ecc_size = 0;
		ni->aux.ecc_t = 0;
		ni->aux.ecc_n = 0;
		ni->aux.ecc_k = 0;
	} else {
		/* custom aux configuration */
		aux_type_size = nc_get_aux_data_type_size(ni);

		/* check that AUX data size is multiple of aux data type size */
		if (ni->aux.data_size % aux_type_size)
			/* Invalid AUX data size */
			return -EINVAL;

		if (((ni->flags & NAND_FLAG_ECC) == 0) ||
		    (ni->aux.ecc_t == 0)) {
			ni->aux.ecc_bytes = 0;
			ni->aux.ecc_size = 0;
			ni->aux.ecc_t = 0;
			ni->aux.ecc_n = 0;
			ni->aux.ecc_k = 0;
		} else {
			/* with ECC on AUX data */
			if (ni->sector.ecc_t > 40)
				/* Unsupported BCH ECC configuration */
				return -EINVAL;

			/*
			 * calculate the other parameters from
			 * aux_size and aux_ecc_t
			 */
			ni->aux.ecc_bytes = (14 * ni->aux.ecc_t + 7) / 8;

			/*
			 *round up aux ecc bytes to multiple of aux
			 * data type size
			 */
			ni->aux.ecc_size = (((ni->aux.ecc_bytes + aux_type_size
					     - 1) / aux_type_size) *
					    aux_type_size);

			/*
			 * Auxiliary data size is on 5 bit in DMAINT register
			 * (due to hardware limitation(HWCAPRI-385)
			 */
			if ((ni->aux.data_size + ni->aux.ecc_size) >
			    (0x1F * aux_type_size))
				/* Unsupported BCH ECC configuration */
				return -EINVAL;

			ni->aux.ecc_n = 8 * (ni->aux.data_size +
					     ni->aux.ecc_bytes);
			ni->aux.ecc_k = ni->aux.ecc_n - 14 * ni->aux.ecc_t;
		}
	}

	/*
	 *if OOB size is known, verify that it can accommodate the configured
	 * AUX data plus AUX and main data ECC bytes
	 */
	if (ni->geometry.oob_size) {
		if (ni->geometry.oob_size < (ni->aux.data_size +
					     ni->aux.ecc_size +
					     NANDBCH_SECTORS(ni) *
					     ni->sector.ecc_size)) {
			/* Not enough OOB bytes to support this configuration */
			return -EINVAL;
		}
	}
	return 0;
}

/**
 * write page from buf to the nand device
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @page:   [in] page number
 * @buf:    [in] source buffer
 */
int nandbch_page_write(struct ctrl_nand_info *ni, uint8_t bank,
		       uint32_t page, uint8_t *buf)
{
	dev_dbg(ni->dev, "bank%u page 0x%x, buf=%p\n", bank, page, buf);

	if (ni->flags & NAND_FLAG_ECC)
		return nandbch_page_write_ecc(ni, bank, page, buf);

	return nandbch_page_write_raw(ni, bank, page, buf);
}

/**
 * Writes the auxiliary data to oob
 * @ni:     [in] nand info structure
 * @bank:   [in] bank number
 * @page:   [in] page number
 * @buf:    [in] source buffer
 */
int nandbch_aux_write(struct ctrl_nand_info *ni, uint8_t bank,
		      uint32_t page, uint8_t *buf)
{
	dev_dbg(ni->dev, "bank%u page 0x%x, buf=%p\n", bank, page, buf);

	if ((ni->flags & NAND_FLAG_ECC) && ni->aux.ecc_t)
		return nandbch_aux_write_ecc(ni, bank, page, buf);

	return nandbch_aux_write_raw(ni, bank, page, buf);
}
