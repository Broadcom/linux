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

#ifndef __BCM281XX_NANDBCH_H__
#define __BCM281XX_NANDBCH_H__

/* NAND flags used during init process */
#define NAND_FLAG_ECC		BIT(0)	/* ecc disabled if not set */
#define NAND_FLAG_GEOMETRY	BIT(1)	/* no onfi, use provided parameters */
#define NAND_FLAG_TIMING	BIT(2)	/* use provided timing */
#define NAND_FLAG_ECC_CONFIG	BIT(3)	/* use provided ecc parameters */
#define NAND_FLAG_AUX_CONFIG	BIT(4)	/* use provided auxiliary parameters */

#define NANDBCH_BANKS_MAX	2
#define NANDBCH_UC_CMD_MAX	32
#define NANDBCH_PAGE_MAX_SIZE	0x4000	/* 16K page */

/* ONFI parameters */
#define NANDBCH_FEATURE(ni)	(((struct ctrl_nand_info *)(ni))->onfi.feature)
#define NANDBCH_OPT_CMD(ni)	(((struct ctrl_nand_info *)(ni))->onfi.opt_cmd)

#define NANDBCH_FEATURE_EXT_PARAM_PAGE	0x80
#define NANDBCH_OPT_CMD_GET_SET_FEATURE	0x4

#define NANDBCH_OOB_MAX_SIZE	0x400	/* 1024 bytes */
#define NANDBCH_AUX_MAX_SIZE	0xf8	/* 248 bytes (31*8) */
#define NANDBCH_ECC_MAX_SIZE	0x48	/* 70 ECC bytes + 2 bytes padding */

/* size of the ID buffer */
#define NANDBCH_ID_BUF_SIZE	8
/* size of the ONFI feature buffer */
#define NANDBCH_FEATURE_SIZE	4

/* Nand ID definitions */
#define NANDBCH_JEDEC_ID_ADDR	0x0
#define NANDBCH_JEDEC_ID_SIZE	8
#define NANDBCH_ONFI_ID_ADDR	0x20
#define NANDBCH_ONFI_ID_SIZE	4

/* NAND timing select values */
enum {
	/* ONFI timing mode 0 */
	NANDBCH_TIMING_SELECT_DEFAULT = 0x00,
	NANDBCH_TIMING_SELECT_TMODE_SET_FEATURE,
	/* Use ONFI timing mode and send */
	/* ONFI SET_FEATURE command to   */
	/* device to change timing mode  */
	NANDBCH_TIMING_SELECT_TMODE_ONLY,   /* Use ONFI timing mode only */
	NANDBCH_TIMING_SELECT_REG_CAPRI,    /* Use NAND controller timing */
	/* configuration register values */
	/* (Capri encoding scheme)       */
	NANDBCH_TIMING_SELECT_MAX
};

/* DMA descriptor entry */
struct nand_prd_entry {
	uint32_t phys_addr;   /* source/destination physical address */
	uint32_t desc;        /* descriptor*/
};

/* NAND timing parameters */
struct nandbch_timing {
	uint8_t select;
	uint8_t mode;
	uint32_t conf1;
	uint32_t conf2;
};

/* NAND geometry information */
struct nandbch_geometry {
	uint32_t bus_width;
	uint32_t addr_cycles;
	uint32_t banks;
	uint32_t page_shift;
	uint32_t block_shift;
	uint32_t bank_shift;
	uint32_t oob_size;
};

/* NAND BCH ECC information */
struct nandbch_bch_ecc {
	uint32_t data_size;
	uint32_t ecc_bytes;   /* number of ECC bytes */
	uint32_t ecc_size;    /* including padding bytes */
	uint16_t ecc_n;
	uint16_t ecc_k;
	uint8_t ecc_t;
};

/* NAND ONFI parameters */
struct nandbch_onfi_param {
	uint16_t feature;
	uint16_t opt_cmd;
};

#define NANDBCH_MAX_SECTORS 32   /* 16K page with 512 byte sectors */
/* NAND ECC statistics */
struct nandbch_eccstats {
	uint8_t len;   /* length of statistics filled in by called function */
	uint8_t errs[NANDBCH_MAX_SECTORS];
};

#define BCMCAPRI_BUF_SIZE (NANDBCH_PAGE_MAX_SIZE + NANDBCH_OOB_MAX_SIZE)

struct bcmnand_buf {
	uint8_t buf[BCMCAPRI_BUF_SIZE];
	dma_addr_t phys_addr;
};
struct bcmnand_desc {
	struct nand_prd_entry desc[2];   /* Maximum 2 descriptors needed */
	dma_addr_t phys_addr;
};
struct bcmnand_param_buf {
	uint8_t buf[512];
	dma_addr_t phys_addr;
};
struct bcmnand_ecc_buf {
	uint8_t buf[NANDBCH_ECC_MAX_SIZE];
	dma_addr_t phys_addr;
};

/* NAND information structure */
struct ctrl_nand_info {
	uint8_t id[NANDBCH_JEDEC_ID_SIZE];
	uint32_t flags;
	void __iomem *nand_base;
	struct nandbch_geometry geometry;
	struct nandbch_timing timing;
	struct nandbch_bch_ecc sector;
	struct nandbch_bch_ecc aux;
	struct nandbch_onfi_param onfi;
	uint16_t uc_offset[NANDBCH_UC_CMD_MAX];
	struct bcmnand_buf buf;
	struct bcmnand_desc desc;
	struct bcmnand_param_buf param_buf;
	struct bcmnand_ecc_buf ecc_buf;
	struct device *dev;
};

#define NANDBCH_PAGE_SHIFT(ni)		(((struct ctrl_nand_info *)\
						  (ni))->geometry.page_shift)
#define NANDBCH_BLOCK_SHIFT(ni)		(((struct ctrl_nand_info *)\
						  (ni))->geometry.block_shift)
#define NANDBCH_BANK_SHIFT(ni)		(((struct ctrl_nand_info *)\
						  (ni))->geometry.bank_shift)
#define NANDBCH_SECTOR_SIZE(ni)		(((struct ctrl_nand_info *)\
						  (ni))->sector.data_size)
#define NANDBCH_PAGE_SIZE(ni)		(0x1U<<NANDBCH_PAGE_SHIFT(ni))

#define NANDBCH_BLOCK_SIZE(ni)		(0x1U<<NANDBCH_BLOCK_SHIFT(ni))

#define NANDBCH_SECTORS(ni)		(NANDBCH_PAGE_SIZE(ni)/ \
						  NANDBCH_SECTOR_SIZE(ni))
#define NANDBCH_PAGES(ni)		(0x1U<<(NANDBCH_BLOCK_SHIFT(ni) - \
						  NANDBCH_PAGE_SHIFT(ni)))
#define NANDBCH_BLOCKS(ni)		(0x1U<<(NANDBCH_BANK_SHIFT(ni) - \
						  NANDBCH_BLOCK_SHIFT(ni)))

int nandbch_init(struct ctrl_nand_info *ni, uint8_t flags);

int nandbch_config(struct ctrl_nand_info *ni, uint8_t bank,
		   uint8_t addr_cycles, uint32_t *page, uint32_t incr,
		   uint32_t iter);

int nandbch_id_get(struct ctrl_nand_info *ni, uint8_t bank, uint8_t addr,
		   uint8_t len, uint8_t *buf);

int nandbch_status_get(struct ctrl_nand_info *ni, uint8_t bank,
		       uint8_t *status);

int nandbch_reset(struct ctrl_nand_info *ni, uint8_t bank);

int nandbch_page_read(struct ctrl_nand_info *ni, uint8_t bank, uint32_t page,
		      uint8_t *buf, struct nandbch_eccstats *stats);

int nandbch_aux_read(struct ctrl_nand_info *ni, uint8_t bank, uint32_t page,
		     uint8_t *buf, uint8_t *stats);

int nandbch_oob_read(struct ctrl_nand_info *ni, uint8_t bank, uint32_t page,
		     uint8_t *buf);

int nandbch_page_write(struct ctrl_nand_info *ni, uint8_t bank, uint32_t page,
		       uint8_t *buf);

int nandbch_aux_write(struct ctrl_nand_info *ni, uint8_t bank, uint32_t page,
		      uint8_t *buf);

int nandbch_block_isbad(struct ctrl_nand_info *ni, uint8_t bank,
			uint32_t block, bool *is_bad);

int nandbch_block_force_erase(struct ctrl_nand_info *ni, uint8_t bank,
			      uint32_t block);

int nandbch_block_erase(struct ctrl_nand_info *ni, uint8_t bank,
			uint32_t block);

int nandbch_block_markbad(struct ctrl_nand_info *ni, uint8_t bank,
			  uint32_t block);

#endif
