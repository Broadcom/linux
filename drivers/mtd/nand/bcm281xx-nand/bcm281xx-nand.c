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

#include <asm/sizes.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/flashchip.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "bcm281xx-nandbch.h"

#ifdef NAND_SUBPAGE_READ
#undef NAND_SUBPAGE_READ
#define NAND_SUBPAGE_READ(chip) ((chip->ecc.mode == NAND_ECC_SOFT) \
					&& (chip->page_shift > 9))
#endif

#define NAND_MAX_OOBSIZE	576
#define NAND_MAX_PAGESIZE	8192
#define NAND_MAX_IDSIZE		8

static const char * const part_probes[] = { "cmdlinepart", "ofpart", NULL };

struct bcm281xx_host {
	/* nand framework related */
	struct nand_chip chip;
	struct nand_hw_control controller;
	struct ctrl_nand_info ni;
	/* the clock for NAND controller */
	struct clk *clk_peri;
	uint8_t nand_id[NAND_MAX_IDSIZE];
	uint8_t current_chip;
	unsigned int last_cmd;
	unsigned int last_byte;
	uint32_t last_addr;
};

static void bcm281xx_nand_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct bcm281xx_host *host = nand_get_controller_data(chip);
	struct ctrl_nand_info *pni = chip->priv;

	/* Multi chips selection is not supported yet. */
	if (chipnr < 0)
		return;
	else if (chipnr > 0)
		dev_err(pni->dev, "multi-chips in one NAND is not supported\n");

	host->current_chip = 0;
}

static int bcm281xx_nand_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
				  int page)
{
	struct bcm281xx_host *host = nand_get_controller_data(chip);
	struct ctrl_nand_info *pni = chip->priv;
	uint8_t *buf = (uint8_t *)chip->oob_poi;
	uint8_t chipnr = host->current_chip;
	uint8_t auxerrs;
	int ret;

	ret = nandbch_aux_read(pni, chipnr, page, buf, &auxerrs);
	if (ret)
		dev_err(pni->dev, "failure to read oob %d errors\n", auxerrs);

	return ret;
}

static int bcm281xx_nand_read_page(struct mtd_info *mtd, struct nand_chip *chip,
				   uint8_t *buf, int oob_required, int page)
{
	struct bcm281xx_host *host = nand_get_controller_data(chip);
	struct ctrl_nand_info *pni = chip->priv;
	struct nandbch_eccstats stats;
	uint8_t chipnr = host->current_chip;
	int ret;

	/* Enable ECC if not required */
	pni->flags |= NAND_FLAG_ECC;
	ret = nandbch_page_read(pni, chipnr, page,
		      buf, &stats);
	if (ret) {
		dev_err(pni->dev, "failure to read data from this page\n");
		return ret;
	}

	if (oob_required)
		ret = bcm281xx_nand_read_oob(mtd, chip, page);

	if (ret) {
		dev_err(pni->dev, "failure to read oob from this page\n");
		return ret;
	}

	return 0;
}

static int bcm281xx_nand_read_page_raw(struct mtd_info *mtd,
				       struct nand_chip *chip,
				       uint8_t *buf,
				       int oob_required,
				       int page)
{
	struct bcm281xx_host *host = nand_get_controller_data(chip);
	struct ctrl_nand_info *pni = chip->priv;
	struct nandbch_eccstats stats;
	uint8_t chipnr = host->current_chip;

	/* disable ECC if not required */
	pni->flags &= ~NAND_FLAG_ECC;
	return nandbch_page_read(pni, chipnr, page,
		      buf, &stats);
}

static int bcm281xx_nand_write_page(struct mtd_info *mtd,
				    struct nand_chip *chip,
				    const uint8_t *buf,
				    int oob_required,
				    int page)
{
	struct bcm281xx_host *host = nand_get_controller_data(chip);
	struct ctrl_nand_info *pni = chip->priv;
	uint8_t chipnr = host->current_chip;

	/* Enable ECC if not required */
	pni->flags |= NAND_FLAG_ECC;
	return nandbch_page_write(pni, chipnr, page, (uint8_t *)buf);
}

static int bcm281xx_nand_write_page_raw(struct mtd_info *mtd,
					struct nand_chip *chip,
					const uint8_t *buf,
					int oob_required,
					int page)
{
	struct bcm281xx_host *host = nand_get_controller_data(chip);
	struct ctrl_nand_info *pni = chip->priv;
	uint8_t chipnr = host->current_chip;

	/* disable ECC if not required */
	pni->flags &= ~NAND_FLAG_ECC;
	return nandbch_page_write(pni, chipnr, page,  (uint8_t *)buf);
}

static int bcm281xx_nand_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
				   int page)
{
	struct bcm281xx_host *host = nand_get_controller_data(chip);
	struct ctrl_nand_info *pni = chip->priv;
	uint8_t chipnr = host->current_chip;
	uint8_t status;
	int ret;

	ret = nandbch_aux_write(pni, chipnr, page, (uint8_t *)chip->oob_poi);
	if (ret) {
		dev_err(pni->dev, "failure to write oob\n");
		return -EIO;
	}

	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

static int bcm281xx_nand_ooblayout_ecc(struct mtd_info *mtd, int section,
				       struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct ctrl_nand_info *pni = chip->priv;

	if (section)
		return -ERANGE;

	oobregion->offset = pni->aux.data_size;
	oobregion->length = pni->geometry.oob_size - pni->aux.data_size;

	return 0;
}

static int bcm281xx_nand_ooblayout_free(struct mtd_info *mtd, int section,
					struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct ctrl_nand_info *pni = chip->priv;

	if (section)
		return -ERANGE;

	oobregion->offset = 1;
	oobregion->length = pni->aux.data_size - 1;

	return 0;
}

static const struct mtd_ooblayout_ops bcm281xx_nand_ooblayout_ops = {
	.ecc = bcm281xx_nand_ooblayout_ecc,
	.free = bcm281xx_nand_ooblayout_free,
};

static int bcm281xx_init_nand(struct bcm281xx_host *host)
{
	uint8_t flags;
	uint32_t page;
	struct ctrl_nand_info *pni = host->chip.priv;
	struct device *dev = pni->dev;
	int ret;

	/* Initialize hardware. */
	/*
	 * Note - it is assumed that the pinmux and clocks are already
	 * setup by the platform initialization code.
	 */
	/*
	 * Don't have to clear pni structure because it was allocated
	 * and zeroed by caller
	 */
	flags = NAND_FLAG_ECC;

	/*
	 * Read this page and fill in the geometry now.
	 * Try 5 then 4 address cycles, every 64th page, 8 checks total.
	 */
	page = 0;
	ret = nandbch_config(pni, 0, 5, &page, 64, 2);
	if (ret != 0) {
		page = 0;
		ret = nandbch_config(pni, 0, 4, &page, 64, 2);
		if (ret != 0) {
			dev_err(dev, "Failed to find nand geometry page.\n");
			return -EINVAL;
		}
	}
	flags |= NAND_FLAG_GEOMETRY; /* Tell the init not to use ONFI */

	if (pni->flags & NAND_FLAG_TIMING)
		/* Config function found timing settings in parameter table */
		flags |= NAND_FLAG_TIMING;

	ret = nandbch_init(pni, flags);
	if (ret != 0) {
		dev_err(dev, "Failed to initialize.\n");
		return -EINVAL;
	}

	dev_info(dev, "\nNAND %d bit, ECC %s\n"
	       "ID %02X%02X%02X%02X%02X%02X%02X%02X\n"
	       "banks %d\n"
	       "bank size %d MB\n"
	       "page size %d KB\n"
	       "block size %d KB\n"
	       "aux data size %d bytes\n"
	       "used oob bytes %d",
	       pni->geometry.bus_width,
	       (pni->flags & NAND_FLAG_ECC) ? "enabled" : "disabled",
	       (pni->id)[0],
	       (pni->id)[1],
	       (pni->id)[2],
	       (pni->id)[3],
	       (pni->id)[4],
	       (pni->id)[5],
	       (pni->id)[6],
	       (pni->id)[7],
	       pni->geometry.banks,
	       0x1 << (NANDBCH_BANK_SHIFT(pni) - 20),
	       NANDBCH_PAGE_SIZE(pni) >> 10,
	       NANDBCH_BLOCK_SIZE(pni) >> 10,
	       pni->aux.data_size,
	       (pni->flags & NAND_FLAG_ECC) ? (pni->aux.data_size
							  + pni->aux.ecc_size +
							  NANDBCH_SECTORS(pni) *
							  pni->sector.ecc_size)
	       : pni->aux.data_size);

	if (pni->geometry.oob_size)
		dev_info(dev, "/%d", pni->geometry.oob_size);

	if (pni->flags & NAND_FLAG_ECC) {
		dev_info(dev, "main data ecc %d/%d\n"
		       "aux data ecc %d/%d\n",
		       pni->sector.ecc_t, NANDBCH_SECTOR_SIZE(pni),
		       pni->aux.ecc_t, pni->aux.data_size);
	}
	if (pni->flags & NAND_FLAG_TIMING) {
		dev_info(dev, "timing_select %d\n"
		       "timing mode %d\n",
		       pni->timing.select,
		       pni->timing.mode);
	}

	return 0;
}

static void bcm281xx_nand_cmdfunc(struct mtd_info *mtd, unsigned int command,
				  int column, int page_addr)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct bcm281xx_host *host = nand_get_controller_data(chip);
	struct ctrl_nand_info *pni = chip->priv;
	uint64_t addr = (uint64_t)page_addr << chip->page_shift;
	uint32_t block = (uint32_t)(addr >> chip->phys_erase_shift);
	uint8_t chipnr = host->current_chip;
	uint32_t id_addr = 0;

	if (command == NAND_CMD_READID || command == NAND_CMD_PARAM ||
			command == NAND_CMD_RNDOUT)
		addr = (uint32_t)column;
	/* Avoid propagating a negative, don't-care address */
	else if (page_addr < 0)
		addr = 0;

	host->last_cmd = command;
	host->last_byte = 0;
	host->last_addr = addr;

	switch (command) {
	case NAND_CMD_RESET:
		nandbch_reset(pni, chipnr);
		break;

	case NAND_CMD_READID:
		nandbch_id_get(pni, chipnr, id_addr, NAND_MAX_IDSIZE,
			       (uint8_t *)host->nand_id);
		break;

	case NAND_CMD_ERASE1:
		nandbch_block_erase(pni, chipnr, block);
		break;

	case NAND_CMD_READ0:
		/* we read the entire page for now */
		WARN_ON(column != 0);
		break;

	case NAND_CMD_SEQIN:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_STATUS:
	default:
		break;
	}

}

static uint8_t bcm281xx_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct bcm281xx_host *host = nand_get_controller_data(chip);
	struct ctrl_nand_info *pni = chip->priv;
	uint8_t status;

	if (host->last_cmd == NAND_CMD_READID) {
		host->last_byte++;
		return *(uint8_t *)(host->nand_id + host->last_byte - 1);
	}

	if (host->last_cmd == NAND_CMD_STATUS) {
		nandbch_status_get(pni, 0, &status);
		host->last_byte++;
		return status;
	}

	return 0;
}

static const struct of_device_id bcm281xx_nand_of_match[] = {
	{ .compatible = "brcm,nand-bcm281xx" },
	{},
};

MODULE_DEVICE_TABLE(of, bcm281xx_nand_of_match);

static int bcm281xx_nand_host_init(struct bcm281xx_host *host,
				   struct ctrl_nand_info *pni,
				   struct device_node *dn)
{
	struct nand_chip *chip = &host->chip;
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct device *dev = pni->dev;
	int ret;

	/* We only support device-tree instantiation */
	if (!dn)
		return -ENODEV;
	if (!of_match_node(bcm281xx_nand_of_match, dn))
		return -ENODEV;

	if (bcm281xx_init_nand(host)) {
		dev_err(dev, "NAND init failed\n");
		ret = -EINVAL;
		return ret;
	}

	mtd->name = dev_name(dev);
	mtd->owner = THIS_MODULE;
	nand_set_flash_node(chip, dn);
	nand_set_controller_data(chip, host);

	chip->priv = pni;
	chip->cmdfunc = bcm281xx_nand_cmdfunc;
	chip->read_byte = bcm281xx_nand_read_byte;
	chip->select_chip = bcm281xx_nand_select_chip;

	chip->controller = &host->controller;
	ret = nand_scan_ident(mtd, 1, NULL);
	if (ret) {
		dev_info(dev, " Unable to do nand_scan_ident()\n");
		return ret;
	}

	chip->options |= NAND_NO_SUBPAGE_WRITE | NAND_USE_BOUNCE_BUFFER |
			 NAND_SKIP_BBTSCAN;

	/* Rewrite parameter after doing NAND ID detection */
	chip->chipsize = 0x1 << (NANDBCH_BANK_SHIFT(pni));
	chip->numchips = pni->geometry.banks;
	chip->page_shift = NANDBCH_PAGE_SHIFT(pni);
	chip->phys_erase_shift = NANDBCH_BLOCK_SHIFT(pni);

	mtd->size = chip->numchips * chip->chipsize;
	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;
	mtd->erasesize = NANDBCH_BLOCK_SIZE(pni);
	mtd->writesize = NANDBCH_PAGE_SIZE(pni);
	mtd->oobsize = pni->aux.data_size;
	mtd->oobavail = pni->aux.data_size - 1;
	mtd->subpage_sft = 0;
	chip->subpagesize = mtd->writesize >> mtd->subpage_sft;

	chip->buffers = kmalloc(sizeof(struct nand_buffers), GFP_KERNEL);
	if (!chip->buffers)
		return -ENOMEM;

	chip->buffers->databuf =
			kmalloc((NAND_MAX_PAGESIZE+NAND_MAX_OOBSIZE),
				GFP_KERNEL);

	chip->ecc.mode = NAND_ECC_HW;
	chip->ecc.size = pni->sector.data_size;
	chip->ecc.bytes = pni->sector.ecc_bytes;
	chip->ecc.strength = 2;

	/* Set the internal oob buffer location, just after the page data */
	chip->oob_poi = chip->buffers->databuf + NANDBCH_PAGE_SIZE(pni);

	/*Set ecc mode */
	chip->ecc.read_page = bcm281xx_nand_read_page;
	chip->ecc.read_page_raw = bcm281xx_nand_read_page_raw;
	chip->ecc.write_page = bcm281xx_nand_write_page;
	chip->ecc.write_page_raw = bcm281xx_nand_write_page_raw;
	chip->ecc.read_oob = bcm281xx_nand_read_oob;
	chip->ecc.write_oob = bcm281xx_nand_write_oob;

	mtd_set_ooblayout(mtd, &bcm281xx_nand_ooblayout_ops);
	ret = nand_scan_tail(mtd);
	if (ret)
		return ret;

	return mtd_device_register(mtd, NULL, 0);
}

static int bcm281xx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn = dev->of_node;
	struct mtd_info *mtd;
	struct bcm281xx_host *host;
	struct nand_chip *chip;
	struct ctrl_nand_info *pni;
	struct resource *res;
	int rate;
	int ret;

	/* We only support device-tree instantiation */
	if (!dn)
		return -ENODEV;

	if (!of_match_node(bcm281xx_nand_of_match, dn))
		return -ENODEV;

	host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	pni = &host->ni;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pni->nand_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pni->nand_base))
		return PTR_ERR(pni->nand_base);

	host->chip.priv = pni;
	pni->dev = &pdev->dev;
	chip = &host->chip;
	chip->priv = pni;
	mtd = nand_to_mtd(chip);
	nand_hw_control_init(&host->controller);

	host->clk_peri = devm_clk_get(dev, "nand_clk");
	if (IS_ERR(host->clk_peri))
		return PTR_ERR(host->clk_peri);

	rate = clk_get_rate(host->clk_peri);
	dev_info(dev, "BCM281XX NAND clock running at %u MHz\n", rate/1000000);

	ret = clk_set_rate(host->clk_peri, rate);
	if (ret) {
		dev_err(dev, "Couldn't set nand clk_peri clock rate, ret=%d\n",
			ret);
		return ret;
	}

	clk_prepare_enable(host->clk_peri);

	/* Descriptor mapping */
	pni->desc.phys_addr = dma_map_single(&pdev->dev, pni->desc.desc,
					     sizeof(pni->desc.desc),
					     DMA_BIDIRECTIONAL);
	if (dma_mapping_error(&pdev->dev, pni->desc.phys_addr)) {
		dev_err(dev, "BCM281XX_NAND: failed to map DMA descriptors\n");
		goto out_free_host;
	}

	dev_info(dev, "%s line %d  desc.phys_addr = 0x%x virtaddr=0x%p size=0x%x\n",
		 __func__, __LINE__, pni->desc.phys_addr, pni->desc.desc,
		 sizeof(pni->desc.desc));

	/* Transfer buffer mapping */
	pni->buf.phys_addr = dma_map_single(&pdev->dev, pni->buf.buf,
					     BCMCAPRI_BUF_SIZE,
					     DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dev, pni->buf.phys_addr)) {
		dev_err(dev, "failed to map DMA transfer buffer\n");
		goto out_free_host;
	}

	/* Param buffer mapping */
	pni->param_buf.phys_addr = dma_map_single(&pdev->dev,
					     pni->param_buf.buf,
					     512,
					     DMA_BIDIRECTIONAL);
	if (dma_mapping_error(&pdev->dev, pni->param_buf.phys_addr)) {
		dev_err(dev, "failed to map DMA param buffer\n");
		goto out_free_host;
	}

	/* ECC buffer mapping */
	pni->ecc_buf.phys_addr = dma_map_single(&pdev->dev, pni->ecc_buf.buf,
					     NANDBCH_ECC_MAX_SIZE,
					     DMA_BIDIRECTIONAL);
	if (dma_mapping_error(&pdev->dev, pni->ecc_buf.phys_addr)) {
		dev_err(&pdev->dev, "failed to map DMA ecc buffer\n");
		goto out_free_host;
	}

	if (bcm281xx_nand_host_init(host, pni, dn)) {
		dev_err(&pdev->dev, "NAND host init failed\n");
		ret = -EINVAL;
		goto out_free_host;
	}

	dev_set_drvdata(&pdev->dev, host);
	return 0;

out_free_host:
	kfree(host);

	return ret;
}

static int bcm281xx_remove(struct platform_device *pdev)
{
	struct bcm281xx_host *host = platform_get_drvdata(pdev);
	struct nand_chip *chip = &host->chip;
	struct ctrl_nand_info *pni = chip->priv;

	dma_unmap_single(&pdev->dev, pni->desc.phys_addr,
				sizeof(pni->desc.desc), DMA_BIDIRECTIONAL);
	dma_unmap_single(&pdev->dev, pni->buf.phys_addr,
				BCMCAPRI_BUF_SIZE, DMA_BIDIRECTIONAL);
	dma_unmap_single(&pdev->dev, pni->param_buf.phys_addr,
				512, DMA_BIDIRECTIONAL);
	dma_unmap_single(&pdev->dev, pni->ecc_buf.phys_addr,
				NANDBCH_ECC_MAX_SIZE, DMA_BIDIRECTIONAL);

	clk_disable_unprepare(host->clk_peri);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static struct platform_driver bcm281xx_driver = {
	.probe = bcm281xx_probe,
	.remove = bcm281xx_remove,
	/* suspend/resume functions are not here - called from mtd parent */
	.driver = {
			.name = "bcm281xx_nand",
			.owner = THIS_MODULE,
			.of_match_table = bcm281xx_nand_of_match,
		   },
};
module_platform_driver(bcm281xx_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("NAND driver for BCM281XX");
