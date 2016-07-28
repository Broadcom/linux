/*
 * Copyright (C) 2015 Broadcom Corporation
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

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>

#include "pcie-iproc.h"

#define IARR2_INDEX                  0
#define IARR3_INDEX                  1
#define IARR4_INDEX                  2
#define MAX_IARR_WINDOWS             3
#define IB_SENTINEL_SZ               0xffff

static const struct of_device_id iproc_pcie_of_match_table[] = {
	{
		.compatible = "brcm,iproc-pcie",
		.data = (int *)IPROC_PCIE_PAXB,
	}, {
		.compatible = "brcm,iproc-pcie-paxb-v2",
		.data = (int *)IPROC_PCIE_PAXB_V2,
	}, {
		.compatible = "brcm,iproc-pcie-paxc",
		.data = (int *)IPROC_PCIE_PAXC,
	}, {
		.compatible = "brcm,iproc-pcie-paxc-v2",
		.data = (int *)IPROC_PCIE_PAXC_V2,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, iproc_pcie_of_match_table);

static int pci_dma_range_parser_init(struct of_pci_range_parser *parser,
				     struct device_node *node)
{
	const int na = 3, ns = 2;
	int rlen;

	parser->node = node;
	parser->pna = of_n_addr_cells(node);
	parser->np = parser->pna + na + ns;

	parser->range = of_get_property(node, "dma-ranges", &rlen);
	if (!parser->range)
		return -ENOENT;

	parser->end = parser->range + rlen / sizeof(__be32);
	return 0;
}

static int iproc_pci_parse_map_dma_ranges(struct iproc_pcie *pcie,
					struct device_node *np)
{
	struct of_pci_range range;
	struct of_pci_range_parser parser;
	int iarr, index = 0, rc, ret;
	const unsigned int *p_iarr_sz = NULL;
	unsigned long wsize_gb, wsize_mb;

	/* Get the dma-ranges from DT */
	rc = pci_dma_range_parser_init(&parser, np);
	if (rc)
		return rc;

	ret = iproc_pcie_setup_ib_map(pcie);
	if (ret < 0) {
		dev_err(pcie->dev, "could not find ib map\n");
		return -EINVAL;
	}

	pcie->ib = devm_kzalloc(pcie->dev, sizeof(struct iproc_pcie_ib) *
			MAX_IARR_WINDOWS, GFP_KERNEL);

	if (pcie->ib == NULL)
		return -ENOMEM;

	/*
	 * PAXB_V2 implements address translation, by which memory
	 * map is aligned.
	 * IARR arrangement could conveniently map 3 DDR regions.
	 * DDR regions to be mapped are
	 * DDR	2GB	0x000_8000_0000	0x000_FFFF_FFFF >> IARR_2
	 * DDR	30GB	0x008_8000_0000	0x00F_FFFF_FFFF >> IARR_3
	 * DDR	480GB	0x088_0000_0000	0x0FF_FFFF_FFFF >> IARR_4
	 * giving us combining inbound window of 512 GB.
	 */
	for_each_of_pci_range(&parser, &range) {
		if (index >= MAX_IARR_WINDOWS) {
			dev_err(pcie->dev, "invalid number of inbound windows\n");
			return -EINVAL;
		}

		if (range.size == 0) {
			dev_err(pcie->dev, "invalid inbound window size\n");
			return -EINVAL;
		}

		wsize_mb = range.size / (unsigned int) (SZ_1M);
		wsize_gb = range.size / (unsigned int) (SZ_1G);

		if ((range.cpu_addr &
			pcie->ib_map[IARR4_INDEX].axi_mask) &&
			(IS_ALIGNED(range.size,
			(pcie->ib_map[IARR3_INDEX].iarr_size[1] * SZ_1G)))) {
			iarr = 2;
			range.size = wsize_gb;
		} else if ((range.cpu_addr &
			pcie->ib_map[IARR3_INDEX].axi_mask) &&
			(IS_ALIGNED(range.size,
			(pcie->ib_map[IARR3_INDEX].iarr_size[1] * SZ_1G)))) {
			iarr = 1;
			range.size = wsize_gb;
		} else if ((range.cpu_addr &
			pcie->ib_map[IARR2_INDEX].axi_mask) &&
			(IS_ALIGNED(range.size,
			(pcie->ib_map[IARR2_INDEX].iarr_size[1] * SZ_1M)))) {
			iarr = 0;
			range.size = wsize_mb;
		} else {
			dev_err(pcie->dev, "invalid inbound start addr\n");
			return -EINVAL;
		}

		pcie->ib[iarr].axi_addr = (unsigned long)range.cpu_addr;
		pcie->ib[iarr].pci_addr = (unsigned long)range.pci_addr;
		pcie->ib[iarr].window_size = (unsigned long)range.size;
		pcie->ib[iarr].wmask = pcie->ib_map[iarr].wmask;
		p_iarr_sz = &pcie->ib_map[iarr].iarr_size[0];

		while (*p_iarr_sz != IB_SENTINEL_SZ) {
			if ((pcie->ib[iarr].window_size > *p_iarr_sz) &&
			pcie->ib[iarr].window_size <= *(p_iarr_sz + 1)) {
				pcie->ib[iarr].iarr_size_bits =
				*(p_iarr_sz + 1) / pcie->ib_map[iarr].divider;
				pcie->ib[iarr].window_size = (iarr == 0) ?
					(resource_size_t)
					(p_iarr_sz + 1) * SZ_1M
					: (resource_size_t)
					(p_iarr_sz + 1) * SZ_1G;
				break;
			}
			p_iarr_sz++;
		}

		if (*p_iarr_sz == IB_SENTINEL_SZ) {
			dev_err(pcie->dev, "window size exceeded\n");
			return -EINVAL;
		}
		index++;
	}

	pcie->num_of_ib = index;
	return 0;
}

static int iproc_pcie_pltfm_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	struct iproc_pcie *pcie;
	struct device_node *np = pdev->dev.of_node;
	struct resource reg;
	resource_size_t iobase = 0;
	LIST_HEAD(res);
	int ret;

	of_id = of_match_device(iproc_pcie_of_match_table, &pdev->dev);
	if (!of_id)
		return -EINVAL;

	pcie = devm_kzalloc(&pdev->dev, sizeof(struct iproc_pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->dev = &pdev->dev;
	pcie->type = (enum iproc_pcie_type)of_id->data;
	platform_set_drvdata(pdev, pcie);

	ret = of_address_to_resource(np, 0, &reg);
	if (ret < 0) {
		dev_err(pcie->dev, "unable to obtain controller resources\n");
		return ret;
	}

	pcie->base = devm_ioremap(pcie->dev, reg.start, resource_size(&reg));
	if (!pcie->base) {
		dev_err(pcie->dev, "unable to map controller registers\n");
		return -ENOMEM;
	}
	pcie->base_addr = reg.start;

	if (!(iproc_pci_parse_map_dma_ranges(pcie, np)))
		pcie->need_ib_cfg = true;

	if (of_property_read_bool(np, "brcm,pcie-ob")) {
		u32 val;

		ret = of_property_read_u32(np, "brcm,pcie-ob-axi-offset",
					   &val);
		if (ret) {
			dev_err(pcie->dev,
				"missing brcm,pcie-ob-axi-offset property\n");
			return ret;
		}
		pcie->ob.axi_offset = val;

		ret = of_property_read_u32(np, "brcm,pcie-ob-window-size",
					   &val);
		if (ret) {
			dev_err(pcie->dev,
				"missing brcm,pcie-ob-window-size property\n");
			return ret;
		}

		if (pcie->type == IPROC_PCIE_PAXB)
			if (val > 256)
				return -EINVAL;

		switch (val) {
		case 128:
			pcie->ob.oarr_size_bits = 0;
			break;
		case 256:
			pcie->ob.oarr_size_bits = 1;
			break;
		case 512:
			pcie->ob.oarr_size_bits = 2;
			break;
		case 1024:
			pcie->ob.oarr_size_bits = 3;
			break;
		default:
			dev_err(pcie->dev,
			"invalid OARR size: %u MB\n", val);
			return -EINVAL;
		}

		pcie->ob.window_size = (resource_size_t)val * SZ_1M;

		pcie->need_ob_cfg = true;
	}

	/* PHY use is optional */
	pcie->phy = devm_phy_get(&pdev->dev, "pcie-phy");
	if (IS_ERR(pcie->phy)) {
		if (PTR_ERR(pcie->phy) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		pcie->phy = NULL;
	}

	ret = of_pci_get_host_bridge_resources(np, 0, 0xff, &res, &iobase);
	if (ret) {
		dev_err(pcie->dev,
			"unable to get PCI host bridge resources\n");
		return ret;
	}

	pcie->map_irq = of_irq_parse_and_map_pci;

	ret = iproc_pcie_setup(pcie, &res);
	if (ret)
		dev_err(pcie->dev, "PCIe controller setup failed\n");

	pci_free_resource_list(&res);

	return ret;
}

static int iproc_pcie_pltfm_remove(struct platform_device *pdev)
{
	struct iproc_pcie *pcie = platform_get_drvdata(pdev);

	return iproc_pcie_remove(pcie);
}

static struct platform_driver iproc_pcie_pltfm_driver = {
	.driver = {
		.name = "iproc-pcie",
		.of_match_table = of_match_ptr(iproc_pcie_of_match_table),
	},
	.probe = iproc_pcie_pltfm_probe,
	.remove = iproc_pcie_pltfm_remove,
};
module_platform_driver(iproc_pcie_pltfm_driver);

MODULE_AUTHOR("Ray Jui <rjui@broadcom.com>");
MODULE_DESCRIPTION("Broadcom iPROC PCIe platform driver");
MODULE_LICENSE("GPL v2");
