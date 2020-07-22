// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 Broadcom.
 */

#include <linux/configfs.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/pci-epc.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>

#define BRCM_EP_MSIX_TBL_BASE		0x20010000
#define BRCM_EP_MSIX_TBL_SIZE		0x1000
#define BRCM_EP_MSIX_TBL_ADDR_L_OFFSET	0
#define BRCM_EP_MSIX_TBL_ADDR_H_OFFSET	4
#define BRCM_EP_MSIX_TBL_DATA_OFFSET	8
#define BRCM_EP_OB_LO_ADDR_MASK		0xf8000000
#define BRCM_EP_PAXB0_CONFIG_IND_ADDR	0x120
#define BRCM_EP_PAXB0_CONFIG_IND_DATA	0x124
#define BRCM_EP_OARR2_LOWER_OFFSET	0xd60
#define BRCM_EP_OARR2_UPPER_OFFSET	0xd64
#define BRCM_EP_OMAP2_LOWER_OFFSET	0xd68
#define BRCM_EP_OMAP2_UPPER_OFFSET	0xd6c
#define BRCM_EP_OARR2_VALID		BIT(0)

/* EP extended configuration space register offset */
#define BRCM_EP_CFG_REG_MSIX_CTRL	0x4c0
#define BRCM_EP_CFG_REG_MSIX_CTRL_MSK	0x3ff

struct brcm_pcie_ep {
	phys_addr_t ob_phys_addr;	/* outbound physical address */
	void __iomem *ob_map_addr;	/* outbound mapped virtual address */
	struct resource *ob_mem_res;	/* outbound memory resource */
	void __iomem *paxb_base;	/* PAXB base */
	void __iomem *msix_tbl;		/* MSIX table info */
	struct pci_epc *epc;		/* PCIe endpoint controller */
	struct device *dev;
};

int brcm_pcie_ep_outbound_map(struct brcm_pcie_ep *ep,
			      u64 addr, u64 *offset)
{
	phys_addr_t omap_base;
	u32 omap_low, omap_high, omap_low_aligned;

	omap_high = upper_32_bits(addr);
	omap_low = lower_32_bits(addr);
	omap_low_aligned = (omap_low & BRCM_EP_OB_LO_ADDR_MASK);

	omap_base = ep->ob_phys_addr;
	writel(((lower_32_bits(omap_base) & BRCM_EP_OB_LO_ADDR_MASK) |
		BRCM_EP_OARR2_VALID),
	       ep->paxb_base + BRCM_EP_OARR2_LOWER_OFFSET);

	writel(upper_32_bits(omap_base),
	       ep->paxb_base + BRCM_EP_OARR2_UPPER_OFFSET);

	writel(omap_low_aligned,
	       ep->paxb_base + BRCM_EP_OMAP2_LOWER_OFFSET);
	writel(omap_high,
	       ep->paxb_base + BRCM_EP_OMAP2_UPPER_OFFSET);

	*offset = (omap_low - omap_low_aligned);
	return 0;
}

int brcm_pcie_ep_mapped_cpu_write(struct brcm_pcie_ep *ep,
				  u64 pci_addr,
				  unsigned int data)
{
	u64 offset;
	int ret = 0;

	ret = brcm_pcie_ep_outbound_map(ep, pci_addr, &offset);
	if (ret)
		return -EIO;

	writel(data, (ep->ob_map_addr + offset));
	return 0;
}

static int brcm_pcie_ep_send_msi_irq(struct brcm_pcie_ep *ep, u8 fn,
				     u8 interrupt_num)
{
	u64 pci_addr;
	unsigned int data;

	data = readl(ep->msix_tbl + 16 * interrupt_num +
		     BRCM_EP_MSIX_TBL_ADDR_H_OFFSET);
	pci_addr = ((u64)data) << 32;
	data = readl(ep->msix_tbl + 16 * interrupt_num +
		     BRCM_EP_MSIX_TBL_ADDR_L_OFFSET);
	pci_addr |= data;

	data = readl(ep->msix_tbl + 16 * interrupt_num +
		     BRCM_EP_MSIX_TBL_DATA_OFFSET);

	brcm_pcie_ep_mapped_cpu_write(ep, pci_addr, data);

	return 0;
}

static void brcm_pcie_ep_rd_ind_reg(struct brcm_pcie_ep *ep, uint32_t addr,
				    uint32_t *data)
{
	writel(addr, ep->paxb_base + BRCM_EP_PAXB0_CONFIG_IND_ADDR);
	*data = readl(ep->paxb_base + BRCM_EP_PAXB0_CONFIG_IND_DATA);
}

static int brcm_pcie_ep_get_msix(struct pci_epc *epc, u8 func_no)
{
	struct brcm_pcie_ep *ep = epc_get_drvdata(epc);
	u32 msix_no;

	brcm_pcie_ep_rd_ind_reg(ep, BRCM_EP_CFG_REG_MSIX_CTRL, &msix_no);
	msix_no &= BRCM_EP_CFG_REG_MSIX_CTRL_MSK;

	if (msix_no == 0)
		return -EIO;
	else
		return msix_no;
}

static int brcm_pcie_ep_raise_irq(struct pci_epc *epc, u8 fn,
				  enum pci_epc_irq_type type,
				  u16 interrupt_num)
{
	struct brcm_pcie_ep *ep = epc_get_drvdata(epc);

	switch (type) {
	case PCI_EPC_IRQ_MSI:
	case PCI_EPC_IRQ_MSIX:
		return brcm_pcie_ep_send_msi_irq(ep, fn, interrupt_num);
	default:
		return -EINVAL;
	}
}

static const struct pci_epc_features brcm_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = false,
	.msix_capable = true,
};

static const struct pci_epc_features*
brcm_pcie_ep_get_features(struct pci_epc *epc, u8 func_no)
{
	return &brcm_pcie_epc_features;
}

static const struct pci_epc_ops brcm_pcie_epc_ops = {
	.get_msix	= brcm_pcie_ep_get_msix,
	.raise_irq	= brcm_pcie_ep_raise_irq,
	.get_features	= brcm_pcie_ep_get_features,
};

static const struct of_device_id brcm_pcie_ep_of_match[] = {
	{ .compatible = "brcm,pcie-ep"},
	{},
};

static int brcm_pcie_ep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct brcm_pcie_ep *ep;
	struct pci_epc *epc;
	int err;
	struct resource *res;

	ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	ep->dev = dev;

	epc = devm_pci_epc_create(dev, &brcm_pcie_epc_ops);
	if (IS_ERR(epc)) {
		dev_err(dev, "failed to create epc device\n");
		return PTR_ERR(epc);
	}
	ep->epc = epc;
	epc_set_drvdata(epc, ep);
	ep->epc->max_functions = 1;

	ep->ob_mem_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						      "ob-mem-base");
	if (!ep->ob_mem_res)
		return -EINVAL;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "paxb-base");
	ep->paxb_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ep->paxb_base)) {
		dev_err(dev, "failed to map paxb-base registers\n");
		return PTR_ERR(ep->paxb_base);
	}

	ep->msix_tbl = ioremap(BRCM_EP_MSIX_TBL_BASE, BRCM_EP_MSIX_TBL_SIZE);
	if (IS_ERR(ep->msix_tbl)) {
		dev_err(dev, "failed to map msix table base\n");
		return PTR_ERR(ep->msix_tbl);
	}

	err = pci_epc_mem_init(epc, ep->ob_mem_res->start,
			       resource_size(ep->ob_mem_res));
	if (err < 0) {
		dev_err(dev, "failed to initialize the memory space\n");
		return err;
	}

	ep->ob_map_addr = pci_epc_mem_alloc_addr(epc, &ep->ob_phys_addr,
						 resource_size(ep->ob_mem_res));
	if (!ep->ob_map_addr) {
		dev_err(dev, "failed to reserve memory space for MSI\n");
		err = -ENOMEM;
		goto err_epc_mem_exit;
	}

	return 0;

err_epc_mem_exit:
	pci_epc_mem_exit(epc);
	return err;
}

static struct platform_driver brcm_pcie_ep_driver = {
	.driver = {
		.name = "brcm-pcie-ep",
		.of_match_table = brcm_pcie_ep_of_match,
	},
	.probe = brcm_pcie_ep_probe,
};

builtin_platform_driver(brcm_pcie_ep_driver);
