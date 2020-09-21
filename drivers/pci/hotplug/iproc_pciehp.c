// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Broadcom
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include "../pci.h"

#define PAX_FLUSH_CONTROL	0xc
#define CFG_FLUSH_CLEAR		BIT(31)
#define CFG_PAX_FLUSH		BIT(0)
#define PAX_FLUSH_STATUS	0x10
#define STAT_RD_FLUSH_DONE	BIT(1)
#define STAT_WR_FLUSH_DONE	BIT(0)
#define STAT_FLUSH_TIMEOUT_MS	10
#define PAX_PCIE_LINK_STATUS	0x49c
#define LINK_PCIE_PHYLINKUP	BIT(3)
#define LINK_PCIE_DL_ACTIVE	BIT(2)
#define PAX_PCIE_INTR_STATUS	0x4ec
#define PAX_PCIE_INTR_MASK	0x4f0
#define PAX_PCIE_INTR_CLEAR	0x4f4
#define COMPLETION_TIMEOUT	BIT(12)
#define PCIE_DL_ACTIVE		BIT(11)
#define PCIE_PHYLINK_STATUS	BIT(10)

struct iproc_pciehp {
	struct device *dev;
	void __iomem *regs;
	struct pci_dev *pdev;
	int domain_num;
	int irq;
	u32 int_mask;
#define IPROC_HP_LINKDOWN_EVENT	BIT(0)
#define IPROC_HP_LINKUP_EVENT	BIT(1)
	atomic_t pending_events;
};

static struct pci_bus *iproc_pciehp_get_bus(struct iproc_pciehp *ipciehp)
{
	struct pci_bus *b;

	b = pci_find_bus(ipciehp->domain_num, 0);
	if (b)
		ipciehp->pdev = list_first_entry(&b->devices,
						 struct pci_dev, bus_list);

	return b;
}

static void iproc_pciehp_plug(struct pci_dev *pdev)
{
	struct pci_bus *parent = pdev->bus;

	dev_dbg(&parent->dev, "%s: domain:bus:dev = %04x:%02x:00\n",
		__func__, pci_domain_nr(parent), parent->number);

	pci_lock_rescan_remove();
	pci_rescan_bus(parent);
	pci_unlock_rescan_remove();
}

static void iproc_pciehp_unplug(struct pci_dev *pdev)
{
	struct pci_dev *dev, *temp;
	struct pci_bus *parent = pdev->subordinate;

	dev_dbg(&parent->dev, "%s: domain:bus:dev = %04x:%02x:00\n",
		__func__, pci_domain_nr(parent), parent->number);

	pci_walk_bus(parent, pci_dev_set_disconnected, NULL);

	list_for_each_entry_safe_reverse(dev, temp, &parent->devices,
					 bus_list)
		pci_stop_and_remove_bus_device_locked(dev);
}

static irqreturn_t iproc_pciehp_isr(int irq, void *dev_id)
{
	struct iproc_pciehp *ipciehp = dev_id;
	uint32_t reg;

	reg = readl(ipciehp->regs + PAX_PCIE_INTR_STATUS);
	dev_dbg(ipciehp->dev, "PAX_PCIE_INTR_STATUS %#x\n", reg);

	if (reg & ipciehp->int_mask) {
		writel(reg, ipciehp->regs + PAX_PCIE_INTR_CLEAR);
		if (reg & PCIE_PHYLINK_STATUS) {
			reg = readl(ipciehp->regs + PAX_PCIE_LINK_STATUS);
			dev_dbg(ipciehp->dev, "PAX LINK STATUS %#x\n", reg);
			if (reg & (LINK_PCIE_PHYLINKUP | LINK_PCIE_DL_ACTIVE))
				atomic_or(IPROC_HP_LINKUP_EVENT,
					  &ipciehp->pending_events);
			else
				atomic_or(IPROC_HP_LINKDOWN_EVENT,
					  &ipciehp->pending_events);

			return IRQ_WAKE_THREAD;
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t iproc_pciehp_ist(int irq, void *dev_id)
{
	struct iproc_pciehp *ipciehp = dev_id;
	u32 events, val, timeout_ms;

	events = atomic_xchg(&ipciehp->pending_events, 0);

	dev_dbg(ipciehp->dev, "events %#x\n", events);

	if (!events || !iproc_pciehp_get_bus(ipciehp))
		return IRQ_NONE;

	if (events & IPROC_HP_LINKDOWN_EVENT) {
		iproc_pciehp_unplug(ipciehp->pdev);
		timeout_ms = STAT_FLUSH_TIMEOUT_MS;
		do {
			val = readl(ipciehp->regs + PAX_FLUSH_STATUS);
			if (val & (STAT_RD_FLUSH_DONE | STAT_WR_FLUSH_DONE))
				break;
			msleep(1);
		} while (timeout_ms--);

		/*
		 * Flush is not completed in given time can happen if PAXB core
		 * went into BAD state.
		 * TODO: Recovery handling from BAD state.
		 */
		if (!timeout_ms)
			dev_err(ipciehp->dev, "FLUSH not done %#x\n", val);

		val = readl(ipciehp->regs + PAX_FLUSH_CONTROL);
		val |= CFG_FLUSH_CLEAR;
		val &= ~CFG_PAX_FLUSH;
		writel(val, ipciehp->regs + PAX_FLUSH_CONTROL);
	} else if (events & IPROC_HP_LINKUP_EVENT)
		iproc_pciehp_plug(ipciehp->pdev);

	return IRQ_HANDLED;
}

static int bcm_iproc_pciehp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn = dev->of_node;
	struct resource *res;
	struct iproc_pciehp *ipciehp;
	struct device_node *rc_node;
	int ret;
	uint32_t reg;

	ipciehp = devm_kzalloc(dev, sizeof(*ipciehp), GFP_KERNEL);
	if (!ipciehp)
		return -ENOMEM;

	ipciehp->dev = dev;
	rc_node = of_parse_phandle(dn, "brcm,iproc-pciehp-rc", 0);
	if (rc_node) {
		ipciehp->domain_num = of_get_pci_domain_nr(rc_node);
		if (ipciehp->domain_num < 0)
			return ipciehp->domain_num;

		if (!iproc_pciehp_get_bus(ipciehp))
			return -EPROBE_DEFER;
	} else {
		return -EPROBE_DEFER;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ipciehp->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(ipciehp->regs))
		return PTR_ERR(ipciehp->regs);

	ipciehp->irq = platform_get_irq(pdev, 0);

	ret = request_threaded_irq(ipciehp->irq, iproc_pciehp_isr,
				   iproc_pciehp_ist, IRQF_SHARED,
				   "iproc-pciehp", ipciehp);
	if (ret)
		return ret;

	/* Enable Interrupts */
	ipciehp->int_mask = COMPLETION_TIMEOUT | PCIE_DL_ACTIVE |
			    PCIE_PHYLINK_STATUS;

	reg = readl(ipciehp->regs + PAX_PCIE_INTR_STATUS);

	/* Clear Interrupts */
	if (reg & ipciehp->int_mask)
		writel(ipciehp->int_mask,
		       ipciehp->regs + PAX_PCIE_INTR_CLEAR);

	writel(~ipciehp->int_mask, ipciehp->regs + PAX_PCIE_INTR_MASK);

	return 0;
}

static const struct of_device_id bcm_iproc_pciehp_of_match[] = {
	{
		.compatible = "brcm,iproc-pciehp",
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, bcm_iproc_pciehp_of_match);

static struct platform_driver bcm_iproc_pciehp_driver = {
	.driver = {
		.name = "iproc-pciehp",
		.of_match_table = bcm_iproc_pciehp_of_match,
	},
	.probe = bcm_iproc_pciehp_probe,
};
module_platform_driver(bcm_iproc_pciehp_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom iProc PCIe HP driver");
MODULE_LICENSE("GPL v2");
