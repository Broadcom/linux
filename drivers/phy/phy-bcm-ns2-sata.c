/*
 * Copyright (C) 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#define MAX_PORTS				2

#define TOP_BUS_CTRL_OFFSET			0x44
 #define MMIO_ENDIAN_SHIFT			0 /* CPU->AHCI */
 #define DMADESC_ENDIAN_SHIFT			2 /* AHCI->DDR */
 #define DMADATA_ENDIAN_SHIFT			4 /* AHCI->DDR */
 #define PIODATA_ENDIAN_SHIFT			6
  #define ENDIAN_SWAP_NONE			0
  #define ENDIAN_SWAP_FULL			2
#define TOP_BUS_CTRL_DEFAULT				\
	((ENDIAN_SWAP_NONE << DMADATA_ENDIAN_SHIFT) |	\
	(ENDIAN_SWAP_NONE << DMADESC_ENDIAN_SHIFT) |	\
	(ENDIAN_SWAP_NONE << MMIO_ENDIAN_SHIFT))

#define PORT0_CTRL_OFFSET			0x4c
#define PORTx_CTRL_SIZE				0x8
#define PORT0_PCB_OFFSET			0x100
#define PORTx_PCB_SIZE				0x1000

/* Top-level control registers */
#define PORTx_CTRL1				0x0
#define PORTx_CTRL1_RESET			BIT(0)

/* PHY control block registers */
#define PORTx_PCB_REG0				0x200
#define PORTx_PCB_BLOCK_ADDR			0x23c

/* SATA3 PHY BLOCK0 registers */
#define SATA3_BLOCK0_BLOCK_ADDR			0x000
#define SATA3_BLOCK0_XGXSSTATUS			0x1
#define SATA3_BLOCK0_XGXSSTATUS_PLL_LOCK	BIT(12)
#define SATA3_BLOCK0_SPARE			0xd
#define SATA3_BLOCK0_SPARE_OOB_CLK_SEL_MASK	0x3
#define SATA3_BLOCK0_SPARE_OOB_CLK_SEL_REFBY2	0x1

/* SATA3 PHY PLL1 registers */
#define SATA3_PLL1_BLOCK_ADDR			0x060
#define SATA3_PLL1_ACTRL2			0x2
#define SATA3_PLL1_ACTRL2_MAGIC			0x1df8
#define SATA3_PLL1_ACTRL3			0x3
#define SATA3_PLL1_ACTRL3_MAGIC			0x2b00
#define SATA3_PLL1_ACTRL4			0x4
#define SATA3_PLL1_ACTRL4_MAGIC			0x8824

/* SATA3 PHY OOB registers */
#define SATA3_OOB_BLOCK_ADDR			0x150
#define SATA3_OOB_CTRL1				0x0
#define SATA3_OOB_CTRL1_BURST_MAX_MASK		0xf
#define SATA3_OOB_CTRL1_BURST_MAX_SHIFT		12
#define SATA3_OOB_CTRL1_BURST_MIN_MASK		0xf
#define SATA3_OOB_CTRL1_BURST_MIN_SHIFT		8
#define SATA3_OOB_CTRL1_WAKE_IDLE_MAX_MASK	0xf
#define SATA3_OOB_CTRL1_WAKE_IDLE_MAX_SHIFT	4
#define SATA3_OOB_CTRL1_WAKE_IDLE_MIN_MASK	0xf
#define SATA3_OOB_CTRL1_WAKE_IDLE_MIN_SHIFT	0
#define SATA3_OOB_CTRL2				0x1
#define SATA3_OOB_CTRL2_SEL_ENA_SHIFT		15
#define SATA3_OOB_CTRL2_SEL_ENA_RC_SHIFT	14
#define SATA3_OOB_CTRL2_RESET_IDLE_MAX_MASK	0x3f
#define SATA3_OOB_CTRL2_RESET_IDLE_MAX_SHIFT	8
#define SATA3_OOB_CTRL2_BURST_CNT_MASK		0x3
#define SATA3_OOB_CTRL2_BURST_CNT_SHIFT		6
#define SATA3_OOB_CTRL2_RESET_IDLE_MIN_MASK	0x3f
#define SATA3_OOB_CTRL2_RESET_IDLE_MIN_SHIFT	0

struct ns2_sata_port {
	int portnum;
	void __iomem *ctrl_base;
	void __iomem *pcb_base;
	struct phy *phy;
	struct ns2_sata_phy *phy_priv;
};

struct ns2_sata_phy {
	struct device *dev;
	struct ns2_sata_port ports[MAX_PORTS];
};

#define PORT_CTRL1(port)		\
	((port)->ctrl_base + PORTx_CTRL1)

#define PORT_PCB_REG(port, reg)		\
	((port)->pcb_base + PORTx_PCB_REG0 + (reg) * 4)

#define PORT_PCB_BLOCK_ADDR(port)	\
	((port)->pcb_base + PORTx_PCB_BLOCK_ADDR)

static int ns2_sata_phy_init(struct phy *phy)
{
	int try;
	unsigned int val;
	struct ns2_sata_port *port = phy_get_drvdata(phy);
	struct device *dev = port->phy_priv->dev;

	/* Configure OOB control */
	writel(SATA3_OOB_BLOCK_ADDR, PORT_PCB_BLOCK_ADDR(port));
	val = 0x0;
	val |= (0xc << SATA3_OOB_CTRL1_BURST_MAX_SHIFT);
	val |= (0x4 << SATA3_OOB_CTRL1_BURST_MIN_SHIFT);
	val |= (0x9 << SATA3_OOB_CTRL1_WAKE_IDLE_MAX_SHIFT);
	val |= (0x3 << SATA3_OOB_CTRL1_WAKE_IDLE_MIN_SHIFT);
	writel(val, PORT_PCB_REG(port, SATA3_OOB_CTRL1));
	val = 0x0;
	val |= (0x1b << SATA3_OOB_CTRL2_RESET_IDLE_MAX_SHIFT);
	val |= (0x2 << SATA3_OOB_CTRL2_BURST_CNT_SHIFT);
	val |= (0x9 << SATA3_OOB_CTRL2_RESET_IDLE_MIN_SHIFT);
	writel(val, PORT_PCB_REG(port, SATA3_OOB_CTRL2));

	/* Configure PHY PLL register bank 1 */
	writel(SATA3_PLL1_BLOCK_ADDR, PORT_PCB_BLOCK_ADDR(port));
	val = SATA3_PLL1_ACTRL2_MAGIC;
	writel(val, PORT_PCB_REG(port, SATA3_PLL1_ACTRL2));
	val = SATA3_PLL1_ACTRL3_MAGIC;
	writel(val, PORT_PCB_REG(port, SATA3_PLL1_ACTRL3));
	val = SATA3_PLL1_ACTRL4_MAGIC;
	writel(val, PORT_PCB_REG(port, SATA3_PLL1_ACTRL4));

	/* Configure PHY BLOCK0 register bank */
	writel(SATA3_BLOCK0_BLOCK_ADDR, PORT_PCB_BLOCK_ADDR(port));
	/* Set oob_clk_sel to refclk/2 */
	val = readl(PORT_PCB_REG(port, SATA3_BLOCK0_SPARE));
	val &= ~SATA3_BLOCK0_SPARE_OOB_CLK_SEL_MASK;
	val |= SATA3_BLOCK0_SPARE_OOB_CLK_SEL_REFBY2;
	writel(val, PORT_PCB_REG(port, SATA3_BLOCK0_SPARE));

	/* Strobe PHY reset */
	writel(PORTx_CTRL1_RESET, PORT_CTRL1(port));
	mdelay(1);
	writel(0x0, PORT_CTRL1(port));
	mdelay(1);

	/* Wait for PHY PLL lock */
	writel(SATA3_BLOCK0_BLOCK_ADDR, PORT_PCB_BLOCK_ADDR(port));
	/* Poll for pll_lock bit */
	try = 50;
	while (try) {
		if (readl(PORT_PCB_REG(port, SATA3_BLOCK0_XGXSSTATUS)) &
			SATA3_BLOCK0_XGXSSTATUS_PLL_LOCK) {
			break;
		}
		msleep(20);
		try--;
	}
	if (!try) {
		/* PLL did not lock; give up */
		dev_err(dev, "port%d PLL did not lock\n", port->portnum);
		return -EIO;
	}

	dev_dbg(dev, "port%d initialized\n", port->portnum);

	return 0;
}

static struct phy_ops ns2_phy_ops = {
	.init		= ns2_sata_phy_init,
	.owner		= THIS_MODULE,
};

static int ns2_sata_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn = dev->of_node, *child;
	struct phy_provider *provider;
	struct ns2_sata_phy *priv;
	struct resource *res;
	void __iomem *phy_base;
	int count = 0;

	if (of_get_child_count(dn) == 0)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy");
	phy_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy_base))
		return PTR_ERR(phy_base);

	for_each_available_child_of_node(dn, child) {
		unsigned int id;
		struct ns2_sata_port *port;

		if (of_property_read_u32(child, "reg", &id)) {
			dev_err(dev, "missing reg property in node %s\n",
					child->name);
			return -EINVAL;
		}

		if (id >= MAX_PORTS) {
			dev_err(dev, "invalid reg: %u\n", id);
			return -EINVAL;
		}
		if (priv->ports[id].phy) {
			dev_err(dev, "already registered port %u\n", id);
			return -EINVAL;
		}

		port = &priv->ports[id];
		port->portnum = id;
		port->ctrl_base = phy_base;
		port->ctrl_base += PORT0_CTRL_OFFSET;
		port->ctrl_base += id * PORTx_CTRL_SIZE;
		port->pcb_base = phy_base;
		port->pcb_base += PORT0_PCB_OFFSET;
		port->pcb_base += id * PORTx_PCB_SIZE;
		port->phy_priv = priv;
		port->phy = devm_phy_create(dev, child, &ns2_phy_ops);
		if (IS_ERR(port->phy)) {
			dev_err(dev, "failed to create PHY\n");
			return PTR_ERR(port->phy);
		}

		phy_set_drvdata(port->phy, port);
		count++;
	}

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(provider)) {
		dev_err(dev, "could not register PHY provider\n");
		return PTR_ERR(provider);
	}

	/*
	 * Ensure that no endianness swapping is done for DMA data,
	 * DMA desc, and MMIO accesses to/from AHCI.
	 */
	writel(TOP_BUS_CTRL_DEFAULT, phy_base + TOP_BUS_CTRL_OFFSET);

	dev_dbg(dev, "registered %d port(s)\n", count);

	return 0;
}

static const struct of_device_id ns2_sata_phy_of_match[] = {
	{ .compatible = "brcm,ns2-sata-phy" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ns2_sata_phy_of_match);

static struct platform_driver ns2_sata_phy_driver = {
	.driver = {
		.name = "ns2-sata-phy",
		.of_match_table = ns2_sata_phy_of_match,
	},
	.probe = ns2_sata_phy_probe,
};
module_platform_driver(ns2_sata_phy_driver);

MODULE_DESCRIPTION("Broadcom Northstar2 SATA PHY driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Anup Patel");
