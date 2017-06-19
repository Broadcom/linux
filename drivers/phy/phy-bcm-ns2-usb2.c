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

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>

/* USB2 IDM Control */
#define IDM_IO_CTL_OFFSET		0x0
#define IDM_CLK_EN			BIT(0)
#define IDM_L2_SUSPEND_OVERRIDE		BIT(13)
#define IDM_L1_SUSPEND_OVERRIDE		BIT(14)
#define IDM_L1_SLEEP_OVERRIDE		BIT(15)

#define IDM_RST_CTL_OFFSET		0x3f8
#define IDM_RST_CTL			BIT(0)

/* USB2 PHY Control */
#define PHY_CTRL_OFFSET			0x0
#define PHY_PLL_PWR_DN			0
#define PHY_PORT_UTMI_PWR_DN		2
#define PHY_PORT_PWR_DN			4
#define PHY_SOFT_RST			6
#define PHY_CORE_RST			8
#define PHY_HARD_RST			9

#define PHY_STRAP_OFFSET		0x500
#define OHCI_EHCI_OVRCUR_POL		BIT(11)
#define OHCI_EHCI_PPC_INV		BIT(12)

/* USB2 CRMU Control */
#define USB2_OFFSET			0x4

#define PLL_LOCK_STS_BIT		BIT(0)
#define PLL_LOCK_RETRY			100

struct ns2_usb2_phy {
	struct phy *phy;
	struct clk *clk;
	void __iomem *idm_regs;
	void __iomem *phy_regs;
	void __iomem *icfg_sts;
	void __iomem *crmu_ctl;
	bool inv_ppc;
	u32 afe_val;
};

static int ns2_usb2_exit(struct phy *phy)
{
	struct ns2_usb2_phy *data = phy_get_drvdata(phy);
	u32 val;

	regmap_read(data->crmu_ctl, USB2_OFFSET, &val);
	val &= ~BIT(data->afe_val);
	regmap_write(data->crmu_ctl, USB2_OFFSET, val);

	val = readl(data->idm_regs + IDM_IO_CTL_OFFSET);
	val &= ~(IDM_CLK_EN);
	writel(val, (data->idm_regs + IDM_IO_CTL_OFFSET));

	val = readl(data->idm_regs + IDM_RST_CTL_OFFSET);
	val |= IDM_RST_CTL;
	writel(val, (data->idm_regs + IDM_RST_CTL_OFFSET));

	return 0;
}

static int ns2_usb2_init(struct phy *phy)
{
	struct ns2_usb2_phy *data = phy_get_drvdata(phy);
	u32 val;
	u32 i = 0;

	val = readl(data->idm_regs + IDM_RST_CTL_OFFSET);
	val |= IDM_RST_CTL;
	writel(val, (data->idm_regs + IDM_RST_CTL_OFFSET));

	val = readl(data->idm_regs + IDM_IO_CTL_OFFSET);
	val &= ~(IDM_CLK_EN);
	writel(val, (data->idm_regs + IDM_IO_CTL_OFFSET));

	regmap_read(data->crmu_ctl, USB2_OFFSET, &val);
	val |= BIT(data->afe_val);
	regmap_write(data->crmu_ctl, USB2_OFFSET, val);

	do {
		usleep_range(10, 100);
		val = readl(data->icfg_sts);
		if (i >= PLL_LOCK_RETRY) {
			dev_err(&phy->dev, "failed to get PLL lock\n");
			return -ETIMEDOUT;
		}
		i++;
	} while (!(val & PLL_LOCK_STS_BIT));

	val = readl(data->idm_regs + IDM_IO_CTL_OFFSET);
	val |= IDM_CLK_EN;
	writel(val, (data->idm_regs + IDM_IO_CTL_OFFSET));

	val = readl(data->idm_regs + IDM_RST_CTL_OFFSET);
	val &= ~(IDM_RST_CTL);
	writel(val, (data->idm_regs + IDM_RST_CTL_OFFSET));

	if (data->inv_ppc) {
		val = readl(data->phy_regs + PHY_STRAP_OFFSET);
		val |= OHCI_EHCI_OVRCUR_POL;
		val |= OHCI_EHCI_PPC_INV;
		writel(val, (data->phy_regs + PHY_STRAP_OFFSET));
	}

	writel(((1 << PHY_HARD_RST) |
		(1 << PHY_CORE_RST) |
		(0x3 << PHY_SOFT_RST) |
		(0x3 << PHY_PORT_PWR_DN) |
		(0x3 << PHY_PORT_UTMI_PWR_DN) |
		(0x3 << PHY_PLL_PWR_DN)),
		(data->phy_regs + PHY_CTRL_OFFSET));

	/*
	 * USB PHY Sleep and Suspend override:
	 * The following settings override the controller's UTMI suspend
	 * and sleep signals going to PHY.
	 * This is required to avoid kernel crash occurred when PHY enters
	 * suspend (because of EHCI entering suspend), but OHCI is still
	 * active. Same PHY is used for both EHCI and OHCI controllers.
	 */
	val = readl(data->idm_regs + IDM_IO_CTL_OFFSET);
	val |= IDM_L1_SLEEP_OVERRIDE;
	val |= IDM_L1_SUSPEND_OVERRIDE;
	val |= IDM_L2_SUSPEND_OVERRIDE;
	writel(val, (data->idm_regs + IDM_IO_CTL_OFFSET));

	return 0;
}

static const struct phy_ops ops = {
	.init		= ns2_usb2_init,
	.exit		= ns2_usb2_exit,
	.owner		= THIS_MODULE,
};

static const struct of_device_id ns2_usb2_dt_ids[] = {
	{ .compatible = "brcm,ns2-usb2-phy", },
	{ }
};
MODULE_DEVICE_TABLE(of, ns2_usb2_dt_ids);

static int ns2_usb2_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct ns2_usb2_phy *data;
	struct resource *res;
	int ret;

	data = devm_kzalloc(dev, sizeof(struct ns2_usb2_phy), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "usb2-idm-ctl");
	data->idm_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->idm_regs))
		return PTR_ERR(data->idm_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "usb2-phy-ctl");
	data->phy_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->phy_regs))
		return PTR_ERR(data->phy_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "usb2-icfg-sts");
	data->icfg_sts = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->icfg_sts))
		return PTR_ERR(data->icfg_sts);

	data->crmu_ctl = syscon_regmap_lookup_by_phandle(dev->of_node,
							 "usb-ctl-syscon");
	if (IS_ERR(data->crmu_ctl))
		return PTR_ERR(data->crmu_ctl);

	ret = of_property_read_u32(dev->of_node, "brcm,ns2-usbh-afe",
				   &data->afe_val);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't read 'afe_val' property\n");
		return -EIO;
	}

	if (of_property_read_bool(dev->of_node, "brcm,ns2-inv-ppc-pol"))
		data->inv_ppc = true;

	data->phy = devm_phy_create(dev, dev->of_node, &ops);
	if (IS_ERR(data->phy)) {
		dev_err(dev, "Failed to create usb phy\n");
		return PTR_ERR(data->phy);
	}

	phy_set_drvdata(data->phy, data);
	platform_set_drvdata(pdev, data);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		dev_err(dev, "Failed to register as phy provider\n");
		return PTR_ERR(phy_provider);
	}

	dev_info(dev, "Registered NS2 USB2H Phy\n");
	return 0;
}

static struct platform_driver ns2_usb2_driver = {
	.probe = ns2_usb2_probe,
	.driver = {
		.name = "bcm-ns2-usb2phy",
		.of_match_table = of_match_ptr(ns2_usb2_dt_ids),
	},
};
module_platform_driver(ns2_usb2_driver);

MODULE_ALIAS("platform:bcm-ns2-usb2phy");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom NS2 USB2 PHY driver");
MODULE_LICENSE("GPL v2");

