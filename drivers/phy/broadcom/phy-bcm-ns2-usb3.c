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
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>

#define PHY_MAX_PORTS			2

/* USB3 ICFG */
#define ICFG_PHY_CFG_CTRL		0x00
#define ICFG_PHY_CFG_CTRL_MASK		(BIT(3) | BIT(4) | BIT(5))
#define ICFG_PHY_PLL_SEQ_START		BIT(6)

#define ICFG_PHY_P0CTL			0x04
#define ICFG_PHY_P1CTL			0x08
#define ICFG_PHY_PXCTL_I		BIT(1)

#define ICFG_PHY_MISC_STS		0x10

/* USB3 IDM */
#define IDM_RST_CTRL_P0			0x3f8
#define IDM_RST_CTRL_P1			0x13f8
#define IDM_RESET_CTRL			BIT(0)

#define IDM_IO_CTRL_P0			0x0
#define IDM_IO_CTRL_P1			0x1000
#define IDM_IO_CTRL_PPC_CFG		(BIT(23) | BIT(24))

/* USB3 CRMU */
#define CRMU_PHY_RST			BIT(5)
#define CRMU_PHY_PLL_RST		BIT(6)

/* USB3 MDIO */
#define MDIO_BLK_ACCESS			0x1F
#define MDIO_PLL30_ADDR			0x8000
#define PLL30_ANAPLL_CTRL		0x14
#define PLL30_ANAPLL_VAL		0x23
#define PLL30_GEN_PLL			0xF
#define PLL30_GEN_PLL_PCLK_SEL		BIT(11)

#define MDIO_P0_AFE30_ADDR		0x8080
#define MDIO_P1_AFE30_ADDR		0x9080
#define AFE30_RX_DET			0x5
#define AFE30_RX_DET_VAL		0xAC0D

#define MDIO_P0_PIPE_BLK_ADDR		0x8060
#define MDIO_P1_PIPE_BLK_ADDR		0x9060
#define PIPE_REG_1_OFFSET		0x1
#define PIPE_REG_1_VAL			0x207

#define MDIO_P0_AEQ_BLK_ADDR		0x80E0
#define MDIO_P1_AEQ_BLK_ADDR		0x90E0
#define AEQ_REG_1_OFFSET		0x1
#define AEQ_REG_1_VAL			0x3000

enum ns2_phy_block {
	PHY_RESET,
	PHY_PLL_RESET,
	PHY_SOFT_RESET,
	PHY_PIPE_RESET,
	PHY_REF_CLOCK,
	PHY_PLL_SEQ_START,
	PHY_PLL_STATUS,
	PHY_VBUS_PPC,
};

enum ns2_reg_base {
	USB3_CRMU_CTL = 1,
	USB3_ICFG_REGS,
	USB3_IDM_REGS,
	USB3_REG_BASE_MAX
};

struct ns2_usb3_phy {
	void __iomem *reg_base[USB3_REG_BASE_MAX];
	struct ns2_usb3_phy_master *mphy;
	struct phy *phy;
	int port_no;
	bool inv_ppc;
};

struct ns2_usb3_phy_master {
	struct ns2_usb3_phy iphys[PHY_MAX_PORTS];
	struct mdio_device *mdiodev;
	struct mutex phy_mutex;
	/*
	 * port_cnt for tracking init/exit for master phy (both ports).
	 * Some common resets in init/exit code needs to be done just
	 * once.
	 */
	int port_cnt;
};

static int ns2_phy_write(struct ns2_usb3_phy *iphy, enum ns2_phy_block block,
			 bool assert)
{
	void __iomem *addr;
	u32  data, count;
	u32 offset = 0;
	int ret;

	switch (block) {
	case PHY_RESET:
		addr = iphy->reg_base[USB3_CRMU_CTL];

		ret = regmap_read(addr, offset, &data);
		if (ret != 0)
			return ret;

		if (assert)
			data &= ~CRMU_PHY_RST;
		else
			data |= CRMU_PHY_RST;

		ret = regmap_write(addr, offset, data);
		break;

	case PHY_PLL_RESET:
		addr = iphy->reg_base[USB3_CRMU_CTL];

		ret = regmap_read(addr, offset, &data);
		if (ret != 0)
			return ret;

		if (assert)
			data &= ~CRMU_PHY_PLL_RST;
		else
			data |= CRMU_PHY_PLL_RST;

		ret = regmap_write(addr, offset, data);
		break;

	case PHY_SOFT_RESET:
		addr = iphy->reg_base[USB3_ICFG_REGS];
		offset = ICFG_PHY_P0CTL;

		ret = regmap_read(addr, offset, &data);
		if (ret != 0)
			return ret;

		if (assert)
			data &= ~ICFG_PHY_PXCTL_I;
		else
			data |= ICFG_PHY_PXCTL_I;

		ret = regmap_write(addr, offset, data);
		if (ret != 0)
			return ret;

		offset = ICFG_PHY_P1CTL;

		ret = regmap_read(addr, offset, &data);
		if (ret != 0)
			return ret;

		if (assert)
			data &= ~ICFG_PHY_PXCTL_I;
		else
			data |= ICFG_PHY_PXCTL_I;

		ret = regmap_write(addr, offset, data);
		break;

	case PHY_PIPE_RESET:
		addr = iphy->reg_base[USB3_IDM_REGS];
		offset = IDM_RST_CTRL_P0;

		ret = regmap_read(addr, offset, &data);
		if (ret != 0)
			return ret;

		if (assert)
			data |= IDM_RESET_CTRL;
		else
			data &= ~IDM_RESET_CTRL;

		ret = regmap_write(addr, offset, data);
		if (ret != 0)
			return ret;

		offset = IDM_RST_CTRL_P1;
		ret = regmap_read(addr, offset, &data);
		if (ret != 0)
			return ret;

		if (assert)
			data |= IDM_RESET_CTRL;
		else
			data &= ~IDM_RESET_CTRL;

		ret = regmap_write(addr, offset, data);
		break;

	case PHY_VBUS_PPC:
		addr = iphy->reg_base[USB3_IDM_REGS];
		offset = IDM_IO_CTRL_P0;

		ret = regmap_read(addr, offset, &data);
		if (ret != 0)
			return ret;

		if (assert)
			data |= IDM_IO_CTRL_PPC_CFG;
		else
			data &= ~IDM_IO_CTRL_PPC_CFG;

		ret = regmap_write(addr, offset, data);
		if (ret != 0)
			return ret;

		offset = IDM_IO_CTRL_P1;
		ret = regmap_read(addr, offset, &data);
		if (ret != 0)
			return ret;

		if (assert)
			data |= IDM_IO_CTRL_PPC_CFG;
		else
			data &= ~IDM_IO_CTRL_PPC_CFG;

		ret = regmap_write(addr, offset, data);
		break;

	case PHY_REF_CLOCK:
		addr = iphy->reg_base[USB3_ICFG_REGS];
		offset = ICFG_PHY_CFG_CTRL;

		ret = regmap_read(addr, offset, &data);
		if (ret != 0)
			return ret;

		data &= ~ICFG_PHY_CFG_CTRL_MASK;

		ret = regmap_write(addr, offset, data);
		break;

	case PHY_PLL_SEQ_START:
		addr = iphy->reg_base[USB3_ICFG_REGS];
		offset = ICFG_PHY_CFG_CTRL;

		ret = regmap_read(addr, offset, &data);
		if (ret != 0)
			return ret;

		data |= ICFG_PHY_PLL_SEQ_START;

		ret = regmap_write(addr, offset, data);
		break;

	case PHY_PLL_STATUS:
		count = 200;
		addr = iphy->reg_base[USB3_ICFG_REGS];
		offset = ICFG_PHY_MISC_STS;

		do {
			usleep_range(1, 10);
			ret = regmap_read(addr, offset, &data);
			if (ret != 0)
				return ret;

			if (data == 1)
				break;
		} while (--count);

		if (!count)
			ret = -ETIMEDOUT;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ns2_usb3_exit(struct phy *phy)
{
	struct ns2_usb3_phy *iphy = phy_get_drvdata(phy);
	int ret = 0;

	mutex_lock(&iphy->mphy->phy_mutex);

	if (iphy->mphy->port_cnt == 1) {
		/* Only put in to reset for last port to exit */
		ret = ns2_phy_write(iphy, PHY_PLL_RESET, true);
		if (ret)
			goto out;

		ret = ns2_phy_write(iphy, PHY_SOFT_RESET, true);
		if (ret)
			goto out;

		ret = ns2_phy_write(iphy, PHY_RESET, true);
		if (ret)
			goto out;

		ret = ns2_phy_write(iphy, PHY_PIPE_RESET, true);
		if (ret)
			goto out;
	}

out:
	iphy->mphy->port_cnt--;
	mutex_unlock(&iphy->mphy->phy_mutex);

	return ret;
}

static int ns2_usb3_init(struct phy *phy)
{
	struct ns2_usb3_phy *iphy = phy_get_drvdata(phy);
	struct mdio_device *mdev = iphy->mphy->mdiodev;
	int ret = 0;
	u16 addr;
	u16 val;

	mutex_lock(&iphy->mphy->phy_mutex);

	if (iphy->mphy->port_cnt)
		goto out;

	ret = ns2_phy_write(iphy, PHY_RESET, false);
	if (ret)
		goto out;

	ret = ns2_phy_write(iphy, PHY_SOFT_RESET, true);
	if (ret)
		goto out;

	ret = ns2_phy_write(iphy, PHY_PIPE_RESET, true);
	if (ret)
		goto out;

	ret = ns2_phy_write(iphy, PHY_REF_CLOCK, true);
	if (ret)
		goto out;

	ret = ns2_phy_write(iphy, PHY_PLL_RESET, true);
	if (ret)
		goto out;

	ret = ns2_phy_write(iphy, PHY_RESET, true);
	if (ret)
		goto out;

	ret = ns2_phy_write(iphy, PHY_RESET, false);
	if (ret)
		goto out;

	/* PLL programming */
	/* PHY PLL30 Block */
	ret = mdiobus_write(mdev->bus, mdev->addr, MDIO_BLK_ACCESS,
			   MDIO_PLL30_ADDR);
	if (ret)
		goto out;

	ret = mdiobus_write(mdev->bus, mdev->addr, PLL30_ANAPLL_CTRL,
			   PLL30_ANAPLL_VAL);
	if (ret)
		goto out;

	val = (u16) mdiobus_read(mdev->bus, mdev->addr, PLL30_GEN_PLL);
	val |= PLL30_GEN_PLL_PCLK_SEL;
	ret = mdiobus_write(mdev->bus, mdev->addr, PLL30_GEN_PLL, val);
	if (ret)
		goto out;

	/* PHY AFE30 Block */
	addr = MDIO_P0_AFE30_ADDR;
	ret = mdiobus_write(mdev->bus, mdev->addr, MDIO_BLK_ACCESS, addr);
	if (ret)
		goto out;

	ret = mdiobus_write(mdev->bus, mdev->addr, AFE30_RX_DET,
			   AFE30_RX_DET_VAL);
	if (ret)
		goto out;

	addr = MDIO_P1_AFE30_ADDR;
	ret = mdiobus_write(mdev->bus, mdev->addr, MDIO_BLK_ACCESS, addr);
	if (ret)
		goto out;

	ret = mdiobus_write(mdev->bus, mdev->addr, AFE30_RX_DET,
			   AFE30_RX_DET_VAL);
	if (ret)
		goto out;

	/* PHY PIPE Block */
	addr = MDIO_P0_PIPE_BLK_ADDR;
	ret = mdiobus_write(mdev->bus, mdev->addr, MDIO_BLK_ACCESS, addr);
	if (ret)
		goto out;

	ret = mdiobus_write(mdev->bus, mdev->addr, PIPE_REG_1_OFFSET,
			   PIPE_REG_1_VAL);
	if (ret)
		goto out;

	addr = MDIO_P1_PIPE_BLK_ADDR;
	ret = mdiobus_write(mdev->bus, mdev->addr, MDIO_BLK_ACCESS, addr);
	if (ret)
		goto out;

	ret = mdiobus_write(mdev->bus, mdev->addr, PIPE_REG_1_OFFSET,
			   PIPE_REG_1_VAL);
	if (ret)
		goto out;

	/* AEQ Block */
	addr = MDIO_P0_AEQ_BLK_ADDR;
	ret = mdiobus_write(mdev->bus, mdev->addr, MDIO_BLK_ACCESS, addr);
	if (ret)
		goto out;

	ret = mdiobus_write(mdev->bus, mdev->addr, AEQ_REG_1_OFFSET,
			   AEQ_REG_1_VAL);
	if (ret)
		goto out;

	addr = MDIO_P1_AEQ_BLK_ADDR;
	ret = mdiobus_write(mdev->bus, mdev->addr, MDIO_BLK_ACCESS, addr);
	if (ret)
		goto out;

	ret = mdiobus_write(mdev->bus, mdev->addr, AEQ_REG_1_OFFSET,
			   AEQ_REG_1_VAL);
	if (ret)
		goto out;

	ret = ns2_phy_write(iphy, PHY_PLL_SEQ_START, true);
	if (ret)
		goto out;

	ret = ns2_phy_write(iphy, PHY_PIPE_RESET, false);
	if (ret)
		goto out;

	ret = ns2_phy_write(iphy, PHY_SOFT_RESET, false);
	if (ret)
		goto out;

	ret = ns2_phy_write(iphy, PHY_PLL_RESET, false);
	if (ret)
		goto out;

	ret = ns2_phy_write(iphy, PHY_PLL_STATUS, true);
	if (ret)
		goto out;

	if (iphy->inv_ppc)
		ret = ns2_phy_write(iphy, PHY_VBUS_PPC, false);
	else
		ret = ns2_phy_write(iphy, PHY_VBUS_PPC, true);

out:
	iphy->mphy->port_cnt++;
	mutex_unlock(&iphy->mphy->phy_mutex);

	return ret;
}

static const struct phy_ops ns2_usb3_ops = {
	.init = ns2_usb3_init,
	.exit = ns2_usb3_exit,
	.owner = THIS_MODULE,
};

static int ns2_usb3_probe(struct mdio_device *mdiodev)
{
	struct device *dev = &mdiodev->dev;
	struct device_node *dn = dev->of_node, *child;
	struct ns2_usb3_phy_master *mphy;
	struct phy_provider *provider;
	int cnt;

	mphy = devm_kzalloc(dev, sizeof(*mphy), GFP_KERNEL);
	if (!mphy)
		return -ENOMEM;
	mphy->mdiodev = mdiodev;
	mutex_init(&mphy->phy_mutex);
	mphy->port_cnt = 0;

	cnt = 0;
	for_each_available_child_of_node(dn, child) {
		struct ns2_usb3_phy *iphy;
		unsigned int val;
		struct regmap *io;

		if (cnt >= PHY_MAX_PORTS) {
			dev_err(dev, "Invalid port count\n");
			return -EINVAL;
		}

		iphy = &mphy->iphys[cnt];
		if (of_property_read_u32(child, "reg", &val)) {
			dev_err(dev, "missing reg property in node %s\n",
					child->name);
			return -EINVAL;
		}
		iphy->port_no = val;
		iphy->mphy = mphy;

		if (of_property_read_bool(child, "brcm,ns2-inv-vbus-ppc-pol"))
			iphy->inv_ppc = true;

		io = syscon_regmap_lookup_by_compatible("brcm,ns2-crmu-usbctl");
		if (IS_ERR(io))
			return PTR_ERR(io);
		iphy->reg_base[USB3_CRMU_CTL] = io;

		io = syscon_regmap_lookup_by_phandle(dn, "usb3-icfg-syscon");
		if (IS_ERR(io))
			return PTR_ERR(io);
		iphy->reg_base[USB3_ICFG_REGS] = io;

		io = syscon_regmap_lookup_by_phandle(dn, "usb3-idm-syscon");
		if (IS_ERR(io))
			return PTR_ERR(io);
		iphy->reg_base[USB3_IDM_REGS] = io;

		iphy->phy = devm_phy_create(dev, child, &ns2_usb3_ops);
		if (IS_ERR(iphy->phy)) {
			dev_err(dev, "failed to create PHY\n");
			return PTR_ERR(iphy->phy);
		}

		phy_set_drvdata(iphy->phy, iphy);
		cnt++;
	}

	dev_set_drvdata(dev, mphy);
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(provider)) {
		dev_err(dev, "could not register PHY provider\n");
		return PTR_ERR(provider);
	}

	dev_info(dev, "registered %d phy(s)\n", cnt);
	return 0;
}

static const struct of_device_id ns2_usb3_of_match[] = {
	{.compatible = "brcm,ns2-usb3-phy",},
	{ }
};
MODULE_DEVICE_TABLE(of, ns2_usb3_of_match);

static struct mdio_driver ns2_usb3_driver = {
	.mdiodrv = {
		.driver = {
			.name = "ns2-usb3-phy",
			.of_match_table = ns2_usb3_of_match,
		},
	},
	.probe = ns2_usb3_probe,
};
mdio_module_driver(ns2_usb3_driver);

MODULE_DESCRIPTION("Broadcom NS2 USB3 PHY driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Broadcom");
