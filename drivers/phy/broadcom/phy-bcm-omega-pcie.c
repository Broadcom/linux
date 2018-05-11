/*
 * Copyright (C) 2018 Broadcom
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

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mfd/syscon.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define MISC_AON_CTRL_OFFSET	0x884
#define PCIE_SERDES_IDDQ_SHIFT	BIT(0)

struct omega_pcie_phy_core;

/**
 * struct omega_pcie_phy - Omega PCIe PHY device
 * @core: pointer to the Omega PCIe PHY core control
 * @phy: pointer to the kernel PHY device
 */
struct omega_pcie_phy {
	struct omega_pcie_phy_core *core;
	struct phy *phy;
};

/**
 * struct omega_pcie_phy_core - Omega PCIe PHY core control
 * @dev: pointer to device
 * @lock: mutex to protect access to individual PHYs
 * @phys: pointer to Cygnus PHY device
 */
struct omega_pcie_phy_core {
	struct device *dev;
	struct regmap *crmu_regmap;
	struct omega_pcie_phy phy;
};

static int omega_pcie_power_config(struct omega_pcie_phy *phy, bool enable)
{
	struct omega_pcie_phy_core *core = phy->core;

	if (enable) {
		regmap_update_bits(core->crmu_regmap, MISC_AON_CTRL_OFFSET,
			PCIE_SERDES_IDDQ_SHIFT,
			0);
		/*
		 * Wait 50 ms for the PCIe Serdes to stabilize after the analog
		 * front end is brought up
		 */
		msleep(50);
	} else
		regmap_update_bits(core->crmu_regmap, MISC_AON_CTRL_OFFSET,
			PCIE_SERDES_IDDQ_SHIFT,
			PCIE_SERDES_IDDQ_SHIFT);

	dev_dbg(core->dev, "PCIe PHY %s\n", enable ? "enabled" : "disabled");
	return 0;
}

static int omega_pcie_phy_power_on(struct phy *p)
{
	struct omega_pcie_phy *phy = phy_get_drvdata(p);

	return omega_pcie_power_config(phy, true);
}

static int omega_pcie_phy_power_off(struct phy *p)
{
	struct omega_pcie_phy *phy = phy_get_drvdata(p);

	return omega_pcie_power_config(phy, false);
}

static const struct phy_ops omega_pcie_phy_ops = {
	.power_on = omega_pcie_phy_power_on,
	.power_off = omega_pcie_phy_power_off,
	.owner = THIS_MODULE,
};

static int omega_pcie_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node, *child;
	struct omega_pcie_phy_core *core;
	struct phy_provider *provider;
	struct omega_pcie_phy *p;
	unsigned int id;
	int ret;

	if (of_get_child_count(node) != 1) {
		dev_err(dev, "PHY should have exactly one child node\n");
		return -ENODEV;
	}

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->dev = dev;

	core->crmu_regmap = syscon_regmap_lookup_by_phandle(dev->of_node,
				"brcm,pciephy-crmu-syscon");
	if (IS_ERR(core->crmu_regmap)) {
		dev_err(dev, "crmu regmap failed\n");
		return PTR_ERR(core->crmu_regmap);
	}

	child = of_get_next_child(node, NULL);
	if (!child) {
		dev_err(dev, "No PHY node found\n");
		return PTR_ERR(child);
	}

	if (of_property_read_u32(child, "reg", &id)) {
		dev_err(dev, "missing reg property for %s\n", child->name);
		ret = -EINVAL;
		goto put_child;
	}

	if (id > 0) {
		dev_err(dev, "invalid PHY id: %u\n", id);
		ret = -EINVAL;
		goto put_child;
	}

	p = &core->phy;
	p->phy = devm_phy_create(dev, child, &omega_pcie_phy_ops);
	if (IS_ERR(p->phy)) {
		dev_err(dev, "failed to create PHY\n");
		ret = PTR_ERR(p->phy);
		goto put_child;
	}

	p->core = core;
	phy_set_drvdata(p->phy, p);

	dev_set_drvdata(dev, core);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(provider)) {
		dev_err(dev, "failed to register PHY provider\n");
		ret = PTR_ERR(provider);
		goto put_child;
	}

	dev_dbg(dev, "registered PCIe PHY\n");

	return 0;
put_child:
	of_node_put(child);
	return ret;
}

static const struct of_device_id omega_pcie_phy_match_table[] = {
	{ .compatible = "brcm,omega-pcie-phy" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, omega_pcie_phy_match_table);

static struct platform_driver omega_pcie_phy_driver = {
	.driver = {
		.name = "omega-pcie-phy",
		.of_match_table = omega_pcie_phy_match_table,
	},
	.probe = omega_pcie_phy_probe,
};
module_platform_driver(omega_pcie_phy_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Cygnus PCIe PHY driver");
MODULE_LICENSE("GPL v2");
