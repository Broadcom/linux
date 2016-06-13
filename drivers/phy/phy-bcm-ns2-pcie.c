/*
 * Copyright (C) 2016 Broadcom
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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/phy/phy.h>
#include <linux/shared_mdio.h>

struct ns2_pci_phy {
	struct shared_mdio_master *master;
	struct phy *phy;
	unsigned int phy_id;
};

#define BLK_ADDR_REG_OFFSET	0x1f
#define PLL_AFE1_100MHZ_BLK	0x2100
#define PLL_CLK_AMP_OFFSET	0x03
#define PLL_CLK_AMP_2P05V	0x2b18

static int ns2_pci_phy_init(struct phy *p)
{
	struct ns2_pci_phy *phy = phy_get_drvdata(p);
	struct device *dev = &phy->master->dev;
	int rc;

	/* select the AFE 100MHz block page */
	rc = shared_mdio_write(phy->master, phy->phy_id, BLK_ADDR_REG_OFFSET,
			       PLL_AFE1_100MHZ_BLK);
	if (rc)
		goto err;

	/* set the 100 MHz reference clock amplitude to 2.05 v */
	rc = shared_mdio_write(phy->master, phy->phy_id, PLL_CLK_AMP_OFFSET,
			       PLL_CLK_AMP_2P05V);
	if (rc)
		goto err;

	return 0;

err:
	dev_err(dev, "Error %d writing to phy\n", rc);
	return rc;
}

static struct phy_ops ns2_pci_phy_ops = {
	.init = ns2_pci_phy_init,
};

static int ns2_pci_phy_probe(struct shared_mdio_master *master)
{
	struct device *dev = &master->dev;
	struct device_node *dn = dev->of_node, *child;
	struct phy_provider *provider;
	struct ns2_pci_phy *p;
	struct phy *phy;
	int rc, phy_id;

	for_each_available_child_of_node(dn, child) {
		rc = of_property_read_u32(child, "reg", &phy_id);
		if (rc < 0)
			goto put_child;

		if (phy_id >= PHY_MAX_ADDR) {
			dev_err(dev, "invalid Phy id: %u\n", phy_id);
			rc = -EINVAL;
			goto put_child;
		}

		phy = devm_phy_create(dev, child, &ns2_pci_phy_ops);
		if (IS_ERR_OR_NULL(phy)) {
			dev_err(dev, "failed to create Phy\n");
			rc = PTR_ERR(phy);
			goto put_child;
		}

		p = devm_kmalloc(dev, sizeof(struct ns2_pci_phy),
				 GFP_KERNEL);
		if (!p) {
			rc = -ENOMEM;
			goto put_child;
		}

		p->master = master;
		p->phy_id = phy_id;
		p->phy = phy;

		phy_set_drvdata(phy, p);

		provider = devm_of_phy_provider_register(&phy->dev,
							 of_phy_simple_xlate);
		if (IS_ERR(provider)) {
			dev_err(dev, "failed to register Phy provider\n");
			rc = PTR_ERR(provider);
			goto put_child;
		}

		dev_info(dev, "%s found\n", child->name);
	}

	return 0;

put_child:
	of_node_put(child);
	return rc;
}

static const struct of_device_id ns2_pci_phy_of_match[] = {
	{ .compatible = "brcm,ns2-pcie-phy", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ns2_pci_phy_of_match);

static struct shared_mdio_driver ns2_pci_phy_driver = {
	.driver = {
		.name = "phy-bcm-ns2-pci",
		.of_match_table = ns2_pci_phy_of_match,
	},
	.probe = ns2_pci_phy_probe,
};
module_shared_mdio_driver(ns2_pci_phy_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Northstar2 PCI Phy driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:phy-bcm-ns2-pci");
