/*
 * Copyright 2018 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 (GPLv2) for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 (GPLv2) along with this source code.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define HS_SS_ICFG_USB20PHY_PLL_CTRL3_OFFSET		0x18
#define PLL_SUSPEND_EN					BIT(0)

#define HHS_SS_ICFG_USB20PHY_STATUS_OFFSET		0x1C
#define USBPHY_PLL_LOCK					BIT(0)

#define HS_SS_ICFG_USB2H_MISC_CTRL_OFFSET		0x30
#define UTMI_SUSPEND_MODE				BIT(0)
#define UTMI_SLEEP_MODE					BIT(1)
#define OVR_CUR_STS					BIT(6)

#define HS_SS_ICFG_USB2H_CTRL0_OFFSET			0x38
#define CLOCK_ENABLE					BIT(0)
#define SUSPEND_OVERRIDE_0				BIT(13)

#define CRMU_USB_PHY_AON_CTRL_OFFSET			0x888
#define CRMU_USBPHY_AFE_CORERDY_VDDC			BIT(1)
#define CRMU_USBPHY_RESETB				BIT(2)

#define IDM_USB2H_M0_IDM_RESET_CONTROL_OFFSET		0x0
#define IDM_RESET					BIT(0)

#define PLL_LOCK_RETRY_COUNT				1000
#define MAX_REGULATOR_NAME_LEN				25

struct omega_phy_instance;

struct omega_phy_driver {
	struct regmap *crmu_regmap;
	void __iomem *cdru_usbphy_regs;
	void __iomem *usb2h_idm_regs;
	struct omega_phy_instance *instance;
	struct platform_device *pdev;
};

struct omega_phy_instance {
	struct omega_phy_driver *driver;
	struct phy *generic_phy;
	struct regulator *vbus_supply;
	spinlock_t lock;
};

static inline int phy_pll_lock_poll(struct omega_phy_driver *phy_driver)
{
	/* Wait for the PLL lock status */
	int retry = PLL_LOCK_RETRY_COUNT;
	u32 reg_val;

	do {
		reg_val = readl(phy_driver->cdru_usbphy_regs +
				HHS_SS_ICFG_USB20PHY_STATUS_OFFSET);
		if (reg_val & USBPHY_PLL_LOCK)
			return 0;
		udelay(1);
	} while (--retry > 0);

	return -ETIMEDOUT;
}

static int omega_phy_shutdown(struct phy *generic_phy)
{
	u32 reg_val;
	int ret;
	unsigned long flags;
	struct omega_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct omega_phy_driver *phy_driver = instance_ptr->driver;

	if (instance_ptr->vbus_supply) {
		ret = regulator_disable(instance_ptr->vbus_supply);
		if (ret) {
			dev_err(&generic_phy->dev,
					"Failed to disable regulator\n");
			return ret;
		}
	}

	spin_lock_irqsave(&instance_ptr->lock, flags);

	/* power down the phy */
	regmap_update_bits(phy_driver->crmu_regmap,
			CRMU_USB_PHY_AON_CTRL_OFFSET,
			CRMU_USBPHY_AFE_CORERDY_VDDC | CRMU_USBPHY_RESETB,
			0);

	reg_val = readl(phy_driver->cdru_usbphy_regs +
			HS_SS_ICFG_USB2H_CTRL0_OFFSET);
	reg_val &= ~CLOCK_ENABLE;
	writel(reg_val, phy_driver->cdru_usbphy_regs +
			HS_SS_ICFG_USB2H_CTRL0_OFFSET);

	reg_val = readl(phy_driver->usb2h_idm_regs +
			IDM_USB2H_M0_IDM_RESET_CONTROL_OFFSET);
	reg_val |= IDM_RESET;
	writel(reg_val, phy_driver->usb2h_idm_regs +
			IDM_USB2H_M0_IDM_RESET_CONTROL_OFFSET);

	spin_unlock_irqrestore(&instance_ptr->lock, flags);

	return 0;
}

static int omega_phy_poweron(struct phy *generic_phy)
{
	int ret;
	unsigned long flags;
	u32 reg_val;
	struct omega_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct omega_phy_driver *phy_driver = instance_ptr->driver;

	/* Switch on the regulator only if in HOST mode */
	if (instance_ptr->vbus_supply) {
		ret = regulator_enable(instance_ptr->vbus_supply);
		if (ret) {
			dev_err(&generic_phy->dev,
				"Failed to enable regulator\n");
			goto err_shutdown;
		}
	}

	spin_lock_irqsave(&instance_ptr->lock, flags);

	/* Bring the AFE block out of reset to start powering up the PHY */
	regmap_update_bits(phy_driver->crmu_regmap,
			CRMU_USB_PHY_AON_CTRL_OFFSET,
			CRMU_USBPHY_AFE_CORERDY_VDDC | CRMU_USBPHY_RESETB,
			CRMU_USBPHY_AFE_CORERDY_VDDC | CRMU_USBPHY_RESETB);

	/* Enable clock to USB host and take the host out of reset */
	reg_val = readl(phy_driver->cdru_usbphy_regs +
			HS_SS_ICFG_USB2H_CTRL0_OFFSET);
	reg_val |= CLOCK_ENABLE | SUSPEND_OVERRIDE_0;
	writel(reg_val, phy_driver->cdru_usbphy_regs +
			HS_SS_ICFG_USB2H_CTRL0_OFFSET);

	reg_val = readl(phy_driver->usb2h_idm_regs +
			IDM_USB2H_M0_IDM_RESET_CONTROL_OFFSET);
	reg_val &= ~IDM_RESET;
	writel(reg_val, phy_driver->usb2h_idm_regs +
			 IDM_USB2H_M0_IDM_RESET_CONTROL_OFFSET);

	reg_val = readl(phy_driver->cdru_usbphy_regs +
			HS_SS_ICFG_USB20PHY_PLL_CTRL3_OFFSET);
	reg_val |= PLL_SUSPEND_EN;
	writel(reg_val, phy_driver->cdru_usbphy_regs +
			HS_SS_ICFG_USB20PHY_PLL_CTRL3_OFFSET);

	reg_val = OVR_CUR_STS | UTMI_SLEEP_MODE | UTMI_SUSPEND_MODE;
	writel(reg_val, phy_driver->cdru_usbphy_regs +
			HS_SS_ICFG_USB2H_MISC_CTRL_OFFSET);

	/* Check for PLL lock */
	ret = phy_pll_lock_poll(phy_driver);
	if (ret < 0) {
		dev_err(&generic_phy->dev,
			"Timed out waiting for USBPHY PLL LOCK");
		spin_unlock_irqrestore(&instance_ptr->lock, flags);

		goto err_shutdown;
	}
	spin_unlock_irqrestore(&instance_ptr->lock, flags);

	return 0;
err_shutdown:
	omega_phy_shutdown(generic_phy);
	return ret;
}

static const struct phy_ops ops = {
	.power_on	= omega_phy_poweron,
	.power_off	= omega_phy_shutdown,
};

static int omega_phy_instance_create(struct omega_phy_driver *phy_driver)
{
	struct device_node *child;
	struct platform_device *pdev = phy_driver->pdev;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct omega_phy_instance *instance_ptr;
	unsigned int id, ret = 0;

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

	instance_ptr = phy_driver->instance;
	instance_ptr->driver = phy_driver;
	spin_lock_init(&instance_ptr->lock);

	instance_ptr->generic_phy =
			devm_phy_create(dev, child, &ops);
	if (IS_ERR(instance_ptr->generic_phy)) {
		dev_err(dev, "Failed to create usb phy");
		ret = PTR_ERR(instance_ptr->generic_phy);
		goto put_child;
	}

	/* regulator use is optional */
	instance_ptr->vbus_supply =
		devm_regulator_get(&instance_ptr->generic_phy->dev, "vbus");
	if (IS_ERR(instance_ptr->vbus_supply))
		instance_ptr->vbus_supply = NULL;
	phy_set_drvdata(instance_ptr->generic_phy, instance_ptr);

put_child:
	of_node_put(child);
	return ret;
}

static int omega_phy_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct omega_phy_driver *phy_driver;
	struct phy_provider *phy_provider;
	int ret;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	int num_phys;

	/* allocate memory for each phy instance */
	phy_driver = devm_kzalloc(dev, sizeof(struct omega_phy_driver),
				GFP_KERNEL);
	if (!phy_driver)
		return -ENOMEM;

	num_phys = of_get_child_count(node);

	if (num_phys != 1) {
		dev_err(dev, "PHY should have exactly one child node\n");
		return -ENODEV;
	}

	phy_driver->instance = devm_kzalloc(dev,
				sizeof(struct omega_phy_instance), GFP_KERNEL);
	phy_driver->pdev = pdev;
	platform_set_drvdata(pdev, phy_driver);

	ret = omega_phy_instance_create(phy_driver);
	if (ret)
		return ret;

	phy_driver->crmu_regmap =
		syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
					"brcm,usbphy-crmu-syscon");
	if (IS_ERR(phy_driver->crmu_regmap)) {
		dev_err(&pdev->dev, "crmu regmap failed\n");
		return PTR_ERR(phy_driver->crmu_regmap);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cdru-usbphy");
	phy_driver->cdru_usbphy_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy_driver->cdru_usbphy_regs))
		return PTR_ERR(phy_driver->cdru_usbphy_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "usb2h-idm");
	phy_driver->usb2h_idm_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy_driver->usb2h_idm_regs))
		return PTR_ERR(phy_driver->usb2h_idm_regs);


	/* Shutdown a port. It can be powered up as required */
	regmap_update_bits(phy_driver->crmu_regmap,
			CRMU_USB_PHY_AON_CTRL_OFFSET,
			CRMU_USBPHY_AFE_CORERDY_VDDC | CRMU_USBPHY_RESETB,
			0);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		dev_err(dev, "Failed to register as phy provider\n");
		ret = PTR_ERR(phy_provider);
		return ret;
	}

	return 0;
}


static const struct of_device_id omega_phy_dt_ids[] = {
	{ .compatible = "brcm,omega-usb-phy", },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm_phy_dt_ids);

static struct platform_driver omega_phy_driver = {
	.probe = omega_phy_probe,
	.driver = {
		.name = "bcm-omega-usbphy",
		.of_match_table = of_match_ptr(omega_phy_dt_ids),
	},
};
module_platform_driver(omega_phy_driver);

MODULE_ALIAS("platform:bcm-omega-usbphy");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Omega USB PHY driver");
MODULE_LICENSE("GPL v2");
