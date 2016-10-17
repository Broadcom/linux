/*
 * Copyright (C) 2016 Broadcom.
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/stddef.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ds1wm.h>

#define D1W_DIN_OFFSET  0x00
#define D1W_DOUT_OFFSET 0x04
#define D1W_ADDR_OFFSET 0x08
#define D1W_CTRL_OFFSET 0x0C

#define WRITE_CMD 0x3
#define READ_CMD 0x2

#define DEFAULT_RESET_RECOVERY_DELAY	1
/* CLOCK_RATE programmed in the IP divisor register DS1WM_CLKDIV*/
#define DEFAULT_CLOCK_RATE		100000000

struct iproc_ds1wm_core {
	struct ds1wm_driver_data plat_data;
	struct clk *ds1wm_clk;
};

static void iproc_d1w_write_register(void __iomem *base, u32 reg,
		u8 val)
{
	writel(val, base + D1W_DIN_OFFSET);
	writel(reg, base + D1W_ADDR_OFFSET);
	writel(WRITE_CMD, base + D1W_CTRL_OFFSET);
}

static u8 iproc_d1w_read_register(void __iomem *base, u32 reg)
{
	writel(reg, base + D1W_ADDR_OFFSET);
	writel(READ_CMD, base + D1W_CTRL_OFFSET);
	return readb(base + D1W_DOUT_OFFSET);
}

static int iproc_ds1wm_enable(struct platform_device *pdev)
{
	struct iproc_ds1wm_core *ds1wm_core;

	ds1wm_core = dev_get_drvdata(pdev->dev.parent);
	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (ds1wm_core->ds1wm_clk)
		return clk_prepare_enable(ds1wm_core->ds1wm_clk);

	return 0;
}

static int iproc_ds1wm_disable(struct platform_device *pdev)
{
	struct iproc_ds1wm_core *ds1wm_core;

	ds1wm_core = dev_get_drvdata(pdev->dev.parent);
	dev_dbg(&pdev->dev, "%s\n", __func__);
	if (ds1wm_core->ds1wm_clk)
		clk_disable_unprepare(ds1wm_core->ds1wm_clk);

	return 0;
}

static struct resource iproc_ds1wm_resources[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_IRQ,
	}
};

static struct mfd_cell iproc_ds1wm_mfd_cell = {
	.name          = "ds1wm",
	.enable        = iproc_ds1wm_enable,
	.disable       = iproc_ds1wm_disable,
	.num_resources = ARRAY_SIZE(iproc_ds1wm_resources),
	.resources     = iproc_ds1wm_resources,
};

static int iproc_d1w_dt_binding(struct platform_device *pdev,
		struct iproc_ds1wm_core *ds1wm_core)
{
	int ret;

	ret = of_property_read_u32(pdev->dev.of_node,
			"reset-recover-delay",
			&ds1wm_core->plat_data.reset_recover_delay);
	if (ret < 0)
		ds1wm_core->plat_data.reset_recover_delay =
				DEFAULT_RESET_RECOVERY_DELAY;

	ret = of_property_read_u32(pdev->dev.of_node,
			"clock-frequency", &ds1wm_core->plat_data.clock_rate);
	if (ret < 0)
		ds1wm_core->plat_data.clock_rate = DEFAULT_CLOCK_RATE;

	ds1wm_core->ds1wm_clk = devm_clk_get(&pdev->dev, "iproc_d1w_clk");
	if (IS_ERR(ds1wm_core->ds1wm_clk)) {
		dev_info(&pdev->dev,
				"No clock specified. Assuming it's enabled\n");
		ds1wm_core->ds1wm_clk = NULL;
	}
	ret = clk_set_rate(ds1wm_core->ds1wm_clk,
		ds1wm_core->plat_data.clock_rate);
	if (ret)
		dev_info(&pdev->dev,
		"Failed to set to %u\n", ds1wm_core->plat_data.clock_rate);
	return ret;
}

static int iproc_d1w_mfd_init(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	int irq;
	struct iproc_ds1wm_core *ds1wm_core;

	ds1wm_core = platform_get_drvdata(pdev);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No Memory Resource specified\n");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		return irq;
	}

	ds1wm_core->plat_data.read = iproc_d1w_read_register;
	ds1wm_core->plat_data.write = iproc_d1w_write_register;

	ret = mfd_add_devices(&pdev->dev, pdev->id,
			      &iproc_ds1wm_mfd_cell, 1, res, irq, NULL);
	if (ret < 0)
		dev_err(&pdev->dev, "failed to register iproc_d1w mfd\n");

	dev_dbg(&pdev->dev, "Added iproc_d1w mfd successfully\n");
	return ret;
}

static int iproc_d1w_probe(struct platform_device *pdev)
{
	int ret;
	struct iproc_ds1wm_core *ds1wm_core;

	ds1wm_core = devm_kzalloc(&pdev->dev,
			sizeof(struct iproc_ds1wm_core), GFP_KERNEL);
	if (!ds1wm_core)
		return -ENOMEM;

	ret = iproc_d1w_dt_binding(pdev, ds1wm_core);
	if (ret) {
		dev_err(&pdev->dev, "Probe failed\n");
		return ret;
	}

	iproc_ds1wm_mfd_cell.platform_data = &ds1wm_core->plat_data;
	iproc_ds1wm_mfd_cell.pdata_size = sizeof(struct ds1wm_driver_data);

	platform_set_drvdata(pdev, ds1wm_core);
	ret = iproc_d1w_mfd_init(pdev);

	return ret;
}

static const struct of_device_id iproc_d1w_of_match[] = {
	{ .compatible = "brcm,iproc-d1w" },
	{}
};
MODULE_DEVICE_TABLE(of, iproc_d1w_of_match);

static struct platform_driver iproc_d1w_driver = {
	.driver = {
		.name = "brcm,iproc-d1w",
		.of_match_table = of_match_ptr(iproc_d1w_of_match),
	},
	.probe = iproc_d1w_probe,
};
module_platform_driver(iproc_d1w_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom one wire busmaster Driver");
MODULE_LICENSE("GPL v2");
