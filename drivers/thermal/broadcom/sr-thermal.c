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

#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#define TMON_CRIT_TEMP 110000 /* temp in millidegree C */

struct sr_thermal {
	struct thermal_zone_device *tz;
	struct device *dev;
	void __iomem *regs;
	unsigned int crit_temp;
};

static int sr_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct sr_thermal *sr_thermal = tz->devdata;

	*temp = readl(sr_thermal->regs);

	return 0;
}

static int sr_get_trip_type(struct thermal_zone_device *tz, int trip,
					enum thermal_trip_type *type)
{
	struct sr_thermal *sr_thermal = tz->devdata;

	switch (trip) {
	case 0:
		*type = THERMAL_TRIP_CRITICAL;
		break;
	default:
		dev_err(sr_thermal->dev,
			"Driver does not support more than 1 trip point\n");
		return -EINVAL;
	}
	return 0;
}

static int sr_get_trip_temp(struct thermal_zone_device *tz, int trip, int *temp)
{
	struct sr_thermal *sr_thermal = tz->devdata;

	switch (trip) {
	case 0:
		*temp = sr_thermal->crit_temp;
		break;
	default:
		dev_err(sr_thermal->dev,
			"Driver does not support more than 1 trip point\n");
		return -EINVAL;
	}
	return 0;
}

static int sr_set_trip_temp(struct thermal_zone_device *tz, int trip, int temp)
{
	struct sr_thermal *sr_thermal = tz->devdata;

	switch (trip) {
	case 0:
		if (temp <= TMON_CRIT_TEMP)
			sr_thermal->crit_temp = temp;
		else
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct thermal_zone_device_ops sr_thermal_ops = {
	.get_temp = sr_get_temp,
	.get_trip_type = sr_get_trip_type,
	.get_trip_temp = sr_get_trip_temp,
	.set_trip_temp = sr_set_trip_temp,
};

static int sr_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sr_thermal *sr_thermal;
	struct resource *res;

	sr_thermal = devm_kzalloc(dev, sizeof(*sr_thermal), GFP_KERNEL);
	if (!sr_thermal)
		return -ENOMEM;
	sr_thermal->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sr_thermal->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sr_thermal->regs)) {
		dev_err(dev, "failed to get io address\n");
		return PTR_ERR(sr_thermal->regs);
	}

	/* initialize tmon value to 0 */
	writel(0, sr_thermal->regs);
	sr_thermal->crit_temp = TMON_CRIT_TEMP;

	sr_thermal->tz = thermal_zone_device_register(dev_name(dev), 1, 1,
							 sr_thermal,
							 &sr_thermal_ops,
							 NULL, 1000, 1000);
	if (IS_ERR(sr_thermal->tz))
		return PTR_ERR(sr_thermal->tz);

	platform_set_drvdata(pdev, sr_thermal);

	return 0;
}

static int sr_thermal_remove(struct platform_device *pdev)
{
	struct sr_thermal *sr_thermal = platform_get_drvdata(pdev);

	thermal_zone_device_unregister(sr_thermal->tz);

	return 0;
}

static const struct of_device_id sr_thermal_of_match[] = {
	{ .compatible = "brcm,sr-thermal", },
	{},
};
MODULE_DEVICE_TABLE(of, sr_thermal_of_match);

static struct platform_driver sr_thermal_driver = {
	.probe		= sr_thermal_probe,
	.remove		= sr_thermal_remove,
	.driver = {
		.name = "sr-thermal",
		.of_match_table = sr_thermal_of_match,
	},
};
module_platform_driver(sr_thermal_driver);

MODULE_AUTHOR("Pramod Kumar <pramod.kumar@broadcom.com>");
MODULE_DESCRIPTION("Stingray thermal driver");
MODULE_LICENSE("GPL v2");
