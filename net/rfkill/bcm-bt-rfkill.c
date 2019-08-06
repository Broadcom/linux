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

/* Broadcom Bluetooth rfkill power control via GPIO.  The GPIOs are
 * configured through Device Tree.
 *
 * Note: This driver doesn't conform with the latest kernel and can't be
 * upstreamed.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/rfkill.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/fs.h>

#define MAX_REGULATOR_NAME_LEN  25

struct bcm_rfkill_instance {
	struct regulator *bt_reg;
};

struct bcm_bt_rfkill_data {
	struct device *dev;
	struct rfkill *rfkill;
	int num_regs;
	struct bcm_rfkill_instance *instances;
	int dev_wake_gpio;
	int host_wake_gpio;
};

static ssize_t show_dev_wake_value(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct bcm_bt_rfkill_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->dev_wake_gpio);
}

static ssize_t store_dev_wake_value(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int value;
	struct bcm_bt_rfkill_data *data = dev_get_drvdata(dev);

	if (kstrtoint(buf, 10, &value) != 0)
		dev_err(dev, "Unable to read value\n");
	else
		data->dev_wake_gpio = value;

	return count;
}

DEVICE_ATTR(dev_wake, S_IRUGO | S_IWUSR,
	    show_dev_wake_value,
	    store_dev_wake_value);

static ssize_t store_host_wake_value(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	int value;
	struct bcm_bt_rfkill_data *data = dev_get_drvdata(dev);

	if (kstrtoint(buf, 10, &value) != 0)
		dev_err(dev, "Unable to read value\n");
	else
		data->host_wake_gpio = value;

	return count;
}

static ssize_t show_host_wake_value(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct bcm_bt_rfkill_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->host_wake_gpio);
}

DEVICE_ATTR(host_wake, S_IRUGO | S_IWUSR,
	    show_host_wake_value,
	    store_host_wake_value);

static int bcm_bt_regulator_get(struct bcm_bt_rfkill_data *pdata)
{
	int num_regs;
	struct bcm_rfkill_instance *instance_ptr;
	struct device_node *np = pdata->dev->of_node;
	int reg_num;

	/* Count number of regulators in phandle list. */
	num_regs = of_count_phandle_with_args(np, "brcm,rfkill-regulator", 0);
	if (num_regs == 0) {
		dev_err(pdata->dev, "must specify at least 1 regulator");
		return -EINVAL;
	}

	pdata->instances = devm_kcalloc(pdata->dev, num_regs,
		sizeof(struct bcm_rfkill_instance), GFP_KERNEL);
	if (!pdata->instances)
		return -ENOMEM;

	/* Register each regulator in phandle list. */
	for (reg_num = 0; reg_num < num_regs; reg_num++) {
		struct device_node *reg_node;
		const char *regulator_name;
		int err;

		reg_node = of_parse_phandle(np,
					    "brcm,rfkill-regulator",
					    reg_num);
		if (!reg_node) {
			dev_err(pdata->dev, "error parsing brcm,rfkill-regulator");
			return -ENODEV;
		}

		/* Get regulator-name property from regulator node. */
		err = of_property_read_string(reg_node,
					      "regulator-name",
					      &regulator_name);
		if (err) {
			dev_err(pdata->dev, "error reading regulator_name property");
			return err;
		}

		instance_ptr = &pdata->instances[reg_num];
		instance_ptr->bt_reg = devm_regulator_get(pdata->dev,
			regulator_name);
		if (IS_ERR(instance_ptr->bt_reg)) {
			dev_err(pdata->dev, "error getting regulator %s",
				reg_node->name);
			return PTR_ERR(instance_ptr->bt_reg);
		}
	}

	pdata->num_regs = num_regs;

	return 0;
}

static int bcm_bt_regulator_enable(struct bcm_bt_rfkill_data *pdata)
{
	int i, err;

	for (i = 0; i < pdata->num_regs; i++) {
		if (pdata->instances[i].bt_reg) {
			err = regulator_enable(pdata->instances[i].bt_reg);
			if (err) {
				dev_err(pdata->dev,
					"failed to enable regulator\n");
				return err;
			}
		}
	}

	return 0;
}

static int bcm_bt_regulator_disable(struct bcm_bt_rfkill_data *pdata)
{
	int i, err;

	for (i = 0; i < pdata->num_regs; i++) {
		if (pdata->instances[i].bt_reg &&
		    regulator_is_enabled(pdata->instances[i].bt_reg)) {
			err = regulator_disable(pdata->instances[i].bt_reg);
			if (err) {
				dev_err(pdata->dev,
					"failed to disable regulator\n");
				return err;
			}
		}
	}

	return 0;
}

static int bcm_bt_rfkill_set_power(void *pdata, bool blocked)
{
	int ret = 0;
	struct bcm_bt_rfkill_data *data = pdata;

	if (blocked) {
		/* Transmitter OFF (Blocked) */
		ret = bcm_bt_regulator_disable(data);
		dev_info(data->dev, "bcm_bt_rfkill_setpower: blocked\n");
	} else {
		/* Transmitter ON (Unblocked) */
		ret = bcm_bt_regulator_enable(data);
		dev_info(data->dev, "bcm_bt_rfkill_setpower: unblocked\n");
	}

	return ret;
}

static const struct rfkill_ops bcm_bt_rfkill_ops = {
	.set_block = bcm_bt_rfkill_set_power,
};

static int bcm_bt_rfkill_probe(struct platform_device *pdev)
{
	int err;
	struct bcm_bt_rfkill_data *data;
	struct device *dev = &pdev->dev;
	int gpio_value;
	struct device_node *np = pdev->dev.of_node;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;

	err = bcm_bt_regulator_get(data);
	if (err)
		return err;

	data->rfkill = rfkill_alloc("bcm_bt", dev, RFKILL_TYPE_BLUETOOTH,
		&bcm_bt_rfkill_ops, data);
	if (!data->rfkill)
		return -ENOMEM;

	/* Keep BT blocked by default as per above init */
	rfkill_init_sw_state(data->rfkill, true);

	platform_set_drvdata(pdev, data);

	err = rfkill_register(data->rfkill);
	if (err)
		goto err_register;

	/* Create sysfs entries for wake GPIOs */
	err = device_create_file(dev, &dev_attr_dev_wake);
	if (err)
		goto err_dev_wake_create;

	err = device_create_file(dev, &dev_attr_host_wake);
	if (err)
		goto err_host_wake_create;

	/* Read device tree to get bluetooth device and host wake GPIOs */
	gpio_value = of_get_named_gpio(np, "bt-dev-wake-gpio", 0);
	if (gpio_value < 0)
		dev_warn(dev, "bt-dev-wake-gpio missing\n");
	else
		data->dev_wake_gpio = gpio_value;

	gpio_value = of_get_named_gpio(np, "bt-host-wake-gpio", 0);
	if (gpio_value < 0)
		dev_warn(dev, "bt-host-wake-gpio missing\n");
	else
		data->host_wake_gpio = gpio_value;

	return 0;

err_host_wake_create:
	device_remove_file(data->dev, &dev_attr_dev_wake);

err_dev_wake_create:
	rfkill_unregister(data->rfkill);

err_register:
	rfkill_destroy(data->rfkill);

	return err;
}

static int bcm_bt_rfkill_remove(struct platform_device *pdev)
{
	struct bcm_bt_rfkill_data *data = pdev->dev.platform_data;

	bcm_bt_regulator_disable(data);

	rfkill_unregister(data->rfkill);
	rfkill_destroy(data->rfkill);

	device_remove_file(data->dev, &dev_attr_dev_wake);
	device_remove_file(data->dev, &dev_attr_host_wake);

	return 0;
}

static const struct of_device_id bcm_bt_rfkill_of_match[] = {
	{.compatible = "brcm,bt-rfkill" },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm_bt_rfkill_of_match);

static struct platform_driver bcm_bt_rfkill_platform_driver = {
	.driver = {
		.name = "bcm-bt-rfkill",
		.owner = THIS_MODULE,
		.of_match_table = bcm_bt_rfkill_of_match,
	},
	.probe = bcm_bt_rfkill_probe,
	.remove = bcm_bt_rfkill_remove,
};

module_platform_driver(bcm_bt_rfkill_platform_driver);

MODULE_DESCRIPTION("bcm-bt-rfkill");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
