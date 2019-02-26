// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Broadcom
 */

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/thermal.h>

/* updating temperature reading in the background once every 2 seconds */
#define BMC_DELAY (HZ * 2)

/*
 * Tencent out of band management specification:
 * Command code 5:  read ASIC temperature.
 * Command code 24: read ASIC temperature limit.
 */
enum i2c_slave_bmc_cmd {
	CMD_ASIC_TEMP = 5,
	CMD_ASIC_TEMP_LIMIT = 24,
};

struct bmc_data {
	struct thermal_zone_device *thermal;
	int16_t asic_temp;
	int16_t asic_temp_limit;
	bool first_write;
	uint8_t command;
	spinlock_t buffer_lock;
	uint8_t buffer_idx;
	uint8_t buffer_len;
	uint8_t buffer[I2C_SMBUS_BLOCK_MAX];
	struct delayed_work work_handler;
	struct device *dev;
};

static int i2c_slave_bmc_setup(struct i2c_client *client)
{
	struct bmc_data *bmc = i2c_get_clientdata(client);
	int16_t temp16;

	memset(bmc->buffer, 0, I2C_SMBUS_BLOCK_MAX);
	bmc->buffer_idx = 0;
	switch (bmc->command) {
	case CMD_ASIC_TEMP:
		temp16 = bmc->asic_temp;
		dev_dbg(&client->dev, "Temperature = %d\n", temp16);
		memcpy(&bmc->buffer[0], &temp16, 2);
		bmc->buffer_len = 2;
		break;
	case CMD_ASIC_TEMP_LIMIT:
		temp16 = bmc->asic_temp_limit;
		memcpy(&bmc->buffer[2], &bmc->asic_temp_limit, 2);
		dev_dbg(&client->dev, "Temperature limit = %d\n", temp16);

		/* Expected temperature is temperature limit divided by 2 */
		temp16 = bmc->asic_temp_limit >> 1;
		memcpy(&bmc->buffer[0], &temp16, 2);
		bmc->buffer_len = 4;
		break;
	default:
		dev_err(&client->dev, "Unsupported command %d\n",
			bmc->command);
		return -EOPNOTSUPP;
	}
	return 0;
}

static int i2c_slave_bmc_cb(struct i2c_client *client,
			    enum i2c_slave_event event, u8 *val)
{
	struct bmc_data *bmc = i2c_get_clientdata(client);

	switch (event) {
	case I2C_SLAVE_WRITE_RECEIVED:
		if (bmc->first_write) {
			bmc->command = *val;
			bmc->first_write = false;
			spin_lock(&bmc->buffer_lock);
			i2c_slave_bmc_setup(client);
			spin_unlock(&bmc->buffer_lock);
		} else {
			dev_err(&client->dev, "Only support 1 byte command\n");
			return -EINVAL;
		}
		break;
	case I2C_SLAVE_READ_PROCESSED:
		spin_lock(&bmc->buffer_lock);
		if (bmc->buffer_idx < bmc->buffer_len) {
			/* The previous byte made it to the bus, get next one */
			bmc->buffer_idx++;
			*val = bmc->buffer[bmc->buffer_idx];
		}
		spin_unlock(&bmc->buffer_lock);
		break;
	case I2C_SLAVE_READ_REQUESTED:
		spin_lock(&bmc->buffer_lock);
		*val = bmc->buffer[bmc->buffer_idx];
		spin_unlock(&bmc->buffer_lock);
		break;
	case I2C_SLAVE_STOP:
	case I2C_SLAVE_WRITE_REQUESTED:
		bmc->first_write = true;
		break;
	default:
		dev_err(&client->dev, "Unknown event %d\n", event);
		return -EINVAL;
	}

	return 0;
}

static void i2c_slave_bmc_handler(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bmc_data *bmc =
			container_of(dwork, struct bmc_data, work_handler);
	unsigned long flags = 0;
	int temperature;
	int ret;

	ret = thermal_zone_get_temp(bmc->thermal, &temperature);
	if (!ret) {
		spin_lock_irqsave(&bmc->buffer_lock, flags);
		/* Temperature in Celsius after dividing by 1000 */
		bmc->asic_temp = temperature / 1000;
		spin_unlock_irqrestore(&bmc->buffer_lock, flags);
	} else {
		dev_err(bmc->dev, "Failed to read temperature, %d\n", ret);
	}
	schedule_delayed_work(&bmc->work_handler, BMC_DELAY);
}

static int i2c_slave_bmc_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct bmc_data *bmc;
	int ret;
	uint32_t temperature;
	struct device *dev = &client->dev;
	struct device_node *dn = client->dev.of_node;
	const char *thermal_zone;

	dev_dbg(dev, "I2C address 0x%x, flags 0x%x\n",
		client->addr, client->flags);
	bmc = devm_kzalloc(dev, sizeof(*bmc), GFP_KERNEL);
	if (!bmc)
		return -ENOMEM;

	bmc->first_write = true;
	bmc->dev = dev;
	spin_lock_init(&bmc->buffer_lock);
	ret = of_property_read_string(dn, "bmc-thermal-zone", &thermal_zone);
	if (ret)
		return ret;

	dev_dbg(dev, "Thermal zone name: %s\n", thermal_zone);
	bmc->thermal = thermal_zone_get_zone_by_name(thermal_zone);
	if (IS_ERR(bmc->thermal)) {
		dev_err(dev, "Failed to get thermal zone: %s\n", thermal_zone);
		return PTR_ERR(bmc->thermal);
	}
	ret = thermal_zone_get_crit_temp(bmc->thermal, &temperature);
	if (ret)
		return ret;

	/* Temperature in Celsius after dividing by 1000 */
	bmc->asic_temp_limit = temperature / 1000;
	dev_dbg(dev, "Temperature limit: %d\n", bmc->asic_temp_limit);

	i2c_set_clientdata(client, bmc);

	ret = i2c_slave_register(client, i2c_slave_bmc_cb);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&bmc->work_handler, i2c_slave_bmc_handler);
	schedule_delayed_work(&bmc->work_handler, 0);
	return 0;
}

static int i2c_slave_bmc_remove(struct i2c_client *client)
{
	struct bmc_data *bmc = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&bmc->work_handler);
	i2c_slave_unregister(client);
	return 0;
}

static const struct i2c_device_id i2c_slave_bmc_id[] = {
	{"i2c-slave-bmc", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, i2c_slave_bmc_id);

static struct i2c_driver i2c_slave_bmc_driver = {
	.driver = {
		.name = "i2c-slave-bmc",
	},
	.probe = i2c_slave_bmc_probe,
	.remove = i2c_slave_bmc_remove,
	.id_table = i2c_slave_bmc_id,
};
module_i2c_driver(i2c_slave_bmc_driver);

MODULE_AUTHOR("Geoffrey Lv <geoffrey.lv@broadcom.com>");
MODULE_DESCRIPTION("I2C slave mode BMC driver");
MODULE_LICENSE("GPL v2");
