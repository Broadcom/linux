/*
 * Copyright 2016 Broadcom
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
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/lcd.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#define POWER_IS_ON(pwr)	((pwr) <= FB_BLANK_NORMAL)

struct wdf6448x_spi_seq {
	u16 cmd;
	u8 delay;
};

struct wdf6448x_data {
	u8 bits_per_word;
	u32 seq_length;
	const struct wdf6448x_spi_seq *seq;
};

struct wdf6448x {
	struct gpio_desc *pwr_en_gpio;
	struct gpio_desc *reset_gpio;
	struct spi_device *spi;
	struct wdf6448x_data *data;
	struct lcd_device *lcd;
	int lcd_power;
};


static const struct wdf6448x_spi_seq wdf6448x_eflwc_seq[] = {
	/* CMD, DELAY (msec) */
	{ 0x0203, 0 },
	{ 0x06c2, 0 },
	{ 0x0a11, 0 },
	{ 0x0e8c, 0 },
	{ 0x128b, 0 },
	{ 0x1622, 0 },
	{ 0x1a00, 0 },
	{ 0x1e30, 0 },
	{ 0x2208, 0 },
	{ 0x2640, 0 },
	{ 0x2a88, 0 },
	{ 0x2e88, 0 },
	{ 0x3220, 0 },
	{ 0x3620, 0 },
	{ 0x8203, 0 },
	{ 0x86c2, 0 },
	{ 0x8a11, 0 },
	{ 0x8e8c, 0 },
	{ 0x928b, 0 },
	{ 0x9622, 0 },
	{ 0x9a00, 0 },
	{ 0x9e30, 0 },
	{ 0xa208, 0 },
	{ 0xa640, 0 },
	{ 0xaa88, 0 },
	{ 0xae88, 0 },
	{ 0xb220, 0 },
	{ 0xb620, 0 }
};

static const struct wdf6448x_data
	wintek_wd_f6448x_eflwc_data = {
		.bits_per_word = 16,
		.seq_length = ARRAY_SIZE(wdf6448x_eflwc_seq),
		.seq = wdf6448x_eflwc_seq,
};

static int send_spi_sequence(struct spi_device *spi,
	const struct wdf6448x_spi_seq seq[], u32 seq_length)
{
	struct spi_message spi_msg;
	struct spi_transfer spi_tran = {};
	u16 cmd;
	int ret;
	int i;

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to setup spi with error %d\n", ret);
		return ret;
	}

	for (i = 0; i < seq_length ; i++) {
		spi_message_init(&spi_msg);

		cmd = seq[i].cmd;
		spi_tran.tx_buf = &cmd;
		spi_tran.len = sizeof(cmd);

		spi_message_add_tail(&spi_tran, &spi_msg);

		ret = spi_sync(spi, &spi_msg);
		if (ret < 0) {
			dev_err(&spi->dev,
				"Failed to spi_sync with error %d\n", ret);
			break;
		}

		msleep(seq[i].delay);
	}
	return ret;
}

static int send_init_sequence(struct wdf6448x *panel)
{
	const struct wdf6448x_spi_seq *seq = panel->data->seq;
	u32 seq_length = panel->data->seq_length;
	int ret;

	panel->spi->bits_per_word = panel->data->bits_per_word;

	ret = send_spi_sequence(panel->spi, seq, seq_length);

	return ret;
}

static int wdf6448x_enable(struct wdf6448x *panel, bool enable)
{
	int ret = 0;

	if (enable) {
		gpiod_set_value_cansleep(panel->pwr_en_gpio, 1);

		msleep(100);

		gpiod_set_value_cansleep(panel->reset_gpio, 1);

		ret = send_init_sequence(panel);

	} else
		gpiod_set_value_cansleep(panel->pwr_en_gpio, 0);

	return ret;
}

static int wdf6448x_gpio_init(struct spi_device *spi)
{
	struct wdf6448x *panel = spi_get_drvdata(spi);
	int ret = 0;

	panel->pwr_en_gpio = devm_gpiod_get_optional(&spi->dev, "power-enable",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(panel->pwr_en_gpio)) {
		ret = PTR_ERR(panel->pwr_en_gpio);
		dev_err(&spi->dev, "Failed to get power-enable-gpio from DT\n");
		return ret;
	}

	panel->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(panel->reset_gpio)) {
		ret = PTR_ERR(panel->reset_gpio);
		dev_err(&spi->dev, "Failed to get reset-gpio from DT\n");
		return ret;
	}
	return 0;
}


static int wdf6448x_get_power(struct lcd_device *lcd)
{
	struct wdf6448x *panel = lcd_get_data(lcd);

	return panel->lcd_power;
}

static int wdf6448x_set_power(struct lcd_device *lcd, int power)
{
	struct wdf6448x *panel = lcd_get_data(lcd);

	if (POWER_IS_ON(power) && !POWER_IS_ON(panel->lcd_power))
		wdf6448x_enable(panel, true);

	if (!POWER_IS_ON(power) && POWER_IS_ON(panel->lcd_power))
		wdf6448x_enable(panel, false);

	panel->lcd_power = power;

	return 0;
}

static const struct of_device_id wdf6448x_of_match[] = {
	{
		.compatible = "wintek,wd-f6448x-eflwc",
		.data = &wintek_wd_f6448x_eflwc_data,
	}, {
		/* sentinel */
	},
};

MODULE_DEVICE_TABLE(of, wdf6448x_of_match);

static struct lcd_ops wdf6448x_lcd_ops = {
	.set_power = wdf6448x_set_power,
	.get_power = wdf6448x_get_power,
};

static int wdf6448x_probe(struct spi_device *spi)
{
	int ret = 0;
	struct wdf6448x *panel;
	const struct of_device_id *match;

	match = of_match_device(wdf6448x_of_match, &spi->dev);
	if (!match)
		return -ENODEV;

	panel = devm_kzalloc(&spi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	spi_set_drvdata(spi, panel);
	panel->spi = spi;
	panel->data = (struct wdf6448x_data *)match->data;

	ret = wdf6448x_gpio_init(spi);
	if (ret)
		goto err_exit;

	ret = wdf6448x_enable(panel, true);
	if (ret)
		goto err_exit;


	panel->lcd = devm_lcd_device_register(&spi->dev, "wintek-f6448x",
					      &spi->dev, panel,
					      &wdf6448x_lcd_ops);

	if (IS_ERR(panel->lcd)) {
		ret = PTR_ERR(panel->lcd);
		goto err_exit;
	}
	return 0;

err_exit:
	return ret;
}

static int wdf6448x_remove(struct spi_device *spi)
{
	struct wdf6448x *panel = spi_get_drvdata(spi);

	wdf6448x_enable(panel, false);

	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int wdf6448x_suspend(struct device *dev)
{
	struct wdf6448x *panel = dev_get_drvdata(dev);

	return wdf6448x_enable(panel, false);
}

static int wdf6448x_resume(struct device *dev)
{
	struct wdf6448x *panel = dev_get_drvdata(dev);

	return wdf6448x_enable(panel, true);
}

#endif

static SIMPLE_DEV_PM_OPS(wdf6448x_pm_ops, wdf6448x_suspend, wdf6448x_resume);

static struct spi_driver wdf6448x_driver = {
	.driver = {
		   .name = "wintek-f6448x",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(wdf6448x_of_match),
		   .pm = &wdf6448x_pm_ops,
		   },
	.probe = wdf6448x_probe,
	.remove = wdf6448x_remove,
};

module_spi_driver(wdf6448x_driver);

MODULE_AUTHOR("Suji Velupillai <suji.velupillai@broadcom.com>");
MODULE_DESCRIPTION("Wintek LCD panel driver");
MODULE_LICENSE("GPL v2");
