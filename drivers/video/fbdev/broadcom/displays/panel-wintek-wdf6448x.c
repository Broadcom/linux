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
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

struct wintek_panel_data {
	struct gpio_desc *lcd_pwr_en;
	struct gpio_desc *lcd_reset;
	struct spi_device *wintek_spi_dev;
	uint16_t *init_seq;
	unsigned int num_words;
};

static int wintek_panel_send_init_sequence(struct wintek_panel_data *panel_data)
{
	int ret;
	struct spi_message spi_msg;
	struct spi_transfer *spi_tran = &(struct spi_transfer){
		.bits_per_word = 16
	};

	spi_message_init(&spi_msg);

	spi_setup(panel_data->wintek_spi_dev);

	/*create spi_message and send */
	spi_tran->tx_buf = panel_data->init_seq;
	spi_tran->len = panel_data->num_words * sizeof(u16);

	spi_message_add_tail(spi_tran, &spi_msg);
	ret = spi_sync(panel_data->wintek_spi_dev, &spi_msg);

	return ret;
}

static int wintek_panel_enable(struct wintek_panel_data *panel_data,
			       bool enable)
{
	int ret;

	if (enable) {
		gpiod_set_value(panel_data->lcd_pwr_en, 1);
		msleep(100);
		gpiod_set_value(panel_data->lcd_reset, 1);
		ret = wintek_panel_send_init_sequence(panel_data);
	} else {
		gpiod_set_value(panel_data->lcd_pwr_en, 0);
		ret = 0;
	}

	return ret;
}

static int wintek_panel_prepare_initseq(struct spi_device *wintek_spi_dev)
{
	int length, num_words, ret;
	uint16_t *tx;
	struct property *prop;
	struct device_node *np = wintek_spi_dev->dev.of_node;

	struct wintek_panel_data *panel_data = spi_get_drvdata(wintek_spi_dev);

	/* Read Init sequence from device tree */
	prop = of_find_property(np, "lcd-init-sequence", &length);
	if (!prop) {
		ret = -EINVAL;
		goto err_exit;
	}

	num_words = length / sizeof(u16);
	/* ensure num_words is at least 1 */
	if (!num_words) {
		ret = -EINVAL;
		goto err_exit;
	}
	tx = kcalloc(num_words, sizeof(u16), GFP_KERNEL);
	ret = of_property_read_u16_array(np, "lcd-init-sequence",
					 tx, num_words);
	if (ret) {
		dev_err(&wintek_spi_dev->dev,
			"DT entry missing lcd-init-sequence\n");

		goto err_free_exit;
	}

	wintek_spi_dev->bits_per_word = 16;
	panel_data->init_seq = tx;
	panel_data->num_words = num_words;

	return 0;

err_free_exit:
	kfree(tx);
err_exit:
	return ret;
}

static int wintek_panel_gpio_init(struct spi_device *wintek_spi_dev)
{
	int ret = 0;
	struct wintek_panel_data *panel_data = spi_get_drvdata(wintek_spi_dev);
	struct device *dev = &wintek_spi_dev->dev;

	panel_data->lcd_pwr_en = devm_gpiod_get(dev, "lcd-pwr-en",
						GPIOD_OUT_LOW);
	if (IS_ERR(panel_data->lcd_pwr_en)) {
		ret = PTR_ERR(panel_data->lcd_pwr_en);
		dev_err(&wintek_spi_dev->dev,
			"Failed to get gpio_lcd_pwr_en: %d\n", ret);
		goto err_exit;
	}

	panel_data->lcd_reset = devm_gpiod_get(dev, "lcd-reset",
						GPIOD_OUT_LOW);
	if (IS_ERR(panel_data->lcd_reset)) {
		ret = PTR_ERR(panel_data->lcd_reset);
		dev_err(&wintek_spi_dev->dev,
			"Failed to get gpio_lcd_reset: %d\n", ret);
		goto err_exit;
	}

	ret = 0;

err_exit:
	return ret;
}

static int wintek_probe(struct spi_device *wintek_spi_dev)
{
	int ret = 0;
	struct wintek_panel_data *panel_data;

	panel_data = devm_kzalloc(&wintek_spi_dev->dev,
				sizeof(*panel_data), GFP_KERNEL);
	if (!panel_data)
		return -ENOMEM;

	spi_set_drvdata(wintek_spi_dev, panel_data);
	panel_data->wintek_spi_dev = wintek_spi_dev;

	ret = wintek_panel_gpio_init(wintek_spi_dev);
	if (ret)
		goto err_exit;

	ret = wintek_panel_prepare_initseq(wintek_spi_dev);
	if (ret)
		goto err_exit;

	ret = wintek_panel_enable(panel_data, true);
	if (ret)
		goto err_free_exit;

	dev_info(&wintek_spi_dev->dev,
		 "Driver Init complete");

	return 0;

err_free_exit:
	kfree(panel_data->init_seq);
err_exit:
	return ret;
}

static int wintek_remove(struct spi_device *wintek_spi_dev)
{
	struct wintek_panel_data *panel_data = spi_get_drvdata(wintek_spi_dev);

	wintek_panel_enable(panel_data, false);
	kfree(panel_data->init_seq);

	return 0;
}

static const struct of_device_id wintek_lcd_panel_dt[] = {
	{ .compatible = "brcm,wintek,wdf6448x",},
	{ },
};

MODULE_DEVICE_TABLE(of, wintek_lcd_panel_dt);

#ifdef CONFIG_PM_SLEEP

static int wintek_lcd_suspend(struct device *dev)
{
	struct wintek_panel_data *panel_data = dev_get_drvdata(dev);

	return wintek_panel_enable(panel_data, false);
}

static int wintek_lcd_resume(struct device *dev)
{
	struct wintek_panel_data *panel_data = dev_get_drvdata(dev);

	return wintek_panel_enable(panel_data, true);
}

static const struct dev_pm_ops wintek_lcd_pm_ops = {
	.suspend = &wintek_lcd_suspend,
	.resume = &wintek_lcd_resume
};

#define WINTEK_LCD_PM_OPS (&wintek_lcd_pm_ops)
#else
#define WINTEK_LCD_PM_OPS NULL
#endif

static struct spi_driver wintek_lcd_panel = {
	.driver = {
		.name = "wintek-lcd-panel",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(wintek_lcd_panel_dt),
		.pm = WINTEK_LCD_PM_OPS,
	},
	 .probe = wintek_probe,
	 .remove = wintek_remove,
};

module_spi_driver(wintek_lcd_panel);

MODULE_AUTHOR("Arun Ramamurthy <arunrama@broadcom.com>");
MODULE_DESCRIPTION("Wintek LCD panel driver");
MODULE_LICENSE("GPL v2");
