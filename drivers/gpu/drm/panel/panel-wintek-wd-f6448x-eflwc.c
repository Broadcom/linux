/*
 * Wintek WD-F6448X-EFLWC LCD drm_panel driver.
 *
 * Copyright (c) 2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <drm/drmP.h>
#include <drm/drm_panel.h>

#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

static struct wdf6448x_spi_seq {
	u16 cmd;
	u8 delay;
} init_seq[] = {
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
	{ 0xb620, 0 },
};


struct wdf6448x_data {
	u8 bits_per_word;
	u32 seq_length;
	const struct wdf6448x_spi_seq *seq;
};

struct wdf6448x {
	struct device *dev;
	struct drm_panel panel;

	struct regulator *supply;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;
	struct backlight_device *backlight;
	struct spi_device *spi;
};

static const struct wdf6448x_data
	wintek_wd_f6448x_eflwc_data = {
		.bits_per_word = 16,
		.seq_length = ARRAY_SIZE(init_seq),
		.seq = init_seq,
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

static int send_init_sequence(struct spi_device *spi)
{
	const struct wdf6448x_spi_seq *seq = init_seq;
	u32 seq_length = ARRAY_SIZE(init_seq);
	int ret;

	spi->bits_per_word = 16;

	ret = send_spi_sequence(spi, seq, seq_length);

	return ret;
}

static inline struct wdf6448x *panel_to_f6448x(struct drm_panel *panel)
{
	return container_of(panel, struct wdf6448x, panel);
}

static int wdf6448x_spi_write_word(struct wdf6448x *wd, u16 data)
{
	struct spi_device *spi = to_spi_device(wd->dev);
	struct spi_transfer xfer = {
		.len		= 2,
		.tx_buf		= &data,
	};
	struct spi_message msg;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	return spi_sync(spi, &msg);
}

static int wdf6448x_disable(struct drm_panel *panel)
{
	struct wdf6448x *wd = panel_to_f6448x(panel);

	if (wd->backlight) {
		wd->backlight->props.power = FB_BLANK_POWERDOWN;
		wd->backlight->props.state |= BL_CORE_FBBLANK;
		backlight_update_status(wd->backlight);
	}

	return 0;
}

static int wdf6448x_unprepare(struct drm_panel *panel)
{
	struct wdf6448x *wd = panel_to_f6448x(panel);
	int ret;

	ret = wdf6448x_spi_write_word(wd, 0x0000);

	gpiod_set_value_cansleep(wd->enable_gpio, 0);

	msleep(100);

	ret = regulator_disable(wd->supply);
	if (ret < 0)
		return ret;

	return 0;
}

static int wdf6448x_prepare(struct drm_panel *panel)
{
	struct wdf6448x *wd = panel_to_f6448x(panel);
	struct spi_device *spi = to_spi_device(wd->dev);
	int ret;

	ret = regulator_enable(wd->supply);
	if (ret < 0)
		return ret;
	gpiod_set_value_cansleep(wd->enable_gpio, 1);

	msleep(100);
	gpiod_set_value_cansleep(wd->reset_gpio, 1);

	/* Docs say this step is "Waiting for releasing reset" without
	 * saying how, exactly.
	 */
	msleep(100);

	ret = send_init_sequence(spi);

	return ret;
}

static int wdf6448x_enable(struct drm_panel *panel)
{
	struct wdf6448x *wd = panel_to_f6448x(panel);

	if (wd->backlight) {
		wd->backlight->props.state &= ~BL_CORE_FBBLANK;
		wd->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(wd->backlight);
	}

	return 0;
}

static int wdf6448x_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct drm_display_mode *mode;
	static const struct drm_display_mode static_mode = {
		.clock = 27000,
		.hdisplay = 640,
		.hsync_start = 640 + 60,
		.hsync_end = 640 + 60 + 70,
		.htotal = 640 + 60 + 70 + 140,
		.vdisplay = 480,
		.vsync_start = 480 + 5,
		.vsync_end = 480 + 5 + 3,
		.vtotal = 480 + 5 + 3 + 33,
		.vrefresh = 60,
		.flags = DRM_MODE_FLAG_PVSYNC | DRM_MODE_FLAG_PHSYNC,
		.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,

		/* XXX: .width_mm */
	};
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	mode = drm_mode_duplicate(connector->dev, &static_mode);
	if (!mode) {
		DRM_ERROR("failed to create a new display mode\n");
		return 0;
	}
	drm_mode_set_name(mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bpc = 6;
	connector->display_info.bus_flags = 0;
	drm_display_info_set_bus_formats(&connector->display_info,
					 &bus_format, 1);

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs wdf6448x_drm_funcs = {
	.disable = wdf6448x_disable,
	.unprepare = wdf6448x_unprepare,
	.prepare = wdf6448x_prepare,
	.enable = wdf6448x_enable,
	.get_modes = wdf6448x_get_modes,
};

static int wdf6448x_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct wdf6448x *wd;
	struct device_node *backlight;
	int ret;

	wd = devm_kzalloc(dev, sizeof(struct wdf6448x), GFP_KERNEL);
	if (!wd)
		return -ENOMEM;

	spi_set_drvdata(spi, wd);

	wd->dev = dev;

	wd->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(wd->supply))
		return PTR_ERR(wd->supply);

	wd->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(wd->reset_gpio)) {
		dev_err(dev, "cannot get reset-gpios %ld\n",
			PTR_ERR(wd->reset_gpio));
		return PTR_ERR(wd->reset_gpio);
	}

	wd->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(wd->enable_gpio)) {
		dev_err(dev, "cannot get enable-gpios %ld\n",
			PTR_ERR(wd->enable_gpio));
		return PTR_ERR(wd->enable_gpio);
	}

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		wd->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!wd->backlight)
			return -EPROBE_DEFER;
	}

	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(dev, "spi setup failed.\n");
		goto free_bl;
	}

	drm_panel_init(&wd->panel);
	wd->panel.dev = dev;
	wd->panel.funcs = &wdf6448x_drm_funcs;

	ret = drm_panel_add(&wd->panel);
	if (ret)
		goto free_bl;

	return 0;

free_bl:
	if (wd->backlight)
		put_device(&wd->backlight->dev);
	return ret;
}

static int wdf6448x_remove(struct spi_device *spi)
{
	struct wdf6448x *wd = spi_get_drvdata(spi);

	drm_panel_remove(&wd->panel);

	return 0;
}

static const struct of_device_id wdf6448x_of_match[] = {
	{ .compatible = "wintek,wd-f6448x-eflwc" },
	{ }
};
MODULE_DEVICE_TABLE(of, wdf6448x_of_match);

static struct spi_driver wdf6448x_driver = {
	.probe = wdf6448x_probe,
	.remove = wdf6448x_remove,
	.driver = {
		.name = "panel-wintek-wd-f6448x-eflwc",
		.of_match_table = wdf6448x_of_match,
	},
};
module_spi_driver(wdf6448x_driver);

MODULE_AUTHOR("Eric Anholt <eric@anholt.net>");
MODULE_DESCRIPTION("Wintek WD-F6448X-EFLWc LCD Driver");
MODULE_LICENSE("GPL v2");
