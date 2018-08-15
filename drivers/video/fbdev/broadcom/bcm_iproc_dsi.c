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

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include "bcm_iproc_dsi_ctrl.h"

const struct bcm_pixel_format fb_fmt[] = {
	{
		.format = DISPDRV_FB_FORMAT_MIN,
		.bpp = 0,
	}, {
		.format = DISPDRV_FB_FORMAT_RGB565,
		.bpp = 16,
		.r = { .length = 5, .offset = 11, .msb_right = 0 },
		.g = { .length = 6, .offset = 5, .msb_right = 0 },
		.b = { .length = 5, .offset = 0, .msb_right = 0 },
	}, {
		.r = { .length = 8, .offset = 16, .msb_right = 0 },
		.g = { .length = 8, .offset = 8, .msb_right = 0 },
		.b = { .length = 8, .offset = 0, .msb_right = 0 },
		.t = { .length = 8, .offset = 24, .msb_right = 0 },
	}, {
		.r = { .length = 8, .offset = 0, .msb_right = 0 },
		.g = { .length = 8, .offset = 8, .msb_right = 0 },
		.b = { .length = 8, .offset = 16, .msb_right = 0 },
		.t = { .length = 8, .offset = 24, .msb_right = 0 },
	}
};

static inline unsigned int convert_bitfield(int val, struct fb_bitfield *bf)
{
	unsigned int mask = BIT(bf->length) - 1;

	return ((val >> (16 - bf->length)) & mask) << bf->offset;
}

static int iproc_fb_open(struct fb_info *info, int user)
{
	struct dsi_platform_data *dsi_pvt;

	dsi_pvt = container_of(info, struct dsi_platform_data, fb);
	dsi_pvt->open_count++;

	return 0;
}

static int iproc_fb_release(struct fb_info *info, int user)
{
	struct dsi_platform_data *dsi_pvt;

	dsi_pvt = container_of(info, struct dsi_platform_data, fb);

	if (dsi_pvt->open_count == 0)
		return -EIO;
	dsi_pvt->open_count--;

	return 0;
}

static int iproc_fb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	struct dsi_platform_data *dsi_pvt;

	dsi_pvt = container_of(info, struct dsi_platform_data, fb);
	var->yres_virtual = dsi_pvt->num_fb * info->var.yres;

	if ((var->xres != info->var.xres) || (var->yres != info->var.yres) ||
			(var->xres_virtual != info->var.xres))
		return -EINVAL;

	return 0;
}

static int iproc_fb_setcolreg(uint32_t regno, uint32_t red, uint32_t green,
			uint32_t blue, uint32_t transp, struct fb_info *info)
{
	struct dsi_platform_data *dsi_pvt =
			container_of(info, struct dsi_platform_data, fb);

	if (regno >= MAX_CMAP_COUNT)
		return -EINVAL;

	dsi_pvt->cmap[regno] =
			convert_bitfield(transp, &dsi_pvt->fb.var.transp) |
			convert_bitfield(blue, &dsi_pvt->fb.var.blue) |
			convert_bitfield(green, &dsi_pvt->fb.var.green) |
			convert_bitfield(red, &dsi_pvt->fb.var.red);

	return 0;
}

static int iproc_fb_pan_display(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	int ret = 0;
	unsigned int buff_idx;
	dma_addr_t fb_buff;
	struct dsi_platform_data *fb =
			container_of(info, struct dsi_platform_data, fb);

	buff_idx = (var->yoffset / var->yres);
	fb_buff = fb->buff[buff_idx];
	ret = dsic_update(fb->display_hdl, fb_buff);

	return ret;
}

static int iproc_fb_sync(struct fb_info *info)
{
	struct dsi_platform_data *dsi_pvt;

	dsi_pvt = container_of(info, struct dsi_platform_data, fb);

	if (!dsi_pvt->blank_state)
		wait_for_completion_interruptible(&dsi_pvt->vsync_event);
	return 0;
}

static int disable_display(struct dsi_platform_data *fb)
{
	dsic_powercontrol(fb->display_hdl, CTRL_PWR_OFF);
	dsic_close(fb->display_hdl);
	dsic_exit(fb->display_hdl);
	dsic_stop();
	return 0;
}

static int enable_display(struct dsi_platform_data *fb)
{
	int ret = 0;

	ret = dsic_init(panel.disp_info, &fb->display_hdl, fb->id);
	if (ret != 0)
		return ret;

	dsic_start();
	ret = dsic_open(fb->display_hdl);
	if (ret != 0)
		goto fail_to_open;

	ret = dsic_powercontrol(fb->display_hdl, CTRL_PWR_ON);
	if (ret != 0)
		goto fail_power_ctrl;

	return 0;

fail_power_ctrl:
	dsic_close(fb->display_hdl);
fail_to_open:
	dsic_stop();
	dsic_exit(fb->display_hdl);
	return ret;
}

static int iproc_fb_blank(int blank_mode, struct fb_info *info)
{
	struct dsi_platform_data *dsi_pvt;
	unsigned int buff_idx;

	dsi_pvt = container_of(info, struct dsi_platform_data, fb);

	switch (blank_mode) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		if (dsi_pvt->blank_state == true)
			break;
		dsic_powercontrol(dsi_pvt->display_hdl, CTRL_SCREEN_OFF);
		disable_display(dsi_pvt);
		dsi_pvt->blank_state = true;
		break;

	case FB_BLANK_UNBLANK:
		if (dsi_pvt->blank_state == false)
			break;

		enable_display(dsi_pvt);
		buff_idx = (dsi_pvt->fb.var.yoffset / dsi_pvt->fb.var.yres);
		dsic_update(dsi_pvt->display_hdl, dsi_pvt->buff[buff_idx]);

		dsic_powercontrol(dsi_pvt->display_hdl, CTRL_SCREEN_ON);
		dsi_pvt->blank_state = false;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static struct fb_ops iproc_fb_ops = {
	.fb_open = iproc_fb_open,
	.fb_release = iproc_fb_release,
	.fb_check_var = iproc_fb_check_var,
	.fb_set_par = NULL,
	.fb_setcolreg = iproc_fb_setcolreg,
	.fb_pan_display = iproc_fb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_ioctl = NULL,
	.fb_sync = iproc_fb_sync,
	.fb_blank = iproc_fb_blank,
};

static int allocate_fb(struct platform_device *pdev, struct dispdrv_info *info)
{
	struct dsi_platform_data *dsi_pvt = platform_get_drvdata(pdev);
	size_t framesize, framesize_alloc;
	dma_addr_t dma_addr;
	int i;

	framesize = info->width * info->height * info->bpp;
	framesize *= dsi_pvt->num_fb;

	framesize_alloc = PAGE_ALIGN(framesize + PAGE_SIZE);
	dsi_pvt->fb.screen_base = dma_alloc_writecombine(&pdev->dev,
				framesize_alloc, &dma_addr, GFP_KERNEL);
	if (dsi_pvt->fb.screen_base == NULL)
		return -ENOMEM;
	dsi_pvt->buff = devm_kzalloc(&pdev->dev,
			(dsi_pvt->num_fb) * sizeof(dma_addr_t), GFP_KERNEL);
	if (dsi_pvt->buff == NULL)
		return -ENOMEM;

	for (i = 0; i < dsi_pvt->num_fb; i++) {
		dsi_pvt->buff[i] = dma_addr + i * framesize / dsi_pvt->num_fb;
	}
	dsi_pvt->fb.fix.smem_start = dma_addr;
	dsi_pvt->fb.fix.smem_len = framesize;

	dev_info(&pdev->dev, "Framebuffer start [%pad]\n", &dma_addr);
	dev_info(&pdev->dev, "Virt[0x%p] with frame size[0x%zx]\n",
				dsi_pvt->fb.screen_base, framesize);

	return 0;
}

int setup_framebuffer(struct platform_device *pdev, struct dispdrv_info *info)
{
	struct dsi_platform_data *dsi_pvt = platform_get_drvdata(pdev);
	enum dispdrv_fb_format in_fmt;
	int ret;

	in_fmt = info->in_fmt;
	if (in_fmt >= DISPDRV_FB_FORMAT_MAX) {
		dev_err(&pdev->dev, "Invalid fb format!\n");
		return -EINVAL;
	}

	ret = allocate_fb(pdev, info);
	if (ret)
		return ret;

	dsi_pvt->fb.fbops = &iproc_fb_ops;
	dsi_pvt->fb.flags = FBINFO_FLAG_DEFAULT;
	dsi_pvt->fb.pseudo_palette = dsi_pvt->cmap;
	dsi_pvt->fb.fix.type = FB_TYPE_PACKED_PIXELS;
	dsi_pvt->fb.fix.visual = FB_VISUAL_TRUECOLOR;
	dsi_pvt->fb.fix.line_length = info->width * info->bpp;
	dsi_pvt->fb.fix.accel = FB_ACCEL_NONE;
	dsi_pvt->fb.fix.ypanstep = 1;
	dsi_pvt->fb.fix.xpanstep = 4;
	dsi_pvt->fb.var.xres = info->width;
	dsi_pvt->fb.var.yres = info->height;
	dsi_pvt->fb.var.xres_virtual = info->width;
	dsi_pvt->fb.var.yres_virtual = info->height * dsi_pvt->num_fb;
	dsi_pvt->fb.var.bits_per_pixel = info->bpp << 3;
	dsi_pvt->fb.var.activate = FB_ACTIVATE_NOW;
	dsi_pvt->fb.var.height = info->phys_height;
	dsi_pvt->fb.var.width = info->phys_width;

	dsi_pvt->fb.var.red.offset = fb_fmt[in_fmt].r.offset;
	dsi_pvt->fb.var.red.length = fb_fmt[in_fmt].r.length;
	dsi_pvt->fb.var.green.offset = fb_fmt[in_fmt].g.offset;
	dsi_pvt->fb.var.green.length = fb_fmt[in_fmt].g.length;
	dsi_pvt->fb.var.blue.offset = fb_fmt[in_fmt].b.offset;
	dsi_pvt->fb.var.blue.length = fb_fmt[in_fmt].b.length;
	dsi_pvt->fb.var.transp.offset = fb_fmt[in_fmt].t.offset;
	dsi_pvt->fb.var.transp.length = fb_fmt[in_fmt].t.length;

	fb_set_var(&dsi_pvt->fb, &dsi_pvt->fb.var);
	enable_display(dsi_pvt);
	dsi_pvt->blank_state = true;

	ret = register_framebuffer(&dsi_pvt->fb);
	if (ret) {
		dev_err(&pdev->dev, "Framebuffer registration failed\n");
		return ret;
	}

	return ret;
}

void calculate_dsi_clock(struct dispdrv_info *info, unsigned long *clk_rate)
{
	unsigned int bits_per_line, bits_per_frame;
	unsigned int clk_per_frame;

	bits_per_line = (info->width + info->hs + info->hbp + info->hfp) *
								8 * info->bpp;
	bits_per_frame = bits_per_line * (info->height + info->vs +
							info->vbp + info->vfp);

	/* divided by 2 as it's double data rate clock */
	clk_per_frame = bits_per_frame * INTERNAL_CLK_DIVIDER
				 / (panel.disp_info->lanes * 2);
	*clk_rate = clk_per_frame * info->fps;
}

static struct dsi_platform_data * __init dsi_get_of_data
					(struct platform_device *pdev)
{
	unsigned int  val;
	unsigned long clk_rate;
	struct dsi_platform_data *dsip_data;
	struct device_node *np, *pnode;
	struct resource *dsi_res, *axipv_res, *pv_res, *genpll_res, *reset_res;
	int ret;
	const char *str;

	np = pdev->dev.of_node;

	dsip_data = devm_kzalloc(&pdev->dev, sizeof(*dsip_data), GFP_KERNEL);
	if (!dsip_data)
		goto of_fail;

	of_property_read_u32(np, "brcm,num-fb", &val);
	dsip_data->num_fb = val;

	if (of_property_read_u32(np, "id", &val))
		goto of_fail;
	dsip_data->id = (uint8_t)val;

	dsi_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dsi-phy");
	dsip_data->dsi_base = devm_ioremap_resource(&pdev->dev, dsi_res);
	if (IS_ERR(dsip_data->dsi_base)) {
		dev_err(&pdev->dev, "iomap of DSI resource failed\n");
		goto of_fail;
	}

	axipv_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "axipv");
	dsip_data->axipv_base = devm_ioremap_resource(&pdev->dev, axipv_res);
	if (IS_ERR(dsip_data->axipv_base)) {
		dev_err(&pdev->dev, "iomap of AXIPV resource failed\n");
		goto of_fail;
	}

	pv_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pv");
	dsip_data->pv_base = devm_ioremap_resource(&pdev->dev, pv_res);
	if (IS_ERR(dsip_data->pv_base)) {
		dev_err(&pdev->dev, "iomap of PV resource failed\n");
		goto of_fail;
	}

	genpll_res = platform_get_resource_byname(
					pdev, IORESOURCE_MEM, "gen-pll");
	dsip_data->dsi_genpll_base =
				devm_ioremap_resource(&pdev->dev, genpll_res);
	if (IS_ERR(dsip_data->dsi_genpll_base)) {
		dev_err(&pdev->dev, "iomap of GENPLL resource failed\n");
		goto of_fail;
	}

	reset_res = platform_get_resource_byname(pdev,
					IORESOURCE_MEM, "crmu_ext_reset");
	if (reset_res) {
		dsip_data->reset_base =
			devm_ioremap_resource(&pdev->dev, reset_res);
		if (IS_ERR(dsip_data->reset_base)) {
			dev_err(&pdev->dev, "iomap of Reset base failed\n");
			goto of_fail;
		}
	}

	dsip_data->reset_gpio = devm_gpiod_get_optional(&pdev->dev, "reset",
						GPIOD_OUT_LOW);
	if (IS_ERR(dsip_data->reset_gpio)) {
		if (PTR_ERR(dsip_data->reset_gpio) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to get reset gpio: %ld\n",
				PTR_ERR(dsip_data->reset_gpio));
		return ERR_CAST(dsip_data->reset_gpio);
	}

	if (of_get_property(np, "brcm,iproc-mipi-reg-errata", NULL))
		dsip_data->has_mipi_errata = 1;
	else
		dsip_data->has_mipi_errata = 0;
	dsip_data->mipi_errata_size = genpll_res->end - genpll_res->start;

	dsip_data->dsi_irq = platform_get_irq(pdev, 0);
	if (dsip_data->dsi_irq < 0) {
		dev_err(&pdev->dev, "DSI interrupt not defined\n");
		goto of_fail;
	}

	dsip_data->axipv_irq = platform_get_irq(pdev, 1);
	if (dsip_data->axipv_irq < 0) {
		dev_err(&pdev->dev, "AXIPV interrupt not defined\n");
		goto of_fail;
	}

	dsip_data->pv_irq = platform_get_irq(pdev, 2);
	if (dsip_data->pv_irq < 0) {
		dev_err(&pdev->dev, "PV interrupt not defined\n");
		goto of_fail;
	}

	dsip_data->clk = devm_clk_get(&pdev->dev, "mipidsi_clk");
	if (IS_ERR(dsip_data->clk)) {
		dev_err(&pdev->dev, "Failed to get MIPIDSI clock\n");
		goto of_fail;
	}

	pnode = of_parse_phandle(np, "dsi_panel", 0);

	of_property_read_string(pnode, "module-name", &str);
	dev_info(&pdev->dev, "DSI Panel found : %s\n", str);
	strcpy(dsip_data->panel_name, str);

	ret = dsi_parse_panel_data(pdev, pnode);
	if (ret) {
		dev_err(&pdev->dev, "Failed to parse panel data %d\n", ret);
		goto of_fail;
	}

	calculate_dsi_clock(panel.disp_info, &clk_rate);
	dev_info(&pdev->dev, "Dsi desired clock rate : %lu\n", clk_rate);

	ret = clk_set_rate(dsip_data->clk, clk_rate);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set to %lu\n", clk_rate);
		goto of_fail;
	}

	dev_info(&pdev->dev, "get actual clock rate %lu",
		 clk_get_rate(dsip_data->clk));

	ret = clk_prepare_enable(dsip_data->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clock\n");
		goto of_fail;
	}

	if (dsi_parse_panel_cmds(pdev, pnode)) {
		dev_err(&pdev->dev, "Failed to parse panel commands\n");
		goto of_fail;
	}

	return dsip_data;

of_fail:
	return NULL;
}

static int __ref dsic_probe(struct platform_device *pdev)
{
	struct dsi_platform_data *dsic_pvt_data;

	dsic_pvt_data = dsi_get_of_data(pdev);

	if (IS_ERR(dsic_pvt_data))
		return PTR_ERR(dsic_pvt_data);

	dsi_pdev = pdev;
	platform_set_drvdata(pdev, dsic_pvt_data);

	init_completion(&dsic_pvt_data->vsync_event);
	dsic_pvt_data->open_count = 0;
	setup_framebuffer(pdev, panel.disp_info);

	dev_info(&pdev->dev, "DSI probe done\n");
	return 0;
}

static int dsic_remove(struct platform_device *pdev)
{
	struct dsi_platform_data *dsic_data = platform_get_drvdata(pdev);

	clk_disable_unprepare(dsic_data->clk);
	return 0;
}

static const struct of_device_id dsic_fb_of_match[] = {
	{ .compatible = "brcm,iproc-dsi", },
	{},
};
MODULE_DEVICE_TABLE(of, dsic_fb_of_match);

static struct platform_driver dsic_driver = {
	.probe = dsic_probe,
	.remove = dsic_remove,
	.driver = {
		.name = "iproc-dsi",
		.of_match_table = dsic_fb_of_match,
	},
};

module_platform_driver(dsic_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom DSI Driver");
MODULE_LICENSE("GPL v2");
