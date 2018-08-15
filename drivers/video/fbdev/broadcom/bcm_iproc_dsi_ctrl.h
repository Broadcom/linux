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

/*  DSI Controller Driver Interface  */

#ifndef __BCM_IPROC_DSI_CTRL_H__
#define __BCM_IPROC_DSI_CTRL_H__

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "bcm_iproc_dsi_lowlevel.h"
#include "bcm_iproc_dsi_panel.h"

#define MAX_NAME_LEN		20

#define INTERNAL_CLK_DIVIDER	4

#define SUSPEND_PRIORITY			11
#define MIPI_DCS_WRITE_MEMORY_START		0x2C
#define MIPI_DCS_WRITE_MEMORY_CONTINUE		0x3C
#define	DSI_DT_LG_DCS_WR			0x39
#define	DSI_DT_SH_SHUT_DOWN			0x22
#define	DSI_DT_SH_TURN_ON			0x32
#define	DSI_DT_SH_DCS_RD_P0			0x06
#define	DSI_DT_SH_GEN_WR_P1			0x13
#define	DSI_DT_SH_GEN_WR_P2			0x23
#define	DSI_DT_SH_DCS_WR_P0			0x05
#define	DSI_DT_SH_DCS_WR_P1			0x15
#define	DSI_DT_LG_GEN_WR			0x29
#define	DSI_DT_SH_DCS_RD_P0			0x06

#define DSI_INITIALIZED				0x13579BDF
#define TX_PKT_ENG_1				((uint8_t)0)
#define DE1_DEF_THRESHOLD_W			(CM_PKT_SIZE_B >> 2)
#define DE1_DEF_THRESHOLD_B			(CM_PKT_SIZE_B)
#define MAX_CMAP_COUNT				16
#define FIFO_ERR_STATUS				22

struct dsi_platform_data {
	void __iomem *dsi_base;
	void __iomem *axipv_base;
	void __iomem *pv_base;
	void __iomem *dsi_genpll_base;
	void __iomem *reset_base;
	uint8_t id;
	int dsi_irq;
	int axipv_irq;
	int pv_irq;
	struct clk *clk;
	bool has_mipi_errata;
	uint32_t mipi_errata_size;
	char panel_name[MAX_NAME_LEN];
	struct fb_info fb;
	bool vmode;
	uint16_t num_fb;
	struct completion vsync_event;
	unsigned int cmap[MAX_CMAP_COUNT];
	uint32_t open_count;
	void *display_hdl;
	bool blank_state;
	dma_addr_t *buff;
	struct gpio_desc *reset_gpio;
};

enum disp_pwr_state {
	CTRL_MIN,
	CTRL_PWR_OFF,		/* PWR Off  : in reset */
	CTRL_PWR_ON,		/* PWR On   : not in reset, init, screen off */
	CTRL_SCREEN_ON,		/* Sleep-out: full power, screen On */
	CTRL_SCREEN_OFF,	/* Sleep-out: full power, screen Off */
	CTRL_MAX,
};

enum dispdrv_fb_format {
	DISPDRV_FB_FORMAT_MIN,		/* Min Number */
	DISPDRV_FB_FORMAT_RGB565,	/* RG5565 2bpp */
	DISPDRV_FB_FORMAT_xRGB8888,	/* xRGB8888 4bpp */
	DISPDRV_FB_FORMAT_xBGR8888,	/* xBGR8888 4bpp */
	DISPDRV_FB_FORMAT_MAX,		/* MAX Number */
};

struct dispdrv_info {
	char *name;
	char *reg_name;
	bool vmode;
	bool cmnd_lp;
	uint8_t te_ctrl;
	uint16_t width;
	uint16_t height;
	uint8_t lanes;
	uint8_t phys_width;
	uint8_t phys_height;
	uint8_t fps;
	enum dispdrv_fb_format in_fmt;
	enum dispdrv_fb_format out_fmt;
	uint8_t bpp;
	char *init_seq;
	char *slp_in_seq;
	char *slp_out_seq;
	char *scrn_on_seq;
	char *scrn_off_seq;
	void *phy_timing;
	uint8_t hs, hbp, hfp, hbllp;
	uint8_t vs, vbp, vfp;
	unsigned int hs_bps;
	unsigned int lp_bps;
	void (*vsync_cb)(void);
	bool sync_pulses;
};

struct bcm_pixel_format {
	uint8_t format;
	uint8_t bpp;
	struct fb_bitfield r;
	struct fb_bitfield g;
	struct fb_bitfield b;
	struct fb_bitfield t;
};

extern struct platform_device *dsi_pdev;

int dsic_init(struct dispdrv_info *display_info, void **handle, int id);
int dsic_exit(void *handle);
int dsic_open(void *handle);
int dsic_close(void *handle);
int dsic_start(void);
int dsic_stop(void);
int dsic_powercontrol(void *handle, enum disp_pwr_state powerstate);
int dsic_update(void *handle, dma_addr_t buff);

#endif		/* __BCM_IPROC_DSI_CTRL_H__ */
