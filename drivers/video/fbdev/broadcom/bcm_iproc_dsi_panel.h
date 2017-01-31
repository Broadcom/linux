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

#include <linux/kernel.h>

#include "bcm_iproc_dsi_ctrl.h"

#ifndef __BCM_IPROC_DSI_PANEL_H__
#define __BCM_IPROC_DSI_PANEL_H__

#define DISPCTRL_TAG_SLEEP	((uint8_t)~0)
#define DISPCTRL_TAG_GEN_WR	(DISPCTRL_TAG_SLEEP - 1)

/* Maximum packet size is limited to 253 */
#define DISPCTRL_MAX_DATA_LEN	(DISPCTRL_TAG_GEN_WR - 1)

enum display_drv_state {
	DRV_STATE_OFF,
	DRV_STATE_INIT,
	DRV_STATE_OPEN,
};

enum display_pwr_state {
	STATE_PWR_OFF,		/* PWR Off, in reset */
	STATE_SLEEP,		/* Sleep-in , Screen Off */
	STATE_SCREEN_ON,	/* Sleep-out, Screen On */
	STATE_SCREEN_OFF,	/* Sleep-out, Screen Off */
};

struct dispdrv_win {
	unsigned int l;
	unsigned int t;
	unsigned int r;
	unsigned int b;
	unsigned int w;
	unsigned int h;
	unsigned int mode; /*0=android use case, !=0 win has 0 offset in fb */
};

struct dispdrv_panel {
	void *client_handle;
	void *dsicmvc_handle;
	enum display_drv_state drvstate;
	enum display_pwr_state pwrstate;
	struct dispdrv_win win_dim;
	struct dsi_cm_vc *cmnd_mode;
	struct dsi_cfg *dsi_cfg;
	struct dispdrv_info *disp_info;
	uint8_t maxretpktsize;
	unsigned int id;
	bool display_enabled;
	bool video_enabled;
};

extern struct dispdrv_panel panel;

int dsic_panel_on(struct dispdrv_panel *panel_t, bool on);

int dsi_parse_panel_cmds(struct platform_device *pdev,
					struct device_node *pnode);

int dsi_parse_panel_data(struct platform_device *pdev,
					struct device_node *pnode);

#endif /* __BCM_IPROC_DSI_PANEL_H__ */
