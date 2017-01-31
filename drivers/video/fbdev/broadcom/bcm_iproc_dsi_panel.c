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

#ifndef __PANELS_H__
#define __PANELS_H__

#include <linux/kernel.h>
#include <video/of_display_timing.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <video/videomode.h>

#include "bcm_iproc_dsi_panel.h"

enum dispctrl {
	DISPCTRL_LIST_END,	/* END OF COMMAND LIST */
	DISPCTRL_WR_CMND,	/* DCS write command */
	DISPCTRL_GEN_WR_CMND,	/* Generic write command */
	DISPCTRL_WR_DATA,	/* write data */
	DISPCTRL_SLEEP_MS,	/* SLEEP for <data> msec */
};

struct dispdrv_panel panel = {
	.pwrstate = STATE_PWR_OFF,
};

int dsi_parse_panel_data(struct platform_device *pdev,
					struct device_node *pnode)
{
	unsigned int val;
	struct display_timing timing;
	struct videomode video;

	panel.disp_info = devm_kzalloc(&pdev->dev,
				sizeof(*(panel.disp_info)), GFP_KERNEL);

	if (!panel.disp_info)
		return -ENOMEM;

	if (of_property_read_u32(pnode, "brcm,panel-width-mm", &val))
		goto of_fail;
	panel.disp_info->phys_width = (uint8_t)val;

	if (of_property_read_u32(pnode, "brcm,panel-height-mm", &val))
		goto of_fail;
	panel.disp_info->phys_height = (uint8_t)val;

	of_get_display_timing(pnode, "panel-timing", &timing);
	videomode_from_timing(&timing, &video);

	panel.disp_info->width = video.hactive;
	panel.disp_info->height = video.vactive;
	panel.disp_info->hfp = video.hfront_porch;
	panel.disp_info->hbp = video.hback_porch;
	panel.disp_info->vfp = video.vfront_porch;
	panel.disp_info->vbp = video.vback_porch;
	panel.disp_info->hs = video.hsync_len;
	panel.disp_info->vs = video.vsync_len;

	if (of_property_read_bool(pnode, "brcm,vmode"))
		panel.disp_info->vmode = true;
	else
		panel.disp_info->vmode = false;
	if (of_property_read_bool(pnode, "brcm,cmd-lp"))
		panel.disp_info->cmnd_lp = true;
	else
		panel.disp_info->cmnd_lp = false;
	if (of_property_read_u32(pnode, "brcm,col-mod-i", &val))
		goto of_fail;
	panel.disp_info->in_fmt = (uint8_t)val;

	if (panel.disp_info->in_fmt == DISPDRV_FB_FORMAT_RGB565)
		panel.disp_info->bpp = 2;
	else
		panel.disp_info->bpp = 4;

	if (of_property_read_u32(pnode, "brcm,col-mod-o", &val))
		goto of_fail;
	panel.disp_info->out_fmt = (uint8_t)val;

	if (of_property_read_u32(pnode, "brcm,hs-bitrate", &val))
		goto of_fail;
	panel.disp_info->hs_bps = val;
	if (of_property_read_u32(pnode, "brcm,lp-bitrate", &val))
		goto of_fail;
	panel.disp_info->lp_bps = val;

	if (of_property_read_u32(pnode, "brcm,fps", &val))
		goto of_fail;
	panel.disp_info->fps = (uint8_t)val;
	if (of_property_read_u32(pnode, "brcm,lanes", &val))
		goto of_fail;
	panel.disp_info->lanes = (uint8_t)val;

	return 0;
of_fail:
	devm_kfree(&pdev->dev, panel.disp_info);
	return -EINVAL;
}

static char *get_seq(struct platform_device *pdev, uint8_t *rec)
{
	char *buff, *dst;
	int list_len, cmd_len;
	uint8_t *cur;
	int i;

	buff = NULL;
	/* Get the length of sequence in bytes = cmd+data+headersize+null */
	cur = rec;
	list_len = 0;
	for (i = 0; cur[i] != DISPCTRL_LIST_END; i = i + 2) {
		if (cur[i] == DISPCTRL_WR_DATA)
			list_len++;
		else if (cur[i] == DISPCTRL_GEN_WR_CMND)
			list_len += 3; /* One more tag for Gen cmd */
		else
			list_len += 2; /* Indicates new packet */
	}

	list_len++; /* NULL termination */

	/* Allocate buff = length */
	buff = devm_kmalloc(&pdev->dev, list_len, GFP_KERNEL);
	if (!buff)
		goto seq_done;


	/* Parse the DISPCTRL_REC_T[], extract data and fill buff */
	cur = rec;
	dst = buff;
	i = 0;
	for (i = 0; cur[i] != DISPCTRL_LIST_END; i = i + 2) {
		switch (cur[i]) {
		case DISPCTRL_GEN_WR_CMND:
			*dst++ = DISPCTRL_TAG_GEN_WR;
			/* fall through */
		case DISPCTRL_WR_CMND:
			cmd_len = 1;
			dst++;
			*dst++ = cur[i+1];
			while (cur[i + 2] == DISPCTRL_WR_DATA) {
				i = i + 2;
				*dst++ = cur[i + 1];
				cmd_len++;
			}
			if (cmd_len > DISPCTRL_MAX_DATA_LEN) {
				dev_err(&pdev->dev, "cmd_len %d reached max\n",
								cmd_len);
				devm_kfree(&pdev->dev, buff);
				buff = NULL;
				goto seq_done;
			}
			*(dst - cmd_len - 1) = cmd_len;
			break;
		case DISPCTRL_SLEEP_MS:
			/* Maximum packet size is limited to 254 */
			*dst++ = DISPCTRL_TAG_SLEEP;
			*dst++ = cur[i+1];
			break;
		default:
			dev_err(&pdev->dev, "Invalid ctrl list %d\n", cur[i]);
		}
	}

	/* Put a NULL at the end */
	*dst = 0;

seq_done:
	return buff;
}

int dsi_parse_panel_cmds(struct platform_device *pdev,
					struct device_node *pnode)
{
	int count;
	uint8_t *cmds;

	of_get_property(pnode, "brcm,slp-in", &count);
	cmds = devm_kcalloc(&pdev->dev, count, sizeof(uint8_t), GFP_KERNEL);
	if (!cmds)
		return -ENOMEM;
	of_property_read_u8_array(pnode, "brcm,slp-in", cmds, count);
	panel.disp_info->slp_in_seq = get_seq(pdev, cmds);
	devm_kfree(&pdev->dev, cmds);
	if (!panel.disp_info->slp_in_seq)
		return -EINVAL;

	of_get_property(pnode, "brcm,slp-out", &count);
	cmds = devm_kcalloc(&pdev->dev, count, sizeof(uint8_t), GFP_KERNEL);
	if (!cmds)
		return -ENOMEM;
	of_property_read_u8_array(pnode, "brcm,slp-out", cmds, count);
	panel.disp_info->slp_out_seq = get_seq(pdev, cmds);
	devm_kfree(&pdev->dev, cmds);
	if (!panel.disp_info->slp_out_seq)
		return -EINVAL;

	of_get_property(pnode, "brcm,scrn-on", &count);
	cmds = devm_kcalloc(&pdev->dev, count, sizeof(uint8_t), GFP_KERNEL);
	if (!cmds)
		return -ENOMEM;
	of_property_read_u8_array(pnode, "brcm,scrn-on", cmds, count);
	panel.disp_info->scrn_on_seq = get_seq(pdev, cmds);
	devm_kfree(&pdev->dev, cmds);
	if (!panel.disp_info->scrn_on_seq)
		return -EINVAL;

	of_get_property(pnode, "brcm,scrn-off", &count);
	cmds = devm_kcalloc(&pdev->dev, count, sizeof(uint8_t), GFP_KERNEL);
	if (!cmds)
		return -ENOMEM;
	of_property_read_u8_array(pnode, "brcm,scrn-off", cmds, count);
	panel.disp_info->scrn_off_seq = get_seq(pdev, cmds);
	devm_kfree(&pdev->dev, cmds);
	if (!panel.disp_info->scrn_off_seq)
		return -EINVAL;

	of_get_property(pnode, "brcm,init-panel", &count);
	cmds = devm_kcalloc(&pdev->dev, count, sizeof(uint8_t), GFP_KERNEL);
	if (!cmds)
		return -ENOMEM;
	of_property_read_u8_array(pnode, "brcm,init-panel", cmds, count);
	panel.disp_info->init_seq = get_seq(pdev, cmds);
	devm_kfree(&pdev->dev, cmds);
	if (!panel.disp_info->init_seq)
		return -EINVAL;

	of_get_property(pnode, "brcm,dsi-timing", &count);
	panel.disp_info->phy_timing = devm_kcalloc(&pdev->dev,
			count / sizeof(uint32_t), sizeof(uint32_t), GFP_KERNEL);
	if (!panel.disp_info->phy_timing)
		return -ENOMEM;
	of_property_read_u32_array(pnode, "brcm,dsi-timing",
				(panel.disp_info->phy_timing),
					count / sizeof(uint32_t));

	return 0;
}

#endif /*__PANELS_H__*/
