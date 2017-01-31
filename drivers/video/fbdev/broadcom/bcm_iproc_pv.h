/*
 * Copyright (C) 2017 Broadcom Ltd
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

#ifndef __BCM_IPROC_PV_H__
#define __BCM_IPROC_PV_H__

#define REG_PV_C		0x00
#define REG_PV_VC		0x04
#define REG_PV_HORZA		0x0C
#define REG_PV_HORZB		0x10
#define REG_PV_VERTA		0x14
#define REG_PV_VERTB		0x18
#define REG_PV_INTEN		0x24
#define REG_PV_INTSTAT		0x28
#define REG_PV_STAT		0x2C
#define REG_PV_DSI_HACT_ACT	0x30

#define PIX_F_SHIFT		21
#define FIFO_FULL_LEVEL_SHIFT	15

#define PIX_STRETCH_SHIFT	4
#define PCLK_SEL_SHIFT		2
#define FIFO_CLR		BIT(1)
#define PVEN			BIT(0)

#define VSYNCD_SHIFT		6
#define DSI_VMODE		BIT(3)
#define FRAMEC			BIT(1)
#define VIDEN			BIT(0)

#define OF_UF			BIT(10)
#define VFP_END			BIT(8)

#define HVS_OF			BIT(11)
#define PV_UF			BIT(10)
#define HVS_UF			BIT(9)

#define PORCH_SHIFT	16

#define DISP_CTRL_DSI	0

struct pv_init {
	struct device *dsi_dev;
	unsigned int irq;
	void __iomem *base_addr;
	void (*eof_cb)(void); /* End-Of-Frame Callback- run in ISR context */
};

struct pv_config {
	u8 pclk_sel;	/* DISP_CTRL type*/
	u8 pix_fmt;	/* PIX_FMT type */
	unsigned int vsyncd;	/* Vsync delay*/
	u8 pix_stretch;	/* No. of times to repeat the same pixel*/
	uint16_t vs;	/* Vertical Sync width */
	uint16_t vbp;	/* Vertical Back Porch width */
	uint16_t vact;	/* Vertical ACTive width */
	uint16_t vfp;	/* Vertical Front Porch wdith*/
	uint16_t hs;	/* Horizontal Sync width */
	uint16_t hbp;	/* Horizontal Back Porch width */
	uint16_t hact;	/* Horizontal ACTive width */
	uint16_t hfp;	/* Horizontal Front Porch wdith*/
	uint16_t hbllp;	/* Horizontal BLLP wdith*/
};

enum {
	HDMI_VEC_SMI_24BPP,
	DSI_VIDEO_16BPP,
	DSI_CMD_16BPP,
	PACKED_DSI_VIDEO_18BPP,
	DSI_VIDEO_CMD_18_24BPP,
	PIX_FMT_TYPE_MAX
};

enum {
	PV_RESET,
	PV_PAUSE_STREAM_SYNC,
	PV_STOP_EOF_ASYNC,
	PV_STOP_IMM,
	PV_MAX_EVENT
};

int pv_init(struct pv_init *init, struct pv_config **config);
int pv_vid_config(struct pv_config *vid_config);
int pv_start(struct pv_config *vid_config);
int pv_send_event(int event, struct pv_config *config);

#endif /* __BCM_IPROC_PV_H__ */
