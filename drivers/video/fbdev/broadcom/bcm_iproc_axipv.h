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

#ifndef __BCM_IPROC_AXIPV_H__
#define __BCM_IPROC_AXIPV_H__

#define CM_PKT_SIZE_B		768

/* Register offsets */
#define REG_NXT_FRAME		0x00
#define REG_CUR_FRAME		0x04
#define REG_LINE_STRIDE		0x0C
#define REG_BYTES_PER_LINE	0x10
#define REG_LINES_PER_FRAME	0x14
#define REG_BURST_LEN		0x18
#define REG_W_LVL_1		0x1C
#define REG_W_LVL_2		0x20
#define REG_PV_THRESH		0x24
#define REG_TE_LINE		0x28
#define REG_CTRL		0x2C
#define REG_AXIPV_STATUS	0x30
#define REG_INTR_EN		0x34
#define REG_INTR_STAT		0x38
#define REG_INTR_CLR		0x3C
#define REG_AXI_ID_CFG_1	0x40
#define REG_AXI_ID_CFG_2	0x44
#define REG_LB_EMPTY_THRES	0x48

#define PV_START_THRESH_INT	BIT(7)
#define AXIPV_DISABLED_INT	BIT(6)
#define FRAME_END_INT		BIT(5)
#define WATER_LVL2_INT		BIT(1)
#define TE_INT			BIT(0)

#define ARCACHE_SHIFT		15
#define ARPROT_SHIFT		12
#define AXI_ID_SYS_DUAL			BIT(11)
#define NUM_OUTSTDG_XFERS_8		(3<<9)
#define PIXEL_FORMAT_SHIFT		5
#define AXIPV_ACTIVE		BIT(4)
#define SFT_RSTN_DONE		BIT(1)
#define AXIPV_EN		BIT(0)

/* In a multiple of 8bytes */
#define AXIPV_LB_SIZE		(16 * 1024/8)

#define AXIPV_BURST_LEN		7
#define AXIPV_PV_THRES		((AXIPV_LB_SIZE * 7) / 8)
#define AXIPV_W_LVL_1		((AXIPV_LB_SIZE * 5) / 8 - 1)
#define AXIPV_W_LVL_2		(AXIPV_LB_SIZE / 4)
#define AXIPV_LB_EMPTY_THRES	(AXIPV_LB_SIZE / 20)
#define AXIPV_LB_EMPTY_THRES_MIN	0
#define AXIPV_W_LVL_2_MIN	(AXIPV_LB_EMPTY_THRES_MIN + 1)
#define AXIPV_W_LVL_1_MIN	(AXIPV_W_LVL_2_MIN + 1)
#define AXIPV_PV_THRES_MIN	(AXIPV_W_LVL_1_MIN + 1)
#define AXIPV_AXI_ID1		0
#define AXIPV_AXI_ID2		0x80
#define AXIPV_ARPROT		(2 << ARPROT_SHIFT)
#define AXIPV_ARCACHE		(2 << ARCACHE_SHIFT)

struct axipv_sync_buf {
	dma_addr_t addr;
	unsigned int xlen;
	unsigned int ylen;
};

struct axipv_init {
	struct device *dsi_dev;
	unsigned int irq;
	void __iomem *base_addr;
	void (*irq_cb)(int err);
	void (*vsync_cb)(void);
};

struct axipv_config {
	union {
		unsigned int async;
		struct axipv_sync_buf sync;
	} buff;
	unsigned int width;
	unsigned int height;
	uint8_t pix_fmt;
};

enum {
	AXIPV_PIXEL_FORMAT_24BPP_RGB,
	AXIPV_PIXEL_FORMAT_24BPP_BGR,
	AXIPV_PIXEL_FORMAT_16BPP_PACKED,
	AXIPV_PIXEL_FORMAT_16BPP_UNPACKED,
	AXIPV_PIXEL_FORMAT_18BPP_PACKED,
	AXIPV_PIXEL_FORMAT_18BPP_UNPACKED
};

enum {
	AXIPV_RESET,
	AXIPV_CONFIG,
	AXIPV_START,
	AXIPV_STOP_EOF,
	AXIPV_MAX_EVENT
};

int axipv_init(struct axipv_init *init, struct axipv_config **config,
						bool display_enabled);
int axipv_change_state(unsigned int event, struct axipv_config *config);
int axipv_wait_for_stop(struct axipv_config *config);
int axipv_post(struct axipv_config *config);
int axipv_check_completion(unsigned int event, struct axipv_config *config);

#endif /* __BCM_IPROC_AXIPV_H__ */
