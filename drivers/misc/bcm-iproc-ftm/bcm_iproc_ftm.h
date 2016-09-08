/*
* Copyright (C) 2016 Broadcom
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License as
* published by the Free Software Foundation version 2.
*
* This program is distributed "as is" WITHOUT ANY WARRANTY of any
* kind, whether express or implied; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/

#ifndef __BCM_IPROC_FTM_H__
#define __BCM_IPROC_FTM_H__

#define CLK_PRESCALE 0xC00
#define FLEX_TIMER_CTRL1 0xC0C
#define FTM_CTRL2 0xC10
#define FTM_INTR_STATUS 0xC04
#define FTM_INTR_MASK 0xC08
#define FTM_DATA_REG0 0xC20
#define FTM_STATUS0 0xC14

#define CONFIG_TIMEOUT 12
#define CONFIG_TIMEOUT_MASK (0xFFFFF << CONFIG_TIMEOUT)
#define WATERMARK_CHAN2 8
#define WATERMARK_CHAN3 16
#define TIMEOUT_EN BIT(11)
#define FTM_EDGE_LH (7 << 0)
#define FTM_EDGE_HL (7 << 4)

#define INTR_SOS BIT(18)
#define INTR_TIMEOUT BIT(17)
#define INTR_WATERMARK 8
#define INTR_FIFO_FULL 11

/* TSC register controlling FTM*/
#define TSC_REG_CTRL2 0x4
#define TSC_FTM_ENA BIT(17)

#define TSC_INTR_MASK_REG 0x00c
#define TSC_FTM_INTR_MASK BIT(8)

#define FTM_MAX_CHNL 3
#define MAX_TS_ENTRIES 1024
#define MAX_TS_BUFF_SIZE (2 * MAX_TS_ENTRIES)
#define MAX_TRACK_DATA_SIZE 128

#define TIMEOUT_VALUE 0xFFFF
#define WATERMARK_VALUE 32
#define INTR_MASK	((7 << INTR_WATERMARK) |\
			 (7 << INTR_FIFO_FULL) | \
			 INTR_SOS | INTR_TIMEOUT)

#define CLK_PRESCALE_DIVIDER 0x08
#define FTM_STATUS0_ENTRIES_MASK 0xff
#define FTM_USER_MASK(minor) (1 << minor)

struct ftm_chnl_desc {
	u16 *ts_buf;  /* buffer to hold time stamp values*/
	u32 ts_cnt; /* ts counter */
	u8 *track_data; /* decoded magnetic stripe track data */
	u32 data_cnt; /* decoded data char cnt */
	bool drdy; /* flag to indicate data ready to user*/
	wait_queue_head_t wq;
};

struct iproc_ftm {
	struct device *dev;
	struct cdev cdev;
	struct class *class;
	u32 users;
	int irq;
	dev_t devno;
	int timeoutc;
	int sosc;
	struct regmap *regmap;
	struct mutex ulock; /* user lock */
	struct clk *ftm_clk;
	struct ftm_chnl_desc ch[FTM_MAX_CHNL];
};

extern int ftm_decode(u16 *p, u32 size, int chan, u8 *out, u32 *outlen);
#endif
