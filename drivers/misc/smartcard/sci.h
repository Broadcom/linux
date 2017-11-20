/*
 * Copyright (C) 2017 Broadcom
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

#ifndef IPROC_SCI_H
#define IPROC_SCI_H
#include <linux/cdev.h>

/*
 * IOCTL number for setting of LCD buffer address .
 * this is used for external allocation, like using VDEC buf for LCD display.
 * 'S' is for smartcard, 0x55 is a number >= 0x20.
 */
#define FBIO_SET_BUFFER_ADDR _IOW('S', 0x55, __u32)

struct sci_ctx {
	/*
	 * A pointer, contains array of channel handles,
	 * allocated and initialized in open()
	 */
	struct bscd_p_handle *mod_hdl;
	dev_t devt;   /* allocated device number */
	struct cdev sci_cdev;
	struct class *sci_class;
	/* Used for power management */
	unsigned int busy;
	struct mutex sc_mutex;
	struct device *dev;
	void __iomem  *sci_regs;
	void __iomem  *scirq_regmap;
	struct clk *clk;
	unsigned int sci_id;
	unsigned int gpio;
	unsigned int coupler;
#define MAX_SCI_INTR 3
	unsigned int irq;
	unsigned int irqen_off;
};

/*
 * This API is called to regiser char as well as other
 * fs interfacs.
 * @pctx -pointer of sci context.
 *
 * retrun value:
 * 0 if success else negative error code.
 */

int init_fs_intfs(struct sci_ctx *pctx);

/*
 * This API removes all the registered fs interfaces.
 * @pctx -pointer of sci context.
 */

void rm_fs_intfs(struct sci_ctx *pctx);
#endif
