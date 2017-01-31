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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include "bcm_iproc_pv.h"

#define FIFO_SIZE		64
#define FIFO_THRE_TO_FETCH	(FIFO_SIZE - 32)

struct pv_dev {
	void __iomem *base_addr;
	struct device *dsi_dev;
	unsigned int irq;  /*Interrupt ID*/
	bool stopped;
	int irq_stat;
	void (*eof_cb)(void); /* End-Of-Frame Callback- run in ISR context */
	struct pv_config vid_config;
};

static DEFINE_SPINLOCK(lock);

static void err_cb(struct pv_dev *dev)
{
	unsigned long flags;
	int irq_stat;

	spin_lock_irqsave(&lock, flags);
	irq_stat = dev->irq_stat;
	dev->irq_stat = 0;
	spin_unlock_irqrestore(&lock, flags);
}

static void eof_cb(struct pv_dev *dev)
{
	void __iomem *pv_base = dev->base_addr;

	if (dev->stopped == true) {
		writel(0, pv_base + REG_PV_INTEN);
		writel(readl(pv_base + REG_PV_C) & ~PVEN, pv_base + REG_PV_C);
	}
	if (dev->eof_cb)
		dev->eof_cb();
}

static irqreturn_t pv_isr(int irq, void *dev_data)
{
	void __iomem *pv_base;
	uint32_t irq_stat;
	struct pv_dev *dev = dev_data;

	pv_base = dev->base_addr;
	irq_stat = readl(pv_base + REG_PV_INTSTAT);

	if (irq_stat & VFP_END) {
		dev->stopped = true;
		writel(VFP_END, pv_base + REG_PV_INTSTAT);
		irq_stat = irq_stat & ~VFP_END;
		eof_cb(dev);
	}


	if (irq_stat & OF_UF) {
		dev->irq_stat |= OF_UF;
		err_cb(dev);
		writel(HVS_UF | HVS_OF | PV_UF, pv_base + REG_PV_STAT);
		writel(OF_UF, pv_base + REG_PV_INTSTAT);
		irq_stat = irq_stat & ~OF_UF;
	}

	if (irq_stat)
		writel(irq_stat, pv_base + REG_PV_INTSTAT);

	return IRQ_HANDLED;
}

int pv_init(struct pv_init *init, struct pv_config **config)
{
	struct pv_dev *dev;
	int ret;

	if (!init || !init->base_addr || !init->eof_cb)
		return -EINVAL;

	dev = devm_kzalloc(init->dsi_dev, sizeof(struct pv_dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	ret = devm_request_irq(init->dsi_dev, init->irq, pv_isr,
						IRQF_TRIGGER_HIGH, "PV", dev);
	if (ret < 0) {
		dev_err(init->dsi_dev, "failed to get irq\n");
		return ret;
	}
	dev->irq = init->irq;
	dev->base_addr = init->base_addr;
	dev->dsi_dev = init->dsi_dev;
	dev->eof_cb = init->eof_cb;
	dev->stopped = true;
	*config = &dev->vid_config;

	return 0;

}

int pv_vid_config(struct pv_config *vid_config)
{
	void __iomem *pv_base;
	unsigned int pv_c = 0, pv_vc = 0;
	struct pv_dev *dev;

	dev = container_of(vid_config, struct pv_dev, vid_config);
	pv_base = dev->base_addr;

	/*Clear the FIFO and disable the PV*/
	writel(FIFO_CLR, pv_base + REG_PV_C);
	writel(0, pv_base + REG_PV_INTEN);
	/*Ack previous interrupts*/
	writel(UINT_MAX, pv_base + REG_PV_INTSTAT);
	writel_relaxed(vid_config->hs | (vid_config->hbp << PORCH_SHIFT),
			pv_base + REG_PV_HORZA);
	writel_relaxed((vid_config->hact + vid_config->hbllp) |
		(vid_config->hfp << PORCH_SHIFT), pv_base + REG_PV_HORZB);
	writel_relaxed((vid_config->hact), pv_base + REG_PV_DSI_HACT_ACT);
	writel_relaxed((vid_config->vs + 1) |
		((vid_config->vbp - 1) << PORCH_SHIFT), pv_base + REG_PV_VERTA);
	writel_relaxed(vid_config->vact | (vid_config->vfp << PORCH_SHIFT),
			pv_base + REG_PV_VERTB);

	pv_c = (vid_config->pix_stretch << PIX_STRETCH_SHIFT) |
		   (vid_config->pclk_sel << PCLK_SEL_SHIFT) |
		   (vid_config->pix_fmt << PIX_F_SHIFT) |
		   (FIFO_THRE_TO_FETCH << FIFO_FULL_LEVEL_SHIFT);
	writel_relaxed(pv_c, pv_base + REG_PV_C);

	pv_vc = FRAMEC | DSI_VMODE | (vid_config->vsyncd << VSYNCD_SHIFT);
	writel(pv_vc, pv_base + REG_PV_VC);

	return 0;
}

int pv_start(struct pv_config *vid_config)
{
	void __iomem *pv_base;
	struct pv_dev *dev;
	unsigned long flags;

	if (!vid_config)
		return -EINVAL;

	dev = container_of(vid_config, struct pv_dev, vid_config);
	pv_base = dev->base_addr;

	spin_lock_irqsave(&lock, flags);
	writel((readl(pv_base + REG_PV_INTEN) & ~VFP_END),
						pv_base + REG_PV_INTEN);
	writel(readl(pv_base + REG_PV_C) | PVEN, pv_base + REG_PV_C);
	writel(readl(pv_base + REG_PV_VC) | VIDEN, pv_base + REG_PV_VC);
	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

int pv_send_event(int event, struct pv_config *vid_config)
{
	void __iomem *pv_base;
	struct pv_dev *dev;
	unsigned long flags;

	if (!vid_config)
		return -EINVAL;

	dev = container_of(vid_config, struct pv_dev, vid_config);
	pv_base = dev->base_addr;

	spin_lock_irqsave(&lock, flags);
	writel(readl(pv_base + REG_PV_VC) & ~VIDEN, pv_base + REG_PV_VC);
	if (event == PV_STOP_EOF_ASYNC)
		writel((readl(pv_base + REG_PV_INTEN) | VFP_END),
						pv_base + REG_PV_INTEN);
	else if (event == PV_STOP_IMM)
		writel(readl(pv_base + REG_PV_C) & ~PVEN, pv_base + REG_PV_C);

	dev->stopped = (event == PV_STOP_IMM) ? true : false;
	spin_unlock_irqrestore(&lock, flags);

	return 0;
}
