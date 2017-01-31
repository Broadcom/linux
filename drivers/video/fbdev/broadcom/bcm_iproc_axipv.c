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
#include <linux/interrupt.h>
#include <linux/io.h>

#include "bcm_iproc_axipv.h"

#define RETRY_COUNT	10

struct axipv_dev {
	spinlock_t lock;
	struct device *dsi_dev;
	void __iomem *base_addr;
	unsigned int nxt, curr;
	uint8_t state;
	int irq_stat;
	bool prev_irq_handled;
	unsigned int irq;
	struct axipv_config config;
	struct work_struct irq_work;
	void (*irq_cb)(int err);
	void (*vsync_cb)(void);
};

enum {
	AXIPV_INIT_DONE,
	AXIPV_STOPPED,
	AXIPV_CONFIGURED,
	AXIPV_ENABLED,
	AXIPV_STOPPING,
	AXIPV_MAX_STATE,
	AXIPV_INVALID_STATE
};

static void process_irq(struct work_struct *work)
{
	struct axipv_dev *dev = container_of(work, struct axipv_dev, irq_work);
	int irq_stat;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	irq_stat = dev->irq_stat;
	dev->irq_stat = 0;
	dev->prev_irq_handled = true;
	spin_unlock_irqrestore(&dev->lock, flags);

	dev->irq_cb(irq_stat);
}

static inline int axipv_config(struct axipv_config *config)
{
	unsigned int ctrl = 0;
	struct axipv_dev *dev;
	void __iomem *axipv_base;
	unsigned int tx_size;
	int arcache = AXIPV_ARCACHE;
	struct axipv_sync_buf *buff;
	bool is_aligned;

	if (!config)
		return -EINVAL;

	dev = container_of(config, struct axipv_dev, config);
	axipv_base = dev->base_addr;

	buff = &config->buff.sync;

	is_aligned = IS_ALIGNED(buff->addr, 4) | IS_ALIGNED(buff->xlen, 4) |
					IS_ALIGNED(config->width, 4);
	if (unlikely(!is_aligned))
		arcache = 0;

	writel_relaxed(buff->xlen, axipv_base + REG_BYTES_PER_LINE);
	writel_relaxed(buff->ylen, axipv_base + REG_LINES_PER_FRAME);
	writel_relaxed(buff->addr, axipv_base + REG_NXT_FRAME);
	dev->nxt = buff->addr;
	dev->curr = buff->addr;

	writel_relaxed(config->width, axipv_base + REG_LINE_STRIDE);
	writel_relaxed(AXIPV_BURST_LEN, axipv_base + REG_BURST_LEN);
	writel_relaxed(config->height - 1, axipv_base + REG_TE_LINE);
	writel_relaxed(0, axipv_base + REG_INTR_EN);
	writel_relaxed(UINT_MAX, axipv_base + REG_INTR_CLR);
	dev->prev_irq_handled = true;
	dev->irq_stat = 0;

	writel_relaxed(AXIPV_AXI_ID1, axipv_base + REG_AXI_ID_CFG_1);
	writel_relaxed(AXIPV_AXI_ID2, axipv_base + REG_AXI_ID_CFG_2);

	ctrl =  SFT_RSTN_DONE | NUM_OUTSTDG_XFERS_8 |
				AXI_ID_SYS_DUAL | AXIPV_ARPROT | arcache;
	ctrl |= (config->pix_fmt << PIXEL_FORMAT_SHIFT);

	tx_size = 2 * buff->xlen * buff->ylen;
	if ((config->pix_fmt == AXIPV_PIXEL_FORMAT_24BPP_RGB)
			|| (config->pix_fmt == AXIPV_PIXEL_FORMAT_24BPP_BGR))
		tx_size *= 2;
	if (tx_size > AXIPV_PV_THRES) {
		writel_relaxed(AXIPV_PV_THRES, axipv_base + REG_PV_THRESH);
		writel_relaxed(AXIPV_W_LVL_1, axipv_base + REG_W_LVL_1);
		writel_relaxed(AXIPV_W_LVL_2, axipv_base + REG_W_LVL_2);
		writel_relaxed(AXIPV_LB_EMPTY_THRES, axipv_base
							+ REG_LB_EMPTY_THRES);
	} else if (tx_size >= AXIPV_PV_THRES_MIN) {
		writel_relaxed(tx_size, axipv_base + REG_PV_THRESH);
		writel_relaxed(AXIPV_W_LVL_1_MIN, axipv_base + REG_W_LVL_1);
		writel_relaxed(AXIPV_W_LVL_2_MIN, axipv_base + REG_W_LVL_2);
		writel_relaxed(AXIPV_LB_EMPTY_THRES_MIN, axipv_base
							+ REG_LB_EMPTY_THRES);
	} else {
		dev_err(dev->dsi_dev, "tx_size=%d is less to send\n", tx_size);
		return -EINVAL;
	}

	writel(ctrl, axipv_base + REG_CTRL);

	return 0;
}

static bool __is_fifo_draining_eof(struct axipv_dev *dev)
{
	void __iomem *axipv_base;
	unsigned int curr_line_num;

	axipv_base = dev->base_addr;
	curr_line_num = readl(axipv_base + REG_AXIPV_STATUS) >> 16;
	if (!curr_line_num || ((dev->config.height - curr_line_num)
				< (AXIPV_LB_SIZE / (dev->config.width >> 2))))
		return true;
	return false;
}

static irqreturn_t axipv_isr(int err, void *dev_id)
{
	struct axipv_dev *dev = dev_id;
	int irq_stat;
	void __iomem *axipv_base;
	bool disable_axipv = false;
	unsigned int ctrl, curr_reg_val;
	unsigned long flags;

	axipv_base = dev->base_addr;
	spin_lock_irqsave(&dev->lock, flags);
	irq_stat = readl(axipv_base + REG_INTR_STAT);

	if ((irq_stat & WATER_LVL2_INT) && ((dev->state == AXIPV_STOPPED)
			|| ((dev->state == AXIPV_STOPPING) &&
					__is_fifo_draining_eof(dev)))) {
		irq_stat = irq_stat & ~WATER_LVL2_INT;
		writel(WATER_LVL2_INT, axipv_base + REG_INTR_CLR);
	}

	if (irq_stat & FRAME_END_INT) {
		curr_reg_val = readl(axipv_base + REG_CUR_FRAME);
		if (dev->curr != curr_reg_val) {
			dev->curr = curr_reg_val;
			if (dev->state == AXIPV_STOPPING)
				dev->state = AXIPV_STOPPED;
		}
		writel(FRAME_END_INT, axipv_base + REG_INTR_CLR);
		if (dev->state != AXIPV_STOPPED)
			irq_stat = irq_stat & ~FRAME_END_INT;
	}
	if (irq_stat & AXIPV_DISABLED_INT) {
		dev->curr = dev->nxt = 0;
		disable_axipv = true;
		ctrl = readl(axipv_base+REG_CTRL);
		ctrl &= ~(BIT(8) | BIT(29));
		writel(ctrl, axipv_base + REG_CTRL);
	}
	if ((irq_stat & TE_INT) && dev->vsync_cb) {
		dev->vsync_cb();
		writel(TE_INT, axipv_base + REG_INTR_CLR);
		irq_stat &= ~TE_INT;
	}

	if (irq_stat) {
		if (dev->prev_irq_handled) {
			dev->irq_stat = irq_stat;
			dev->prev_irq_handled = false;
		} else {
			dev->irq_stat |= irq_stat;
		}
		schedule_work(&dev->irq_work);
		writel(irq_stat, axipv_base + REG_INTR_CLR);
	}
	if (disable_axipv) {
		dev->state = AXIPV_STOPPED;
		writel(AXIPV_DISABLED_INT, axipv_base + REG_INTR_CLR);
	}
	spin_unlock_irqrestore(&dev->lock, flags);

	return IRQ_HANDLED;
}

int axipv_init(struct axipv_init *init, struct axipv_config **config,
							bool display_enabled)
{
	struct axipv_dev *dev;
	int ret;

	if (!init || !init->irq || !init->base_addr || !init->irq_cb)
		return -EINVAL;

	dev = devm_kzalloc(init->dsi_dev, sizeof(struct axipv_dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	ret = devm_request_irq(init->dsi_dev, init->irq, axipv_isr,
					IRQF_TRIGGER_HIGH, "axipv", dev);
	if (ret < 0) {
		dev_err(init->dsi_dev, "irq request failed\n");
		return ret;
	}
	dev->dsi_dev = init->dsi_dev;
	dev->base_addr = init->base_addr;
	dev->irq_cb = init->irq_cb;
	dev->vsync_cb = init->vsync_cb;
	dev->irq = init->irq;
	INIT_WORK(&dev->irq_work, process_irq);
	dev->state = AXIPV_INIT_DONE;
	*config = &dev->config;

	return 0;
}

int axipv_post(struct axipv_config *config)
{
	unsigned int curr_reg_val;
	struct axipv_dev *dev;
	void __iomem *axipv_base;

	dev = container_of(config, struct axipv_dev, config);
	axipv_base = dev->base_addr;

	if ((config->buff.async == dev->curr) ||
					(config->buff.async == dev->nxt)) {
		dev_info(dev->dsi_dev, "Likely tearing on screen posted:");
		return 0;
	}

	writel(config->buff.async, axipv_base + REG_NXT_FRAME);
	curr_reg_val = readl(axipv_base + REG_CUR_FRAME);

	if (dev->nxt && curr_reg_val != dev->nxt)
		dev->nxt = 0;
	dev->nxt = config->buff.async;

	return 0;
}

int axipv_change_state(unsigned int event, struct axipv_config *config)
{
	int ret = 0, cnt;
	struct axipv_dev *dev;
	void __iomem *axipv_base;
	unsigned long flags;
	unsigned long intr_en = UINT_MAX;

	if (!config)
		return -EINVAL;

	dev = container_of(config, struct axipv_dev, config);
	axipv_base = dev->base_addr;

	switch (event) {
	case AXIPV_CONFIG:
		cnt = RETRY_COUNT;
		while (cnt-- && (dev->state == AXIPV_STOPPING))
			usleep_range(1000, 1100);
		ret = axipv_config(config);
		if (!ret)
			dev->state = AXIPV_CONFIGURED;
		break;
	case AXIPV_START:
		spin_lock_irqsave(&dev->lock, flags);
		writel_relaxed(intr_en, axipv_base + REG_INTR_EN);
		writel(readl(axipv_base + REG_CTRL)
			 | AXIPV_EN, axipv_base + REG_CTRL);
		dev->state = AXIPV_ENABLED;
		spin_unlock_irqrestore(&dev->lock, flags);
		break;
	case AXIPV_STOP_EOF:
		spin_lock_irqsave(&dev->lock, flags);
		writel(readl(axipv_base + REG_CTRL) & ~AXIPV_EN,
						axipv_base + REG_CTRL);
		dev->state = AXIPV_STOPPING;
		spin_unlock_irqrestore(&dev->lock, flags);
		break;
	default:
		dev_err(dev->dsi_dev, "Invalid event:%d\n", event);
		ret = -EINVAL;
		break;
	}
	return ret;
}

int axipv_wait_for_stop(struct axipv_config *config)
{
	void __iomem *axipv_base;
	unsigned int intr, ctrl;
	struct axipv_dev *dev;
	unsigned long flags;
	int count;

	if (!config)
		return -EINVAL;

	dev = container_of(config, struct axipv_dev, config);
	axipv_base = dev->base_addr;

	for (count = 0; count < RETRY_COUNT; count++) {
		ctrl = readl(axipv_base + REG_CTRL);
		intr = readl(axipv_base + REG_INTR_STAT);
		if (!(ctrl & AXIPV_ACTIVE) || (intr & PV_START_THRESH_INT))
			break;

		usleep_range(1000, 1100);
	}
	writel(intr, axipv_base + REG_INTR_CLR);

	if (count == RETRY_COUNT)
		return -ETIMEDOUT;
	if (ctrl)
		return 0;

	spin_lock_irqsave(&dev->lock, flags);
	dev->state = AXIPV_STOPPED;
	dev->curr = dev->nxt = 0;
	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

int axipv_check_completion(unsigned int event, struct axipv_config *config)
{
	int ret = -EINVAL;
	void __iomem *axipv_base;
	struct axipv_dev *dev;
	unsigned long flags;
	unsigned int int_stat, int_en;
	uint32_t curr_buff;

	if (!config)
		return ret;

	dev = container_of(config, struct axipv_dev, config);
	axipv_base = dev->base_addr;

	spin_lock_irqsave(&dev->lock, flags);
	int_stat = readl(axipv_base + REG_INTR_STAT);
	int_en = readl(axipv_base + REG_INTR_EN);

	switch (event) {
	case AXIPV_START:
		if (int_stat & PV_START_THRESH_INT)
			writel(PV_START_THRESH_INT, axipv_base + REG_INTR_CLR);
		else if (dev->irq_stat & PV_START_THRESH_INT)
			dev->irq_stat &= (~PV_START_THRESH_INT);
		ret = 0;
		break;
	case AXIPV_STOP_EOF:
		if (int_stat & FRAME_END_INT)
			writel(FRAME_END_INT, axipv_base + REG_INTR_CLR);
		else if (dev->irq_stat & FRAME_END_INT)
			dev->irq_stat &= (~FRAME_END_INT);
		else
			break;
		curr_buff = readl(axipv_base + REG_CUR_FRAME);
		dev->curr = (curr_buff == dev->curr) ? 0 : dev->curr;
		dev->nxt = (curr_buff == dev->nxt) ? 0 : dev->nxt;
		ret = 0;
		break;
	default:
		dev_err(dev->dsi_dev, "%d is invalid event\n", event);
		break;
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return ret;
}
