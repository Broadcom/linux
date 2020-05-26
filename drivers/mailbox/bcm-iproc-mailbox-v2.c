// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 Broadcom.
 */
#include <linux/bcm_iproc_mailbox_v2.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define IPROC_MAIL_BOX_MSG0		0x00
#define IPROC_MAIL_BOX_MSG1		0x04
#define IPROC_MAIL_BOX_CLR		0x08
#define IPROC_MAIL_BOX_MASK		0x0c

/*
 *            CHANNEL MESSAGE STRUCTUE
 *
 * ============================================
 * |              Channel Status              |
 * ============================================
 * |                  Command                 |
 * |-------------------------------------------
 * |          Parameter[0]/pointer high       |
 * |-------------------------------------------
 * |          Parameter[1]/pointer low        |
 * |-------------------------------------------
 * |              Extra_parameter[0]          |
 * |-------------------------------------------
 * |              Extra_parameter[1]          |
 * |===========================================
 *
 */

#define IPROC_MBOX_CH_SIZE		24

#define IPROC_MBOX_CH_STATUS_OFF	0
#define IPROC_MBOX_CH_CMD_OFF		4
#define IPROC_MBOX_CH_PARAM0_OFF	8
#define IPROC_MBOX_CH_PARAM1_OFF	12
#define IPROC_MBOX_CH_EXTRA_PARAM0_OFF	16
#define IPROC_MBOX_CH_EXTRA_PARAM1_OFF	20

enum iproc_mbox_ch_status {
	IPROC_MBOX_CH_FREE = 0,
	IPROC_MBOX_CH_MSG_VALID = 1,
	IPROC_MBOX_CH_MSG_CONSUMED = 2
};

enum iproc_mbox_ch_type {
	IPROC_MBOX_CH_TX,
	IPROC_MBOX_CH_RX,
	IPROC_MBOX_CH_MAX
};

struct iproc_mbox {
	struct device *dev;
	struct regmap *base;
	void __iomem *shared_mem;
	struct mbox_controller controller;
	uint32_t num_chans;
	uint32_t tx_offset;
	uint32_t rx_offset;
	struct iproc_mbox_ch_priv *ch_priv;
};

struct iproc_mbox_ch_priv {
	void __iomem *base;
	enum iproc_mbox_ch_type type;
	bool active;
};

static const struct of_device_id iproc_mbox_of_match[] = {
	{ .compatible = "brcm,iproc-mailbox-v2" },
	{ }
};
MODULE_DEVICE_TABLE(of, iproc_mbox_of_match);

static bool iproc_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct iproc_mbox_ch_priv *ch_priv = chan->con_priv;
	uint32_t data;

	data = readl(ch_priv->base + IPROC_MBOX_CH_STATUS_OFF);

	if (data == IPROC_MBOX_CH_MSG_VALID)
		return false;

	return true;
}

static int iproc_mbox_send_data(struct mbox_chan *chan, void *ptr)
{
	struct iproc_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	struct iproc_mbox_msg *msg = (struct iproc_mbox_msg *)ptr;
	struct iproc_mbox_ch_priv *ch_priv = chan->con_priv;
	uint32_t data;

	if (!msg)
		return -EINVAL;

	/* Check if previous message to get consumed */
	data = iproc_mbox_last_tx_done(chan);
	if (!data)
		return -EBUSY;

	writel(msg->cmd, ch_priv->base + IPROC_MBOX_CH_CMD_OFF);
	writel(msg->param[0], ch_priv->base + IPROC_MBOX_CH_PARAM0_OFF);
	writel(msg->param[1], ch_priv->base + IPROC_MBOX_CH_PARAM1_OFF);
	writel(msg->extra_param[0], (ch_priv->base +
				     IPROC_MBOX_CH_EXTRA_PARAM0_OFF));
	writel(msg->extra_param[1], (ch_priv->base +
				     IPROC_MBOX_CH_EXTRA_PARAM1_OFF));

	writel(IPROC_MBOX_CH_MSG_VALID, (ch_priv->base +
					 IPROC_MBOX_CH_STATUS_OFF));

	/* Generate the mailbox interrupt */
	regmap_write(mbox->base, mbox->tx_offset + IPROC_MAIL_BOX_MSG1, 1);

	return 0;
}

static struct mbox_chan *iproc_mbox_xlate(struct mbox_controller *mbox,
					  const struct of_phandle_args *sp)
{
	int ch_idx, ch_type;
	struct mbox_chan *chan;
	struct iproc_mbox_ch_priv *ch_priv;

	ch_idx = sp->args[0];
	ch_type = sp->args[1];

	if (ch_idx >= mbox->num_chans || ch_type >= IPROC_MBOX_CH_MAX)
		return ERR_PTR(-EINVAL);

	chan = &mbox->chans[ch_idx];

	ch_priv = chan->con_priv;
	ch_priv->type = ch_type;

	return chan;
}

static int iproc_mbox_startup(struct mbox_chan *chan)
{
	struct iproc_mbox_ch_priv *ch_priv = chan->con_priv;
	uint32_t data;

	ch_priv->active = true;
	data = readl(ch_priv->base + IPROC_MBOX_CH_STATUS_OFF);
	if (data != IPROC_MBOX_CH_FREE) {
		pr_warn("%s: force aquiring the channel\n", __func__);
		writel(IPROC_MBOX_CH_FREE, (ch_priv->base +
					    IPROC_MBOX_CH_STATUS_OFF));
	}

	return 0;
}

static void iproc_mbox_shutdown(struct mbox_chan *chan)
{
	struct iproc_mbox_ch_priv *ch_priv = chan->con_priv;
	uint32_t data;

	data = readl(ch_priv->base + IPROC_MBOX_CH_STATUS_OFF);
	if (data != IPROC_MBOX_CH_FREE) {
		pr_warn("%s: force releasing the channel\n", __func__);
		writel(IPROC_MBOX_CH_FREE, (ch_priv->base +
					    IPROC_MBOX_CH_STATUS_OFF));
	}
	ch_priv->active = false;
}

static const struct mbox_chan_ops iproc_mbox_ops = {
	.send_data    = iproc_mbox_send_data,
	.last_tx_done = iproc_mbox_last_tx_done,
	.startup      = iproc_mbox_startup,
	.shutdown     = iproc_mbox_shutdown,
};

static irqreturn_t iproc_mbox_isr(int irq, void *dev_id)
{
	struct iproc_mbox *mbox = (struct iproc_mbox *)dev_id;

	regmap_write(mbox->base, mbox->rx_offset + IPROC_MAIL_BOX_CLR, 1);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t iproc_mbox_thread(int irq, void *dev_id)
{
	struct iproc_mbox *mbox = (struct iproc_mbox *)dev_id;
	struct iproc_mbox_ch_priv *ch_priv;
	struct iproc_mbox_msg msg;
	void __iomem *chan_base;
	uint32_t data;
	int i;

	for (i = 0; i < mbox->num_chans; i++) {
		ch_priv = &mbox->ch_priv[i];
		if (!(ch_priv->active) || ch_priv->type == IPROC_MBOX_CH_TX)
			continue;

		chan_base = ch_priv->base;
		data = readl(chan_base + IPROC_MBOX_CH_STATUS_OFF);

		if (data != IPROC_MBOX_CH_MSG_VALID)
			continue;

		msg.cmd = readl(chan_base + IPROC_MBOX_CH_CMD_OFF);
		msg.param[0] = readl(chan_base + IPROC_MBOX_CH_PARAM0_OFF);
		msg.param[1] = readl(chan_base + IPROC_MBOX_CH_PARAM1_OFF);
		msg.extra_param[0] = readl(chan_base +
					   IPROC_MBOX_CH_EXTRA_PARAM0_OFF);
		msg.extra_param[1] = readl(chan_base +
					   IPROC_MBOX_CH_EXTRA_PARAM1_OFF);
		/* Send data to client */
		mbox_chan_received_data(&mbox->controller.chans[i], &msg);

		writel(IPROC_MBOX_CH_MSG_CONSUMED, (chan_base +
						    IPROC_MBOX_CH_STATUS_OFF));
	}

	return IRQ_HANDLED;
}

static int iproc_mbox_map_memory(struct platform_device *pdev,
				 struct iproc_mbox *mbox,
				 struct mbox_chan *chans)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	int i, ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mbox->shared_mem = devm_ioremap_resource(dev, res);
	if (IS_ERR(mbox->shared_mem)) {
		dev_err(&pdev->dev, "unable to map I/O memory\n");
		return PTR_ERR(mbox->shared_mem);
	}

	for (i = 0; i < mbox->num_chans; i++) {
		mbox->ch_priv[i].base = mbox->shared_mem +
					 (i * IPROC_MBOX_CH_SIZE);
		chans[i].con_priv = &mbox->ch_priv[i];
	}

	return ret;
}

static int iproc_mbox_probe(struct platform_device *pdev)
{
	int irq, ret;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct iproc_mbox *iproc_mbox;
	struct mbox_chan *chans;

	iproc_mbox = devm_kzalloc(dev, sizeof(*iproc_mbox), GFP_KERNEL);
	if (!iproc_mbox)
		return -ENOMEM;

	iproc_mbox->dev = dev;

	platform_set_drvdata(pdev, iproc_mbox);

	if (of_property_read_u32(np, "brcm,iproc-mbox-ch-count",
				 &iproc_mbox->num_chans)) {
		dev_err(dev, "couldn't read mbox channel count\n");
		return -EINVAL;
	}

	chans = devm_kzalloc(&pdev->dev,
			     sizeof(*chans) * iproc_mbox->num_chans,
			     GFP_KERNEL);
	if (!chans)
		return -ENOMEM;

	iproc_mbox->ch_priv = devm_kzalloc(&pdev->dev,
					   (sizeof(struct iproc_mbox_ch_priv) *
					    iproc_mbox->num_chans),
					   GFP_KERNEL);
	if (!iproc_mbox->ch_priv)
		return -ENOMEM;

	iproc_mbox->base =
		syscon_regmap_lookup_by_phandle(np, "brcm,iproc-mbox");
	if (IS_ERR(iproc_mbox->base)) {
		dev_err(&pdev->dev, "unable to find mbox syscon node\n");
		ret = PTR_ERR(iproc_mbox->base);
		goto err;
	}

	if (of_property_read_u32_index(np, "brcm,iproc-mbox",
				       1, &iproc_mbox->rx_offset)) {
		dev_err(dev, "couldn't read the syscon rx register offset\n");
		return -EINVAL;
	}

	if (of_property_read_u32_index(np, "brcm,iproc-mbox",
				       2, &iproc_mbox->tx_offset)) {
		dev_err(dev, "couldn't read the syscon tx register offset\n");
		return -EINVAL;
	}

	ret = iproc_mbox_map_memory(pdev, iproc_mbox, chans);
	if (ret) {
		dev_err(&pdev->dev, "Failed to mmap channel memory\n");
		goto err;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get platform irq\n");
		ret = irq;
		goto err;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq,
					iproc_mbox_isr,
					iproc_mbox_thread,
					IRQF_TRIGGER_HIGH | IRQF_SHARED,
					"iproc_mbox", iproc_mbox);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request for irq\n");
		goto err;
	}

	/* Initialize mailbox controller. */
	iproc_mbox->controller.dev = iproc_mbox->dev;
	iproc_mbox->controller.num_chans = iproc_mbox->num_chans;
	iproc_mbox->controller.chans = chans;
	iproc_mbox->controller.ops = &iproc_mbox_ops;
	iproc_mbox->controller.of_xlate = iproc_mbox_xlate;
	iproc_mbox->controller.txdone_irq = false;
	iproc_mbox->controller.txdone_poll = true;

	ret = mbox_controller_register(&iproc_mbox->controller);
	if (ret) {
		dev_err(&pdev->dev, "Register mailbox failed\n");
		goto err;
	}

	dev_info(&pdev->dev, "iProc mailbox initailized successfully\n");

err:
	return ret;
}

static struct platform_driver iproc_mbox_driver = {
	.driver = {
		.name = "brcm,iproc-mailbox-v2",
		.of_match_table = iproc_mbox_of_match,
	},
	.probe = iproc_mbox_probe,
};

static int __init iproc_mbox_init(void)
{
	return platform_driver_register(&iproc_mbox_driver);
}
arch_initcall(iproc_mbox_init);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom iProc Mailbox Driver V2");
MODULE_LICENSE("GPL v2");
