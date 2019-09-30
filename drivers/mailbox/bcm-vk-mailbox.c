// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Broadcom.
 */
#include <asm-generic/barrier.h>
#include <linux/bcm_vk_mailbox.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* Valkyrie has 2 pairs of mailboxes */
#define VK_MAILBOX_MAX_CHANS	2

/* Max time M7 firmware can take to respond */
#define MAX_M7_TIMEOUT_MS	10

/* mailbox register offsets */
#define MAILBOX_0_OFFSET	0x024
#define MAILBOX_1_OFFSET	0xce4
#define CORE_CRMU_MAIL_BOX_L	0x00
#define CORE_CRMU_MAIL_BOX_H	0x04
#define CRMU_CORE_MAIL_BOX_L	0x08
#define CRMU_CORE_MAIL_BOX_H	0x0c

/* interrupt register offsets */
#define CRMU_CORE_INTR_STATUS	0x6c
#define CRMU_CORE_INTR_MASK	0x70
#define CRMU_CORE_INTR_CLEAR	0x74

/* Interrupt status/mask/clear bits */
#define MAILBOX0_INTR		BIT(0)
#define MAILBOX1_INTR		BIT(5)

/* command receive 'ack' bit */
#define CMD_RECVD_ACK		BIT(31)

struct vk_mbox_mb_info {
	int chanid;
	int irq;
	void __iomem *mbox_base;
};

/* mailbox driver data */
struct vk_mbox {
	struct device *dev;
	struct mbox_controller controller;
	spinlock_t lock;
	void __iomem *base;
	int num_chans;
	struct vk_mbox_mb_info mb_info[VK_MAILBOX_MAX_CHANS];
};

static const char *const intr_names[VK_MAILBOX_MAX_CHANS] = {
	"vk_mailbox0",
	"vk_mailbox1"
};

static const uint32_t reg_offsets[VK_MAILBOX_MAX_CHANS] = {
	MAILBOX_0_OFFSET,
	MAILBOX_1_OFFSET
};

static const struct of_device_id vk_mbox_of_match[] = {
	{.compatible = "brcm,vk-mailbox"},
	{ }
};
MODULE_DEVICE_TABLE(of, vk_mbox_of_match);

static int vk_mbox_send_data_to_crmu(struct mbox_chan *chan, void *data)
{
	struct vk_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	struct vk_mbox_msg *msg = (struct vk_mbox_msg *)data;
	struct vk_mbox_mb_info *mb_info = chan->con_priv;
	const int poll_period_us = 5;
	const int max_retries = (MAX_M7_TIMEOUT_MS * 1000) / poll_period_us;
	unsigned long flags;
	int err = -ETIMEDOUT, retries;
	uint32_t val;

	if (!msg)
		return -EINVAL;

	spin_lock_irqsave(&mbox->lock, flags);
	dev_dbg(mbox->dev, "send msg to crmu: %x, %x\n", msg->cmd, msg->arg);
	writel(msg->cmd, mb_info->mbox_base + CORE_CRMU_MAIL_BOX_L);
	writel(msg->arg, mb_info->mbox_base + CORE_CRMU_MAIL_BOX_H);

	/* wait for command received 'ACK' from M7 */
	for (retries = 0; retries < max_retries; retries++) {
		val = readl(mbox->base + CORE_CRMU_MAIL_BOX_L);
		if (val & CMD_RECVD_ACK) {
			err = 0;
			break;
		}
		udelay(poll_period_us);
	}
	spin_unlock_irqrestore(&mbox->lock, flags);

	return 0;
}

static struct mbox_chan_ops vk_mbox_ops = {
	.send_data    = vk_mbox_send_data_to_crmu,
};

static irqreturn_t vk_mailbox_isr(int n, void *arg)
{
	uint32_t mask, stat, chanid, cmd_ack;
	struct vk_mbox *mbox = (struct vk_mbox *)arg;
	struct vk_mbox_msg msg;
	struct vk_mbox_mb_info *mb_info;
	struct mbox_chan *chan;
	unsigned long flags;

	spin_lock_irqsave(&mbox->lock, flags);
	stat = readl(mbox->base + CRMU_CORE_INTR_STATUS);
	if (stat & MAILBOX0_INTR) {
		chanid = 0;
		/* clear interrupt */
		mask = MAILBOX0_INTR;
		writel(mask, mbox->base + CRMU_CORE_INTR_CLEAR);
	} else if (stat & MAILBOX1_INTR) {
		chanid = 1;
		/* clear interrupt */
		mask = MAILBOX1_INTR;
		writel(mask, mbox->base + CRMU_CORE_INTR_CLEAR);
	} else {
		/* should not reach here! */
		dev_err(mbox->dev, "spurious mailbox isr %d!\n", n);
		return IRQ_NONE;
	}

	mb_info = &mbox->mb_info[chanid];
	/* read mailbox message */
	msg.cmd = readl(mb_info->mbox_base + CRMU_CORE_MAIL_BOX_L);
	msg.arg = readl(mb_info->mbox_base + CRMU_CORE_MAIL_BOX_H);
	dev_dbg(mbox->dev, "mailbox %d cmd:0x%08x,arg:0x%08x\n",
		mb_info->chanid, msg.cmd, msg.arg);

	chan = &mbox->controller.chans[chanid];
	/* push received message to registered client */
	if (chan->cl)
		mbox_chan_received_data(chan, &msg);

	/* send command received 'ACK' */
	cmd_ack = msg.cmd | CMD_RECVD_ACK;
	writel(cmd_ack, mb_info->mbox_base + CRMU_CORE_MAIL_BOX_L);
	spin_unlock_irqrestore(&mbox->lock, flags);

	return IRQ_HANDLED;
}

static void mailbox_intfc_init(void __iomem *base)
{
	uint32_t reg;

	/* enable/unmask mailbox interrupts */
	reg = readl(base + CRMU_CORE_INTR_MASK);
	reg &= ~(MAILBOX0_INTR | MAILBOX0_INTR);
	writel(reg, base + CRMU_CORE_INTR_MASK);

	/* clear any set interrupts */
	reg = MAILBOX0_INTR | MAILBOX0_INTR;
	writel(reg, base + CRMU_CORE_INTR_CLEAR);

	dsb(sy);
}

static struct mbox_chan *vk_mbox_of_xlate(struct mbox_controller *mbox_ctrl,
					  const struct of_phandle_args *args)
{
	struct vk_mbox *mbox = dev_get_drvdata(mbox_ctrl->dev);
	unsigned int chanid = args->args[0];
	struct mbox_chan *chan;
	unsigned long flags;
	struct vk_mbox_mb_info *mb_priv;

	if (chanid >= VK_MAILBOX_MAX_CHANS) {
		dev_err(mbox->dev, "chan id %d exceeding max\n", chanid);
		return ERR_PTR(-EINVAL);
	}

	spin_lock_irqsave(&mbox->lock, flags);
	chan = &mbox_ctrl->chans[chanid];
	mb_priv = &mbox->mb_info[chanid];

	if (!chan->con_priv) {
		chan->con_priv = mb_priv;
	} else {
		/* channel already taken */
		chan = ERR_PTR(-EBUSY);
	}
	spin_unlock_irqrestore(&mbox->lock, flags);

	return chan;
}

static int vk_mbox_probe(struct platform_device *pdev)
{
	int err, irq, i;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct vk_mbox *vk_mbox;
	struct mbox_chan *chans;
	struct vk_mbox_mb_info *mb_info;

	vk_mbox = devm_kzalloc(dev, sizeof(*vk_mbox), GFP_KERNEL);
	if (!vk_mbox)
		return -ENOMEM;

	vk_mbox->dev = dev;
	vk_mbox->num_chans = VK_MAILBOX_MAX_CHANS;
	spin_lock_init(&vk_mbox->lock);
	platform_set_drvdata(pdev, vk_mbox);

	chans = devm_kzalloc(dev, sizeof(*chans) * vk_mbox->num_chans,
			     GFP_KERNEL);
	if (!chans) {
		err = -ENOMEM;
		goto err_ret1;
	}

	/* Initialize mailbox controller. */
	vk_mbox->controller.dev = vk_mbox->dev;
	vk_mbox->controller.num_chans = vk_mbox->num_chans;
	vk_mbox->controller.chans = chans;
	vk_mbox->controller.ops = &vk_mbox_ops;
	vk_mbox->controller.of_xlate = vk_mbox_of_xlate;
	vk_mbox->controller.txdone_irq = false;
	vk_mbox->controller.txdone_poll = false;
	err = devm_mbox_controller_register(dev, &vk_mbox->controller);
	if (err) {
		dev_err(dev, "Register mailbox failed\n");
		goto err_ret2;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vk_mbox->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(vk_mbox->base)) {
		dev_err(dev, "unable to map I/O memory\n");
		err = PTR_ERR(vk_mbox->base);
		goto err_ret2;
	}

	/* clear and intiailize mailbox interafce */
	mailbox_intfc_init(vk_mbox->base);

	/* get resources for each channel */
	for (i = 0; i < vk_mbox->num_chans; i++) {
		mb_info = &vk_mbox->mb_info[i];
		irq = platform_get_irq_byname(pdev, intr_names[i]);
		if (irq < 0) {
			dev_err(dev, "Failed to parse interrupt chan%d\n", i);
			err = -ENODEV;
			goto err_ret2;
		}
		err = devm_request_irq(dev, irq, vk_mailbox_isr,
					0, intr_names[i], vk_mbox);
		if (err < 0) {
			dev_err(dev, "Failed to register interrupt %d\n", irq);
			err = -EINVAL;
			goto err_ret2;
		}
		mb_info->chanid = i;
		mb_info->irq = irq;
		/* Each mailbox starts at different register offsets */
		mb_info->mbox_base = vk_mbox->base + reg_offsets[i];
	}

	dev_info(dev, "Mailbox driver init, total %d channels\n",
		 vk_mbox->num_chans);

	return 0;

err_ret2:
	devm_kfree(dev, chans);
err_ret1:
	devm_kfree(dev, vk_mbox);

	return err;
}

static struct platform_driver vk_mbox_driver = {
	.driver = {
		.name = "brcm,vk-mailbox",
		.of_match_table = vk_mbox_of_match,
	},
	.probe = vk_mbox_probe,
};

static int __init vk_mbox_init(void)
{
	return platform_driver_register(&vk_mbox_driver);
}
core_initcall(vk_mbox_init);

static void __exit vk_mbox_exit(void)
{
	platform_driver_unregister(&vk_mbox_driver);
}
module_exit(vk_mbox_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Valkyrie Mailbox Driver");
MODULE_LICENSE("GPL v2");
