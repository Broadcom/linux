/*
 * Copyright (C) 2017 Broadcom.
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
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/mailbox_client.h>
#include <linux/bcm_iproc_mailbox.h>

#define CRMU_IPROC_MAILBOX0_OFFSET       0x0
#define CRMU_IPROC_MAILBOX1_OFFSET       0x4

#define IPROC_INTR_STATUS                0x0
#define IPROC_MAILBOX_INTR_SHIFT         0
#define IPROC_MAILBOX_INTR_MASK          0x1

#define IPROC_INTR_CLEAR                 0x8
#define IPROC_MAILBOX_INTR_CLR_SHIFT     0

/* Domains that interrupts get forwarded to. */
enum mbox_domain {
	AON_GPIO_DOMAIN = 0,
};

enum iproc_m0_cmd {
	/*
	 * Enable/disable GPIO event forwarding from M0 to A9
	 * Param - 1 to enable, 0 to disable
	 * Response - return code
	 */
	M0_IPC_M0_CMD_AON_GPIO_FORWARDING_ENABLE = 0xe,

	/*
	 * AON GPIO interrupt ("forwarded" to IPROC)
	 * Param - AON GPIO mask
	 */
	M0_IPC_HOST_CMD_AON_GPIO_EVENT = 0x102,
};

struct iproc_crmu_intc_data {
	struct device      *dev;
	void __iomem       *mbox_base;
	void __iomem       *intr_base;
	struct irq_domain  *domain;
	int                mbox_irq;
	struct mbox_client client;
	struct mbox_chan   *mbox_chan;
};

static void iproc_crmu_intc_aon_gpio_forwarding_enable(
	struct iproc_crmu_intc_data *data, bool en)
{
	int err;
	struct iproc_mbox_msg msg;

	msg.cmd = M0_IPC_M0_CMD_AON_GPIO_FORWARDING_ENABLE;
	msg.param = en ? 1 : 0;
	msg.wait_ack = true;

	err = mbox_send_message(data->mbox_chan, &msg);
	if (err < 0)
		dev_err(data->dev,
			"mbox_send_message failed: %d\n", err);
	else if (msg.reply_code)
		dev_err(data->dev,
			"M0 command failed: 0x%x\n", msg.reply_code);
	mbox_client_txdone(data->mbox_chan, 0);
}

static void iproc_crmu_intc_irq_unmask(struct irq_data *d)
{
	struct iproc_crmu_intc_data *data = irq_data_get_irq_chip_data(d);

	iproc_crmu_intc_aon_gpio_forwarding_enable(data, true);
}

static void iproc_crmu_intc_irq_mask(struct irq_data *d)
{
	/* Do nothing - Mask callback is not required, since upon GPIO event,
	 * M0 disables GPIO forwarding to A9. Hence, GPIO forwarding is already
	 * disabled when in mbox irq handler, and no other mbox events from M0
	 * to A9 are expected until GPIO forwarding is enabled following
	 * iproc_crmu_intc_irq_unmask()
	 */
}

static struct irq_chip iproc_crmu_intc_irq_chip = {
	.name = "bcm-iproc-crmu-intc",
	.irq_mask = iproc_crmu_intc_irq_mask,
	.irq_unmask = iproc_crmu_intc_irq_unmask,
};

static int iproc_crmu_intc_irq_map(struct irq_domain *d, unsigned int irq,
	irq_hw_number_t hwirq)
{
	int ret;

	ret = irq_set_chip_data(irq, d->host_data);
	if (ret < 0)
		return ret;
	irq_set_chip_and_handler(irq, &iproc_crmu_intc_irq_chip,
		handle_simple_irq);

	return 0;
}

static void iproc_crmu_intc_irq_unmap(struct irq_domain *d, unsigned int irq)
{
	irq_set_chip_and_handler(irq, NULL, NULL);
	irq_set_chip_data(irq, NULL);
}

static const struct irq_domain_ops iproc_crmu_intc_irq_ops = {
	.map = iproc_crmu_intc_irq_map,
	.unmap = iproc_crmu_intc_irq_unmap,
	.xlate = irq_domain_xlate_onecell,
};

static void iproc_crmu_intc_irq_handler(struct irq_desc *desc)
{
	struct iproc_crmu_intc_data *data = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long status;
	u32 cmd, param;
	int virq;

	chained_irq_enter(chip, desc);

	/* Determine type of interrupt. */
	status = readl(data->intr_base + IPROC_INTR_STATUS);
	status = (status >> IPROC_MAILBOX_INTR_SHIFT) &
		IPROC_MAILBOX_INTR_MASK;

	/* Process mailbox interrupts. */
	if (status) {
		writel(BIT(IPROC_MAILBOX_INTR_CLR_SHIFT),
			data->intr_base + IPROC_INTR_CLEAR);

		cmd = readl(data->mbox_base + CRMU_IPROC_MAILBOX0_OFFSET);
		param = readl(data->mbox_base + CRMU_IPROC_MAILBOX1_OFFSET);

		dev_dbg(data->dev,
			"Received message from M0: cmd 0x%x param 0x%x\n",
			cmd, param);

		/* Process AON GPIO interrupt - forward to GPIO handler. */
		if (cmd == M0_IPC_HOST_CMD_AON_GPIO_EVENT) {
			virq = irq_find_mapping(data->domain,
				AON_GPIO_DOMAIN);
			generic_handle_irq(virq);
		}
	}

	chained_irq_exit(chip, desc);
}

static int iproc_crmu_intc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn = dev->of_node;
	int virq;
	struct resource *res;
	struct iproc_crmu_intc_data *data;
	int err = 0;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	platform_set_drvdata(pdev, data);

	/* Request mailbox channel. */
	data->client.dev          = dev;
	data->client.tx_block     = false;
	data->client.tx_tout      = 2;
	data->client.knows_txdone = true;
	data->mbox_chan = mbox_request_channel(&data->client, 0);
	if (IS_ERR(data->mbox_chan)) {
		dev_err(dev, "unable to get mbox channel\n");
		return PTR_ERR(data->mbox_chan);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->mbox_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->mbox_base)) {
		dev_err(dev, "failed to map base register\n");
		err = -ENOMEM;
		goto free_mbox_chan;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	data->intr_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->intr_base)) {
		dev_err(&pdev->dev, "unable to map I/O memory\n");
		err = PTR_ERR(data->intr_base);
		goto free_mbox_chan;
	}

	data->mbox_irq = irq_of_parse_and_map(dn, 0);
	if (!data->mbox_irq) {
		dev_err(dev, "irq_of_parse_and_map failed\n");
		err = -ENODEV;
		goto free_mbox_chan;
	}

	data->domain = irq_domain_add_linear(dn, 1,
		&iproc_crmu_intc_irq_ops, data);
	if (!data->domain) {
		dev_err(dev, "unable to allocate IRQ domain\n");
		err = -ENXIO;
		goto dispose_mapping;
	}

	/* Map irq for AON GPIO interrupt handling into this domain. */
	virq = irq_create_mapping(data->domain, AON_GPIO_DOMAIN);
	if (!virq) {
		dev_err(dev, "failed mapping irq into domain\n");
		err = -ENXIO;
		goto domain_remove;
	}
	dev_dbg(dev, "irq for aon gpio domain: %d\n", virq);

	irq_set_chained_handler_and_data(data->mbox_irq,
		iproc_crmu_intc_irq_handler, data);

	return 0;

domain_remove:
	irq_domain_remove(data->domain);

dispose_mapping:
	irq_dispose_mapping(data->mbox_irq);

free_mbox_chan:
	mbox_free_channel(data->mbox_chan);
	return err;
}

#ifdef CONFIG_PM_SLEEP
static int iproc_crmu_intc_suspend(struct device *dev)
{
	struct iproc_crmu_intc_data *data = dev_get_drvdata(dev);

	dev_info(dev, "Suspending iproc crmu intc: disabling GPIO forwarding\n");
	iproc_crmu_intc_aon_gpio_forwarding_enable(data, false);
	synchronize_irq(data->mbox_irq);

	return 0;
}

static int iproc_crmu_intc_resume(struct device *dev)
{
	struct iproc_crmu_intc_data *data = dev_get_drvdata(dev);

	dev_info(dev, "Resuming iproc crmu intc: enabling AON GPIO forwarding\n");
	iproc_crmu_intc_aon_gpio_forwarding_enable(data, true);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(iproc_crmu_intc_pm_ops, iproc_crmu_intc_suspend,
	iproc_crmu_intc_resume);

static const struct of_device_id iproc_crmu_intc_of_match[] = {
	{ .compatible = "brcm,iproc-crmu-intc", },
	{},
};
MODULE_DEVICE_TABLE(of, iproc_crmu_intc_dt_id);

static struct platform_driver iproc_crmu_intc_driver = {
	.driver = {
		.name = "brcm_iproc_crmu_intc",
		.of_match_table = iproc_crmu_intc_of_match,
		.pm = &iproc_crmu_intc_pm_ops,
	},
	.probe = iproc_crmu_intc_probe,
};

static int __init iproc_crmu_intc_init(void)
{
	return platform_driver_register(&iproc_crmu_intc_driver);
}
arch_initcall_sync(iproc_crmu_intc_init);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom iProc CRMU Interrupt Controller Driver");
MODULE_LICENSE("GPL v2");
