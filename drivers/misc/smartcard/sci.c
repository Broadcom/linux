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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "bscd_datatypes.h"
#include "bscd_priv.h"
#include "sci.h"
#include "sci_regs.h"

struct class *brcm_sci_class;
char const *irq_name[MAX_SCI_INTR] = {"sci_irq1", "sci_irq2", "sci_irq3" };

int create_event(struct bscd_event_obj **p_event)
{
	struct bscd_event_obj *event = kzalloc(sizeof(*event), GFP_KERNEL);

	if (!event)
		return -ENOMEM;
	event->eventset = 0;

	init_waitqueue_head(&event->wq);
	*p_event = event;
	return BERR_SUCCESS;
}


int wait_for_event(struct bscd_event_obj *event, int timeout_msec)
{
	int ret;

	if (timeout_msec > 0 && timeout_msec < 30) {
		/* wait at least 30 msec */
		timeout_msec = 30;
	}

	ret = wait_event_interruptible_timeout(event->wq, event->eventset == 1,
						(timeout_msec * HZ) / 1000);
	if (ret == 0) {
		pr_err("SCI: timeout for requested time:%d\n", timeout_msec);
		return BERR_TIMEOUT;
	}

	if (event->eventset == 1)
		event->eventset = 0;

	return BERR_SUCCESS;
}
void set_event(struct bscd_event_obj *event)
{
	event->eventset = 1;
	wake_up_interruptible(&event->wq);
}
static void sci_set_sci_irq(struct sci_ctx *pctx, int en)
{
	int val = 0x0;

	dev_dbg(pctx->dev, "%s SCI IRQ\n", en ? "Enable" : "Disable");

	regmap_read(pctx->scirq_regmap, 0x28/* pctx->irqen_off */, &val);
	if (en)
		val = val | (1 << pctx->sci_id);
	else
		val = val & (~(1 << pctx->sci_id));
	regmap_write(pctx->scirq_regmap, pctx->irqen_off, val);
}
static irqreturn_t sci_isr_thread(int irq, void *drv_ctx)
{
	struct sci_ctx  *pctx = drv_ctx;

	dev_info(pctx->dev, "SCI thread  IRQ %d occurred\n", irq);

	if (pctx->mod_hdl->chnl_hdls[pctx->sci_id] == NULL) {
		dev_err(pctx->dev, "channel(%d) is not opened\n", pctx->sci_id);
		return IRQ_HANDLED;
	}
	chnl_p_intr_handler_bh(pctx->mod_hdl->chnl_hdls[pctx->sci_id],
								pctx->sci_id);

	return IRQ_HANDLED;
}
static irqreturn_t sci_isr(int irq, void *drv_ctx)
{
	struct sci_ctx  *pctx = drv_ctx;
	int    bh;

	dev_dbg(pctx->dev, "SCI IRQ %d occurred for channel %d\n",
							irq, pctx->sci_id);

	if (pctx->mod_hdl->chnl_hdls[pctx->sci_id] == NULL) {
		dev_err(pctx->dev, "channel(%d) is not opened\n", pctx->sci_id);
		return IRQ_HANDLED;
	}

	bh = chnl_p_intr_handler_isr(pctx->mod_hdl->chnl_hdls[pctx->sci_id],
								pctx->sci_id);
	if (bh)
		return IRQ_WAKE_THREAD;

	return IRQ_HANDLED;
}
static int get_devres(struct sci_ctx  *pctx)
{
	int ret, val;
	struct resource res;
	struct device *dev = pctx->dev;

	pctx->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(pctx->clk)) {
		dev_err(dev, "Failed to get sci clock\n");
		return -ENODEV;
	}
	ret = clk_prepare_enable(pctx->clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "sci-id", &val);
	if (ret) {
		dev_err(dev, "Failed to receive Smart card id\n");
		return -ENODEV;
	}
	pctx->sci_id = val;
	dev_info(pctx->dev, "Smartcard%d found\n", pctx->sci_id);

	ret = of_property_read_u32(dev->of_node, "nxp-coupler", &val);
	if (ret) {
		dev_err(dev, "Failed to receive coupler\n");
		return -ENODEV;
	}

	pctx->coupler = val;
	if (pctx->coupler == 0)
		dev_info(pctx->dev, "Using coupler NXP8024\n");
	else
		dev_info(pctx->dev, "Using coupler NXP8026\n");

	return 0;
}
#ifdef CONFIG_PM
static int sci_suspend(struct device *dev)
{
	struct sci_ctx *pctx = dev_get_drvdata(dev);

	dev_dbg(pctx->dev, "Enter SCI suspend\n");
	if (pctx->busy) {
		dev_err(pctx->dev, "driver busy\n");
		return -EAGAIN;
	}
	sci_set_sci_irq(pctx, 0);   /* disable SCI irqs */

	return 0;
}

static int sci_resume(struct device *dev)
{
	struct sci_ctx *pctx = dev_get_drvdata(dev);

	dev_dbg(pctx->dev, "Enter SCI resume\n");
	if (pctx->busy) {
		dev_err(pctx->dev, "driver busy\n");
		return -EAGAIN;
	}
	sci_set_sci_irq(pctx, 1);

	return 0;
}
static SIMPLE_DEV_PM_OPS(sci_pm, sci_suspend, sci_resume);
#endif
static int sci_remove(struct platform_device *pdev)
{
	struct sci_ctx *pctx = platform_get_drvdata(pdev);

	rm_fs_intfs(pctx);
	sci_set_sci_irq(pctx, 0);

	return 0;
}
static int sci_probe(struct platform_device *pdev)
{
	int ret;
	struct sci_ctx  *pctx;
	struct device *dev = &pdev->dev;
	struct resource *res;

	pctx = devm_kzalloc(dev, sizeof(*pctx), GFP_KERNEL);
	if (pctx == NULL)
		return -ENOMEM;

	pctx->sci_class = brcm_sci_class;

	mutex_init(&pctx->sc_mutex);
	pctx->dev = &pdev->dev;
	platform_set_drvdata(pdev, pctx);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "sc-uart-cmd1");
	pctx->sci_regs = devm_ioremap_resource(pctx->dev, res);
	if (IS_ERR(pctx->sci_regs))
		return PTR_ERR(pctx->sci_regs);

	pctx->scirq_regmap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							"sc_syscon");
	if (IS_ERR(pctx->scirq_regmap))
		return pctx->scirq_regmap;

	pctx->irq = platform_get_irq_byname(pdev, "sci-irq");
	if (pctx->irq < 0) {
		dev_err(dev, "Failed to parse interrupt\n");
		ret = -ENODEV;
	}
	ret = devm_request_threaded_irq(dev, pctx->irq, sci_isr,
			sci_isr_thread, IRQF_SHARED | IRQF_ONESHOT,
			irq_name[pctx->sci_id], pctx);
	if (ret) {
		dev_err(dev, "Failed to register interrupt\n");
		return ret;
	}
	pctx->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(pctx->clk)) {
		dev_err(dev, "Failed to get sci clock\n");
		return -ENODEV;
	}
	ret = clk_prepare_enable(pctx->clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		return ret;
	}

	ret = get_devres(pctx);
	if (ret)
		return ret;

	ret = init_fs_intfs(pctx);
	if (ret)
		return -ENODEV;

	if (pctx->sci_id == 0)
		pctx->irqen_off = SCI_SCIRQ0_SCIRQEN;
	else if (pctx->sci_id == 1)
		pctx->irqen_off = SCI_SCIRQ1_SCIRQEN;
	else
		pctx->irqen_off = SCI_SCIRQ_SCPU_SCIRQEN;

	/* Enable irqs */
	sci_set_sci_irq(pctx, 1);

	/*
	 * Initialize the smart card stack
	 * The SCI block power should have been enabled
	 * (DMU_SCI_PWR_ENABLE | DMU_SCICLK_PWR_ENABLE)
	 */
	ret = open(&pctx->mod_hdl, NULL,
		(pctx->coupler == 0) ? COUPLER_NXP8024 : COUPLER_NXP8026);
	if (ret) {
		dev_err(pctx->dev, "open failed\n");
		ret = -ENODEV;
		goto out;
	}
	dev_info(pctx->dev, "%s completed successfully\n", __func__);
	return 0;

out:
	rm_fs_intfs(pctx);
	return ret;
}
static const struct of_device_id sci_of_match[] = {
	{.compatible = "brcm,smart-card", },
	{ },
};
MODULE_DEVICE_TABLE(of, sci_of_match);

static struct platform_driver sci_driver = {
	.driver  = {
		.name  = "Smartcard",
		.of_match_table = sci_of_match,
#ifdef CONFIG_PM
		.pm    = &sci_pm,
#endif
	},
	.probe   = sci_probe,
	.remove  = sci_remove,
};

static int __init sci_init(void)
{
	int ret;

	brcm_sci_class = class_create(THIS_MODULE, "smartcard");
	if (IS_ERR(brcm_sci_class)) {
		pr_err("SCI:Failed to create class\n");
		return PTR_ERR(brcm_sci_class);
	}
	ret = platform_driver_register(&sci_driver);
	if (ret) {
		pr_err("SCI: Failed to register sci driver\n");
		goto out;
	}
	return ret;
out:
	class_destroy(brcm_sci_class);
	return ret;
}

static void __exit sci_exit(void)
{
	platform_driver_unregister(&sci_driver);
	class_destroy(brcm_sci_class);
}
module_init(sci_init);
module_exit(sci_exit);

MODULE_DESCRIPTION("Broadcom SmartCard driver support for Cygnus");
MODULE_AUTHOR("pramodku@broadcom.com");
MODULE_LICENSE("GPL v2");
