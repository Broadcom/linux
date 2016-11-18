/*
 * Copyright 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 (GPLv2) for more details.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>

#define SATA_PCB_BANK_OFFSET		0x23c
#define SATA_PCB_REG_OFFSET(ofs)	((ofs) * 4)
#define RX_DFE_PASS_VAL 0x8000

enum sata_phy_regs {
	BLOCK0_REG_BANK			= 0x0000,
	BLOCK0_XGXSCONTROL		= 0x80,
	BLOCK0_XGXSSTATUS		= 0x81,
	BLOCK0_XGXSSTATUS_PLL_LOCK	= BIT(12),
	BLOCK0_SPARE			= 0x8d,
	BLOCK0_SPEEDCTRL		= 0x8e,
	/*
	 * For Gen 1 - 0x400
	 * For Gen 2 - 0x1400
	 * For Gen 3 - 0x2400
	 */
	SAPIS_DATA_RATE_OVERRIDE_VAL	= 0x1400,

	/*
	 * For Gen 1 - 0x10
	 * For Gen 2 - 0x11
	 * For Gen 3 - 0x12
	 */
	TX0_DATA_RATE_VAL		= 0x0011,
	/*
	 * 0x600 - 2   x REFCLK = 200 MHz
	 * 0x601 - 1/2 x REFCLK = 50  MHz - Passed for Gen 1
	 * 0x602 - 1   x REFCLK = 100 MHz - Passed for Gen 2
	 * 0x603 - 1   x REFCLK = 100 MHz
	 */
	OOB_REF_CLK_SEL			= 0x602,

	BLOCK1_REG_BANK			= 0x0010,
	BLOCK1_TX_TEST			= 0x83,
	BLOCK1_PRBSCONTROL		= 0x88,

	PLL0_REG_BANK			= 0x0050,
	PLL0_ACTRL6			= 0x86,

	PLL1_REG_BANK			= 0x0060,
	PLL1_ACTRL2			= 0x82,
	PLL1_ACTRL3			= 0x83,
	PLL1_ACTRL4			= 0x84,

	TX_REG_BANK			= 0x0070,
	TX_GEN_CTRL1			= 0x80,

	RX_REG_BANK			= 0x00b0,
	RX_STATUS			= 0x80,
	RX_GEN_CTRL1			= 0x81,
	RX_GEN_CTRL2			= 0x89,
};

/* SR PHY PLL1 registers values for shortening PLL Lock period */
#define SR_PLL1_ACTRL2_MAGIC		0x32
#define SR_PLL1_ACTRL3_MAGIC		0x2
#define SR_PLL1_ACTRL4_MAGIC		0x3e8
#define SR_PLL0_ACTRL6_MAGIC		0xa


struct sata_prbs_test {
	struct device *dev;
	void __iomem *pcb_base;
	struct mutex test_lock;
	unsigned int test_start;
	unsigned int test_retries;
	unsigned int err_status;
};

static void sata_phy_write(void __iomem *pcb_base, u32 bank,
			      u32 ofs, u32 msk, u32 value)
{
	u32 tmp;

	writel(bank, pcb_base + SATA_PCB_BANK_OFFSET);
	tmp = readl(pcb_base + SATA_PCB_REG_OFFSET(ofs));
	tmp = (tmp & msk) | value;
	writel(tmp, pcb_base + SATA_PCB_REG_OFFSET(ofs));
}

static u32 sata_phy_read(void __iomem *pcb_base, u32 bank, u32 ofs)
{
	writel(bank, pcb_base + SATA_PCB_BANK_OFFSET);
	return readl(pcb_base + SATA_PCB_REG_OFFSET(ofs));
}

static int sata_phy_bert_setup(struct sata_prbs_test *test)
{
	uint16_t data_rd;
	void __iomem *base = test->pcb_base;
	int try;
	unsigned int val;

	/* Shorten PLL Lock time - writes to speed up tuning */
	val = SR_PLL1_ACTRL2_MAGIC;
	sata_phy_write(base, PLL1_REG_BANK, PLL1_ACTRL2, 0x0, val);
	val = SR_PLL1_ACTRL3_MAGIC;
	sata_phy_write(base, PLL1_REG_BANK, PLL1_ACTRL3, 0x0, val);
	val = SR_PLL1_ACTRL4_MAGIC;
	sata_phy_write(base, PLL1_REG_BANK, PLL1_ACTRL4, 0x0, val);
	val = SR_PLL0_ACTRL6_MAGIC;
	sata_phy_write(base, PLL0_REG_BANK, PLL0_ACTRL6, 0x0, val);
	dev_info(test->dev, "Finished writes to speed up tuning\n");

	/* Clear status register */
	sata_phy_read(base, BLOCK0_REG_BANK, BLOCK0_XGXSSTATUS);

	/* Wait for PHY PLL Lock by polling pll_lock bit */
	try = 50;
	while (try) {
		val = sata_phy_read(base, BLOCK0_REG_BANK, BLOCK0_XGXSSTATUS);
		if (val & BLOCK0_XGXSSTATUS_PLL_LOCK)
			break;
		msleep(20);
		try--;
	}
	if (!try) {
		/* PLL did not lock; give up */
		dev_err(test->dev, "PLL did not lock\n");
		return -ETIMEDOUT;
	}

	/* override SAPIS_DATA_RATE input */
	sata_phy_write(base, BLOCK0_REG_BANK, BLOCK0_SPEEDCTRL, 0x0,
			SAPIS_DATA_RATE_OVERRIDE_VAL);
	/* Software override SAPIS enable and enable tx0 driver */
	sata_phy_write(base, BLOCK0_REG_BANK, BLOCK0_SPARE, 0x0,
			OOB_REF_CLK_SEL);

	/* Verifying that data has been written */
	data_rd = sata_phy_read(base, BLOCK0_REG_BANK, BLOCK0_SPEEDCTRL);

	if (data_rd == SAPIS_DATA_RATE_OVERRIDE_VAL) {
		dev_info(test->dev, "Data read from 0x%x is correct- 0x%x\n",
			 BLOCK0_SPEEDCTRL, data_rd);
		dev_info(test->dev, "overridden SAPIS_DATA_RATE input");
	}
	else {
		dev_err(test->dev, "Data read from address 0x%x is incorrect\n",
			BLOCK0_SPEEDCTRL);
		dev_info(test->dev, "Expected is 0x%x, Received is 0x%x\n",
			 SAPIS_DATA_RATE_OVERRIDE_VAL, data_rd);
		return -EIO;
	}

	data_rd = sata_phy_read(base, BLOCK0_REG_BANK, BLOCK0_SPARE);
	if (data_rd == OOB_REF_CLK_SEL) {
		dev_info(test->dev, "Data read from 0x%x is correct- 0x%x\n",
			 BLOCK0_SPARE, data_rd);
		dev_info(test->dev, "Software override SAPIS enable and enabled tx0 driver");
	}
	else {
		dev_err(test->dev, "Data read from address 0x%x is incorrect\n",
			BLOCK0_SPARE);
		dev_info(test->dev, "Expected is 0x%x, Received is 0x%x\n",
			 OOB_REF_CLK_SEL, data_rd);
		return -EIO;
	}

	/* Set tx0_data_rate_val for tx0 driver */
	sata_phy_write(base, BLOCK1_REG_BANK, BLOCK1_TX_TEST, 0x0,
			TX0_DATA_RATE_VAL);

	data_rd = sata_phy_read(base, BLOCK1_REG_BANK, BLOCK1_TX_TEST);
	if (data_rd == TX0_DATA_RATE_VAL) {
		dev_info(test->dev, "Data read from 0x%x is correct- 0x%x\n",
			 BLOCK1_TX_TEST, data_rd);
		dev_info(test->dev, "Set tx0_data_rate_val for tx0 driver");
	}
	else {
		dev_err(test->dev, "Data read from address 0x%x is incorrect\n",
			BLOCK1_TX_TEST);
		dev_info(test->dev, "Expected is 0x%x, Received is 0x%x\n",
			 TX0_DATA_RATE_VAL, data_rd);
		return -EIO;
	}
	return 0;
}

static int sata_phy_begin_test(struct sata_prbs_test *test)
{
	uint16_t data_rd;
	void __iomem *base = test->pcb_base;

	/* Select prbs data as TX test data source */
	data_rd = sata_phy_read(base, TX_REG_BANK, TX_GEN_CTRL1);
	dev_info(test->dev, "Data read from 0x%x is- 0x%x\n",
			TX_GEN_CTRL1, data_rd);

	sata_phy_write(base, TX_REG_BANK, TX_GEN_CTRL1, 0x0, 0x42);

	/* Enable PRBS7 monitor */
	sata_phy_write(base, BLOCK1_REG_BANK, BLOCK1_PRBSCONTROL, 0x0, 0x88);
	udelay(50);
	dev_info(test->dev, "Enabled PRBS monitor\n");

	sata_phy_write(base, RX_REG_BANK, RX_GEN_CTRL2, 0x0, 0x0800);
	/* Select PRBS status register as RX_STATUS register input */
	sata_phy_write(base, RX_REG_BANK, RX_GEN_CTRL1, 0x0, 0x5CD7);

	/* Read status register */
	data_rd = sata_phy_read(base, RX_REG_BANK, RX_STATUS);
	if (data_rd == RX_DFE_PASS_VAL)
		dev_info(test->dev, "Data read from DFE is correct - 0x%x",
			 data_rd);
	else {
		dev_err(test->dev, "Data read from DFE is incorrect");
		dev_info(test->dev, "Expected is 0x%x, Received is 0x%x\n",
				RX_DFE_PASS_VAL, data_rd);
		return -EIO;
	}
	return 0;
}

static int do_prbs_test(struct sata_prbs_test *test)
{
	int ret = 0;

	ret = sata_phy_bert_setup(test);
	if (ret) {
		dev_err(test->dev, "SATA PHY BERT setup FAILED\n");
		return 0;
	} else
		dev_info(test->dev, "BERT setup done");

	ret = sata_phy_begin_test(test);
	if (ret)
		dev_err(test->dev, "SATA PHY PRBS test FAILED\n");
	else
		dev_info(test->dev, "SATA PHY PRBS test PASSED\n");

	return ret;
}

/* sysfs callbacks */
static ssize_t sata_prbs_err_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	ssize_t ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct sata_prbs_test *test = platform_get_drvdata(pdev);

	mutex_lock(&test->test_lock);
	ret = sprintf(buf, "%u\n", test->err_status);
	mutex_unlock(&test->test_lock);

	return ret;
}

static ssize_t sata_prbs_retries_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	ssize_t ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct sata_prbs_test *test = platform_get_drvdata(pdev);

	mutex_lock(&test->test_lock);
	ret = sprintf(buf, "%u\n", test->test_retries);
	mutex_unlock(&test->test_lock);

	return ret;
}

static ssize_t sata_prbs_retries_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int state;
	struct platform_device *pdev = to_platform_device(dev);
	struct sata_prbs_test *test = platform_get_drvdata(pdev);

	if (kstrtoint(buf, 0, &state) != 0)
		return -EINVAL;
	if (state < 1)
		return -EINVAL;
	mutex_lock(&test->test_lock);
	test->test_retries = state;
	mutex_unlock(&test->test_lock);

	return strnlen(buf, count);
}

static ssize_t sata_prbs_start_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	ssize_t ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct sata_prbs_test *test = platform_get_drvdata(pdev);

	mutex_lock(&test->test_lock);
	ret = sprintf(buf, "%u\n", test->test_start);
	mutex_unlock(&test->test_lock);

	return ret;
}

static ssize_t sata_prbs_start_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int state, ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct sata_prbs_test *test = platform_get_drvdata(pdev);

	if (kstrtoint(buf, 0, &state) != 0)
		return -EINVAL;
	if (state < 1)
		return -EINVAL;

	mutex_lock(&test->test_lock);
	test->test_start = state;
	if (test->test_start) {
		dev_info(test->dev, "SATA PHY test started\n");
		ret = do_prbs_test(test);
		if (ret)
			test->err_status = 1;
		test->test_start = 0;
	}
	mutex_unlock(&test->test_lock);

	return strnlen(buf, count);
}

static DEVICE_ATTR(test_retries, S_IRUGO | S_IWUSR,
		   sata_prbs_retries_show, sata_prbs_retries_store);

static DEVICE_ATTR(err_status, S_IRUGO,
		   sata_prbs_err_show, NULL);

static DEVICE_ATTR(test_start, S_IRUGO | S_IWUSR,
		   sata_prbs_start_show, sata_prbs_start_store);

static int stingray_sata_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn = dev->of_node, *child;
	struct sata_prbs_test *test;
	struct resource *res;
	void __iomem *base;
	unsigned int count = 0;
	int ret = 0;

	for_each_available_child_of_node(dn, child)
		count++;
	if (count < 1)
		return -ENODEV;

	test = devm_kzalloc(dev, sizeof(*test), GFP_KERNEL);
	if (!test)
		return -ENOMEM;

	platform_set_drvdata(pdev, test);
	test->dev = dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy");
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	test->pcb_base = base;

	/* creating sysfs entries */
	ret = device_create_file(dev, &dev_attr_test_retries);
	if (ret < 0)
		return ret;
	ret = device_create_file(dev, &dev_attr_err_status);
	if (ret < 0)
		goto destroy_test_retries;
	ret = device_create_file(dev, &dev_attr_test_start);
	if (ret < 0)
		goto destroy_err_status;

	mutex_init(&test->test_lock);
	test->test_retries = 0;
	test->test_start = 0;
	test->err_status = 0;

	dev_info(dev, "SATA PHY initialized for PRBS test\n");

	return 0;

destroy_err_status:
	device_remove_file(dev, &dev_attr_err_status);
destroy_test_retries:
	device_remove_file(dev, &dev_attr_test_retries);
	return ret;
}

static const struct of_device_id stingray_sata_phy_of_match[] = {
	{ .compatible = "brcm,stingray-sata-phy-prbs" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, stingray_sata_phy_of_match);

static struct platform_driver stingray_sata_prbs_driver = {
	.driver = {
		.name = "stingray-sata-prbs",
		.of_match_table = stingray_sata_phy_of_match,
	},
	.probe = stingray_sata_phy_probe,
};
module_platform_driver(stingray_sata_prbs_driver);

MODULE_DESCRIPTION("Broadcom Stingray SATA PHY PRBS test driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Broadcom");
