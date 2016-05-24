/* Broadcom Stingray PCIe PRBS Driver
 *
 * Copyright (C) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/shared_mdio.h>
#include <linux/string.h>

#define MAX_PHY_COUNT 8

/* PCIe SERDES Block Addresses */
#define PCIE5_BLK_ADR 0x1500
#define PCIE4_BLK_ADR 0x1400
#define PCIE3_BLK_ADR 0x1300
#define RX_DFE0_BLK_ADR 0x7000
#define MERLIN16_ADDR_MDIO_MMDSEL_AER_COM 0xffd0

/* PCIE SERDES Registers  */
#define GEN2_CTRL1_A 0x11
#define LANE_CTRL2_A 0x12
#define LANE_PRBS0_A 0x11
#define LANE_PRBS3_A 0x14
#define LANE_PRBS4_A 0x15
#define RX_DFE0_CTRL4_A 0x16
#define RX_DFE0_CTRL1_A 0x13
#define RX_DFE0_STATUS1_A 0x10
#define MERLIN16_MDIO_MMDSEL_AER_COMMDIO_AER_A  0x1e
#define RX_DFE0_PASS_VAL 0x8000

enum pcie_modes {
	PCIE_MODE0 = 0,
	PCIE_MODE1,
	PCIE_MODE2,
	PCIE_MODE3,
	PCIE_MODE4,
	PCIE_MODE5,
	PCIE_MODE6,
	PCIE_MODE7,
	PCIE_MODE8,
	PCIE_MODE9,
	PCIE_MODE10,
	PCIE_MODE11,
	PCIE_MODE12,
	PCIE_MODE13,
	PCIE_MODE_DEFAULT = -1,
};
struct pcie_prbs_test {
	struct device *dev;
	struct shared_mdio_master *pcie;
	unsigned int pcie_phy_id[MAX_PHY_COUNT];
	void __iomem *pcie_strap_regbase;
	enum pcie_modes pcie_mode;
	struct mutex test_lock;
	unsigned int phy_count;
	unsigned int test_retries;
	unsigned int slot_num;
	unsigned int err_count;
	unsigned int test_start;
};
static unsigned int phy_mask[15][8] = {
	/* Mode 0: 1x16(EP) */
	[PCIE_MODE0] = {0xff},
	/* Mode 1: 2x8 (EP) */
	[PCIE_MODE1] = {0x0f, 0xf0},
	/* Mode 2: 4x4 (EP) */
	[PCIE_MODE2] = {0x03, 0x0c, 0x30, 0xc0},
	/* Mode 3: 2x8 (RC) */
	[PCIE_MODE3] = {0x0f, 0xf0},
	/* Mode 4: 4x4 (RC) */
	[PCIE_MODE4] = {0x03, 0x0c, 0x30, 0xc0},
	/* Mode 5: 8x2 (RC) */
	[PCIE_MODE5] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80},
	/* Mode 6: 3x4 , 2x2 (RC) */
	[PCIE_MODE6] = {0x03, 0x0c, 0x30, 0x40, 0x80},
	/* Mode 7: 1x4 , 6x2 (RC) */
	[PCIE_MODE7] = {0x03, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80},
	/* Mode 8: 1x8(EP), 4x2(RC) */
	[PCIE_MODE8] = {0x0f, 0x10, 0x20, 0x40, 0x80},
	/* Mode 9: 1x8(EP), 2x4(RC) */
	[PCIE_MODE9] = {0x0f, 0x30, 0xc0},
	/* Mode 10: 2x4(EP), 2x4(RC) */
	[PCIE_MODE10] = {0x03, 0x0c, 0x30, 0xc0},
	/* Mode 11: 2x4(EP), 4x2(RC) */
	[PCIE_MODE11] = {0x03, 0x0c, 0x10, 0x20, 0x40, 0x80},
	/* Mode 12: 1x4(EP), 6x2(RC) */
	[PCIE_MODE12] = {0x03, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80},
	/* Mode 13: 2x4(EP), 1x4(RC), 2x2(RC) */
	[PCIE_MODE13] = {0x03, 0x0c, 0x30, 0x40, 0x80}
};
/* PCIe PRBS loopback test sequence */

static void pcie_phy_bert_setup(struct pcie_prbs_test *test, int phy_num)
{
	unsigned int phy_id = test->pcie_phy_id[phy_num];
	struct shared_mdio_master *pcie = test->pcie;

	shared_mdio_write(pcie, phy_id, 0x1f, PCIE3_BLK_ADR);
	shared_mdio_write(pcie, phy_id, GEN2_CTRL1_A, 0x05);

	shared_mdio_write(pcie, phy_id, 0x1f, PCIE4_BLK_ADR);
	shared_mdio_write(pcie, phy_id, LANE_CTRL2_A, 0x00c0);

	/*setup prbs test*/
	shared_mdio_write(pcie, phy_id, 0x1f, PCIE5_BLK_ADR);
	shared_mdio_write(pcie, phy_id, LANE_PRBS3_A, 0xe4e4);
	shared_mdio_write(pcie, phy_id, LANE_PRBS4_A, 0xe4e4);
}

static int pcie_phy_begin_test(struct pcie_prbs_test *test, int phy_num)
{
	uint16_t data_rd;
	unsigned int phy_id = test->pcie_phy_id[phy_num];
	struct shared_mdio_master *pcie = test->pcie;
	int ret = 0;

	/* Lane 0 */
	shared_mdio_write(pcie, phy_id, 0x1f,
			  MERLIN16_ADDR_MDIO_MMDSEL_AER_COM);
	shared_mdio_write(pcie, phy_id, MERLIN16_MDIO_MMDSEL_AER_COMMDIO_AER_A,
			  0x0800);

	shared_mdio_write(pcie, phy_id, 0x1f, RX_DFE0_BLK_ADR);
	shared_mdio_write(pcie, phy_id, RX_DFE0_CTRL4_A, 0x1130);
	shared_mdio_write(pcie, phy_id, RX_DFE0_CTRL1_A, 0xe002);

	/* Lane 1 */
	shared_mdio_write(pcie, phy_id, 0x1f,
			  MERLIN16_ADDR_MDIO_MMDSEL_AER_COM);
	shared_mdio_write(pcie, phy_id, MERLIN16_MDIO_MMDSEL_AER_COMMDIO_AER_A,
			  0x0801);

	shared_mdio_write(pcie, phy_id, 0x1f, RX_DFE0_BLK_ADR);
	shared_mdio_write(pcie, phy_id, RX_DFE0_CTRL4_A, 0x1130);
	shared_mdio_write(pcie, phy_id, RX_DFE0_CTRL1_A, 0xe002);

	shared_mdio_write(pcie, phy_id, 0x1f,
			  MERLIN16_ADDR_MDIO_MMDSEL_AER_COM);
	shared_mdio_write(pcie, phy_id, MERLIN16_MDIO_MMDSEL_AER_COMMDIO_AER_A,
			  0x0800);
	shared_mdio_write(pcie, phy_id, 0x1f, PCIE5_BLK_ADR);
	shared_mdio_write(pcie, phy_id, LANE_PRBS0_A, 0xffff);

	ndelay(500);

	/* Clear PRBS Error status */
	shared_mdio_write(pcie, phy_id, 0x1f,
			  MERLIN16_ADDR_MDIO_MMDSEL_AER_COM);
	shared_mdio_write(pcie, phy_id, MERLIN16_MDIO_MMDSEL_AER_COMMDIO_AER_A,
			  0x0800);

	shared_mdio_write(pcie, phy_id, 0x1f, RX_DFE0_BLK_ADR);
	shared_mdio_read(pcie, phy_id, RX_DFE0_STATUS1_A);

	udelay(20);

	/* Read PRBS Error status */
	shared_mdio_write(pcie, phy_id, 0x1f,
			  MERLIN16_ADDR_MDIO_MMDSEL_AER_COM);
	shared_mdio_write(pcie, phy_id, MERLIN16_MDIO_MMDSEL_AER_COMMDIO_AER_A,
			  0x0800);

	shared_mdio_write(pcie, phy_id, 0x1f, RX_DFE0_BLK_ADR);
	data_rd = shared_mdio_read(pcie, phy_id, RX_DFE0_STATUS1_A);

	if (data_rd == RX_DFE0_PASS_VAL)
		dev_info(&pcie->dev, "Data read from DFE0 for phy-%d is correct- 0x%x",
			 phy_id, data_rd);
	else {
		dev_err(&pcie->dev, "Data read from DFE0 for phy-%d is incorrect",
			phy_id);
		dev_err(&pcie->dev, "Expected is 0x%x, Received is 0x%x\n",
			RX_DFE0_PASS_VAL, data_rd);
		ret = 1;
	}

	shared_mdio_write(pcie, phy_id, 0x1f,
			  MERLIN16_ADDR_MDIO_MMDSEL_AER_COM);
	shared_mdio_write(pcie, phy_id, MERLIN16_MDIO_MMDSEL_AER_COMMDIO_AER_A,
			  0x0801);

	shared_mdio_write(pcie, phy_id, 0x1f, RX_DFE0_BLK_ADR);
	data_rd = shared_mdio_read(pcie, phy_id, RX_DFE0_STATUS1_A);

	if (data_rd == RX_DFE0_PASS_VAL)
		dev_info(&pcie->dev, "Data read from DFE1 for phy-%d is correct - 0x%x",
			 phy_id, data_rd);
	else {
		dev_err(&pcie->dev, "Data read from DFE1 for phy-%d is incorrect",
			phy_id);
		dev_err(&pcie->dev, "Expected is 0x%x, Received is 0x%x\n",
			RX_DFE0_PASS_VAL, data_rd);
		ret = 1;
	}
	return ret;
}

static int do_prbs_test(struct pcie_prbs_test *test)
{
	int phy_num, phy_err, ret, i;

	for (i = 0; i < test->test_retries; i++) {
		phy_err = 0;
		for (phy_num = 0; phy_num < test->phy_count; phy_num++) {
			if (!((phy_mask[test->pcie_mode][test->slot_num]) &
			     (1 << phy_num)))
				continue;

			pcie_phy_bert_setup(test, phy_num);
			ret = pcie_phy_begin_test(test, phy_num);
			if (ret) {
				dev_err(test->dev, "PCIe PHY-%d test failed\n",
					test->pcie_phy_id[phy_num]);
				phy_err++;
			} else
				dev_info(test->dev, "PCIe PHY-%d test passed\n",
					test->pcie_phy_id[phy_num]);
		}
		if (phy_err == 0) {
			dev_info(test->dev, "Try %d: PCIe PRBS test PASSED (error %d)\n",
				test->test_retries, phy_err);
			return 0;
		}
		dev_err(test->dev, "Try %d: PCIe PRBS test Failed (error %d)\n",
			test->test_retries, phy_err);
		test->err_count = phy_err;
	}
	return 1;
}

/* sysfs callbacks */
static ssize_t pcie_prbs_retries_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct shared_mdio_master *pdev = to_shared_mdio_master(dev);
	struct pcie_prbs_test *test = shared_mdio_get_drvdata(pdev);
	ssize_t ret;

	mutex_lock(&test->test_lock);
	ret = sprintf(buf, "%u\n", test->test_retries);
	mutex_unlock(&test->test_lock);
	return ret;
}

static ssize_t pcie_prbs_retries_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct shared_mdio_master *pdev = to_shared_mdio_master(dev);
	struct pcie_prbs_test *test = shared_mdio_get_drvdata(pdev);
	int state;

	if (kstrtoint(buf, 0, &state) != 1)
		return -EINVAL;
	if (state < 1)
		return -EINVAL;
	mutex_lock(&test->test_lock);
	test->test_retries = state;
	mutex_unlock(&test->test_lock);
	return strnlen(buf, count);
}

static ssize_t pcie_prbs_pcie_mode_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct shared_mdio_master *pdev = to_shared_mdio_master(dev);
	struct pcie_prbs_test *test = shared_mdio_get_drvdata(pdev);
	ssize_t ret;

	mutex_lock(&test->test_lock);
	ret = sprintf(buf, "%u\n", test->pcie_mode);
	mutex_unlock(&test->test_lock);
	return ret;
}

static ssize_t pcie_prbs_pcie_mode_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct shared_mdio_master *pdev = to_shared_mdio_master(dev);
	struct pcie_prbs_test *test = shared_mdio_get_drvdata(pdev);
	int state;

	if (kstrtoint(buf, 0, &state) != 1)
		return -EINVAL;
	if (state < 1)
		return -EINVAL;
	mutex_lock(&test->test_lock);
	test->pcie_mode = state;
	mutex_unlock(&test->test_lock);
	return strnlen(buf, count);
}

static ssize_t pcie_prbs_slot_num_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct shared_mdio_master *pdev = to_shared_mdio_master(dev);
	struct pcie_prbs_test *test = shared_mdio_get_drvdata(pdev);
	ssize_t ret;

	mutex_lock(&test->test_lock);
	ret = sprintf(buf, "%u\n", test->slot_num);
	mutex_unlock(&test->test_lock);
	return ret;
}

static ssize_t pcie_prbs_slot_num_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct shared_mdio_master *pdev = to_shared_mdio_master(dev);
	struct pcie_prbs_test *test = shared_mdio_get_drvdata(pdev);
	int state;

	if (kstrtoint(buf, 0, &state) != 1)
		return -EINVAL;
	if (state < 1)
		return -EINVAL;
	mutex_lock(&test->test_lock);
	test->slot_num = state;
	mutex_unlock(&test->test_lock);
	return strnlen(buf, count);
}

static ssize_t pcie_prbs_start_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct shared_mdio_master *pdev = to_shared_mdio_master(dev);
	struct pcie_prbs_test *test = shared_mdio_get_drvdata(pdev);
	ssize_t ret;

	mutex_lock(&test->test_lock);
	ret = sprintf(buf, "%u\n", test->test_start);
	mutex_unlock(&test->test_lock);
	return ret;
}

static ssize_t pcie_prbs_start_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct shared_mdio_master *pdev = to_shared_mdio_master(dev);
	struct pcie_prbs_test *test = shared_mdio_get_drvdata(pdev);
	int state;
	unsigned int val;

	if (kstrtoint(buf, 0, &state) != 1)
		return -EINVAL;
	if (state < 1)
		return -EINVAL;
	mutex_lock(&test->test_lock);
	test->test_start = state;
	if (test->test_start) {
		if (test->pcie_mode == PCIE_MODE_DEFAULT) {
			/* read pcie pipemux configuration register */
			val = readl(test->pcie_strap_regbase + 0x10c);
			val &= 0xf;
			test->pcie_mode = val;
		}
		do_prbs_test(test);
	}
	mutex_unlock(&test->test_lock);
	return strnlen(buf, count);
}

static ssize_t pcie_phy_err_count_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct shared_mdio_master *pdev = to_shared_mdio_master(dev);
	struct pcie_prbs_test *test = shared_mdio_get_drvdata(pdev);
	ssize_t ret;

	mutex_lock(&test->test_lock);
	ret = sprintf(buf, "%u\n", test->err_count);
	mutex_unlock(&test->test_lock);
	return ret;
}

static DEVICE_ATTR(test_retries, S_IRUGO | S_IWUSR,
		   pcie_prbs_retries_show, pcie_prbs_retries_store);

static DEVICE_ATTR(pcie_mode, S_IRUGO | S_IWUSR,
		   pcie_prbs_pcie_mode_show, pcie_prbs_pcie_mode_store);

static DEVICE_ATTR(slot_num, S_IRUGO | S_IWUSR,
		   pcie_prbs_slot_num_show, pcie_prbs_slot_num_store);

static DEVICE_ATTR(test_start, S_IRUGO | S_IWUSR,
		   pcie_prbs_start_show, pcie_prbs_start_store);

static DEVICE_ATTR(err_count, S_IRUGO,
		   pcie_phy_err_count_show, NULL);

static int stingray_pcie_phy_probe(struct shared_mdio_master *master)
{
	int ret = 0;
	struct device *dev = &master->dev;
	struct device_node *dn = dev->of_node, *child, *pcie_strap;
	struct pcie_prbs_test *test;

	test = devm_kzalloc(dev, sizeof(*test), GFP_KERNEL);
	if (!test)
		return -ENOMEM;
	test->dev = dev;
	shared_mdio_set_drvdata(master, test);

	if (of_get_child_count(dn) == 0)
		return -ENODEV;

	pcie_strap = of_parse_phandle(dn, "brcm,pcie-strap", 0);
	if (!pcie_strap)
		return -ENODEV;

	test->pcie_strap_regbase = of_iomap(pcie_strap, 0);
	of_node_put(pcie_strap);
	if (!test->pcie_strap_regbase)
		return -ENODEV;
	test->pcie = master;

	/* find-out number of PHYs and their IDs */
	test->phy_count = 0;
	for_each_available_child_of_node(dn, child) {
		unsigned int id;

		if (of_property_read_u32(child, "reg", &id)) {
			dev_err(dev, "missing reg property in node %s\n",
					child->name);
			ret = -EINVAL;
			goto release_iomap;
		}
		test->pcie_phy_id[test->phy_count++] = id;
	}

	/* creating sysfs entries */
	ret = device_create_file(dev, &dev_attr_test_retries);
	if (ret < 0)
		goto release_iomap;
	ret = device_create_file(dev, &dev_attr_pcie_mode);
	if (ret < 0)
		goto destroy_test_retries;
	ret = device_create_file(dev, &dev_attr_slot_num);
	if (ret < 0)
		goto destroy_pcie_mode;
	ret = device_create_file(dev, &dev_attr_err_count);
	if (ret < 0)
		goto destroy_slot_num;
	ret = device_create_file(dev, &dev_attr_test_start);
	if (ret < 0)
		goto destroy_err_count;

	mutex_init(&test->test_lock);
	test->test_retries = 10;
	test->test_start = 0;
	test->pcie_mode = PCIE_MODE_DEFAULT;
	test->slot_num = 0;
	dev_info(dev, "Stingray PCIe PRBS test ready\n");
	return 0;

destroy_err_count:
	device_remove_file(dev, &dev_attr_err_count);
destroy_slot_num:
	device_remove_file(dev, &dev_attr_slot_num);
destroy_pcie_mode:
	device_remove_file(dev, &dev_attr_pcie_mode);
destroy_test_retries:
	device_remove_file(dev, &dev_attr_test_retries);
release_iomap:
	iounmap(pcie_strap);
	return ret;
}

static const struct of_device_id stingray_pcie_phy_of_match[] = {
	{ .compatible = "brcm,stingray-mdio-master-pcie-prbs" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, stingray_pcie_phy_of_match);

static struct shared_mdio_driver stingray_pcie_prbs_driver = {
	.driver = {
		.name = "stingray-pcie-prbs",
		.of_match_table = stingray_pcie_phy_of_match,
	},
	.probe = stingray_pcie_phy_probe,
};
module_shared_mdio_driver(stingray_pcie_prbs_driver);

MODULE_DESCRIPTION("Broadcom Stingray PCIe PHY PRBS test driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Broadcom");
