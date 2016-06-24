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
 *
 * You should have received a copy of the GNU General Public License
 * version 2 (GPLv2) along with this source code.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mdio.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/regmap.h>
#include <linux/string.h>

#define MAX_PHY_COUNT 8
#define PIPE_MUX_CONFIG 0x10C

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

static struct mdio_device *pcie[MAX_PHY_COUNT];
static struct regmap *pcie_strap_map;
static enum pcie_modes pcie_mode;
static unsigned int test_retries;
static unsigned int slot_num;
static unsigned int err_count;
static unsigned int test_start;
static unsigned int phy_count;
static DEFINE_MUTEX(test_lock);

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

static void pcie_phy_bert_setup(int phy_num)
{
	unsigned int phy_id = pcie[phy_num]->addr;
	struct mii_bus *bus = pcie[phy_num]->bus;

	mdiobus_write(bus, phy_id, 0x1f, PCIE3_BLK_ADR);
	mdiobus_write(bus, phy_id, GEN2_CTRL1_A, 0x05);

	mdiobus_write(bus, phy_id, 0x1f, PCIE4_BLK_ADR);
	mdiobus_write(bus, phy_id, LANE_CTRL2_A, 0x00c0);

	/*setup prbs test*/
	mdiobus_write(bus, phy_id, 0x1f, PCIE5_BLK_ADR);
	mdiobus_write(bus, phy_id, LANE_PRBS3_A, 0xe4e4);
	mdiobus_write(bus, phy_id, LANE_PRBS4_A, 0xe4e4);
}

static int pcie_phy_begin_test(int phy_num)
{
	uint16_t data_rd;
	unsigned int phy_id = pcie[phy_num]->addr;
	struct mii_bus *bus = pcie[phy_num]->bus;
	struct device *dev = &pcie[phy_num]->dev;

	int ret = 0;

	/* Lane 0 */
	mdiobus_write(bus, phy_id, 0x1f,
			  MERLIN16_ADDR_MDIO_MMDSEL_AER_COM);
	mdiobus_write(bus, phy_id, MERLIN16_MDIO_MMDSEL_AER_COMMDIO_AER_A,
			  0x0800);

	mdiobus_write(bus, phy_id, 0x1f, RX_DFE0_BLK_ADR);
	mdiobus_write(bus, phy_id, RX_DFE0_CTRL4_A, 0x1130);
	mdiobus_write(bus, phy_id, RX_DFE0_CTRL1_A, 0xe002);

	/* Lane 1 */
	mdiobus_write(bus, phy_id, 0x1f,
			  MERLIN16_ADDR_MDIO_MMDSEL_AER_COM);
	mdiobus_write(bus, phy_id, MERLIN16_MDIO_MMDSEL_AER_COMMDIO_AER_A,
			  0x0801);

	mdiobus_write(bus, phy_id, 0x1f, RX_DFE0_BLK_ADR);
	mdiobus_write(bus, phy_id, RX_DFE0_CTRL4_A, 0x1130);
	mdiobus_write(bus, phy_id, RX_DFE0_CTRL1_A, 0xe002);

	mdiobus_write(bus, phy_id, 0x1f,
			  MERLIN16_ADDR_MDIO_MMDSEL_AER_COM);
	mdiobus_write(bus, phy_id, MERLIN16_MDIO_MMDSEL_AER_COMMDIO_AER_A,
			  0x0800);
	mdiobus_write(bus, phy_id, 0x1f, PCIE5_BLK_ADR);
	mdiobus_write(bus, phy_id, LANE_PRBS0_A, 0xffff);

	ndelay(500);

	/* Clear PRBS Error status */
	mdiobus_write(bus, phy_id, 0x1f,
			  MERLIN16_ADDR_MDIO_MMDSEL_AER_COM);
	mdiobus_write(bus, phy_id, MERLIN16_MDIO_MMDSEL_AER_COMMDIO_AER_A,
			  0x0800);

	mdiobus_write(bus, phy_id, 0x1f, RX_DFE0_BLK_ADR);
	mdiobus_read(bus, phy_id, RX_DFE0_STATUS1_A);

	udelay(20);

	/* Read PRBS Error status */
	mdiobus_write(bus, phy_id, 0x1f,

			  MERLIN16_ADDR_MDIO_MMDSEL_AER_COM);
	mdiobus_write(bus, phy_id, MERLIN16_MDIO_MMDSEL_AER_COMMDIO_AER_A,
			  0x0800);

	mdiobus_write(bus, phy_id, 0x1f, RX_DFE0_BLK_ADR);
	data_rd = mdiobus_read(bus, phy_id, RX_DFE0_STATUS1_A);

	if (data_rd == RX_DFE0_PASS_VAL)
		dev_info(dev, "Data read from DFE0 for phy-%d is correct- 0x%x",
			 phy_id, data_rd);
	else {
		dev_err(dev, "Data read from DFE0 for phy-%d is incorrect",
			phy_id);
		dev_err(dev, "Expected is 0x%x, Received is 0x%x\n",
			RX_DFE0_PASS_VAL, data_rd);
		ret = 1;
	}

	mdiobus_write(bus, phy_id, 0x1f,
			  MERLIN16_ADDR_MDIO_MMDSEL_AER_COM);
	mdiobus_write(bus, phy_id, MERLIN16_MDIO_MMDSEL_AER_COMMDIO_AER_A,
			  0x0801);

	mdiobus_write(bus, phy_id, 0x1f, RX_DFE0_BLK_ADR);
	data_rd = mdiobus_read(bus, phy_id, RX_DFE0_STATUS1_A);

	if (data_rd == RX_DFE0_PASS_VAL)
		dev_info(dev, "Data read from DFE1 for phy-%d is correct - 0x%x",
			 phy_id, data_rd);
	else {
		dev_err(dev, "Data read from DFE1 for phy-%d is incorrect",
			phy_id);
		dev_err(dev, "Expected is 0x%x, Received is 0x%x\n",
			RX_DFE0_PASS_VAL, data_rd);
		ret = 1;
	}
	return ret;
}

static int do_prbs_test(struct device *dev)
{
	int phy_num, phy_err, ret, i;

	if (phy_count < MAX_PHY_COUNT) {
		dev_err(dev, "All the PCIe phys not registered\n");
		return -EINVAL;
	}
	if (phy_mask[pcie_mode][slot_num] == 0x00) {
		dev_info(dev, "pcie_mode(%d) and slot_num(%d)\n",
			pcie_mode, slot_num);
		dev_err(dev, "no such combination exists\n");
		return -EINVAL;
	}
	for (i = 0; i <= test_retries; i++) {
		phy_err = 0;

		/* check pcie phys according to self loopback cable position */
		for (phy_num = 0; phy_num < phy_count; phy_num++) {
			if (!((phy_mask[pcie_mode][slot_num]) &
			     (1 << phy_num)))
				continue;

			pcie_phy_bert_setup(phy_num);
			ret = pcie_phy_begin_test(phy_num);
			if (ret) {
				dev_err(dev, "PCIe PHY(0x%.2x) test failed\n",
					pcie[phy_num]->addr);
				phy_err++;
			} else
				dev_info(dev, "PCIe PHY(0x%.2x) test passed\n",
					pcie[phy_num]->addr);
		}
		if (phy_err == 0) {
			dev_info(dev, "Try %d: PCIe PRBS test PASSED (error %d)\n",
				i, phy_err);
			return 0;
		}
		dev_err(dev, "Try %d: PCIe PRBS test Failed (error %d)\n",
			i, phy_err);
		err_count = phy_err;
	}
	return 1;
}

/* sysfs callbacks */
static ssize_t pcie_prbs_retries_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	ssize_t ret;

	mutex_lock(&test_lock);
	ret = sprintf(buf, "%u\n", test_retries);
	mutex_unlock(&test_lock);
	return ret;
}

static ssize_t pcie_prbs_retries_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int state;

	if (kstrtoint(buf, 0, &state) != 0)
		return -EINVAL;
	if (state < 1)
		return -EINVAL;
	mutex_lock(&test_lock);
	test_retries = state;
	mutex_unlock(&test_lock);
	return strnlen(buf, count);
}

static ssize_t pcie_prbs_pcie_mode_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	ssize_t ret;

	mutex_lock(&test_lock);
	ret = sprintf(buf, "%u\n", pcie_mode);
	mutex_unlock(&test_lock);
	return ret;
}

static ssize_t pcie_prbs_pcie_mode_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int state;

	if (kstrtoint(buf, 0, &state) != 0)
		return -EINVAL;
	if (state < PCIE_MODE_DEFAULT || state > PCIE_MODE13)
		return -EINVAL;
	mutex_lock(&test_lock);
	pcie_mode = state;
	mutex_unlock(&test_lock);
	return strnlen(buf, count);
}

static ssize_t pcie_prbs_slot_num_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	ssize_t ret;

	mutex_lock(&test_lock);
	ret = sprintf(buf, "%u\n", slot_num);
	mutex_unlock(&test_lock);
	return ret;
}

static ssize_t pcie_prbs_slot_num_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int state;

	if (kstrtoint(buf, 0, &state) != 0)
		return -EINVAL;
	if (state < 0 || state > 8)
		return -EINVAL;
	mutex_lock(&test_lock);
	slot_num = state;
	mutex_unlock(&test_lock);
	return strnlen(buf, count);
}

static ssize_t pcie_prbs_start_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	ssize_t ret;

	mutex_lock(&test_lock);
	ret = sprintf(buf, "%u\n", test_start);
	mutex_unlock(&test_lock);
	return ret;
}

static ssize_t pcie_prbs_start_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int state;
	unsigned int val;

	if (kstrtoint(buf, 0, &state) != 0)
		return -EINVAL;
	if (state < 1)
		return -EINVAL;
	mutex_lock(&test_lock);
	test_start = state;
	if (test_start) {
		if (pcie_mode == PCIE_MODE_DEFAULT) {
			/* read pcie pipemux configuration register */
			regmap_read(pcie_strap_map, PIPE_MUX_CONFIG, &val);
			pcie_mode = val;
		}
		do_prbs_test(dev);
	}
	mutex_unlock(&test_lock);
	return strnlen(buf, count);
}

static ssize_t pcie_phy_err_count_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	ssize_t ret;

	mutex_lock(&test_lock);
	ret = sprintf(buf, "%u\n", err_count);
	mutex_unlock(&test_lock);
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

static int stingray_pcie_phy_probe(struct mdio_device *mdiodev)
{
	int ret = 0;
	struct device *dev = &mdiodev->dev;
	struct device_node *dn = dev->of_node;

	if (mdiodev->addr >= MAX_PHY_COUNT)
		return -ENODEV;
	pcie_strap_map =
		syscon_regmap_lookup_by_phandle(dn, "brcm, pcie-strap-syscon");
	if (IS_ERR(pcie_strap_map))
		return PTR_ERR(pcie_strap_map);

	pcie[mdiodev->addr] = mdiodev;
	phy_count++;

	/* creating sysfs entries */
	ret = device_create_file(dev, &dev_attr_test_retries);
	if (ret < 0)
		return ret;
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

	mutex_init(&test_lock);
	test_retries = 0;
	test_start = 0;
	pcie_mode = PCIE_MODE_DEFAULT;
	slot_num = 0;
	dev_info(dev, "%s PHY registered\n", dev_name(dev));
	return 0;

destroy_err_count:
	device_remove_file(dev, &dev_attr_err_count);
destroy_slot_num:
	device_remove_file(dev, &dev_attr_slot_num);
destroy_pcie_mode:
	device_remove_file(dev, &dev_attr_pcie_mode);
destroy_test_retries:
	device_remove_file(dev, &dev_attr_test_retries);
	return ret;
}

static const struct of_device_id stingray_pcie_phy_of_match[] = {
	{ .compatible = "brcm,stingray-pcie-phy-prbs" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, stingray_pcie_phy_of_match);

static struct mdio_driver stingray_pcie_prbs_driver = {
	.mdiodrv = {
		.driver = {
			.name = "stingray-pcie-prbs",
			.of_match_table = stingray_pcie_phy_of_match,
		},
	},
	.probe = stingray_pcie_phy_probe,
};
mdio_module_driver(stingray_pcie_prbs_driver);

MODULE_DESCRIPTION("Broadcom Stingray PCIe PHY PRBS test driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Broadcom");
