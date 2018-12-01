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
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/string.h>

#define MAX_PHY_COUNT 8

#define CDRU_STRAP_DATA_LSW_OFFSET	0x5c

#define PCIE_PIPEMUX_CFG_OFFSET		0x10c
#define PCIE_PIPEMUX_SHIFT		19
#define PCIE_PIPEMUX_MASK		0xf

#define PCIE_CORE0_PMI_SEL_CFG		0x864
#define PCIE_CORE1_PMI_SEL_CFG		0x964
#define PCIE_CORE2_PMI_SEL_CFG		0xa64
#define PCIE_CORE3_PMI_SEL_CFG		0xb64
#define PCIE_CORE4_PMI_SEL_CFG		0xc64
#define PCIE_CORE5_PMI_SEL_CFG		0xd64
#define PCIE_CORE6_PMI_SEL_CFG		0xe64
#define PCIE_CORE7_PMI_SEL_CFG		0xf64

#define PAXB_CFG_CFG_TYPE_MASK		0x1
#define PAXB_CFG_IND_ADDR_OFFSET	0x120
#define PAXB_CFG_IND_ADDR_MASK		0x00001ffc
#define PAXB_CFG_IND_DATA_OFFSET	0x124

#define CFG_RC_PMI_ADDR			0x1130
#define CFG_RC_DEV_ID_SHIFT		27
#define CFG_RC_DEV_ID			BIT(27)
#define CFG_RC_BCAST_SHIFT		21
#define CFG_RC_BCAST_VAL		0xf
#define CFG_RC_LANE_SHIFT		16
#define LANE_BCAST_VAL			0x1f
#define NR_LANES_PER_SET		4
#define NR_LANES_PER_PHY		2
#define LANE_OFFSET_SHIFT		2

#define CFG_RC_PMI_WDATA		0x1134
#define CFG_RC_WCMD_SHIFT		31
#define CFG_RC_WCMD_MASK		(1 << CFG_RC_WCMD_SHIFT)
#define CFG_RC_RCMD_SHIFT		30
#define CFG_RC_RCMD_MASK		(1 << CFG_RC_RCMD_SHIFT)
#define CFG_RC_PMI_RDATA		0x1138
#define CFG_RC_RWCMD_MASK		(CFG_RC_WCMD_MASK | CFG_RC_RCMD_MASK)
#define CFG_RC_RDATA_MASK		0xffff
#define CFG_RC_RACK_SHIFT		31
#define CFG_RC_RACK_MASK		(1 << CFG_RC_RACK_SHIFT)

#define MERLIN16_PCIE_BLK2_PWRMGMT_7		0x1208
#define MERLIN16_PCIE_BLK2_PWRMGMT_8		0x1209
#define MERLIN16_AMS_TX_CTRL_5			0xd0a5
#define MERLIN16_AMS_TX_CTRL_5_POST2_TO_1	BIT(13)
#define MERLIN16_AMS_TX_CTRL_5_ENA_PRE		BIT(12)
#define MERLIN16_AMS_TX_CTRL_5_ENA_POST1	BIT(11)
#define MERLIN16_AMS_TX_CTRL_5_ENA_POST2	BIT(10)
#define MERLIN16_PCIE_BLK2_PWRMGMT_7_VAL	0x96
#define MERLIN16_PCIE_BLK2_PWRMGMT_8_VAL	0x12c

/* allow up to 5 ms for PMI read/write transaction to finish */
#define PMI_TIMEOUT_MS			5
#define GEN1_PRBS_VAL			0x4
#define GEN2_PRBS_VAL			0x5
#define GEN3_PRBS_VAL			0x6
#define GEN_STR_LEN			4
#define MAX_LANE_RETRIES		10
#define PMI_PASS_STATUS			0x8000

#define RC_PCIE_RST_OUTPUT_SHIFT	0
#define RC_PCIE_RST_OUTPUT		BIT(RC_PCIE_RST_OUTPUT_SHIFT)

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

struct pcie_prbs_dev {
	struct device *dev;
	struct regmap *pipemux_strap_map;
	void __iomem *pcie_ss_base;
	void __iomem *paxb_base[MAX_PHY_COUNT];
	char test_gen[GEN_STR_LEN];
	unsigned int test_retries;
	unsigned int slot_num;
	unsigned int err_count;
	unsigned int test_start;
	unsigned int phy_count;
	unsigned int lane;
	enum pcie_modes pcie_mode;
	struct mutex test_lock;
};

/*
 * Following table gives information about PHYs are wired to which
 * core in given pcie RC mode.
 */
static unsigned int phy_mask[14][8] = {
	/* Mode 0: 1x16(EP) */
	[PCIE_MODE0] = {0x00},
	/* Mode 1: 2x8 (EP) */
	[PCIE_MODE1] = {0x00},
	/* Mode 2: 4x4 (EP) */
	[PCIE_MODE2] = {0x00},
	/* Mode 3: 2x8 (RC) */
	[PCIE_MODE3] = {0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0},
	/* Mode 4: 4x4 (RC) */
	[PCIE_MODE4] = {0x03, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x30, 0xc0},
	/* Mode 5: 8x2 (RC) */
	[PCIE_MODE5] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80},
	/* Mode 6: 3x4 , 2x2 (RC) */
	[PCIE_MODE6] = {0x03, 0x00, 0x04, 0x08, 0x00, 0x00, 0x30, 0xc0},
	/* Mode 7: 1x4 , 6x2 (RC) */
	[PCIE_MODE7] = {0x03, 0x00, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80},
	/* Mode 8: 1x8(EP), 4x2(RC) */
	[PCIE_MODE8] = {0x00, 0x00, 0x00, 0x00, 0x10, 0x20, 0x40, 0x80},
	/* Mode 9: 1x8(EP), 2x4(RC) */
	[PCIE_MODE9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0xc0},
	/* Mode 10: 2x4(EP), 2x4(RC) */
	[PCIE_MODE10] = {0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00},
	/* Mode 11: 2x4(EP), 4x2(RC) */
	[PCIE_MODE11] = {0x00, 0x00, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00},
	/* Mode 12: 1x4(EP), 6x2(RC) */
	[PCIE_MODE12] = {0x00, 0x00, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80},
	/* Mode 13: 2x4(EP), 1x4(RC), 2x2(RC) */
	[PCIE_MODE13] = {0x00, 0x00, 0x04, 0x08, 0x00, 0x00, 0x30, 0x00}
};

/* Following table indicates the PHYs requiring work-around */
static unsigned int phy_workaround_table[14] = {
	/* Mode 0: 1x16(EP) */
	0x00,
	/* Mode 1: 2x8 (EP) */
	0x00,
	/* Mode 2: 4x4 (EP) */
	0x00,
	/* Mode 3: 2x8 (RC) */
	0x88,		/* work-around is needed for serdes 3 and 7 */
	/* Mode 4: 4x4 (RC) */
	0x00,
	/* Mode 5: 8x2 (RC) */
	0x00,
	/* Mode 6: 3x4 , 2x2 (RC) */
	0x00,
	/* Mode 7: 1x4 , 6x2 (RC) */
	0x00,
	/* Mode 8: 1x8(EP), 4x2(RC) */
	0x00,
	/* Mode 9: 1x8(EP), 2x4(RC) */
	0x00,
	/* Mode 10: 2x4(EP), 2x4(RC) */
	0x00,
	/* Mode 11: 2x4(EP), 4x2(RC) */
	0x00,
	/* Mode 12: 1x4(EP), 6x2(RC) */
	0x00,
	/* Mode 13: 2x4(EP), 1x4(RC), 2x2(RC) */
	0x00
};

static inline u32 pmi_addr(u16 addr, u8 lane)
{
	u32 val;

	if (lane == LANE_BCAST_VAL) {
		val = CFG_RC_DEV_ID |
		      (CFG_RC_BCAST_VAL << CFG_RC_BCAST_SHIFT) |
		      (LANE_BCAST_VAL << CFG_RC_LANE_SHIFT) |
		      addr;
	} else {
		lane = ((lane / NR_LANES_PER_SET) << LANE_OFFSET_SHIFT) |
		       (lane % NR_LANES_PER_SET);
		val = CFG_RC_DEV_ID | (lane << CFG_RC_LANE_SHIFT) | addr;
	}

	return val;
}

static uint32_t pcie_pipemux_strap_read(struct pcie_prbs_dev *pd)
{
	uint32_t pipemux;

	/* read the PCIe PIPEMUX strap setting */
	regmap_read(pd->pipemux_strap_map,
				CDRU_STRAP_DATA_LSW_OFFSET, &pipemux);
	pipemux >>= PCIE_PIPEMUX_SHIFT;
	pipemux &= PCIE_PIPEMUX_MASK;

	return pipemux;
}

static void paxb_rc_write_config(void __iomem *base, unsigned int where,
				unsigned int val)
{
	writel((where & PAXB_CFG_IND_ADDR_MASK) | PAXB_CFG_CFG_TYPE_MASK,
		base + PAXB_CFG_IND_ADDR_OFFSET);
	writel(val, base + PAXB_CFG_IND_DATA_OFFSET);
}

static unsigned int paxb_rc_read_config(void __iomem *base, unsigned int where)
{
	unsigned int val;

	writel((where & PAXB_CFG_IND_ADDR_MASK) | PAXB_CFG_CFG_TYPE_MASK,
		base + PAXB_CFG_IND_ADDR_OFFSET);
	val = readl(base + PAXB_CFG_IND_DATA_OFFSET);

	return val;
}

/*
 * Function for writes to the Serdes registers through the PMI interface
 */
static int pmi_write(struct pcie_prbs_dev *pd, uint32_t pmi_addr,
		     u16 data)
{
	void __iomem *base = pd->paxb_base[pd->slot_num];
	uint32_t status, val;
	unsigned int timeout = PMI_TIMEOUT_MS;

	dev_info(pd->dev, "%s: pmi = 0x%x, data = 0x%x\n",
		 __func__, pmi_addr, data);
	paxb_rc_write_config(base, CFG_RC_PMI_ADDR, pmi_addr);

	/* initiate pmi write transaction */
	val = data | CFG_RC_WCMD_MASK;
	paxb_rc_write_config(base, CFG_RC_PMI_WDATA, val);

	/* poll for PMI write transaction completion */
	do {
		status = paxb_rc_read_config(base, CFG_RC_PMI_WDATA);

		/* wait for write command bit to clear */
		if ((status & CFG_RC_WCMD_MASK) == 0)
			return 0;
	} while (timeout--);

	dev_err(pd->dev, "PMI write timeout!\n");
	return -EIO;
}

/*
 * Function to read the Serdes registers through the PMI interface
 */
static int pmi_read(struct pcie_prbs_dev *pd, uint32_t pmi_addr,
		    u16 *data)
{
	void __iomem *base = pd->paxb_base[pd->slot_num];
	uint32_t status;
	unsigned int timeout = PMI_TIMEOUT_MS;

	paxb_rc_write_config(base, CFG_RC_PMI_ADDR, pmi_addr);

	/* initiate PMI read transaction */
	paxb_rc_write_config(base, CFG_RC_PMI_WDATA, CFG_RC_RCMD_MASK);

	/* poll for PMI read transaction completion */
	*data = 0;
	do {
		status = paxb_rc_read_config(base, CFG_RC_PMI_RDATA);
		/* wait for read ack bit set */
		if (status & CFG_RC_RACK_MASK) {
			status = paxb_rc_read_config(base, CFG_RC_PMI_RDATA);
			*data = status & CFG_RC_RDATA_MASK;
			dev_info(pd->dev, "%s : 0x%x = 0x%x\n",
				 __func__, pmi_addr, *data);
			return 0;
		}
	} while (timeout--);

	dev_err(pd->dev, "PMI read timeout!\n");
	return -EIO;
}

static int workaround_needed_for_phy(struct pcie_prbs_dev *pd, int phy_num)
{
	if (phy_workaround_table[pd->pcie_mode] & (1 << phy_num))
		return 1;
	return 0;
}

/* PCIe PRBS loopback test sequence */
static int pcie_phy_bert_setup(struct pcie_prbs_dev *pd, int phy_num)
{
	struct device *dev = pd->dev;
	u32 addr;

	dev_info(dev, "Setting up BERT for PHY 0x%x\n", phy_num);

	/*
	 * Although, signal integrity code is already present in firmware,
	 * if this driver tries to write PIPEMUX register to change PIPEMUX
	 * setting, then SERDES registers are seen be changed causing GEN2
	 * PRBS failure. So applying signal integrity code to SERDES here.
	 */

	/* Enable pre/post cursors */
	addr = pmi_addr(MERLIN16_AMS_TX_CTRL_5, LANE_BCAST_VAL);
	pmi_write(pd, addr,
		  MERLIN16_AMS_TX_CTRL_5_POST2_TO_1 |
		  MERLIN16_AMS_TX_CTRL_5_ENA_PRE |
		  MERLIN16_AMS_TX_CTRL_5_ENA_POST1 |
		  MERLIN16_AMS_TX_CTRL_5_ENA_POST2);

	/* Configure Ref Clock sense counters */
	addr = pmi_addr(MERLIN16_PCIE_BLK2_PWRMGMT_7, LANE_BCAST_VAL);
	pmi_write(pd, addr, MERLIN16_PCIE_BLK2_PWRMGMT_7_VAL);

	addr = pmi_addr(MERLIN16_PCIE_BLK2_PWRMGMT_8, LANE_BCAST_VAL);
	pmi_write(pd, addr, MERLIN16_PCIE_BLK2_PWRMGMT_8_VAL);

	addr = pmi_addr(0x1300, LANE_BCAST_VAL);
	pmi_write(pd, addr, 0x2080);

	/* set speed */
	addr = pmi_addr(0x1301, LANE_BCAST_VAL);
	if (!strncasecmp(pd->test_gen, "gen1", GEN_STR_LEN)) {
		pmi_write(pd, addr, GEN1_PRBS_VAL);
	} else if (!strncasecmp(pd->test_gen, "gen2", GEN_STR_LEN)) {
		pmi_write(pd, addr, GEN2_PRBS_VAL);
	} else if (!strncasecmp(pd->test_gen, "gen3", GEN_STR_LEN)) {
		pmi_write(pd, addr, GEN3_PRBS_VAL);
	} else {
		dev_err(pd->dev, "PCIe GEN: Invalid option\n");
		return -EINVAL;
	}

	/* Disable 8b10b & verify. */
	addr = pmi_addr(0x1402, LANE_BCAST_VAL);
	pmi_write(pd, addr, 0x0000);

	/* PRBS7 is default order ;Set PRBS enable */
	addr = pmi_addr(0x1501, LANE_BCAST_VAL);
	pmi_write(pd, addr, 0xffff);

	/* Set RX status = PRBS monitor on all lanes. */
	addr = pmi_addr(0x7003, LANE_BCAST_VAL);
	pmi_write(pd, addr, 0xe020);

	/* Set sigdet, disable EIEOS in gen3. */
	addr = pmi_addr(0x7007, LANE_BCAST_VAL);
	pmi_write(pd, addr, 0xf010);

	/* workaround for PHY3 and PHY7 PRBS in x8 RC */
	if (workaround_needed_for_phy(pd, phy_num)) {
		addr = pmi_addr(0xd073, LANE_BCAST_VAL);
		pmi_write(pd, addr, 0x7110);
	}

	return 0;
}

static void connect_pcie_core_to_phy(struct pcie_prbs_dev *pd, int phy_num)
{
	struct device *dev = pd->dev;
	void __iomem *pcie_ss_base = pd->pcie_ss_base;
	/* First tie the serdes under test to the given core */
	dev_info(dev, "pcie core=%d and phy=%d\n",
				pd->slot_num, phy_num);
	switch (pd->slot_num) {
	case 0:
		dev_info(dev, "phy %d wired to core0", phy_num);
		writel(phy_num, pcie_ss_base + PCIE_CORE0_PMI_SEL_CFG);
		break;
	case 1:
		writel(phy_num, pcie_ss_base + PCIE_CORE1_PMI_SEL_CFG);
		dev_info(dev, "phy %d wired to core1", phy_num);
		break;
	case 2:
		writel(phy_num, pcie_ss_base + PCIE_CORE2_PMI_SEL_CFG);
		dev_info(dev, "phy %d wired to core2", phy_num);
		break;
	case 3:
		writel(phy_num, pcie_ss_base + PCIE_CORE3_PMI_SEL_CFG);
		dev_info(dev, "phy %d wired to core3", phy_num);
		break;
	case 4:
		writel(phy_num, pcie_ss_base + PCIE_CORE4_PMI_SEL_CFG);
		dev_info(dev, "phy %d wired to core4", phy_num);
		break;
	case 5:
		writel(phy_num, pcie_ss_base + PCIE_CORE5_PMI_SEL_CFG);
		dev_info(dev, "phy %d wired to core5", phy_num);
		break;
	case 6:
		writel(phy_num, pcie_ss_base + PCIE_CORE6_PMI_SEL_CFG);
		dev_info(dev, "phy %d wired to core6", phy_num);
		break;
	case 7:
		writel(phy_num, pcie_ss_base + PCIE_CORE7_PMI_SEL_CFG);
		dev_info(dev, "phy %d wired to core7", phy_num);
		break;
	};
}

static int pcie_phy_lane_prbs_flush(struct pcie_prbs_dev *pd, int lane)
{
	uint32_t addr;
	u16 data;

	addr = pmi_addr(0x7000, lane);
	pmi_read(pd, addr, &data);

	return 0;
}

static int pcie_phy_lane_prbs_status(struct pcie_prbs_dev *pd, int lane)
{
	struct device *dev = pd->dev;
	uint32_t addr;
	u16 data;
	int lane_retries = 0;

	addr = pmi_addr(0x7000, lane);
	do {
		pmi_read(pd, addr, &data);
		dev_info(dev, "Status on Lane %d:[0x%x]\n", lane, data);
		lane_retries++;
	} while ((data != PMI_PASS_STATUS) &&
				(lane_retries < MAX_LANE_RETRIES));

	if (lane_retries == MAX_LANE_RETRIES)
		return -EIO;

	return 0;
}

static int pcie_phy_prbs_status(struct pcie_prbs_dev *pd, int phy_num)
{
	int ret = 0, lane_idx;
	struct device *dev = pd->dev;

	dev_info(dev, "Checking PRBS status for PHY 0x%x\n", phy_num);
	/* Flush PRBS monitor status */
	for (lane_idx = 0; lane_idx < NR_LANES_PER_PHY; lane_idx++)
		pcie_phy_lane_prbs_flush(pd, lane_idx);

	/* Checking PRBS status */
	for (lane_idx = 0; lane_idx < NR_LANES_PER_PHY; lane_idx++) {
		ret = pcie_phy_lane_prbs_status(pd, lane_idx);
		if (ret) {
			dev_err(dev, "PHY 0x%x: Lane %d PRBS failed\n",
				phy_num, lane_idx);
			return ret;
		}

		dev_info(dev, "PHY 0x%x: Lane %d PRBS Passed\n",
			 phy_num, lane_idx);
	}

	return ret;
}

static void iproc_pcie_assert_reset(void __iomem *paxb_base)
{
	uint32_t val;
       /*
	* Select perst_b signal as reset source and put the device into reset
	*/
	val = readl(paxb_base);
	val &= ~RC_PCIE_RST_OUTPUT;
	writel(val, paxb_base);
	udelay(250);
}

static void iproc_pcie_release_reset(void __iomem *paxb_base)
{
	uint32_t val;
	/* Bring the device out of reset */
	val = readl(paxb_base);
	val |= RC_PCIE_RST_OUTPUT;
	writel(val, paxb_base);
	msleep(100);
}

static int do_prbs_test(struct pcie_prbs_dev *pd, unsigned int pipemux_mode)
{
	struct device *dev = pd->dev;
	int phy_num, ret, i;

	if (phy_mask[pipemux_mode][pd->slot_num] == 0x00) {
		dev_info(dev, "pcie_mode(%d) and slot_num(%d)\n",
			pipemux_mode, pd->slot_num);
		dev_err(dev, "no such combination exists in PCIe RC modes!\n");
		/* Set err_count in event of non-existent combination */
		pd->err_count = 1;
		return -EINVAL;
	}

	for (i = 0; i <= pd->test_retries; i++) {
		pd->err_count = 0;
		/*
		 * setup BERT on pcie phys that need to be tested
		 * according to self loopback cable position
		 */
		iproc_pcie_assert_reset(pd->paxb_base[pd->slot_num]);
		for (phy_num = 0; phy_num < pd->phy_count; phy_num++) {
			if (!((phy_mask[pipemux_mode][pd->slot_num]) &
			     (1 << phy_num)))
				continue;
			connect_pcie_core_to_phy(pd, phy_num);
			ret = pcie_phy_bert_setup(pd, phy_num);
			if (ret) {
				dev_err(pd->dev, "PHY 0x%x: BERT setup FAILED",
					phy_num);
				return -EIO;
			}
			dev_info(pd->dev, "PHY 0x%x: BERT setup done", phy_num);
		}
		iproc_pcie_release_reset(pd->paxb_base[pd->slot_num]);

		/* Now check PRBS status for each PHY */
		for (phy_num = 0; phy_num < pd->phy_count; phy_num++) {
			if (!((phy_mask[pipemux_mode][pd->slot_num]) &
			     (1 << phy_num)))
				continue;
			connect_pcie_core_to_phy(pd, phy_num);
			ret = pcie_phy_prbs_status(pd, phy_num);
			if (!ret)
				dev_info(dev, "PHY 0x%x: PRBS test passed\n\n",
						phy_num);
			else {
				dev_err(dev, "PHY 0x%x: PRBS test failed\n\n",
						phy_num);
				pd->err_count++;
			}
		}

		if (pd->err_count == 0) {
			dev_info(dev, "Try %d: PCIe %s PRBS test PASSED\n\n",
					i, pd->test_gen);
			return 0;
		}
		dev_err(dev, "Try %d: PCIe %s PRBS test FAILED (error %d)\n",
				i, pd->test_gen, pd->err_count);
	}
	return 1;
}

/* sysfs callbacks */
static ssize_t pcie_prbs_retries_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	ssize_t ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *test = platform_get_drvdata(pdev);

	mutex_lock(&test->test_lock);
	ret = sprintf(buf, "%u\n", test->test_retries);
	mutex_unlock(&test->test_lock);
	return ret;
}

static ssize_t pcie_prbs_retries_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int state;
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *test = platform_get_drvdata(pdev);

	if (kstrtoint(buf, 0, &state) != 0)
		return -EINVAL;
	if (state < 0)
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
	ssize_t ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *test = platform_get_drvdata(pdev);

	mutex_lock(&test->test_lock);
	ret = sprintf(buf, "%d\n", test->pcie_mode);
	mutex_unlock(&test->test_lock);
	return ret;
}

static ssize_t pcie_prbs_pcie_mode_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int state;
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *test = platform_get_drvdata(pdev);

	if (kstrtoint(buf, 0, &state) != 0)
		return -EINVAL;
	if (state < PCIE_MODE_DEFAULT || state > PCIE_MODE13)
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
	ssize_t ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *test = platform_get_drvdata(pdev);

	mutex_lock(&test->test_lock);
	ret = sprintf(buf, "%u\n", test->slot_num);
	mutex_unlock(&test->test_lock);
	return ret;
}

static ssize_t pcie_prbs_slot_num_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int state;
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *test = platform_get_drvdata(pdev);

	if (kstrtoint(buf, 0, &state) != 0)
		return -EINVAL;
	if (state < 0 || state > 7)
		return -EINVAL;
	mutex_lock(&test->test_lock);
	test->slot_num = state;
	mutex_unlock(&test->test_lock);
	return strnlen(buf, count);
}

static ssize_t pcie_prbs_test_gen_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	ssize_t ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *test = platform_get_drvdata(pdev);

	mutex_lock(&test->test_lock);
	ret = sprintf(buf, "%s", test->test_gen);
	mutex_unlock(&test->test_lock);
	return ret;
}

static ssize_t pcie_prbs_test_gen_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *test = platform_get_drvdata(pdev);

	mutex_lock(&test->test_lock);
	sprintf(test->test_gen, "%s", buf);
	mutex_unlock(&test->test_lock);
	return strnlen(buf, count);
}

static ssize_t pcie_prbs_start_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	ssize_t ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *test = platform_get_drvdata(pdev);

	mutex_lock(&test->test_lock);
	ret = sprintf(buf, "%u\n", test->test_start);
	mutex_unlock(&test->test_lock);
	return ret;
}

static ssize_t pcie_prbs_start_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int state;
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *test = platform_get_drvdata(pdev);
	void __iomem *pcie_ss_base = test->pcie_ss_base;
	unsigned int pipemux_mode;

	if (kstrtoint(buf, 0, &state) != 0)
		return -EINVAL;
	if (state < 1)
		return -EINVAL;
	mutex_lock(&test->test_lock);
	test->test_start = state;
	if (test->test_start) {
		if (test->pcie_mode == PCIE_MODE_DEFAULT) {
			/* read pipemux strap register */
			dev_info(dev, "reading pipemux strap register\n");
			pipemux_mode = pcie_pipemux_strap_read(test);
		} else {
			pipemux_mode =
				readl(pcie_ss_base + PCIE_PIPEMUX_CFG_OFFSET);
			if (pipemux_mode != test->pcie_mode) {
				/*
				 * If value read from PCIE_PIPEMUX_CFG Register
				 * is not same as pcie_mode specified by user,
				 * then configure the PIPE-MUX for pcie_mode
				 */
				pipemux_mode = test->pcie_mode;
				dev_info(dev, "Configuring PIPE-MUX to mode %x\n",
						pipemux_mode);
				writel(pipemux_mode,
					pcie_ss_base + PCIE_PIPEMUX_CFG_OFFSET);
			}
		}
		dev_info(dev, "pcie mode = %d\n", pipemux_mode);
		do_prbs_test(test, pipemux_mode);
	}
	mutex_unlock(&test->test_lock);
	return strnlen(buf, count);
}

static ssize_t pcie_phy_err_count_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	ssize_t ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *test = platform_get_drvdata(pdev);

	mutex_lock(&test->test_lock);
	ret = sprintf(buf, "%u\n", test->err_count);
	mutex_unlock(&test->test_lock);
	return ret;
}

static ssize_t lane_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *pd = platform_get_drvdata(pdev);
	unsigned int lane;
	int ret;

	ret = kstrtouint(buf, 0, &lane);
	if (ret)
		return ret;

	if (lane >= NR_LANES_PER_PHY)
		return -EINVAL;

	mutex_lock(&pd->test_lock);
	pd->lane = lane;
	mutex_unlock(&pd->test_lock);

	return count;
}

static ssize_t lane_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	ssize_t ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *pd = platform_get_drvdata(pdev);

	mutex_lock(&pd->test_lock);
	ret = sprintf(buf, "%u\n", pd->lane);
	mutex_unlock(&pd->test_lock);

	return ret;
}

static ssize_t pmi_read_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *pd = platform_get_drvdata(pdev);
	int ret;
	u32 addr, lane;
	u16 data;

	ret = sscanf(buf, "%x %x", &addr, &lane);
	if ((ret != 1) && (ret != 2))
		return -EINVAL;
	if (ret == 1)
		lane = 0;

	mutex_lock(&pd->test_lock);
	addr = pmi_addr(addr, lane);
	pmi_read(pd, addr, &data);
	mutex_unlock(&pd->test_lock);

	return count;
}

static ssize_t pmi_write_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *pd = platform_get_drvdata(pdev);
	int ret;
	u32 addr, lane, val;

	ret = sscanf(buf, "%x %x %x", &addr, &val, &lane);
	if ((ret != 2) && (ret != 3))
		return -EINVAL;
	if (ret == 2)
		lane = 0;

	mutex_lock(&pd->test_lock);
	addr = pmi_addr(addr, lane);
	pmi_write(pd, addr, val & 0xffff);
	mutex_unlock(&pd->test_lock);

	return count;
}

static DEVICE_ATTR(test_retries, 0644,		/* S_IRUGO | S_IWUSR */
		   pcie_prbs_retries_show, pcie_prbs_retries_store);

static DEVICE_ATTR(pcie_mode, 0644,		/* S_IRUGO | S_IWUSR */
		   pcie_prbs_pcie_mode_show, pcie_prbs_pcie_mode_store);

static DEVICE_ATTR(slot_num, 0644,		/* S_IRUGO | S_IWUSR */
		   pcie_prbs_slot_num_show, pcie_prbs_slot_num_store);

static DEVICE_ATTR(test_gen, 0644,		/* S_IRUGO | S_IWUSR */
		   pcie_prbs_test_gen_show, pcie_prbs_test_gen_store);

static DEVICE_ATTR(test_start, 0644,		/* S_IRUGO | S_IWUSR */
		   pcie_prbs_start_show, pcie_prbs_start_store);

static DEVICE_ATTR(err_count, 0444,		/* S_IRUGO */
		   pcie_phy_err_count_show, NULL);

static DEVICE_ATTR_RW(lane);
static DEVICE_ATTR_WO(pmi_read);
static DEVICE_ATTR_WO(pmi_write);

static int stingray_pcie_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn = dev->of_node, *child;
	struct pcie_prbs_dev *pd;
	struct resource reg;
	uint32_t child_cnt = 0;
	int ret = 0;

	child_cnt = of_get_child_count(dn);
	if (child_cnt < MAX_PHY_COUNT) {
		dev_err(dev, "All the PCIe PHY nodes not present\n");
		return -EINVAL;
	}

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	platform_set_drvdata(pdev, pd);
	pd->dev = dev;

	/* Allocate PCIE_SS resources */
	ret = of_address_to_resource(dn, 0, &reg);
	if (ret < 0) {
		dev_err(dev, "unable to obtain PCIE_SS resources\n");
		return ret;
	}

	pd->pcie_ss_base = devm_ioremap(dev, reg.start, resource_size(&reg));
	if (IS_ERR(pd->pcie_ss_base)) {
		dev_err(dev, "unable to map controller registers\n");
		return PTR_ERR(pd->pcie_ss_base);
	}

	pd->pipemux_strap_map = syscon_regmap_lookup_by_phandle(dn,
					"brcm,pcie-pipemux-strap-syscon");
	if (IS_ERR(pd->pipemux_strap_map)) {
		dev_err(dev, "unable to find CDRU device\n");
		return PTR_ERR(pd->pipemux_strap_map);
	}

	pd->phy_count = 0;
	for_each_available_child_of_node(dn, child) {
		/* Allocate resources for each PCIe PHY */
		ret = of_address_to_resource(child, 0, &reg);
		if (ret < 0) {
			dev_err(dev, "unable to obtain PCIe core %d resources\n",
					pd->phy_count);
			of_node_put(child);
			return -ENOMEM;
		}

		pd->paxb_base[pd->phy_count] =
			devm_ioremap(dev, reg.start, resource_size(&reg));
		if (IS_ERR(pd->paxb_base[pd->phy_count])) {
			dev_err(dev, "unable to map PCIe core %d registers\n",
					pd->phy_count);
			of_node_put(child);
			return PTR_ERR(pd->paxb_base[pd->phy_count]);
		}

		pd->phy_count++;
	}


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
	ret = device_create_file(dev, &dev_attr_test_gen);
	if (ret < 0)
		goto destroy_slot_num;
	ret = device_create_file(dev, &dev_attr_err_count);
	if (ret < 0)
		goto destroy_test_gen;
	ret = device_create_file(dev, &dev_attr_test_start);
	if (ret < 0)
		goto destroy_err_count;
	ret = device_create_file(dev, &dev_attr_lane);
	if (ret < 0)
		goto destroy_test_start;
	ret = device_create_file(dev, &dev_attr_pmi_read);
	if (ret < 0)
		goto destroy_lane;
	ret = device_create_file(dev, &dev_attr_pmi_write);
	if (ret < 0)
		goto destroy_pmi_read;

	mutex_init(&pd->test_lock);
	pd->test_retries = 0;
	pd->test_start = 0;
	pd->pcie_mode = PCIE_MODE_DEFAULT;
	pd->slot_num = 0;
	strncpy(pd->test_gen, "gen2", GEN_STR_LEN);
	dev_info(dev, "%d PCIe PHYs registered\n", pd->phy_count);
	return 0;

destroy_pmi_read:
	device_remove_file(dev, &dev_attr_pmi_read);
destroy_lane:
	device_remove_file(dev, &dev_attr_lane);
destroy_test_start:
	device_remove_file(dev, &dev_attr_test_start);
destroy_err_count:
	device_remove_file(dev, &dev_attr_err_count);
destroy_test_gen:
	device_remove_file(dev, &dev_attr_test_gen);
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

static struct platform_driver stingray_pcie_prbs_driver = {
	.driver = {
		.name = "stingray-pcie-prbs",
		.of_match_table = stingray_pcie_phy_of_match,
	},
	.probe = stingray_pcie_phy_probe,
};
module_platform_driver(stingray_pcie_prbs_driver);

MODULE_DESCRIPTION("Broadcom Stingray PCIe PHY PRBS test driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Broadcom");
