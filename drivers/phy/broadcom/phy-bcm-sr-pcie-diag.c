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
#include <linux/slab.h>
#include <linux/string.h>

#include "phy-bcm-sr-eye-ref.h"

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

/* microcode related fields */
#define DSC_A_UC_CTRL			0xd00d
#define DSC_SUPP_INFO_SHIFT		8
#define DSC_READY_SHIFT			7
#define DSC_READY			BIT(DSC_READY_SHIFT)
#define DSC_ERR_SHIFT			6
#define DSC_ERR				BIT(DSC_ERR_SHIFT)

#define DSC_SCRATCH			0xd00e

#define DSC_E_CTRL			0xd040
#define DSC_P1_THRESH_SEL_SHIFT		3
#define DSC_P1_THRESH_SEL		BIT(DSC_P1_THRESH_SEL_SHIFT)

#define RX_LOCK_STATUS			0xd0dc
#define RX_LOCK				BIT(0)

#define UC_A_AHB_CTRL0			0xd202
#define UC_AUTO_INC_RADDR_EN_SHIFT	13
#define UC_AUTO_INC_RADDR_EN		BIT(UC_AUTO_INC_RADDR_EN_SHIFT)
#define UC_AUTO_INC_WADDR_EN_SHIFT	12
#define UC_AUTO_INC_WADDR_EN		BIT(UC_AUTO_INC_WADDR_EN_SHIFT)
#define UC_RSIZE_SHIFT			4
#define UC_RSIZE_8			0
#define UC_RSIZE_16			1
#define UC_RSIZE_32			2
#define UC_WORD_SIZE			16
#define UC_WORD_MASK			(BIT(UC_WORD_SIZE) - 1)
#define UC_WSIZE_SHIFT			0

#define UC_A_AHB_WADDR_LSW		0xd204
#define UC_A_AHB_WADDR_MSW		0xd205
#define UC_A_AHB_WDATA_LSW		0xd206
#define UC_A_AHB_WDATA_MSW		0xd207
#define UC_A_AHB_RADDR_LSW		0xd208
#define UC_A_AHB_RADDR_MSW		0xd209
#define UC_A_AHB_RDATA_LSW		0xd20a
#define UC_A_AHB_RDATA_MSW		0xd20b

#define LANE_VAR_RAM_BASE		0x500
#define LANE_VAR_RAM_SIZE		0x100

#define UC_RAM_BASE			0x0100
#define UC_SIGNATURE_OFFSET		((UC_RAM_BASE) + 0x0000)
#define UC_SIGNATURE_MASK		0x00ffffff
#define UC_SIGNATURE			0x00666e49
#define UC_VERSION_SHIFT		24
#define UC_VERSION_MASK			0xff

#define UC_TRACE_LANE_MEM_SIZE_OFFSET	((UC_RAM_BASE) + 0x0008)
#define UC_DIAG_MEM_SIZE_MASK		0xffff
#define UC_TRACE_MEM_SIZE_MASK		0xffff

#define UC_OTHER_SIZE_OFFSET		((UC_RAM_BASE) + 0x000c)
#define UC_LANE_SIZE_SHIFT		0
#define UC_LANE_SIZE_MASK		0xff
#define UC_TR_MEM_DESC_WR_MASK		0x1000000
#define UC_CORE_VAR_RAM_SIZE_SHIFT	8
#define UC_CORE_VAR_RAM_SIZE_MASK	0xff
#define UC_LANE_VAR_RAM_SIZE_SHIFT	16
#define UC_LANE_VAR_RAM_SIZE_MASK	0xffff

#define UC_TRACE_MEM_PTR_OFFSET		((UC_RAM_BASE) + 0x0010)
#define UC_CORE_MEM_PTR_OFFSET		((UC_RAM_BASE) + 0x0014)
#define UC_LANE_MEM_PTR_OFFSET		((UC_RAM_BASE) + 0x001c)
#define UC_MICRO_MEM_PTR_OFFSET		((UC_RAM_BASE) + 0x0064)

#define UC_OTHER_SIZE_2_OFFSET		((UC_RAM_BASE) + 0x0060)
#define UC_MICRO_MEM_SIZE_SHIFT		4
#define UC_MICRO_MEM_SIZE_MASK		0xff
#define UC_MICRO_CNT_MASK		0xf

#define UC_DIAG_TIME_CTRL_OFFSET	0x0011
#define UC_DIAG_ERR_CTRL_OFFSET		0x0012

#define UC_DIAG_STATUS_OFFSET		0x0014
#define UC_DIAG_RDY_MASK		0x8000
#define UC_DIAG_RD_PTR_OFFSET		0x0016
#define UC_DIAG_VAR_MASK		0xff
#define UC_DIAG_WR_PTR_OFFSET		0x001c

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

#define MAX_EYE_Y			63
#define Y_START				31

#define MAX_EYE_X			64
#define X_START				(-31)
#define X_END				31
#define STRIPE_SIZE			MAX_EYE_X

#define EYE_REF_MODE			11

#define BER_MAX_SAMPLES			64
#define BER_NR_MODES			4

#define BER_MODE_POS			0
#define BER_MODE_NEG			1
#define BER_MODE_VERT			(0 << 1)
#define BER_MODE_HORZ			(1 << 1)

#define BITS_PER_NIBBLE			4
#define NIBBLE_MASK			(BIT(BITS_PER_NIBBLE) - 1)
#define BYTE_MASK			(BIT(BITS_PER_BYTE) - 1)

/* default BER sampling time */
#define BER_DFT_SAMPLING_TIME		96

/* default BER error number */
#define BER_MAX_ERR_CTRL		100

/*
 * Scale actual time down with a factor of 1.33 before sending command to
 * serdes microcode
 */
#define BER_TIME_SCALING(time)		((time) * 3 / 4)

/*
 * Scale actual error number down with a factor of 16 before sending command to
 * serdes microcode
 */
#define BER_ERR_SCALING(err)		((err) / 16)

#define MAX_BER_SAMPLING_TIME		256

#define BER_SIGNATURE			0x42455253

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

/* UC command */
enum uc_cmd {
	UC_CMD_NULL = 0,
	UC_CMD_UC_CTRL = 1,
	UC_CMD_EN_DIAG = 5,
	UC_CMD_CAPTURE_BER_START = 16,
	UC_CMD_READ_DIAG_DATA_BYTE = 17,
	UC_CMD_READ_DIAG_DATA_WORD = 18,
	UC_CMD_CAPTURE_BER_END = 19,
};

/* UC control command */
enum uc_cmd_ctrl {
	UC_CMD_CTRL_STOP_GRACEFULLY = 0,
	UC_CMD_CTRL_STOP_IMMEDIATE = 1,
	UC_CMD_CTRL_RESUME = 2
};

/* UC diag command */
enum uc_cmd_diag {
	UC_CMD_DIAG_DISABLE = 3,
	UC_CMD_DIAG_START_VSCAN_EYE = 4,
	UC_CMD_DIAG_START_HSCAN_EYE = 5,
	UC_CMD_DIAG_GET_EYE_SAMPLE = 6,
};

/* UC parameters stored in RAM */
struct uc_info {
	u32 signature;
	u32 diag_mem_ram_base;
	u32 diag_mem_ram_size;
	u32 core_var_ram_base;
	u32 core_var_ram_size;
	u32 lane_var_ram_base;
	u32 lane_var_ram_size;
	u32 trace_mem_ram_base;
	u32 trace_mem_ram_size;
	u32 micro_var_ram_base;
	u8 lane_count;
	u8 trace_mem_desc_writes;
	u8 micro_count;
	u8 micro_var_ram_size;
	u16 grp_ram_size;
	u8 version;
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
	unsigned int ber_sampling_time;
	enum pcie_modes pcie_mode;
	struct mutex test_lock;

	struct uc_info info;
};

/*
 * Following table gives information about PHYs are wired to which
 * core in given pcie RC mode.
 */
static unsigned int phy_mask[][8] = {
	/* Mode 0: 1x16(EP) */
	[PCIE_MODE0] = {0x00},
	/* Mode 1: 1x8 (EP), 1x8 (RC) */
	[PCIE_MODE1] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0},
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
static unsigned int phy_workaround_table[] = {
	/* Mode 0: 1x16(EP) */
	0x00,
	/* Mode 1: 1x8 (EP), 1x8 (RC) */
	0x80,
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

static u32 ber_mode[BER_NR_MODES] = {
	BER_MODE_HORZ | BER_MODE_NEG,
	BER_MODE_HORZ | BER_MODE_POS,
	BER_MODE_VERT | BER_MODE_NEG,
	BER_MODE_VERT | BER_MODE_POS,
};

static u32 ber_time[BER_NR_MODES][BER_MAX_SAMPLES];
static u32 ber_err[BER_NR_MODES][BER_MAX_SAMPLES];

/* for eye scan stripe storage */
static u32 stripe[MAX_EYE_Y][MAX_EYE_X];

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

	dev_dbg(pd->dev, "%s: pmi = 0x%x, data = 0x%x\n",
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
			dev_dbg(pd->dev, "%s : 0x%x = 0x%x\n",
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

static void uc_ram_write8(struct pcie_prbs_dev *pd, u32 ram_addr, u8 data)
{
	u32 paddr;
	u16 val;

	paddr = pmi_addr(UC_A_AHB_CTRL0, pd->lane);
	val = UC_RSIZE_8 << UC_WSIZE_SHIFT;
	pmi_write(pd, paddr, val);

	paddr = pmi_addr(UC_A_AHB_WADDR_MSW, pd->lane);
	pmi_write(pd, paddr, ram_addr >> UC_WORD_SIZE);
	paddr = pmi_addr(UC_A_AHB_WADDR_LSW, pd->lane);
	pmi_write(pd, paddr, ram_addr & UC_WORD_MASK);
	paddr = pmi_addr(UC_A_AHB_WDATA_LSW, pd->lane);
	pmi_write(pd, paddr, data);
}

static u8 uc_ram_read8(struct pcie_prbs_dev *pd, u32 ram_addr)
{
	u32 paddr;
	u16 val, lsw = 0;

	paddr = pmi_addr(UC_A_AHB_CTRL0, pd->lane);
	val = UC_RSIZE_8 << UC_RSIZE_SHIFT;
	pmi_write(pd, paddr, val);

	paddr = pmi_addr(UC_A_AHB_RADDR_MSW, pd->lane);
	pmi_write(pd, paddr, ram_addr >> UC_WORD_SIZE);
	paddr = pmi_addr(UC_A_AHB_RADDR_LSW, pd->lane);
	pmi_write(pd, paddr, ram_addr & UC_WORD_MASK);
	paddr = pmi_addr(UC_A_AHB_RDATA_LSW, pd->lane);
	pmi_read(pd, paddr, &lsw);

	return (lsw & 0xff);
}

static u16 uc_ram_read16(struct pcie_prbs_dev *pd, u32 ram_addr)
{
	u32 paddr;
	u16 val, lsw = 0;

	paddr = pmi_addr(UC_A_AHB_CTRL0, pd->lane);
	val = UC_RSIZE_16 << UC_RSIZE_SHIFT;
	pmi_write(pd, paddr, val);

	paddr = pmi_addr(UC_A_AHB_RADDR_MSW, pd->lane);
	pmi_write(pd, paddr, ram_addr >> UC_WORD_SIZE);
	paddr = pmi_addr(UC_A_AHB_RADDR_LSW, pd->lane);
	pmi_write(pd, paddr, ram_addr & UC_WORD_MASK);
	paddr = pmi_addr(UC_A_AHB_RDATA_LSW, pd->lane);
	pmi_read(pd, paddr, &lsw);

	return lsw;
}

static int uc_ram_read(struct pcie_prbs_dev *pd, u32 *buf, u32 ram_addr,
		       u32 len)
{
	u32 i, paddr;
	u16 val, lsw, wsw;

	if (!buf)
		return -EINVAL;

	ram_addr = ALIGN_DOWN(ram_addr, SZ_4);
	len = ALIGN_DOWN(len, SZ_4);

	paddr = pmi_addr(UC_A_AHB_CTRL0, pd->lane);
	val = UC_AUTO_INC_RADDR_EN | (UC_RSIZE_32 << UC_RSIZE_SHIFT);
	pmi_write(pd, paddr, val);

	paddr = pmi_addr(UC_A_AHB_RADDR_MSW, pd->lane);
	pmi_write(pd, paddr, ram_addr >> UC_WORD_SIZE);
	paddr = pmi_addr(UC_A_AHB_RADDR_LSW, pd->lane);
	pmi_write(pd, paddr, ram_addr & UC_WORD_MASK);

	for (i = 0; i < len; i += SZ_4, buf++) {
		paddr = pmi_addr(UC_A_AHB_RDATA_MSW, pd->lane);
		pmi_read(pd, paddr, &wsw);
		paddr = pmi_addr(UC_A_AHB_RDATA_LSW, pd->lane);
		pmi_read(pd, paddr, &lsw);
		*buf = (wsw << UC_WORD_SIZE) | lsw;
	}

	return 0;
}

#define UC_CMD_POLL 10

static int uc_cmd_ready(struct pcie_prbs_dev *pd)
{
	u32 paddr, poll;
	u16 val;
	int ret = 0;

	poll = 0;
	while (poll < UC_CMD_POLL) {
		paddr = pmi_addr(DSC_A_UC_CTRL, pd->lane);
		pmi_read(pd, paddr, &val);
		if (val & DSC_READY) {
			if (val & DSC_ERR)
				ret = -EIO;
			break;
		}
		msleep(20);
		poll++;
	}

	if (poll >= UC_CMD_POLL)
		ret = -ETIMEDOUT;

	return ret;
}

static int uc_cmd_run(struct pcie_prbs_dev *pd, u8 cmd, u8 supp)
{
	int ret;
	u32 paddr;

	ret = uc_cmd_ready(pd);
	if (ret) {
		dev_err(pd->dev,
			"uc cmd not ready, cmd=0x%02x supp=0x%0x2\n",
			cmd, supp);
		return ret;
	}

	paddr = pmi_addr(DSC_A_UC_CTRL, pd->lane);
	pmi_write(pd, paddr, cmd | (supp << DSC_SUPP_INFO_SHIFT));

	ret = uc_cmd_ready(pd);
	if (ret) {
		dev_err(pd->dev,
			"uc cmd failed, cmd=0x%02x supp=0x%0x2\n",
			cmd, supp);
		return ret;
	}

	return 0;
}

static bool uc_rx_lock(struct pcie_prbs_dev *pd)
{
	u32 paddr;
	u16 data;

	paddr = pmi_addr(RX_LOCK_STATUS, pd->lane);
	pmi_read(pd, paddr, &data);

	return !!(data & RX_LOCK);
}

static int uc_get_info(struct pcie_prbs_dev *pd)
{
	struct uc_info *info = &pd->info;
	u32 val;

	memset(info, 0, sizeof(*info));

	/* validate signature from uc */
	uc_ram_read(pd, &val, UC_SIGNATURE_OFFSET, SZ_4);
	info->signature = val;
	if ((info->signature & UC_SIGNATURE_MASK) != UC_SIGNATURE)
		return -ENODEV;

	info->version = (val >> UC_VERSION_SHIFT) & UC_VERSION_MASK;

	uc_ram_read(pd, &val, UC_OTHER_SIZE_OFFSET, SZ_4);
	info->lane_count = (val >> UC_LANE_SIZE_SHIFT) & UC_LANE_SIZE_MASK;
	info->trace_mem_desc_writes = !!(val & UC_TR_MEM_DESC_WR_MASK);
	info->core_var_ram_size = (val >> UC_CORE_VAR_RAM_SIZE_SHIFT) &
				  UC_CORE_VAR_RAM_SIZE_MASK;

	uc_ram_read(pd, &val, UC_TRACE_LANE_MEM_SIZE_OFFSET, SZ_4);
	info->lane_var_ram_size = (val >> UC_LANE_VAR_RAM_SIZE_SHIFT) &
				  UC_LANE_VAR_RAM_SIZE_MASK;
	if (info->lane_var_ram_size != LANE_VAR_RAM_SIZE)
		return -EFAULT;

	info->diag_mem_ram_size = (val & UC_DIAG_MEM_SIZE_MASK) /
				  info->lane_count;
	info->trace_mem_ram_size = val & UC_TRACE_MEM_SIZE_MASK;

	uc_ram_read(pd, &val, UC_TRACE_MEM_PTR_OFFSET, SZ_4);
	info->diag_mem_ram_base = val;
	info->trace_mem_ram_base = info->diag_mem_ram_base;

	uc_ram_read(pd, &val, UC_CORE_MEM_PTR_OFFSET, SZ_4);
	info->core_var_ram_base = val;

	uc_ram_read(pd, &val, UC_MICRO_MEM_PTR_OFFSET, SZ_4);
	info->micro_var_ram_base = val;

	uc_ram_read(pd, &val, UC_OTHER_SIZE_2_OFFSET, SZ_4);
	info->micro_var_ram_size = (val >> UC_MICRO_MEM_SIZE_SHIFT) &
				   UC_MICRO_MEM_SIZE_MASK;

	/* for uc version below 0x34, micro code count is always 1  */
	if (info->version < 0x34)
		info->micro_count = 1;
	else
		info->micro_count = val & UC_MICRO_CNT_MASK;

	uc_ram_read(pd, &val, UC_LANE_MEM_PTR_OFFSET, SZ_4);
	info->lane_var_ram_base = val;

	pr_info("Microcode info for Lane[%u]:", pd->lane);
	pr_info("Signature = 0x%08x\n", info->signature);
	pr_info("Diag MEM RAM base = 0x%08x\n", info->diag_mem_ram_base);
	pr_info("Diag MEM RAM size = 0x%08x\n", info->diag_mem_ram_size);
	pr_info("Core VAR RAM base = 0x%08x\n", info->core_var_ram_base);
	pr_info("Core VAR RAM size = 0x%08x\n", info->core_var_ram_size);
	pr_info("Lane VAR RAM base = 0x%08x\n", info->lane_var_ram_base);
	pr_info("Lane VAR RAM size = 0x%08x\n", info->lane_var_ram_size);
	pr_info("Trace MEM RAM base = 0x%08x\n", info->trace_mem_ram_base);
	pr_info("Trace MEM RAM size = 0x%08x\n", info->trace_mem_ram_size);
	pr_info("Micro VAR RAM base = 0x%08x\n", info->micro_var_ram_base);
	pr_info("Lane Count = %u\n", info->lane_count);
	pr_info("Trace MEM descending writes = %u\n",
		info->trace_mem_desc_writes);
	pr_info("Micro Count = %u\n", info->micro_count);
	pr_info("Micro VAR RAM size =0x%08x\n", info->micro_var_ram_size);
	pr_info("GRP RAM size = 0x%08x\n", info->grp_ram_size);
	pr_info("Version = 0x%02x\n", info->version);

	return 0;
}

static u8 uc_diag_var_read8(struct pcie_prbs_dev *pd, u32 offset)
{
	struct uc_info *info = &pd->info;
	u32 addr;

	addr = info->lane_var_ram_base + (pd->lane * info->lane_var_ram_size) +
	       offset;

	return uc_ram_read8(pd, addr);
}

static u16 uc_diag_var_read16(struct pcie_prbs_dev *pd, u32 offset)
{
	struct uc_info *info = &pd->info;
	u32 addr;

	addr = info->lane_var_ram_base + (pd->lane * info->lane_var_ram_size) +
	       offset;

	return uc_ram_read16(pd, addr);
}

static void uc_diag_var_write8(struct pcie_prbs_dev *pd, u32 offset, u8 data)
{
	struct uc_info *info = &pd->info;
	u32 addr;

	addr = info->lane_var_ram_base + (pd->lane * info->lane_var_ram_size) +
	       offset;

	uc_ram_write8(pd, addr, data);
}

static u8 uc_diag_rd_ptr_read(struct pcie_prbs_dev *pd)
{
	return uc_diag_var_read8(pd, UC_DIAG_RD_PTR_OFFSET);
}

static void uc_diag_rd_ptr_write(struct pcie_prbs_dev *pd, u8 data)
{
	uc_diag_var_write8(pd, UC_DIAG_RD_PTR_OFFSET, data);
}

static u8 uc_diag_wr_ptr_read(struct pcie_prbs_dev *pd)
{
	return uc_diag_var_read8(pd, UC_DIAG_WR_PTR_OFFSET);
}

static void uc_diag_core_var_write8(struct pcie_prbs_dev *pd, u32 offset,
				    u8 data)
{
	struct uc_info *info = &pd->info;
	u32 addr = info->core_var_ram_base + offset;

	uc_ram_write8(pd, addr, data);
}

static u16 uc_diag_status_read(struct pcie_prbs_dev *pd)
{
	return uc_diag_var_read16(pd, UC_DIAG_STATUS_OFFSET);
}

static int uc_diag_poll_diag_done(struct pcie_prbs_dev *pd,
				  unsigned long timeout_ms)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);
	u16 status;

	do {
		status = uc_diag_status_read(pd);
		status &= UC_DIAG_RDY_MASK;
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;

		cpu_relax();
		cond_resched();
	} while (!status);

	return 0;
}

static u32 float12_to_u32(u8 byte, u8 multi)
{
	return ((u32)byte) << multi;
}

#define BER_NR_SAMPLES_PER_READ     3
#define BER_NR_SAMPLES_READ_MASK    0xff
static int uc_ber_read_scan_data(struct pcie_prbs_dev *pd,
				 unsigned int data_idx)
{
	u16 i, sample_cnt, val;
	u32 paddr;
	u8 time_byte, time_multi, prbs_byte, prbs_multi;
	int ret;

	sample_cnt = uc_diag_status_read(pd);
	if ((sample_cnt & UC_DIAG_RDY_MASK) != UC_DIAG_RDY_MASK) {
		dev_err(pd->dev, "BER sample is not ready\n");
		return -EFAULT;
	}

	memset(ber_time[data_idx], 0, sizeof(u32) * BER_MAX_SAMPLES);
	memset(ber_err[data_idx], 0, sizeof(u32) * BER_MAX_SAMPLES);
	sample_cnt = (sample_cnt & BER_NR_SAMPLES_READ_MASK) /
		     BER_NR_SAMPLES_PER_READ;
	for (i = 0; i < sample_cnt; i++) {
		ret = uc_cmd_run(pd, UC_CMD_READ_DIAG_DATA_WORD, 0);
		paddr = pmi_addr(DSC_SCRATCH, pd->lane);
		pmi_read(pd, paddr, &val);

		time_byte = (u8)(val >> BITS_PER_BYTE);
		prbs_multi = (u8)(val & NIBBLE_MASK);
		time_multi = (u8)val >> BITS_PER_NIBBLE;

		ret = uc_cmd_run(pd, UC_CMD_READ_DIAG_DATA_BYTE, 0);
		pmi_read(pd, paddr, &val);
		prbs_byte = val & BYTE_MASK;

		ber_time[data_idx][i] = float12_to_u32(time_byte, time_multi);
		ber_err[data_idx][i] = float12_to_u32(prbs_byte, prbs_multi);
	}

	return 0;
}

static void uc_diag_max_time_ctrl(struct pcie_prbs_dev *pd, u8 time)
{
	uc_diag_core_var_write8(pd, UC_DIAG_TIME_CTRL_OFFSET, time);
}

static void uc_diag_max_err_ctrl(struct pcie_prbs_dev *pd, u8 err)
{
	uc_diag_core_var_write8(pd, UC_DIAG_ERR_CTRL_OFFSET, err);
}

#define UC_DIAG_POLL 1000

static int uc_poll_diag_data(struct pcie_prbs_dev *pd, u8 rdp, u8 len)
{
	struct uc_info *info = &pd->info;
	u32 lane_diag_size = info->diag_mem_ram_size;
	unsigned int poll;

	/*
	 * Do not support the case when the len is larger than half of the
	 * lane_diag_size. In such case, the read pointer cannot be updated
	 * fast enough
	 */
	if (len > (lane_diag_size / 2)) {
		dev_err(pd->dev, "Excessive len %u detected\n", len);
		return -EINVAL;
	}

	/* wait until diag data is ready to be read */
	poll = 0;
	while (poll < UC_DIAG_POLL) {
		u8 wrp, full_count;

		wrp = uc_diag_wr_ptr_read(pd);
		if (wrp >= rdp)
			full_count = wrp - rdp;
		else
			full_count = (u16)wrp + lane_diag_size - rdp;
		if (full_count >= len)
			break;

		msleep(20);
	}

	if (poll >= UC_DIAG_POLL) {
		dev_err(pd->dev, "UC diag poll data timed out\n");
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Magic calculation ported from code from the PCIe Serdes team
 */
static s16 ladder_setting_to_mV(s8 ctrl, bool range_250)
{
	u16 absv = abs(ctrl);
	s16 nlmv, nlv;

	nlv = 25 * absv;
	if (absv > 22)
		nlv += (absv - 22) * 25;

	if (range_250)
		nlmv = (nlv + 2) / 4;
	else
		nlmv = (nlv * 3 + 10) / 20;
	return ((ctrl >= 0) ? nlmv : -nlmv);
}

/*
 * Magic calculation ported from code from the PCIe Serdes team
 */
static u32 float8_to_u32(u8 float8)
{
	u32 x;

	if (float8 == 0)
		return 0;

	x = (float8 >> 5) + 8;
	if ((float8 & 0x1f) < 3)
		return (x >> (3 - (float8 & 0x1f)));

	return (x << ((float8 & 0x1f) - 3));
}

static void uc_eye_scan_convert(u32 *buf, u8 len, u16 data)
{
	unsigned int i;

	for (i = 0; i < len; i++)
		buf[i] = float8_to_u32((data >> (8 * i)) & 0xff);
}

static void uc_eye_display_header(void)
{
	pr_info("\n");
	pr_info(" Each character N represents approximate error rate 1e-N at that location\n");
	pr_info("  UI/64  : -30  -25  -20  -15  -10  -5    0    5    10   15   20   25   30\n");
	pr_info("         : -|----|----|----|----|----|----|----|----|----|----|----|----|-\n");
}

static void uc_eye_display_footer(void)
{
	pr_info("         : -|----|----|----|----|----|----|----|----|----|----|----|----|-");
	pr_info("  UI/64  : -30  -25  -20  -15  -10  -5    0    5    10   15   20   25   30");
	pr_info("\n");
}

#define NR_LIMITS	7
#define NR_CR		5
static void uc_eye_display_stripe(struct pcie_prbs_dev *pd, uint32_t *buf, s8 y)
{
	const u32 limits[NR_LIMITS] = {917504, 91750, 9175, 917, 91, 9, 1};
	s8 x, i;
	s16 level;
	u32 paddr;
	u16 val;
	bool p1_select;

	paddr = pmi_addr(DSC_E_CTRL, pd->lane);
	pmi_read(pd, paddr, &val);
	p1_select = !!(val & DSC_P1_THRESH_SEL);
	level = ladder_setting_to_mV(y, p1_select);

	printk(KERN_INFO "%6dmV : ", level);

	for (x = X_START; x <= X_END; x++) {
		for (i = 0; i < NR_LIMITS; i++) {
			if (buf[x + abs(X_START)] >= limits[i]) {
				printk(KERN_CONT "%c", '0' + i + 1);
				break;
			}
		}

		if (i == NR_LIMITS) {
			if ((x % NR_CR) == 0 && (y % NR_CR) == 0)
				printk(KERN_CONT "+");
			else if ((x % NR_CR) != 0 && (y % NR_CR) == 0)
				printk(KERN_CONT "-");
			else if ((x % NR_CR) == 0 && (y % NR_CR) != 0)
				printk(KERN_CONT ":");
			else
				printk(KERN_CONT " ");
		}
	}
}

static void uc_eye_display_ref(void)
{
	unsigned int i;

	printk(KERN_INFO "\n");

	for (i = 0; i < sizeof(pcie_eye_ref); i++)
		printk(KERN_CONT "%c", pcie_eye_ref[i]);

	printk(KERN_INFO"\n");
}

static int uc_blk_read_generic_ram(struct pcie_prbs_dev *pd, u32 blk_addr,
				   u16 blk_size, u16 offset, u16 cnt,
				   u32 *buf)
{
	u32 val, read_val = 0;
	u8 defecit = 0;
	u32 paddr, addr = blk_addr + offset;
	u16 lsw;
	unsigned int lane = pd->lane;
	unsigned int word_size = sizeof(u16);

	if (offset >= blk_size)
		return -EINVAL;

	while (cnt > 0) {
		u16 bytes_left = blk_size - offset;
		u16 blk_cnt = min(cnt, bytes_left);

		cnt -= blk_cnt;

		/* set up for word reads */
		paddr = pmi_addr(UC_A_AHB_CTRL0, lane);
		val = UC_AUTO_INC_RADDR_EN | (UC_RSIZE_16 << UC_RSIZE_SHIFT);
		pmi_write(pd, paddr, val);

		paddr = pmi_addr(UC_A_AHB_RADDR_MSW, lane);
		pmi_write(pd, paddr, addr >> UC_WORD_SIZE);
		paddr = pmi_addr(UC_A_AHB_RADDR_LSW, lane);
		pmi_write(pd, paddr, addr & UC_WORD_MASK);

		/* read the leading byte, if starting at an odd address */
		if ((addr & 1)) {
			paddr = pmi_addr(UC_A_AHB_RDATA_LSW, lane);
			pmi_read(pd, paddr, &lsw);

			read_val |= ((lsw >> 8) << defecit);
			if (defecit == 8) {
				uc_eye_scan_convert(buf, word_size,
						    (u16)read_val);
				read_val = 0;
				buf += word_size;
			}

			defecit ^= 8;
			blk_cnt--;
		}

		/* read the whole word */
		while (blk_cnt >= 2) {
			paddr = pmi_addr(UC_A_AHB_RDATA_LSW, lane);
			pmi_read(pd, paddr, &lsw);
			read_val |= (lsw << defecit);
			uc_eye_scan_convert(buf, word_size, (u16)read_val);
			read_val >>= 16;
			buf += word_size;
			blk_cnt -= word_size;
		}

		/* read the trailing byte */
		if (blk_cnt > 0) {
			paddr = pmi_addr(UC_A_AHB_RDATA_LSW, lane);
			pmi_read(pd, paddr, &lsw);
			read_val |= ((lsw & 0xff) << defecit);
			if (defecit == 8) {
				uc_eye_scan_convert(buf, word_size,
						    (u16)read_val);
				read_val = 0;
				buf += word_size;
			}
			defecit ^= 8;
			blk_cnt--;
		}
		addr = blk_addr;
		offset = 0;
	}

	/* when the last odd byte is left behind */
	if (defecit > 0)
		uc_eye_scan_convert(buf, 1, (u16)read_val);

	return 0;
}

static int uc_eye_scan_start(struct pcie_prbs_dev *pd, bool vertical)
{
	enum uc_cmd_diag diag;

	diag = vertical ?
	       UC_CMD_DIAG_START_VSCAN_EYE : UC_CMD_DIAG_START_HSCAN_EYE;

	return uc_cmd_run(pd, UC_CMD_EN_DIAG, diag);
}

static int uc_eye_scan_done(struct pcie_prbs_dev *pd)
{
	return uc_cmd_run(pd, UC_CMD_EN_DIAG, UC_CMD_DIAG_DISABLE);
}

static int uc_eye_scan_stripe(struct pcie_prbs_dev *pd, u32 *buf)
{
	struct uc_info *info = &pd->info;
	u32 lane_diag_base;
	unsigned int lane = pd->lane;
	u8 rdp;
	int ret;

	lane_diag_base = info->diag_mem_ram_base +
			 ((lane % info->lane_count) * info->diag_mem_ram_size);

	/* obtain the read pointer from uc and wait for data to be ready */
	rdp = uc_diag_rd_ptr_read(pd);
	ret = uc_poll_diag_data(pd, rdp, STRIPE_SIZE);
	if (ret)
		return ret;

	/* now reading eye diagram data into buffer */
	ret = uc_blk_read_generic_ram(pd, lane_diag_base,
				      info->diag_mem_ram_size,
				      rdp, STRIPE_SIZE, buf);
	if (ret)
		return ret;

	/* update the read pointer and write back to uc */
	rdp = (rdp + STRIPE_SIZE) % info->diag_mem_ram_size;
	uc_diag_rd_ptr_write(pd, rdp);

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

static ssize_t ber_sampling_time_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	ssize_t ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *pd = platform_get_drvdata(pdev);

	mutex_lock(&pd->test_lock);
	ret = sprintf(buf, "%u\n", pd->ber_sampling_time);
	mutex_unlock(&pd->test_lock);

	return ret;
}

static ssize_t ber_sampling_time_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	ssize_t ret;
	unsigned int ber_sampling_time;
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *pd = platform_get_drvdata(pdev);

	ret = kstrtoint(buf, 0, &ber_sampling_time);
	if (ret)
		return ret;

	if (ber_sampling_time > MAX_BER_SAMPLING_TIME)
		return -EINVAL;

	mutex_lock(&pd->test_lock);
	pd->ber_sampling_time = ber_sampling_time;
	mutex_unlock(&pd->test_lock);

	return strnlen(buf, count);
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
	dev_info(dev, "lane: %u pmi_addr: 0x%08x val: 0x%04x\n",
		 lane, addr, data);
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

static ssize_t uc_ram_store(struct device *dev,
			    struct device_attribute *attr, const char *buf,
			    size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *pd = platform_get_drvdata(pdev);
	int ret;
	u32 i, addr, len, lane;
	u32 *data;

	ret = sscanf(buf, "%x %x %x", &addr, &len, &lane);
	if (ret != 2 && ret != 3)
		return -EINVAL;
	if (len < SZ_4)
		return -EINVAL;
	if (ret == 2)
		lane = 0;

	addr = ALIGN_DOWN(addr, SZ_4);
	len = ALIGN_DOWN(len, SZ_4);
	pd->lane = lane;

	data = kzalloc(len, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mutex_lock(&pd->test_lock);
	ret = uc_ram_read(pd, data, addr, len);
	if (ret) {
		mutex_unlock(&pd->test_lock);
		kfree(data);
		return ret;
	}

	pr_info("Dump of ucode RAM starting from Lane[%u]...\n\n", lane);
	for (i = 0; i < len; i += SZ_4) {
		pr_info("[0x%08x] 0x%08x\n", addr + i, *data);
		data++;
	}
	pr_info("\nDump of ucode RAM finished\n");

	mutex_unlock(&pd->test_lock);
	kfree(data);
	return count;
}

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *pd = platform_get_drvdata(pdev);
	unsigned int reset;

	if (kstrtouint(buf, 0, &reset) != 0)
		return -EINVAL;

	mutex_lock(&pd->test_lock);
	if (reset)
		iproc_pcie_assert_reset(pd->paxb_base[pd->slot_num]);
	else
		iproc_pcie_release_reset(pd->paxb_base[pd->slot_num]);
	mutex_unlock(&pd->test_lock);

	return count;
}

static ssize_t eye_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *pd = platform_get_drvdata(pdev);
	unsigned int phy, lane;
	int ret = 0, i;
	s8 y;
	u32 pipemux;
	enum uc_cmd_ctrl ctrl;

	ret = sscanf(buf, "%u %u", &phy, &lane);
	if (ret != 2)
		return -EINVAL;

	if (lane >= NR_LANES_PER_PHY)
		return -EINVAL;

	/* display reference frame and return immediately */
	if (phy == EYE_REF_MODE) {
		uc_eye_display_ref();
		return count;
	}

	pipemux = pcie_pipemux_strap_read(pd);
	if (pd->pcie_mode == PCIE_MODE_DEFAULT) {
		pipemux = pcie_pipemux_strap_read(pd);
		dev_info(dev, "PIPEMUX from strap 0x%x\n", pipemux);
	} else if (pipemux != pd->pcie_mode) {
		dev_info(dev, "Override PIPEMUX from 0x%x to 0x%x\n",
			 pipemux, pd->pcie_mode);
		pipemux = pd->pcie_mode;
		writel(pipemux, pd->pcie_ss_base + PCIE_PIPEMUX_CFG_OFFSET);
	}

	if (!((phy_mask[pipemux][pd->slot_num]) & BIT(phy)))
		return -EINVAL;

	mutex_lock(&pd->test_lock);
	pd->lane = lane;

	connect_pcie_core_to_phy(pd, phy);
	iproc_pcie_assert_reset(pd->paxb_base[pd->slot_num]);
	iproc_pcie_release_reset(pd->paxb_base[pd->slot_num]);

	/* populate uc information */
	ret = uc_get_info(pd);
	if (ret) {
		dev_err(dev, "unable to get uc info\n");
		goto err;
	}

	/* check if RX is locked */
	if (uc_rx_lock(pd))
		ctrl = UC_CMD_CTRL_STOP_GRACEFULLY;
	else
		ctrl = UC_CMD_CTRL_STOP_IMMEDIATE;

	/* stop uc before diag test */
	ret = uc_cmd_run(pd, UC_CMD_UC_CTRL, ctrl);
	if (ret) {
		dev_err(dev, "unable to stop uc\n");
		goto err;
	}

	uc_eye_display_header();

	ret = uc_eye_scan_start(pd, false);
	if (ret) {
		dev_err(dev, "unable to start eye scan\n");
		goto err_resume_uc;
	}

	for (i = 0, y = Y_START; i < MAX_EYE_Y; i++, y--) {
		memset(&stripe[i][0], 0, MAX_EYE_X);
		ret = uc_eye_scan_stripe(pd, &stripe[i][0]);
		if (ret) {
			dev_err(dev, "eye scan failed\n");
			goto err_stop_eye_scan;
		}
		uc_eye_display_stripe(pd, &stripe[i][0], y);
	}

	uc_eye_display_footer();

err_stop_eye_scan:
	uc_eye_scan_done(pd);
err_resume_uc:
	uc_cmd_run(pd, UC_CMD_UC_CTRL, UC_CMD_CTRL_RESUME);
err:
	mutex_unlock(&pd->test_lock);
	return ret < 0 ? ret : count;
}

static ssize_t ber_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *pd = platform_get_drvdata(pdev);
	unsigned int i, phy, lane;
	int ret = 0;
	u32 pipemux;
	enum uc_cmd_ctrl ctrl;

	ret = sscanf(buf, "%u %u", &phy, &lane);
	if (ret != 2)
		return -EINVAL;

	if (lane >= NR_LANES_PER_PHY)
		return -EINVAL;

	pipemux = pcie_pipemux_strap_read(pd);
	if (pd->pcie_mode == PCIE_MODE_DEFAULT) {
		pipemux = pcie_pipemux_strap_read(pd);
		dev_info(dev, "PIPEMUX from strap 0x%x\n", pipemux);
	} else if (pipemux != pd->pcie_mode) {
		dev_info(dev, "Override PIPEMUX from 0x%x to 0x%x\n",
			 pipemux, pd->pcie_mode);
		pipemux = pd->pcie_mode;
		writel(pipemux, pd->pcie_ss_base + PCIE_PIPEMUX_CFG_OFFSET);
	}

	if (!((phy_mask[pipemux][pd->slot_num]) & BIT(phy)))
		return -EINVAL;

	mutex_lock(&pd->test_lock);
	pd->lane = lane;

	connect_pcie_core_to_phy(pd, phy);
	iproc_pcie_assert_reset(pd->paxb_base[pd->slot_num]);
	iproc_pcie_release_reset(pd->paxb_base[pd->slot_num]);

	/* populate uc information */
	ret = uc_get_info(pd);
	if (ret) {
		dev_err(dev, "unable to get uc info\n");
		goto err;
	}

	/* check if RX is locked */
	if (uc_rx_lock(pd))
		ctrl = UC_CMD_CTRL_STOP_GRACEFULLY;
	else
		ctrl = UC_CMD_CTRL_STOP_IMMEDIATE;

	/* stop uc before diag test */
	ret = uc_cmd_run(pd, UC_CMD_UC_CTRL, ctrl);
	if (ret) {
		dev_err(dev, "unable to stop uc\n");
		goto err;
	}

	pr_info("\nTrying to extrapolate for BER at 1e-12\n");
	pr_info("This may take several minutes...\n");

	for (i = 0; i < BER_NR_MODES; i++) {
		pr_info("BER mode 0x%x\n", ber_mode[i]);

		/* configure BER time and error parameters */
		uc_diag_max_time_ctrl(pd,
				      BER_TIME_SCALING(pd->ber_sampling_time));
		uc_diag_max_err_ctrl(pd,
				     BER_ERR_SCALING(BER_MAX_ERR_CTRL));

		/* start BER test */
		ret = uc_cmd_run(pd, UC_CMD_CAPTURE_BER_START, ber_mode[i]);
		if (ret) {
			dev_err(dev, "unable to start BER\n");
			goto err_resume_uc;
		}

		/*
		 * Wait up to 8 times of BER sampling time for BER
		 * extrapolation
		 */
		ret = uc_diag_poll_diag_done(pd,
					     8 * MSEC_PER_SEC *
					     pd->ber_sampling_time);
		if (ret) {
			dev_err(dev, "diag poll failed\n");
			goto err_resume_uc;
		}

		/* read BER results */
		ret = uc_ber_read_scan_data(pd, i);
		if (ret) {
			dev_err(dev, "unable to read BER data\n");
			goto err_resume_uc;
		}

		/* stop BER test */
		ret = uc_cmd_run(pd, UC_CMD_CAPTURE_BER_END, 0x0);
		if (ret) {
			dev_err(dev, "unable to start BER\n");
			goto err_resume_uc;
		}
	}

	pr_info("BER extrapolation done\n");

err_resume_uc:
	uc_cmd_run(pd, UC_CMD_UC_CTRL, UC_CMD_CTRL_RESUME);
err:
	mutex_unlock(&pd->test_lock);
	return ret < 0 ? ret : count;
}

static ssize_t ber_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcie_prbs_dev *pd = platform_get_drvdata(pdev);
	ssize_t count = 0;
	unsigned int i, j;

	mutex_lock(&pd->test_lock);
	for (i = 0; i < BER_NR_MODES; i++) {
		count += sprintf(&buf[count], "%u ", BER_SIGNATURE);
		count += sprintf(&buf[count], "%u ", ber_mode[i]);
		for (j = 0; j < BER_MAX_SAMPLES; j++)
			count += sprintf(&buf[count], "%u ", ber_time[i][j]);
		for (j = 0; j < BER_MAX_SAMPLES; j++)
			count += sprintf(&buf[count], "%u ", ber_err[i][j]);
	}
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
static DEVICE_ATTR_RW(ber_sampling_time);
static DEVICE_ATTR_WO(pmi_read);
static DEVICE_ATTR_WO(pmi_write);
static DEVICE_ATTR_WO(uc_ram);
static DEVICE_ATTR_WO(reset);
static DEVICE_ATTR_WO(eye);
static DEVICE_ATTR_RW(ber);

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
	ret = device_create_file(dev, &dev_attr_ber_sampling_time);
	if (ret < 0)
		goto destroy_lane;
	ret = device_create_file(dev, &dev_attr_pmi_read);
	if (ret < 0)
		goto destroy_ber_sampling_time;
	ret = device_create_file(dev, &dev_attr_pmi_write);
	if (ret < 0)
		goto destroy_pmi_read;
	ret = device_create_file(dev, &dev_attr_uc_ram);
	if (ret < 0)
		goto destroy_pmi_write;
	ret = device_create_file(dev, &dev_attr_reset);
	if (ret < 0)
		goto destroy_uc_ram;
	ret = device_create_file(dev, &dev_attr_eye);
	if (ret < 0)
		goto destroy_reset;
	ret = device_create_file(dev, &dev_attr_ber);
	if (ret < 0)
		goto destroy_eye;

	mutex_init(&pd->test_lock);
	pd->test_retries = 0;
	pd->test_start = 0;
	pd->pcie_mode = PCIE_MODE_DEFAULT;
	pd->slot_num = 0;
	pd->ber_sampling_time = BER_DFT_SAMPLING_TIME;
	strncpy(pd->test_gen, "gen2", GEN_STR_LEN);
	dev_info(dev, "%d PCIe PHYs registered\n", pd->phy_count);
	return 0;

destroy_eye:
	device_remove_file(dev, &dev_attr_eye);
destroy_reset:
	device_remove_file(dev, &dev_attr_reset);
destroy_uc_ram:
	device_remove_file(dev, &dev_attr_uc_ram);
destroy_pmi_write:
	device_remove_file(dev, &dev_attr_pmi_write);
destroy_pmi_read:
	device_remove_file(dev, &dev_attr_pmi_read);
destroy_ber_sampling_time:
	device_remove_file(dev, &dev_attr_ber_sampling_time);
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
