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

#include <asm/byteorder.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "bcm_iproc_dsi_lowlevel.h"

#define DSI_ONE_MEGA			(1000 * 1000)

/*
 * The following two macros create a "combine" routine which can be nested.
 * Without the nesting you can't call BRCM_CONCAT(BRCM_CONCAT(x,y),z)
 */
#define BRCM_CONCAT(a, b) BRCM_CONCATX(a, b)
#define BRCM_CONCATX(a, b) a ## b

/*
 * These are used to combine names for the actual read and write macros.
 */

#define BRCM_FIELDNAME(r, f)	BRCM_CONCAT(BRCM_CONCAT(r, _), f)
#define BRCM_REGOFS(r)		BRCM_CONCAT(r, _OFFSET)
#define BRCM_REGTYPE(r)		BRCM_CONCAT(r, _TYPE)
#define BRCM_REGADDR(b, r)	((b) + (BRCM_REGOFS(r)))
#define BRCM_FIELDMASK(r, f)	(BRCM_CONCAT(BRCM_FIELDNAME(r, f), _MASK))
#define BRCM_FIELDSHIFT(r, f)	(BRCM_CONCAT(BRCM_FIELDNAME(r, f), _SHIFT))

/*
 * The following macros read and write registers or bit fields.
 *
 * b is the base address name
 * r is the register name
 * f is the field name
 * d is the data to write.
 *
 * Reserved bits handling is enforced by the macros:
 * 1.  Reserved bit must be written to 0.
 * 2.  Reserved bits are undefined when read, so masked off to zero.
 */

#define BRCM_READ_REG(b, r)	(((readl(BRCM_REGADDR(b, r))) \
			& ~(BRCM_CONCAT(r, _RESERVED_MASK))))
#define BRCM_WRITE_REG(b, r, d)		\
	(dsi_hw_write_reg(((d) & ~(BRCM_CONCAT(r, _RESERVED_MASK))),\
			   BRCM_REGADDR(b, r)))
#define BRCM_WRITE_REG_FIELD(b, r, f, d)	(BRCM_WRITE_REG(b, r,\
		((((d) << BRCM_FIELDSHIFT(r, f)) & BRCM_FIELDMASK(r, f)) | \
		 (BRCM_READ_REG(b, r) & (~BRCM_FIELDMASK(r, f))))))

struct hw_errata_reg {
	bool errata;
	void __iomem *addr_start;
	void __iomem *addr_end;
};

struct dsi_hw {
	void __iomem *base_addr;
	int dlcount;
	struct hw_errata_reg hw_errata;
};

/* NUMBER OF DATA LINES SUPPORTED */
#define DSI_DL_COUNT		    2

static struct dsi_hw dsi_dev;

static inline void dsi_hw_write_reg(uint32_t b, void *r)
{
	struct hw_errata_reg *hw_errata = &dsi_dev.hw_errata;

	writel(b, r);
	if ((hw_errata->errata) && (r >= hw_errata->addr_start)
				&& (r <= hw_errata->addr_end))
		readl(r);
}

#define DSI_REG_FIELD_SET(r, f, d)	\
	(((BRCM_REGTYPE(r))(d) << BRCM_FIELDSHIFT(r, f)) &	\
	 BRCM_FIELDMASK(r, f))

#define DSI_REG_WRITE_MASKED(b, r, m, d)	\
	(BRCM_WRITE_REG(b, r, (BRCM_READ_REG(b, r) & (~m)) | d))

/* DSI COMMAND TYPE */
#define CMND_CTRL_CMND_PKT	    0
#define CMND_CTRL_TRIG		    2
#define CMND_CTRL_BTA		    3

/* DSI PACKET SOURCE */
#define DSI_PKT_SRC_CMND_FIFO	    0
#define DSI_PKT_SRC_DE0		    1
#define DSI_PKT_SRC_DE1		    2

/* record has MAX value set */
#define DSI_C_HAS_MAX	     1
/* record MIN value is MAX of 2 values */
#define DSI_C_MIN_MAX_OF_2   2

/* counts in HS Bit Clk */
#define DSI_C_TIME_HS	     1
/* counts in ESC CLKs */
#define DSI_C_TIME_ESC	     2

struct dsi_counter {
	unsigned int type;
	unsigned int time_base;
	unsigned int mode;
	unsigned int time_lpx;
	unsigned int time_min1_ns;
	unsigned int time_min1_ui;
	unsigned int time_min2_ns;
	unsigned int time_min2_ui;
	unsigned int time_max_ns;
	unsigned int time_max_ui;
	unsigned int counter_min;
	unsigned int counter_max;
	unsigned int counter_step;
	unsigned int counter_offs;
	unsigned int counter;
	unsigned int period;
};

/* DSI Core Timing Registers */
enum dsi_hw_timing_c {
	DSI_C_ESC2LP_RATIO = 0,
	DSI_C_HS_INIT,
	DSI_C_HS_WAKEUP,
	DSI_C_LP_WAKEUP,
	DSI_C_HS_CLK_PRE,
	DSI_C_HS_CLK_PREPARE,
	DSI_C_HS_CLK_ZERO,
	DSI_C_HS_CLK_POST,
	DSI_C_HS_CLK_TRAIL,
	DSI_C_HS_LPX,
	DSI_C_HS_PRE,
	DSI_C_HS_ZERO,
	DSI_C_HS_TRAIL,
	DSI_C_HS_EXIT,
	DSI_C_LPX,
	DSI_C_LP_TA_GO,
	DSI_C_LP_TA_SURE,
	DSI_C_LP_TA_GET,
	DSI_C_MAX,
};

void *dsi_hw_init(void __iomem *base_addr, struct dsi_hw_init *dsi_init)
{
	struct dsi_hw *dsihw = NULL;

	if (dsi_init->dlcount > DSI_DL_COUNT)
		return (void *)NULL;

	dsihw = (struct dsi_hw *)&dsi_dev;
	dsihw->base_addr = base_addr;
	dsihw->dlcount = dsi_init->dlcount;

	return (void *)dsihw;
}

void dsi_errata_init(void __iomem *base_addr, int size, bool has_mipi_errata)
{
	struct hw_errata_reg *dsihw = NULL;

	dsihw = &dsi_dev.hw_errata;
	dsihw->addr_start = base_addr;
	dsihw->addr_end = base_addr + size;
	dsihw->errata = has_mipi_errata;
}

void dsi_hw_phy_state(void *handle, enum dsi_hw_phy_state state)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;
	unsigned int reg_mask = 0;
	uint32_t regval = 0;

	reg_mask = DSI_REG_FIELD_SET(DSI1_PHYC, FORCE_TXSTOP_0, 1)
		| DSI_REG_FIELD_SET(DSI1_PHYC, TXULPSCLK, 1)
		| DSI_REG_FIELD_SET(DSI1_PHYC, TX_HSCLK_CONT, 1);

	switch (dsihw->dlcount) {
	case 2:
		reg_mask |= DSI_REG_FIELD_SET(DSI1_PHYC, TXULPSESC_1, 1);
		regval |= DSI_REG_FIELD_SET(DSI1_PHYC, TXULPSESC_1, 1);
		/* fall-through */
	default:
		reg_mask |= DSI_REG_FIELD_SET(DSI1_PHYC, TXULPSESC_0, 1);
		regval |= DSI_REG_FIELD_SET(DSI1_PHYC, TXULPSESC_0, 1);
	}

	switch (state) {
	case PHY_TXSTOP:
		regval |= DSI_REG_FIELD_SET(DSI1_PHYC, FORCE_TXSTOP_0, 1);
		break;
	case PHY_ULPS:
		regval |= DSI_REG_FIELD_SET(DSI1_PHYC, TXULPSCLK, 1);
		break;
	default:
		regval = 0;
		break;
	}

	DSI_REG_WRITE_MASKED(dsihw->base_addr, DSI1_PHYC, reg_mask, regval);
}

void dsi_hw_phy_afe_off(void *handle)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;
	uint32_t afe_val = 0;
	unsigned int afe_mask = 0;

	afe_mask = DSI1_PHY_AFEC0_PD_MASK | DSI1_PHY_AFEC0_PD_BG_MASK;

	afe_val = DSI_REG_FIELD_SET(DSI1_PHY_AFEC0, PD, 1) /* Pwr Down AFE */
		| DSI_REG_FIELD_SET(DSI1_PHY_AFEC0, PD_BG, 1); /* Pwr Down BG */

	DSI_REG_WRITE_MASKED
			(dsihw->base_addr, DSI1_PHY_AFEC0, afe_mask, afe_val);
}

static bool dsihw_timingdiv_roundup(struct dsi_counter *pdsihw_c,
		unsigned int i,	unsigned int dividend, unsigned int divisor)
{
	unsigned int counter;
	unsigned int counter_remainder;

	counter = dividend / divisor;
	counter_remainder = dividend % divisor;

	if (counter_remainder)
		counter++;

	if ((counter % pdsihw_c[i].counter_step) != 0)
		counter += pdsihw_c[i].counter_step;

	counter = counter & (~(pdsihw_c[i].counter_step - 1));
	counter -= pdsihw_c[i].counter_offs;
	pdsihw_c[i].counter = counter;

	if (counter > pdsihw_c[i].counter_max)
		return false;

	return true;
}

bool dsi_hw_set_timing(void *handle, void *phy_timing,
		enum dsi_clk_sel coreclksel, unsigned int esc_clk_mhz,
		unsigned int hs_bitrate_mbps, unsigned int lpbitrate_mbps)
{
	bool res = false;

	unsigned int scaled_time_min;
	unsigned int scaled_time_min1;
	unsigned int scaled_time_min2;
	unsigned int scaled_time_max;
	unsigned int scaled_period;
	unsigned int scaled_ui_ns;
	unsigned int scaled_esc_clk_ns;
	unsigned int lp_clk_khz;
	unsigned int i;
	struct dsi_counter *pdsihw_c;
	struct dsi_hw *dsihw;
	unsigned int counter_offs;
	unsigned int counter_step;
	unsigned int lp_lpx_ns;

	dsihw = (struct dsi_hw *)handle;
	pdsihw_c = (struct dsi_counter *)phy_timing;
	scaled_ui_ns = DSI_ONE_MEGA / hs_bitrate_mbps;

	scaled_esc_clk_ns = DSI_ONE_MEGA / esc_clk_mhz;

	/* figure step & offset for HS counters */
	if (coreclksel == DSI_HW_BIT_CLK_DIV_BY_8) {
		counter_offs = 8;
		counter_step = 8;
	} else if (coreclksel == DSI_HW_BIT_CLK_DIV_BY_4) {
		counter_offs = 4;
		counter_step = 4;
	} else {
		counter_offs = 2;
		counter_step = 2;
	}

	/* init offset & step for HS counters */
	for (i = 1; i < DSI_C_MAX; i++) {
		/* Period_units [ns] */
		if (pdsihw_c[i].time_base & DSI_C_TIME_HS) {
			pdsihw_c[i].counter_offs = counter_offs;
			pdsihw_c[i].counter_step = counter_step;
		}
	}

	/* LP clk (LP Symbol Data Rate) = esc_clk / esc2lp_ratio */
	/* calculate esc2lp_ratio */
	if (!dsihw_timingdiv_roundup(pdsihw_c, DSI_C_ESC2LP_RATIO,
				esc_clk_mhz, lpbitrate_mbps * 2)) {
		return false;
	}
	/* actual lp clock */
	lp_clk_khz = 1000 * esc_clk_mhz
		/ (pdsihw_c[DSI_C_ESC2LP_RATIO].counter
				+ pdsihw_c[DSI_C_ESC2LP_RATIO].counter_offs);

	/* lp_esc_clk == lp_data_clock */
	lp_lpx_ns = (DSI_ONE_MEGA / lp_clk_khz);
	/* set LP LPX to be equal to LP bit rate */

	/* set time_min_ns for LP esc_clk counters */
	pdsihw_c[DSI_C_LPX].time_min1_ns =
			pdsihw_c[DSI_C_LPX].time_lpx * lp_lpx_ns;
	pdsihw_c[DSI_C_LP_TA_GO].time_min1_ns =
		pdsihw_c[DSI_C_LP_TA_GO].time_lpx * lp_lpx_ns;
	pdsihw_c[DSI_C_LP_TA_SURE].time_min1_ns =
		pdsihw_c[DSI_C_LP_TA_SURE].time_lpx * lp_lpx_ns;
	pdsihw_c[DSI_C_LP_TA_GET].time_min1_ns =
		pdsihw_c[DSI_C_LP_TA_GET].time_lpx * lp_lpx_ns;

	/* start from 1, skip [0]=esc2lp_ratio */
	for (i = 1; i < DSI_C_MAX; i++) {
		/* Period_min1 [ns] */
		scaled_time_min1 = pdsihw_c[i].time_min1_ns * 1000
			+ pdsihw_c[i].time_min1_ui * scaled_ui_ns;

		/* Period_min2 [ns] */
		if (pdsihw_c[i].mode & DSI_C_MIN_MAX_OF_2)
			scaled_time_min2 = pdsihw_c[i].time_min2_ns * 1000
				+ pdsihw_c[i].time_min2_ui * scaled_ui_ns;
		else
			scaled_time_min2 = 0;

		/* Period_min [ns] = max(min1, min2) */
		if (scaled_time_min1 >= scaled_time_min2)
			scaled_time_min = scaled_time_min1;
		else
			scaled_time_min = scaled_time_min2;

		/* Period_max [ns] */
		if (pdsihw_c[i].mode & DSI_C_HAS_MAX)
			scaled_time_max = pdsihw_c[i].time_max_ns * 1000
				+ pdsihw_c[i].time_max_ui * scaled_ui_ns;
		else
			scaled_time_max = 0;

		/* Period_units [ns] */
		if (pdsihw_c[i].time_base & DSI_C_TIME_HS)
			scaled_period = scaled_ui_ns;
		else if (pdsihw_c[i].time_base & DSI_C_TIME_ESC)
			scaled_period = scaled_esc_clk_ns;
		else
			scaled_period = 0;

		pdsihw_c[i].period = scaled_period;

		if (scaled_period != 0) {
			res = dsihw_timingdiv_roundup(pdsihw_c, i,
					scaled_time_min, scaled_period);
			if (!res)
				return res;

			if ((pdsihw_c[i].mode & DSI_C_HAS_MAX) &&
				((pdsihw_c[i].counter * scaled_period) >
						scaled_time_max)) {
				return false;
			}
		}
	}

	/* set ESC 2 LPDT ratio */
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_PHYC,
			ESC_CLK_LPDT, pdsihw_c[DSI_C_ESC2LP_RATIO].counter);

	/* HS_DLT5  INIT */
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_HS_DLT5,
			HS_INIT, pdsihw_c[DSI_C_HS_INIT].counter);

	/* HS_CLT2  ULPS WakeUp */
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_HS_CLT2,
			HS_WUP, pdsihw_c[DSI_C_HS_WAKEUP].counter);
	/* LP_DLT7  ULPS WakeUp */
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_LP_DLT7,
			LP_WUP, pdsihw_c[DSI_C_LP_WAKEUP].counter);

	/* HS CLK - HS_CLT0 reg */
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_HS_CLT0,
			HS_CZERO, pdsihw_c[DSI_C_HS_CLK_ZERO].counter);
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_HS_CLT0,
			HS_CPRE, pdsihw_c[DSI_C_HS_CLK_PRE].counter);
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_HS_CLT0,
			HS_CPREP, pdsihw_c[DSI_C_HS_CLK_PREPARE].counter);

	/* HS CLK - HS_CLT1 reg */
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_HS_CLT1,
			HS_CTRAIL, pdsihw_c[DSI_C_HS_CLK_TRAIL].counter);
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_HS_CLT1,
			HS_CPOST, pdsihw_c[DSI_C_HS_CLK_POST].counter);

	/* HS DATA HS_DLT3 REG */
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_HS_DLT3,
			HS_EXIT, pdsihw_c[DSI_C_HS_EXIT].counter);
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_HS_DLT3,
			HS_ZERO, pdsihw_c[DSI_C_HS_ZERO].counter);
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_HS_DLT3,
			HS_PRE, pdsihw_c[DSI_C_HS_PRE].counter);

	/* HS DATA HS_DLT4 REG */
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_HS_DLT4, HS_ANLAT, 0);
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_HS_DLT4,
			HS_TRAIL, pdsihw_c[DSI_C_HS_TRAIL].counter);
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_HS_DLT4,
			HS_LPX, pdsihw_c[DSI_C_HS_LPX].counter);

	/* LP_DLT6 REG */
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_LP_DLT6,
			TA_GET, pdsihw_c[DSI_C_LP_TA_GET].counter);
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_LP_DLT6,
			TA_SURE, pdsihw_c[DSI_C_LP_TA_SURE].counter);
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_LP_DLT6,
			TA_GO, pdsihw_c[DSI_C_LP_TA_GO].counter);
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_LP_DLT6,
			LPX, pdsihw_c[DSI_C_LPX].counter);

	return true;
}

void dsi_hw_off(void *handle)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	BRCM_WRITE_REG(dsihw->base_addr, DSI1_PHYC, 0);
	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_CTRL, DSI_EN, 0);
}

uint32_t dsi_hw_get_ena_int(void *handle)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	return BRCM_READ_REG(dsihw->base_addr, DSI1_INT_EN);
}

void dsi_hw_ena_int(void *handle, uint32_t intmask)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	BRCM_WRITE_REG(dsihw->base_addr, DSI1_INT_EN, intmask);
}

uint32_t dsi_hw_get_int(void *handle)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	return BRCM_READ_REG(dsihw->base_addr, DSI1_INT_STAT);
}

void dsi_hw_clr_int(void *handle, uint32_t intmask)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	BRCM_WRITE_REG(dsihw->base_addr, DSI1_INT_STAT, intmask);
}

void dsi_hw_clr_fifo(void *handle, uint32_t fifomask)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	DSI_REG_WRITE_MASKED(dsihw->base_addr, DSI1_CTRL, fifomask, fifomask);
}

uint32_t dsi_hw_get_status(void *handle)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	return BRCM_READ_REG(dsihw->base_addr, DSI1_STAT);
}

void dsi_hw_clr_status(void *handle, uint32_t statmask)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	BRCM_WRITE_REG(dsihw->base_addr, DSI1_STAT, statmask);
}

void dsi_hw_tx_bta(void *handle, uint8_t tx_eng)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;
	uint32_t pktc = 0;

	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_CTRL, CMND_CTRL_BTA);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_REPEAT, 1);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_MODE, 1);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_TX_TIME,
			DSI_HW_CMND_WHEN_BEST_EFFORT);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, DISPLAY_NO,
			DSI_PKT_SRC_CMND_FIFO);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_EN, 1);

	if (tx_eng)
		BRCM_WRITE_REG(dsihw->base_addr, DSI1_TXPKT2_C, pktc);
	else
		BRCM_WRITE_REG(dsihw->base_addr, DSI1_TXPKT1_C, pktc);

}

void dsi_hw_tx_trig(void *handle, uint8_t tx_eng, uint8_t trig)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;
	uint32_t pktc = 0;

	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_CTRL, CMND_CTRL_TRIG);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_REPEAT, 1);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_MODE, 1);	/* LowPower */

	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_TX_TIME,
			DSI_HW_CMND_WHEN_BEST_EFFORT);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, DISPLAY_NO,
			DSI_PKT_SRC_CMND_FIFO);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_EN, 1);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, TRIG_CMD, trig);

	if (tx_eng)
		BRCM_WRITE_REG(dsihw->base_addr, DSI1_TXPKT2_C, pktc);
	else
		BRCM_WRITE_REG(dsihw->base_addr, DSI1_TXPKT1_C, pktc);
}

enum dsi_hw_res dsi_hw_tx_short(void *handle,
		uint8_t tx_eng, struct dsi_tx_cfg *tx_cfg)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;
	uint32_t pktc = 0;
	uint32_t pkth = 0;
	uint32_t dsi_dt = 0;

	if (tx_cfg->msg_len > 2)
		return DSI_HW_MSG_SIZE;

	if (tx_cfg->is_lp)
		pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_MODE, 1);

	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_CTRL,
						CMND_CTRL_CMND_PKT);

	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_REPEAT, 1);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_TX_TIME, tx_cfg->vm_when);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, DISPLAY_NO,
			DSI_PKT_SRC_CMND_FIFO);

	if (tx_cfg->start)
		pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_EN, 1);

	dsi_dt = ((tx_cfg->vc & 0x00000003) << 6) | tx_cfg->dsi_cmnd;

	if (tx_cfg->msg_len >= 1)
		pkth |= (uint32_t)tx_cfg->msg[0];
	if (tx_cfg->msg_len == 2)
		pkth |= ((uint32_t)tx_cfg->msg[1] << 8);

	pkth = DSI_REG_FIELD_SET(DSI1_TXPKT1_H, WC_PARAM, pkth);

	pkth |= dsi_dt;

	if (tx_eng) {
		BRCM_WRITE_REG(dsihw->base_addr, DSI1_TXPKT2_H, pkth);
		BRCM_WRITE_REG(dsihw->base_addr, DSI1_TXPKT2_C, pktc);
	} else {
		BRCM_WRITE_REG(dsihw->base_addr, DSI1_TXPKT1_H, pkth);
		BRCM_WRITE_REG(dsihw->base_addr, DSI1_TXPKT1_C, pktc);
	}

	return DSI_HW_OK;
}

enum dsi_hw_res dsi_hw_tx_long(void *handle,
		uint8_t tx_eng, struct dsi_tx_cfg *tx_cfg)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;
	uint32_t pktc = 0;
	uint32_t pkth = 0;
	uint32_t dsi_dt = 0;
	uint32_t pfifo_count_b;

	if (tx_cfg->msg_len_cfifo > DSI_HW_CMND_FIFO_SIZE_B)
		return DSI_HW_MSG_SIZE;

	pfifo_count_b = tx_cfg->msg_len - tx_cfg->msg_len_cfifo;

	if ((pfifo_count_b > DSI_HW_PIXEL_FIFO_SIZE_B)
			|| (pfifo_count_b % 4 != 0)) {
		return DSI_HW_MSG_SIZE;
	}

	if (tx_cfg->is_lp)
		pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_MODE, 1);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_TYPE, 1);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_CTRL,
						CMND_CTRL_CMND_PKT);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_REPEAT, tx_cfg->repeat);
	pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_TX_TIME, tx_cfg->vm_when);

	if (tx_cfg->msg_len == tx_cfg->msg_len_cfifo)
		pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, DISPLAY_NO,
				DSI_PKT_SRC_CMND_FIFO);
	else if (tx_cfg->disp_engine == 1)
		pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, DISPLAY_NO,
				DSI_PKT_SRC_DE1);
	else
		pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, DISPLAY_NO,
				DSI_PKT_SRC_DE0);

	if (tx_cfg->start)
		pktc |= DSI_REG_FIELD_SET(DSI1_TXPKT1_C, CMD_EN, 1);

	dsi_dt = ((tx_cfg->vc & 0x00000003) << 6) | tx_cfg->dsi_cmnd;

	pkth = DSI_REG_FIELD_SET(DSI1_TXPKT1_H,
			WC_CDFIFO, tx_cfg->msg_len_cfifo)
		| DSI_REG_FIELD_SET(DSI1_TXPKT1_H, WC_PARAM, tx_cfg->msg_len);

	pkth |= dsi_dt;

	if (tx_eng) {
		BRCM_WRITE_REG(dsihw->base_addr, DSI1_TXPKT2_H, pkth);
		BRCM_WRITE_REG(dsihw->base_addr, DSI1_TXPKT2_C, pktc);
	} else {
		BRCM_WRITE_REG(dsihw->base_addr, DSI1_TXPKT1_H, pkth);
		BRCM_WRITE_REG(dsihw->base_addr, DSI1_TXPKT1_C, pktc);
	}

	return DSI_HW_OK;
}

void dsi_hw_tx_start(void *handle, uint8_t tx_eng, bool start)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	if (tx_eng)
		BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_TXPKT2_C,
					CMD_EN, start);
	else
		BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_TXPKT1_C,
					CMD_EN, start);
}

enum dsi_hw_res dsi_hw_wr_cfifo(void *handle,
		uint8_t *buff_ptr, unsigned int byte_count)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	if (byte_count > DSI_HW_CMND_FIFO_SIZE_B)
		return DSI_HW_MSG_SIZE;

	while (byte_count--)
		BRCM_WRITE_REG(dsihw->base_addr,
				DSI1_TXPKT_CMD_FIFO, *buff_ptr++);

	return DSI_HW_OK;
}

enum dsi_hw_res dsi_hw_wr_pfifo_be(void *handle,
		uint8_t *buff_ptr, unsigned int byte_count)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;
	unsigned int word_count;
	uint32_t word;

	if (byte_count > DSI_HW_PIXEL_FIFO_SIZE_B)
		return DSI_HW_MSG_SIZE;

	if (byte_count % 4)
		return DSI_HW_MSG_SIZE;

	word_count = byte_count >> 2;
	while (word_count--) {
		word = __cpu_to_be32(*(uint32_t *)buff_ptr);
		BRCM_WRITE_REG(dsihw->base_addr, DSI1_TXPKT_PIXD_FIFO, word);
		buff_ptr += 4;
	}

	return DSI_HW_OK;
}

void __iomem *dsi_hw_de1_get_dma_address(void *handle)
{
	struct dsi_hw *dsihw_handle = (struct dsi_hw *)handle;

	return BRCM_REGADDR(dsihw_handle->base_addr, DSI1_TXPKT_PIXD_FIFO);
}

void dsi_hw_de1_set_dma_thresh(void *handle, uint32_t thresh)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	BRCM_WRITE_REG_FIELD(dsihw->base_addr,
				DSI1_DISP1_CTRL, DMA_THRESH, thresh);
}

void dsi_hw_de1_set_cm(void *handle, enum dsi_de1_col_mod cm)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_DISP1_CTRL, PFORMAT, cm);
}

void dsi_hw_de1_enable(void *handle, bool ena)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	if (ena)
		BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_DISP1_CTRL, EN, 1);
	else
		BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_DISP1_CTRL, EN, 0);
}

void dsi_hw_de0_set_cm(void *handle, enum dsi_de0_col_mod cm)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_DISP0_CTRL, PFORMAT, cm);
}

void dsi_hw_de0_set_pix_clk_div(void *handle, uint32_t div)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	BRCM_WRITE_REG_FIELD(dsihw->base_addr,
					DSI1_DISP0_CTRL, PIX_CLK_DIV, div);
}

void dsi_hw_de0_enable(void *handle, bool ena)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	if (ena)
		BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_DISP0_CTRL, EN, 1);
	else
		BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_DISP0_CTRL, EN, 0);
}

void dsi_hw_de0_set_mode(void *handle, enum dsi_de0_mode mode)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	BRCM_WRITE_REG_FIELD(dsihw->base_addr, DSI1_DISP0_CTRL, MODE, mode);
}

void dsi_hw_de0_st_end(void *handle, bool ena)
{
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	if (ena)
		BRCM_WRITE_REG_FIELD(dsihw->base_addr,
				DSI1_DISP0_CTRL, ST_END, 1);
	else
		BRCM_WRITE_REG_FIELD(dsihw->base_addr,
				DSI1_DISP0_CTRL, ST_END, 0);
}

static void dsi_hw_enable_digitalphy(void __iomem *dsihw_genpll_base)
{
	void __iomem *dsi_dphy_ctrl = dsihw_genpll_base +
					MIPI_DSI_DPHY_CTRL_OFFSET;
	void __iomem *dsi_misc_ctrl = dsihw_genpll_base +
					MIPI_DSI_MISC_CTRL_OFFSET;

	dsi_hw_write_reg(DSI_DPHY_LDOCONTROL_RESET, dsi_dphy_ctrl);
	dsi_hw_write_reg(DSI_DPHY_MISC_CMDINTF_DMA_EN, dsi_misc_ctrl);

	dsi_hw_write_reg(DSI_DPHY_LDOCONTROL_RESET_OUT, dsi_dphy_ctrl);
}

static void dsi_hw_enable_analogphy(void __iomem *dsihw_base)
{
	void __iomem *dsi_dphy_afec0 = dsihw_base + DSI1_PHY_AFEC0_OFFSET;
	void __iomem *dsi_dphy_afec1 = dsihw_base + DSI1_PHY_AFEC1_OFFSET;

	dsi_hw_write_reg(DSI_AFEC0_RESET, dsi_dphy_afec0);
	dsi_hw_write_reg(DSI_APHY1_CLKCNTRL, dsi_dphy_afec1);
	dsi_hw_write_reg(DSI_AFEC0_RESET_OUT, dsi_dphy_afec0);
	dsi_hw_write_reg(DSI_AFEC0_RESET, dsi_dphy_afec0);
	dsi_hw_write_reg(DSI_AFEC0_RESET_OUT, dsi_dphy_afec0);
}

static void dsi_hw_enable_phy(void __iomem *dsihw_base,
					void __iomem *dsihw_genpll_base)
{
	uint32_t rd_val;
	void __iomem *dsi_dphy_ctrl = dsihw_genpll_base +
					MIPI_DSI_DPHY_CTRL_OFFSET;

	dsi_hw_enable_analogphy(dsihw_base);
	dsi_hw_enable_digitalphy(dsihw_genpll_base);

	rd_val = readl(dsi_dphy_ctrl);

	while (rd_val != DSI_DPHY_LDOCONTROL_RESET_OUT) {
		dsi_hw_enable_digitalphy(dsihw_genpll_base);
		rd_val = readl(dsi_dphy_ctrl);
	}
}

void dsi_hw_on(void *handle, void __iomem *dsi_genpll_base)
{
	uint32_t ctrl, phyc_val;
	struct dsi_hw *dsihw = (struct dsi_hw *)handle;

	/* Byte clock */
	ctrl = BIT(0) | BIT(DSI1_CTRL_DISP_ECCC_SHIFT)
		| BIT(DSI1_CTRL_DISP_CRCC_SHIFT)
		| BIT(DSI1_CTRL_HSDT_EOT_EN_SHIFT);

	dsi_hw_write_reg(ctrl, dsihw->base_addr);

	/* DE1 */
	dsi_hw_write_reg(DSI_DISP1_DMA_THRESHOLD,
			dsihw->base_addr + DSI1_DISP1_CTRL_OFFSET);

	/* PHYC and Timing */
	dsi_hw_write_reg(DSI_HSTX_TO_EN,
			dsihw->base_addr + DSI_HSTX_TO_CNT_OFFSET);
	dsi_hw_write_reg(DSI_LPRX_TO_EN,
			dsihw->base_addr + DSI_LPRX_TO_CNT_OFFSET);
	dsi_hw_write_reg(DSI_TA_TO_EN,
			dsihw->base_addr + DSI_TA_TO_CNT_OFFSET);
	dsi_hw_write_reg(DSI_PR_TO_EN,
			dsihw->base_addr + DSI_PR_TO_CNT_OFFSET);

	dsi_hw_enable_phy(dsihw->base_addr, dsi_genpll_base);

	/* PHYC enable */
	phyc_val = readl(dsihw->base_addr + DSI1_PHYC_OFFSET);
	phyc_val |=
		(DSI1_PHYC_PHY_DLANE0_EN_MASK | DSI1_PHYC_PHY_CLANE_EN_MASK);
	phyc_val |= DSI1_PHYC_PHY_DLANE1_EN_MASK;

	dsi_hw_write_reg(phyc_val, dsihw->base_addr + DSI1_PHYC_OFFSET);
}

void dsi_reset(void __iomem *dsi_reset_base, int reset_active, bool on)
{
	if (!on) {
		/* To reset the panel module the reset pin needs
		 * to be toggled with 1, 0, 1 sequence as per
		 * dsi standard with the delay in between depending
		 * on the panel module.
		 */
		dsi_hw_write_reg(0x1, dsi_reset_base);
		udelay(700);
		dsi_hw_write_reg(0x0, dsi_reset_base);
		udelay(1000);
		dsi_hw_write_reg(0x1, dsi_reset_base);
		mdelay(100);
		return;

	}
	dsi_hw_write_reg(reset_active, dsi_reset_base);
}
