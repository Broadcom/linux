/*
 * Copyright 2018 Broadcom
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
#include <linux/err.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

#include <dt-bindings/clock/bcm-omega.h>
#include "clk-iproc.h"

#define REG_VAL(o, s, w) { .offset = o, .shift = s, .width = w, }

#define AON_VAL(o, pw, ps, is) { .offset = o, .pwr_width = pw, \
	.pwr_shift = ps, .iso_shift = is }

#define SW_CTRL_VAL(o, s) { .offset = o, .shift = s, }

#define RESET_VAL(o, rs, prs) { .offset = o, .reset_shift = rs, \
	.p_reset_shift = prs }

#define DF_VAL(o, kis, kiw, kps, kpw, kas, kaw) { .offset = o, .ki_shift = kis,\
	.ki_width = kiw, .kp_shift = kps, .kp_width = kpw, .ka_shift = kas,    \
	.ka_width = kaw }

#define VCO_CTRL_VAL(uo, lo) { .u_offset = uo, .l_offset = lo }

#define ENABLE_VAL(o, es, hs, bs) { .offset = o, .enable_shift = es, \
	.hold_shift = hs, .bypass_shift = bs }

static const struct iproc_pll_ctrl audiopll = {
	.flags = IPROC_CLK_PLL_NEEDS_SW_CFG | IPROC_CLK_PLL_HAS_NDIV_FRAC |
		IPROC_CLK_PLL_USER_MODE_ON | IPROC_CLK_PLL_RESET_ACTIVE_LOW |
		IPROC_CLK_PLL_CALC_PARAM,
	.aon = AON_VAL(0x0, 4, 17, 16),
	.reset = RESET_VAL(0x6c, 0, 1),
	.dig_filter = DF_VAL(0x58, 0, 3, 6, 4, 3, 3),
	.sw_ctrl = SW_CTRL_VAL(0x4, 0),
	.ndiv_int = REG_VAL(0x8, 0, 10),
	.ndiv_frac = REG_VAL(0x8, 10, 20),
	.pdiv = REG_VAL(0x54, 0, 4),
	.vco_ctrl = VCO_CTRL_VAL(0x0c, 0x10),
	.status = REG_VAL(0x64, 0, 1),
	.macro_mode = REG_VAL(0x0, 0, 3),
};

static const struct iproc_clk_ctrl audiopll_clk[] = {
	[BCM_OMEGA_AUDIOPLL_CH0] = {
		.channel = BCM_OMEGA_AUDIOPLL_CH0,
		.flags = IPROC_CLK_AON | IPROC_CLK_MCLK_DIV_BY_2,
		.enable = ENABLE_VAL(0x14, 8, 10, 9),
		.mdiv = REG_VAL(0x14, 0, 8),
	},
	[BCM_OMEGA_AUDIOPLL_CH1] = {
		.channel = BCM_OMEGA_AUDIOPLL_CH1,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x18, 8, 10, 9),
		.mdiv = REG_VAL(0x18, 0, 8),
	},
	[BCM_OMEGA_AUDIOPLL_CH2] = {
		.channel = BCM_OMEGA_AUDIOPLL_CH2,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x1c, 8, 10, 9),
		.mdiv = REG_VAL(0x1c, 0, 8),
	},
	[BCM_OMEGA_AUDIOPLL_CH3] = {
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x20, 8, 10, 9),
		.mdiv = REG_VAL(0x20, 0, 8),
	},
	[BCM_OMEGA_AUDIOPLL_CH4] = {
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x24, 8, 10, 9),
		.mdiv = REG_VAL(0x24, 0, 8),
	},
	[BCM_OMEGA_AUDIOPLL_CH5] = {
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x28, 8, 10, 9),
		.mdiv = REG_VAL(0x28, 0, 8),
	},
};

static void __init audiopll_clk_init(struct device_node *node)
{
	iproc_pll_clk_setup(node, &audiopll, NULL, 0,
			    audiopll_clk,  ARRAY_SIZE(audiopll_clk));
}
CLK_OF_DECLARE(omega_audiopll, "brcm,omega-audiopll", audiopll_clk_init);

static void __init audiomux_init(struct device_node *node)
{
	iproc_audiomux_setup(node);
}
CLK_OF_DECLARE(audiomux_clk, "brcm,iproc-mux-clk", audiomux_init);

