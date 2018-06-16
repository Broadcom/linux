// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Broadcom
 */
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/types.h>

#define MAX_ROWS                        8

#define KEYPAD_ROW_MUX_SEL_OFFSET       0x00000200
#define KEYPAD_COL_MUX_SEL_OFFSET       0x00000204

#define MUX_SEL_DEFAULT                 0x0FFFFFFF

static const struct of_device_id top_ctrl_match[] = {
	{ .compatible = "brcm,omega-top-ctrl", },
	{ }
};

/* Structure representing various run-time entities */
struct omega_icfg {
	struct regmap	*syscon;
};

static int bcm_kp_parse_mux_select(struct device_node *np,
				   const char *prop_name,
				   u32 *mux_sel)
{
	int count;
	u32 mux_select[MAX_ROWS];
	int ii, err;

	count = of_property_count_elems_of_size(np, prop_name, sizeof(u32));
	if (count <= 0)
		return count;

	err = of_property_read_u32_array(np, prop_name, mux_select, count);
	if (err < 0)
		return err;

	*mux_sel = MUX_SEL_DEFAULT;
	for (ii = 0; ii < count; ii++) {
		*mux_sel &= ~(0xF << (ii*4));
		*mux_sel |= (mux_select[ii] << (ii*4));
	}

	return count;
}

static int __init bcm_omega_icfg_init(void)
{
	struct device_node *np;
	struct omega_icfg *icfg;
	u32 row_mux_sel;
	u32 col_mux_sel;
	int ret = 0;

	np = of_find_matching_node(NULL, top_ctrl_match);
	if (!np)
		return ret;

	icfg = kzalloc(sizeof(*icfg), GFP_KERNEL);
	if (!icfg)
		return -ENOMEM;

	icfg->syscon = syscon_regmap_lookup_by_phandle(np, "brcm,omega-icfg");
	if (IS_ERR(icfg->syscon))
		return PTR_ERR(icfg->syscon);

	if (bcm_kp_parse_mux_select(np,
				    "brcm,omega-key-row-mux",
				    &row_mux_sel) > 0) {
		/*
		 * keypad row mux select -- assign keypad
		 * I/Os as key rows (outputs)
		 */
		regmap_write(icfg->syscon,
			     KEYPAD_ROW_MUX_SEL_OFFSET,
			     row_mux_sel);
	}

	if (bcm_kp_parse_mux_select(np,
				    "brcm,omega-key-col-mux",
				    &col_mux_sel) > 0) {
		/*
		 * keypad col mux select -- assign keypad
		 * I/Os as key columns (inputs)
		 */
		regmap_write(icfg->syscon,
			     KEYPAD_COL_MUX_SEL_OFFSET,
			     col_mux_sel);
	}

	return 0;
}
arch_initcall(bcm_omega_icfg_init);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM Omega ICFG");
