/*
 * Copyright 2017 Broadcom
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
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "clk-iproc.h"

struct audiomux_clk {
	struct clk_hw		hw;
	struct regmap		*regmap;
	u32			reg_offset;

	u32			bit_shift;
	u32			bit_width;
};

#define to_iproc_audmux(_hw) container_of(_hw, struct audiomux_clk, hw)

static u8 audiomux_get_parent(struct clk_hw *hw)
{
	struct audiomux_clk *mux = to_iproc_audmux(hw);
	unsigned int mask;
	unsigned int val;

	mask = GENMASK(mux->bit_width - 1, 0);
	regmap_read(mux->regmap, mux->reg_offset, &val);

	val >>= mux->bit_shift;
	val &= mask;

	return val;
}

static int audiomux_set_parent(struct clk_hw *hw, u8 index)
{
	struct audiomux_clk *mux = to_iproc_audmux(hw);
	unsigned int mask;
	unsigned int val;

	mask = GENMASK(mux->bit_width + mux->bit_shift - 1, mux->bit_shift);
	val = (index << mux->bit_shift);

	return regmap_update_bits(mux->regmap, mux->reg_offset, mask, val);
}

static const struct clk_ops audiomux_clk_ops = {
	.get_parent = audiomux_get_parent,
	.set_parent = audiomux_set_parent,
	.determine_rate = __clk_mux_determine_rate,
};

int iproc_audiomux_setup(struct device_node *node)
{
	struct audiomux_clk *audiomux;
	struct clk_init_data init;
	struct regmap *regmap;
	u8 num_parents;
	const char **parent_names = NULL;
	unsigned int reg_offset;
	unsigned int bit_shift;
	unsigned int bit_width;
	int ret;

	/*  get regmap */
	regmap = syscon_regmap_lookup_by_phandle(node, "clk-mux-syscon");
	if (IS_ERR(regmap)) {
		pr_err("%s failed to get clk-mux-syscon regmap\n", __func__);
		return PTR_ERR(regmap);
	}

	if (of_property_read_u32(node, "bit-shift", &bit_shift)) {
		pr_err("%s missing bit-shift property", node->name);
		return -EINVAL;
	}

	if (of_property_read_u32(node, "bit-width", &bit_width)) {
		pr_err("%s missing bit-width property", node->name);
		return -EINVAL;
	}

	if (of_property_read_u32(node, "reg-offset", &reg_offset)) {
		pr_err("%s missing reg-offset property", node->name);
		return -EINVAL;
	}

	/* allocate the mux */
	audiomux = kzalloc(sizeof(*audiomux), GFP_KERNEL);
	if (WARN_ON(!audiomux))
		return -ENOMEM;

	num_parents = of_clk_get_parent_count(node);
	if (num_parents < 2) {
		pr_err("%s must have 2 or more parent\n", node->name);
		ret = -EINVAL;
		goto err_parent;
	}

	parent_names = kcalloc(num_parents, sizeof(char *), GFP_KERNEL);
	if (WARN_ON(!parent_names)) {
		ret = -ENOMEM;
		goto err_parent;
	}

	ret = of_clk_parent_fill(node, parent_names, num_parents);
	if (ret != num_parents) {
		pr_err("%s Error parsing parents.\n", node->name);
		goto err_parent;
	}

	init.name = node->name;
	init.flags = CLK_SET_RATE_NO_REPARENT | CLK_SET_RATE_PARENT;
	init.parent_names = parent_names;
	init.num_parents = num_parents;

	init.ops = &audiomux_clk_ops;

	audiomux->regmap = regmap;
	audiomux->reg_offset = reg_offset;
	audiomux->bit_shift = bit_shift;
	audiomux->bit_width = bit_width;

	audiomux->hw.init = &init;

	ret = clk_hw_register(NULL, &audiomux->hw);
	if (WARN_ON(ret))
		goto err_clk_register;

	ret = of_clk_add_hw_provider(node, of_clk_hw_simple_get, &audiomux->hw);
	if (WARN_ON(ret))
		goto err_add_provider;

	return 0;

err_add_provider:
	clk_hw_unregister(&audiomux->hw);
err_clk_register:
	kfree(parent_names);
err_parent:
	kfree(audiomux);

	return ret;
}
