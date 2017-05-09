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
#include <sound/soc.h>

struct pll_tweak_info {
	struct snd_soc_dai *cpu_dai;  /* input */
	long		ppb_adj;
	unsigned long	pll_nominal_rate;
};

int cygnus_ssp_pll_tweak_initialize(struct pll_tweak_info *tweak_info,
				    const char *prefix);
int cygnus_ssp_pll_tweak_update(struct pll_tweak_info *tweak_info,
				unsigned long nominal_freq);
