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
#include "cygnus-clk-utils.h"

/*
 * AUDIO PLL VCO frequency parameter table
 *
 * PLL output frequency = ((ndiv_int + ndiv_frac / 2^20) *
 *                         (parent clock rate / pdiv)
 *
 * On Cygnus, parent is the 25MHz oscillator
 *
 * The PLL output (Vco freq) is fed into 3 post dividers (1 - 256 dividers).
 * The output from the PLL's post-dividers is the MCLK.
 * Ideally we will be able to pick a Vco frequency that will allow us
 * to generated MCLK frequencies useful for audio.
 * For example, these MCLK freqs are perfect multiples of common audio rates:
 *   48   kHz domain:   6,144,000  12,288,000  24,576,000
 *   44.1 kHz domain:   5,644,800  11,289,600  22,579,200
 *
 * Choosing a Vco frequency will depend on the applications use cases, and
 * perfomance requirements for the clock signal.
 *
 * These Vco frequencies would allow us to produce exact and MCLK in each
 * domain.
 *  Vco: 1,806,336,000
 *    ndiv_int: 72  ndiv_frac: 265,751
 *    post div     MCLK
 *     147       12,288,000
 *     160       11,289,600
 *      80       22,579,200
 *
 *  Vco: 3,612,672,000
 *    ndiv_int: 144  ndiv_frac: 531,502
 *    post div     MCLK
 *     147       24,576,000
 *     160       22,579,200
 *      80       45,158,400
 *
 * These Vco frequencies would have a predicted 55 ppm error in producing
 * the 44.1 kHz domain and be exact for 48 kHz domain.
 * The benefit of the previous set is that we get an addition MCLK for 48 kHz.
 *  Vco: 2,777,088,000
 *    ndiv_int: 111   ndiv_frec: 87,577
 *    post div      MCLK
 *     246       11,288,976 (error 55 ppm)
 *     226       12,288,000
 *     123       22,577,951 (error 55 ppm)
 *     113       24,576,000
 *
 *  Vco: 1,388,544,000
 *    post div      MCLK
 *     246        5,644,800 (error 55 ppm)
 *     226        6,144,000
 *     123       11,288,976 (error 55 ppm)
 *     113       12,288,000
 *
 * This Vco can generate a large range of exact MCLKs for 48 Khz.
 * Also, does a half decent job for some 44.1 kHz clocks.
 *   Vco: 1,376,256,000
 *    ndiv_int: 55   ndiv_frac: 52,680
 *    post div      MCLK
 *     224        6,144,000
 *     112       12,288,000
 *      56       24,576,000
 *      28       49,152,000
 *      61       22,561,574 (error 781 ppm)
 *     122       11,280,787 (error 781 ppm)
 *
 *  Vco: 2,162,688,000
 *   ndiv_int: 86  ndiv_frac: 532,173
 *  - Exact for 48 kHz domain
 *  - Good frac for tweaking
 *  - Good range of values from Post div:
 *    post div      MCLK
 *      88       24.576 MHz
 *      44       49.152 MHz
 *     176       12.288 MHz
 *  - Channel 0 has and extra div 2 so that channel could be used for 6.144 MHz
 *
 *  Vco:
 *   ndiv_int: 88   ndiv_frac:  535,260
 *  - Exact for 44.1 kHz domain
 *  - Good Frac for tweaking
 *  - Good range of values from Post div:
 *    post div      MCLK
 *       98       22,579,200
 *       49       45,158,400
 *      196       11,289,600
 */
static bool g_pll_tweak_init;

static unsigned long calc_adj(unsigned long rate_in, long ppb)
{
	unsigned long rate_out;
	unsigned long inc;

	inc = div64_u64((u64)((u64)rate_in * (u64)abs(ppb)), (u64)1000000000);

	if (ppb < 0)
		rate_out = rate_in - inc;
	else
		rate_out = rate_in + inc;

	return rate_out;
}

/*
 * pll_tweakppb_get - read the pll fractional setting.
 *   kcontrol: The control for the speaker gain.
 *   ucontrol: The value that needs to be updated.
 */
static int pll_tweakppb_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct pll_tweak_info *tweak_info = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = tweak_info->ppb_adj;

	return 0;
}

/*
 * pll_tweakppb_put - set the pll fractional setting.
 *   kcontrol: The control for the pll tweak.
 *   ucontrol: The value that needs to be set.
 */
static int pll_tweakppb_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct pll_tweak_info *tweak_info = snd_kcontrol_chip(kcontrol);
	int value;
	unsigned long new_rate;

	value = ucontrol->value.integer.value[0];

	tweak_info->ppb_adj = value;

	/* If nominal rate not set yet, then just exit */
	if (!tweak_info->pll_nominal_rate)
		return 0;

	new_rate = calc_adj(tweak_info->pll_nominal_rate,
			    tweak_info->ppb_adj);

	snd_soc_dai_set_pll(tweak_info->cpu_dai, 0, 0, 0, new_rate);

	return 0;
}

static int tweakppb_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = -10000000;
	uinfo->value.integer.max = 10000000;
	return 0;
}

static struct snd_kcontrol_new pll_tweakppb_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "PLL Tweak ppb",
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.info = tweakppb_info,
	.get = pll_tweakppb_get,
	.put = pll_tweakppb_put
};

int cygnus_ssp_pll_tweak_initialize(struct pll_tweak_info *tweak_info,
				    const char *prefix)
{
	struct snd_card *card;
	int ret;

	/* There is only 1 PLL so we do not want multiple instances */
	if (g_pll_tweak_init)
		return -EBUSY;

	card = tweak_info->cpu_dai->component->card->snd_card;

	tweak_info->ppb_adj = 0;
	tweak_info->pll_nominal_rate = 0;

	ret = snd_ctl_add(card, snd_soc_cnew(&pll_tweakppb_control,
					     tweak_info,
					     pll_tweakppb_control.name,
					     prefix));
	if (ret)
		return ret;

	g_pll_tweak_init = true;
	return 0;
}
EXPORT_SYMBOL_GPL(cygnus_ssp_pll_tweak_initialize);

/* Change the nominal rate.  ppb adjustment will be applied to this rate */
int cygnus_ssp_pll_tweak_update(struct pll_tweak_info *tweak_info,
				unsigned long nominal_freq)
{
	unsigned long new_rate;

	if (!g_pll_tweak_init)
		return -EINVAL;

	if (tweak_info->pll_nominal_rate == nominal_freq)
		return 0;

	new_rate = calc_adj(nominal_freq, tweak_info->ppb_adj);
	snd_soc_dai_set_pll(tweak_info->cpu_dai, 0, 0, 0, new_rate);

	tweak_info->pll_nominal_rate = nominal_freq;
	return 0;
}
EXPORT_SYMBOL_GPL(cygnus_ssp_pll_tweak_update);
