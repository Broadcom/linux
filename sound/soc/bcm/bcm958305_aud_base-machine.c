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
 *
 *
 * This is the ASoC machine file for the Cygnus BCM958305_AUD_BASE board
 * with the module BCM958305_AUDIO_SA.
 * This board ensemble consists of a Cygnus SoC and two codecs:
 * WM8994 and AK4385.
 */
#include <asm/mach-types.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "cygnus-ssp.h"
#include "bcm-card-utils.h"
#include "cygnus-clk-utils.h"
#include "../codecs/wm8994.h"

struct card_state_data {
	unsigned int    card_clocking_mode;
	struct pll_tweak_info tweak_info;
};

/*
 * Configure audio route as follows:
 *
 * # DAC1
 * amixer sset 'DAC1' on,on
 * amixer sset 'DAC1R Mixer AIF1.1' on
 * amixer sset 'DAC1L Mixer AIF1.1' on
 *
 * # Headphone
 * amixer sset 'Right Headphone Mux' 'DAC'
 * amixer sset 'Left Headphone Mux' 'DAC'
 *
 * # Speakers
 * amixer sset 'SPKR DAC1' on
 * amixer sset 'SPKL DAC1' on
 * amixer sset 'Speaker Mixer' 3,3
 * amixer sset 'Speaker Boost' 7,7
 *
 * # Digital Mic
 * amixer sset 'ADCR Mux' 'DMIC'
 * amixer sset 'ADCL Mux' 'DMIC'
 * amixer sset 'AIF1ADC1R Mixer ADC/DMIC' on
 * amixer sset 'AIF1ADC1L Mixer ADC/DMIC' on
 *
 * # Line Input
 * amixer sset 'IN1L' on
 * amixer sset 'IN1L PGA IN1LP' on
 * amixer sset 'MIXINL IN1L' on
 * amixer sset 'ADCL Mux' 'ADC'
 * amixer sset 'AIF1ADC1L Mixer ADC/DMIC' on
 * amixer sset 'IN1R' on
 * amixer sset 'IN1R PGA IN1RP' on
 * amixer sset 'MIXINR IN1R' on
 * amixer sset 'ADCR Mux' 'ADC'
 * amixer sset 'AIF1ADC1R Mixer ADC/DMIC' on
 */

/* Max string length of our dt property names */
#define PROP_LEN_MAX 40

static int cygnus_hw_params_wm8994(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->card->dev;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct card_state_data *card_data;
	int ret = 0;
	unsigned int mclk_freq = 0;
	unsigned long pll_freq = 0;

	card_data = snd_soc_card_get_drvdata(rtd->card);

	switch (params_rate(params)) {
	case 8000:
	case 11025:
		mclk_freq = 6144000;
		break;

	case 16000:
	case 32000:
	case 48000:
	case 96000:
		if (card_data->card_clocking_mode == 1)
			pll_freq = 2162688000UL;

		mclk_freq = 12288000;
		break;

	case 22050:
	case 44100:
	case 88200:
		if (card_data->card_clocking_mode == 1)
			pll_freq = 2212761600UL;

		mclk_freq = 11289600;
		break;

	default:
		return -EINVAL;
	}

	if (pll_freq) {
		card_data->tweak_info.cpu_dai = rtd->cpu_dai;
		ret = cygnus_ssp_pll_tweak_update(&card_data->tweak_info,
						  pll_freq);
		if (ret < 0)
			return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_MCLK1,
				mclk_freq, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk codec_dai %u\n",
			__func__, mclk_freq);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, CYGNUS_SSP_CLKSRC_PLL,
				mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk cpu_dai %u\n",
			__func__, mclk_freq);
		return ret;
	}

	return ret;
}

static struct snd_soc_ops cygnus_ops_wm8994 = {
	.hw_params = cygnus_hw_params_wm8994,
};

/* Machine DAPM */
static const struct snd_soc_dapm_widget wm8994_dapm_widgets[] = {

	SND_SOC_DAPM_HP("Wolfson Headphone", NULL),
	SND_SOC_DAPM_SPK("Wolfson Left Speaker", NULL),
	SND_SOC_DAPM_SPK("Wolfson Right Speaker", NULL),
	SND_SOC_DAPM_LINE("Wolfson Line Input", NULL),
	SND_SOC_DAPM_MIC("Wolfson Digital Mic", NULL),
};

/* Routings for WM8994 */
static const struct snd_soc_dapm_route wm8994_audio_map[] = {
	{"Wolfson Headphone", NULL, "HPOUT1L"},
	{"Wolfson Headphone", NULL, "HPOUT1R"},

	{"Wolfson Left Speaker", NULL, "SPKOUTLP"},
	{"Wolfson Left Speaker", NULL, "SPKOUTLN"},
	{"Wolfson Right Speaker", NULL, "SPKOUTRP"},
	{"Wolfson Right Speaker", NULL, "SPKOUTRN"},

	{"IN1LP", NULL, "Wolfson Line Input"},
	{"IN1RP", NULL, "Wolfson Line Input"},

	{"DMIC1DAT", NULL, "Wolfson Digital Mic"},
	{"DMIC2DAT", NULL, "Wolfson Digital Mic"},
};

static int cygnus_init_wm8994(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dapm_context *dapm = &card->dapm;
	struct card_state_data *card_data = snd_soc_card_get_drvdata(rtd->card);
	int ret;

	snd_soc_dapm_new_controls(dapm, wm8994_dapm_widgets,
				ARRAY_SIZE(wm8994_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, wm8994_audio_map,
				ARRAY_SIZE(wm8994_audio_map));

	snd_soc_dapm_enable_pin(dapm, "HPOUT1L");
	snd_soc_dapm_enable_pin(dapm, "HPOUT1R");
	snd_soc_dapm_enable_pin(dapm, "SPKOUTLP");
	snd_soc_dapm_enable_pin(dapm, "SPKOUTLN");
	snd_soc_dapm_enable_pin(dapm, "SPKOUTRP");
	snd_soc_dapm_enable_pin(dapm, "SPKOUTRN");
	snd_soc_dapm_enable_pin(dapm, "IN1LP");
	snd_soc_dapm_enable_pin(dapm, "IN1RP");
	snd_soc_dapm_enable_pin(dapm, "DMIC1DAT");
	snd_soc_dapm_enable_pin(dapm, "DMIC2DAT");
	snd_soc_dapm_enable_pin(dapm, "Clock");

	/* Other pins NC */
	snd_soc_dapm_nc_pin(dapm, "HPOUT2P");
	snd_soc_dapm_nc_pin(dapm, "HPOUT2N");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT1N");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT1P");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT2N");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT2P");
	snd_soc_dapm_nc_pin(dapm, "IN1LN");
	snd_soc_dapm_nc_pin(dapm, "IN1RN");
	snd_soc_dapm_nc_pin(dapm, "IN2LP:VXRN");
	snd_soc_dapm_nc_pin(dapm, "IN2RP:VXRP");

	/*
	 * For testing and as an example add in the ssp kcontrols that
	 * allow us to tweak the pll.  This could be done on any port
	 * but this one has the headphone jack so it is easiest.
	 */
	card_data->tweak_info.cpu_dai = rtd->cpu_dai;
	ret = cygnus_ssp_pll_tweak_initialize(&card_data->tweak_info, "Link0");
	if (ret) {
		dev_err(card->dev, "Could not init Tweak control\n");
		return ret;
	}

	return 0;
}

static int cygnus_hw_params_ak4385(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->card->dev;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;
	unsigned int mclk_freq = 0;

	switch (params_rate(params)) {
	case 32000:
		mclk_freq = 16384000;
		break;
	case 48000:
	case 96000:
	case 192000:
		mclk_freq = 24576000;
		break;

	case  44100:
	case  88200:
	case 176400:
		mclk_freq = 22579200;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk_freq, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk codec_dai\n",
			__func__);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, CYGNUS_SSP_CLKSRC_PLL,
				mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk cpu_dai\n",
			__func__);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops cygnus_ops_ak4385 = {
	.hw_params = cygnus_hw_params_ak4385,
};

/* Machine DAPM */
static const struct snd_soc_dapm_widget ak4385_dapm_widgets[] = {

	SND_SOC_DAPM_SPK("AKM Speaker", NULL),
};

/* Routings for AK4385 */
static const struct snd_soc_dapm_route ak4385_audio_map[] = {

	{"AKM Speaker", NULL, "VOUTLP"},
	{"AKM Speaker", NULL, "VOUTLN"},
	{"AKM Speaker", NULL, "VOUTRP"},
	{"AKM Speaker", NULL, "VOUTRN"},
};

static int cygnus_init_ak4385(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dapm_context *dapm = &card->dapm;

	snd_soc_dapm_new_controls(dapm, ak4385_dapm_widgets,
				ARRAY_SIZE(ak4385_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, ak4385_audio_map,
				ARRAY_SIZE(ak4385_audio_map));

	snd_soc_dapm_enable_pin(dapm, "VOUTLP");
	snd_soc_dapm_enable_pin(dapm, "VOUTLN");
	snd_soc_dapm_enable_pin(dapm, "VOUTRP");
	snd_soc_dapm_enable_pin(dapm, "VOUTRN");

	return 0;
}

static int cygnus_hw_params_spdif(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->card->dev;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int mclk_freq = 0;
	int ret = 0;

	switch (params_rate(params)) {
	case  32000:
	case  48000:
	case  96000:
	case 192000:
		mclk_freq = 24576000;
		break;
	case  44100:
	case  88200:
	case 176400:
		mclk_freq = 22579200;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, CYGNUS_SSP_CLKSRC_PLL,
				mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk\n",
			__func__);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops cygnus_ops_spdif = {
	.hw_params = cygnus_hw_params_spdif,
};

static struct snd_soc_dai_link cygnus_dai_links[] = {
{
	.ops = &cygnus_ops_wm8994,
	.init = cygnus_init_wm8994,
},
{
	.ops = &cygnus_ops_ak4385,
	.init = cygnus_init_ak4385,
},
{
	.ops = &cygnus_ops_spdif,
},
};

static int card_clocking_mode_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *soc_card = snd_kcontrol_chip(kcontrol);
	struct card_state_data *card_data = snd_soc_card_get_drvdata(soc_card);

	ucontrol->value.integer.value[0] = card_data->card_clocking_mode;
	return 0;
}

static int card_clocking_mode_set(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct card_state_data *card_data = snd_soc_card_get_drvdata(card);
	int value;

	value = ucontrol->value.integer.value[0];
	card_data->card_clocking_mode = value;

	return 0;
}

static int card_clocking_mode_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 2;
	return 0;
}

static struct snd_kcontrol_new card_clock_mode_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Board Clocking Mode",
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.info = card_clocking_mode_info,
	.get = card_clocking_mode_get,
	.put = card_clocking_mode_set
};

int cygnus_audiobase_card_probe(struct snd_soc_card *card)
{
	return snd_soc_add_card_controls(card, &card_clock_mode_control, 1);
}

/* Audio machine driver */
static struct snd_soc_card cygnus_audio_card = {
	.name = "bcm-cygnus-aud-base",
	.owner = THIS_MODULE,
	.dai_link = cygnus_dai_links,
	.num_links = ARRAY_SIZE(cygnus_dai_links),
	.probe     = cygnus_audiobase_card_probe,
};

static int cygnus_aud_base_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &cygnus_audio_card;
	struct snd_soc_pcm_runtime *rtd;
	struct card_state_data *card_data;
	struct device_node *link_np;
	char name[PROP_LEN_MAX];
	int ret = 0;
	int i;

	dev_dbg(&pdev->dev, "Enter %s\n", __func__);

	card->dev = &pdev->dev;

	card_data = devm_kzalloc(&pdev->dev, sizeof(*card_data), GFP_KERNEL);
	if (card_data == NULL)
		return -ENOMEM;

	card_data->card_clocking_mode = 0;

	for (i = 0; i < ARRAY_SIZE(cygnus_dai_links); i++) {
		snprintf(name, PROP_LEN_MAX, "link%d", i);
		link_np = of_get_child_by_name(pdev->dev.of_node, name);
		if (!link_np) {
			dev_err(&pdev->dev, "Child node %s missing\n", name);
			return -EINVAL;
		}
		ret = bcm_card_util_parse_link_node(pdev, link_np,
						&cygnus_dai_links[i]);
		if (ret)
			goto err_exit;
	}

	snd_soc_card_set_drvdata(card, card_data);
	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed: %d\n", ret);
		goto err_exit;
	}

	/* Need early MCLK for wm8994 to be configured */
	rtd = snd_soc_get_pcm_runtime(card, cygnus_dai_links[0].name);
	cygnus_ssp_get_clk(rtd->cpu_dai, 12288000);

err_exit:
	return ret;
}

static int cygnus_aud_base_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct snd_soc_pcm_runtime *rtd;

	rtd = snd_soc_get_pcm_runtime(card, cygnus_dai_links[0].name);
	cygnus_ssp_put_clk(rtd->cpu_dai);

	return 0;
}

static const struct of_device_id cygnus_aud_base_mach_of_match[] = {
	{.compatible = "brcm,bcm958305-aud-base-machine"},
	{ },
};
MODULE_DEVICE_TABLE(of, cygnus_aud_base_mach_of_match);

static struct platform_driver bcm_cygnus_aud_base_driver = {
	.driver = {
		.name = "bcm958305-aud-base-machine",
		.pm = &snd_soc_pm_ops,
		.of_match_table = cygnus_aud_base_mach_of_match,
	},
	.probe  = cygnus_aud_base_probe,
	.remove = cygnus_aud_base_remove,
};

module_platform_driver(bcm_cygnus_aud_base_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("ASoC machine driver for bcm958305 audio base");
MODULE_LICENSE("GPL v2");
