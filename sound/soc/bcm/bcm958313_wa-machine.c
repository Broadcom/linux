// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Broadcom
 */
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "bcm-card-utils.h"
#include "cygnus-clk-utils.h"

#define AUDIOH_LINK	0
#define AK4458_LINK	1
#define MAX_LINKS	2

#define PROP_LEN_MAX  80

struct card_state_data {
	struct snd_soc_dai_link  bcm_omega_wa_dai_links[MAX_LINKS];
	struct gpio_desc *gpio_ak4458_reset;
	struct pll_tweak_info tweak_info;
};

static int omega_hw_params_ak4458(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct device *dev = rtd->card->dev;
	unsigned int mclk_freq = (2 * 128 * 48000 * 2);
	unsigned int channels = 0;
	unsigned int width = 32;
	unsigned int mask = 0x0;
	int slots = 8;
	int i;
	int ret;

	channels = params_channels(params);
	for (i = 0; i < channels; i++)
		mask |= BIT(i);

	ret = snd_soc_dai_set_tdm_slot(codec_dai, mask, mask, slots, width);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_tdm_slot codec_dai\n",
			__func__);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, mask, mask, slots, width);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_tdm_slot cpu_dai\n",
			__func__);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk\n",
			__func__);
		return ret;
	}

	return 0;
}

#define MAX_PREFIX  40

/* This frequency (3612672000 Hz) will only be good for the 48 kHz
 * range of frame rates.
 */
#define PLL_TWEAKING_FREQ  3612672000UL

static int bcm_omega_wa_init_ak4458(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct card_state_data *card_data = snd_soc_card_get_drvdata(rtd->card);
	char prefix[MAX_PREFIX];
	int ret;

	snprintf(prefix, MAX_PREFIX, "Link %d (AK4458)", AK4458_LINK);

	/*
	 * For testing and as an example add in the ssp kcontrols that
	 * allow us to tweak the pll.  This could be done on any port
	 * but this one has the headphone jack so it is easiest.
	 */
	card_data->tweak_info.cpu_dai = rtd->cpu_dai;
	ret = cygnus_ssp_pll_tweak_initialize(&card_data->tweak_info, prefix);
	if (ret) {
		dev_err(card->dev, "Could not init Tweak control\n");
		return ret;
	}

	/*
	 * The mixer tweaking interface will control the ppb adjustment.  We
	 * need to first set the base rate which the adjustment will be applied
	 * to. This frequency should be carefully chosen so the vco will have
	 * enough glitch free tweaking range. Consult the SoCs audio clock
	 * specifics to make this decision.
	 */
	card_data = snd_soc_card_get_drvdata(rtd->card);
	ret = cygnus_ssp_pll_tweak_update(&card_data->tweak_info,
					  PLL_TWEAKING_FREQ);

	return ret;
}

static struct snd_soc_ops bcm_omega_wa_ops_ak4458 = {
	.hw_params = omega_hw_params_ak4458,
};

/* AudioH requires 8 x 32 bit slots */
#define AUDIOH_NUM_TDM_SLOTS   8
#define AUDIOH_TDM_SLOT_WIDTH  32

/* We will be using 4 of these slots */
#define ACTIVE_TDM_SLOTS       4

static int bcm_omega_wa_audioh_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct device *dev = rtd->card->dev;
	unsigned int mask;
	int ret;

	mask = BIT(ACTIVE_TDM_SLOTS) - 1;
	ret = snd_soc_dai_set_tdm_slot(cpu_dai, mask, mask,
				AUDIOH_NUM_TDM_SLOTS, AUDIOH_TDM_SLOT_WIDTH);
	if (ret < 0)
		dev_err(dev, "%s Failed snd_soc_dai_set_tdm_slot\n", __func__);

	return ret;
}

/* Machine DAPM */
static const struct snd_soc_dapm_widget wa_card_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone jack", NULL),
	SND_SOC_DAPM_SPK("Handsfree", NULL),
	SND_SOC_DAPM_SPK("Handset Earpiece", NULL),

	SND_SOC_DAPM_MIC("Analog Mic1", NULL),
	SND_SOC_DAPM_MIC("Analog Mic2", NULL),
	SND_SOC_DAPM_MIC("Analog Mic3", NULL),

	SND_SOC_DAPM_MIC("Digital Mic1", NULL),
	SND_SOC_DAPM_MIC("Digital Mic2", NULL),
	SND_SOC_DAPM_MIC("Digital Mic3", NULL),
	SND_SOC_DAPM_MIC("Digital Mic4", NULL),

	SND_SOC_DAPM_HP("AK_Chan1 jack", NULL),
	SND_SOC_DAPM_HP("AK_Chan2 jack", NULL),
	SND_SOC_DAPM_HP("AK_Chan3 jack", NULL),
	SND_SOC_DAPM_HP("AK_Chan4 jack", NULL),
};

static const struct snd_soc_dapm_route wa_card_dapm_routes[] = {
	/* Outputs AudioH codec */
	{"Headphone jack", NULL, "HS_OUT_L"},
	{"Headphone jack", NULL, "HS_OUT_R"},

	/* Inputs  AudioH codec */
	{"AMIC1", NULL, "Analog Mic1"},
	{"AUXMIC1", NULL, "Analog Mic2"},
	{"AMIC3", NULL, "Analog Mic3"},
	{"DMIC1", NULL, "Digital Mic1"},
	{"DMIC2", NULL, "Digital Mic2"},
	{"DMIC3", NULL, "Digital Mic3"},
	{"DMIC4", NULL, "Digital Mic4"},

	/* Outputs AK4458 */
	{"AK_Chan1 jack", NULL, "AK4458 AOUTA"},
	{"AK_Chan2 jack", NULL, "AK4458 AOUTB"},
	{"AK_Chan3 jack", NULL, "AK4458 AOUTC"},
	{"AK_Chan4 jack", NULL, "AK4458 AOUTD"},
};

static int bcm_omega_wa_card_probe(struct snd_soc_card *card)
{
	struct pinctrl  *pinctrl;
	struct pinctrl_state *state_default;
	struct pinctrl_state *state_gpio;

	struct device *dev;
	int ret;
	struct card_state_data *card_data = snd_soc_card_get_drvdata(card);
	struct gpio_desc *gpio = card_data->gpio_ak4458_reset;

	dev = card->dev;

	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		dev_err(dev, "Failed getting pinctrl %d.\n", ret);
		return ret;
	}

	state_default = pinctrl_lookup_state(pinctrl, "default");
	if (IS_ERR(state_default)) {
		ret = PTR_ERR(state_default);
		dev_err(dev, "Failed looking up pinctrl state %d.\n", ret);
		return ret;
	}

	state_gpio = pinctrl_lookup_state(pinctrl, "gpio");
	if (IS_ERR(state_gpio)) {
		ret = PTR_ERR(state_gpio);
		dev_err(dev, "Failed looking up pinctrl state %d.\n", ret);
		return ret;
	}

	ret = pinctrl_select_state(pinctrl, state_gpio);
	if (ret < 0) {
		dev_err(dev, "Failed selecting pinctrl state gpio.\n");
		return ret;
	}

	/* power down codec */
	gpiod_set_raw_value_cansleep(gpio, 0);
	usleep_range(1000, 2000);

	/* power up codec */
	gpiod_set_raw_value_cansleep(gpio, 1);
	usleep_range(1000, 2000);

	/* set pins back to i2c */
	ret = pinctrl_select_state(pinctrl, state_default);
	if (ret < 0) {
		dev_err(dev, "Failed selecting pinctrl state i2c.\n");
		return ret;
	}

	return 0;
}

/* Audio machine driver */
static struct snd_soc_card bcm_omega_wa_audio_card = {
	.name = "bcm-omega-wa-card",
	.owner = THIS_MODULE,

	.dapm_widgets = wa_card_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(wa_card_dapm_widgets),

	.dapm_routes = wa_card_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(wa_card_dapm_routes),

	.probe =  bcm_omega_wa_card_probe,
};

static int bcm_omega_wa_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &bcm_omega_wa_audio_card;
	struct card_state_data *card_data;
	struct device_node *link_np;
	char name[PROP_LEN_MAX];
	int ret = 0;
	int linknum, linktotal;

	dev_info(&pdev->dev, "Enter %s\n", __func__);

	card->dev = &pdev->dev;

	linktotal = of_get_child_count(pdev->dev.of_node);
	if ((linktotal < 1) || (linktotal > MAX_LINKS)) {
		dev_err(&pdev->dev,
			"child nodes is %d. Must be between 1 and %d\n",
			linktotal, MAX_LINKS);
		return -EINVAL;
	}

	card_data = devm_kzalloc(&pdev->dev, sizeof(*card_data), GFP_KERNEL);
	if (card_data == NULL)
		return -ENOMEM;

	card->dai_link = card_data->bcm_omega_wa_dai_links;
	card->num_links = linktotal;

	card_data->gpio_ak4458_reset = devm_gpiod_get(&pdev->dev,
				"brcm,reset-ak4458", GPIOD_OUT_LOW);
	if (IS_ERR(card_data->gpio_ak4458_reset)) {
		ret = PTR_ERR(card_data->gpio_ak4458_reset);
		dev_err(&pdev->dev, "Invalid gpio for ak4458 reset %d\n", ret);
		return ret;
	}

	for (linknum = 0; linknum < linktotal; linknum++) {
		snprintf(name, PROP_LEN_MAX, "link%d", linknum);
		link_np = of_get_child_by_name(pdev->dev.of_node, name);
		if (!link_np) {
			dev_err(&pdev->dev, "Child node %s missing\n", name);
			return -EINVAL;
		}
		ret = bcm_card_util_parse_link_node(pdev, link_np,
						&card->dai_link[linknum]);
		if (ret)
			goto err_exit;

		if (linknum == AK4458_LINK) {
			card->dai_link[linknum].ops = &bcm_omega_wa_ops_ak4458;
			card->dai_link[linknum].init = bcm_omega_wa_init_ak4458;
		} else if (linknum == AUDIOH_LINK) {
			card->dai_link[linknum].init = bcm_omega_wa_audioh_init;
		}
	}

	snd_soc_card_set_drvdata(card, card_data);
	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed: %d\n", ret);
		goto err_exit;
	}

	return 0;

err_exit:
	return ret;
}

static const struct of_device_id bcm_omega_wa_mach_of_match[] = {
	{.compatible = "brcm,omega-wa-machine"},
	{ },
};
MODULE_DEVICE_TABLE(of, bcm_omega_wa_mach_of_match);

static struct platform_driver bcm_bcm_omega_wa_audio_driver = {
	.driver = {
		.name = "omega-wa-machine",
		.pm = &snd_soc_pm_ops,
		.of_match_table = bcm_omega_wa_mach_of_match,
	},
	.probe  = bcm_omega_wa_probe,
};

module_platform_driver(bcm_bcm_omega_wa_audio_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM958313 Wireless Audio ASoC machine driver");
MODULE_LICENSE("GPL v2");
