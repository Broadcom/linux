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

/*
 * This is the ASoC machine file for the Cygnus BCM958305_AK4458 board.
 * The board consists of a Cygnus SoC, the wm8804 SPDIF transceiver and
 * the AK4458 codec.
 */
#include <asm/mach-types.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "cygnus-ssp.h"

#include "../codecs/wm8804.h"
#include "bcm-card-utils.h"

/* 1 = tdm128 mode: 4 x 32 bit channels per data line.  Up to 216 kHz */
/* 2 = tdm256 mode: 8 x 32 bit channels per data line.  Up to 108 kHz */
/* 3 = tdm512 mode: 16 x 32 bit channels per data line.  Up to 54 kHz */
#define DEFAULT_TDM	2

struct cygnus_audio_data {
	unsigned int ak4458_tdm;
};

static int cygnus_hw_params_wm8804(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->card->dev;
	unsigned int mclk_freq = 0;

	dev_dbg(dev, "Enter %s\n", __func__);

	mclk_freq = 256 * params_rate(params);

	if (mclk_freq > 49152000)
		return -EINVAL;

	return 0;
}

static struct snd_soc_ops cygnus_ops_wm8804 = {
	.hw_params = cygnus_hw_params_wm8804,
};

static int cygnus_init_wm8804(struct snd_soc_pcm_runtime *rtd)
{
	struct device *dev = rtd->card->dev;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret = 0;

	/* Set MCLK output to 256fs */
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8804_MCLK_DIV,
				     WM8804_MCLKDIV_256FS);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_clkdiv codec_dai\n",
			__func__);
		return ret;
	}

	/* Set PLL to 11.7888 MHz */
	ret = snd_soc_dai_set_pll(codec_dai, 0, 0, 12000000, 11788800);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_clkdiv codec_dai\n",
			__func__);
	}

	return ret;
}

static int cygnus_hw_params_ak4458(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->card->dev;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct cygnus_audio_data *card_data =
		snd_soc_card_get_drvdata(rtd->card);
	int ret = 0;
	unsigned int bits_per_frame = 0;
	unsigned int mclk_freq = 0;
	unsigned int channels = 0;
	unsigned int width = 0;
	unsigned int mask = 0;
	int slots = 0;
	int i;

	dev_dbg(dev, "Enter %s\n", __func__);

	bits_per_frame = 64 << card_data->ak4458_tdm;

	/* Cygnus' mclk must be twice the bit clock */
	mclk_freq = 2 * bits_per_frame * params_rate(params);
	if (mclk_freq > 49152000)
		return -EINVAL;

	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
				mclk_freq, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk\n",
			__func__);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, CYGNUS_SSP_CLKSRC_PLL,
				mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk\n",
			__func__);
		return ret;
	}

	slots = 2 << card_data->ak4458_tdm;
	channels = params_channels(params);

	/* ak4458 only supports 32 bit wide slots */
	width = 32;

	/*
	 * AK4458 does not reference the mask, so this is only useful
	 * for Cygnus
	 */
	for (i = 0; i < channels; i++)
		mask |= BIT(i);

	dev_dbg(dev, "%s Set TDM mode: mask 0x%x slots %d width %d\n",
		__func__, mask, slots, width);
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
	}
	return ret;
}

static struct snd_soc_ops cygnus_ops_ak4458 = {
	.hw_params = cygnus_hw_params_ak4458,
};

/* Machine DAPM widgets */
static const struct snd_soc_dapm_widget ak4458_dapm_widgets[] = {

	/* wm8804 widgets */
	SND_SOC_DAPM_LINE("Wolfson SPDIF In", NULL),

	/* ak4458 widgets */
	SND_SOC_DAPM_LINE("AK4458 DAC1 Line Output", NULL),
	SND_SOC_DAPM_LINE("AK4458 DAC2 Line Output", NULL),
	SND_SOC_DAPM_LINE("AK4458 DAC3 Line Output", NULL),
	SND_SOC_DAPM_LINE("AK4458 DAC4 Line Output", NULL),
};

/* Machine DAPM routing */
static const struct snd_soc_dapm_route ak4458_audio_map[] = {

	/* wm8804 routing */
	{"SPDIF In", NULL, "Wolfson SPDIF In"},

	/* ak4458 routing */
	{"AK4458 DAC1 Line Output", NULL, "AOUTL1P"},
	{"AK4458 DAC1 Line Output", NULL, "AOUTL1N"},
	{"AK4458 DAC1 Line Output", NULL, "AOUTR1P"},
	{"AK4458 DAC1 Line Output", NULL, "AOUTR1N"},

	{"AK4458 DAC2 Line Output", NULL, "AOUTL2P"},
	{"AK4458 DAC2 Line Output", NULL, "AOUTL2N"},
	{"AK4458 DAC2 Line Output", NULL, "AOUTR2P"},
	{"AK4458 DAC2 Line Output", NULL, "AOUTR2N"},

	{"AK4458 DAC3 Line Output", NULL, "AOUTL3P"},
	{"AK4458 DAC3 Line Output", NULL, "AOUTL3N"},
	{"AK4458 DAC3 Line Output", NULL, "AOUTR3P"},
	{"AK4458 DAC3 Line Output", NULL, "AOUTR3N"},

	{"AK4458 DAC4 Line Output", NULL, "AOUTL4P"},
	{"AK4458 DAC4 Line Output", NULL, "AOUTL4N"},
	{"AK4458 DAC4 Line Output", NULL, "AOUTR4P"},
	{"AK4458 DAC4 Line Output", NULL, "AOUTR4N"},
};

static int cygnus_hw_params_spdif(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->card->dev;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int mclk_freq = 0;
	int ret = 0;

	dev_dbg(dev, "Enter %s\n", __func__);

	switch (params_rate(params)) {
	case 32000:
	case 48000:
	case 96000:
		mclk_freq = 12288000;
		break;
	case 192000:
		mclk_freq = 24576000;
		break;
	case 44100:
		mclk_freq = 5644800;
		break;
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
	}

	return ret;
}

static struct snd_soc_ops cygnus_ops_spdif = {
	.hw_params = cygnus_hw_params_spdif,
};

/* digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link cygnus_dai_links[] = {
{
	.ops = &cygnus_ops_ak4458,
},
{
	.ops = &cygnus_ops_spdif,
},
{
	.ops = &cygnus_ops_wm8804,
	.init = cygnus_init_wm8804,
},
};

/* Audio machine driver */
static struct snd_soc_card cygnus_audio_card = {
	.name = "bcm-cygnus-ak4458",
	.owner = THIS_MODULE,
	.dai_link = cygnus_dai_links,
	.num_links = ARRAY_SIZE(cygnus_dai_links),
	.dapm_widgets = ak4458_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ak4458_dapm_widgets),
	.dapm_routes = ak4458_audio_map,
	.num_dapm_routes = ARRAY_SIZE(ak4458_audio_map),
	.fully_routed = true,
};

static int cygnus_ak4458_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &cygnus_audio_card;
	struct cygnus_audio_data *card_data;
	struct device_node *link_np;
	char name[PROP_LEN_MAX];
	int ret = 0;
	int i;

	dev_dbg(&pdev->dev, "Enter %s\n", __func__);

	card->dev = &pdev->dev;

	card_data = devm_kzalloc(&pdev->dev, sizeof(*card_data), GFP_KERNEL);
	if (card_data == NULL)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(cygnus_dai_links); i++) {
		snprintf(name, PROP_LEN_MAX, "link%d", i);
		link_np = of_get_child_by_name(pdev->dev.of_node, name);
		if (!link_np) {
			dev_err(&pdev->dev, "Child node %s missing\n", name);
			return -EINVAL;
		}
		ret = bcm_card_util_parse_link_node(pdev, link_np,
						    &cygnus_dai_links[i]);
		if (ret) {
			dev_err(&pdev->dev,
				"Parse link node %d failed: %d\n", i, ret);
			return ret;
		}
	}

	if (of_property_read_u32(np, "bcm,audio-codec-tdm",
				 &card_data->ak4458_tdm)) {
		card_data->ak4458_tdm = DEFAULT_TDM;
	}

	if ((card_data->ak4458_tdm == 0) || (card_data->ak4458_tdm > 3)) {
		dev_warn(&pdev->dev,
			 "Invalid bcm,audio-codec-tdm property %d ignored\n",
			 card_data->ak4458_tdm);
		card_data->ak4458_tdm = DEFAULT_TDM;
	}

	snd_soc_card_set_drvdata(card, card_data);

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",
			ret);
	}

	return ret;
}

static const struct of_device_id cygnus_ak4458_mach_of_match[] = {
	{.compatible = "brcm,bcm958305-ak4458-machine"},
	{ },
};
MODULE_DEVICE_TABLE(of, cygnus_ak4458_mach_of_match);

static struct platform_driver bcm_cygnus_ak4458_mach_driver = {
	.driver = {
		.name = "bcm958305-ak4458-machine",
		.pm = &snd_soc_pm_ops,
		.of_match_table = cygnus_ak4458_mach_of_match,
	},
	.probe  = cygnus_ak4458_probe,
};

module_platform_driver(bcm_cygnus_ak4458_mach_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("ALSA SoC for Cygnus APs");
MODULE_LICENSE("GPL v2");
