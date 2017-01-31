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
 * This is the ASoC machine file for the Cygnus Enterprise reference phone
 * The board consists of a Cygnus 911360 SoC and a CS42L73 codec
 * wired onto i2s0
 */
#include <asm/mach-types.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "../codecs/cs42l73.h"
#include "cygnus-ssp.h"

struct bcm_cygvoipphone_data {
	struct gpio_desc *gpio_handsfree_amp_en;
	unsigned int  bt_bclk_bps;
	bool bt_tdm_mode;
};

static int bcm_cygvoipphone_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, int clkid)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->card->dev;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int mclk_freq = 0;
	int ret = 0;

	/*
	 * The CS42L73 has max frame rate of 48 kHz
	 * Cygnus can not create 8kHz frame rate on i2s from 12.288 MHz
	 * clock.  Need to use slower clock.
	 */
	switch (params_rate(params)) {
	case  8000:
		mclk_freq = 6144000;
		break;
	case 16000:
	case 32000:
	case 48000:
		mclk_freq = 12288000;
		break;

	case 11025:
	case 22050:
	case 44100:
		mclk_freq = 5644800;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, clkid, mclk_freq,
				SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk %u\n",
			__func__, mclk_freq);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, CYGNUS_SSP_CLKSRC_PLL,
				mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk\n", __func__);
		return ret;
	}

	return 0;
}

static int bcm_cygvoipphone_hw_params_xsp(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return bcm_cygvoipphone_hw_params(substream, params,
					CS42L73_CLKID_MCLK2);
}

static struct snd_soc_ops bcm_cygvoipphone_ops_xsp = {
	.hw_params = bcm_cygvoipphone_hw_params_xsp,
};

static int bcm_cygvoipphone_hw_params_bluetooth(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->card->dev;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct bcm_cygvoipphone_data *card_data =
				snd_soc_card_get_drvdata(soc_card);
	unsigned int channels = 0;
	unsigned int width = 0;
	unsigned int bits_per_frame = 0;
	unsigned int mask = 0;
	unsigned int rate = 0;
	unsigned int mclk_freq = 0;
	int slots;
	int ret = 0;

	if (card_data->bt_tdm_mode == true)
		mclk_freq = 12288000;
	else
		mclk_freq = 6144000;

	ret = snd_soc_dai_set_sysclk(cpu_dai, CYGNUS_SSP_CLKSRC_PLL,
				mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk\n", __func__);
		return ret;
	}

	/* Nothing else to do if we are not in TDM mode */
	if (!card_data->bt_tdm_mode)
		return 0;

	/*
	 * The Bluetooth chip wants a constant bps, so half the number of bits
	 * per frame when we do wideband (16 kHz)
	 */
	rate = params_rate(params);
	bits_per_frame = card_data->bt_bclk_bps / rate;

	/*
	 * The BT codec driver will enforce a width of 16 and a channel
	 * count of 2.  Nonetheless, extract the info from the params rather
	 * than hardcoding it.
	 */
	channels = params_channels(params);
	width = snd_pcm_format_physical_width(params_format(params));
	slots = bits_per_frame / width;

	/* Set a bit for each valid slot */
	mask = (1 << channels) - 1;

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, mask, mask, slots, width);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_tdm_slot\n", __func__);
		return ret;
	}
	dev_dbg(dev, "%s set_tdm_slot: mask 0x%x slots %d slot width %d\n",
		__func__, mask, slots, width);

	return 0;
}

static struct snd_soc_ops bcm_cygvoipphone_ops_bluetooth = {
	.hw_params = bcm_cygvoipphone_hw_params_bluetooth,
};

static int handsfree_spk_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm_context = w->dapm;
	struct snd_soc_card *soc_card = dapm_context->card;
	struct bcm_cygvoipphone_data *card_data =
		snd_soc_card_get_drvdata(soc_card);

	/* Set gpio to enable the op amp that will drive the handsfree jack */
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		gpiod_set_value(card_data->gpio_handsfree_amp_en, 1);
		dev_dbg(soc_card->dev, "%s speaker on\n", __func__);
	} else {
		gpiod_set_value(card_data->gpio_handsfree_amp_en, 0);
		dev_dbg(soc_card->dev, "%s speaker off\n", __func__);
	}

	return 0;
}

/* Machine DAPM */
static const struct snd_soc_dapm_widget dapm_widgets[] = {
	/* Outputs */
	SND_SOC_DAPM_HP("Headset Spk", NULL),
	SND_SOC_DAPM_HP("Handset Spk", NULL),
	SND_SOC_DAPM_LINE("Handsfree Spk", handsfree_spk_event),

	/* Inputs */
	SND_SOC_DAPM_MIC("Handset Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Handsfree Mic", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Routings for outputs */
	{"Headset Spk", NULL, "SPKOUT"},
	{"Handset Spk", NULL, "EAROUT"},
	{"Handsfree Spk", NULL, "SPKLINEOUT"},

	/* Routings for inputs */
	{"MIC1", NULL, "Headset Mic"},
	{"MIC2", NULL, "Handset Mic"},
	{"Headset Mic", NULL, "MIC1 Bias"},
	{"Handset Mic", NULL, "MIC2 Bias"},

	/* handsfree has 2 digital mics */
	{"DMICA", NULL, "Handsfree Mic"},
	{"DMICB", NULL, "Handsfree Mic"},
};

/*  digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link cygvoipphone_dai_links[] = {
{
	.name = "CS42L73_XSP",
	.stream_name = "CS42L73_XSP",

	.ops = &bcm_cygvoipphone_ops_xsp,
},
{
	.name = "bluetooth",
	.stream_name = "bluetooth",

	.ops = &bcm_cygvoipphone_ops_bluetooth,
},
};

/* Audio machine driver */
static struct snd_soc_card bcm_cygvoipphone_card = {
	.name = "bcm-911360_entphn",
	.owner = THIS_MODULE,
	.dai_link = cygvoipphone_dai_links,
	.num_links = ARRAY_SIZE(cygvoipphone_dai_links),

	.dapm_widgets = dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(dapm_widgets),
	.dapm_routes = audio_map,
	.num_dapm_routes = ARRAY_SIZE(audio_map),
	.fully_routed = true,
};

int parse_link_node_common(struct platform_device *pdev,
		struct device_node *link_np,
		struct snd_soc_dai_link *dai_link)
{
	struct device *dev = &pdev->dev;
	struct device_node *codec_np;
	struct device_node *cpu_np;
	unsigned int daifmt;
	int ret = 0;

	codec_np = of_get_child_by_name(link_np, "codec");
	if (!codec_np) {
		dev_err(dev, "Could not find codec child node\n");
		ret = -EINVAL;
		goto err_exit;
	}

	dai_link->codec_of_node = of_parse_phandle(codec_np, "sound-dai", 0);
	if (dai_link->codec_of_node == NULL) {
		dev_err(dev, "Property sound-dai missing or invalid\n");
		ret = -EINVAL;
		goto err_exit;
	}

	ret = snd_soc_of_get_dai_name(codec_np, &dai_link->codec_dai_name);
	if (ret < 0) {
		dev_err(dev, "ERROR snd_soc_of_get_dai_name for codec node\n");
		ret = -EINVAL;
		goto err_exit;
	}

	cpu_np = of_get_child_by_name(link_np, "cpu");
	if (!cpu_np) {
		dev_err(dev, "Could not find cpu child node\n");
		ret = -EINVAL;
		goto err_exit;
	}

	dai_link->cpu_of_node = of_parse_phandle(cpu_np, "sound-dai", 0);
	if (dai_link->cpu_of_node == NULL) {
		dev_err(dev, "Property sound-dai missing or invalid\n");
		ret = -EINVAL;
		goto err_exit;
	}

	ret = snd_soc_of_get_dai_name(cpu_np, &dai_link->cpu_dai_name);
	if (ret < 0) {
		dev_err(dev, "ERROR snd_soc_of_get_dai_name for cpu node\n");
		ret = -EINVAL;
		goto err_exit;
	}

	/*
	 * This function does not seem to parse the bitclock-master and
	 * frame-master correctly. It seems the function only checks to see
	 * if the property has has any value whatsoever disregarding which
	 * node (codec or cpu) the property is referencing.
	 * For our card all links have codec as slave.
	 * The only other example usage of this function is in
	 * asoc_simple_card_parse_daifmt.  In that usage the calling function
	 * actually "corrects" these bits.
	 */
	daifmt = snd_soc_of_parse_daifmt(link_np, NULL, NULL, NULL);
	daifmt &= ~SND_SOC_DAIFMT_MASTER_MASK;
	daifmt |= SND_SOC_DAIFMT_CBS_CFS;
	dai_link->dai_fmt = daifmt;

	ret = 0;

err_exit:
	if (codec_np)
		of_node_put(codec_np);
	if (cpu_np)
		of_node_put(cpu_np);

	return ret;
}

static int parse_link0_node(struct platform_device *pdev,
			struct snd_soc_dai_link *dai_link,
			struct bcm_cygvoipphone_data *card_data)
{
	struct device *dev = &pdev->dev;
	struct device_node *link_np;
	int ret;

	link_np = of_get_child_by_name(dev->of_node, "ipp-primary-link");
	if (!link_np) {
		dev_err(dev, "Child node ipp-primary-link missing\n");
		return -EINVAL;
	}

	ret = parse_link_node_common(pdev, link_np, &cygvoipphone_dai_links[0]);

	return ret;
}


#define  PROP_NAME_BT_BCLK "cypress,bt-voice-tdm-bps"

static int parse_link1_node(struct platform_device *pdev,
			struct snd_soc_dai_link *dai_link,
			struct bcm_cygvoipphone_data *card_data)
{
	struct device *dev = &pdev->dev;
	struct device_node *link_np;
	unsigned int format;
	int ret;

	link_np = of_get_child_by_name(dev->of_node,
					"ipp-bluetooth-voice-link");
	if (!link_np) {
		dev_err(dev, "Node ipp-bluetooth-voice-link missing\n");
		return -EINVAL;
	}

	ret = parse_link_node_common(pdev, link_np, dai_link);
	if (ret)
		goto err_exit;

	format = (dai_link->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK);
	card_data->bt_tdm_mode = false;
	if (format == SND_SOC_DAIFMT_DSP_A) {
		card_data->bt_tdm_mode = true;

		ret = of_property_read_u32(dai_link->codec_of_node,
					PROP_NAME_BT_BCLK,
					&card_data->bt_bclk_bps);
		if (ret) {
			dev_err(dev,
				"Could not find %s property in device tree\n",
				PROP_NAME_BT_BCLK);
			ret = -EINVAL;
			goto err_exit;
		}
	}

err_exit:
	return ret;
}

static int cygvoipphone_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &bcm_cygvoipphone_card;
	struct bcm_cygvoipphone_data *card_data;
	struct device_node *platform_np;
	struct gpio_desc *gpio;
	int ret = 0;

	card->dev = &pdev->dev;

	card_data = devm_kzalloc(&pdev->dev, sizeof(*card_data), GFP_KERNEL);
	if (card_data == NULL)
		return -ENOMEM;

	parse_link0_node(pdev, &cygvoipphone_dai_links[0], card_data);
	parse_link1_node(pdev, &cygvoipphone_dai_links[1], card_data);

	platform_np = of_parse_phandle(np, "brcm,cygnus-pcm", 0);
	if (platform_np == NULL) {
		dev_err(&pdev->dev,
			"Property brcm,cygnus-pcm missing or invalid\n");
		ret = -EINVAL;
		goto err_exit;
	}

	cygvoipphone_dai_links[0].platform_of_node = platform_np;
	cygvoipphone_dai_links[1].platform_of_node = platform_np;

	/*
	 * Get the gpio number from the device tree for the gpio used
	 * to control the handsfree amplifier.
	 */
	gpio = devm_gpiod_get(&pdev->dev,
				"brcm,handsfree-amp-en", GPIOD_OUT_LOW);
	if (IS_ERR(gpio)) {
		dev_err(&pdev->dev, "Error with brcm,handsfree-amp-en\n");
		ret = PTR_ERR(gpio);
		goto err_exit;
	}
	card_data->gpio_handsfree_amp_en = gpio;

	snd_soc_card_set_drvdata(card, card_data);

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed: %d\n", ret);
		goto err_exit;
	}

	return 0;

err_exit:
	return ret;
}

static int cygvoipphone_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id cygnus_mach_of_match[] = {
	{.compatible = "brcm,bcm911360_entphn-machine", },
	{ },
};
MODULE_DEVICE_TABLE(of, cygnus_mach_of_match);

static struct platform_driver bcm_cygnus_driver = {
	.driver = {
		.name = "brcm-bcm911360_entphn-machine",
		.of_match_table = cygnus_mach_of_match,
	},
	.probe  = cygvoipphone_probe,
	.remove = cygvoipphone_remove,
};

module_platform_driver(bcm_cygnus_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("ALSA SoC for Cygnus APs");
MODULE_LICENSE("GPL v2");
