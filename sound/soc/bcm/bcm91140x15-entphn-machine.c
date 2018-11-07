// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2018 Broadcom
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "bcm-card-utils.h"

#define MAX_PORTS  5
#define PROP_LEN_MAX  80

struct card_state_data {
	struct snd_soc_dai_link  bcm91140x15_entphn_dai_links[MAX_PORTS];
	struct gpio_desc *gpio_ext_headset_amp_en;
};

static int headset_spk_evt(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *k,
			   int event)
{
	struct snd_soc_dapm_context *dapm_context = w->dapm;
	struct snd_soc_card *soc_card = dapm_context->card;
	struct card_state_data *card_data = snd_soc_card_get_drvdata(soc_card);
	struct gpio_desc *gpio = card_data->gpio_ext_headset_amp_en;

	/* Enable the op amp that will drive the heasdset speaker */
	if (SND_SOC_DAPM_EVENT_ON(event))
		gpiod_set_value(gpio, 1);
	else
		gpiod_set_value(gpio, 0);

	return 0;
}

/* Machine DAPM */
static const struct snd_soc_dapm_widget audioh_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone jack", NULL),
	SND_SOC_DAPM_SPK("Handsfree", NULL),
	SND_SOC_DAPM_SPK("Handset Earpiece", NULL),
	SND_SOC_DAPM_SPK("Mono Headset Earpiece", headset_spk_evt),

	SND_SOC_DAPM_MIC("Analog Mic1", NULL),
	SND_SOC_DAPM_MIC("Analog Mic2", NULL),
	SND_SOC_DAPM_MIC("Analog Mic3", NULL),

	SND_SOC_DAPM_MIC("Digital Mic1", NULL),
	SND_SOC_DAPM_MIC("Digital Mic2", NULL),
	SND_SOC_DAPM_MIC("Digital Mic3", NULL),
	SND_SOC_DAPM_MIC("Digital Mic4", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Outputs */
	{"Handsfree", NULL, "LSPK_OUT"},
	{"Handset Earpiece", NULL, "EP_OUT"},
	{"Mono Headset Earpiece", NULL, "EP_OUT2"},
	{"Headphone jack", NULL, "HS_OUT_L"},
	{"Headphone jack", NULL, "HS_OUT_R"},

	/* Inputs */
	{"AMIC1", NULL, "Analog Mic1"},
	{"AUXMIC1", NULL, "Analog Mic2"},
	{"AMIC3", NULL, "Analog Mic3"},
	{"DMIC1", NULL, "Digital Mic1"},
	{"DMIC2", NULL, "Digital Mic2"},
	{"DMIC3", NULL, "Digital Mic3"},
	{"DMIC4", NULL, "Digital Mic4"},
};

/* AudioH requires 8 x 32 bit slots */
#define AUDIOH_NUM_TDM_SLOTS   8
#define AUDIOH_TDM_SLOT_WIDTH  32

/* We will be using 4 of these slots */
#define ACTIVE_TDM_SLOTS       4

static int bcm91140x15_entphn_audioh_init(struct snd_soc_pcm_runtime *rtd)
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

static struct snd_soc_card bcm91140x15_entphn_audio_card = {
	.name = "bcm91140x15_entphn-card",
	.owner = THIS_MODULE,

	.dapm_widgets = audioh_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(audioh_dapm_widgets),
	.dapm_routes = audio_map,
	.num_dapm_routes = ARRAY_SIZE(audio_map),
};

static int bcm91140x15_entphn_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &bcm91140x15_entphn_audio_card;
	struct card_state_data *card_data;
	struct device_node *link_np;
	char name[PROP_LEN_MAX];
	int ret = 0;
	int linknum;

	dev_info(&pdev->dev, "Enter %s\n", __func__);

	card->dev = &pdev->dev;

	card_data = devm_kzalloc(&pdev->dev, sizeof(*card_data), GFP_KERNEL);
	if (card_data == NULL)
		return -ENOMEM;

	card->dai_link = card_data->bcm91140x15_entphn_dai_links;
	card->num_links = 1;

	linknum = 0;
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

	card->dai_link[linknum].init = bcm91140x15_entphn_audioh_init;

	/* External headset amplifier enable */
	card_data->gpio_ext_headset_amp_en =
				devm_gpiod_get_optional(&pdev->dev,
							"brcm,ext-headset-amp-en",
							GPIOD_OUT_LOW);

	if (IS_ERR(card_data->gpio_ext_headset_amp_en)) {
		dev_err(&pdev->dev, "Invalid gpio for headset amp enable\n");
		ret = PTR_ERR(card_data->gpio_ext_headset_amp_en);
		goto err_exit;
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


static const struct of_device_id bcm91140x15_entphn_mach_of_match[] = {
	{.compatible = "brcm,bcm91140x15_entphn-machine"},
	{ },
};
MODULE_DEVICE_TABLE(of, bcm91140x15_entphn_mach_of_match);

static struct platform_driver bcm_bcm91140x15_entphn_audio_driver = {
	.driver = {
		.name = "bcm91140x15_entphn-machine",
		.pm = &snd_soc_pm_ops,
		.of_match_table = bcm91140x15_entphn_mach_of_match,
	},
	.probe  = bcm91140x15_entphn_probe,
};

module_platform_driver(bcm_bcm91140x15_entphn_audio_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("ASoC machine driver for Omega bcm91140x15_entphn");
MODULE_LICENSE("GPL v2");
