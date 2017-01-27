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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/soc.h>

struct bt_voice_priv {
	bool    tdm_mode;
	unsigned int bclk_bps;
	unsigned int tdm_bits_per_frame;
};

static const struct snd_soc_dapm_widget bt_widgets[] = {
	SND_SOC_DAPM_INPUT("Rx"),
	SND_SOC_DAPM_OUTPUT("Tx"),
};

static const struct snd_soc_dapm_route bt_routes[] = {
	{ "Capture", NULL, "Rx" },
	{ "Tx", NULL, "Playback" },
};

static int bt_voice_set_dai_fmt(struct snd_soc_dai *codec_dai,
					unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct bt_voice_priv *btvoice = snd_soc_codec_get_drvdata(codec);
	int ret  = 0;

	/* Only support slave mode */
	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) !=  SND_SOC_DAIFMT_CBS_CFS) {
		dev_err(codec->dev, "Does not support master mode.\n");
		return -EINVAL;
	}

	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF) {
		dev_err(codec->dev, "Does not support inverted clocks.\n");
		return -EINVAL;
	}

	/*
	 * Check that the requested format is the same as what was configure
	 * via the probe.
	 */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		if (btvoice->tdm_mode)
			ret = -EINVAL;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		if (!btvoice->tdm_mode)
			ret = -EINVAL;
		break;
	default:
		dev_err(codec->dev, "Use only i2s or dsp_a format.\n");
		return -EINVAL;
	}
	return 0;
}

/* slots should be set to 0 if not using tdm mode */
static int bt_voice_set_dai_tdm_slot(struct snd_soc_dai *codec_dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct bt_voice_priv *btvoice = snd_soc_codec_get_drvdata(codec);

	btvoice->tdm_bits_per_frame = slots * slot_width;

	return 0;
}

static int bt_voice_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *codec_dai)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct bt_voice_priv *btvoice = snd_soc_codec_get_drvdata(codec);
	unsigned int bps;
	int rate;
	int ret = 0;

	rate = params_rate(params);

	/*
	 * If doing tdm transfer then check that the transfer is compliant with
	 * the bits per second configuration.
	 */
	if (btvoice->tdm_bits_per_frame) {
		bps = (rate * btvoice->tdm_bits_per_frame);
		if (bps != btvoice->bclk_bps) {
			dev_err(codec->dev,
				"bps %u does not match configred %u\n",
				bps, btvoice->bclk_bps);
			ret = -EINVAL;
		}
	}

	return ret;
}

static const struct snd_soc_dai_ops bt_voice_ops = {
	.set_fmt = bt_voice_set_dai_fmt,
	.set_tdm_slot = bt_voice_set_dai_tdm_slot,
	.hw_params = bt_voice_hw_params,
};

static struct snd_soc_dai_driver bt_voice_dai[] = {
{
	.name = "cyp_bt_voice_codec",
	.playback = {
		.stream_name  = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates        = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
		.formats      = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name  = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates        = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
		.formats      = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &bt_voice_ops,
},
};

static struct snd_soc_codec_driver cyp_bt_voice = {
	.component_driver = {
		.dapm_widgets		= bt_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(bt_widgets),
		.dapm_routes		= bt_routes,
		.num_dapm_routes	= ARRAY_SIZE(bt_routes),
	},
};

static int cyp_btvoice_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bt_voice_priv *btvoice;
	struct device_node *np = pdev->dev.of_node;
	const char *mode_str;
	char prop[80];
	int ret;

	btvoice = devm_kzalloc(dev, sizeof(struct bt_voice_priv), GFP_KERNEL);
	if (!btvoice)
		return -ENOMEM;

	dev_set_drvdata(dev, btvoice);

	snprintf(prop, sizeof(prop), "cypress,bt-voice-link-mode");
	ret = of_property_read_string(np, prop, &mode_str);
	if (ret == 0) {
		if (strcmp(mode_str, "i2s") == 0)
			btvoice->tdm_mode = false;
		else if (strcmp(mode_str, "tdm") == 0)
			btvoice->tdm_mode = true;
		else {
			dev_err(dev, "Invalid value for %s", prop);
			goto err_exit;
		}
	} else {
		dev_err(dev, "Error parsing %s", prop);
		ret = -EINVAL;
		goto err_exit;
	}

	if (btvoice->tdm_mode) {
		snprintf(prop, sizeof(prop), "cypress,bt-voice-tdm-bps");
		ret = of_property_read_u32(np, prop, &btvoice->bclk_bps);
		if (ret) {
			dev_err(&pdev->dev,
				"Could not find %s property in device tree\n",
				prop);
			ret = -EINVAL;
			goto err_exit;
		}
	}

	ret = snd_soc_register_codec(&pdev->dev, &cyp_bt_voice,
					bt_voice_dai, ARRAY_SIZE(bt_voice_dai));
err_exit:
	return ret;
}

static int cyp_btvoice_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id bt_voice_of_match[] = {
	{.compatible = "cypress,bt_voice_codec", },
	{},
};
MODULE_DEVICE_TABLE(of, bt_voice_of_match);
#endif

static struct platform_driver cyp_bt_voice_driver = {
	.driver = {
		.name = "cyp-bt-voice-codec",
		.of_match_table = of_match_ptr(bt_voice_of_match),
	},
	.probe = cyp_btvoice_probe,
	.remove = cyp_btvoice_remove,
};

module_platform_driver(cyp_bt_voice_driver);

MODULE_DESCRIPTION("Codec driver for Cypress BT Voice");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
