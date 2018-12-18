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
/*
 * This driver is very similar to the "dummy" driver found in the ASoC
 * framework, but we are allowing this driver to be continuous frame rate up to
 * a maximum of 384 kHz.  It is meant to model a piece of hardware that may
 * be attached to a port for such uses as testing and development.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/pcm.h>

static struct snd_soc_component_driver bcm_dummy_codec;

#define DUMMY_FORMATS	(SNDRV_PCM_FMTBIT_S8 | \
			SNDRV_PCM_FMTBIT_U8 | \
			SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_U16_LE | \
			SNDRV_PCM_FMTBIT_S24_LE | \
			SNDRV_PCM_FMTBIT_U24_LE | \
			SNDRV_PCM_FMTBIT_S32_LE | \
			SNDRV_PCM_FMTBIT_U32_LE | \
			SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE)

static struct snd_soc_dai_driver bcm_dummy_dai = {
	.name		= "bcm-dummy-codec-dai",
	.playback	= {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 384,
		.rates		= SNDRV_PCM_RATE_CONTINUOUS,
		.rate_min	= 0,
		.rate_max	= 384000,
		.formats	= DUMMY_FORMATS,
	},
	.capture	= {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 384,
		.rates		= SNDRV_PCM_RATE_CONTINUOUS,
		.rate_min	= 0,
		.rate_max	= 384000,
		.formats	= DUMMY_FORMATS,
	},
};

static int bcm_dummy_probe(struct platform_device *pdev)
{
	return devm_snd_soc_register_component(&pdev->dev, &bcm_dummy_codec,
			&bcm_dummy_dai, 1);
}

static const struct of_device_id bcm_dummy_of_match[] = {
	{.compatible = "brcm,dummy_codec", },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm_dummy_of_match);

static struct platform_driver bcm_dummy_codec_driver = {
	.probe		= bcm_dummy_probe,
	.driver		= {
		.name	= "bcm-dummy-codec",
		.of_match_table = of_match_ptr(bcm_dummy_of_match),
	},
};

module_platform_driver(bcm_dummy_codec_driver);

MODULE_DESCRIPTION("BCM Dummy Codec driver");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
