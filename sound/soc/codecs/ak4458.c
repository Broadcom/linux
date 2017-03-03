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
 * ak4458.c  --  AK4458 ALSA SoC Audio driver
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

/* AK4458 registers addresses */
#define AK4458_REG_CONTROL1		0x00
#define AK4458_REG_CONTROL2		0x01
#define AK4458_REG_CONTROL3		0x02
#define AK4458_REG_L1CH_ATT		0x03
#define AK4458_REG_R1CH_ATT		0x04
#define AK4458_REG_CONTROL4		0x05
#define AK4458_REG_DSD1			0x06
#define AK4458_REG_CONTROL5		0x07
#define AK4458_REG_SOUND_CONTROL	0x08
#define AK4458_REG_DSD2			0x09
#define AK4458_REG_CONTROL6		0x0a
#define AK4458_REG_CONTROL7		0x0b
#define AK4458_REG_CONTROL8		0x0c
#define AK4458_REG_CONTROL9		0x0d
#define AK4458_REG_CONTROL10		0x0e
#define AK4458_REG_L2CH_ATT		0x0f
#define AK4458_REG_R2CH_ATT		0x10
#define AK4458_REG_L3CH_ATT		0x11
#define AK4458_REG_R3CH_ATT		0x12
#define AK4458_REG_L4CH_ATT		0x13
#define AK4458_REG_R4CH_ATT		0x14

#define AK4458_NUM_REGS			21

#define AK4458_CONTROL1_RSTN		0x01
#define AK4458_CONTROL1_DIF		0x0e
#define AK4458_CONTROL1_DIF_16LSBJ	0x00
#define AK4458_CONTROL1_DIF_20LSBJ	0x02
#define AK4458_CONTROL1_DIF_24MSBJ	0x04
#define AK4458_CONTROL1_DIF_24I2S	0x06
#define AK4458_CONTROL1_DIF_24LSBJ	0x08
#define AK4458_CONTROL1_DIF_32LSBJ	0x0a
#define AK4458_CONTROL1_DIF_32MSBJ	0x0c
#define AK4458_CONTROL1_DIF_32I2S	0x0e
#define AK4458_CONTROL1_ACKS		0x80

#define AK4458_CONTROL2_SMUTE		0x01

#define AK4458_CONTROL3_DP		0x80

#define AK4458_CONTROL6_SDS2		0x10
#define AK4458_CONTROL6_SDS1		0x20
#define AK4458_CONTROL6_TDM		0xc0
#define AK4458_CONTROL6_TDM_SHIFT	0x06

#define AK4458_CONTROL7_SDS0		0x10

/* codec private data */
struct ak4458_priv {
	struct regmap *regmap;
	unsigned int sysclk;
	struct gpio_desc *amp_mute_gpio;
	unsigned int tdm;
	unsigned int sds;
};

static const struct reg_default ak4458_reg_defaults[] = {
	{AK4458_REG_CONTROL1, 0x0c},
	{AK4458_REG_CONTROL2, 0x22},
	{AK4458_REG_CONTROL3, 0x00},
	{AK4458_REG_L1CH_ATT, 0xff},
	{AK4458_REG_R1CH_ATT, 0xff},
	{AK4458_REG_CONTROL4, 0x00},
	{AK4458_REG_DSD1, 0x00},
	{AK4458_REG_CONTROL5, 0x03},
	{AK4458_REG_SOUND_CONTROL, 0x00},
	{AK4458_REG_DSD2, 0x00},
	{AK4458_REG_CONTROL6, 0x0d},
	{AK4458_REG_CONTROL7, 0x0c},
	{AK4458_REG_CONTROL8, 0x00},
	{AK4458_REG_CONTROL9, 0x00},
	{AK4458_REG_CONTROL10, 0x50},
	{AK4458_REG_L2CH_ATT, 0xff},
	{AK4458_REG_R2CH_ATT, 0xff},
	{AK4458_REG_L3CH_ATT, 0xff},
	{AK4458_REG_R3CH_ATT, 0xff},
	{AK4458_REG_L4CH_ATT, 0xff},
	{AK4458_REG_R4CH_ATT, 0xff},
};

static const DECLARE_TLV_DB_SCALE(attenuation_tlv, -12750, 50, 1);

static const char * const ak4458_dem[] = {"44.1kHz", "Off", "48kHz", "32kHz"};
static const struct soc_enum ak4458_enum[] = {
	SOC_ENUM_SINGLE(AK4458_REG_CONTROL2, 1, 4, ak4458_dem),
	SOC_ENUM_SINGLE(AK4458_REG_CONTROL6, 0, 4, ak4458_dem),
	SOC_ENUM_SINGLE(AK4458_REG_CONTROL10, 4, 4, ak4458_dem),
	SOC_ENUM_SINGLE(AK4458_REG_CONTROL10, 6, 4, ak4458_dem),
};

static const struct snd_kcontrol_new ak4458_snd_controls[] = {
	SOC_DOUBLE_R_TLV("DAC1 attenuation", AK4458_REG_L1CH_ATT,
			 AK4458_REG_R1CH_ATT, 0, 255, 0, attenuation_tlv),
	SOC_DOUBLE_R_TLV("DAC2 attenuation", AK4458_REG_L2CH_ATT,
			 AK4458_REG_R2CH_ATT, 0, 255, 0, attenuation_tlv),
	SOC_DOUBLE_R_TLV("DAC3 attenuation", AK4458_REG_L3CH_ATT,
			 AK4458_REG_R3CH_ATT, 0, 255, 0, attenuation_tlv),
	SOC_DOUBLE_R_TLV("DAC4 attenuation", AK4458_REG_L4CH_ATT,
			 AK4458_REG_R4CH_ATT, 0, 255, 0, attenuation_tlv),
	SOC_ENUM("DAC1 de-emphasis mode", ak4458_enum[0]),
	SOC_ENUM("DAC2 de-emphasis mode", ak4458_enum[1]),
	SOC_ENUM("DAC3 de-emphasis mode", ak4458_enum[2]),
	SOC_ENUM("DAC4 de-emphasis mode", ak4458_enum[3]),
};

static const struct snd_soc_dapm_widget ak4458_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC1L", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC1R", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC2L", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC2R", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC3L", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC3R", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC4L", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC4R", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("AOUTL1P"),
	SND_SOC_DAPM_OUTPUT("AOUTL1N"),
	SND_SOC_DAPM_OUTPUT("AOUTR1P"),
	SND_SOC_DAPM_OUTPUT("AOUTR1N"),
	SND_SOC_DAPM_OUTPUT("AOUTL2P"),
	SND_SOC_DAPM_OUTPUT("AOUTL2N"),
	SND_SOC_DAPM_OUTPUT("AOUTR2P"),
	SND_SOC_DAPM_OUTPUT("AOUTR2N"),
	SND_SOC_DAPM_OUTPUT("AOUTL3P"),
	SND_SOC_DAPM_OUTPUT("AOUTL3N"),
	SND_SOC_DAPM_OUTPUT("AOUTR3P"),
	SND_SOC_DAPM_OUTPUT("AOUTR3N"),
	SND_SOC_DAPM_OUTPUT("AOUTL4P"),
	SND_SOC_DAPM_OUTPUT("AOUTL4N"),
	SND_SOC_DAPM_OUTPUT("AOUTR4P"),
	SND_SOC_DAPM_OUTPUT("AOUTR4N"),
};

static const struct snd_soc_dapm_route ak4458_dapm_routes[] = {
	{ "AOUTL1P", NULL, "DAC1L" },
	{ "AOUTL1N", NULL, "DAC1L" },
	{ "AOUTR1P", NULL, "DAC1R" },
	{ "AOUTR1N", NULL, "DAC1R" },
	{ "AOUTL2P", NULL, "DAC2L" },
	{ "AOUTL2N", NULL, "DAC2L" },
	{ "AOUTR2P", NULL, "DAC2R" },
	{ "AOUTR2N", NULL, "DAC2R" },
	{ "AOUTL3P", NULL, "DAC3L" },
	{ "AOUTL3N", NULL, "DAC3L" },
	{ "AOUTR3P", NULL, "DAC3R" },
	{ "AOUTR3N", NULL, "DAC3R" },
	{ "AOUTL4P", NULL, "DAC4L" },
	{ "AOUTL4N", NULL, "DAC4L" },
	{ "AOUTR4P", NULL, "DAC4R" },
	{ "AOUTR4N", NULL, "DAC4R" },
};

static int ak4458_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);
	unsigned int fs = params_rate(params);
	unsigned int mclk_ratio = ak4458->sysclk / fs;
	unsigned int bick_ratio = 64 << ak4458->tdm;

	if (ak4458->tdm) {
		/* check if MCLK ratio is supported */
		if ((mclk_ratio % bick_ratio != 0) ||
		   ((mclk_ratio / bick_ratio) & 0x1)) {
			dev_err(codec->dev, "MCLK/fs ratio %d unsupported\n",
				mclk_ratio);
			return -EINVAL;
		}
	}

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		break;
	default:
		dev_err(codec->dev, "ak4458_hw_params: Unsupported bit size param = %d",
			params_format(params));
		return -EINVAL;
	}

	dev_dbg(codec->dev, "ak4458_hw_params: fs = %d bit size param = %d",
		fs, params_format(params));

	/* reset */
	snd_soc_update_bits(codec,
			    AK4458_REG_CONTROL1,
			    AK4458_CONTROL1_RSTN,
			    0);

	udelay(100);

	/* un-reset */
	snd_soc_update_bits(codec,
			    AK4458_REG_CONTROL1,
			    AK4458_CONTROL1_RSTN,
			    AK4458_CONTROL1_RSTN);
	return 0;
}

static int ak4458_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "ak4458_set_dai_sysclk info: freq=%dHz\n", freq);

	ak4458->sysclk = freq;
	return 0;
}

static int ak4458_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned int dif = 0;

	/* check master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_LEFT_J:
		dif = AK4458_CONTROL1_DIF_32MSBJ;
		break;
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_DSP_A:
		dif = AK4458_CONTROL1_DIF_32I2S;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(codec->dev, "ak4458_set_dai_fmt: %x\n", fmt);

	snd_soc_update_bits(codec,
			    AK4458_REG_CONTROL1,
			    AK4458_CONTROL1_DIF,
			    dif);
	return 0;
}

static int ak4458_set_tdm_slot(struct snd_soc_dai *codec_dai,
		unsigned int tx_mask, unsigned int rx_mask,
		int slots, int slot_width)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	switch (slots) {
	case 4:
		ak4458->tdm = 1;
		break;
	case 8:
		ak4458->tdm = 2;
		break;
	case 16:
		ak4458->tdm = 3;
		break;
	default:
		return -EINVAL;
	}

	while (ak4458->sds < sizeof(unsigned int)) {
		if (tx_mask & (0x1 << ak4458->sds))
			break;
		ak4458->sds++;
	}
	if (ak4458->sds > 7)
		return -EINVAL;

	dev_dbg(codec->dev, "AK4458: tdm %d, sds %d\n",
		ak4458->tdm, ak4458->sds);

	/* Set Audio Interface Format */
	snd_soc_update_bits(codec,
			   AK4458_REG_CONTROL6,
			   AK4458_CONTROL6_TDM |
			   AK4458_CONTROL6_SDS1 | AK4458_CONTROL6_SDS2,
			   (ak4458->tdm << AK4458_CONTROL6_TDM_SHIFT) |
			   ((ak4458->sds & 2) ? AK4458_CONTROL6_SDS1 : 0) |
			   ((ak4458->sds & 4) ? AK4458_CONTROL6_SDS2 : 0));
	snd_soc_update_bits(codec,
			   AK4458_REG_CONTROL7,
			   AK4458_CONTROL7_SDS0,
			   ((ak4458->sds & 1) ? AK4458_CONTROL7_SDS0 : 0));

	return 0;
}

static int ak4458_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	if (!mute) {
		snd_soc_update_bits(codec,
				   AK4458_REG_CONTROL2,
				   AK4458_CONTROL2_SMUTE,
				   0);
	} else {
		snd_soc_update_bits(codec,
				   AK4458_REG_CONTROL2,
				   AK4458_CONTROL2_SMUTE,
				   AK4458_CONTROL2_SMUTE);
	}
	return 0;
}

#define AK4458_RATES (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
			SNDRV_PCM_RATE_192000)

#define AK4458_FORMATS (SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops ak4458_dai_ops = {
	.hw_params	= ak4458_hw_params,
	.set_sysclk	= ak4458_set_dai_sysclk,
	.set_fmt	= ak4458_set_dai_fmt,
	.set_tdm_slot	= ak4458_set_tdm_slot,
	.digital_mute	= ak4458_mute,
};

static struct snd_soc_dai_driver ak4458_dai = {
	.name = "ak4458-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = AK4458_RATES,
		.formats = AK4458_FORMATS,
	},
	.ops = &ak4458_dai_ops,
};

static int ak4458_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	switch (level) {
	case SND_SOC_BIAS_ON:
		/* un-mute */
		gpiod_set_value_cansleep(ak4458->amp_mute_gpio, 0);
		break;
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		/* mute */
		gpiod_set_value_cansleep(ak4458->amp_mute_gpio, 1);
		break;
	}
	return 0;
}

static int ak4458_probe(struct snd_soc_codec *codec)
{
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	/* Mute */
	gpiod_set_value_cansleep(ak4458->amp_mute_gpio, 1);

	/* Put codec in software reset */
	snd_soc_update_bits(codec,
			    AK4458_REG_CONTROL1,
			    AK4458_CONTROL1_RSTN,
			    0);

	/* Enable PCM mode */
	snd_soc_update_bits(codec,
			   AK4458_REG_CONTROL3,
			   AK4458_CONTROL3_DP,
			   0);

	/* Enable Master Clock Frequency Auto Setting Mode */
	snd_soc_update_bits(codec,
			   AK4458_REG_CONTROL1,
			   AK4458_CONTROL1_ACKS,
			   AK4458_CONTROL1_ACKS);

	return 0;
}

static int ak4458_remove(struct snd_soc_codec *codec)
{
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	/* Mute */
	gpiod_set_value_cansleep(ak4458->amp_mute_gpio, 1);

	/* Put codec in software reset */
	snd_soc_update_bits(codec,
			    AK4458_REG_CONTROL1,
			    AK4458_CONTROL1_RSTN,
			    0);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_ak4458 = {
	.probe =	ak4458_probe,
	.remove =	ak4458_remove,
	.set_bias_level = ak4458_set_bias_level,

	.component_driver = {
		.controls = ak4458_snd_controls,
		.num_controls = ARRAY_SIZE(ak4458_snd_controls),
		.dapm_widgets = ak4458_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(ak4458_dapm_widgets),
		.dapm_routes = ak4458_dapm_routes,
		.num_dapm_routes = ARRAY_SIZE(ak4458_dapm_routes),
	},
};

static const struct regmap_config ak4458_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AK4458_NUM_REGS - 1,

	.reg_defaults = ak4458_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ak4458_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
};

static int ak4458_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct ak4458_priv *ak4458;
	int ret;

	ak4458 = devm_kzalloc(&i2c->dev, sizeof(struct ak4458_priv),
			      GFP_KERNEL);
	if (ak4458 == NULL)
		return -ENOMEM;

	ak4458->amp_mute_gpio = devm_gpiod_get_optional(&i2c->dev,
							"amp-mute",
							GPIOD_OUT_HIGH);

	if (IS_ERR(ak4458->amp_mute_gpio)) {
		ret = PTR_ERR(ak4458->amp_mute_gpio);
		dev_err(&i2c->dev, "Failed to request amp-mute gpio\n");
		return ret;
	}

	ak4458->regmap = devm_regmap_init_i2c(i2c, &ak4458_regmap);
	if (IS_ERR(ak4458->regmap)) {
		ret = PTR_ERR(ak4458->regmap);
		dev_err(&i2c->dev, "Failed to init regmap: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(i2c, ak4458);

	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_ak4458, &ak4458_dai, 1);
	return ret;
}

static int ak4458_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	return 0;
}

static const struct of_device_id ak4458_of_match[] = {
	{ .compatible = "asahi-kasei,ak4458", },
	{ }
};
MODULE_DEVICE_TABLE(of, ak4458_of_match);

static const struct i2c_device_id ak4458_i2c_id[] = {
	{ "ak4458", 0 },
	{ }
};
MODULE_DEVICE_TABLE(of, ak4458_i2c_id);

static struct i2c_driver ak4458_i2c_driver = {
	.driver = {
		.name	= "ak4458-codec",
		.of_match_table = ak4458_of_match,
	},
	.probe		= ak4458_i2c_probe,
	.remove		= ak4458_i2c_remove,
	.id_table	= ak4458_i2c_id,
};

module_i2c_driver(ak4458_i2c_driver);

MODULE_DESCRIPTION("ASoC ak4458 driver");
MODULE_AUTHOR("Corneliu Doban <cdoban@broadcom.com>");
MODULE_LICENSE("GPL v2");
