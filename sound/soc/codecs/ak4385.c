/*
 * Copyright (C) 2015-2017 Broadcom
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
 * ak4385.c  --  AK4385 ALSA SoC Audio driver
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

/* AK4385 registers addresses */
#define AK4385_REG_CONTROL1		0x00
#define AK4385_REG_CONTROL2		0x01
#define AK4385_REG_CONTROL3		0x02
#define AK4385_REG_LCH_ATT		0x03
#define AK4385_REG_RCH_ATT		0x04
#define AK4385_NUM_REGS			5

#define AK4385_REG_MASK			0x1f
#define AK4385_READ			0x40
#define AK4385_WRITE			0x60

#define AK4385_CONTROL1_RSTN		0x01
#define AK4385_CONTROL1_PW		0x02
#define AK4385_CONTROL1_DIF		0x1c
#define AK4385_CONTROL1_DIF_16LSBJ	0x00
#define AK4385_CONTROL1_DIF_20LSBJ	0x04
#define AK4385_CONTROL1_DIF_24MSBJ	0x08
#define AK4385_CONTROL1_DIF_24I2S	0x0c
#define AK4385_CONTROL1_DIF_24LSBJ	0x10
#define AK4385_CONTROL1_ACKS		0x80

#define AK4385_CONTROL2_SMUTE		0x01
#define AK4385_CONTROL2_DEM		0x06
#define AK4385_CONTROL2_DEM_44P1KHZ	0x00
#define AK4385_CONTROL2_DEM_OFF		0x02
#define AK4385_CONTROL2_DEM_48KHZ	0x04
#define AK4385_CONTROL2_DEM_32KHZ	0x06
#define AK4385_CONTROL2_DFS		0x18
#define AK4385_CONTROL2_DFS_NORMAL	0x00
#define AK4385_CONTROL2_DFS_DOUBLE	0x08
#define AK4385_CONTROL2_DFS_QUAD	0x10
#define AK4385_CONTROL2_SLOW		0x20
#define AK4385_CONTROL2_DZFM		0x40
#define AK4385_CONTROL2_DZFE		0x80

#define AK4385_CONTROL3_DZFB		0x04

/* codec private data */
struct ak4385_priv {
	struct regmap *regmap;
	unsigned int sysclk;
	struct gpio_desc *power_gpio;
	struct gpio_desc *amp_mute_gpio;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
};

static const struct reg_default ak4385_reg_defaults[] = {
	{AK4385_REG_CONTROL1, 0x8b},
	{AK4385_REG_CONTROL2, 0x02},
	{AK4385_REG_CONTROL3, 0x00},
	{AK4385_REG_LCH_ATT, 0xff},
	{AK4385_REG_RCH_ATT, 0xff},
};

static bool ak4385_readable(struct device *dev, unsigned int reg)
{
	/* ak4385 does not support register read operations */
	return false;
}

static bool ak4385_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AK4385_REG_CONTROL1:
	case AK4385_REG_CONTROL2:
	case AK4385_REG_CONTROL3:
	case AK4385_REG_LCH_ATT:
	case AK4385_REG_RCH_ATT:
		return true;
	default:
		return false;
	}
}

static const DECLARE_TLV_DB_LINEAR(db_scale_linear, TLV_DB_GAIN_MUTE, 0);

static const struct snd_kcontrol_new ak4385_snd_controls[] = {
	SOC_DOUBLE_R_TLV("Volume", AK4385_REG_LCH_ATT,
			AK4385_REG_RCH_ATT, 0, 255, 0, db_scale_linear),
};

static const struct snd_soc_dapm_widget ak4385_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DACL", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DACR", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("VOUTLP"),
	SND_SOC_DAPM_OUTPUT("VOUTLN"),
	SND_SOC_DAPM_OUTPUT("VOUTRP"),
	SND_SOC_DAPM_OUTPUT("VOUTRN"),
};

static const struct snd_soc_dapm_route ak4385_dapm_routes[] = {
	{ "VOUTLP", NULL, "DACL" },
	{ "VOUTLN", NULL, "DACL" },
	{ "VOUTRP", NULL, "DACR" },
	{ "VOUTRN", NULL, "DACR" },
};

static unsigned int lrclk_ratios[] = {
	128, 192, 256, 384, 512, 768
};

static unsigned int rates_16384[] = {
	32000
};

static struct snd_pcm_hw_constraint_list constraints_16384 = {
	.count	= ARRAY_SIZE(rates_16384),
	.list	= rates_16384,
};

static unsigned int rates_22579[] = {
	44100, 88200, 176400
};

static struct snd_pcm_hw_constraint_list constraints_22579 = {
	.count	= ARRAY_SIZE(rates_22579),
	.list	= rates_22579,
};

static unsigned int rates_24576[] = {
	32000, 48000, 96000, 192000
};

static struct snd_pcm_hw_constraint_list constraints_24576 = {
	.count	= ARRAY_SIZE(rates_24576),
	.list	= rates_24576,
};

static unsigned int rates_36864[] = {
	48000, 96000, 192000
};

static struct snd_pcm_hw_constraint_list constraints_36864 = {
	.count	= ARRAY_SIZE(rates_36864),
	.list	= rates_36864,
};

static int ak4385_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct ak4385_priv *ak4385 = snd_soc_component_get_drvdata(component);
	unsigned int fs = params_rate(params);
	int i;
	unsigned int val;

	/* check if LRCLK ratio is supported */
	for (i = 0; i < ARRAY_SIZE(lrclk_ratios); i++) {
		if (ak4385->sysclk / fs == lrclk_ratios[i])
			break;
	}

	if (i == ARRAY_SIZE(lrclk_ratios)) {
		dev_err(component->dev, "MCLK/fs ratio %d unsupported\n",
			ak4385->sysclk / fs);
		return -EINVAL;
	}

	/* check if the fs is supported */
	for (i = 0; i < ak4385->sysclk_constraints->count; i++) {
		if (fs == ak4385->sysclk_constraints->list[i])
			break;
	}

	if (i == ak4385->sysclk_constraints->count) {
		dev_err(component->dev, "fs %d unsupported\n", fs);
		return -EINVAL;
	}

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		break;
	default:
		dev_err(component->dev, "ak4385_hw_params: Unsupported bit size param = %d",
			params_format(params));
		return -EINVAL;
	}

	dev_dbg(component->dev, "ak4385_hw_params: fs = %d bit size param = %d",
		fs, params_format(params));

	/* power up */
	gpiod_set_value_cansleep(ak4385->power_gpio, 1);

	udelay(10);

	regcache_mark_dirty(ak4385->regmap);
	regcache_cache_only(ak4385->regmap, false);

	/* sync all registers */
	for (i = 0; i < AK4385_NUM_REGS; i++) {
		regmap_read(ak4385->regmap, i, &val);
		regmap_write(ak4385->regmap, i, val);
	}

	/* reset */
	regmap_update_bits(ak4385->regmap,
			   AK4385_REG_CONTROL1,
			   AK4385_CONTROL1_RSTN,
			   0);

	udelay(10);

	/* un-reset */
	regmap_update_bits(ak4385->regmap,
			   AK4385_REG_CONTROL1,
			   AK4385_CONTROL1_RSTN,
			   AK4385_CONTROL1_RSTN);

	udelay(10);

	/* un-mute AMP */
	gpiod_set_value_cansleep(ak4385->amp_mute_gpio, 0);
	return 0;
}

static int ak4385_hw_free(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct ak4385_priv *ak4385 = snd_soc_component_get_drvdata(component);

	regcache_cache_only(ak4385->regmap, true);

	/* mute AMP */
	gpiod_set_value_cansleep(ak4385->amp_mute_gpio, 1);
	udelay(10);

	/* power down */
	gpiod_set_value_cansleep(ak4385->power_gpio, 0);

	return 0;
}

static int ak4385_set_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = dai->component;
	struct ak4385_priv *ak4385 = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "ak4385_set_dai_sysclk info: freq=%dHz\n", freq);

	switch (freq) {
	case 16384000:
		ak4385->sysclk_constraints = &constraints_16384;
		ak4385->sysclk = freq;
		return 0;

	case 22579200:
	case 33868800:
		ak4385->sysclk_constraints = &constraints_22579;
		ak4385->sysclk = freq;
		return 0;

	case 24576000:
		ak4385->sysclk_constraints = &constraints_24576;
		ak4385->sysclk = freq;
		return 0;

	case 36864000:
		ak4385->sysclk_constraints = &constraints_36864;
		ak4385->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}

static int ak4385_set_dai_fmt(struct snd_soc_dai *dai,
		unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
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
	case SND_SOC_DAIFMT_I2S:
		dif = AK4385_CONTROL1_DIF_24I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		dif = AK4385_CONTROL1_DIF_24MSBJ;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(component->dev, "ak4385_set_dai_fmt: %x\n", fmt);

	snd_soc_component_update_bits(component, AK4385_REG_CONTROL1,
				      AK4385_CONTROL1_DIF, dif);
	return 0;
}


#define AK4385_RATES (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
			SNDRV_PCM_RATE_192000)

#define AK4385_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
			SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops ak4385_dai_ops = {
	.hw_params	= ak4385_hw_params,
	.hw_free	= ak4385_hw_free,
	.set_sysclk	= ak4385_set_dai_sysclk,
	.set_fmt	= ak4385_set_dai_fmt,
};

static struct snd_soc_dai_driver ak4385_dai = {
	.name = "ak4385-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = AK4385_RATES,
		.formats = AK4385_FORMATS,
	},
	.ops = &ak4385_dai_ops,
};

static const struct snd_soc_component_driver soc_component_dev_ak4385 = {
	.controls = ak4385_snd_controls,
	.num_controls = ARRAY_SIZE(ak4385_snd_controls),
	.dapm_widgets = ak4385_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ak4385_dapm_widgets),
	.dapm_routes = ak4385_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(ak4385_dapm_routes),
};

static const struct regmap_config ak4385_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AK4385_NUM_REGS - 1,

	.reg_defaults = ak4385_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ak4385_reg_defaults),
	.read_flag_mask = AK4385_READ,
	.write_flag_mask = AK4385_WRITE,
	.cache_type = REGCACHE_RBTREE,

	.readable_reg = ak4385_readable,
	.writeable_reg = ak4385_writeable,
};

static int ak4385_spi_probe(struct spi_device *spi)
{
	struct ak4385_priv *ak4385;
	int ret;

	ak4385 = devm_kzalloc(&spi->dev, sizeof(struct ak4385_priv),
			     GFP_KERNEL);
	if (ak4385 == NULL)
		return -ENOMEM;

	ak4385->power_gpio = devm_gpiod_get(&spi->dev, "power", GPIOD_OUT_LOW);

	if (IS_ERR(ak4385->power_gpio)) {
		dev_err(&spi->dev, "Invalid or missing power gpio\n");
		return PTR_ERR(ak4385->power_gpio);
	}

	ak4385->amp_mute_gpio = devm_gpiod_get_optional(&spi->dev,
						"amp-mute", GPIOD_OUT_HIGH);

	if (IS_ERR(ak4385->amp_mute_gpio)) {
		dev_err(&spi->dev, "Failed to request amp-mute gpio\n");
		return PTR_ERR(ak4385->amp_mute_gpio);
	}

	ak4385->regmap = devm_regmap_init_spi(spi, &ak4385_regmap);
	if (IS_ERR(ak4385->regmap)) {
		ret = PTR_ERR(ak4385->regmap);
		dev_err(&spi->dev, "Failed to init regmap: %d\n", ret);
		return ret;
	}

	regcache_cache_only(ak4385->regmap, true);

	spi_set_drvdata(spi, ak4385);

	ret = devm_snd_soc_register_component(&spi->dev,
			&soc_component_dev_ak4385, &ak4385_dai, 1);
	return ret;
}

static const struct of_device_id ak4385_of_match[] = {
	{ .compatible = "asahi-kasei,ak4385", },
	{ }
};
MODULE_DEVICE_TABLE(of, ak4385_of_match);

static struct spi_driver ak4385_spi_driver = {
	.driver = {
		.name	= "ak4385",
		.of_match_table = ak4385_of_match,
	},
	.probe		= ak4385_spi_probe,
};

module_spi_driver(ak4385_spi_driver);

MODULE_DESCRIPTION("ASoC ak4385 driver");
MODULE_AUTHOR("Corneliu Doban <cdoban@broadcom.com>");
MODULE_LICENSE("GPL v2");
