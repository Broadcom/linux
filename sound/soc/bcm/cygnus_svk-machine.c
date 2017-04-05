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
#include <asm/mach-types.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/gpio/consumer.h>
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

#include "cygnus-ssp.h"
#include "bcm-card-utils.h"
#include "../codecs/tlv320aic3x.h"
#include "../codecs/wm8750.h"

struct testport_cfg_info {
	bool tdm_mode;
	bool slave_mode;
	u32 mclk_freq;
	u32 tdm_framesize;
	u32 fsync_width;
	bool invert_bclk;
};

#define CYGNUSSVK_MAX_LINKS   4
#define MAX_TESTPORTS  CYGNUSSVK_MAX_LINKS

static struct dentry *svk_debugfs_root;

struct cygnussvk_ti3106_daughtercard_info {
	struct gpio_desc *gpio_bank_sel0;
	struct gpio_desc *gpio_bank_sel1;
	struct gpio_desc *gpio_ext_headset_amp_en;
	struct gpio_desc *gpio_handsfree_amp_en;
	int smartcardslot;
};

struct cygnussvk_data {
	struct testport_cfg_info testport_cfg[MAX_TESTPORTS];
	struct cygnussvk_ti3106_daughtercard_info  ti3106cfg;
};

static int cygnussvk_hw_params_aic3x(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct device *dev = rtd->card->dev;
	unsigned int mclk_freq = 0;
	int ret = 0;

	dev_dbg(dev, "%s Enter\n", __func__);

	switch (params_rate(params)) {
	case  8000:
		mclk_freq = 6144000;
		break;
	case 16000:
	case 32000:
	case 48000:
	case 96000:
		mclk_freq = 12288000;
		break;

	case 11025:
		mclk_freq = 5644800;
		break;
	case 22050:
	case 44100:
	case 88200:
		mclk_freq = 11289600;
		break;

	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, CLKIN_MCLK, mclk_freq,
				SND_SOC_CLOCK_IN);
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

	return 0;
}

static int cygnussvk_startup_aic3x(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct cygnussvk_data *card_data = snd_soc_card_get_drvdata(soc_card);

	/* Set the GPIO multiplex to select appropriate daughter card */
	if (card_data->ti3106cfg.smartcardslot == 0) {
		gpiod_set_value(card_data->ti3106cfg.gpio_bank_sel0, 0);
		gpiod_set_value(card_data->ti3106cfg.gpio_bank_sel1, 1);
	} else {
		gpiod_set_value(card_data->ti3106cfg.gpio_bank_sel0, 1);
		gpiod_set_value(card_data->ti3106cfg.gpio_bank_sel1, 0);
	}
	return 0;
}

static struct snd_soc_ops cygnussvk_ops_aic3x = {
	.startup = cygnussvk_startup_aic3x,
	.hw_params = cygnussvk_hw_params_aic3x,
};

static int handsfree_spk_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm_context = w->dapm;
	struct snd_soc_card *soc_card = dapm_context->card;
	struct cygnussvk_data *card_data =
		snd_soc_card_get_drvdata(soc_card);
	struct gpio_desc *gpio = card_data->ti3106cfg.gpio_handsfree_amp_en;

	/* Enable the op amp that will drive the handsfree jack */
	if (SND_SOC_DAPM_EVENT_ON(event))
		gpiod_set_value(gpio, 1);
	else
		gpiod_set_value(gpio, 0);

	return 0;
}

static int ext_spk_evt_aic3x(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm_context = w->dapm;
	struct snd_soc_card *soc_card = dapm_context->card;
	struct cygnussvk_data *card_data =
		snd_soc_card_get_drvdata(soc_card);
	struct gpio_desc *gpio = card_data->ti3106cfg.gpio_ext_headset_amp_en;

	/* Enable the op amp that will drive the 3.5mm stereo jack */
	if (SND_SOC_DAPM_EVENT_ON(event))
		gpiod_set_value(gpio, 1);
	else
		gpiod_set_value(gpio, 0);

	return 0;
}

/* Machine DAPM */
static const struct snd_soc_dapm_widget aic3106_dapm_widgets[] = {
	/* Outputs */
	SND_SOC_DAPM_HP("Headset Spk", NULL),
	SND_SOC_DAPM_HP("Handset Spk", NULL),
	SND_SOC_DAPM_LINE("Handsfree Spk", handsfree_spk_event),
	SND_SOC_DAPM_SPK("External Spk", ext_spk_evt_aic3x), /* 3.5 mm jack */

	/* Inputs */
	SND_SOC_DAPM_MIC("Handset Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Handsfree Mic", NULL),
	SND_SOC_DAPM_LINE("External Mic", NULL),   /* 3.5 mm jack */
};

/* machine audio map (connections to the codec pins on tlv320aic3x)
 *============================================================================
 * Outputs
 * Codec Pin	Schematic Name		Physical connector
 * ------------------------------------------------------
 * HPLOUT	MONO_HEADSET_SPK_P      (J501) Headset RJ11 Jack
 * HPLCOM	MONO_HEADSET_SPK_N
 * -------------------------------------------------------------------------
 * HPROUT	HANDSET_SPK_N		(J403) Handset RJ11 Jack
 * HPRCOM	HANDSET_SPK_P
 * -------------------------------------------------------------------------
 * MONO_LOP	HANDFREE_SPK_N		(J401) Handsfree Speaker
 * MONO_LOM	HANDFREE_SPK_P		via AMP (U401) (needs enable via GPIO)
 * -------------------------------------------------------------------------
 * LEFT_LOP	STEREO_HEADSET_SPKL_P
 * LEFT_LOM	STEREO_HEADSET_SPKL_N
 * ----------------------------------	(J502) 3.5mm Stereo Headset Jack
 * RIGHT_LOP	STEREO_HEADSET_SPKR_P	via AMP (U501) (needs enable via GPIO)
 * RIGHT_LOM	STEREO_HEADSET_SPKR_N
 *
 *============================================================================
 * Inputs
 * Codec Pin	Schematic Name		Physical connector
 * ------------------------------------------------------
 * LINE1LP	HNDFREE_MIC_P		(J402) Currently unpopulate on Voip DM
 * LINE1LM	HNDFREE_MIC_N
 * ------------------------------------------------------
 * LINE1RP	No connect
 * LINE1RM	No connect
 * ------------------------------------------------------
 * LINE2LP	HANDSET_MIC_P		(J403) Handset RJ11 Jack
 * LINE2LM	HANDSET_MIC_N
 * ------------------------------------------------------
 * LINE2RP	MONO_HEADSET_MIC_P	(J501) Headset RJ11 Jack
 * LINE2RM	MONO_HEADSET_MIC_N
 * ------------------------------------------------------
 * MIC3L	No connect
 * MIC3R	STEREO_HEADSET_MIC_P	(J502) 3.5mm Stereo Headset Jack
 *
 * Daughter module connector
 *	Pin 10: External (3.5mm jack) Headset AMP enable
 *	Pin 12: Codec Reset
 *	Pin 14: Handsfree AMP enable
 *	Pin 24  MCLK
 *	Pin 26  SDIN  (from codec to cygnus)
 *	Pin 28  SDOUT (from cygnus to codec)
 *	Pin 30  WS
 *	Pin 32  BITCLK
 *
 * GPIO information
 * Three GPIOs from Cygnus are routed to the codec.  The GPIOs pass through
 * a multiplexer (IDTQS3253), allowing them to be used for other devices.
 * Another 2 GPIOs (AON_GPIO[0,1] from Cygnus are used to set the multiplexor.
 *
 * On Combo SVK (958300k)
 * ---------------------
 * "Smartcard 0" (J2398) - uses Cygnus' i2s0
 * Pin 10 - GPIO 0  }
 * Pin 12 - GPIO 1  } via IDTQS3253 mulitpx (AON_GPIO1=1 AON_GPIO0=0) SC0
 * Pin 14 - GPIO 2  }
 *
 * "Smartcard 1" (J2399) - uses Cygnus' i2s1
 * Pin 10 - GPIO 0  }
 * Pin 12 - GPIO 1  } via IDTQS3253 mulitpx (AON_GPIO1=0 AON_GPIO0=1) SC1
 * Pin 14 - GPIO 2  }
 *
 * On Voip SVK (911360k)
 * ---------------------
 * "Smartcard 0" (J2398) - uses Cygnus' i2s1 (mislabeled on schematic)
 * Pin 10 - GPIO 5  }
 * Pin 12 - GPIO 6  } via IDTQS3253 mulitpx (AON_GPIO1=1 AON_GPIO0=0) SC0
 * Pin 14 - GPIO 7  }
 *
 * "Smartcard 1" (J2399) - Does not work for codec
 */
static const struct snd_soc_dapm_route aic3106_audio_map[] = {
	/* Routings for outputs */
	{"Headset Spk",   NULL, "TI HPLOUT"},
	{"Handset Spk",   NULL, "TI HPROUT"},
	{"Handsfree Spk", NULL, "TI MONO_LOUT"},
	{"External Spk",  NULL, "TI LLOUT"},
	{"External Spk",  NULL, "TI RLOUT"},

	/* Routings for inputs */
	{"TI LINE1L", NULL, "Handset Mic"},
	{"TI LINE2L", NULL, "Headset Mic"},
	{"TI LINE2R", NULL, "Handsfree Mic"},

	{"Handset Mic",   NULL, "TI Mic Bias"},
	{"Headset Mic",   NULL, "TI Mic Bias"},
	{"Handsfree Mic", NULL, "TI Mic Bias"},

	{"TI MIC3R", NULL, "External Mic"},
};

static int cygnussvk_init_aic3106(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dapm_context *dapm = &card->dapm;

	snd_soc_dapm_new_controls(dapm, aic3106_dapm_widgets,
				ARRAY_SIZE(aic3106_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, aic3106_audio_map,
				ARRAY_SIZE(aic3106_audio_map));

	snd_soc_dapm_nc_pin(dapm, "TI LINE1R");
	snd_soc_dapm_nc_pin(dapm, "TI MIC3L");

	return 0;
}

static int cygnussvk_hw_params_wm8750(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct device *dev = rtd->card->dev;
	unsigned int mclk_freq = 0;
	int ret = 0;

	dev_dbg(dev, "Enter %s\n", __func__);

	switch (params_rate(params)) {
	/* These will not work. Cygnus and WM8750 do not have a common MCLK */
	case 8000:
	case 11025:
		dev_err(dev,
			"%s: %d Hz will not work because there is not a frequency suitable for both Cygnus and the WM8750.\n",
			__func__, params_rate(params));
		return -EINVAL;

	case 16000:
	case 32000:
	case 48000:
	case 96000:
		mclk_freq = 12288000;
		break;

	case 22050:
	case 44100:
	case 88200:
		mclk_freq = 11289600;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, WM8750_SYSCLK,
				mclk_freq, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk %u\n",
			__func__, mclk_freq);
		return ret;
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

static struct snd_soc_ops cygnussvk_ops_wm8750 = {
	.hw_params = cygnussvk_hw_params_wm8750,
};

static const struct snd_soc_dapm_widget wm8750_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Wolfson Headphone", NULL),
	SND_SOC_DAPM_MIC("Wolfson Mic", NULL),
};

/* Routings for WM8750 */
static const struct snd_soc_dapm_route wm8750_audio_map[] = {
	{"Wolfson Headphone", NULL, "WM8750 LOUT1"},
	{"Wolfson Headphone", NULL, "WM8750 ROUT1"},

	{"WM8750 LINPUT1", NULL, "Wolfson Mic"},
	{"WM8750 RINPUT1", NULL, "Wolfson Mic"},
	{"Wolfson Mic", NULL, "WM8750 Mic Bias"},
};

static int cygnussvk_init_wm8750(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dapm_context *dapm = &card->dapm;

	snd_soc_dapm_new_controls(dapm, wm8750_dapm_widgets,
				ARRAY_SIZE(wm8750_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, wm8750_audio_map,
				ARRAY_SIZE(wm8750_audio_map));

	snd_soc_dapm_nc_pin(dapm, "WM8750 LOUT2");
	snd_soc_dapm_nc_pin(dapm, "WM8750 ROUT2");

	snd_soc_dapm_nc_pin(dapm, "WM8750 OUT3");
	snd_soc_dapm_nc_pin(dapm, "WM8750 MONO1");

	snd_soc_dapm_nc_pin(dapm, "WM8750 LINPUT2");
	snd_soc_dapm_nc_pin(dapm, "WM8750 RINPUT2");
	snd_soc_dapm_nc_pin(dapm, "WM8750 LINPUT3");
	snd_soc_dapm_nc_pin(dapm, "WM8750 RINPUT3");

	return 0;
}
static int cygnussvk_hw_params_spdif(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->card->dev;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int mclk_freq = 0;
	int ret = 0;

	dev_dbg(dev, "Enter %s\n", __func__);

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

static struct snd_soc_ops cygnussvk_ops_spdif = {
	.hw_params = cygnussvk_hw_params_spdif,
};

static int cygnussvk_testport_hw_params_common(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct testport_cfg_info *testport_sspcfg)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct device *dev = rtd->card->dev;
	unsigned int format = 0;
	int ret = 0;

	dev_dbg(dev, "Enter %s\n", __func__);

	/* Only set mclk in master mode */
	if (testport_sspcfg->slave_mode == 0) {
		ret = snd_soc_dai_set_sysclk(cpu_dai, CYGNUS_SSP_CLKSRC_PLL,
				testport_sspcfg->mclk_freq, SND_SOC_CLOCK_OUT);
		if (ret < 0) {
			dev_err(dev, "%s Failed snd_soc_dai_set_sysclk\n",
			__func__);
			return ret;
		}
	}

	if (testport_sspcfg->tdm_mode == 1) {
		cygnus_ssp_set_custom_fsync_width(cpu_dai,
				testport_sspcfg->fsync_width);

		format = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF;
		dev_dbg(dev, "Configure system for TDM transfer.\n");
	} else {
		dev_dbg(dev, "Configure system for I2S transfer.\n");
		format = SND_SOC_DAIFMT_I2S;
		if (testport_sspcfg->invert_bclk)
			format |= SND_SOC_DAIFMT_IB_NF;
		else
			format |= SND_SOC_DAIFMT_NB_NF;
	}

	if (testport_sspcfg->slave_mode == 0) {
		dev_dbg(dev, "Cygnus testport MASTER.\n");
		format |= SND_SOC_DAIFMT_CBS_CFS;
	} else {
		dev_dbg(dev, "Cygnus testport SLAVE.\n");
		format |= SND_SOC_DAIFMT_CBM_CFM;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, format);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_fmt cpu_dai\n",
			__func__);
		return ret;
	}

	if (testport_sspcfg->tdm_mode == 1) {
		unsigned int channels = 0;
		unsigned int width = 0;
		unsigned int mask;
		unsigned int bit_per_frame = testport_sspcfg->tdm_framesize;
		int slots;
		int i;

		channels = params_channels(params);
		width = snd_pcm_format_physical_width(params_format(params));

		mask = 0;
		for (i = 0; i < channels; i++)
			mask |= BIT(i);

		slots = bit_per_frame / width;

		ret = snd_soc_dai_set_tdm_slot(cpu_dai, mask, mask,
					slots, width);
		if (ret < 0) {
			dev_err(dev, "%s Failed snd_soc_dai_set_tdm_slot\n",
				__func__);
			return ret;
		}
	}
	return 0;
}

static int cygnussvk_testport_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct cygnussvk_data *card_data = snd_soc_card_get_drvdata(soc_card);
	int num = rtd->num;

	return cygnussvk_testport_hw_params_common(substream, params,
			&card_data->testport_cfg[num]);
}

static struct snd_soc_ops cygnussvk_testport_ops  = {
	.hw_params = cygnussvk_testport_hw_params,
};

int cygnussvk_card_probe(struct snd_soc_card *card)
{
	return 0;
}

static struct snd_soc_dai_link cygnussvk_dai_links[CYGNUSSVK_MAX_LINKS];

/*
 * Wolfson and TI codec have conflicting name for "Mic Bias" widget
 * Need to apply the codec name prefix to make the names unique.
 */
static struct snd_soc_codec_conf cygnussvk_codec_conf[CYGNUSSVK_MAX_LINKS];

static struct snd_soc_card cygnussvk_card = {
	.name  = "bcm-cygnussvk",
	.owner = THIS_MODULE,
	.probe = cygnussvk_card_probe,
};

static int create_testport_debugfs(unsigned int link_num,
				struct testport_cfg_info *cfg)
{
	char port_name[20];
	static struct dentry *parent;
	umode_t perm;

	snprintf(port_name, 20, "ssp_testport%d", link_num);

	parent = debugfs_create_dir(port_name, svk_debugfs_root);
	if (IS_ERR(parent)) {
		pr_err(" FAILED to create debugfs subdir %s", port_name);
		parent = NULL;
		return PTR_ERR(parent);
	}

	perm = 0666;
	debugfs_create_bool("tdm_mode", perm, parent, &cfg->tdm_mode);
	cfg->tdm_mode = false;

	debugfs_create_bool("slave_mode", perm, parent, &cfg->slave_mode);
	cfg->slave_mode = false;

	debugfs_create_u32("mclk_freq", perm, parent, &cfg->mclk_freq);
	cfg->mclk_freq = 12288000;

	debugfs_create_u32("tdm_framesize", perm, parent, &cfg->tdm_framesize);
	cfg->tdm_framesize = 128;

	debugfs_create_u32("fsync_width", perm, parent, &cfg->fsync_width);
	cfg->fsync_width = 1;

	debugfs_create_bool("invert_bclk", perm, parent, &cfg->invert_bclk);
	cfg->invert_bclk = false;

	return 0;
}

/* Parse the DT entries spefically for the TI3106 daughter card */
static int get_ti3106_daughtercard_dt_into(struct platform_device *pdev,
					struct cygnussvk_data *card_data)
{
	struct cygnussvk_ti3106_daughtercard_info  *cfg_3106;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	cfg_3106 = &card_data->ti3106cfg;

	ret = of_property_read_u32(np, "brcm,scslot", &cfg_3106->smartcardslot);
	if (ret) {
		dev_err(&pdev->dev, "Could not find the sc slot num\n");
		return ret;
	}

	/* bank selection gpio. Routes signals to daughter card */
	cfg_3106->gpio_bank_sel0 = devm_gpiod_get_optional(&pdev->dev,
					"brcm,bank-sel0", GPIOD_OUT_LOW);
	if (IS_ERR(cfg_3106->gpio_bank_sel0)) {
		dev_err(&pdev->dev, "Invalid or missing bank_sel0 gpio\n");
		return PTR_ERR(cfg_3106->gpio_bank_sel0);
	}

	cfg_3106->gpio_bank_sel1 = devm_gpiod_get_optional(&pdev->dev,
					"brcm,bank-sel1", GPIOD_OUT_HIGH);
	if (IS_ERR(cfg_3106->gpio_bank_sel1)) {
		dev_err(&pdev->dev, "Invalid or missing bank_sel1 gpio\n");
		return PTR_ERR(cfg_3106->gpio_bank_sel1);
	}

	/* External headset amplifier enable */
	cfg_3106->gpio_ext_headset_amp_en = devm_gpiod_get(&pdev->dev,
				"brcm,ext-headset-amp-en", GPIOD_OUT_LOW);
	if (IS_ERR(cfg_3106->gpio_ext_headset_amp_en)) {
		dev_err(&pdev->dev, "Invalid gpio for headset amp enable\n");
		return PTR_ERR(cfg_3106->gpio_ext_headset_amp_en);
	}

	/* handsfree amplifier enable */
	cfg_3106->gpio_handsfree_amp_en = devm_gpiod_get(&pdev->dev,
				"brcm,handsfree-amp-en", GPIOD_OUT_LOW);
	if (IS_ERR(cfg_3106->gpio_handsfree_amp_en)) {
		dev_err(&pdev->dev, "Invalid gpio for handsfree amp enable\n");
		return PTR_ERR(cfg_3106->gpio_handsfree_amp_en);
	}

	return 0;
}

static int cygnussvk_probe(struct platform_device *pdev)
{
	struct device_node *card_node = pdev->dev.of_node;
	struct snd_soc_card *card = &cygnussvk_card;
	struct device_node *link_np;
	struct cygnussvk_data *card_data;
	struct snd_soc_dai_link *dai_link;
	struct snd_soc_codec_conf *codec_conf;
	int ret = 0;
	int linknum;

	dev_dbg(&pdev->dev, "Enter %s\n", __func__);

	card->dev = &pdev->dev;

	card_data = devm_kzalloc(&pdev->dev, sizeof(*card_data), GFP_KERNEL);
	if (card_data == NULL)
		return -ENOMEM;

	/*
	 * Add configurable entries for the testport.
	 * These can be found under: /sys/kernel/debug/cygnussvk_cfg/
	 */
	svk_debugfs_root = debugfs_create_dir("cygnussvk_cfg", NULL);
	if (IS_ERR(svk_debugfs_root)) {
		ret = PTR_ERR(svk_debugfs_root);
		svk_debugfs_root = NULL;
		goto err_exit;
	}

	card->num_configs = 0;

	link_np = of_get_child_by_name(card_node, "dai-link");
	if (link_np == NULL)
		goto err_exit;

	linknum = 0;
	for_each_child_of_node(card_node, link_np) {
		dai_link = &cygnussvk_dai_links[linknum];

		ret = bcm_card_util_parse_link_node(pdev, link_np, dai_link);
		if (ret < 0) {
			dev_err(&pdev->dev, "Error parsing link %d", linknum);
			goto err_exit;
		}

		if (!strncmp(dai_link->codec_dai_name, "bcm-dummy-codec-dai",
				sizeof("bcm-dummy-codec-dai"))) {
			if (linknum >= MAX_TESTPORTS) {
				ret = -EINVAL;
				goto err_exit;
			}

			ret = create_testport_debugfs(linknum,
					&card_data->testport_cfg[linknum]);
			if (ret) {
				dev_err(&pdev->dev, "testport debugfs failed");
				goto err_exit;
			}

			dai_link->ops = &cygnussvk_testport_ops;
		}

		if (!strncmp(dai_link->codec_dai_name, "tlv320aic3x-hifi",
				sizeof("tlv320aic3x-hifi"))) {

			ret = get_ti3106_daughtercard_dt_into(pdev, card_data);
			if (ret < 0)
				goto err_exit;

			dai_link->ops = &cygnussvk_ops_aic3x;
			dai_link->init = cygnussvk_init_aic3106;

			codec_conf = &cygnussvk_codec_conf[card->num_configs];
			codec_conf->of_node = dai_link->codec_of_node;
			codec_conf->name_prefix = "TI";
			card->num_configs++;
		}

		if (!strncmp(dai_link->codec_dai_name, "wm8750-hifi",
				sizeof("wm8750-hifi"))) {
			dai_link->ops = &cygnussvk_ops_wm8750;
			dai_link->init = cygnussvk_init_wm8750;

			codec_conf = &cygnussvk_codec_conf[card->num_configs];
			codec_conf->of_node = dai_link->codec_of_node;
			codec_conf->name_prefix = "WM8750";
			card->num_configs++;
		}

		if (!strncmp(dai_link->codec_dai_name, "dit-hifi",
				sizeof("dit-hifi"))) {
			dai_link->ops = &cygnussvk_ops_spdif;
		}

		linknum++;
	}

	card->dai_link  = cygnussvk_dai_links;
	card->num_links = linknum;

	if (card->num_configs > 0)
		card->codec_conf = cygnussvk_codec_conf;

	snd_soc_card_set_drvdata(card, card_data);

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",
			ret);
		goto err_exit;
	}

	return 0;

err_exit:
	debugfs_remove_recursive(svk_debugfs_root);
	return ret;
}

static int cygnussvk_remove(struct platform_device *pdev)
{
	debugfs_remove_recursive(svk_debugfs_root);

	return 0;
}

static const struct of_device_id cygnus_mach_of_match[] = {
	{.compatible = "brcm,cygnussvk-machine", },
	{ },
};
MODULE_DEVICE_TABLE(of, cygnus_mach_of_match);

static struct platform_driver bcm_cygnus_driver = {
	.driver = {
		.name = "bcm-cygnussvk-machine",
		.pm = &snd_soc_pm_ops,
		.of_match_table = cygnus_mach_of_match,
	},
	.probe  = cygnussvk_probe,
	.remove = cygnussvk_remove,
};

module_platform_driver(bcm_cygnus_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Cygnus SVK sound card");
MODULE_LICENSE("GPL v2");
