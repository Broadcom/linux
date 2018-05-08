/*
 * Copyright (C) 2018 Broadcom
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
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "omega-ssp.h"
#include "bcm-card-utils.h"

#define SVK_MAX_PORTS  5
#define PROP_LEN_MAX  80

static struct dentry *svk_debugfs_root;

struct testport_cfg_info {
	u32 transfer_mode;  /* 0 = i2s, 1 = dsp_a, 2 = dsp_b , 3 = custom*/
	bool slave_mode;
	u32 mclk_freq;
	u32 tdm_framesize;
	u32 fsync_width;
	bool invert_bclk;
	u32 slot_size;
};


/* Machine DAPM */
static const struct snd_soc_dapm_widget audioh_dapm_widgets[] = {

	SND_SOC_DAPM_HP("SVK Headphone jack", NULL),
	SND_SOC_DAPM_SPK("SVK Handsfree", NULL),
	SND_SOC_DAPM_SPK("SVK Handset Earpiece", NULL),

	SND_SOC_DAPM_MIC("SVK Analog Mic1", NULL),  /* Handset or J169 */
	SND_SOC_DAPM_MIC("SVK Analog Mic2", NULL),  /* 3.5mm jack or J150 */
	SND_SOC_DAPM_MIC("SVK Analog Mic3", NULL),  /* J159 */

	SND_SOC_DAPM_MIC("SVK Digital Mic1", NULL),
	SND_SOC_DAPM_MIC("SVK Digital Mic2", NULL),
	SND_SOC_DAPM_MIC("SVK Digital Mic3", NULL),
	SND_SOC_DAPM_MIC("SVK Digital Mic4", NULL),
};

struct card_state_data {
	unsigned int    dummy0;
	struct testport_cfg_info testport_cfg[SVK_MAX_PORTS];
	struct snd_soc_dai_link  omegasvk_demo_dai_links[SVK_MAX_PORTS];
};

static int testport_hw_params_common(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct testport_cfg_info *testport_sspcfg)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct device *dev = rtd->card->dev;
	unsigned int format = 0;
	bool use_tdm;
	int ret = 0;

	dev_dbg(dev, "Enter %s\n", __func__);

	/* Only set mclk in master mode */
	if (testport_sspcfg->slave_mode == 0) {
		ret = snd_soc_dai_set_sysclk(cpu_dai, 0,
				testport_sspcfg->mclk_freq, SND_SOC_CLOCK_OUT);
		if (ret < 0) {
			dev_err(dev, "%s Failed snd_soc_dai_set_sysclk\n",
			__func__);
			return ret;
		}
	}

	use_tdm = (testport_sspcfg->transfer_mode == 1) ||
			(testport_sspcfg->transfer_mode == 2);

	if (use_tdm) {
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
		dev_dbg(dev, "Testport MASTER.\n");
		format |= SND_SOC_DAIFMT_CBS_CFS;
	} else {
		dev_dbg(dev, "Testport SLAVE.\n");
		format |= SND_SOC_DAIFMT_CBM_CFM;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, format);
	if (ret < 0) {
		dev_err(dev, "%s Failed snd_soc_dai_set_fmt cpu_dai\n",
			__func__);
		return ret;
	}

	if (use_tdm) {
		unsigned int channels = 0;
		unsigned int width = 0;
		unsigned int mask;
		unsigned int bit_per_frame = testport_sspcfg->tdm_framesize;
		int slots;
		int i;

		channels = params_channels(params);
		width = snd_pcm_format_physical_width(params_format(params));

		/* if slot_size is programmed (non-zero) then use that value */
		if (testport_sspcfg->slot_size != 0)
			width = testport_sspcfg->slot_size;

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

		if (testport_sspcfg->fsync_width != 0)
			omega_ssp_set_custom_fsync_width(cpu_dai,
					testport_sspcfg->fsync_width);
	}
	return 0;
}

static int omegasvk_demo_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct card_state_data *card_data = snd_soc_card_get_drvdata(soc_card);
	int num = rtd->num;

	return testport_hw_params_common(substream, params,
			&card_data->testport_cfg[num]);
}

static struct snd_soc_ops omegasvk_demo_ops = {
	.hw_params = omegasvk_demo_hw_params,
};

/* Audio machine driver */
static struct snd_soc_card omegasvk_audio_card = {
	.name = "bcm-oemgasvk-demo-card",
	.owner = THIS_MODULE,

	.dapm_widgets = audioh_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(audioh_dapm_widgets),
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
	debugfs_create_u32("transfer_mode", perm, parent, &cfg->transfer_mode);
	cfg->transfer_mode = 0;

	debugfs_create_bool("slave_mode", perm, parent, &cfg->slave_mode);
	cfg->slave_mode = false;

	debugfs_create_u32("mclk_freq", perm, parent, &cfg->mclk_freq);
	cfg->mclk_freq = 12288000*2;

	debugfs_create_u32("tdm_framesize", perm, parent, &cfg->tdm_framesize);
	cfg->tdm_framesize = 256;

	debugfs_create_u32("fsync_width", perm, parent, &cfg->fsync_width);
	cfg->fsync_width = 0;

	debugfs_create_bool("invert_bclk", perm, parent, &cfg->invert_bclk);
	cfg->invert_bclk = false;

	debugfs_create_u32("slot_size", perm, parent, &cfg->slot_size);
	cfg->slot_size = 32;

	return 0;
}

static int parse_preset_link_prop(struct device_node *dn,
				unsigned int link_num,
				struct testport_cfg_info *cfg)
{
	u32 rawval;

	if (of_property_read_u32(dn, "brcm,tdm_framesize", &rawval) == 0)
		cfg->tdm_framesize = rawval;

	if (of_property_read_u32(dn, "brcm,slot_size", &rawval) == 0)
		cfg->slot_size = rawval;


	if (of_property_read_u32(dn, "brcm,slave_mode", &rawval) == 0) {
		if (rawval == 0)
			cfg->slave_mode = false;
		else
			cfg->slave_mode = true;
	}

	if (of_property_read_u32(dn, "brcm,mclk_freq", &rawval) == 0)
		cfg->mclk_freq = rawval;

	if (of_property_read_u32(dn, "brcm,invert_bclk", &rawval) == 0) {
		if (rawval == 0)
			cfg->invert_bclk = false;
		else
			cfg->invert_bclk = true;
	}

	if (of_property_read_u32(dn, "brcm,fsync_width", &rawval) == 0)
		cfg->fsync_width = rawval;

	if (of_property_read_u32(dn, "brcm,transfer_mode", &rawval) == 0)
		cfg->transfer_mode = rawval;

	return 0;
}

static int omegasvk_demo_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &omegasvk_audio_card;
	struct card_state_data *card_data;
	struct device_node *link_np;
	char name[PROP_LEN_MAX];
	int ret = 0;
	int linknum, linktotal;

	dev_info(&pdev->dev, "Enter %s\n", __func__);

	card->dev = &pdev->dev;

	linktotal = of_get_child_count(pdev->dev.of_node);
	if ((linktotal < 1) || (linktotal > SVK_MAX_PORTS)) {
		dev_err(&pdev->dev,
			"child nodes is %d. Must be between 1 and %d\n",
			linktotal, SVK_MAX_PORTS);
		return -EINVAL;
	}

	card_data = devm_kzalloc(&pdev->dev, sizeof(*card_data), GFP_KERNEL);
	if (card_data == NULL)
		return -ENOMEM;

	card->dai_link = card_data->omegasvk_demo_dai_links;
	card->num_links = linktotal;

	/*
	 * Add configurable entries for the testport.
	 * These can be found under: /sys/kernel/debug/omegasvk_cfg/
	 */
	svk_debugfs_root = debugfs_create_dir("omegasvk_cfg", NULL);
	if (IS_ERR(svk_debugfs_root)) {
		ret = PTR_ERR(svk_debugfs_root);
		svk_debugfs_root = NULL;
		goto err_exit;
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

		card->dai_link[linknum].ops = &omegasvk_demo_ops;

		ret = create_testport_debugfs(linknum,
				&card_data->testport_cfg[linknum]);
		if (ret) {
			dev_err(&pdev->dev, "testport debugfs failed");
			goto err_exit;
		}

		parse_preset_link_prop(link_np, linknum,
				&card_data->testport_cfg[linknum]);
	}

	ret = snd_soc_of_parse_audio_routing(card, "brcm,audio-routing");
	if (ret)
		goto err_exit;

	snd_soc_card_set_drvdata(card, card_data);
	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed: %d\n", ret);
		goto err_exit;
	}

	return 0;

err_exit:
	debugfs_remove_recursive(svk_debugfs_root);
	return ret;
}

static int omegasvk_demo_remove(struct platform_device *pdev)
{
	debugfs_remove_recursive(svk_debugfs_root);

	return 0;
}

static const struct of_device_id omegasvk_demo_mach_of_match[] = {
	{.compatible = "brcm,omega-svk-machine"},
	{ },
};
MODULE_DEVICE_TABLE(of, omegasvk_demo_mach_of_match);

static struct platform_driver bcm_omegasvk_audio_driver = {
	.driver = {
		.name = "omega-svk-machine",
		.pm = &snd_soc_pm_ops,
		.of_match_table = omegasvk_demo_mach_of_match,
	},
	.probe  = omegasvk_demo_probe,
	.remove = omegasvk_demo_remove,
};

module_platform_driver(bcm_omegasvk_audio_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("ASoC machine driver for Omega");
MODULE_LICENSE("GPL v2");
