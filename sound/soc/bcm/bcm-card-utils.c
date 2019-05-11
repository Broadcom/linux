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
#include <linux/module.h>
#include <linux/of.h>
#include <sound/simple_card_utils.h>
#include <sound/soc.h>

struct bcm_link_data {
	struct snd_soc_dai_link_component codecs;
	struct snd_soc_dai_link_component platform;
};

int bcm_card_util_parse_link_node(struct platform_device *pdev,
		struct device_node *link_np,
		struct snd_soc_dai_link *dai_link)
{
	struct device *dev = &pdev->dev;
	struct device_node *codec_np;
	struct device_node *cpu_np;
	struct bcm_link_data *link_data;
	int ret = 0;
	int single_cpu_link;

	link_data = devm_kzalloc(dev, sizeof(*link_data), GFP_KERNEL);
	if (!link_data)
		return -ENOMEM;

	dai_link->codecs     = &link_data->codecs;
	dai_link->num_codecs = 1;
	dai_link->platforms  = &link_data->platform;

	codec_np = of_get_child_by_name(link_np, "codec");
	if (!codec_np) {
		dev_err(dev, "%s could not find codec child\n", link_np->name);
		ret = -EINVAL;
		goto err_exit;
	}

	cpu_np = of_get_child_by_name(link_np, "cpu");
	if (!cpu_np) {
		dev_err(dev, "%s Could not find cpu child node.\n",
				link_np->name);
		ret = -EINVAL;
		goto err_exit;
	}

	ret = asoc_simple_card_parse_daifmt(dev, link_np,
					codec_np, NULL,
					&dai_link->dai_fmt);
	if (ret < 0) {
		dev_err(dev, "%s Error parsing dai fmt\n", link_np->name);
		goto err_exit;
	}

	ret = asoc_simple_card_parse_codec(codec_np, dai_link,
					"sound-dai", "#sound-dai-cells");
	if (ret < 0) {
		dev_err(dev, "%s Error parsing codec node\n", link_np->name);
		goto err_exit;
	}

	ret = asoc_simple_card_parse_cpu(cpu_np, dai_link,
					"sound-dai", "#sound-dai-cells",
					&single_cpu_link);
	if (ret < 0) {
		dev_err(dev, "%s Error parsing cpu node\n", link_np->name);
		goto err_exit;
	}

	asoc_simple_card_canonicalize_platform(dai_link);

	ret = asoc_simple_card_set_dailink_name(dev, dai_link,
						"%s-%s",
						dai_link->cpu_dai_name,
						dai_link->codec_dai_name);
	if (ret < 0)
		goto err_exit;

	dev_dbg(dev, "\tname : %s\n",     dai_link->stream_name);
	dev_dbg(dev, "\tformat : %04x\n", dai_link->dai_fmt);
	dev_dbg(dev, "\tcpu : %s\n",      dai_link->cpu_dai_name);
	dev_dbg(dev, "\tcodec : %s\n",    dai_link->codec_dai_name);

	asoc_simple_card_canonicalize_cpu(dai_link, single_cpu_link);

err_exit:
	if (codec_np)
		of_node_put(codec_np);
	if (cpu_np)
		of_node_put(cpu_np);

	return ret;
}
EXPORT_SYMBOL_GPL(bcm_card_util_parse_link_node);

