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
#include <sound/soc.h>

int bcm_card_util_parse_link_node(struct platform_device *pdev,
		struct device_node *link_np,
		struct snd_soc_dai_link *dai_link);
