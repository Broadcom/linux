/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 Broadcom.
 */

#ifndef BCM_VK_HWMON_H
#define BCM_VK_HWMON_H

/*
 * HWMON APIs
 */
int bcm_vk_hwmon_init(struct bcm_vk *vk);

void bcm_vk_hwmon_deinit(struct bcm_vk *vk);

#endif
