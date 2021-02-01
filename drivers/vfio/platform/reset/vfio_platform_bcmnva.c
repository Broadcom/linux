// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Broadcom
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
 * This driver provides reset support for Broadcom NVA device
 * to VFIO platform.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "../vfio_platform_private.h"

static int vfio_platform_bcmnva_reset(struct vfio_platform_device *vdev)
{
	int ret = 0;

	pr_info("%s called\n", __func__);
	/* TODO: Do some reset work here  */

	return ret;
}

module_vfio_reset_handler("brcm,iproc-nva",
			  vfio_platform_bcmnva_reset);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Broadcom <ccx@broadcom.com>");
MODULE_DESCRIPTION("Reset support for Broadcom NVA IO platform device");
