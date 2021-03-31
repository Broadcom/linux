// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Broadcom
 */

/*
 * This driver provides reset support for the Broadcom VEE device
 * to VFIO platform.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "../vfio_platform_private.h"

static int vfio_platform_bcmvee_reset(struct vfio_platform_device *vdev)
{
	int ret = 0;

	pr_info("%s called\n", __func__);
	/* TODO: Do some reset work here  */

	return ret;
}

module_vfio_reset_handler("brcm,iproc-vee",
			  vfio_platform_bcmvee_reset);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Broadcom <ccx@broadcom.com>");
MODULE_DESCRIPTION("Reset support for Broadcom VEE platform device");
