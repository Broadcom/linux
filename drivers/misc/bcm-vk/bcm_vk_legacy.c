// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018-2020 Broadcom.
 */

#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/version.h>

#include "bcm_vk.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0) || \
    defined(CONFIG_REQ_FW_INTO_BUF_PRIV)

int request_firmware_into_buf_priv(const struct firmware **firmware_p,
				   const char *name, struct device *device,
				   void *buf, size_t size,
				   size_t offset, unsigned int pread_flags)
{
	int ret;

	if (offset != 0)
		return -EINVAL;

	ret = request_firmware(firmware_p, name, device);
	if (ret)
		return ret;

	if ((*firmware_p)->size > size) {
		release_firmware(*firmware_p);
		ret = -EFBIG;
	} else {
		memcpy(buf, (*firmware_p)->data, (*firmware_p)->size);
	}

	return ret;
}
#endif
