/* SPDX-License-Identifier: ((GPL-2.0 WITH Linux-syscall-note) OR BSD-2-Clause) */
/*
 * Copyright(c) 2018 Broadcom
 */

#ifndef __UAPI_LINUX_MISC_BCM_VK_H
#define __UAPI_LINUX_MISC_BCM_VK_H

#include <linux/ioctl.h>
#include <linux/types.h>

struct vk_metadata {
	/* struct version, always backwards compatible */
	__u32 version;

	/* Version 0 fields */
	__u32 card_status;
#define VK_CARD_STATUS_FASTBOOT_READY BIT(0)
#define VK_CARD_STATUS_FWLOADER_READY BIT(1)

	__u32 firmware_version;
	__u32 fw_status;
	/* End version 0 fields */

	__u64 reserved[14];
	/* Total of 16*u64 for all versions */
};

struct vk_image {
	__u32 type;     /* Type of image */
#define VK_IMAGE_TYPE_BOOT1 1 /* 1st stage (load to SRAM) */
#define VK_IMAGE_TYPE_BOOT2 2 /* 2nd stage (load to DDR) */
	char filename[64]; /* Filename of image */
};

/* default firmware images names */
#define VK_BOOT1_DEF_VALKYRIE_FILENAME	"vk-boot1.bin"
#define VK_BOOT2_DEF_VALKYRIE_FILENAME	"vk-boot2.bin"

#define VK_BOOT1_DEF_VIPER_FILENAME	"vp-boot1.bin"
#define VK_BOOT2_DEF_VIPER_FILENAME	"vp-boot2.bin"

struct vk_access {
	__u8 barno;     /* BAR number to use */
	__u8 type;      /* Type of access */
#define VK_ACCESS_READ 0
#define VK_ACCESS_WRITE 1
	__u32 len;      /* length of data */
	__u64 offset;   /* offset in BAR */
	__u32 *data;    /* where to read/write data to */
};

struct vk_reset {
	__u32 arg1;
	__u32 arg2;
};

#define VK_MAGIC              0x5E

/* Get metadata from Valkyrie (firmware version, card status, etc) */
#define VK_IOCTL_GET_METADATA _IOR(VK_MAGIC, 0x1, struct vk_metadata)

/* Load image to Valkyrie */
#define VK_IOCTL_LOAD_IMAGE   _IOW(VK_MAGIC, 0x2, struct vk_image)

/* Read data from Valkyrie */
#define VK_IOCTL_ACCESS_BAR   _IOWR(VK_MAGIC, 0x3, struct vk_access)

/* Send Reset to Valkyrie */
#define VK_IOCTL_RESET        _IOW(VK_MAGIC, 0x4, struct vk_reset)

/*
 * message block - basic unit in the message where a message's size is always
 *		   N x sizeof(basic_block)
 */
struct vk_msg_blk {
	__u8 function_id;
#define VK_FID_TRANS_BUF 5
#define VK_FID_SHUTDOWN  8
	__u8 size;
	__u16 queue_id:4;
	__u16 msg_id:12;
	__u32 context_id;
	__u32 args[2];
#define VK_CMD_PLANES_MASK 0x000F /* number of planes to up/download */
#define VK_CMD_UPLOAD      0x0400 /* memory transfer to vk */
#define VK_CMD_DOWNLOAD    0x0500 /* memory transfer from vk */
#define VK_CMD_MASK        0x0F00 /* command mask */
};

#endif /* __UAPI_LINUX_MISC_BCM_VK_H */
