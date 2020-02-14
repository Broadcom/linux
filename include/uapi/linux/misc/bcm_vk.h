/* SPDX-License-Identifier: ((GPL-2.0 WITH Linux-syscall-note) OR BSD-2-Clause) */
/*
 * Copyright 2018-2020 Broadcom.
 */

#ifndef __UAPI_LINUX_MISC_BCM_VK_H
#define __UAPI_LINUX_MISC_BCM_VK_H

#include <linux/ioctl.h>
#include <linux/types.h>

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

#define VK_BAR_FWSTS			0x41C
/* VK_FWSTS definitions */
#define VK_FWSTS_RELOCATION_ENTRY	BIT(0)
#define VK_FWSTS_RELOCATION_EXIT	BIT(1)
#define VK_FWSTS_INIT_START		BIT(2)
#define VK_FWSTS_ARCH_INIT_DONE		BIT(3)
#define VK_FWSTS_PRE_KNL1_INIT_DONE	BIT(4)
#define VK_FWSTS_PRE_KNL2_INIT_DONE	BIT(5)
#define VK_FWSTS_POST_KNL_INIT_DONE	BIT(6)
#define VK_FWSTS_INIT_DONE		BIT(7)
#define VK_FWSTS_APP_INIT_START		BIT(8)
#define VK_FWSTS_APP_INIT_DONE		BIT(9)
#define VK_FWSTS_MASK			0xFFFFFFFF
#define VK_FWSTS_READY			(VK_FWSTS_INIT_START | \
					 VK_FWSTS_ARCH_INIT_DONE | \
					 VK_FWSTS_PRE_KNL1_INIT_DONE | \
					 VK_FWSTS_PRE_KNL2_INIT_DONE | \
					 VK_FWSTS_POST_KNL_INIT_DONE | \
					 VK_FWSTS_INIT_DONE | \
					 VK_FWSTS_APP_INIT_START | \
					 VK_FWSTS_APP_INIT_DONE)
/* Deinit */
#define VK_FWSTS_APP_DEINIT_START	BIT(23)
#define VK_FWSTS_APP_DEINIT_DONE	BIT(24)
#define VK_FWSTS_DRV_DEINIT_START	BIT(25)
#define VK_FWSTS_DRV_DEINIT_DONE	BIT(26)
#define VK_FWSTS_RESET_DONE		BIT(27)
#define VK_FWSTS_DEINIT_TRIGGERED	(VK_FWSTS_APP_DEINIT_START | \
					 VK_FWSTS_APP_DEINIT_DONE  | \
					 VK_FWSTS_DRV_DEINIT_START | \
					 VK_FWSTS_DRV_DEINIT_DONE)
/* Last nibble for reboot reason */
#define VK_FWSTS_RESET_REASON_SHIFT	28
#define VK_FWSTS_RESET_REASON_MASK	(0xF << VK_FWSTS_RESET_REASON_SHIFT)
#define VK_FWSTS_RESET_SYS_PWRUP	(0x0 << VK_FWSTS_RESET_REASON_SHIFT)
#define VK_FWSTS_RESET_MBOX_DB		(0x1 << VK_FWSTS_RESET_REASON_SHIFT)
#define VK_FWSTS_RESET_M7_WDOG		(0x2 << VK_FWSTS_RESET_REASON_SHIFT)
#define VK_FWSTS_RESET_TEMP		(0x3 << VK_FWSTS_RESET_REASON_SHIFT)
#define VK_FWSTS_RESET_PCI_FLR		(0x4 << VK_FWSTS_RESET_REASON_SHIFT)
#define VK_FWSTS_RESET_PCI_HOT		(0x5 << VK_FWSTS_RESET_REASON_SHIFT)
#define VK_FWSTS_RESET_PCI_WARM		(0x6 << VK_FWSTS_RESET_REASON_SHIFT)
#define VK_FWSTS_RESET_PCI_COLD		(0x7 << VK_FWSTS_RESET_REASON_SHIFT)
#define VK_FWSTS_RESET_L1		(0x8 << VK_FWSTS_RESET_REASON_SHIFT)
#define VK_FWSTS_RESET_L0		(0x9 << VK_FWSTS_RESET_REASON_SHIFT)
#define VK_FWSTS_RESET_UNKNOWN		(0xF << VK_FWSTS_RESET_REASON_SHIFT)

#endif /* __UAPI_LINUX_MISC_BCM_VK_H */
