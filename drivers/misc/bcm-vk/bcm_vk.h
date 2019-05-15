/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright(c) 2018 Broadcom
 */

#ifndef __BCM_VK_H
#define __BCM_VK_H

#include <linux/pci.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "bcm_vk_msg.h"

/*
 * Load Image is completed in two stages:
 *
 * 1) When the VK device boot-up, M7 CPU runs and executes the BootROM.
 * The Secure Boot Loader (SBL) as part of the BootROM will run
 * fastboot to open up ITCM for host to push BOOT1 image.
 * SBL will authenticate the image before jumping to BOOT1 image.
 *
 * 2) Because BOOT1 image is a secured image, we also called it the
 * Secure Boot Image (SBI). At second stage, SBI will initialize DDR
 * and run fastboot for host to push BOOT2 image to DDR.
 * SBI will authenticate the image before jumping to BOOT2 image.
 *
 */
/* Location of registers of interest in BAR0 */
/* Fastboot request for Secure Boot Loader (SBL) */
#define BAR_CODEPUSH_SBL	0x400
/* Fastboot progress */
#define BAR_FB_OPEN		0x404
/* Fastboot request for Secure Boot Image (SBI) */
#define BAR_CODEPUSH_SBI	0x408
#define BAR_CARD_STATUS		0x410
#define BAR_FW_STATUS		0x41C
#define BAR_METADATA_VERSION	0x440
#define BAR_FIRMWARE_VERSION	0x444
#define BAR_CARD_TEMPERATURE	0x45C
#define BAR_CARD_VOLTAGE	0x460

#define CODEPUSH_BOOT1_ENTRY	0x00400000
#define CODEPUSH_BOOT2_ENTRY	0x60000000
#define CODEPUSH_FASTBOOT	BIT(0)
#define SRAM_OPEN		BIT(16)
#define DDR_OPEN		BIT(17)

/* FW_STATUS definitions */
#define FW_STATUS_RELOCATION_ENTRY		BIT(0)
#define FW_STATUS_RELOCATION_EXIT		BIT(1)
#define FW_STATUS_ZEPHYR_INIT_START		BIT(2)
#define FW_STATUS_ZEPHYR_ARCH_INIT_DONE		BIT(3)
#define FW_STATUS_ZEPHYR_PRE_KERNEL1_INIT_DONE	BIT(4)
#define FW_STATUS_ZEPHYR_PRE_KERNEL2_INIT_DONE	BIT(5)
#define FW_STATUS_ZEPHYR_POST_KERNEL_INIT_DONE	BIT(6)
#define FW_STATUS_ZEPHYR_INIT_DONE		BIT(7)
#define FW_STATUS_ZEPHYR_APP_INIT_START		BIT(8)
#define FW_STATUS_ZEPHYR_APP_INIT_DONE		BIT(9)
#define FW_STATUS_MASK				0xFFFFFFFF
#define FW_STATUS_ZEPHYR_READY	(FW_STATUS_ZEPHYR_INIT_START | \
				 FW_STATUS_ZEPHYR_ARCH_INIT_DONE | \
				 FW_STATUS_ZEPHYR_PRE_KERNEL1_INIT_DONE | \
				 FW_STATUS_ZEPHYR_PRE_KERNEL2_INIT_DONE | \
				 FW_STATUS_ZEPHYR_POST_KERNEL_INIT_DONE | \
				 FW_STATUS_ZEPHYR_INIT_DONE | \
				 FW_STATUS_ZEPHYR_APP_INIT_START | \
				 FW_STATUS_ZEPHYR_APP_INIT_DONE)

/* VK MSG_ID Bitmap Size */
#define VK_MSG_ID_BITMAP_SIZE 4096

/*
 * Use legacy way of implementation with older version
 */
#define BCM_VK_MISC_API  (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))

#define MAX_BAR 3
enum pci_barno {
	BAR_0 = 0,
	BAR_1,
	BAR_2
};

struct bcm_vk {
	struct pci_dev *pdev;
	void __iomem *bar[MAX_BAR];
	int num_irqs;

#if BCM_VK_MISC_API
	struct msix_entry msix[32];
#endif
	/* mutex to protect the ioctls */
	struct mutex mutex;
	struct miscdevice miscdev;
	int misc_devid; /* dev id allocated */

	/* Reference-counting to handle file operations */
	struct kref kref;

	spinlock_t msg_id_lock;
	uint16_t msg_id;
	DECLARE_BITMAP(bmap, VK_MSG_ID_BITMAP_SIZE);
	spinlock_t ctx_lock;
	struct bcm_vk_ctx op_ctx[VK_CMPT_CTX_MAX];
	struct bcm_vk_ht_entry pid_ht[VK_PID_HT_SZ];
	struct task_struct *reset_ppid; /* process that issue reset */

	bool msgq_inited; /* indicate if info has been synced with vk */
	struct bcm_vk_msg_chan h2vk_msg_chan;
	struct bcm_vk_msg_chan vk2h_msg_chan;

	struct workqueue_struct *vk2h_wq_thread;
	struct work_struct vk2h_wq; /* work queue for deferred job */
	void *tdma_vaddr; /* test dma segment virtual addr */
	dma_addr_t tdma_addr; /* test dma segment bus addr */
};

static inline u32 vkread32(struct bcm_vk *vk,
			   enum pci_barno bar,
			   uint64_t offset)
{
	u32 value;

	value = ioread32(vk->bar[bar] + offset);
	return value;
}

static inline void vkwrite32(struct bcm_vk *vk,
			     u32 value,
			     enum pci_barno bar,
			     uint64_t offset)
{
	iowrite32(value, vk->bar[bar] + offset);
}

static inline void vkwrite8(struct bcm_vk *vk,
			    u8 value,
			    enum pci_barno bar,
			    uint64_t offset)
{
	iowrite8(value, vk->bar[bar] + offset);
}

int bcm_vk_open(struct inode *inode, struct file *p_file);
ssize_t bcm_vk_read(struct file *p_file, char __user *buf, size_t count,
		    loff_t *f_pos);
ssize_t bcm_vk_write(struct file *p_file, const char __user *buf,
		     size_t count, loff_t *f_pos);
int bcm_vk_release(struct inode *inode, struct file *p_file);
void bcm_vk_release_data(struct kref *kref);
irqreturn_t bcm_vk_irqhandler(int irq, void *dev_id);
int bcm_vk_msg_init(struct bcm_vk *vk);
void bcm_vk_msg_remove(struct bcm_vk *vk);
int bcm_vk_sync_msgq(struct bcm_vk *vk);
int bcm_vk_send_shutdown_msg(struct bcm_vk *vk, uint32_t shut_type, pid_t pid);
void bcm_vk_trigger_reset(struct bcm_vk *vk);

#if BCM_VK_MISC_API

/*
 * For legacy kernels, the following 2 PCI APIs will be missing, and
 * have to use msix_entry[] instead.  The APIs are provided in file bcm_vk_pci.c
 */
#define PCI_IRQ_MSI		    BIT(0)
#define PCI_IRQ_MSIX		    BIT(1)

int pci_irq_vector(struct pci_dev *pdev, unsigned int nr);
int pci_alloc_irq_vectors(struct pci_dev *pdev, unsigned int min_vecs,
			  unsigned int max_vecs, unsigned int flags);

#endif

#endif
