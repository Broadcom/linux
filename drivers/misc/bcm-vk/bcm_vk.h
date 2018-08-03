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

#include "bcm_vk_msg.h"

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
	/* mutex to protect the ioctls */
	struct mutex mutex;
	struct miscdevice miscdev;

	spinlock_t msg_id_lock;
	uint16_t msg_id;
	spinlock_t ctx_lock;
	struct bcm_vk_ctx op_ctx[VK_CMPT_CTX_MAX];

	struct bcm_vk_msg_chan h2vk_msg_chan;
	struct bcm_vk_msg_chan vk2h_msg_chan;

	struct work_struct vk2h_wq; /* work queue for deferred job */
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

int bcm_vk_open(struct inode *inode, struct file *p_file);
ssize_t bcm_vk_read(struct file *p_file, char __user *buf, size_t count,
		    loff_t *f_pos);
ssize_t bcm_vk_write(struct file *p_file, const char __user *buf,
		     size_t count, loff_t *f_pos);
int bcm_vk_release(struct inode *inode, struct file *p_file);
irqreturn_t bcm_vk_irqhandler(int irq, void *dev_id);
int bcm_vk_msg_init(struct bcm_vk *vk);
void bcm_vk_msg_remove(struct bcm_vk *vk);

#endif
