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
 * Use legacy way of implementation with older version
 */
#define BCM_VK_MISC_API  (LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0))

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

	spinlock_t msg_id_lock;
	uint16_t msg_id;
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
irqreturn_t bcm_vk_irqhandler(int irq, void *dev_id);
int bcm_vk_msg_init(struct bcm_vk *vk);
void bcm_vk_msg_remove(struct bcm_vk *vk);
int bcm_vk_sync_msgq(struct bcm_vk *vk);

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
