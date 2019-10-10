/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2018-2019 Broadcom.
 */

#ifndef BCM_VK_H
#define BCM_VK_H

#include <linux/pci.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/tty.h>
#if defined(BCM_VK_LEGACY_API)
#include <linux/pid.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/sizes.h>
#define task_pid(_tsk) get_task_pid((_tsk), PIDTYPE_PID)
#else
#include <linux/sched/signal.h>
#endif
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
#define BAR_CODEPUSH_SBL		0x400
/* Fastboot progress */
#define BAR_FB_OPEN			0x404
/* Fastboot request for Secure Boot Image (SBI) */
#define BAR_CODEPUSH_SBI		0x408

#define BAR_CARD_STATUS			0x410
/* CARD_STATUS definitions */
#define CARD_STATUS_TTYVK0_READY	BIT(0)
#define CARD_STATUS_TTYVK1_READY	BIT(1)

#define BAR_FW_STATUS			0x41C
/* FW_STATUS definitions */
#define FW_STATUS_RELOCATION_ENTRY	BIT(0)
#define FW_STATUS_RELOCATION_EXIT	BIT(1)
#define FW_STATUS_INIT_START		BIT(2)
#define FW_STATUS_ARCH_INIT_DONE	BIT(3)
#define FW_STATUS_PRE_KNL1_INIT_DONE	BIT(4)
#define FW_STATUS_PRE_KNL2_INIT_DONE	BIT(5)
#define FW_STATUS_POST_KNL_INIT_DONE	BIT(6)
#define FW_STATUS_INIT_DONE		BIT(7)
#define FW_STATUS_APP_INIT_START	BIT(8)
#define FW_STATUS_APP_INIT_DONE		BIT(9)
#define FW_STATUS_MASK			0xFFFFFFFF
#define FW_STATUS_READY			(FW_STATUS_INIT_START | \
					 FW_STATUS_ARCH_INIT_DONE | \
					 FW_STATUS_PRE_KNL1_INIT_DONE | \
					 FW_STATUS_PRE_KNL2_INIT_DONE | \
					 FW_STATUS_POST_KNL_INIT_DONE | \
					 FW_STATUS_INIT_DONE | \
					 FW_STATUS_APP_INIT_START | \
					 FW_STATUS_APP_INIT_DONE)
/* Deinit */
#define FW_STATUS_APP_DEINIT_START	BIT(23)
#define FW_STATUS_APP_DEINIT_DONE	BIT(24)
#define FW_STATUS_DRV_DEINIT_START	BIT(25)
#define FW_STATUS_DRV_DEINIT_DONE	BIT(26)
#define FW_STATUS_RESET_DONE		BIT(27)
#define FW_STATUS_DEINIT_TRIGGERED	(FW_STATUS_APP_DEINIT_START | \
					 FW_STATUS_APP_DEINIT_DONE  | \
					 FW_STATUS_DRV_DEINIT_START | \
					 FW_STATUS_DRV_DEINIT_DONE)
/* Last nibble for reboot reason */
#define FW_STATUS_RESET_REASON_SHIFT	28
#define FW_STATUS_RESET_REASON_MASK	(0xF << FW_STATUS_RESET_REASON_SHIFT)
#define FW_STATUS_RESET_SYS_PWRUP	(0x0 << FW_STATUS_RESET_REASON_SHIFT)
#define FW_STATUS_RESET_MBOX_DB		(0x1 << FW_STATUS_RESET_REASON_SHIFT)
#define FW_STATUS_RESET_M7_WDOG		(0x2 << FW_STATUS_RESET_REASON_SHIFT)
#define FW_STATUS_RESET_TEMP		(0x3 << FW_STATUS_RESET_REASON_SHIFT)
#define FW_STATUS_RESET_PCI_FLR		(0x4 << FW_STATUS_RESET_REASON_SHIFT)
#define FW_STATUS_RESET_PCI_HOT		(0x5 << FW_STATUS_RESET_REASON_SHIFT)
#define FW_STATUS_RESET_PCI_WARM	(0x6 << FW_STATUS_RESET_REASON_SHIFT)
#define FW_STATUS_RESET_PCI_COLD	(0x7 << FW_STATUS_RESET_REASON_SHIFT)
#define FW_STATUS_RESET_L1		(0x8 << FW_STATUS_RESET_REASON_SHIFT)
#define FW_STATUS_RESET_L0		(0x9 << FW_STATUS_RESET_REASON_SHIFT)
#define FW_STATUS_RESET_UNKNOWN		(0xF << FW_STATUS_RESET_REASON_SHIFT)

#define BAR_METADATA_VERSION		0x440
#define BAR_CHIP_ID			0x448
#define BAR_CARD_TEMPERATURE		0x45C
#define BAR_CARD_VOLTAGE		0x460
#define BAR_CARD_ERR_LOG		0x464
#define BAR_CARD_ERR_MEM		0x468
#define BAR_CARD_PWR_AND_THRE		0x46C
#define BAR_CARD_STATIC_INFO		0x470
#define BAR_BOOTSRC_SELECT		0xC78

/* Start of ITCM */
#define CODEPUSH_BOOT1_ENTRY		0x00400000
/* 64M mapped to BAR2 */
#define CODEPUSH_BOOT2_ENTRY		0x60000000
#define CODEPUSH_MASK			0xFFFFF000
#define CODEPUSH_FASTBOOT		BIT(0)

/* Fastboot progress definitions */
#define SRAM_OPEN			BIT(16)
#define DDR_OPEN			BIT(17)

/* BOOTSRC definitions */
#define BOOTSRC_SOFT_ENABLE		BIT(14)

/* Card OS Firmware version size */
#define BAR_FIRMWARE_TAG_SIZE		50
#define FIRMWARE_STATUS_PRE_INIT_DONE	0x1F

/* Fastboot firmware loader status definitions */
#define FW_LOADER_ACK_SEND_MORE_DATA	BIT(18)
#define FW_LOADER_ACK_IN_PROGRESS	BIT(19)
#define FW_LOADER_ACK_RCVD_ALL_DATA	BIT(20)

/* Error log register bit definition - register for error alerts */
#define ERR_LOG_ALERT_ECC		BIT(0)
#define ERR_LOG_ALERT_SSIM_BUSY		BIT(1)
#define ERR_LOG_ALERT_AFBC_BUSY		BIT(2)
#define ERR_LOG_HIGH_TEMP_ERR		BIT(3)
#define ERR_LOG_MEM_ALLOC_FAIL		BIT(8)
#define ERR_LOG_LOW_TEMP_WARN		BIT(9)
#define ERR_LOG_ECC_WARN		BIT(10)

/* Fast boot register derived states */
#define FB_BOOT_STATE_MASK		0xFFF3FFFF
#define FB_BOOT1_RUNNING		(DDR_OPEN | 0x6)
#define FB_BOOT2_RUNNING		(FW_LOADER_ACK_RCVD_ALL_DATA | 0x6)

/* VK MSG_ID defines */
#define VK_MSG_ID_BITMAP_SIZE		4096
#define VK_MSG_ID_BITMAP_MASK		(VK_MSG_ID_BITMAP_SIZE - 1)
#define VK_MSG_ID_OVERFLOW		0xFFFF

/* VK device supports a maximum of 3 bars */
#define MAX_BAR	3

/* default number of msg blk for inband SGL */
#define BCM_VK_DEF_IB_SGL_BLK_LEN	 16
#define BCM_VK_IB_SGL_BLK_MAX		 24

enum pci_barno {
	BAR_0 = 0,
	BAR_1,
	BAR_2
};

#define BCM_VK_NUM_TTY 2

struct bcm_vk_tty {
	struct tty_port port;
	uint32_t to_offset;	/* bar offset to use */
	uint32_t to_size;	/* to VK buffer size */
	uint32_t wr;		/* write offset shadow */
	uint32_t from_offset;	/* bar offset to use */
	uint32_t from_size;	/* from VK buffer size */
	uint32_t rd;		/* read offset shadow */
};

/* VK device max power state, supports 3, full, reduced and low */
#define MAX_OPP 3
#define MAX_CARD_INFO_TAG_SIZE 64

struct bcm_vk_card_info {
	uint32_t version;
	char os_tag[MAX_CARD_INFO_TAG_SIZE];
	char cmpt_tag[MAX_CARD_INFO_TAG_SIZE];
	uint32_t cpu_freq_mhz;
	uint32_t cpu_scale[MAX_OPP];
	uint32_t ddr_freq_mhz;
	uint32_t ddr_size_MB;
	uint32_t video_core_freq_mhz;
};

struct bcm_vk {
	struct pci_dev *pdev;
	void __iomem *bar[MAX_BAR];
	int num_irqs;

	struct bcm_vk_card_info card_info;

#if defined(BCM_VK_LEGACY_API)
	struct msix_entry msix[32];
#endif
	/* mutex to protect the ioctls */
	struct mutex mutex;
	struct miscdevice miscdev;
	int misc_devid; /* dev id allocated */

	struct tty_driver *tty_drv;
	struct timer_list serial_timer;
	spinlock_t timer_lock;
	struct bcm_vk_tty tty[BCM_VK_NUM_TTY];

	/* Reference-counting to handle file operations */
	struct kref kref;

	spinlock_t msg_id_lock;
	uint16_t msg_id;
	DECLARE_BITMAP(bmap, VK_MSG_ID_BITMAP_SIZE);
	spinlock_t ctx_lock;
	struct bcm_vk_ctx ctx[VK_CMPT_CTX_MAX];
	struct bcm_vk_ht_entry pid_ht[VK_PID_HT_SZ];
	struct task_struct *reset_ppid; /* process that issue reset */

	bool msgq_inited; /* indicate if info has been synced with vk */
	struct bcm_vk_msg_chan h2vk_msg_chan;
	struct bcm_vk_msg_chan vk2h_msg_chan;

	struct workqueue_struct *wq_thread;
	struct work_struct wq_work; /* work queue for deferred job */
	unsigned long wq_offload; /* various flags on wq requested */
	void *tdma_vaddr; /* test dma segment virtual addr */
	dma_addr_t tdma_addr; /* test dma segment bus addr */

	struct notifier_block panic_nb;
	uint32_t ib_sgl_size; /* size allocated for inband sgl insertion */
};

/* wq offload work items bits definitions */
#define BCM_VK_WQ_DWNLD_PEND	0
#define BCM_VK_WQ_DWNLD_AUTO	1

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

static inline u8 vkread8(struct bcm_vk *vk,
			 enum pci_barno bar,
			 uint64_t offset)
{
	u8 value;

	value = ioread8(vk->bar[bar] + offset);
	return value;
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
bool bcm_vk_msgq_marker_valid(struct bcm_vk *vk);
int bcm_vk_send_shutdown_msg(struct bcm_vk *vk, uint32_t shut_type, pid_t pid);
void bcm_vk_trigger_reset(struct bcm_vk *vk);
void bcm_h2vk_doorbell(struct bcm_vk *vk, uint32_t q_num, uint32_t db_val);
int bcm_vk_auto_load_all_images(struct bcm_vk *vk);
int bcm_vk_tty_init(struct bcm_vk *vk, char *name);
void bcm_vk_tty_exit(struct bcm_vk *vk);

#if defined(BCM_VK_LEGACY_API)

/*
 * For legacy kernels, the following 2 PCI APIs will be missing, and
 * have to use msix_entry[] instead.  The APIs are provided in file bcm_vk_pci.c
 */
#define PCI_IRQ_MSI		BIT(0)
#define PCI_IRQ_MSIX		BIT(1)

int pci_irq_vector(struct pci_dev *pdev, unsigned int nr);
int pci_alloc_irq_vectors(struct pci_dev *pdev, unsigned int min_vecs,
			  unsigned int max_vecs, unsigned int flags);

#endif

#endif
