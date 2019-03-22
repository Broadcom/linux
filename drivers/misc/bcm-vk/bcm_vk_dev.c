// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2018 Broadcom
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/sched/signal.h>
#include <linux/sizes.h>
#include <uapi/linux/misc/bcm_vk.h>

#include "bcm_vk.h"

#define DRV_MODULE_NAME		"bcm-vk"

#define PCI_DEVICE_ID_VALKYRIE	0x5E87

static DEFINE_IDA(bcm_vk_ida);

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
#define FW_STATUS_ZEPHYR_READY	(FW_STATUS_RELOCATION_ENTRY | \
				 FW_STATUS_RELOCATION_EXIT | \
				 FW_STATUS_ZEPHYR_INIT_START | \
				 FW_STATUS_ZEPHYR_ARCH_INIT_DONE | \
				 FW_STATUS_ZEPHYR_PRE_KERNEL1_INIT_DONE | \
				 FW_STATUS_ZEPHYR_PRE_KERNEL2_INIT_DONE | \
				 FW_STATUS_ZEPHYR_POST_KERNEL_INIT_DONE | \
				 FW_STATUS_ZEPHYR_INIT_DONE | \
				 FW_STATUS_ZEPHYR_APP_INIT_START | \
				 FW_STATUS_ZEPHYR_APP_INIT_DONE)


/* Location of memory base addresses of interest in BAR1 */
/* Load Boot1 to start of ITCM */
#define BAR1_CODEPUSH_BASE_BOOT1	0x100000
/* Load Boot2 to start of DDR0 */
#define BAR1_CODEPUSH_BASE_BOOT2	0x300000
/* Allow minimum 1s for Load Image timeout responses */
#define LOAD_IMAGE_TIMEOUT_MS		1000

#define VK_MSIX_IRQ_MAX			3

#define BCM_VK_DMA_BITS			64

#define BCM_VK_MIN_RESET_TIME_SEC	2

static long bcm_vk_get_metadata(struct bcm_vk *vk, struct vk_metadata *arg)
{
	struct device *dev = &vk->pdev->dev;
	struct vk_metadata metadata;
	long ret = 0;

	metadata.version = vkread32(vk, BAR_0, BAR_METADATA_VERSION);
	dev_dbg(dev, "version=0x%x\n", metadata.version);
	metadata.card_status = vkread32(vk, BAR_0, BAR_CARD_STATUS);
	dev_dbg(dev, "card_status=0x%x\n", metadata.card_status);
	metadata.firmware_version = vkread32(vk, BAR_0, BAR_FIRMWARE_VERSION);
	dev_dbg(dev, "firmware_version=0x%x\n", metadata.firmware_version);
	metadata.fw_status = vkread32(vk, BAR_0, BAR_FW_STATUS);
	dev_dbg(dev, "fw_status=0x%x\n", metadata.fw_status);

	if (copy_to_user(arg, &metadata, sizeof(metadata)))
		ret = -EFAULT;

	return ret;
}

static inline int bcm_vk_wait(struct bcm_vk *vk, enum pci_barno bar,
			      uint64_t offset, u32 mask, u32 value,
			      unsigned long timeout_ms)
{
	struct device *dev = &vk->pdev->dev;
	unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);
	u32 rd_val;

	do {
		rd_val = vkread32(vk, bar, offset);
		dev_dbg(dev, "BAR%d Offset=0x%llx: 0x%x\n",
			bar, offset, rd_val);

		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;

		cpu_relax();
		cond_resched();
	} while ((rd_val & mask) != value);

	return 0;
}

static long bcm_vk_load_image(struct bcm_vk *vk, struct vk_image *arg)
{
	struct device *dev = &vk->pdev->dev;
	const struct firmware  *fw;
	struct vk_image image;
	int ret;
	unsigned long i;
	uint64_t offset;
	uint64_t offset_codepush;
	u32 codepush;
	u32 ram_open;
	int remainder;

	if (copy_from_user(&image, arg, sizeof(image))) {
		ret = -EACCES;
		goto err_out;
	}
	dev_dbg(dev, "image type: 0x%x name: %s\n",
		image.type, image.filename);

	ret = request_firmware(&fw, image.filename, dev);
	if (ret) {
		dev_err(dev, "Error %d requesting firmware file: %s\n",
			ret, image.filename);
		goto err_out;
	}
	dev_dbg(dev, "size=0x%zx\n", fw->size);

	if (image.type == VK_IMAGE_TYPE_BOOT1) {
		offset = BAR1_CODEPUSH_BASE_BOOT1;
		codepush = CODEPUSH_FASTBOOT + CODEPUSH_BOOT1_ENTRY;
		offset_codepush = BAR_CODEPUSH_SBL;
		if (fw->size > SZ_256K) {
			dev_err(dev, "Error size 0x%zx > 256K\n", fw->size);
			ret = -EFBIG;
			goto err_firmware_out;
		}

		ram_open = vkread32(vk, BAR_0, BAR_FB_OPEN);
		dev_dbg(dev, "ram_open=0x%x\n", ram_open);

		/* Write a 1 to request SRAM open bit */
		vkwrite32(vk, CODEPUSH_FASTBOOT, BAR_0, offset_codepush);

		/* Wait for VK to respond */
		ret = bcm_vk_wait(vk, BAR_0, BAR_FB_OPEN, SRAM_OPEN, SRAM_OPEN,
				  LOAD_IMAGE_TIMEOUT_MS);
	} else if (image.type == VK_IMAGE_TYPE_BOOT2) {
		offset = BAR1_CODEPUSH_BASE_BOOT2;
		codepush = CODEPUSH_FASTBOOT + CODEPUSH_BOOT2_ENTRY;
		offset_codepush = BAR_CODEPUSH_SBI;
		if (fw->size > SZ_2M) {
			dev_err(dev, "Error size 0x%zx > 2M\n", fw->size);
			ret = -EFBIG;
			goto err_firmware_out;
		}

		/* Wait for VK to respond */
		ret = bcm_vk_wait(vk, BAR_0, BAR_FB_OPEN, DDR_OPEN, DDR_OPEN,
				  LOAD_IMAGE_TIMEOUT_MS);
	} else {
		dev_err(dev, "Error invalid image type 0x%x\n", image.type);
		ret = -EINVAL;
		goto err_firmware_out;
	}

	if (ret < 0) {
		dev_err(dev, "timeout\n");
		goto err_firmware_out;
	}

	remainder = fw->size % sizeof(u32);
	for (i = 0; i < (fw->size - remainder); i += sizeof(u32))
		vkwrite32(vk, *((u32 *)&fw->data[i]), BAR_1, offset + i);

	/* for image that has sizes not divisible by 4 */
	while (i < fw->size) {
		vkwrite8(vk, fw->data[i], BAR_1, offset + i);
		i++;
	}

	dev_dbg(dev, "Signaling 0x%x to 0x%llx\n", codepush, offset_codepush);
	vkwrite32(vk, codepush, BAR_0, offset_codepush);

	/* Initialize Message Q if we are loading boot2 */
	if (image.type == VK_IMAGE_TYPE_BOOT2) {
		/* wait for fw status bits to indicate Zephyr app ready */
		ret = bcm_vk_wait(vk, BAR_0, BAR_FW_STATUS, FW_STATUS_MASK,
				  FW_STATUS_ZEPHYR_READY,
				  LOAD_IMAGE_TIMEOUT_MS);
		if (ret < 0) {
			dev_err(dev, "Boot2 MSG Q not ready - timeout\n");
			goto err_firmware_out;
		}

		/* sync queues when zephyr is up */
		if (bcm_vk_sync_msgq(vk)) {
			dev_err(dev, "Error reading comm msg Q info\n");
			ret = -EIO;
			goto err_firmware_out;
		}
	}

err_firmware_out:
	release_firmware(fw);

err_out:
	return ret;
}

static long bcm_vk_access_bar(struct bcm_vk *vk, struct vk_access *arg)
{
	struct device *dev = &vk->pdev->dev;
	struct vk_access access;
	long ret = 0;
	u32 value;
	long i;
	long num;

	if (copy_from_user(&access, arg, sizeof(struct vk_access))) {
		ret = -EACCES;
		goto err_out;
	}
	dev_dbg(dev, "barno=0x%x\n", access.barno);
	dev_dbg(dev, "type=0x%x\n", access.type);
	if (access.type == VK_ACCESS_READ) {
		dev_dbg(dev, "read barno:%d offset:0x%llx len:0x%x\n",
			access.barno, access.offset, access.len);
		num = access.len / sizeof(u32);
		for (i = 0; i < num; i++) {
			value = vkread32(vk, access.barno,
					 access.offset + (i * sizeof(u32)));
			ret = put_user(value, access.data + i);
			if (ret)
				goto err_out;

			dev_dbg(dev, "0x%x\n", value);
		}
	} else if (access.type == VK_ACCESS_WRITE) {
		dev_dbg(dev, "write barno:%d offset:0x%llx len:0x%x\n",
			access.barno, access.offset, access.len);
		num = access.len / sizeof(u32);
		for (i = 0; i < num; i++) {
			ret = get_user(value, access.data + i);
			if (ret)
				goto err_out;

			vkwrite32(vk, value, access.barno,
				  access.offset + (i * sizeof(u32)));
			dev_dbg(dev, "0x%x\n", value);
		}
	} else {
		dev_dbg(dev, "error\n");
		ret = -EINVAL;
		goto err_out;
	}
err_out:
	return ret;
}

static long bcm_vk_reset(struct bcm_vk *vk, struct vk_reset *arg)
{
	struct device *dev = &vk->pdev->dev;
	struct vk_reset reset;
	long ret = 0;
	int i;
	u32 ram_open;

	if (copy_from_user(&reset, arg, sizeof(struct vk_reset))) {
		ret = -EACCES;
		goto err_out;
	}
	dev_info(dev, "Issue Reset 0x%x, 0x%x\n", reset.arg1, reset.arg2);
	if (reset.arg2 < BCM_VK_MIN_RESET_TIME_SEC)
		reset.arg2 = BCM_VK_MIN_RESET_TIME_SEC;

	spin_lock(&vk->ctx_lock);

	/*
	 * check if someone has already issued the reset, and if so, return
	 * error
	 */
	if (!vk->reset_ppid) {

		vk->reset_ppid = current;
		for (i = 0; i < VK_PID_HT_SZ; i++) {

			struct bcm_vk_ctx *p_ctx;

			list_for_each_entry(p_ctx,
					    &vk->pid_ht[i].fd_head,
					    list_node) {

				if (p_ctx->p_pid != vk->reset_ppid) {
					dev_info(dev,
						 "Send kill signal to pid %d\n",
						 task_pid_nr(p_ctx->p_pid));
					kill_pid(task_pid(p_ctx->p_pid),
						 SIGKILL, 1);
				}
			}
		}

	} else {
		dev_err(dev, "Reset already launched by process pid %d\n",
			task_pid_nr(vk->reset_ppid));
		ret = -EACCES;
	}
	spin_unlock(&vk->ctx_lock);

	if (ret)
		goto err_out;

	/* sleep for time specified by arg2 in seconds */
	msleep(reset.arg2 * 1000);

	/* read BAR0 BAR_FB_OPEN register and dump out the value */
	ram_open = vkread32(vk, BAR_0, BAR_FB_OPEN);
	if (!(ram_open & SRAM_OPEN))
		ret = -EINVAL;

	dev_info(dev, "Reset completed - RB_OPEN = 0x%x SRAM_OPEN %s\n",
		 ram_open, ret ? "false" : "true");

err_out:
	return ret;
}

static int bcm_vk_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct bcm_vk_ctx *ctx = file->private_data;
	struct bcm_vk *vk = container_of(ctx->p_miscdev, struct bcm_vk,
					 miscdev);
	unsigned long pg_size;
	/* only BAR2 is mmap possible, which is bar num 4 due to 64bit */
#define VK_MMAPABLE_BAR	   4

	pg_size = ((pci_resource_len(vk->pdev, VK_MMAPABLE_BAR) - 1)
		    >> PAGE_SHIFT) + 1;
	if (vma->vm_pgoff + vma_pages(vma) > pg_size)
		return -EINVAL;

	vma->vm_pgoff += (pci_resource_start(vk->pdev, VK_MMAPABLE_BAR)
			  >> PAGE_SHIFT);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	return io_remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				  vma->vm_end - vma->vm_start,
				  vma->vm_page_prot);
}

static long bcm_vk_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = -EINVAL;
	struct bcm_vk_ctx *ctx = file->private_data;
	struct bcm_vk *vk = container_of(ctx->p_miscdev, struct bcm_vk,
					 miscdev);
	void __user *argp = (void __user *)arg;

	dev_dbg(&vk->pdev->dev,
		"ioctl, cmd=0x%02x, arg=0x%02lx\n",
		cmd, arg);

	mutex_lock(&vk->mutex);

	switch (cmd) {
	case VK_IOCTL_GET_METADATA:
		ret = bcm_vk_get_metadata(vk, argp);
		break;

	case VK_IOCTL_LOAD_IMAGE:
		ret = bcm_vk_load_image(vk, argp);
		break;

	case VK_IOCTL_ACCESS_BAR:
		ret = bcm_vk_access_bar(vk, argp);
		break;

	case VK_IOCTL_RESET:
		ret = bcm_vk_reset(vk, argp);
		break;

	default:
		break;
	}

	mutex_unlock(&vk->mutex);

	return ret;
}

static const struct file_operations bcm_vk_fops = {
	.owner = THIS_MODULE,
	.open = bcm_vk_open,
	.read = bcm_vk_read,
	.write = bcm_vk_write,
	.release = bcm_vk_release,
	.mmap = bcm_vk_mmap,
	.unlocked_ioctl = bcm_vk_ioctl,
};

static int bcm_vk_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int err;
	int i;
	int id;
	int irq;
	char name[20];
	struct bcm_vk *vk;
	struct device *dev = &pdev->dev;
	struct miscdevice *misc_device;

	vk = devm_kzalloc(dev, sizeof(*vk), GFP_KERNEL);
	if (!vk)
		return -ENOMEM;

	vk->pdev = pdev;

	mutex_init(&vk->mutex);

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(dev, "Cannot enable PCI device\n");
		return err;
	}

	err = pci_request_regions(pdev, DRV_MODULE_NAME);
	if (err) {
		dev_err(dev, "Cannot obtain PCI resources\n");
		goto err_disable_pdev;
	}

	/* make sure DMA is good */
	err = dma_set_mask_and_coherent(&pdev->dev,
					DMA_BIT_MASK(BCM_VK_DMA_BITS));
	if (err) {
		dev_err(dev, "failed to set DMA mask\n");
		goto err_disable_pdev;
	}

	pci_set_master(pdev);
	pci_set_drvdata(pdev, vk);

	irq = pci_alloc_irq_vectors(pdev,
				    1,
				    VK_MSIX_IRQ_MAX,
				    PCI_IRQ_MSI | PCI_IRQ_MSIX);

	if (irq < VK_MSIX_IRQ_MAX) {
		dev_err(dev, "failed to get %d MSIX interrupts, ret(%d)\n",
			VK_MSIX_IRQ_MAX, irq);
		err = (irq >= 0) ? -EINVAL : irq;
		goto err_disable_pdev;
	}

	dev_info(dev, "Number of IRQs %d allocated.\n", irq);

	for (i = 0; i < MAX_BAR; i++) {
		/* multiple by 2 for 64 bit BAR mapping */
		vk->bar[i] = pci_ioremap_bar(pdev, i * 2);
		if (!vk->bar[i]) {
			dev_err(dev, "failed to remap BAR%d\n", i);
			goto err_iounmap;
		}
	}

	for (vk->num_irqs = 0; vk->num_irqs < irq; vk->num_irqs++) {
		err = devm_request_irq(dev, pci_irq_vector(pdev, vk->num_irqs),
				       bcm_vk_irqhandler,
				       IRQF_SHARED, DRV_MODULE_NAME, vk);
		if (err) {
			dev_err(dev, "failed to request IRQ %d for MSIX %d\n",
				pdev->irq + vk->num_irqs, vk->num_irqs + 1);
			goto err_irq;
		}
	}

	id = ida_simple_get(&bcm_vk_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		err = id;
		dev_err(dev, "unable to get id\n");
		goto err_irq;
	}

	snprintf(name, sizeof(name), DRV_MODULE_NAME ".%d", id);
	misc_device = &vk->miscdev;
	misc_device->minor = MISC_DYNAMIC_MINOR;
	misc_device->name = kstrdup(name, GFP_KERNEL);
	if (!misc_device->name) {
		err = -ENOMEM;
		goto err_ida_remove;
	}
	misc_device->fops = &bcm_vk_fops,

	err = misc_register(misc_device);
	if (err) {
		dev_err(dev, "failed to register device\n");
		goto err_kfree_name;
	}

	err = bcm_vk_msg_init(vk);
	if (err) {
		dev_err(dev, "failed to init msg queue info\n");
		goto err_kfree_name;
	}

	dev_info(dev, "BCM-VK:%u created\n", id);

	return 0;

err_kfree_name:
	kfree(misc_device->name);
	misc_device->name = NULL;

err_ida_remove:
	ida_simple_remove(&bcm_vk_ida, id);

err_irq:
	for (i = 0; i < vk->num_irqs; i++)
		devm_free_irq(dev, pci_irq_vector(pdev, i), vk);

	pci_disable_msix(pdev);
	pci_disable_msi(pdev);

err_iounmap:
	for (i = 0; i < MAX_BAR; i++) {
		if (vk->bar[i])
			pci_iounmap(pdev, vk->bar[i]);
	}
	pci_release_regions(pdev);

err_disable_pdev:
	pci_disable_device(pdev);

	return err;
}

static void bcm_vk_remove(struct pci_dev *pdev)
{
	int i;
	int id = -1;
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	struct miscdevice *misc_device = &vk->miscdev;

	cancel_work_sync(&vk->vk2h_wq);
	bcm_vk_msg_remove(vk);

	/* remove if name is set which means misc dev registered */
	if (misc_device->name) {
		if (sscanf(misc_device->name, DRV_MODULE_NAME ".%d", &id) != 1)
			return;
		if (id < 0)
			return;

		misc_deregister(&vk->miscdev);
		kfree(misc_device->name);
		ida_simple_remove(&bcm_vk_ida, id);
	}
	for (i = 0; i < vk->num_irqs; i++)
		devm_free_irq(&pdev->dev, pci_irq_vector(pdev, i), vk);

	pci_disable_msix(pdev);
	pci_disable_msi(pdev);

	for (i = 0; i < MAX_BAR; i++) {
		if (vk->bar[i])
			pci_iounmap(pdev, vk->bar[i]);
	}

	dev_info(&pdev->dev, "BCM-VK:%d released\n", id);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static const struct pci_device_id bcm_vk_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_BROADCOM, PCI_DEVICE_ID_VALKYRIE), },
	{ }
};
MODULE_DEVICE_TABLE(pci, bcm_vk_ids);

static struct pci_driver pci_driver = {
	.name     = DRV_MODULE_NAME,
	.id_table = bcm_vk_ids,
	.probe    = bcm_vk_probe,
	.remove   = bcm_vk_remove,
};
module_pci_driver(pci_driver);

MODULE_DESCRIPTION("Broadcom Valkyrie Host Driver");
MODULE_AUTHOR("Scott Branden <scott.branden@broadcom.com>");
MODULE_LICENSE("GPL v2");
