// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2018 Broadcom
 */

#include "bcm_vk.h"
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/sizes.h>
#include <uapi/linux/misc/bcm_vk.h>

#define DRV_MODULE_NAME		"bcm-vk"

#define PCI_DEVICE_ID_VALKYRIE	0x5E87

static DEFINE_IDA(bcm_vk_ida);

/* Location of registers of interest in BAR0 */
#define BAR_FB_REQ		0x400
#define BAR_CODEPUSH_ADDRESS	0x404
#define CODEPUSH_FASTBOOT	BIT(0)
#define CODEPUSH_FIRMWARE	BIT(1)
#define BAR_FB_RAM_OPEN		0x40C
#define RAM_OPEN_SRAM		BIT(16)
#define RAM_OPEN_DDR		BIT(17)
#define BAR_CARD_STATUS		0x410
#define BAR_FW_STATUS		0x41C

#define BAR_METADATA_VERSION	0x440
#define BAR_FIRMWARE_VERSION	0x444

/* Location of memory base addresses of interest in BAR1 */
/* Load Boot1 to start of ITCM */
#define BAR1_CODEPUSH_BASE_FASTBOOT	0x100000
/* Load Boot2 to start of DDR0 */
#define BAR1_CODEPUSH_BASE_FIRMWARE	0x300000

#define VK_MSIX_IRQ_MAX			3

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

static long bcm_vk_load_image(struct bcm_vk *vk, struct vk_image *arg)
{
	struct device *dev = &vk->pdev->dev;
	const struct firmware  *fw;
	struct vk_image image;
	long ret = 0;
	unsigned long i;
	uint64_t offset;
	u32 codepush;
	u32 ram_open;
	int timeout_ms = 100; /* Allow minimum 100ms for timeout responses */

	if (copy_from_user(&image, arg, sizeof(image))) {
		ret = -EACCES;
		goto err_out;
	}
	dev_dbg(dev, "image type 0x%x", image.type);

	ret = request_firmware(&fw, image.filename, dev);
	if (ret) {
		dev_err(dev, "Error %ld requesting firmware file: %s\n",
			ret, image.filename);
		goto err_out;
	}
	dev_dbg(dev, "size=0x%zx\n", fw->size);

	if (image.type == VK_IMAGE_TYPE_BOOT1) {
		offset = BAR1_CODEPUSH_BASE_FASTBOOT;
		codepush = CODEPUSH_FASTBOOT;
		if (fw->size > SZ_256K) {
			dev_err(dev, "Error size 0x%zx > 256K\n", fw->size);
			ret = -EINVAL;
			goto err_firmware_out;
		}

		ram_open = vkread32(vk, BAR_0, BAR_FB_RAM_OPEN);
		dev_dbg(dev, "ram_open=0x%x\n", ram_open);

		/* Write a 1 to request SRAM open bit */
		vkwrite32(vk, 1, BAR_0, BAR_FB_REQ);

		/* Wait for SRAM to open */
		do {
			ram_open = vkread32(vk, BAR_0, BAR_FB_RAM_OPEN);
			dev_dbg(dev, "ram_open=0x%x\n", ram_open);
			if (ram_open & RAM_OPEN_SRAM)
				break;

			/* Sleep minimum of 1ms per loop */
			usleep_range(1000, 10000);
			timeout_ms--;
		} while (timeout_ms);
	} else if (image.type == VK_IMAGE_TYPE_BOOT2) {
		offset = BAR1_CODEPUSH_BASE_FIRMWARE;
		codepush = CODEPUSH_FIRMWARE;
		if (fw->size > SZ_2M) {
			dev_err(dev, "Error size 0x%zx > 2M\n", fw->size);
			ret = -EINVAL;
			goto err_firmware_out;
		}

		do {
			ram_open = vkread32(vk, BAR_0, BAR_FB_RAM_OPEN);
			dev_dbg(dev, "ram_open=0x%x\n", ram_open);
			if (ram_open & RAM_OPEN_DDR)
				break;

			/* Sleep minimum of 1ms per loop */
			usleep_range(1000, 10000);
			timeout_ms--;
		} while (timeout_ms);
	} else {
		dev_err(dev, "Error invalid image type 0x%x\n", image.type);
		ret = -EINVAL;
		goto err_firmware_out;
	}

	if (timeout_ms <= 0) {
		dev_err(dev, "timeout\n");
		ret = -ETIMEDOUT;
		goto err_firmware_out;
	}

	for (i = 0; i < fw->size; i += sizeof(u32))
		vkwrite32(vk, fw->data[i], BAR_1, offset + i);

	dev_dbg(dev, "Signaling 0x%x\n", codepush);
	vkwrite32(vk, codepush, BAR_0, BAR_CODEPUSH_ADDRESS);

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
			value = vkread32(vk, access.barno, access.offset + i);
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

			vkwrite32(vk, value, access.barno, access.offset + i);
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

	if (copy_from_user(&reset, arg, sizeof(struct vk_reset))) {
		ret = -EACCES;
		goto err_out;
	}
	dev_err(dev, "TODO: Issue Reset 0x%x, 0x%x\n", reset.arg1, reset.arg2);

err_out:
	return ret;
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

	pci_set_master(pdev);
	pci_set_drvdata(pdev, vk);

	irq = pci_alloc_irq_vectors(pdev,
				    1,
				    VK_MSIX_IRQ_MAX,
				    PCI_IRQ_MSI | PCI_IRQ_MSIX);

	if (irq < 0) {
		dev_err(dev, "failed to get MSIX interrupts\n");
		err = irq;
		goto err_disable_pdev;
	}

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

	dev_info(dev, "BCM-VK:%u\n", id);

	return 0;

err_kfree_name:
	kfree(misc_device->name);
	misc_device->name = NULL;

err_ida_remove:
	ida_simple_remove(&bcm_vk_ida, id);

err_irq:
	for (i = 0; i < vk->num_irqs; i++)
		devm_free_irq(dev, pci_irq_vector(pdev, i), vk);

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
	int id;
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	struct miscdevice *misc_device = &vk->miscdev;

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

	pci_disable_msi(pdev);
	for (i = 0; i < MAX_BAR; i++) {
		if (vk->bar[i])
			pci_iounmap(pdev, vk->bar[i]);
	}
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
