// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2018 Broadcom
 */

#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/pci_regs.h>

#define DRV_MODULE_NAME "bcm-vk"

static DEFINE_IDA(bcm_vk_ida);

struct bcm_vk {
	struct pci_dev *pdev;
	struct miscdevice miscdev;
};

static long bcm_vk_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = -EINVAL;
	return ret;
}

static const struct file_operations bcm_vk_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = bcm_vk_ioctl,
};

static int bcm_vk_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int err;
	int id;
	char name[20];
	struct bcm_vk *vk;
	struct device *dev = &pdev->dev;
	struct miscdevice *misc_device;

	vk = devm_kzalloc(dev, sizeof(*vk), GFP_KERNEL);
	if (!vk)
		return -ENOMEM;

	vk->pdev = pdev;

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

	id = ida_simple_get(&bcm_vk_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		err = id;
		dev_err(dev, "unable to get id\n");
		goto err_iounmap;
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

	return 0;

err_kfree_name:
	kfree(misc_device->name);

err_ida_remove:
	ida_simple_remove(&bcm_vk_ida, id);

err_iounmap:
	pci_disable_msi(pdev);
	pci_release_regions(pdev);

err_disable_pdev:
	pci_disable_device(pdev);

	return err;
}

static void bcm_vk_remove(struct pci_dev *pdev)
{
	int id;
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	struct miscdevice *misc_device = &vk->miscdev;

	if (sscanf(misc_device->name, DRV_MODULE_NAME ".%d", &id) != 1)
		return;
	if (id < 0)
		return;

	misc_deregister(&vk->miscdev);
	kfree(misc_device->name);
	ida_simple_remove(&bcm_vk_ida, id);
	pci_disable_msi(pdev);
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
