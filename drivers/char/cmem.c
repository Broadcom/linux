/*
 * Copyright (C) 2018 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/device.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define CMEM_DEVICE_NAME   "cmem"
#define VM_RESERVED        (VM_DONTEXPAND | VM_DONTDUMP)

struct cmem_node {
	struct resource res;
	struct miscdevice cmem_dev;
};

static int cmem_fault(struct vm_fault *vmf)
{
	struct page *page;
	struct vm_area_struct *vma = vmf->vma;
	unsigned int offset;
	struct cmem_node *cmem;

	cmem  = container_of(vma->vm_private_data, struct cmem_node, cmem_dev);

	offset = vmf->address - vma->vm_start;
	if (offset < (cmem->res.end - cmem->res.start)) {
		page = pfn_to_page(PFN_DOWN(cmem->res.start + offset));
		get_page(page);
		vmf->page = page;
	}

	return 0;
}

const struct vm_operations_struct cmem_vm_ops = {
	.fault = cmem_fault,
};

static int cmem_open(struct inode *inode, struct file *filp)
{
	return capable(CAP_SYS_RAWIO) ? 0 : -EPERM;
}

static int cmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	vma->vm_ops = &cmem_vm_ops;
	vma->vm_flags |= VM_RESERVED;
	vma->vm_private_data = file->private_data;

	return 0;
}

static loff_t cmem_llseek(struct file *file, loff_t offset, int origin)
{
	struct cmem_node *cmem;
	unsigned int cmem_size;

	cmem  = container_of(file->private_data, struct cmem_node, cmem_dev);
	cmem_size = cmem->res.end - cmem->res.start;

	return generic_file_llseek_size(file, offset, origin, cmem_size,
					cmem_size & PAGE_MASK);
}

static const struct file_operations cmem_fops = {
	.owner = THIS_MODULE,
	.mmap = cmem_mmap,
	.open = cmem_open,
	.llseek = cmem_llseek,
};

static int cmem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cmem_node *cmem;
	struct device_node *np;
	int ret;

	cmem = devm_kzalloc(dev, sizeof(*cmem), GFP_KERNEL);
	if (!cmem)
		return -ENOMEM;

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		dev_err(dev, "No %s specified\n", "memory-region");
		return -ENOMEM;
	}

	ret = of_address_to_resource(np, 0, &cmem->res);
	if (ret) {
		dev_err(dev, "No memory address assigned to the region\n");
		return -ENOMEM;
	}

	cmem->cmem_dev.minor = 255,
	cmem->cmem_dev.name = CMEM_DEVICE_NAME,
	cmem->cmem_dev.fops = &cmem_fops,
	cmem->cmem_dev.nodename = CMEM_DEVICE_NAME,
	ret = misc_register(&cmem->cmem_dev);
	if (ret) {
		dev_err(dev, "misc device register failed\n");
		return ret;
	}
	platform_set_drvdata(pdev, cmem);

	return 0;
}

static int cmem_remove(struct platform_device *pdev)
{
	struct cmem_node *cmem = platform_get_drvdata(pdev);

	misc_deregister(&cmem->cmem_dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id cmem_of_match[] = {
	{ .compatible = "mmap-cmem", },
	{},
};
MODULE_DEVICE_TABLE(of, cmem_of_match);

static struct platform_driver cmem_driver = {
	.driver = {
		.name = "mmap-cmem",
		.of_match_table = cmem_of_match,
	},
	.probe    = cmem_probe,
	.remove   = cmem_remove,
};
module_platform_driver(cmem_driver);


MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Srinath Mannam");
MODULE_DESCRIPTION("cmem driver");
MODULE_ALIAS("devname:cmem");

