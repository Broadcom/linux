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

struct cmem_node {
	struct resource res;
	struct miscdevice cmem_dev;
};

static int cmem_fault(struct vm_fault *vmf)
{
	struct vm_area_struct *vma = vmf->vma;
	struct cmem_node *cmem;
	unsigned int offset;
	int ret;

	cmem  = container_of(vma->vm_private_data, struct cmem_node, cmem_dev);

	offset = vmf->address - vma->vm_start;
	if (offset < (cmem->res.end - cmem->res.start)) {
		ret = vm_insert_pfn(vma, vmf->address,
				    PFN_DOWN(cmem->res.start + offset));
		if (!ret)
			return VM_FAULT_NOPAGE;
	}

	return VM_FAULT_SIGBUS;
}

const struct vm_operations_struct cmem_vm_ops = {
	.fault = cmem_fault,
};

static int cmem_open(struct inode *inode, struct file *filp)
{
	/**
	 * cmem opens at resource start address i.e.
	 * filp->f_pos = cmem->res.start;
	 */
	return capable(CAP_SYS_RAWIO) ? 0 : -EPERM;
}

static int cmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct cmem_node *cmem;

	if (vma->vm_pgoff)
		return -EINVAL;

	vma->vm_ops = &cmem_vm_ops;
	vma->vm_flags |= (VM_DONTEXPAND | VM_DONTDUMP | VM_PFNMAP | VM_IO);
	vma->vm_private_data = file->private_data;

	/* set vm_pgoff to first pfn for PFNMAP */
	cmem  = container_of(file->private_data, struct cmem_node, cmem_dev);
	vma->vm_pgoff = PFN_DOWN(cmem->res.start);

	return 0;
}

static loff_t cmem_llseek(struct file *file, loff_t offset, int origin)
{
	struct cmem_node *cmem;

	if (offset)
		return -ENXIO;

	cmem  = container_of(file->private_data, struct cmem_node, cmem_dev);
	switch (origin) {
	case SEEK_END:
		return cmem->res.end;
	case SEEK_CUR:
		return cmem->res.start;
	}

	return -ENXIO;
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

