// SPDX-License-Identifier: (GPL-2.0 OR BSD-2-Clause)
/******************************************************************************
 *
 *       Copyright (C) 2015-2019 Ichiro Kawazome
 *       All rights reserved.
 *
 *       Redistribution and use in source and binary forms, with or without
 *       modification, are permitted provided that the following conditions
 *       are met:
 *
 *         1. Redistributions of source code must retain the above copyright
 *            notice, this list of conditions and the following disclaimer.
 *
 *         2. Redistributions in binary form must reproduce the above copyright
 *            notice, this list of conditions and the following disclaimer in
 *            the documentation and/or other materials provided with the
 *            distribution.
 *
 *       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *       A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 *       OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *       SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *       LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *       DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *       THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *       (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *       OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include <asm/page.h>
#include <asm/byteorder.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-map-ops.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/pagemap.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/sysctl.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#define DRIVER_VERSION			"2.1.2"
#define DRIVER_NAME			"dma-pubuf"
#define DEVICE_NAME_FORMAT		"dmapubuf%d"
#define DEVICE_MAX_NUM			256
#define DMAPUBUF_DEBUG			1
#define DMAPUBUF_MGR_NAME		"dma-pubuf-mgr"

#define SYNC_MODE_INVALID		0x00
#define SYNC_MODE_NONCACHED		0x01
#define SYNC_MODE_WRITECOMBINE		0x02
#define SYNC_MODE_DMACOHERENT		0x03
#define SYNC_MODE_MASK			0x03
#define SYNC_MODE_MIN			0x01
#define SYNC_MODE_MAX			0x03
#define SYNC_ALWAYS			0x04

#define SYNC_COMMAND_DIR_MASK		0x000000000000000C
#define SYNC_COMMAND_DIR_SHIFT		2
#define SYNC_COMMAND_SIZE_MASK		0x00000000FFFFFFF0
#define SYNC_COMMAND_SIZE_SHIFT		0
#define SYNC_COMMAND_OFFSET_MASK	0xFFFFFFFF00000000
#define SYNC_COMMAND_OFFSET_SHIFT	32
#define SYNC_COMMAND_ARGMENT_MASK	0xFFFFFFFFFFFFFFFE

#if DMAPUBUF_DEBUG
#define DMAPUBUF_DEBUG_CHECK(this, debug) (this->debug)
#else
#define DMAPUBUF_DEBUG_CHECK(this, debug) 0
#endif

#define DMAPUBUF_MGR_BUFFER_SIZE 256

#define DEF_ATTR_SHOW(__attr_name, __format, __value)			\
	static ssize_t dmapubuf_show_ ## __attr_name			\
					(struct device *dev,		\
					 struct device_attribute *attr,	\
					 char *buf)			\
{									\
	ssize_t status;							\
	struct dmapubuf_device_data *this = dev_get_drvdata(dev);	\
	if (mutex_lock_interruptible(&this->sem) != 0)			\
		return -ERESTARTSYS;					\
	status = sprintf(buf, __format, __value);			\
	mutex_unlock(&this->sem);					\
	return status;							\
}


#define DEF_ATTR_SET(__attr_name, __min, __max, __pre_action, __post_action) \
	static ssize_t dmapubuf_set_ ## __attr_name			\
					(struct device *dev,		\
					 struct device_attribute *attr,	\
					 const char *buf, size_t size)	\
{									\
	ssize_t status;							\
	u64 value;							\
	struct dmapubuf_device_data *this = dev_get_drvdata(dev);	\
	if (mutex_lock_interruptible(&this->sem) != 0)			\
		return -ERESTARTSYS;					\
	status = kstrtoull(buf, 0, &value);				\
	if (status != 0)						\
		goto failed;						\
	if ((value < __min) || (__max < value)) {			\
		status = -EINVAL;					\
		goto failed;						\
	}								\
	status = __pre_action(this);					\
	if (status != 0)						\
		goto failed;						\
	this->__attr_name = value;					\
	status = __post_action(this);					\
	if (status != 0)						\
		goto failed;						\
	status = size;							\
failed:									\
	mutex_unlock(&this->sem);					\
	return status;							\
}

/* dmapubuf_sys_class - dmapubuf system class */
static struct class *dmapubuf_sys_class;

/* info_enable module parameter */
static int info_enable = 1;
module_param(info_enable, int, 0644);
MODULE_PARM_DESC(info_enable, "dmapubuf install/uninstall information enable");

/* dma_mask_bit module parameter */
static int dma_mask_bit = 39;
module_param(dma_mask_bit, int, 0644);
MODULE_PARM_DESC(dma_mask_bit, "dmapubuf dma mask bit(default=32)");

/* dmapubuf device data structure */
struct dmapubuf_device_data {
	struct device *sys_dev;
	struct device *dma_dev;
	struct cdev cdev;
	dev_t device_number;
	struct mutex sem;
	bool is_open;
	int size;
	size_t alloc_size;
	void *virt_addr;
	dma_addr_t phys_addr;
	int sync_mode;
	u64 sync_offset;
	size_t sync_size;
	int sync_direction;
	bool sync_owner;
	u64 sync_for_cpu;
	u64 sync_for_device;
	bool of_reserved_mem;
#if DMAPUBUF_DEBUG
	bool debug_vma;
#endif
};

/**
 * DOC: dmapubuf System Class Device File Description
 *
 * This section define the device file created in system class when dmapubuf is
 * loaded into the kernel.
 *
 * The device file created in system class is as follows.
 *
 * * /sys/class/dma-pubuf/<device-name>/driver_version
 * * /sys/class/dma-pubuf/<device-name>/phys_addr
 * * /sys/class/dma-pubuf/<device-name>/size
 * * /sys/class/dma-pubuf/<device-name>/sync_mode
 * * /sys/class/dma-pubuf/<device-name>/sync_offset
 * * /sys/class/dma-pubuf/<device-name>/sync_size
 * * /sys/class/dma-pubuf/<device-name>/sync_direction
 * * /sys/class/dma-pubuf/<device-name>/sync_owner
 * * /sys/class/dma-pubuf/<device-name>/sync_for_cpu
 * * /sys/class/dma-pubuf/<device-name>/sync_for_device
 * * /sys/class/dma-pubuf/<device-name>/dma_coherent
 * *
 */

/**
 * Get argment for dma_sync_single_for_cpu() or dma_sync_single_for_device()
 *
 * @this:       Pointer to the dmapubuf device data structure.
 * @command     sync command (this->sync_for_cpu or this->sync_for_device)
 * @phys_addr   Pointer to the phys_addr for dma_sync_single_for_...()
 * @size        Pointer to the size for dma_sync_single_for_...()
 * @direction   Pointer to the direction for dma_sync_single_for_...()
 * Return:      Success(=0) or error status(<0).
 */
static int dmapubuf_sync_command_argments(struct dmapubuf_device_data *this,
					  u64 command, dma_addr_t *phys_addr,
					  size_t *size,
					  enum dma_data_direction *direction)
{
	u64 sync_offset;
	size_t sync_size;
	int sync_direction;

	if (command & SYNC_COMMAND_ARGMENT_MASK) {
		sync_offset = (u64)((command & SYNC_COMMAND_OFFSET_MASK) >>
				    SYNC_COMMAND_OFFSET_SHIFT);
		sync_size = (size_t)((command & SYNC_COMMAND_SIZE_MASK) >>
				     SYNC_COMMAND_SIZE_SHIFT);
		sync_direction = (int)((command & SYNC_COMMAND_DIR_MASK) >>
				       SYNC_COMMAND_DIR_SHIFT);
	} else {
		sync_offset = this->sync_offset;
		sync_size = this->sync_size;
		sync_direction = this->sync_direction;
	}
	if ((sync_offset + sync_size) > this->size)
		return -EINVAL;

	switch (sync_direction) {
	case 1:
		*direction = DMA_TO_DEVICE;
		break;
	case 2:
		*direction = DMA_FROM_DEVICE;
		break;
	default:
		*direction = DMA_BIDIRECTIONAL;
		break;
	}

	*phys_addr = this->phys_addr + sync_offset;
	*size = sync_size;
	return 0;
}

/**
 * Call dma_sync_single_for_cpu() when (sync_for_cpu != 0)
 * @this:       Pointer to the dmapubuf device data structure.
 * Return:      Success(=0) or error status(<0).
 */
static int dmapubuf_sync_for_cpu(struct dmapubuf_device_data *this)
{
	int status = 0;
	dma_addr_t phys_addr;
	size_t size;
	enum dma_data_direction direction;

	if (this->sync_for_cpu) {
		status = dmapubuf_sync_command_argments(this,
							this->sync_for_cpu,
							&phys_addr,
							&size,
							&direction);
		if (status == 0) {
			dma_sync_single_for_cpu(this->dma_dev,
						phys_addr,
						size,
						direction);
			this->sync_for_cpu = 0;
			this->sync_owner   = 0;
		}
	}
	return status;
}

/**
 * Call dma_sync_single_for_device() when (sync_for_device != 0)
 * @this:       Pointer to the dmapubuf device data structure.
 * Return:      Success(=0) or error status(<0).
 */
static int dmapubuf_sync_for_device(struct dmapubuf_device_data *this)
{
	int status = 0;
	dma_addr_t phys_addr;
	size_t size;
	enum dma_data_direction direction;

	if (this->sync_for_device) {
		status = dmapubuf_sync_command_argments(this,
							this->sync_for_device,
							&phys_addr, &size,
							&direction);
		if (status == 0) {
			dma_sync_single_for_device(this->dma_dev, phys_addr,
						   size, direction);
			this->sync_for_device = 0;
			this->sync_owner = 1;
		}
	}
	return status;
}

static inline int NO_ACTION(struct dmapubuf_device_data *this)
{
	return 0;
}

DEF_ATTR_SHOW(driver_version, "%s\n", DRIVER_VERSION);
DEF_ATTR_SHOW(size, "%d\n", this->size);
DEF_ATTR_SHOW(phys_addr, "%pad\n", &this->phys_addr);
DEF_ATTR_SHOW(sync_mode, "%d\n", this->sync_mode);
DEF_ATTR_SET(sync_mode, 0, 7, NO_ACTION, NO_ACTION);
DEF_ATTR_SHOW(sync_offset, "0x%llx\n", this->sync_offset);
DEF_ATTR_SET(sync_offset, 0, U64_MAX, NO_ACTION, NO_ACTION);
DEF_ATTR_SHOW(sync_size, "%zu\n", this->sync_size);
DEF_ATTR_SET(sync_size, 0, SIZE_MAX, NO_ACTION, NO_ACTION);
DEF_ATTR_SHOW(sync_direction, "%d\n", this->sync_direction);
DEF_ATTR_SET(sync_direction, 0, 2, NO_ACTION, NO_ACTION);
DEF_ATTR_SHOW(sync_owner, "%d\n", this->sync_owner);
DEF_ATTR_SHOW(sync_for_cpu, "%llu\n", this->sync_for_cpu);
DEF_ATTR_SET(sync_for_cpu, 0, U64_MAX,  NO_ACTION, dmapubuf_sync_for_cpu);
DEF_ATTR_SHOW(sync_for_device, "%llu\n", this->sync_for_device);
DEF_ATTR_SET(sync_for_device, 0, U64_MAX, NO_ACTION, dmapubuf_sync_for_device);
DEF_ATTR_SHOW(dma_coherent, "%d\n", dev_is_dma_coherent(this->dma_dev));
#if DMAPUBUF_DEBUG == 1
DEF_ATTR_SHOW(debug_vma, "%d\n", this->debug_vma);
DEF_ATTR_SET(debug_vma, 0, 1, NO_ACTION, NO_ACTION);
#endif

static struct device_attribute dmapubuf_device_attrs[] = {
	__ATTR(driver_version, 0444, dmapubuf_show_driver_version, NULL),
	__ATTR(size, 0444, dmapubuf_show_size, NULL),
	__ATTR(phys_addr, 0444, dmapubuf_show_phys_addr, NULL),
	__ATTR(sync_mode, 0664, dmapubuf_show_sync_mode,
	       dmapubuf_set_sync_mode),
	__ATTR(sync_offset, 0664, dmapubuf_show_sync_offset,
	       dmapubuf_set_sync_offset),
	__ATTR(sync_size, 0664, dmapubuf_show_sync_size,
	       dmapubuf_set_sync_size),
	__ATTR(sync_direction, 0664, dmapubuf_show_sync_direction,
	       dmapubuf_set_sync_direction),
	__ATTR(sync_owner, 0664, dmapubuf_show_sync_owner, NULL),
	__ATTR(sync_for_cpu, 0664, dmapubuf_show_sync_for_cpu,
	       dmapubuf_set_sync_for_cpu),
	__ATTR(sync_for_device, 0664, dmapubuf_show_sync_for_device,
	       dmapubuf_set_sync_for_device),
	__ATTR(dma_coherent, 0664, dmapubuf_show_dma_coherent, NULL),
#if DMAPUBUF_DEBUG == 1
	__ATTR(debug_vma, 0664, dmapubuf_show_debug_vma,
	       dmapubuf_set_debug_vma),
#endif
	__ATTR_NULL,
};

#define dmapubuf_device_attrs_size (sizeof(dmapubuf_device_attrs) / \
				    sizeof(dmapubuf_device_attrs[0]))

static struct attribute *dmapubuf_attrs[dmapubuf_device_attrs_size] = {
	NULL
};
static struct attribute_group dmapubuf_attr_group = {
	.attrs = dmapubuf_attrs
};
static const struct attribute_group *dmapubuf_attr_groups[] = {
	&dmapubuf_attr_group,
	NULL
};

static inline void dmapubuf_sys_class_set_attributes(void)
{
	int i;

	for (i = 0; i < dmapubuf_device_attrs_size - 1; i++)
		dmapubuf_attrs[i] = &(dmapubuf_device_attrs[i].attr);

	dmapubuf_attrs[i] = NULL;
	dmapubuf_sys_class->dev_groups = dmapubuf_attr_groups;
}

/**
 * DOC: dmapubuf Device VM Area Operations
 *
 * This section defines the operation of vm when mmap-ed the dmapubuf
 * device file.
 *
 * * dmapubuf_device_vma_open()  - dmapubuf device vm area open operation.
 * * dmapubuf_device_vma_close() - dmapubuf device vm area close operation.
 * * dmapubuf_device_vma_fault() - dmapubuf device vm area fault operation.
 * * dmapubuf_device_vm_ops      - dmapubuf device vm operation table.
 */

/**
 * dmapubuf device vm area open operation.
 * @vma:        Pointer to the vm area structure.
 * Return:      None
 */
static void dmapubuf_device_vma_open(struct vm_area_struct *vma)
{
	struct dmapubuf_device_data *this = vma->vm_private_data;

	if (DMAPUBUF_DEBUG_CHECK(this, debug_vma))
		dev_info(this->dma_dev, "vma_open(virt_addr=0x%lx, offset=0x%lx)\n",
			 vma->vm_start, vma->vm_pgoff<<PAGE_SHIFT);
}

/**
 * dmapubuf device vm area close operation.
 * @vma:        Pointer to the vm area structure.
 * Return:      None
 */
static void dmapubuf_device_vma_close(struct vm_area_struct *vma)
{
	struct dmapubuf_device_data *this = vma->vm_private_data;

	if (DMAPUBUF_DEBUG_CHECK(this, debug_vma))
		dev_info(this->dma_dev, "vma_close()\n");
}

/**
 * dmapubuf device vm area fault operation.
 * @vma:        Pointer to the vm area structure.
 * @vfm:        Pointer to the vm fault structure.
 * Return:      vm_fault_t (Success(=0) or error status(!=0)).
 */
static inline vm_fault_t _dmapubuf_device_vma_fault(
						struct vm_area_struct *vma,
						struct vm_fault *vmf)
{
	struct dmapubuf_device_data *this = vma->vm_private_data;
	unsigned long offset = vmf->pgoff << PAGE_SHIFT;
	unsigned long phys_addr = this->phys_addr + offset;
	unsigned long page_frame_num = phys_addr >> PAGE_SHIFT;
	unsigned long request_size = 1 << PAGE_SHIFT;
	unsigned long available_size = this->alloc_size - offset;
	unsigned long virt_addr;

	virt_addr = vmf->address;

	if (DMAPUBUF_DEBUG_CHECK(this, debug_vma))
		dev_info(this->dma_dev,
			 "vma_fault(virt_addr=%pad, phys_addr=%pad)\n",
			 &virt_addr, &phys_addr);

	if (request_size > available_size)
		return VM_FAULT_SIGBUS;

	if (!pfn_valid(page_frame_num))
		return VM_FAULT_SIGBUS;

	return vmf_insert_pfn(vma, virt_addr, page_frame_num);
}

/**
 * dmapubuf device vm area fault operation.
 * @vfm:        Pointer to the vm fault structure.
 * Return:      vm_fault_t (Success(=0) or error status(!=0)).
 */
static vm_fault_t dmapubuf_device_vma_fault(struct vm_fault *vmf)
{
	return _dmapubuf_device_vma_fault(vmf->vma, vmf);
}

static const struct vm_operations_struct dmapubuf_device_vm_ops = {
	.open    = dmapubuf_device_vma_open,
	.close   = dmapubuf_device_vma_close,
	.fault   = dmapubuf_device_vma_fault,
};

/**
 * DOC: dmapubuf Device File Operations
 *
 * This section defines the operation of the dmapubuf device file.
 *
 * * dmapubuf_device_file_open()    - dmapubuf device file open operation.
 * * dmapubuf_device_file_release() - dmapubuf device file release operation.
 * * dmapubuf_device_file_mmap()    - dmapubuf device file memory map operation.
 * * dmapubuf_device_file_read()    - dmapubuf device file read operation.
 * * dmapubuf_device_file_write()   - dmapubuf device file write operation.
 * * dmapubuf_device_file_llseek()  - dmapubuf device file llseek operation.
 * * dmapubuf_device_file_ops       - dmapubuf device file operation table.
 */

/**
 * dmapubuf device file open operation.
 * @inode:      Pointer to the inode structure of this device.
 * @file:       to the file structure.
 * Return:      Success(=0) or error status(<0).
 */
static int dmapubuf_device_file_open(struct inode *inode, struct file *file)
{
	struct dmapubuf_device_data *this;
	int status = 0;

	this = container_of(inode->i_cdev, struct dmapubuf_device_data, cdev);
	file->private_data = this;
	this->is_open = 1;

	return status;
}

/**
 * dmapubuf device file release operation.
 * @inode:      Pointer to the inode structure of this device.
 * @file:       Pointer to the file structure.
 * Return:      Success(=0) or error status(<0).
 */
static int dmapubuf_device_file_release(struct inode *inode, struct file *file)
{
	struct dmapubuf_device_data *this = file->private_data;

	this->is_open = 0;

	return 0;
}

/**
 * _PGPROT_NONCACHED : vm_page_prot value when((sync_mode & SYNC_MODE_MASK) ==
 *				SYNC_MODE_NONCACHED)
 * _PGPROT_WRITECOMBINE : vm_page_prot value when ((sync_mode &
 *						SYNC_MODE_MASK) ==
 *						SYNC_MODE_WRITECOMBINE)
 * _PGPROT_DMACOHERENT  : vm_page_prot value when ((sync_mode &
 *						SYNC_MODE_MASK) ==
 *						SYNC_MODE_DMACOHERENT )
 */
#if defined(CONFIG_ARM)
#define _PGPROT_NONCACHED(vm_page_prot)  pgprot_noncached(vm_page_prot)
#define _PGPROT_WRITECOMBINE(vm_page_prot) pgprot_writecombine(vm_page_prot)
#define _PGPROT_DMACOHERENT(vm_page_prot) pgprot_dmacoherent(vm_page_prot)
#elif defined(CONFIG_ARM64)
#define _PGPROT_NONCACHED(vm_page_prot) pgprot_noncached(vm_page_prot)
#define _PGPROT_WRITECOMBINE(vm_page_prot) pgprot_writecombine(vm_page_prot)
#define _PGPROT_DMACOHERENT(vm_page_prot) pgprot_writecombine(vm_page_prot)
#else
#define _PGPROT_NONCACHED(vm_page_prot) pgprot_noncached(vm_page_prot)
#define _PGPROT_WRITECOMBINE(vm_page_prot) pgprot_writecombine(vm_page_prot)
#define _PGPROT_DMACOHERENT(vm_page_prot) pgprot_writecombine(vm_page_prot)
#endif

/**
 * dmapubuf device file memory map operation.
 * @file:       Pointer to the file structure.
 * @vma:        Pointer to the vm area structure.
 * Return:      Success(=0) or error status(<0).
 */
static int dmapubuf_device_file_mmap(struct file *file,
				     struct vm_area_struct *vma)
{
	struct dmapubuf_device_data *this = file->private_data;
	unsigned long page_frame_num;

	if ((vma->vm_pgoff + vma_pages(vma)) > (this->alloc_size >> PAGE_SHIFT))
		return -EINVAL;

	if ((file->f_flags & O_SYNC) | (this->sync_mode & SYNC_ALWAYS)) {
		switch (this->sync_mode & SYNC_MODE_MASK) {
		case SYNC_MODE_NONCACHED:
			vma->vm_flags |= VM_IO;
			vma->vm_page_prot =
				_PGPROT_NONCACHED(vma->vm_page_prot);
			break;
		case SYNC_MODE_WRITECOMBINE:
			vma->vm_flags |= VM_IO;
			vma->vm_page_prot =
				_PGPROT_WRITECOMBINE(vma->vm_page_prot);
			break;
		case SYNC_MODE_DMACOHERENT:
			vma->vm_flags |= VM_IO;
			vma->vm_page_prot =
				_PGPROT_DMACOHERENT(vma->vm_page_prot);
			break;
		default:
			break;
		}
	}
	vma->vm_private_data = this;

	page_frame_num = ((this->phys_addr >> PAGE_SHIFT) + vma->vm_pgoff);

	if (pfn_valid(page_frame_num)) {
		vma->vm_flags |= VM_PFNMAP;
		vma->vm_ops    = &dmapubuf_device_vm_ops;
		dmapubuf_device_vma_open(vma);
		return 0;
	}

	return dma_mmap_coherent(this->dma_dev, vma, this->virt_addr,
				 this->phys_addr, this->alloc_size);
}

/**
 * dmapubuf device file read operation.
 * @file:       Pointer to the file structure.
 * @buff:       Pointer to the user buffer.
 * @count:      The number of bytes to be read.
 * @ppos:       Pointer to the offset value.
 * Return:      Transferred size.
 */
static ssize_t dmapubuf_device_file_read(struct file *file, char __user *buff,
					 size_t count, loff_t *ppos)
{
	struct dmapubuf_device_data *this = file->private_data;
	int result = 0;
	size_t xfer_size;
	dma_addr_t phys_addr;
	void *virt_addr;

	if (mutex_lock_interruptible(&this->sem))
		return -ERESTARTSYS;

	if (*ppos >= this->size) {
		result = 0;
		goto return_unlock;
	}

	phys_addr = this->phys_addr + *ppos;
	virt_addr = this->virt_addr + *ppos;
	xfer_size = (*ppos + count >= this->size) ? this->size - *ppos : count;

	if ((file->f_flags & O_SYNC) | (this->sync_mode & SYNC_ALWAYS))
		dma_sync_single_for_cpu(this->dma_dev, phys_addr, xfer_size,
					DMA_FROM_DEVICE);

	if (copy_to_user(buff, virt_addr, xfer_size) != 0) {
		result = 0;
		goto return_unlock;
	}

	if ((file->f_flags & O_SYNC) | (this->sync_mode & SYNC_ALWAYS))
		dma_sync_single_for_device(this->dma_dev, phys_addr, xfer_size,
					   DMA_FROM_DEVICE);

	*ppos += xfer_size;
	result = xfer_size;
return_unlock:
	mutex_unlock(&this->sem);
	return result;
}

/**
 * dmapubuf device file write operation.
 * @file:       Pointer to the file structure.
 * @buff:       Pointer to the user buffer.
 * @count:      The number of bytes to be written.
 * @ppos:       Pointer to the offset value
 * Return:      Transferred size.
 */
static ssize_t dmapubuf_device_file_write(struct file *file,
					  const char __user *buff,
					  size_t count, loff_t *ppos)
{
	struct dmapubuf_device_data *this = file->private_data;
	int result = 0;
	size_t xfer_size;
	dma_addr_t phys_addr;
	void *virt_addr;

	if (mutex_lock_interruptible(&this->sem))
		return -ERESTARTSYS;

	if (*ppos >= this->size) {
		result = 0;
		goto return_unlock;
	}

	phys_addr = this->phys_addr + *ppos;
	virt_addr = this->virt_addr + *ppos;
	xfer_size = (*ppos + count >= this->size) ? this->size - *ppos : count;

	if ((file->f_flags & O_SYNC) | (this->sync_mode & SYNC_ALWAYS))
		dma_sync_single_for_cpu(this->dma_dev, phys_addr, xfer_size,
					DMA_TO_DEVICE);

	if (copy_from_user(virt_addr, buff, xfer_size) != 0) {
		result = 0;
		goto return_unlock;
	}

	if ((file->f_flags & O_SYNC) | (this->sync_mode & SYNC_ALWAYS))
		dma_sync_single_for_device(this->dma_dev, phys_addr, xfer_size,
					   DMA_TO_DEVICE);

	*ppos += xfer_size;
	result = xfer_size;
return_unlock:
	mutex_unlock(&this->sem);
	return result;
}

/**
 * dmapubuf device file llseek operation.
 * @file:       Pointer to the file structure.
 * @offset:     File offset to seek.
 * @whence:     Type of seek.
 * Return:      The new position.
 */
static loff_t dmapubuf_device_file_llseek(struct file *file, loff_t offset,
					  int whence)
{
	struct dmapubuf_device_data *this = file->private_data;
	loff_t new_pos;

	switch (whence) {
	case 0: /* SEEK_SET */
		new_pos = offset;
		break;
	case 1: /* SEEK_CUR */
		new_pos = file->f_pos + offset;
		break;
	case 2: /* SEEK_END */
		new_pos = this->size  + offset;
		break;
	default:
		return -EINVAL;
	}
	if (new_pos < 0)
		return -EINVAL;
	if (new_pos > this->size)
		return -EINVAL;
	file->f_pos = new_pos;
	return new_pos;
}

static const struct file_operations dmapubuf_device_file_ops = {
	.owner = THIS_MODULE,
	.open = dmapubuf_device_file_open,
	.release = dmapubuf_device_file_release,
	.mmap = dmapubuf_device_file_mmap,
	.read = dmapubuf_device_file_read,
	.write = dmapubuf_device_file_write,
	.llseek = dmapubuf_device_file_llseek,
};

/**
 * DOC: dmapubuf Device Data Operations
 *
 * This section defines the operation of dmapubuf device data.
 *
 * * dmapubuf_device_ida      - dmapubuf Device Minor Number allocator variable.
 * * dmapubuf_device_number   - dmapubuf Device Major Number.
 * * dmapubuf_device_create() - Create dmapubuf device data.
 * * dmapubuf_device_setup()  - Setup the dmapubuf device data.
 * * dmapubuf_device_info()   - Print information the dmapubuf device data.
 * * dmapubuf_device_destroy()- Destroy the dmapubuf device data.
 */
static DEFINE_IDA(dmapubuf_device_ida);
static dev_t dmapubuf_device_number;

/**
 * Create dmapubuf device data.
 * @name:       device name   or NULL.
 * @parent:     parent device or NULL.
 * @minor:      minor_number  or -1 or -2.
 * Return:      Pointer to the dmapubuf device data or NULL.
 */
static struct dmapubuf_device_data *dmapubuf_device_create(const char *name,
							struct device *parent,
							int minor)
{
	struct dmapubuf_device_data *this = NULL;
	unsigned int done = 0;
	int retval;
	const unsigned int DONE_ALLOC_MINOR = (1 << 0);
	const unsigned int DONE_CHRDEV_ADD = (1 << 1);
	const unsigned int DONE_DEVICE_CREATE = (1 << 3);
	const unsigned int DONE_SET_DMA_DEV = (1 << 4);

	/* allocate device minor number */
	if ((minor >= 0) && (minor < DEVICE_MAX_NUM)) {
		if (ida_simple_get(&dmapubuf_device_ida, minor, minor + 1,
				   GFP_KERNEL) < 0) {
			pr_err("couldn't allocate minor number(=%d)\n", minor);
			goto failed;
		}
	} else if (minor < 0) {
		minor = ida_simple_get(&dmapubuf_device_ida, 0, DEVICE_MAX_NUM,
				       GFP_KERNEL);
		if (minor < 0) {
			pr_err("couldn't allocate new minor number. return=%d\n",
			       minor);
			goto failed;
		}
	} else {
		pr_err("invalid minor number(=%d), valid range is 0 to %d\n",
		       minor, DEVICE_MAX_NUM - 1);
		goto failed;
	}
	done |= DONE_ALLOC_MINOR;

	/* create (dmapubuf_device_data*) */
	this = kzalloc(sizeof(*this), GFP_KERNEL);
	if (IS_ERR_OR_NULL(this)) {
		retval = PTR_ERR(this);
		this = NULL;
		pr_err("kzalloc() failed return=%d\n", retval);
		goto failed;
	}

	/* set device_number */
	this->device_number = MKDEV(MAJOR(dmapubuf_device_number), minor);

	/* register /sys/class/dmapubuf/<name> */
	if (name == NULL)
		this->sys_dev = device_create(dmapubuf_sys_class,
					      parent,
					      this->device_number,
					      (void *)this,
					      DEVICE_NAME_FORMAT,
					      MINOR(this->device_number));
	else
		this->sys_dev = device_create(dmapubuf_sys_class,
					      parent,
					      this->device_number,
					      (void *)this,
					      "%s", name);

	if (IS_ERR_OR_NULL(this->sys_dev)) {
		retval = PTR_ERR(this->sys_dev);
		this->sys_dev = NULL;
		pr_err("device_create() failed. return=%d\n", retval);
		goto failed;
	}
	done |= DONE_DEVICE_CREATE;

	/* add chrdev. */
	cdev_init(&this->cdev, &dmapubuf_device_file_ops);
	this->cdev.owner = THIS_MODULE;
	retval = cdev_add(&this->cdev, this->device_number, 1);
	if (retval) {
		pr_err("cdev_add() failed. return=%d\n", retval);
		goto failed;
	}
	done |= DONE_CHRDEV_ADD;

	/* set dma_dev */
	if (parent != NULL)
		this->dma_dev = get_device(parent);
	else
		this->dma_dev = get_device(this->sys_dev);
	done |= DONE_SET_DMA_DEV;

	/* initialize other variables. */
	this->size = 0;
	this->alloc_size = 0;
	this->sync_mode = SYNC_MODE_NONCACHED;
	this->sync_offset = 0;
	this->sync_size = 0;
	this->sync_direction = 0;
	this->sync_owner = 0;
	this->sync_for_cpu = 0;
	this->sync_for_device = 0;
	this->of_reserved_mem = 0;
#if DMAPUBUF_DEBUG == 1
	this->debug_vma = 0;
#endif
	mutex_init(&this->sem);

	return this;

failed:
	if (done & DONE_SET_DMA_DEV)
		put_device(this->dma_dev);
	if (done & DONE_CHRDEV_ADD)
		cdev_del(&this->cdev);
	if (done & DONE_DEVICE_CREATE)
		device_destroy(dmapubuf_sys_class, this->device_number);
	if (done & DONE_ALLOC_MINOR)
		ida_simple_remove(&dmapubuf_device_ida, minor);
	if (this != NULL)
		kfree(this);
	return NULL;
}

/**
 * Setup the dmapubuf device data.
 * @this:       Pointer to the dmapubuf device data.
 * Return:      Success(=0) or error status(<0).
 */
static int dmapubuf_device_setup(struct dmapubuf_device_data *this)
{
	if (!this)
		return -ENODEV;

	/* set this->dma_dev->dma_mask */
	if (*this->dma_dev->dma_mask != DMA_BIT_MASK(dma_mask_bit)) {
		if (dma_set_mask(this->dma_dev,
				 DMA_BIT_MASK(dma_mask_bit)) == 0) {
			dma_set_coherent_mask(this->dma_dev,
					      DMA_BIT_MASK(dma_mask_bit));
		} else {
			pr_warn("dma_set_mask(DMA_BIT_MASK(%d)) failed\n",
				dma_mask_bit);
			dma_set_mask(this->dma_dev, DMA_BIT_MASK(32));
			dma_set_coherent_mask(this->dma_dev, DMA_BIT_MASK(32));
		}
	}

	/* setup buffer size and allocation size */
	this->alloc_size = ((this->size + ((1 << PAGE_SHIFT) - 1))
			    >> PAGE_SHIFT)
			   << PAGE_SHIFT;

	/* dma buffer allocation */
	this->virt_addr = dma_alloc_coherent(this->dma_dev, this->alloc_size,
					     &this->phys_addr, GFP_KERNEL);
	if (IS_ERR_OR_NULL(this->virt_addr)) {
		int retval = PTR_ERR(this->virt_addr);

		pr_err("dma_alloc_coherent() failed. return(%d)\n", retval);
		this->virt_addr = NULL;
		return (retval == 0) ? -ENOMEM : retval;
	}
	return 0;
}

/**
 * Print information the dmapubuf device data structure.
 * @this:       Pointer to the dmapubuf device data structure.
 */
static void dmapubuf_device_info(struct dmapubuf_device_data *this)
{
	dev_info(this->sys_dev, "driver version = %s\n", DRIVER_VERSION);
	dev_info(this->sys_dev, "major number   = %d\n",
		 MAJOR(this->device_number));
	dev_info(this->sys_dev, "minor number   = %d\n",
		 MINOR(this->device_number));
	dev_info(this->sys_dev, "phys address   = %pad\n", &this->phys_addr);
	dev_info(this->sys_dev, "buffer size    = %zu\n", this->alloc_size);
	dev_info(this->sys_dev, "dma device     = %s\n",
		 dev_name(this->dma_dev));
	dev_info(this->sys_dev, "dma coherent   = %d\n",
		 dev_is_dma_coherent(this->dma_dev));
}

/**
 * Destroy the dmapubuf device data.
 * @this:       Pointer to the dmapubuf device data.
 * Return:      Success(=0) or error status(<0).
 *
 * Unregister the device after releasing the resources.
 */
static int dmapubuf_device_destroy(struct dmapubuf_device_data *this)
{
	if (!this)
		return -ENODEV;

	if (this->virt_addr != NULL) {
		dma_free_coherent(this->dma_dev, this->alloc_size,
				  this->virt_addr, this->phys_addr);
		this->virt_addr = NULL;
	}
	put_device(this->dma_dev);
	cdev_del(&this->cdev);
	device_destroy(dmapubuf_sys_class, this->device_number);
	ida_simple_remove(&dmapubuf_device_ida, MINOR(this->device_number));
	kfree(this);
	return 0;
}

/**
 * DOC: dmapubuf Platform Device.
 *
 * This section defines the dmapubuf platform device list.
 *
 * * struct dmapubuf_platform_device
 *	- dmapubuf platform device structure.
 * * dmapubuf_platform_device_list
 *	- list of dmapubuf platform device structure.
 * * dmapubuf_platform_device_sem
 *	- semaphore of dmapubuf platform device list.
 * * dmapubuf_platform_device_create(
 *	- Create dmapubuf platform device and add to list.
 * * dmapubuf_platform_device_remove()
 *	- Remove dmapubuf platform device and delete from list.
 * * dmapubuf_platform_device_remove_all()
 *	- Remove all dmapubuf platform devices and clear list.
 * * dmapubuf_platform_device_search()
 *	- Search dmapubuf platform device from list by name or number.
 * * dmapubuf_get_device_name_property()
 *	- Get "device-name" property from dmapubuf device.
 * * dmapubuf_get_size_property()
 *	- Get "buffer-size" property from dmapubuf device.
 * * dmapubuf_get_minor_number_property()
 *	- Get "minor-number" property from dmapubuf device.
 */


/* dmapubuf platform device structure. */
struct dmapubuf_platform_device {
	struct device *dev;
	struct list_head list;
};

static struct list_head dmapubuf_platform_device_list;
static struct mutex dmapubuf_platform_device_sem;

/**
 * Get "device-name" property from dmapubuf device.
 * @dev:        handle to the device structure.
 * @name:       address of device name.
 * Return:      Success(=0) or error status(<0).
 */
static int dmapubuf_get_device_name_property(struct device *dev,
					      const char **name)
{
	return device_property_read_string(dev, "device-name", name);
}

/**
 * Get "buffer-size"  property from dmapubuf device.
 * @dev:        handle to the device structure.
 * @value:      address of buffer size value.
 * Return:      Success(=0) or error status(<0).
 */
static int dmapubuf_get_size_property(struct device *dev, u32 *value)
{
	return device_property_read_u32(dev, "size", value);
}

/**
 * Get "minor-number" property from dmapubuf device.
 * @dev:        handle to the device structure.
 * @value:      address of minor number value.
 * Return:      Success(=0) or error status(<0).
 */

static int dmapubuf_get_minor_number_property(struct device *dev, u32 *value)
{
	return device_property_read_u32(dev, "minor-number", value);
}

/**
 * Search dmapubuf platform device from
 * list by name or number.
 * @name:       device name or NULL.
 * @id:         device id.
 * Return:      Pointer to the dmapubuf_platform_device or NULL.
 */
static struct dmapubuf_platform_device *dmapubuf_platform_device_search(
						const char *name, int id)
{
	struct dmapubuf_platform_device *plat;
	struct dmapubuf_platform_device *found_plat = NULL;

	mutex_lock(&dmapubuf_platform_device_sem);
	list_for_each_entry(plat, &dmapubuf_platform_device_list, list) {
		bool found_by_name = true;
		bool found_by_id   = true;

		if (name != NULL) {
			const char *device_name;

			found_by_name = false;
			if (dmapubuf_get_device_name_property(plat->dev,
							     &device_name) == 0)
				if (strcmp(name, device_name) == 0)
					found_by_name = true;
		}
		if (id >= 0) {
			u32 minor_number;

			found_by_id = false;
			if (dmapubuf_get_minor_number_property(plat->dev,
							&minor_number) == 0)
				if (id == minor_number)
					found_by_id = true;
		}
		if ((found_by_name == true) && (found_by_id == true))
			found_plat = plat;
	}
	mutex_unlock(&dmapubuf_platform_device_sem);
	return found_plat;
}

/**
 * Create dmapubuf platform device and add to list.
 * @name:       device name or NULL.
 * @id:         device id.
 * @size:       buffer size.
 * Return:      Success(=0) or error status(<0).
 */
static int dmapubuf_platform_device_create(const char *name, int id,
					  unsigned int size)
{
	struct platform_device *pdev;
	struct dmapubuf_platform_device *plat = NULL;
	int retval = 0;
	bool list_added = false;

	struct property_entry props_list[] = {
		PROPERTY_ENTRY_STRING("device-name", name),
		PROPERTY_ENTRY_U32("size", size),
		PROPERTY_ENTRY_U32("minor-number", id),
		{},
	};
	struct property_entry *props = ((name != NULL) ?
					&props_list[0] : &props_list[1]);

	if (size == 0)
		return -EINVAL;

	pdev = platform_device_alloc(DRIVER_NAME, id);
	if (IS_ERR_OR_NULL(pdev)) {
		retval = PTR_ERR(pdev);
		pdev = NULL;
		pr_err("platform_device_alloc(%s,%d) failed. return=%d\n",
		       DRIVER_NAME, id, retval);
		goto failed;
	}

	plat = kzalloc(sizeof(*plat), GFP_KERNEL);
	if (IS_ERR_OR_NULL(plat)) {
		retval = PTR_ERR(plat);
		plat = NULL;
		dev_err(&pdev->dev, "kzalloc() failed. return=%d\n", retval);
		goto failed;
	}

	retval = device_add_properties(&pdev->dev, props);
	if (retval != 0) {
		dev_err(&pdev->dev, "device_add_properties failed. return=%d\n",
			retval);
		goto failed;
	}

	plat->dev  = &pdev->dev;
	mutex_lock(&dmapubuf_platform_device_sem);
	list_add_tail(&plat->list, &dmapubuf_platform_device_list);
	list_added = true;
	mutex_unlock(&dmapubuf_platform_device_sem);

	retval = platform_device_add(pdev);
	if (retval) {
		dev_err(&pdev->dev, "platform_device_add failed. return=%d\n",
			retval);
		goto failed;
	}

	return retval;

failed:
	if (list_added == true) {
		mutex_lock(&dmapubuf_platform_device_sem);
		list_del(&plat->list);
		mutex_unlock(&dmapubuf_platform_device_sem);
	}
	if (pdev != NULL)
		platform_device_put(pdev);

	if (plat != NULL)
		kfree(plat);

	return retval;
}

/**
 * Remove dmapubuf platform device and delete from list.
 * @plat:       dmapubuf_platform_device*
 */
static void dmapubuf_platform_device_remove(
					struct dmapubuf_platform_device *plat)
{
	struct device *dev  = plat->dev;
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_del(pdev);
	platform_device_put(pdev);
	mutex_lock(&dmapubuf_platform_device_sem);
	list_del(&plat->list);
	mutex_unlock(&dmapubuf_platform_device_sem);
	kfree(plat);
}

/**
 * Remove all dmapubuf platform devices and clear list.
 */
static void dmapubuf_platform_device_remove_all(void)
{
	while (!list_empty(&dmapubuf_platform_device_list)) {
		struct dmapubuf_platform_device *plat =
			list_first_entry(&dmapubuf_platform_device_list,
					 typeof(*(plat)), list);
		dmapubuf_platform_device_remove(plat);
	}
}

/**
 * DOC: dmapubuf Static Devices.
 *
 * This section defines the dmapubuf device to be created with
 * arguments when loaded into ther kernel with insmod.
 *
 */
#define DEFINE_DMAPUBUF_STATIC_DEVICE_PARAM(__num)			\
	static int dmapubuf ## __num;					\
module_param(dmapubuf ## __num, int, 0644);				\
MODULE_PARM_DESC(dmapubuf ## __num, DRIVER_NAME #__num " buffer size")

#define CALL_DMAPUBUF_STATIC_DEVICE_CREATE(__num)			\
	do {								\
		if (dmapubuf ## __num != 0) {				\
			ida_simple_remove(&dmapubuf_device_ida, __num);	\
			dmapubuf_platform_device_create(NULL, __num,	\
							dmapubuf ## __num); \
		}							\
	} while (0)

#define CALL_DMAPUBUF_STATIC_DEVICE_RESERVE_MINOR_NUMBER(__num)		\
	do {								\
		if (dmapubuf ## __num != 0) {				\
			ida_simple_get(&dmapubuf_device_ida, __num, __num+1, \
				       GFP_KERNEL);			\
		}							\
	} while (0)

DEFINE_DMAPUBUF_STATIC_DEVICE_PARAM(0);
DEFINE_DMAPUBUF_STATIC_DEVICE_PARAM(1);
DEFINE_DMAPUBUF_STATIC_DEVICE_PARAM(2);
DEFINE_DMAPUBUF_STATIC_DEVICE_PARAM(3);
DEFINE_DMAPUBUF_STATIC_DEVICE_PARAM(4);
DEFINE_DMAPUBUF_STATIC_DEVICE_PARAM(5);
DEFINE_DMAPUBUF_STATIC_DEVICE_PARAM(6);
DEFINE_DMAPUBUF_STATIC_DEVICE_PARAM(7);

/* Reserve dmapubuf static device's minor-number. */
static void dmapubuf_static_device_reserve_minor_number_all(void)
{
	CALL_DMAPUBUF_STATIC_DEVICE_RESERVE_MINOR_NUMBER(0);
	CALL_DMAPUBUF_STATIC_DEVICE_RESERVE_MINOR_NUMBER(1);
	CALL_DMAPUBUF_STATIC_DEVICE_RESERVE_MINOR_NUMBER(2);
	CALL_DMAPUBUF_STATIC_DEVICE_RESERVE_MINOR_NUMBER(3);
	CALL_DMAPUBUF_STATIC_DEVICE_RESERVE_MINOR_NUMBER(4);
	CALL_DMAPUBUF_STATIC_DEVICE_RESERVE_MINOR_NUMBER(5);
	CALL_DMAPUBUF_STATIC_DEVICE_RESERVE_MINOR_NUMBER(6);
	CALL_DMAPUBUF_STATIC_DEVICE_RESERVE_MINOR_NUMBER(7);
}

/* Create dmapubuf static devices. */
static void dmapubuf_static_device_create_all(void)
{
	CALL_DMAPUBUF_STATIC_DEVICE_CREATE(0);
	CALL_DMAPUBUF_STATIC_DEVICE_CREATE(1);
	CALL_DMAPUBUF_STATIC_DEVICE_CREATE(2);
	CALL_DMAPUBUF_STATIC_DEVICE_CREATE(3);
	CALL_DMAPUBUF_STATIC_DEVICE_CREATE(4);
	CALL_DMAPUBUF_STATIC_DEVICE_CREATE(5);
	CALL_DMAPUBUF_STATIC_DEVICE_CREATE(6);
	CALL_DMAPUBUF_STATIC_DEVICE_CREATE(7);
}

/**
 * Remove dmapubuf device driver.
 * @dev:        handle to the device structure.
 * @devdata     Pointer to the dmapubuf device data structure.
 * Return:      Success(=0) or error status(<0).
 */
static int dmapubuf_device_remove(struct device *dev,
				  struct dmapubuf_device_data *devdata)
{
	int retval = 0;

	if (devdata != NULL) {
		bool of_reserved_mem = devdata->of_reserved_mem;

		retval = dmapubuf_device_destroy(devdata);
		dev_set_drvdata(dev, NULL);
		if (of_reserved_mem)
			of_reserved_mem_device_release(dev);

	} else {
		retval = -ENODEV;
	}
	return retval;
}

/**
 * Probe call for the device.
 * @dev:        handle to the device structure.
 * Return:      Success(=0) or error status(<0).
 *
 * It does all the memory allocation and registration for the device.
 */
static int dmapubuf_device_probe(struct device *dev)
{
	int retval = 0;
	unsigned int u32_value = 0;
	unsigned int size = 0;
	int minor_number = -1;
	struct dmapubuf_device_data *device_data = NULL;
	const char *device_name = NULL;

	/* size property */
	if ((dmapubuf_get_size_property(dev, &u32_value)) == 0) {
		size = u32_value;
	} else if ((of_property_read_u32(dev->of_node, "size",
					 &u32_value)) == 0) {
		size = u32_value;
	} else {
		dev_err(dev, "invalid property size\n");
		retval = -ENODEV;
		goto failed;
	}

	/* minor-number property */
	if (dmapubuf_get_minor_number_property(dev, &u32_value) == 0)
		minor_number = u32_value;
	else if (of_property_read_u32(dev->of_node, "minor-number",
				      &u32_value) == 0)
		minor_number = u32_value;
	else
		minor_number = -1;

	/* device-name property */
	if (dmapubuf_get_device_name_property(dev, &device_name) != 0)
		device_name = of_get_property(dev->of_node, "device-name",
					      NULL);
	if (IS_ERR_OR_NULL(device_name)) {
		if (minor_number < 0)
			device_name = dev_name(dev);
		else
			device_name = NULL;
	}

	device_data = dmapubuf_device_create(device_name, dev, minor_number);
	if (IS_ERR_OR_NULL(device_data)) {
		retval = PTR_ERR(device_data);
		dev_err(dev, "driver create failed. return=%d.\n", retval);
		device_data = NULL;
		retval = (retval == 0) ? -EINVAL : retval;
		goto failed;
	}
	dev_set_drvdata(dev, device_data);

	device_data->size = size;

	if (dev->of_node != NULL) {
		retval = of_reserved_mem_device_init(dev);
		if (retval == 0) {
			device_data->of_reserved_mem = 1;
		} else if (retval != -ENODEV) {
			dev_err(dev, "of_reserved_mem_device_init failed. return=%d\n",
				retval);
			goto failed;
		}
	}

	/*
	 * of_dma_configure()
	 * - set pdev->dev->dma_mask
	 * - set pdev->dev->coherent_dma_mask
	 * - call of_dma_is_coherent()
	 * - call arch_setup_dma_ops()
	 */
	/* If "memory-region" property is spsecified, of_dma_configure()
	 * will not be executed. Because in that case, it is already executed
	 * in of_reserved_mem_device_init().
	 */
	if (device_data->of_reserved_mem == 0) {
		retval = of_dma_configure(dev, dev->of_node, true);
		if (retval != 0) {
			dev_err(dev, "of_dma_configure failed. return=%d\n",
				retval);
			goto failed;
		}
	}

	/* sync-mode property */
	if (of_property_read_u32(dev->of_node, "sync-mode", &u32_value) == 0) {
		if ((u32_value < SYNC_MODE_MIN) ||
		    (u32_value > SYNC_MODE_MAX)) {
			dev_err(dev, "invalid sync-mode property value=%d\n",
				u32_value);
			goto failed;
		}
		device_data->sync_mode &= ~SYNC_MODE_MASK;
		device_data->sync_mode |= (int)u32_value;
	}

	/* sync-always property */
	if (of_property_read_bool(dev->of_node, "sync-always"))
		device_data->sync_mode |= SYNC_ALWAYS;

	/* sync-direction property */
	if (of_property_read_u32(dev->of_node, "sync-direction",
				 &u32_value) == 0) {
		if (u32_value > 2) {
			dev_err(dev, "invalid sync-direction property value=%d\n",
				u32_value);
			goto failed;
		}
		device_data->sync_direction = (int)u32_value;
	}

	/* sync-offset property */
	if (of_property_read_u32(dev->of_node, "sync-offset",
				 &u32_value) == 0) {
		if (u32_value >= device_data->size) {
			dev_err(dev, "invalid sync-offset property value=%d\n",
				u32_value);
			goto failed;
		}
		device_data->sync_offset = (int)u32_value;
	}

	/* sync-size property */
	if (of_property_read_u32(dev->of_node, "sync-size", &u32_value) == 0) {
		if (device_data->sync_offset + u32_value > device_data->size) {
			dev_err(dev, "invalid sync-size property value=%d\n",
				u32_value);
			goto failed;
		}
		device_data->sync_size = (size_t)u32_value;
	} else {
		device_data->sync_size = device_data->size;
	}

	retval = dmapubuf_device_setup(device_data);
	if (retval) {
		dev_err(dev, "driver setup failed. return=%d\n", retval);
		goto failed;
	}

	if (info_enable)
		dmapubuf_device_info(device_data);

	return 0;

failed:
	dmapubuf_device_remove(dev, device_data);

	return retval;
}

/**
 * Probe call for the device.
 * @pdev:       Handle to the platform device structure.
 * Return:      Success(=0) or error status(<0).
 *
 * It does all the memory allocation and registration for the device.
 */
static int dmapubuf_platform_driver_probe(struct platform_device *pdev)
{
	int retval = 0;

	dev_dbg(&pdev->dev, "driver probe start.\n");

	retval = dmapubuf_device_probe(&pdev->dev);

	if (info_enable)
		dev_info(&pdev->dev, "driver installed.\n");

	return retval;
}
/**
 * Remove call for the device.
 * @pdev:       Handle to the platform device structure.
 * Return:      Success(=0) or error status(<0).
 *
 * Unregister the device after releasing the resources.
 */
static int dmapubuf_platform_driver_remove(struct platform_device *pdev)
{
	struct dmapubuf_device_data *this = dev_get_drvdata(&pdev->dev);
	int retval = 0;

	dev_dbg(&pdev->dev, "driver remove start.\n");

	retval = dmapubuf_device_remove(&pdev->dev, this);

	if (info_enable)
		dev_info(&pdev->dev, "driver removed.\n");

	return retval;
}

static const struct of_device_id dmapubuf_of_match[] = {
	{ .compatible = "dma-pubuf", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dmapubuf_of_match);

static struct platform_driver dmapubuf_platform_driver = {
	.probe  = dmapubuf_platform_driver_probe,
	.remove = dmapubuf_platform_driver_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
		.of_match_table = dmapubuf_of_match,
	},
};

/**
 * DOC: dmapubuf manager device
 *
 * * enum dmapubuf_manager_state       - dmapubuf manager state enumeration.
 * * struct dmapubuf_manager_data      - dmapubuf manager data structure.
 * * dmapubuf_manager_state_clear()    - dmapubuf manager state clear.
 * * dmapubuf_manager_buffer_overflow()- dmapubuf manager check buffer overflow.
 * * dmapubuf_manager_parse()          - dmapubuf manager parse buffer.
 * * dmapubuf_manager_file_open()      - dmapubuf manager file open operation.
 * * dmapubuf_manager_file_release()  - dmapubuf manager file release operation.
 * * dmapubuf_manager_file_write()     - dmapubuf manager file write operation.
 * * dmapubuf_manager_file_ops         - dmapubuf manager file operation table.
 * * dmapubuf_manager_device           - dmapubuf manager misc device structure.
 * *
 */

/* dmapubuf manager state enumeration. */
enum dmapubuf_manager_state {
	dmapubuf_manager_init_state,
	dmapubuf_manager_create_command,
	dmapubuf_manager_delete_command,
	dmapubuf_manager_parse_error,
};

/* dmapubuf manager data structure. */
struct dmapubuf_manager_data {
	const char *device_name;
	int minor_number;
	unsigned int size;
	enum dmapubuf_manager_state state;
	unsigned int buffer_offset;
	char buffer[DMAPUBUF_MGR_BUFFER_SIZE];
};

/**
 * dmapubuf manager check buffer overflow.
 * @this:       Pointer to the dmapubuf manager data structure.
 */
static bool dmapubuf_manager_buffer_overflow(struct dmapubuf_manager_data *this)
{
	if (this == NULL)
		return true;
	else
		return (this->buffer_offset >= DMAPUBUF_MGR_BUFFER_SIZE);
}

/**
 * dmapubuf manager state clear.
 * @this:       Pointer to the dmapubuf manager data structure.
 */
static void dmapubuf_manager_state_clear(struct dmapubuf_manager_data *this)
{
	this->device_name = NULL;
	this->minor_number = PLATFORM_DEVID_AUTO;
	this->size = 0;
	this->state = dmapubuf_manager_init_state;
	this->buffer_offset = 0;
}

/**
 * dmapubuf manager parse buffer.
 * @this:       Pointer to the dmapubuf manager data structure.
 * @buff:       Pointer to the user buffer.
 * @count:      The number of bytes to be written.
 * Return:      Size of copy from buff to this->buffer.
 */
static int dmapubuf_manager_parse(struct dmapubuf_manager_data *this,
				  const char __user *buff, size_t count)
{
	bool copy_done = false;
	size_t copy_size;
	int parse_count;
	u64 value;
	char *parse_buffer;
	char *ptr;

	if (this->buffer_offset + count > DMAPUBUF_MGR_BUFFER_SIZE)
		copy_size = DMAPUBUF_MGR_BUFFER_SIZE - this->buffer_offset;
	else
		copy_size = count;

	if (copy_from_user(&(this->buffer[this->buffer_offset]),
			   buff, copy_size) != 0)
		return -EFAULT;

	parse_count = 0;
	while (parse_count < copy_size) {
		char *ptr = &(this->buffer[this->buffer_offset+parse_count]);

		parse_count++;
		if ((*ptr == '\n') || (*ptr == '\0') || (*ptr == ';')) {
			*ptr = '\0';
			copy_done = true;
			break;
		}
	}
	this->buffer_offset += parse_count;

	if (copy_done == true) {
		parse_buffer = this->buffer;
		ptr = strsep(&parse_buffer, " ");

		if (ptr == NULL) {
			this->state = dmapubuf_manager_parse_error;
			goto failed;
		} else if (strncmp(ptr, "create", strlen("create")) == 0) {
			this->state = dmapubuf_manager_create_command;
		} else if (strncmp(ptr, "delete", strlen("delete")) == 0) {
			this->state = dmapubuf_manager_delete_command;
		} else {
			this->state = dmapubuf_manager_parse_error;
			goto failed;
		}
		ptr = strsep(&parse_buffer, " ");
		if (ptr == NULL) {
			this->state = dmapubuf_manager_parse_error;
			goto failed;
		} else {
			this->device_name = ptr;
		}
		if (this->state == dmapubuf_manager_create_command) {
			ptr = strsep(&parse_buffer, " ");
			if (ptr == NULL) {
				this->state = dmapubuf_manager_parse_error;
				goto failed;
			} else {
				if (kstrtoull(ptr, 0, &value) != 0) {
					this->state =
						dmapubuf_manager_parse_error;
					goto failed;
				} else {
					this->size = value;
				}
			}
		}
	}
failed:
	return parse_count;
}

/**
 * dmapubuf manager file open operation.
 * @inode:      Pointer to the inode structure of this device.
 * @file:       to the file structure.
 * Return:      Success(=0) or error status(<0).
 */
static int dmapubuf_manager_file_open(struct inode *inode, struct file *file)
{
	struct dmapubuf_manager_data *this;
	int status = 0;

	this = kzalloc(sizeof(*this), GFP_KERNEL);
	if (IS_ERR_OR_NULL(this)) {
		status = PTR_ERR(this);
	} else {
		dmapubuf_manager_state_clear(this);
		file->private_data = this;
	}
	return status;
}

/**
 * dmapubuf manager file release operation.
 * @inode:      Pointer to the inode structure of this device.
 * @file:       Pointer to the file structure.
 * Return:      Success(=0) or error status(<0).
 */
static int dmapubuf_manager_file_release(struct inode *inode, struct file *file)
{
	struct dmapubuf_manager_data *this = file->private_data;

	if (this != NULL)
		kfree(this);
	return 0;
}

/**
 * dmapubuf manager file write operation.
 * @file:       Pointer to the file structure.
 * @buff:       Pointer to the user buffer.
 * @count:      The number of bytes to be written.
 * @ppos:       Pointer to the offset value
 * Return:      Transferred size.
 */
static ssize_t dmapubuf_manager_file_write(struct file *file,
					   const char __user *buff,
					   size_t count, loff_t *ppos)
{
	struct dmapubuf_manager_data *this = file->private_data;
	struct dmapubuf_platform_device *plat;
	int parse_size;
	int result = 0;
	size_t xfer_size = 0;

	if (this == NULL)
		return -EINVAL;

	if (dmapubuf_manager_buffer_overflow(this))
		return -ENOSPC;

	while (xfer_size < count) {
		parse_size = dmapubuf_manager_parse(this, buff + xfer_size,
						    count - xfer_size);
		if (parse_size < 0) {
			result = parse_size;
			goto failed;
		}
		switch (this->state) {
		case dmapubuf_manager_create_command:
			pr_err("%s : create %s %d\n", DMAPUBUF_MGR_NAME,
			       this->device_name, this->size);
			result = dmapubuf_platform_device_create(
							this->device_name,
							this->minor_number,
							this->size);
			if (result == 0) {
				dmapubuf_manager_state_clear(this);
			} else {
				pr_err("%s : create error: %s result = %d\n",
				       DMAPUBUF_MGR_NAME, this->device_name,
				       result);
				dmapubuf_manager_state_clear(this);
				goto failed;
			}
			break;
		case dmapubuf_manager_delete_command:
			pr_err("%s : delete %s\n", DMAPUBUF_MGR_NAME,
			       this->device_name);
			plat = dmapubuf_platform_device_search(
							this->device_name,
							this->minor_number);
			if (plat != NULL) {
				dmapubuf_platform_device_remove(plat);
				dmapubuf_manager_state_clear(this);
			} else {
				pr_err("%s : delete error: %s not found\n",
				       DMAPUBUF_MGR_NAME, this->device_name);
				dmapubuf_manager_state_clear(this);
				result = -EINVAL;
				goto failed;
			}
			break;
		case dmapubuf_manager_parse_error:
			pr_err("%s : parse error: %s\n", DMAPUBUF_MGR_NAME,
			       this->buffer);
			dmapubuf_manager_state_clear(this);
			result = -EINVAL;
			goto failed;
		default:
			break;
		}
		xfer_size += parse_size;
	}
	*ppos += xfer_size;
	result = xfer_size;
failed:
	return result;
}

/* dmapubuf manager file operation table. */
static const struct file_operations dmapubuf_manager_file_ops = {
	.owner   = THIS_MODULE,
	.open    = dmapubuf_manager_file_open,
	.release = dmapubuf_manager_file_release,
	.write   = dmapubuf_manager_file_write,
};

/* dmapubuf manager misc device structure. */
static struct miscdevice dmapubuf_manager_device = {
	.minor   = MISC_DYNAMIC_MINOR,
	.name    = DMAPUBUF_MGR_NAME,
	.fops    = &dmapubuf_manager_file_ops,
};

static void u_dma_buf_cleanup(void)
{
	misc_deregister(&dmapubuf_manager_device);
	dmapubuf_platform_device_remove_all();
	platform_driver_unregister(&dmapubuf_platform_driver);
	class_destroy(dmapubuf_sys_class);

	if (dmapubuf_device_number != 0)
		unregister_chrdev_region(dmapubuf_device_number, 0);
	ida_destroy(&dmapubuf_device_ida);
}

static int __init u_dma_buf_init(void)
{
	int retval = 0;

	ida_init(&dmapubuf_device_ida);
	INIT_LIST_HEAD(&dmapubuf_platform_device_list);
	mutex_init(&dmapubuf_platform_device_sem);

	retval = alloc_chrdev_region(&dmapubuf_device_number, 0, 0,
				     DRIVER_NAME);
	if (retval != 0) {
		pr_err("%s: couldn't allocate device major number, ret=%d\n",
		       DRIVER_NAME, retval);
		dmapubuf_device_number = 0;
		goto failed;
	}

	dmapubuf_sys_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR_OR_NULL(dmapubuf_sys_class)) {
		retval = PTR_ERR(dmapubuf_sys_class);
		dmapubuf_sys_class = NULL;
		pr_err("%s: couldn't create sys class, ret=%d\n", DRIVER_NAME,
		       retval);
		retval = (retval == 0) ? -ENOMEM : retval;
		goto failed;
	}

	dmapubuf_sys_class_set_attributes();
	dmapubuf_static_device_reserve_minor_number_all();

	retval = platform_driver_register(&dmapubuf_platform_driver);
	if (retval) {
		pr_err("%s: couldn't register platform driver, ret=%d\n",
		       DRIVER_NAME, retval);
		goto failed;
	}

	dmapubuf_static_device_create_all();

	retval = misc_register(&dmapubuf_manager_device);
	if (retval) {
		pr_err("%s: couldn't register dmapubuf-mgr, ret=%d\n",
		       DRIVER_NAME, retval);
		goto failed;
	}

	return 0;

failed:
	u_dma_buf_cleanup();
	return retval;
}

static void __exit u_dma_buf_exit(void)
{
	u_dma_buf_cleanup();
}

module_init(u_dma_buf_init);
module_exit(u_dma_buf_exit);

MODULE_DESCRIPTION("User space mappable DMA buffer device driver");
MODULE_AUTHOR("ikwzm");
MODULE_LICENSE("Dual BSD/GPL");
