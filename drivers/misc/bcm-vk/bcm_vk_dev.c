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

/* Location of memory base addresses of interest in BAR1 */
/* Load Boot1 to start of ITCM */
#define BAR1_CODEPUSH_BASE_BOOT1	0x100000
/* Load Boot2 to start of DDR0 */
#define BAR1_CODEPUSH_BASE_BOOT2	0x300000
/* Allow minimum 1s for Load Image timeout responses */
#define LOAD_IMAGE_TIMEOUT_MS		1000
/* Allow extended time for maximum Load Image timeout responses */
#define LOAD_IMAGE_EXT_TIMEOUT_MS	30000

#define VK_MSIX_IRQ_MAX			3

#define BCM_VK_DMA_BITS			64

#define BCM_VK_MIN_RESET_TIME_SEC	2

#define BCM_VK_BUS_SYMLINK_NAME		"pci"

/* defines for voltage rail conversion */
#define BCM_VK_VOLT_RAIL_MASK		0xFFFF
#define BCM_VK_3P3_VOLT_REG_SHIFT	16

/* defines for power and temp threshold, all fields have same width */
#define BCM_VK_PWR_AND_THRE_FIELD_MASK	0xFF
#define BCM_VK_LOW_TEMP_THRE_SHIFT	0
#define BCM_VK_HIGH_TEMP_THRE_SHIFT	8
#define BCM_VK_PWR_STATE_SHIFT		16

/* defines for mem err, all fields have same width */
#define BCM_VK_MEM_ERR_FIELD_MASK	0xFF
#define BCM_VK_INT_MEM_ERR_SHIFT	0
#define BCM_VK_EXT_MEM_ERR_SHIFT	8

/* a macro to get an individual field with mask and shift */
#define BCM_VK_EXTRACT_FIELD(_field, _reg, _mask, _shift) \
	(_field = (((_reg) >> (_shift)) & (_mask)))

/*
 * check if PCIe interface is down on read.  Use it when it is
 * certain that _val should never be all ones.
 */
#define BCM_VK_INTF_IS_DOWN(_val)          ((_val) == 0xFFFFFFFF)
#define BCM_VK_BITS_NOT_SET(_val, _bitmask) \
	(((_val) & (_bitmask)) != (_bitmask))

/*
 * deinit time for the zephyr os after receiving doorbell,
 * 2 seconds should be enough
 */
#define BCM_VK_ZEPHYR_DEINIT_TIME_MS    (2 * MSEC_PER_SEC)

/* structure that is used to faciliate displaying of register content */
struct bcm_vk_sysfs_reg_entry {
	const uint32_t mask;
	const uint32_t exp_val;
	const char *str;
};

struct bcm_vk_sysfs_reg_list {
	const uint64_t offset;
	struct bcm_vk_sysfs_reg_entry const *tab;
	const uint32_t size;
	const char *hdr;
};

static int bcm_vk_sysfs_dump_reg(uint32_t reg_val,
				 struct bcm_vk_sysfs_reg_entry const *entry_tab,
				 const uint32_t table_size, char *buf)
{
	uint32_t i, masked_val;
	struct bcm_vk_sysfs_reg_entry const *p_entry;
	char *p_buf = buf;
	int ret;

	for (i = 0; i < table_size; i++) {
		p_entry = &entry_tab[i];
		masked_val = p_entry->mask & reg_val;
		if (masked_val == p_entry->exp_val) {
			ret = sprintf(p_buf, "  [0x%08x]    : %s\n",
				      masked_val, p_entry->str);
			if (ret < 0)
				return ret;

			p_buf += ret;
		}
	}

	return (p_buf - buf);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
#define KERNEL_PREAD_FLAG_PART	0x0001 /* Allow reading part of file */
static int request_firmware_into_buf(const struct firmware **firmware_p,
				     const char *name, struct device *device,
				     void *buf, size_t size,
				     size_t offset, unsigned int pread_flags)
{
	int ret;

	if (offset != 0)
		return -EINVAL;

	ret = request_firmware(firmware_p, name, device);
	if (ret)
		return ret;

	if ((*firmware_p)->size > size) {
		release_firmware(*firmware_p);
		ret = -EFBIG;
	} else {
		memcpy(buf, (*firmware_p)->data, (*firmware_p)->size);
	}

	return ret;
}
#endif

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
	const struct firmware *fw;
	void *bufp;
	size_t max_buf;
	struct vk_image image;
	int ret;
	uint64_t offset_codepush;
	u32 codepush;
	u32 ram_open;

	if (copy_from_user(&image, arg, sizeof(image))) {
		ret = -EACCES;
		goto err_out;
	}

	/*
	 * First, do a read for the ram_open and do a check. If interface goes
	 * down, bail out early.
	 */
	ram_open = vkread32(vk, BAR_0, BAR_FB_OPEN);
	dev_dbg(dev, "image type: 0x%x name: %s, ram_open = 0x%x\n",
		image.type, image.filename, ram_open);

	if (BCM_VK_INTF_IS_DOWN(ram_open)) {
		ret = -EFAULT;
		dev_err(dev, "Download Fails, PCIe interface down!");
		goto err_out;
	}

	if (image.type == VK_IMAGE_TYPE_BOOT1) {
		codepush = CODEPUSH_FASTBOOT + CODEPUSH_BOOT1_ENTRY;
		offset_codepush = BAR_CODEPUSH_SBL;

		/* Write a 1 to request SRAM open bit */
		vkwrite32(vk, CODEPUSH_FASTBOOT, BAR_0, offset_codepush);

		/* Wait for VK to respond */
		ret = bcm_vk_wait(vk, BAR_0, BAR_FB_OPEN, SRAM_OPEN, SRAM_OPEN,
				  LOAD_IMAGE_TIMEOUT_MS);
		if (ret < 0) {
			dev_err(dev, "boot1 timeout\n");
			goto err_out;
		}

		bufp = vk->bar[BAR_1] + BAR1_CODEPUSH_BASE_BOOT1;
		max_buf = SZ_256K;
	} else if (image.type == VK_IMAGE_TYPE_BOOT2) {
		codepush = CODEPUSH_BOOT2_ENTRY;
		offset_codepush = BAR_CODEPUSH_SBI;

		/* Wait for VK to respond */
		ret = bcm_vk_wait(vk, BAR_0, BAR_FB_OPEN, DDR_OPEN, DDR_OPEN,
				  LOAD_IMAGE_TIMEOUT_MS);
		if (ret < 0) {
			dev_err(dev, "boot2 timeout\n");
			goto err_out;
		}

		bufp = vk->bar[BAR_2];
		max_buf = SZ_64M;
	} else {
		dev_err(dev, "Error invalid image type 0x%x\n", image.type);
		ret = -EINVAL;
		goto err_out;
	}

	ret = request_firmware_into_buf(&fw, image.filename, dev,
					bufp, max_buf, 0,
					KERNEL_PREAD_FLAG_PART);
	if (ret) {
		dev_err(dev, "Error %d requesting firmware file: %s\n",
			ret, image.filename);
		goto err_out;
	}
	dev_dbg(dev, "size=0x%zx\n", fw->size);

	dev_dbg(dev, "Signaling 0x%x to 0x%llx\n", codepush, offset_codepush);
	vkwrite32(vk, codepush, BAR_0, offset_codepush);

	if (image.type == VK_IMAGE_TYPE_BOOT2) {
		/* To send more data to VK than max_buf allowed at a time */
		do {
			/* Wait for VK to move data from BAR space */
			ret = bcm_vk_wait(vk, BAR_0, BAR_FB_OPEN,
					  FW_LOADER_ACK_IN_PROGRESS,
					  FW_LOADER_ACK_IN_PROGRESS,
					  LOAD_IMAGE_EXT_TIMEOUT_MS);
			if (ret < 0)
				dev_dbg(dev, "boot2 timeout - transfer in progress\n");

			/* Wait for VK to acknowledge if it received all data */
			ret = bcm_vk_wait(vk, BAR_0, BAR_FB_OPEN,
					  FW_LOADER_ACK_RCVD_ALL_DATA,
					  FW_LOADER_ACK_RCVD_ALL_DATA,
					  LOAD_IMAGE_EXT_TIMEOUT_MS);
			if (ret < 0)
				dev_dbg(dev, "boot2 timeout - received all data\n");
			else
				break; /* VK received all data, break out */

			/* Wait for VK to request to send more data */
			ret = bcm_vk_wait(vk, BAR_0, BAR_FB_OPEN,
					  FW_LOADER_ACK_SEND_MORE_DATA,
					  FW_LOADER_ACK_SEND_MORE_DATA,
					  LOAD_IMAGE_EXT_TIMEOUT_MS);
			if (ret < 0) {
				dev_err(dev, "boot2 timeout - data send\n");
				break;
			}

			/* Wait for VK to open BAR space to copy new data */
			ret = bcm_vk_wait(vk, BAR_0, BAR_FB_OPEN,
					  DDR_OPEN, DDR_OPEN,
					  LOAD_IMAGE_EXT_TIMEOUT_MS);
			if (ret == 0) {
				ret = request_firmware_into_buf(
							&fw,
							image.filename,
							dev, bufp,
							max_buf,
							fw->size,
							KERNEL_PREAD_FLAG_PART);
				if (ret) {
					dev_err(dev, "Error %d requesting firmware file: %s offset: 0x%zx\n",
						ret, image.filename,
						fw->size);
					goto err_firmware_out;
				}
				dev_dbg(dev, "size=0x%zx\n", fw->size);
				dev_dbg(dev, "Signaling 0x%x to 0x%llx\n",
					codepush, offset_codepush);
				vkwrite32(vk, codepush, BAR_0, offset_codepush);
			}
		} while (1);

		/* Initialize Message Q if we are loading boot2 */
		/* wait for fw status bits to indicate Zephyr app ready */
		ret = bcm_vk_wait(vk, BAR_0, BAR_FW_STATUS,
				  FW_STATUS_ZEPHYR_READY,
				  FW_STATUS_ZEPHYR_READY,
				  LOAD_IMAGE_TIMEOUT_MS);
		if (ret < 0) {
			dev_err(dev, "Boot2 not ready - timeout\n");
			goto err_firmware_out;
		}

		/* sync queues when zephyr is up */
		if (bcm_vk_sync_msgq(vk)) {
			dev_err(dev, "Boot2 Error reading comm msg Q info\n");
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

static int bcm_vk_reset_successful(struct bcm_vk *vk)
{
	struct device *dev = &vk->pdev->dev;
	u32 fw_status, reset_reason;
	int ret = -EAGAIN;

	/*
	 * Reset could be triggered when the card in several state:
	 *   i)   in bootROM
	 *   ii)  after boot1
	 *   iii) boot2 running
	 *
	 * i) & ii) - no status bits will be updated.  If vkboot1
	 * runs automatically after reset, it  will update the reason
	 * to be unknown reason
	 * iii) - reboot reason match + deinit done.
	 */
	fw_status = vkread32(vk, BAR_0, BAR_FW_STATUS);
	/* immediate exit if interface goes down */
	if (BCM_VK_INTF_IS_DOWN(fw_status)) {
		dev_err(dev, "PCIe Intf Down!\n");
		goto bcm_vk_reset_exit;
	}

	/* initial check on reset reason */
	reset_reason = (fw_status & FW_STATUS_ZEPHYR_RESET_REASON_MASK);
	if ((reset_reason == FW_STATUS_ZEPHYR_RESET_MBOX_DB)
	     || (reset_reason == FW_STATUS_ZEPHYR_RESET_UNKNOWN))
		ret = 0;

	/*
	 * if some of the deinit bits are set, but done
	 * bit is not, this is a failure if triggered while boot2 is running
	 */
	if ((fw_status & FW_STATUS_ZEPHYR_DEINIT_TRIGGERED)
	    && !(fw_status & FW_STATUS_ZEPHYR_RESET_DONE))
		ret = -EAGAIN;

bcm_vk_reset_exit:
	dev_dbg(dev, "FW status = 0x%x ret %d\n", fw_status, ret);

	return ret;
}

static long bcm_vk_reset(struct bcm_vk *vk, struct vk_reset *arg)
{
	struct device *dev = &vk->pdev->dev;
	struct vk_reset reset;
	int ret = 0;
	int i;

	if (copy_from_user(&reset, arg, sizeof(struct vk_reset))) {
		ret = -EACCES;
		goto err_out;
	}
	dev_info(dev, "Issue Reset 0x%x, 0x%x\n", reset.arg1, reset.arg2);
	if (reset.arg2 < BCM_VK_MIN_RESET_TIME_SEC)
		reset.arg2 = BCM_VK_MIN_RESET_TIME_SEC;

	/*
	 * The following is the sequence of reset:
	 * - send card level graceful shut down
	 * - wait enough time for VK to handle its business, stopping DMA etc
	 * - kill host apps
	 * - Trigger interrupt with DB
	 */
	bcm_vk_send_shutdown_msg(vk, VK_SHUTDOWN_GRACEFUL, 0);

	spin_lock(&vk->ctx_lock);
	if (!vk->reset_ppid) {
		vk->reset_ppid = current;
	} else {
		dev_err(dev, "Reset already launched by process pid %d\n",
			task_pid_nr(vk->reset_ppid));
		ret = -EACCES;
	}
	spin_unlock(&vk->ctx_lock);
	if (ret)
		goto err_out;

	/* sleep time as specified by user in seconds, which is arg2 */
	msleep(reset.arg2 * MSEC_PER_SEC);

	spin_lock(&vk->ctx_lock);
	for (i = 0; i < VK_PID_HT_SZ; i++) {

		struct bcm_vk_ctx *p_ctx;

		list_for_each_entry(p_ctx,
				    &vk->pid_ht[i].fd_head,
				    list_node) {

			if (p_ctx->p_pid != vk->reset_ppid) {
				dev_dbg(dev, "Send kill signal to pid %d\n",
					task_pid_nr(p_ctx->p_pid));
				kill_pid(task_pid(p_ctx->p_pid), SIGKILL, 1);
			}
		}
	}
	spin_unlock(&vk->ctx_lock);
	if (ret)
		goto err_out;

	bcm_vk_trigger_reset(vk);

	/*
	 * Wait enough time for zephyr to deinit + populate the reset
	 * reason.
	 */
	msleep(BCM_VK_ZEPHYR_DEINIT_TIME_MS);

	ret = bcm_vk_reset_successful(vk);

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

static ssize_t temperature_show(struct device *dev,
				struct device_attribute *devattr, char *buf)
{
	unsigned int temperature;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t fw_status;

	/* if ZEPHYR is not running, no one will update the value */
	fw_status = vkread32(vk, BAR_0, BAR_FW_STATUS);
	if (BCM_VK_INTF_IS_DOWN(fw_status))
		return sprintf(buf, "PCIe Intf Down!\n");
	else if (BCM_VK_BITS_NOT_SET(fw_status, FW_STATUS_ZEPHYR_READY))
		return sprintf(buf, "Temperature: n/a (fw not running)\n");

#define _TEMP_FMT "Temperature : %u Celsius\n"
	temperature = vkread32(vk, BAR_0, BAR_CARD_TEMPERATURE);
	dev_dbg(dev, _TEMP_FMT, temperature);
	return sprintf(buf, _TEMP_FMT, temperature);
}

static ssize_t voltage_show(struct device *dev,
			    struct device_attribute *devattr, char *buf)
{
	unsigned int voltage;
	unsigned int volt_1p8, volt_3p3;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t fw_status;

	/* if ZEPHYR is not running, no one will update the value */
	fw_status = vkread32(vk, BAR_0, BAR_FW_STATUS);
	if (BCM_VK_INTF_IS_DOWN(fw_status))
		return sprintf(buf, "PCIe Intf Down!\n");
	else if (BCM_VK_BITS_NOT_SET(fw_status, FW_STATUS_ZEPHYR_READY))
		return sprintf(buf, "Voltage: n/a (fw not running)\n");

#define _VOLTAGE_FMT "[1.8v] : %u mV\n[3.3v] : %u mV\n"
	voltage = vkread32(vk, BAR_0, BAR_CARD_VOLTAGE);
	volt_1p8 = voltage & BCM_VK_VOLT_RAIL_MASK;
	volt_3p3 = (voltage >> BCM_VK_3P3_VOLT_REG_SHIFT)
		    & BCM_VK_VOLT_RAIL_MASK;
	dev_dbg(dev, _VOLTAGE_FMT, volt_1p8, volt_3p3);
	return sprintf(buf, _VOLTAGE_FMT, volt_1p8, volt_3p3);
}

static ssize_t firmware_version_show(struct device *dev,
				     struct device_attribute *devattr,
				     char *buf)
{
	int count = 0;
	unsigned long offset = BAR_FIRMWARE_TAG;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t fw_status;
	uint32_t chip_id;
	uint32_t loop_count = 0;

	/* Print driver version first, which is always available */
	count  = sprintf(buf, "Driver  : %s %s, srcversion %s\n",
			 DRV_MODULE_NAME, THIS_MODULE->version,
			 THIS_MODULE->srcversion);

	/* Check if ZEPHYR_PRE_KERNEL1_INIT_DONE */
	fw_status = vkread32(vk, BAR_0, BAR_FW_STATUS);
	if (BCM_VK_INTF_IS_DOWN(fw_status))
		return (count + sprintf(&buf[count], "PCIe Intf Down!\n"));
	else if (BCM_VK_BITS_NOT_SET(fw_status, FIRMWARE_STATUS_PRE_INIT_DONE))
		return (count + sprintf(&buf[count],
					"FW Version: n/a (fw not running)\n"));

	/* retrieve chip id for display */
	chip_id = vkread32(vk, BAR_0, BAR_CHIP_ID);
	count += sprintf(&buf[count], "Chip id : 0x%x\n", chip_id);

	/* TO_FIX: boot1 and others to be added here */

	count += sprintf(&buf[count], "zephyr  : ");

	do {
		buf[count] = vkread8(vk, BAR_1, offset);
		if (buf[count] == '\0')
			break;
		offset++;
		count++;
		loop_count++;
	} while (loop_count != BAR_FIRMWARE_TAG_SIZE);

	if (loop_count == BAR_FIRMWARE_TAG_SIZE)
		buf[--count] = '\0';

	buf[count++] = '\n'; /* append a CR */
	dev_dbg(dev, "FW version: %s", buf);
	return count;
}

static ssize_t firmware_status_show(struct device *dev,
				    struct device_attribute *devattr, char *buf)
{
	int ret, i;
	uint32_t reg_status;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	char *p_buf = buf;
	/*
	 * for firmware status register, they are bit definitions,
	 * so mask == exp_val
	 */
	static struct bcm_vk_sysfs_reg_entry const fw_status_reg_tab[] = {
		{FW_STATUS_RELOCATION_ENTRY,
		 FW_STATUS_RELOCATION_ENTRY,                "relo_entry"},
		{FW_STATUS_RELOCATION_EXIT,
		 FW_STATUS_RELOCATION_EXIT,                 "relo_exit"},
		{FW_STATUS_ZEPHYR_INIT_START,
		 FW_STATUS_ZEPHYR_INIT_START,               "init_st"},
		{FW_STATUS_ZEPHYR_ARCH_INIT_DONE,
		 FW_STATUS_ZEPHYR_ARCH_INIT_DONE,           "arch_inited"},
		{FW_STATUS_ZEPHYR_PRE_KERNEL1_INIT_DONE,
		 FW_STATUS_ZEPHYR_PRE_KERNEL1_INIT_DONE,    "pre_kern1_inited"},
		{FW_STATUS_ZEPHYR_PRE_KERNEL2_INIT_DONE,
		 FW_STATUS_ZEPHYR_PRE_KERNEL2_INIT_DONE,    "pre_kern2_inited"},
		{FW_STATUS_ZEPHYR_POST_KERNEL_INIT_DONE,
		 FW_STATUS_ZEPHYR_POST_KERNEL_INIT_DONE,    "kern_inited"},
		{FW_STATUS_ZEPHYR_INIT_DONE,
		 FW_STATUS_ZEPHYR_INIT_DONE,                "zephyr_inited"},
		{FW_STATUS_ZEPHYR_APP_INIT_START,
		 FW_STATUS_ZEPHYR_APP_INIT_START,           "app_init_st"},
		{FW_STATUS_ZEPHYR_APP_INIT_DONE,
		 FW_STATUS_ZEPHYR_APP_INIT_DONE,            "app_inited"},
	};
	/* for FB register */
	static struct bcm_vk_sysfs_reg_entry const fb_open_reg_tab[] = {
		{FW_LOADER_ACK_SEND_MORE_DATA,
		 FW_LOADER_ACK_SEND_MORE_DATA,                "bt1_needs_data"},
		{FW_LOADER_ACK_IN_PROGRESS,
		 FW_LOADER_ACK_IN_PROGRESS,                   "bt1_inprog"},
		{FW_LOADER_ACK_RCVD_ALL_DATA,
		 FW_LOADER_ACK_RCVD_ALL_DATA,                 "bt2_dload_done"},
		{0xFFE3FFFF, SRAM_OPEN | FB_STATE_WAIT_BOOT1, "wait_boot1"},
		{0xFFE3FFFF, DDR_OPEN  | FB_STATE_WAIT_BOOT2, "wait_boot2"},
		{0xFFF3FFFF, FW_LOADER_ACK_RCVD_ALL_DATA | FB_STATE_WAIT_BOOT2,
		 "boot2_running"},
	};
	/*
	 * shut down is lumped with fw-status register, but we use a different
	 * table to isolate it out.
	 */
	static struct bcm_vk_sysfs_reg_entry const fw_shutdown_reg_tab[] = {
		{FW_STATUS_ZEPHYR_APP_DEINIT_START,
		 FW_STATUS_ZEPHYR_APP_DEINIT_START,        "app_deinit_st"},
		{FW_STATUS_ZEPHYR_APP_DEINIT_DONE,
		 FW_STATUS_ZEPHYR_APP_DEINIT_DONE,         "app_deinited"},
		{FW_STATUS_ZEPHYR_DRV_DEINIT_START,
		 FW_STATUS_ZEPHYR_DRV_DEINIT_START,        "drv_deinit_st"},
		{FW_STATUS_ZEPHYR_DRV_DEINIT_DONE,
		 FW_STATUS_ZEPHYR_DRV_DEINIT_DONE,         "drv_deinited"},
		{FW_STATUS_ZEPHYR_RESET_DONE,
		 FW_STATUS_ZEPHYR_RESET_DONE,              "reset_done"},
		/* reboot reason */
		{FW_STATUS_ZEPHYR_RESET_REASON_MASK, 0,    "R-sys_pwrup"},
		{FW_STATUS_ZEPHYR_RESET_REASON_MASK,
		 FW_STATUS_ZEPHYR_RESET_MBOX_DB,	   "R-reset_doorbell"},
		{FW_STATUS_ZEPHYR_RESET_REASON_MASK,
		 FW_STATUS_ZEPHYR_RESET_M7_WDOG,	   "R-wdog"},
		{FW_STATUS_ZEPHYR_RESET_REASON_MASK,
		 FW_STATUS_ZEPHYR_RESET_TEMP,	           "R-overheat"},
		{FW_STATUS_ZEPHYR_RESET_REASON_MASK,
		 FW_STATUS_ZEPHYR_RESET_PCI_FLR,           "R-pci_flr"},
		{FW_STATUS_ZEPHYR_RESET_REASON_MASK,
		 FW_STATUS_ZEPHYR_RESET_PCI_HOT,	   "R-pci_hot"},
		{FW_STATUS_ZEPHYR_RESET_REASON_MASK,
		 FW_STATUS_ZEPHYR_RESET_PCI_WARM,	   "R-pci_warm" },
		{FW_STATUS_ZEPHYR_RESET_REASON_MASK,
		 FW_STATUS_ZEPHYR_RESET_PCI_COLD,          "R-pci_cold" },
		{FW_STATUS_ZEPHYR_RESET_REASON_MASK,
		 FW_STATUS_ZEPHYR_RESET_UNKNOWN,           "R-unknown" },

	};
	/* list of registers */
	static struct bcm_vk_sysfs_reg_list const fw_status_reg_list[] = {
		{BAR_FW_STATUS, fw_status_reg_tab,
		 ARRAY_SIZE(fw_status_reg_tab), "FW status"},
		{BAR_FB_OPEN, fb_open_reg_tab,
		 ARRAY_SIZE(fb_open_reg_tab), "FastBoot status"},
		{BAR_FW_STATUS, fw_shutdown_reg_tab,
		 ARRAY_SIZE(fw_shutdown_reg_tab), "Last Reset status"},
	};

	reg_status = vkread32(vk, BAR_0, BAR_FW_STATUS);
	if (BCM_VK_INTF_IS_DOWN(reg_status))
		return sprintf(buf, "PCIe Intf Down!\n");

	for (i = 0; i < ARRAY_SIZE(fw_status_reg_list); i++) {
		reg_status = vkread32(vk, BAR_0, fw_status_reg_list[i].offset);
		dev_dbg(dev, "%s: 0x%08x\n", fw_status_reg_list[i].hdr,
			reg_status);
		ret = sprintf(p_buf, "%s: 0x%08x\n",
			      fw_status_reg_list[i].hdr, reg_status);
		if (ret < 0)
			goto fw_status_show_fail;
		p_buf += ret;

		ret = bcm_vk_sysfs_dump_reg(reg_status,
					    fw_status_reg_list[i].tab,
					    fw_status_reg_list[i].size,
					    p_buf);
		if (ret < 0)
			goto fw_status_show_fail;
		p_buf += ret;
	}

	/* return total length written */
	return (p_buf - buf);

fw_status_show_fail:
	return ret;
}

static ssize_t bus_show(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);

#define _BUS_NUM_FMT "[pci_bus] %04d:%02d:%02d.%1d\n"
	dev_dbg(dev, _BUS_NUM_FMT,
		pci_domain_nr(pdev->bus), pdev->bus->number,
		PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));
	return sprintf(buf, _BUS_NUM_FMT,
		       pci_domain_nr(pdev->bus), pdev->bus->number,
		       PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));
}

static ssize_t card_state_show(struct device *dev,
			       struct device_attribute *devattr, char *buf)
{
	int ret;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t reg, fw_status;
	uint32_t low_temp_thre, high_temp_thre, pwr_state;
	uint32_t int_mem_err, ext_mem_err;
	char *p_buf = buf;
	static struct bcm_vk_sysfs_reg_entry const err_log_reg_tab[] = {
		{ERR_LOG_ALERT_ECC, ERR_LOG_ALERT_ECC,
		 "ecc"},
		{ERR_LOG_ALERT_SSIM_BUSY, ERR_LOG_ALERT_SSIM_BUSY,
		 "ssim_busy"},
		{ERR_LOG_ALERT_AFBC_BUSY, ERR_LOG_ALERT_AFBC_BUSY,
		 "afbc_busy"},
		{ERR_LOG_HIGH_TEMP_ERR, ERR_LOG_HIGH_TEMP_ERR,
		 "high_temp"},
		{ERR_LOG_MEM_ALLOC_FAIL, ERR_LOG_MEM_ALLOC_FAIL,
		 "malloc_fail warn"},
		{ERR_LOG_LOW_TEMP_WARN, ERR_LOG_LOW_TEMP_WARN,
		 "low_temp warn"}
	};
	static const char * const pwr_state_tab[] = {
		"Full", "Reduced", "Lowest"};
	char *pwr_state_str;

	/* if ZEPHYR is not running, no one will update the value */
	fw_status = vkread32(vk, BAR_0, BAR_FW_STATUS);
	if (BCM_VK_INTF_IS_DOWN(fw_status))
		return sprintf(buf, "PCIe Intf Down!\n");
	else if (BCM_VK_BITS_NOT_SET(fw_status, FW_STATUS_ZEPHYR_READY))
		return sprintf(buf, "card_state: n/a (fw not running)\n");

	/* First, get power state and the threshold */
	reg = vkread32(vk, BAR_0, BAR_CARD_PWR_AND_THRE);
	BCM_VK_EXTRACT_FIELD(low_temp_thre, reg,
			     BCM_VK_PWR_AND_THRE_FIELD_MASK,
			     BCM_VK_LOW_TEMP_THRE_SHIFT);
	BCM_VK_EXTRACT_FIELD(high_temp_thre, reg,
			     BCM_VK_PWR_AND_THRE_FIELD_MASK,
			     BCM_VK_HIGH_TEMP_THRE_SHIFT);
	BCM_VK_EXTRACT_FIELD(pwr_state, reg,
			     BCM_VK_PWR_AND_THRE_FIELD_MASK,
			     BCM_VK_PWR_STATE_SHIFT);

#define _PWR_AND_THRE_FMT "Pwr&Thre: 0x%08x\n"       \
		"  [Pwr_state]     : %d (%s)\n"      \
		"  [Low_thre]      : %d Celsius\n"   \
		"  [High_thre]     : %d Celsius\n"

	pwr_state_str = (pwr_state < ARRAY_SIZE(pwr_state_tab)) ?
			 (char *) pwr_state_tab[pwr_state] : "n/a";
	ret = sprintf(buf, _PWR_AND_THRE_FMT, reg, pwr_state, pwr_state_str,
		      low_temp_thre, high_temp_thre);
	if (ret < 0)
		goto card_state_show_fail;
	p_buf += ret;
	dev_dbg(dev, _PWR_AND_THRE_FMT, reg, pwr_state, pwr_state_str,
		low_temp_thre, high_temp_thre);

	/* next, see if there is any alert, also display them */
	reg = vkread32(vk, BAR_0, BAR_CARD_ERR_LOG);
	ret = sprintf(p_buf, "Alerts: 0x%08x\n", reg);
	if (ret < 0)
		goto card_state_show_fail;
	p_buf += ret;

	dev_dbg(dev, "Alerts: 0x%08x\n", reg);
	ret = bcm_vk_sysfs_dump_reg(reg,
				    err_log_reg_tab,
				    ARRAY_SIZE(err_log_reg_tab),
				    p_buf);
	if (ret < 0)
		goto card_state_show_fail;
	p_buf += ret;

	/* display memory error */
	reg = vkread32(vk, BAR_0, BAR_CARD_ERR_MEM);
	BCM_VK_EXTRACT_FIELD(int_mem_err, reg,
			     BCM_VK_MEM_ERR_FIELD_MASK,
			     BCM_VK_INT_MEM_ERR_SHIFT);
	BCM_VK_EXTRACT_FIELD(ext_mem_err, reg,
			     BCM_VK_MEM_ERR_FIELD_MASK,
			     BCM_VK_EXT_MEM_ERR_SHIFT);

#define _MEM_ERR_FMT "MemErr: 0x%08x\n"    \
		"  [internal]      : %d\n" \
		"  [external]      : %d\n"
	ret = sprintf(p_buf, _MEM_ERR_FMT, reg, int_mem_err, ext_mem_err);
	if (ret < 0)
		goto card_state_show_fail;
	p_buf += ret;
	dev_dbg(dev, _MEM_ERR_FMT, reg, int_mem_err, ext_mem_err);

	return (p_buf - buf);

card_state_show_fail:
	return ret;
}

static DEVICE_ATTR_RO(temperature);
static DEVICE_ATTR_RO(voltage);
static DEVICE_ATTR_RO(firmware_status);
static DEVICE_ATTR_RO(firmware_version);
static DEVICE_ATTR_RO(bus);
static DEVICE_ATTR_RO(card_state);

static struct attribute *bcm_vk_attributes[] = {
	&dev_attr_temperature.attr,
	&dev_attr_voltage.attr,
	&dev_attr_firmware_status.attr,
	&dev_attr_firmware_version.attr,
	&dev_attr_bus.attr,
	&dev_attr_card_state.attr,
	NULL,
};

static const struct attribute_group bcm_vk_attribute_group = {
	.name = "vk-card-status",
	.attrs = bcm_vk_attributes,
};

static const struct file_operations bcm_vk_fops = {
	.owner = THIS_MODULE,
	.open = bcm_vk_open,
	.read = bcm_vk_read,
	.write = bcm_vk_write,
	.release = bcm_vk_release,
	.mmap = bcm_vk_mmap,
	.unlocked_ioctl = bcm_vk_ioctl,
};

static int bcm_vk_on_panic(struct notifier_block *nb,
			   unsigned long e, void *p)
{
	struct bcm_vk *vk = container_of(nb, struct bcm_vk, panic_nb);

	bcm_h2vk_doorbell(vk, VK_BAR0_RESET_DB_NUM, VK_BAR0_RESET_DB_HARD);

	return 0;
}

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

	/* allocate vk structure which is tied to kref for freeing */
	vk = kzalloc(sizeof(*vk), GFP_KERNEL);
	if (!vk)
		return -ENOMEM;

	kref_init(&vk->kref);
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

	vk->tdma_vaddr = dma_alloc_coherent(dev, PAGE_SIZE, &vk->tdma_addr,
					    GFP_KERNEL);

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

	vk->misc_devid = id;
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

	dev_info(dev, "create sysfs group for bcm-vk.%d\n", id);
	err = sysfs_create_group(&pdev->dev.kobj, &bcm_vk_attribute_group);
	if (err < 0) {
		dev_err(dev, "failed to create sysfs attr for bcm.vk.%d\n", id);
		goto err_kfree_name;
	}
	/* create symbolic link from misc device to bus directory */
	err = sysfs_create_link(&misc_device->this_device->kobj,
				&pdev->dev.kobj, BCM_VK_BUS_SYMLINK_NAME);
	if (err < 0) {
		dev_err(dev, "failed to create symlink for bcm.vk.%d\n", id);
		goto err_free_sysfs_group;
	}
	/* create symbolic link from bus to misc device also */
	err = sysfs_create_link(&pdev->dev.kobj,
				&misc_device->this_device->kobj,
				misc_device->name);
	if (err < 0) {
		dev_err(dev,
			"failed to create reverse symlink for bcm.vk.%d\n",
			id);
		goto err_free_sysfs_entry;
	}


	/* last, register for panic notifier */
	vk->panic_nb.notifier_call = bcm_vk_on_panic;
	atomic_notifier_chain_register(&panic_notifier_list,
				       &vk->panic_nb);

	dev_info(dev, "BCM-VK:%u created, 0x%p\n", id, vk);

	return 0;

err_free_sysfs_entry:
	sysfs_remove_link(&misc_device->this_device->kobj,
			  BCM_VK_BUS_SYMLINK_NAME);

err_free_sysfs_group:
	sysfs_remove_group(&pdev->dev.kobj, &bcm_vk_attribute_group);

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

void bcm_vk_release_data(struct kref *kref)
{
	struct bcm_vk *vk = container_of(kref, struct bcm_vk, kref);

	/* use raw print, as dev is gone */
	pr_info("BCM-VK:%d release data 0x%p\n", vk->misc_devid, vk);
	kfree(vk);
}

static void bcm_vk_remove(struct pci_dev *pdev)
{
	int i;
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	struct miscdevice *misc_device = &vk->miscdev;

	/* unregister panic notifier */
	atomic_notifier_chain_unregister(&panic_notifier_list,
				       &vk->panic_nb);

	/* remove the sysfs entry and symlinks associated */
	sysfs_remove_link(&pdev->dev.kobj, misc_device->name);
	sysfs_remove_link(&misc_device->this_device->kobj,
			  BCM_VK_BUS_SYMLINK_NAME);
	sysfs_remove_group(&pdev->dev.kobj, &bcm_vk_attribute_group);

	cancel_work_sync(&vk->vk2h_wq);
	bcm_vk_msg_remove(vk);

	if (vk->tdma_vaddr)
		dma_free_coherent(&pdev->dev, PAGE_SIZE, vk->tdma_vaddr,
				  vk->tdma_addr);

	/* remove if name is set which means misc dev registered */
	if (misc_device->name) {
		misc_deregister(&vk->miscdev);
		kfree(misc_device->name);
		ida_simple_remove(&bcm_vk_ida, vk->misc_devid);
	}
	for (i = 0; i < vk->num_irqs; i++)
		devm_free_irq(&pdev->dev, pci_irq_vector(pdev, i), vk);

	pci_disable_msix(pdev);
	pci_disable_msi(pdev);

	for (i = 0; i < MAX_BAR; i++) {
		if (vk->bar[i])
			pci_iounmap(pdev, vk->bar[i]);
	}

	dev_info(&pdev->dev, "BCM-VK:%d released\n", vk->misc_devid);

	pci_release_regions(pdev);
	pci_disable_device(pdev);

	kref_put(&vk->kref, bcm_vk_release_data);
}

static void bcm_vk_shutdown(struct pci_dev *pdev)
{
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	bcm_h2vk_doorbell(vk, VK_BAR0_RESET_DB_NUM, VK_BAR0_RESET_DB_HARD);
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
	.shutdown = bcm_vk_shutdown,
};
module_pci_driver(pci_driver);

MODULE_DESCRIPTION("Broadcom Valkyrie Host Driver");
MODULE_AUTHOR("Scott Branden <scott.branden@broadcom.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
