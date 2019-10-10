// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018-2019 Broadcom.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <uapi/linux/misc/bcm_vk.h>

#include "bcm_vk.h"

#define DRV_MODULE_NAME		"bcm-vk"

#define PCI_DEVICE_ID_VALKYRIE	0x5E87
#define PCI_DEVICE_ID_VIPER	0x5E88

static DEFINE_IDA(bcm_vk_ida);

struct _load_image_tab {
		const uint32_t image_type;
		const char *image_name;
};

enum soc_idx {
	VALKYRIE = 0,
	VIPER
};

#define NUM_BOOT_STAGES 2
const struct _load_image_tab image_tab[][NUM_BOOT_STAGES] = {
	[VALKYRIE] = {
		{VK_IMAGE_TYPE_BOOT1, VK_BOOT1_DEF_VALKYRIE_FILENAME},
		{VK_IMAGE_TYPE_BOOT2, VK_BOOT2_DEF_VALKYRIE_FILENAME}
	},
	[VIPER] = {
		{VK_IMAGE_TYPE_BOOT1, VK_BOOT1_DEF_VIPER_FILENAME},
		{VK_IMAGE_TYPE_BOOT2, VK_BOOT2_DEF_VIPER_FILENAME}
	}
};

/* Location of memory base addresses of interest in BAR1 */
/* Load Boot1 to start of ITCM */
#define BAR1_CODEPUSH_BASE_BOOT1	0x100000
/* Allow minimum 1s for Load Image timeout responses */
#define LOAD_IMAGE_TIMEOUT_MS		1000

#define VK_MSIX_IRQ_MAX			3

#define BCM_VK_DMA_BITS			64

#define BCM_VK_MIN_RESET_TIME_SEC	2

#define BCM_VK_BOOT1_STARTUP_TIME_MS    (3 * MSEC_PER_SEC)

#define BCM_VK_BUS_SYMLINK_NAME		"pci"

/* defines for voltage rail conversion */
#define BCM_VK_VOLT_RAIL_MASK		0xFFFF
#define BCM_VK_3P3_VOLT_REG_SHIFT	16

/* defines for power and temp threshold, all fields have same width */
#define BCM_VK_PWR_AND_THRE_FIELD_MASK	0xFF
#define BCM_VK_LOW_TEMP_THRE_SHIFT	0
#define BCM_VK_HIGH_TEMP_THRE_SHIFT	8
#define BCM_VK_PWR_STATE_SHIFT		16

/* defines for all temperature sensor */
#define BCM_VK_TEMP_FIELD_MASK		0xFF
#define BCM_VK_CPU_TEMP_SHIFT		0
#define BCM_VK_DDR0_TEMP_SHIFT		8
#define BCM_VK_DDR1_TEMP_SHIFT		16

/* defines for mem err, all fields have same width */
#define BCM_VK_MEM_ERR_FIELD_MASK	0xFF
#define BCM_VK_ECC_MEM_ERR_SHIFT	0
#define BCM_VK_UECC_MEM_ERR_SHIFT	8

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
 * deinit time for the card os after receiving doorbell,
 * 2 seconds should be enough
 */
#define BCM_VK_DEINIT_TIME_MS    (2 * MSEC_PER_SEC)

/*
 * module parameters
 */
static bool auto_load = true;
module_param(auto_load, bool, 0444);
MODULE_PARM_DESC(auto_load,
		 "Load images automatically at PCIe probe time.\n");
static uint nr_scratch_pages = VK_BAR1_SCRATCH_DEF_NR_PAGES;
module_param(nr_scratch_pages, uint, 0444);
MODULE_PARM_DESC(nr_scratch_pages,
		 "Number of pre allocated DMAable coherent pages.\n");
static uint nr_ib_sgl_blk = BCM_VK_DEF_IB_SGL_BLK_LEN;
module_param(nr_ib_sgl_blk, uint, 0444);
MODULE_PARM_DESC(nr_ib_sgl_blk,
		 "Number of in-band msg blks for short SGL.\n");

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

/*
 * table structure for all shutdown related info in fw status register
 */
static struct bcm_vk_sysfs_reg_entry const fw_shutdown_reg_tab[] = {
	{FW_STATUS_APP_DEINIT_START, FW_STATUS_APP_DEINIT_START,
	 "app_deinit_st"},
	{FW_STATUS_APP_DEINIT_DONE, FW_STATUS_APP_DEINIT_DONE,
	 "app_deinited"},
	{FW_STATUS_DRV_DEINIT_START, FW_STATUS_DRV_DEINIT_START,
	 "drv_deinit_st"},
	{FW_STATUS_DRV_DEINIT_DONE, FW_STATUS_DRV_DEINIT_DONE,
	 "drv_deinited"},
	{FW_STATUS_RESET_DONE, FW_STATUS_RESET_DONE,
	 "reset_done"},
	 /* reboot reason */
	{FW_STATUS_RESET_REASON_MASK, FW_STATUS_RESET_SYS_PWRUP,
	 "sys_pwrup"},
	{FW_STATUS_RESET_REASON_MASK, FW_STATUS_RESET_MBOX_DB,
	 "reset_doorbell"},
	{FW_STATUS_RESET_REASON_MASK, FW_STATUS_RESET_M7_WDOG,
	 "wdog"},
	{FW_STATUS_RESET_REASON_MASK, FW_STATUS_RESET_TEMP,
	 "overheat"},
	{FW_STATUS_RESET_REASON_MASK, FW_STATUS_RESET_PCI_FLR,
	 "pci_flr"},
	{FW_STATUS_RESET_REASON_MASK, FW_STATUS_RESET_PCI_HOT,
	 "pci_hot"},
	{FW_STATUS_RESET_REASON_MASK, FW_STATUS_RESET_PCI_WARM,
	 "pci_warm" },
	{FW_STATUS_RESET_REASON_MASK, FW_STATUS_RESET_PCI_COLD,
	 "pci_cold" },
	{FW_STATUS_RESET_REASON_MASK, FW_STATUS_RESET_L1,
	 "L1_reset" },
	{FW_STATUS_RESET_REASON_MASK, FW_STATUS_RESET_L0,
	 "L0_reset" },
	{FW_STATUS_RESET_REASON_MASK, FW_STATUS_RESET_UNKNOWN,
	 "unknown" },
};
/* define for the start of the reboot reason */
#define FW_STAT_RB_REASON_START 5

static int bcm_vk_sysfs_dump_reg(uint32_t reg_val,
				 struct bcm_vk_sysfs_reg_entry const *entry_tab,
				 const uint32_t table_size, char *buf)
{
	uint32_t i, masked_val;
	struct bcm_vk_sysfs_reg_entry const *entry;
	char *p_buf = buf;
	int ret;

	for (i = 0; i < table_size; i++) {
		entry = &entry_tab[i];
		masked_val = entry->mask & reg_val;
		if (masked_val == entry->exp_val) {
			ret = sprintf(p_buf, "  [0x%08x]    : %s\n",
				      masked_val, entry->str);
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
	metadata.firmware_version = vk->card_info.version;
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

static void bcm_vk_get_card_info(struct bcm_vk *vk)
{
	struct device *dev = &vk->pdev->dev;
	uint32_t offset;
	int i;
	uint8_t *dst;
	struct bcm_vk_card_info *info = &vk->card_info;

	/* first read the offset from spare register */
	offset = vkread32(vk, BAR_0, BAR_CARD_STATIC_INFO);
	offset &= (pci_resource_len(vk->pdev, BAR_2 * 2) - 1);

	/* based on the offset, read info to internal card info structure */
	dst = (uint8_t *)info;
	for (i = 0; i < sizeof(*info); i++)
		*dst++ = vkread8(vk, BAR_2, offset++);

#define CARD_INFO_LOG_FMT "version   : %x\n" \
			  "os_tag    : %s\n" \
			  "cmpt_tag  : %s\n" \
			  "cpu_freq  : %d MHz\n" \
			  "cpu_scale : %d full, %d lowest\n" \
			  "ddr_freq  : %d MHz\n" \
			  "ddr_size  : %d MB\n" \
			  "video_freq: %d MHz\n"
	dev_dbg(dev, CARD_INFO_LOG_FMT, info->version, info->os_tag,
		info->cmpt_tag, info->cpu_freq_mhz, info->cpu_scale[0],
		info->cpu_scale[MAX_OPP - 1], info->ddr_freq_mhz,
		info->ddr_size_MB, info->video_core_freq_mhz);
}

static int bcm_vk_sync_card_info(struct bcm_vk *vk)
{
	uint32_t rdy_marker = vkread32(vk, BAR_1, VK_BAR1_MSGQ_DEF_RDY);

	/* check for marker, but allow diags mode to skip sync */
	if (!bcm_vk_msgq_marker_valid(vk))
		return (rdy_marker == VK_BAR1_DIAG_RDY_MARKER ? 0 : -EINVAL);

	/*
	 * Write down scratch addr which is used for DMA. For
	 * signed part, BAR1 is accessible only after boot2 has come
	 * up
	 */
	if (vk->tdma_addr) {
		vkwrite32(vk, vk->tdma_addr >> 32, BAR_1,
			  VK_BAR1_SCRATCH_OFF_LO);
		vkwrite32(vk, (uint32_t)vk->tdma_addr, BAR_1,
			  VK_BAR1_SCRATCH_OFF_HI);
		vkwrite32(vk, nr_scratch_pages * PAGE_SIZE, BAR_1,
			  VK_BAR1_SCRATCH_SZ_ADDR);
	}

	/* get static card info, only need to read once */
	bcm_vk_get_card_info(vk);

	return 0;
}

static int bcm_vk_load_image_by_type(struct bcm_vk *vk, u32 load_type,
				     const char *filename)
{
	struct device *dev = &vk->pdev->dev;
	const struct firmware *fw;
	void *bufp;
	size_t max_buf;
	int ret;
	uint64_t offset_codepush;
	u32 codepush;
	u32 value;

	if (load_type == VK_IMAGE_TYPE_BOOT1) {
		/*
		 * After POR, enable VK soft BOOTSRC so bootrom do not clear
		 * the pushed image (the TCM memories).
		 */
		value = vkread32(vk, BAR_0, BAR_BOOTSRC_SELECT);
		value |= BOOTSRC_SOFT_ENABLE;
		vkwrite32(vk, value, BAR_0, BAR_BOOTSRC_SELECT);

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
	} else if (load_type == VK_IMAGE_TYPE_BOOT2) {
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
		dev_err(dev, "Error invalid image type 0x%x\n", load_type);
		ret = -EINVAL;
		goto err_out;
	}

	ret = request_firmware_into_buf(&fw, filename, dev,
					bufp, max_buf, 0,
					KERNEL_PREAD_FLAG_PART);
	if (ret) {
		dev_err(dev, "Error %d requesting firmware file: %s\n",
			ret, filename);
		goto err_out;
	}
	dev_dbg(dev, "size=0x%zx\n", fw->size);

	dev_dbg(dev, "Signaling 0x%x to 0x%llx\n", codepush, offset_codepush);
	vkwrite32(vk, codepush, BAR_0, offset_codepush);

	if (load_type == VK_IMAGE_TYPE_BOOT1) {

		/* allow minimal time for boot1 to run */
		msleep(2 * MSEC_PER_SEC);

		/* wait until done */
		ret = bcm_vk_wait(vk, BAR_0, BAR_FB_OPEN,
				  FB_BOOT1_RUNNING,
				  FB_BOOT1_RUNNING,
				  BCM_VK_BOOT1_STARTUP_TIME_MS);
		if (ret) {
			dev_err(dev,
				"Timeout %ld ms waiting for boot1 to come up\n",
				BCM_VK_BOOT1_STARTUP_TIME_MS);
			goto err_firmware_out;
		}

	} else if (load_type == VK_IMAGE_TYPE_BOOT2) {
		/* To send more data to VK than max_buf allowed at a time */
		do {
			/* Wait for VK to move data from BAR space */
			ret = bcm_vk_wait(vk, BAR_0, BAR_FB_OPEN,
					  FW_LOADER_ACK_IN_PROGRESS,
					  FW_LOADER_ACK_IN_PROGRESS,
					  LOAD_IMAGE_TIMEOUT_MS);
			if (ret < 0)
				dev_dbg(dev, "boot2 timeout - transfer in progress\n");

			/* Wait for VK to request to send more data */
			ret = bcm_vk_wait(vk, BAR_0, BAR_FB_OPEN,
					  FW_LOADER_ACK_SEND_MORE_DATA,
					  FW_LOADER_ACK_SEND_MORE_DATA,
					  LOAD_IMAGE_TIMEOUT_MS);
			if (ret < 0) {
				/*
				 * Wait for VK to acknowledge if it received
				 * all data
				 */
				ret = bcm_vk_wait(vk, BAR_0, BAR_FB_OPEN,
						  FW_LOADER_ACK_RCVD_ALL_DATA,
						  FW_LOADER_ACK_RCVD_ALL_DATA,
						  LOAD_IMAGE_TIMEOUT_MS);
				if (ret < 0)
					dev_dbg(dev, "boot2 timeout - received all data\n");
				/* break either way: received all or not */
				break;
			}

			/* Wait for VK to open BAR space to copy new data */
			ret = bcm_vk_wait(vk, BAR_0, BAR_FB_OPEN,
					  DDR_OPEN, DDR_OPEN,
					  LOAD_IMAGE_TIMEOUT_MS);
			if (ret == 0) {
				ret = request_firmware_into_buf(
							&fw,
							filename,
							dev, bufp,
							max_buf,
							fw->size,
							KERNEL_PREAD_FLAG_PART);
				if (ret) {
					dev_err(dev, "Error %d requesting firmware file: %s offset: 0x%zx\n",
						ret, filename,
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
		/* wait for fw status bits to indicate app ready */
		ret = bcm_vk_wait(vk, BAR_0, BAR_FW_STATUS,
				  FW_STATUS_READY,
				  FW_STATUS_READY,
				  LOAD_IMAGE_TIMEOUT_MS);
		if (ret < 0) {
			dev_err(dev, "Boot2 not ready - timeout\n");
			goto err_firmware_out;
		}

		/* sync queues when card os is up */
		ret = bcm_vk_sync_msgq(vk);
		if (ret) {
			dev_err(dev, "Boot2 Error reading comm msg Q info\n");
			ret = -EIO;
			goto err_firmware_out;
		}

		/* sync & channel other info */
		ret = bcm_vk_sync_card_info(vk);
		if (ret) {
			dev_err(dev, "Syncing Card Info failure\n");
			goto err_firmware_out;
		}
	}

err_firmware_out:
	release_firmware(fw);

err_out:
	return ret;
}

static u32 bcm_vk_next_boot_image(struct bcm_vk *vk)
{
	uint32_t fb_open = vkread32(vk, BAR_0, BAR_FB_OPEN);
	u32 load_type = 0;  /* default for unknown */

	if (!BCM_VK_INTF_IS_DOWN(fb_open) && (fb_open & SRAM_OPEN))
		load_type = VK_IMAGE_TYPE_BOOT1;
	else if (fb_open == FB_BOOT1_RUNNING)
		load_type = VK_IMAGE_TYPE_BOOT2;

	/*
	 * TO_FIX: For now, like to know what value we get everytime
	 *         for debugging.
	 */
	dev_info(&vk->pdev->dev, "FB_OPEN value on deciding next image: 0x%x\n",
		 fb_open);

	return load_type;
}

int bcm_vk_auto_load_all_images(struct bcm_vk *vk)
{
	int i, id, ret = -1;
	struct device *dev = &vk->pdev->dev;
	struct pci_dev *pdev = to_pci_dev(dev);
	uint32_t curr_type;
	const char *curr_name;

	switch (pdev->device) {
	case PCI_DEVICE_ID_VALKYRIE:
		id = VALKYRIE;
		break;

	case PCI_DEVICE_ID_VIPER:
		id = VIPER;
		break;

	default:
		dev_err(dev, "no images for 0x%x\n", pdev->device);
		goto bcm_vk_auto_load_all_exit;
	}

	/* log a message to know the relative loading order */
	dev_info(dev, "Load All for device %d\n", vk->misc_devid);

	for (i = 0; i < NUM_BOOT_STAGES; i++) {
		curr_type = image_tab[id][i].image_type;
		if (bcm_vk_next_boot_image(vk) == curr_type) {
			curr_name = image_tab[id][i].image_name;
			ret = bcm_vk_load_image_by_type(vk, curr_type,
							curr_name);
			dev_info(dev, "Auto load %s, ret %d\n",
				 curr_name, ret);

			if (ret) {
				dev_err(dev, "Error loading default %s\n",
					curr_name);
				goto bcm_vk_auto_load_all_exit;
			}
		}
	}

bcm_vk_auto_load_all_exit:
	return ret;
}

static int bcm_vk_trigger_autoload(struct bcm_vk *vk)
{
	if (test_and_set_bit(BCM_VK_WQ_DWNLD_PEND, &vk->wq_offload) != 0)
		return -EPERM;

	set_bit(BCM_VK_WQ_DWNLD_AUTO, &vk->wq_offload);
	queue_work(vk->wq_thread, &vk->wq_work);

	return 0;
}

static long bcm_vk_load_image(struct bcm_vk *vk, struct vk_image *arg)
{
	int ret;
	struct device *dev = &vk->pdev->dev;
	struct vk_image image;
	u32 next_loadable;

	if (copy_from_user(&image, arg, sizeof(image))) {
		ret = -EACCES;
		goto bcm_vk_load_image_exit;
	}

	/* first check if fw is at the right state for the download */
	next_loadable = bcm_vk_next_boot_image(vk);
	if (next_loadable != image.type) {
		dev_err(dev, "Next expected image %u, Loading %u\n",
			next_loadable, image.type);
		ret = -EPERM;
		goto bcm_vk_load_image_exit;
	}

	/*
	 * if something is pending download already.  This could only happen
	 * for now when the driver is being loaded, or if someone has issued
	 * another download command in another shell.
	 */
	if (test_and_set_bit(BCM_VK_WQ_DWNLD_PEND, &vk->wq_offload) != 0) {
		dev_err(dev, "Download operation already pending.\n");
		return -EPERM;
	}

	ret = bcm_vk_load_image_by_type(vk, image.type, image.filename);
	clear_bit(BCM_VK_WQ_DWNLD_PEND, &vk->wq_offload);

bcm_vk_load_image_exit:
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
	reset_reason = (fw_status & FW_STATUS_RESET_REASON_MASK);
	if ((reset_reason == FW_STATUS_RESET_MBOX_DB)
	     || (reset_reason == FW_STATUS_RESET_UNKNOWN))
		ret = 0;

	/*
	 * if some of the deinit bits are set, but done
	 * bit is not, this is a failure if triggered while boot2 is running
	 */
	if ((fw_status & FW_STATUS_DEINIT_TRIGGERED)
	    && !(fw_status & FW_STATUS_RESET_DONE))
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

		struct bcm_vk_ctx *ctx;

		list_for_each_entry(ctx, &vk->pid_ht[i].head, node) {
			if (ctx->ppid != vk->reset_ppid) {
				dev_dbg(dev, "Send kill signal to pid %d\n",
					task_pid_nr(ctx->ppid));
				kill_pid(task_pid(ctx->ppid), SIGKILL, 1);
			}
		}
	}
	spin_unlock(&vk->ctx_lock);
	if (ret)
		goto err_out;

	bcm_vk_trigger_reset(vk);

	/*
	 * Wait enough time for card os to deinit + populate the reset
	 * reason.
	 */
	msleep(BCM_VK_DEINIT_TIME_MS);

	ret = bcm_vk_reset_successful(vk);

err_out:
	return ret;
}

static int bcm_vk_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct bcm_vk_ctx *ctx = file->private_data;
	struct bcm_vk *vk = container_of(ctx->miscdev, struct bcm_vk, miscdev);
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
	struct bcm_vk *vk = container_of(ctx->miscdev, struct bcm_vk, miscdev);
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

static int bcm_vk_sysfs_chk_fw_status(struct bcm_vk *vk, uint32_t mask,
				      char *buf, const char *err_log)
{
	uint32_t fw_status;
	int ret = 0;

	/* if card OS is not running, no one will update the value */
	fw_status = vkread32(vk, BAR_0, BAR_FW_STATUS);
	if (BCM_VK_INTF_IS_DOWN(fw_status))
		return sprintf(buf, "PCIe Intf Down!\n");
	else if (BCM_VK_BITS_NOT_SET(fw_status, mask))
		return sprintf(buf, err_log);

	return ret;
}

static int bcm_vk_sysfs_get_tag(struct bcm_vk *vk, enum pci_barno barno,
				uint32_t offset, char *buf, const char *fmt)
{
	uint32_t magic;

#define REL_MAGIC_TAG         0x68617368   /* this stands for "hash" */

	magic = vkread32(vk, barno, offset);
	return sprintf(buf, fmt, (magic == REL_MAGIC_TAG) ?
		       (char *)(vk->bar[barno] + offset + sizeof(magic)) : "");
}

static ssize_t temperature_sensor_show(struct device *dev,
				       struct device_attribute *devattr,
				       char *buf,
				       const char *tag,
				       uint offset)
{
	unsigned int temperature = 0; /* default if invalid */
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	temperature = vkread32(vk, BAR_0, BAR_CARD_TEMPERATURE);
	temperature = (temperature >> offset) & BCM_VK_TEMP_FIELD_MASK;

	dev_dbg(dev, "Temperature_%s : %u Celsius\n", tag, temperature);
	return sprintf(buf, "%d\n", temperature);
}

static ssize_t temperature_sensor_1_c_show(struct device *dev,
					   struct device_attribute *devattr,
					   char *buf)
{
	return temperature_sensor_show(dev, devattr, buf, "CPU",
				       BCM_VK_CPU_TEMP_SHIFT);
}

static ssize_t temperature_sensor_2_c_show(struct device *dev,
					   struct device_attribute *devattr,
					   char *buf)
{
	return temperature_sensor_show(dev, devattr, buf, "DDR0",
				       BCM_VK_DDR0_TEMP_SHIFT);
}

static ssize_t temperature_sensor_3_c_show(struct device *dev,
					   struct device_attribute *devattr,
					   char *buf)
{
	return temperature_sensor_show(dev, devattr, buf, "DDR1",
				       BCM_VK_DDR1_TEMP_SHIFT);
}

static ssize_t voltage_18_mv_show(struct device *dev,
				  struct device_attribute *devattr, char *buf)
{
	unsigned int voltage;
	unsigned int volt_1p8;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	voltage = vkread32(vk, BAR_0, BAR_CARD_VOLTAGE);
	volt_1p8 = voltage & BCM_VK_VOLT_RAIL_MASK;

	dev_dbg(dev, "[1.8v] : %u mV\n", volt_1p8);
	return sprintf(buf, "%d\n", volt_1p8);
}

static ssize_t voltage_33_mv_show(struct device *dev,
				  struct device_attribute *devattr, char *buf)
{
	unsigned int voltage;
	unsigned int volt_3p3 = 0;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	voltage = vkread32(vk, BAR_0, BAR_CARD_VOLTAGE);
	volt_3p3 = (voltage >> BCM_VK_3P3_VOLT_REG_SHIFT)
		    & BCM_VK_VOLT_RAIL_MASK;

	dev_dbg(dev, "[3.3v] : %u mV\n", volt_3p3);
	return sprintf(buf, "%d\n", volt_3p3);
}

static ssize_t chip_id_show(struct device *dev,
			    struct device_attribute *devattr, char *buf)
{
	uint32_t chip_id;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	chip_id = vkread32(vk, BAR_0, BAR_CHIP_ID);

	return sprintf(buf, "0x%x\n", chip_id);
}

static ssize_t firmware_status_reg_show(struct device *dev,
					struct device_attribute *devattr,
					char *buf)
{
	uint32_t fw_status;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	fw_status = vkread32(vk, BAR_0, BAR_FW_STATUS);

	return sprintf(buf, "0x%x\n", fw_status);
}

static ssize_t fastboot_reg_show(struct device *dev,
				 struct device_attribute *devattr,
				 char *buf)
{
	uint32_t fb_reg;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	fb_reg = vkread32(vk, BAR_0, BAR_FB_OPEN);

	return sprintf(buf, "0x%x\n", fb_reg);
}

static ssize_t pwr_state_show(struct device *dev,
			      struct device_attribute *devattr,
			      char *buf)
{
	uint32_t card_pwr_and_thre;
	uint32_t pwr_state;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	card_pwr_and_thre = vkread32(vk, BAR_0, BAR_CARD_PWR_AND_THRE);
	BCM_VK_EXTRACT_FIELD(pwr_state, card_pwr_and_thre,
			     BCM_VK_PWR_AND_THRE_FIELD_MASK,
			     BCM_VK_PWR_STATE_SHIFT);

	return sprintf(buf, "%u\n", pwr_state);
}

static ssize_t firmware_version_show(struct device *dev,
				     struct device_attribute *devattr,
				     char *buf)
{
	int count = 0;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t chip_id;
	int ret;

	/* Print driver version first, which is always available */
	count  = sprintf(buf, "Driver  : %s %s, srcversion %s\n",
			 DRV_MODULE_NAME, THIS_MODULE->version,
			 THIS_MODULE->srcversion);

	/* check for ucode and vk-boot1 versions */
	count += bcm_vk_sysfs_get_tag(vk, BAR_1, VK_BAR1_UCODE_VER_TAG,
				      &buf[count], "UCODE   : %s\n");
	count += bcm_vk_sysfs_get_tag(vk, BAR_1, VK_BAR1_BOOT1_VER_TAG,
				      &buf[count], "Boot1   : %s\n");

	/* Check if FIRMWARE_STATUS_PRE_INIT_DONE for rest of items */
	ret = bcm_vk_sysfs_chk_fw_status(vk, FIRMWARE_STATUS_PRE_INIT_DONE,
					 &buf[count],
					 "FW Version: n/a (fw not running)\n");
	if (ret)
		return (ret + count);

	/* retrieve chip id for display */
	chip_id = vkread32(vk, BAR_0, BAR_CHIP_ID);
	count += sprintf(&buf[count], "Chip id : 0x%x\n", chip_id);
	count += sprintf(&buf[count], "Card os : %s\n", vk->card_info.os_tag);
	return count;
}

static ssize_t rev_flash_rom_show(struct device *dev,
				  struct device_attribute *devattr,
				  char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return bcm_vk_sysfs_get_tag(vk, BAR_1, VK_BAR1_UCODE_VER_TAG,
				     buf, "%s\n");
}

static ssize_t rev_boot1_show(struct device *dev,
			      struct device_attribute *devattr,
			      char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return bcm_vk_sysfs_get_tag(vk, BAR_1, VK_BAR1_BOOT1_VER_TAG,
				     buf, "%s\n");
}

static ssize_t rev_boot2_show(struct device *dev,
			      struct device_attribute *devattr,
			      char *buf)
{
	int ret;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	/* Check if FIRMWARE_STATUS_PRE_INIT_DONE */
	ret = bcm_vk_sysfs_chk_fw_status(vk, FIRMWARE_STATUS_PRE_INIT_DONE,
					 buf, "n/a\n");
	if (ret)
		return ret;

	return sprintf(buf, "%s\n", vk->card_info.os_tag);
}

static ssize_t rev_driver_show(struct device *dev,
			       struct device_attribute *devattr,
			       char *buf)
{
	return sprintf(buf, "%s_%s-srcversion_%s\n",
		       DRV_MODULE_NAME, THIS_MODULE->version,
		       THIS_MODULE->srcversion);
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
		{FW_STATUS_RELOCATION_ENTRY, FW_STATUS_RELOCATION_ENTRY,
		 "relo_entry"},
		{FW_STATUS_RELOCATION_EXIT, FW_STATUS_RELOCATION_EXIT,
		 "relo_exit"},
		{FW_STATUS_INIT_START, FW_STATUS_INIT_START,
		 "init_st"},
		{FW_STATUS_ARCH_INIT_DONE, FW_STATUS_ARCH_INIT_DONE,
		 "arch_inited"},
		{FW_STATUS_PRE_KNL1_INIT_DONE, FW_STATUS_PRE_KNL1_INIT_DONE,
		 "pre_kern1_inited"},
		{FW_STATUS_PRE_KNL2_INIT_DONE, FW_STATUS_PRE_KNL2_INIT_DONE,
		  "pre_kern2_inited"},
		{FW_STATUS_POST_KNL_INIT_DONE, FW_STATUS_POST_KNL_INIT_DONE,
		  "kern_inited"},
		{FW_STATUS_INIT_DONE, FW_STATUS_INIT_DONE,
		 "card_os_inited"},
		{FW_STATUS_APP_INIT_START, FW_STATUS_APP_INIT_START,
		 "app_init_st"},
		{FW_STATUS_APP_INIT_DONE, FW_STATUS_APP_INIT_DONE,
		 "app_inited"},
	};
	/* for FB register */
	static struct bcm_vk_sysfs_reg_entry const fb_open_reg_tab[] = {
		{FW_LOADER_ACK_SEND_MORE_DATA, FW_LOADER_ACK_SEND_MORE_DATA,
		 "bt1_needs_data"},
		{FW_LOADER_ACK_IN_PROGRESS, FW_LOADER_ACK_IN_PROGRESS,
		 "bt1_inprog"},
		{FW_LOADER_ACK_RCVD_ALL_DATA, FW_LOADER_ACK_RCVD_ALL_DATA,
		 "bt2_dload_done"},
		{SRAM_OPEN, SRAM_OPEN,
		 "wait_boot1"},
		{FB_BOOT_STATE_MASK, FB_BOOT1_RUNNING,
		 "wait_boot2"},
		{FB_BOOT_STATE_MASK, FB_BOOT2_RUNNING,
		 "boot2_running"},
	};
	/* list of registers */
	static struct bcm_vk_sysfs_reg_list const fw_status_reg_list[] = {
		{BAR_FW_STATUS, fw_status_reg_tab,
		 ARRAY_SIZE(fw_status_reg_tab),
		 "FW status"},
		{BAR_FB_OPEN, fb_open_reg_tab,
		 ARRAY_SIZE(fb_open_reg_tab),
		 "FastBoot status"},
		{BAR_FW_STATUS, fw_shutdown_reg_tab,
		 ARRAY_SIZE(fw_shutdown_reg_tab),
		 "Last Reset status"},
	};

	reg_status = vkread32(vk, BAR_0, BAR_FW_STATUS);
	if (BCM_VK_INTF_IS_DOWN(reg_status))
		return sprintf(buf, "PCIe Intf Down!\n");

	for (i = 0; i < ARRAY_SIZE(fw_status_reg_list); i++) {
		reg_status = vkread32(vk, BAR_0, fw_status_reg_list[i].offset);

		dev_dbg(dev, "%s: 0x%08x\n",
			fw_status_reg_list[i].hdr, reg_status);

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

static ssize_t reset_reason_show(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	uint32_t reg, i;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	static struct bcm_vk_sysfs_reg_entry const *tab =
		&fw_shutdown_reg_tab[FW_STAT_RB_REASON_START];

	reg = vkread32(vk, BAR_0, BAR_FW_STATUS);
	if (BCM_VK_INTF_IS_DOWN(reg))
		return sprintf(buf, "PCIe Intf Down!\n");

	for (i = 0;
	     i < (ARRAY_SIZE(fw_shutdown_reg_tab) - FW_STAT_RB_REASON_START);
	     i++) {
		if ((tab[i].mask & reg) == tab[i].exp_val)
			return sprintf(buf, "%s\n", tab[i].str);
	}

	return sprintf(buf, "invalid\n");
}

static ssize_t os_state_show(struct device *dev,
			     struct device_attribute *devattr, char *buf)
{
	uint32_t reg, i;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	static struct bcm_vk_sysfs_reg_entry const tab[] = {
		{SRAM_OPEN, SRAM_OPEN,
		 "wait_boot1"},
		{FB_BOOT_STATE_MASK, FB_BOOT1_RUNNING,
		 "wait_boot2"},
		{FB_BOOT_STATE_MASK, FB_BOOT2_RUNNING,
		 "boot2_running"},
	};

	reg = vkread32(vk, BAR_0, BAR_FW_STATUS);
	if (BCM_VK_INTF_IS_DOWN(reg))
		return sprintf(buf, "PCIe Intf Down!\n");

	reg = vkread32(vk, BAR_0, BAR_FB_OPEN);
	for (i = 0; i < ARRAY_SIZE(tab); i++) {
		if ((tab[i].mask & reg) == tab[i].exp_val)
			return sprintf(buf, "%s\n", tab[i].str);
	}

	return sprintf(buf, "invalid\n");
}

static ssize_t bus_show(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);

#define _BUS_NUM_FMT "%04x:%02x:%02x.%1d\n"
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
	uint32_t reg;
	uint32_t low_temp_thre, high_temp_thre, pwr_state;
	uint32_t ecc_mem_err, uecc_mem_err;
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
		 "low_temp warn"},
		{ERR_LOG_ECC_WARN, ERR_LOG_ECC_WARN,
		 "ecc_correctable"},
	};
	static const char * const pwr_state_tab[] = {
		"Full", "Reduced", "Lowest"};
	char *pwr_state_str;

	/* if OS is not running, no one will update the value */
	ret = bcm_vk_sysfs_chk_fw_status(vk, FW_STATUS_READY, buf,
					 "card_state: n/a (fw not running)\n");
	if (ret)
		return ret;

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

	pwr_state_str = ((pwr_state - 1) < ARRAY_SIZE(pwr_state_tab)) ?
			 (char *) pwr_state_tab[pwr_state - 1] : "n/a";
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
	BCM_VK_EXTRACT_FIELD(ecc_mem_err, reg,
			     BCM_VK_MEM_ERR_FIELD_MASK,
			     BCM_VK_ECC_MEM_ERR_SHIFT);
	BCM_VK_EXTRACT_FIELD(uecc_mem_err, reg,
			     BCM_VK_MEM_ERR_FIELD_MASK,
			     BCM_VK_UECC_MEM_ERR_SHIFT);

#define _MEM_ERR_FMT "MemErr: 0x%08x\n"    \
		"  [ECC]       : %d\n" \
		"  [UECC]      : %d\n"
	ret = sprintf(p_buf, _MEM_ERR_FMT, reg, ecc_mem_err, uecc_mem_err);
	if (ret < 0)
		goto card_state_show_fail;
	p_buf += ret;
	dev_dbg(dev, _MEM_ERR_FMT, reg, ecc_mem_err, uecc_mem_err);

	return (p_buf - buf);

card_state_show_fail:
	return ret;
}

static ssize_t mem_ecc_show(struct device *dev,
			    struct device_attribute *devattr, char *buf)
{
	int ret;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t reg;
	uint32_t ecc_mem_err;

	/* if OS is not running, no one will update the value */
	ret = bcm_vk_sysfs_chk_fw_status(vk, FW_STATUS_READY, buf,
					 "0\n");
	if (ret)
		return ret;

	/* display memory error */
	reg = vkread32(vk, BAR_0, BAR_CARD_ERR_MEM);
	BCM_VK_EXTRACT_FIELD(ecc_mem_err, reg,
			     BCM_VK_MEM_ERR_FIELD_MASK,
			     BCM_VK_ECC_MEM_ERR_SHIFT);

	return sprintf(buf, "%d\n", ecc_mem_err);
}

static ssize_t mem_uecc_show(struct device *dev,
			     struct device_attribute *devattr, char *buf)
{
	int ret;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t reg;
	uint32_t uecc_mem_err;

	/* if OS is not running, no one will update the value */
	ret = bcm_vk_sysfs_chk_fw_status(vk, FW_STATUS_READY, buf,
					 "0\n");
	if (ret)
		return ret;

	/* display memory error */
	reg = vkread32(vk, BAR_0, BAR_CARD_ERR_MEM);
	BCM_VK_EXTRACT_FIELD(uecc_mem_err, reg,
			     BCM_VK_MEM_ERR_FIELD_MASK,
			     BCM_VK_UECC_MEM_ERR_SHIFT);

	return sprintf(buf, "%d\n", uecc_mem_err);
}

static ssize_t alert_ecc_show(struct device *dev,
			      struct device_attribute *devattr, char *buf)
{
	int ret;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t reg;

	/* if OS is not running, no one will update the value, just return 0 */
	ret = bcm_vk_sysfs_chk_fw_status(vk, FW_STATUS_READY, buf,
					 "0\n");
	if (ret)
		return ret;

	reg = vkread32(vk, BAR_0, BAR_CARD_ERR_LOG);
	return sprintf(buf, "%d\n", reg & ERR_LOG_ALERT_ECC ? 1 : 0);
}

static ssize_t alert_ssim_busy_show(struct device *dev,
				    struct device_attribute *devattr, char *buf)
{
	int ret;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t reg;

	/* if OS is not running, no one will update the value, just return 0 */
	ret = bcm_vk_sysfs_chk_fw_status(vk, FW_STATUS_READY, buf,
					 "0\n");
	if (ret)
		return ret;

	reg = vkread32(vk, BAR_0, BAR_CARD_ERR_LOG);
	return sprintf(buf, "%d\n", reg & ERR_LOG_ALERT_SSIM_BUSY ? 1 : 0);
}

static ssize_t alert_afbc_busy_show(struct device *dev,
				    struct device_attribute *devattr, char *buf)
{
	int ret;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t reg;

	/* if OS is not running, no one will update the value, just return 0 */
	ret = bcm_vk_sysfs_chk_fw_status(vk, FW_STATUS_READY, buf,
					 "0\n");
	if (ret)
		return ret;

	reg = vkread32(vk, BAR_0, BAR_CARD_ERR_LOG);
	return sprintf(buf, "%d\n", reg & ERR_LOG_ALERT_AFBC_BUSY ? 1 : 0);
}

static ssize_t alert_high_temp_show(struct device *dev,
				    struct device_attribute *devattr, char *buf)
{
	int ret;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t reg;

	/* if OS is not running, no one will update the value, just return 0 */
	ret = bcm_vk_sysfs_chk_fw_status(vk, FW_STATUS_READY, buf,
					 "0\n");
	if (ret)
		return ret;

	reg = vkread32(vk, BAR_0, BAR_CARD_ERR_LOG);
	return sprintf(buf, "%d\n", reg & ERR_LOG_HIGH_TEMP_ERR ? 1 : 0);
}

static ssize_t alert_malloc_fail_warn_show(struct device *dev,
					   struct device_attribute *devattr,
					   char *buf)
{
	int ret;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t reg;

	/* if OS is not running, no one will update the value, just return 0 */
	ret = bcm_vk_sysfs_chk_fw_status(vk, FW_STATUS_READY, buf,
					 "0\n");
	if (ret)
		return ret;

	reg = vkread32(vk, BAR_0, BAR_CARD_ERR_LOG);
	return sprintf(buf, "%d\n", reg & ERR_LOG_MEM_ALLOC_FAIL ? 1 : 0);
}

static ssize_t alert_low_temp_warn_show(struct device *dev,
					struct device_attribute *devattr,
					char *buf)
{
	int ret;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t reg;

	/* if OS is not running, no one will update the value, just return 0 */
	ret = bcm_vk_sysfs_chk_fw_status(vk, FW_STATUS_READY, buf,
					 "0\n");
	if (ret)
		return ret;

	reg = vkread32(vk, BAR_0, BAR_CARD_ERR_LOG);
	return sprintf(buf, "%d\n", reg & ERR_LOG_LOW_TEMP_WARN ? 1 : 0);
}

static ssize_t alert_ecc_warn_show(struct device *dev,
				   struct device_attribute *devattr,
				   char *buf)
{
	int ret;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t reg;

	/* if OS is not running, no one will update the value, just return 0 */
	ret = bcm_vk_sysfs_chk_fw_status(vk, FW_STATUS_READY, buf,
					 "0\n");
	if (ret)
		return ret;

	reg = vkread32(vk, BAR_0, BAR_CARD_ERR_LOG);
	return sprintf(buf, "%d\n", reg & ERR_LOG_ECC_WARN ? 1 : 0);
}

static ssize_t temp_threshold_lower_c_show(struct device *dev,
					   struct device_attribute *devattr,
					   char *buf)
{
	int ret;
	uint32_t low_temp_thre;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t reg;

	ret = bcm_vk_sysfs_chk_fw_status(vk, FW_STATUS_READY, buf,
					 "0\n");
	if (ret)
		return ret;

	reg = vkread32(vk, BAR_0, BAR_CARD_PWR_AND_THRE);
	BCM_VK_EXTRACT_FIELD(low_temp_thre, reg,
			     BCM_VK_PWR_AND_THRE_FIELD_MASK,
			     BCM_VK_LOW_TEMP_THRE_SHIFT);

	return sprintf(buf, "%d\n", low_temp_thre);
}

static ssize_t temp_threshold_upper_c_show(struct device *dev,
					   struct device_attribute *devattr,
					   char *buf)
{
	int ret;
	uint32_t high_temp_thre;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t reg;

	ret = bcm_vk_sysfs_chk_fw_status(vk, FW_STATUS_READY, buf,
					 "0\n");
	if (ret)
		return ret;

	reg = vkread32(vk, BAR_0, BAR_CARD_PWR_AND_THRE);
	BCM_VK_EXTRACT_FIELD(high_temp_thre, reg,
			     BCM_VK_PWR_AND_THRE_FIELD_MASK,
			     BCM_VK_HIGH_TEMP_THRE_SHIFT);

	return sprintf(buf, "%d\n", high_temp_thre);
}


static ssize_t freq_core_mhz_show(struct device *dev,
				  struct device_attribute *devattr,
				  char *buf)
{
	uint32_t card_pwr_and_thre;
	uint32_t pwr_state;
	uint32_t scale_f = 0;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	struct bcm_vk_card_info *info = &vk->card_info;

	card_pwr_and_thre = vkread32(vk, BAR_0, BAR_CARD_PWR_AND_THRE);
	BCM_VK_EXTRACT_FIELD(pwr_state, card_pwr_and_thre,
			     BCM_VK_PWR_AND_THRE_FIELD_MASK,
			     BCM_VK_PWR_STATE_SHIFT);

	if (pwr_state && (pwr_state <= MAX_OPP))
		scale_f = info->cpu_scale[pwr_state - 1];

	return sprintf(buf, "%d\n",
		       info->cpu_freq_mhz / (scale_f ? scale_f : 1));
}

static ssize_t freq_mem_mhz_show(struct device *dev,
				 struct device_attribute *devattr,
				 char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	struct bcm_vk_card_info *info = &vk->card_info;

	return sprintf(buf, "%d\n", info->ddr_freq_mhz);
}

static ssize_t mem_size_mb_show(struct device *dev,
				 struct device_attribute *devattr,
				 char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	struct bcm_vk_card_info *info = &vk->card_info;

	return sprintf(buf, "%d\n", info->ddr_size_MB);
}

static ssize_t sotp_common_show(struct device *dev,
				struct device_attribute *devattr,
				char *buf, uint32_t tag_offset)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return bcm_vk_sysfs_get_tag(vk, BAR_1, tag_offset, buf, "%s\n");
}

static ssize_t sotp_dauth_1_show(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	return sotp_common_show(dev, devattr, buf,
				VK_BAR1_DAUTH_STORE_ADDR(0));
}

static ssize_t sotp_dauth_1_valid_show(struct device *dev,
				       struct device_attribute *devattr,
				       char *buf)
{
	return sotp_common_show(dev, devattr, buf,
				VK_BAR1_DAUTH_VALID_ADDR(0));
}

static ssize_t sotp_dauth_2_show(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	return sotp_common_show(dev, devattr, buf,
				VK_BAR1_DAUTH_STORE_ADDR(1));
}

static ssize_t sotp_dauth_2_valid_show(struct device *dev,
				       struct device_attribute *devattr,
				       char *buf)
{
	return sotp_common_show(dev, devattr, buf,
				VK_BAR1_DAUTH_VALID_ADDR(1));
}

static ssize_t sotp_dauth_3_show(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	return sotp_common_show(dev, devattr, buf,
				VK_BAR1_DAUTH_STORE_ADDR(2));
}

static ssize_t sotp_dauth_3_valid_show(struct device *dev,
				       struct device_attribute *devattr,
				       char *buf)
{
	return sotp_common_show(dev, devattr, buf,
				VK_BAR1_DAUTH_VALID_ADDR(2));
}

static ssize_t sotp_dauth_4_show(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	return sotp_common_show(dev, devattr, buf,
				VK_BAR1_DAUTH_STORE_ADDR(3));
}

static ssize_t sotp_dauth_4_valid_show(struct device *dev,
				       struct device_attribute *devattr,
				       char *buf)
{
	return sotp_common_show(dev, devattr, buf,
				VK_BAR1_DAUTH_VALID_ADDR(3));
}

static ssize_t sotp_boot1_rev_id_show(struct device *dev,
				      struct device_attribute *devattr,
				      char *buf)
{
	return sotp_common_show(dev, devattr, buf,
				VK_BAR1_SOTP_REVID_ADDR(0));
}

static ssize_t sotp_boot2_rev_id_show(struct device *dev,
				      struct device_attribute *devattr,
				      char *buf)
{
	return sotp_common_show(dev, devattr, buf,
				VK_BAR1_SOTP_REVID_ADDR(1));
}

static DEVICE_ATTR_RO(firmware_status);
static DEVICE_ATTR_RO(reset_reason);
static DEVICE_ATTR_RO(os_state);
static DEVICE_ATTR_RO(firmware_version);
static DEVICE_ATTR_RO(rev_flash_rom);
static DEVICE_ATTR_RO(rev_boot1);
static DEVICE_ATTR_RO(rev_boot2);
static DEVICE_ATTR_RO(rev_driver);
static DEVICE_ATTR_RO(bus);
static DEVICE_ATTR_RO(card_state);
static DEVICE_ATTR_RO(mem_ecc);
static DEVICE_ATTR_RO(mem_uecc);
static DEVICE_ATTR_RO(alert_ecc);
static DEVICE_ATTR_RO(alert_ssim_busy);
static DEVICE_ATTR_RO(alert_afbc_busy);
static DEVICE_ATTR_RO(alert_high_temp);
static DEVICE_ATTR_RO(alert_malloc_fail_warn);
static DEVICE_ATTR_RO(alert_low_temp_warn);
static DEVICE_ATTR_RO(alert_ecc_warn);
static DEVICE_ATTR_RO(temp_threshold_lower_c);
static DEVICE_ATTR_RO(temp_threshold_upper_c);
static DEVICE_ATTR_RO(freq_core_mhz);
static DEVICE_ATTR_RO(freq_mem_mhz);
static DEVICE_ATTR_RO(mem_size_mb);
static DEVICE_ATTR_RO(sotp_dauth_1);
static DEVICE_ATTR_RO(sotp_dauth_1_valid);
static DEVICE_ATTR_RO(sotp_dauth_2);
static DEVICE_ATTR_RO(sotp_dauth_2_valid);
static DEVICE_ATTR_RO(sotp_dauth_3);
static DEVICE_ATTR_RO(sotp_dauth_3_valid);
static DEVICE_ATTR_RO(sotp_dauth_4);
static DEVICE_ATTR_RO(sotp_dauth_4_valid);
static DEVICE_ATTR_RO(sotp_boot1_rev_id);
static DEVICE_ATTR_RO(sotp_boot2_rev_id);
static DEVICE_ATTR_RO(temperature_sensor_1_c);
static DEVICE_ATTR_RO(temperature_sensor_2_c);
static DEVICE_ATTR_RO(temperature_sensor_3_c);
static DEVICE_ATTR_RO(voltage_18_mv);
static DEVICE_ATTR_RO(voltage_33_mv);
static DEVICE_ATTR_RO(chip_id);
static DEVICE_ATTR_RO(firmware_status_reg);
static DEVICE_ATTR_RO(fastboot_reg);
static DEVICE_ATTR_RO(pwr_state);

static struct attribute *bcm_vk_card_stat_attributes[] = {

	&dev_attr_chip_id.attr,
	&dev_attr_firmware_status.attr,
	&dev_attr_reset_reason.attr,
	&dev_attr_os_state.attr,
	&dev_attr_firmware_version.attr,
	&dev_attr_rev_flash_rom.attr,
	&dev_attr_rev_boot1.attr,
	&dev_attr_rev_boot2.attr,
	&dev_attr_rev_driver.attr,
	&dev_attr_bus.attr,
	&dev_attr_card_state.attr,
	&dev_attr_temp_threshold_lower_c.attr,
	&dev_attr_temp_threshold_upper_c.attr,
	&dev_attr_freq_core_mhz.attr,
	&dev_attr_freq_mem_mhz.attr,
	&dev_attr_mem_size_mb.attr,
	&dev_attr_sotp_dauth_1.attr,
	&dev_attr_sotp_dauth_1_valid.attr,
	&dev_attr_sotp_dauth_2.attr,
	&dev_attr_sotp_dauth_2_valid.attr,
	&dev_attr_sotp_dauth_3.attr,
	&dev_attr_sotp_dauth_3_valid.attr,
	&dev_attr_sotp_dauth_4.attr,
	&dev_attr_sotp_dauth_4_valid.attr,
	&dev_attr_sotp_boot1_rev_id.attr,
	&dev_attr_sotp_boot2_rev_id.attr,
	NULL,
};

static struct attribute *bcm_vk_card_mon_attributes[] = {

	&dev_attr_temperature_sensor_1_c.attr,
	&dev_attr_temperature_sensor_2_c.attr,
	&dev_attr_temperature_sensor_3_c.attr,
	&dev_attr_voltage_18_mv.attr,
	&dev_attr_voltage_33_mv.attr,
	&dev_attr_firmware_status_reg.attr,
	&dev_attr_fastboot_reg.attr,
	&dev_attr_pwr_state.attr,
	&dev_attr_mem_ecc.attr,
	&dev_attr_mem_uecc.attr,
	&dev_attr_alert_ecc.attr,
	&dev_attr_alert_ssim_busy.attr,
	&dev_attr_alert_afbc_busy.attr,
	&dev_attr_alert_high_temp.attr,
	&dev_attr_alert_malloc_fail_warn.attr,
	&dev_attr_alert_low_temp_warn.attr,
	&dev_attr_alert_ecc_warn.attr,
	NULL,
};

static const struct attribute_group bcm_vk_card_stat_attribute_group = {
	.name = "vk-card-status",
	.attrs = bcm_vk_card_stat_attributes,
};

static const struct attribute_group bcm_vk_card_mon_attribute_group = {
	.name = "vk-card-mon",
	.attrs = bcm_vk_card_mon_attributes,
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
	if (nr_ib_sgl_blk > BCM_VK_IB_SGL_BLK_MAX) {
		dev_warn(dev, "Inband SGL blk %d limited to max %d\n",
			 nr_ib_sgl_blk, BCM_VK_IB_SGL_BLK_MAX);
		nr_ib_sgl_blk = BCM_VK_IB_SGL_BLK_MAX;
	}
	vk->ib_sgl_size = nr_ib_sgl_blk * VK_MSGQ_BLK_SIZE;
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

	/* The tdma is a scratch area for some DMA testings. */
	if (nr_scratch_pages) {
		vk->tdma_vaddr = dma_alloc_coherent(dev,
					 nr_scratch_pages * PAGE_SIZE,
					 &vk->tdma_addr, GFP_KERNEL);
		if (!vk->tdma_vaddr) {
			err = -ENOMEM;
			goto err_disable_pdev;
		}
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

	/* sync other info */
	bcm_vk_sync_card_info(vk);

	dev_info(dev, "create sysfs group for bcm-vk.%d\n", id);
	err = sysfs_create_group(&pdev->dev.kobj,
				 &bcm_vk_card_stat_attribute_group);
	if (err < 0) {
		dev_err(dev,
			"failed to create card status attr for bcm.vk.%d\n",
			id);
		goto err_kfree_name;
	}
	err = sysfs_create_group(&pdev->dev.kobj,
				 &bcm_vk_card_mon_attribute_group);
	if (err < 0) {
		dev_err(dev,
			"failed to create card mon attr for bcm.vk.%d\n",
			id);
		goto err_free_card_stat_group;
	}

	/* create symbolic link from misc device to bus directory */
	err = sysfs_create_link(&misc_device->this_device->kobj,
				&pdev->dev.kobj, BCM_VK_BUS_SYMLINK_NAME);
	if (err < 0) {
		dev_err(dev, "failed to create symlink for bcm.vk.%d\n", id);
		goto err_free_card_mon_group;
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

	/* register for panic notifier */
	vk->panic_nb.notifier_call = bcm_vk_on_panic;
	atomic_notifier_chain_register(&panic_notifier_list,
				       &vk->panic_nb);

	snprintf(name, sizeof(name), KBUILD_MODNAME ".%d_ttyVK", id);
	err = bcm_vk_tty_init(vk, name);
	if (err)
		goto err_free_sysfs_entry;

	/*
	 * lets trigger an auto download.  We don't want to do it serially here
	 * because at probing time, it is not supposed to block for a long time.
	 */
	if (auto_load)
		if (bcm_vk_trigger_autoload(vk))
			goto err_bcm_vk_tty_exit;

	dev_info(dev, "BCM-VK:%u created, 0x%p\n", id, vk);

	return 0;

err_bcm_vk_tty_exit:
	bcm_vk_tty_exit(vk);

err_free_sysfs_entry:
	sysfs_remove_link(&misc_device->this_device->kobj,
			  BCM_VK_BUS_SYMLINK_NAME);

err_free_card_mon_group:
	sysfs_remove_group(&pdev->dev.kobj, &bcm_vk_card_mon_attribute_group);
err_free_card_stat_group:
	sysfs_remove_group(&pdev->dev.kobj, &bcm_vk_card_stat_attribute_group);

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

	bcm_vk_tty_exit(vk);

	/* unregister panic notifier */
	atomic_notifier_chain_unregister(&panic_notifier_list,
				       &vk->panic_nb);

	/* remove the sysfs entry and symlinks associated */
	sysfs_remove_link(&pdev->dev.kobj, misc_device->name);
	sysfs_remove_link(&misc_device->this_device->kobj,
			  BCM_VK_BUS_SYMLINK_NAME);
	sysfs_remove_group(&pdev->dev.kobj, &bcm_vk_card_mon_attribute_group);
	sysfs_remove_group(&pdev->dev.kobj, &bcm_vk_card_stat_attribute_group);

	cancel_work_sync(&vk->wq_work);
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

static const struct pci_device_id bcm_vk_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_BROADCOM, PCI_DEVICE_ID_VALKYRIE), },
	{ PCI_DEVICE(PCI_VENDOR_ID_BROADCOM, PCI_DEVICE_ID_VIPER), },
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
MODULE_VERSION("1.0");
