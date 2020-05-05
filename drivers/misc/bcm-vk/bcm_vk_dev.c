// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018-2020 Broadcom.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/poll.h>
#include <uapi/linux/misc/bcm_vk.h>

#include "bcm_vk.h"

#define PCI_DEVICE_ID_VALKYRIE	0x5E87
#define PCI_DEVICE_ID_VIPER	0x5E88

static DEFINE_IDA(bcm_vk_ida);

struct load_image_tab {
	const uint32_t image_type;
	const char *image_name;
};

enum soc_idx {
	VALKYRIE = 0,
	VIPER,
	VK_IDX_INVALID
};

#define NUM_BOOT_STAGES 2
/* default firmware images names */
static const struct load_image_tab image_tab[][NUM_BOOT_STAGES] = {
	[VALKYRIE] = {
		{VK_IMAGE_TYPE_BOOT1, "vk-boot1.bin"},
		{VK_IMAGE_TYPE_BOOT2, "vk-boot2.bin"}
	},
	[VIPER] = {
		{VK_IMAGE_TYPE_BOOT1, "vp-boot1.bin"},
		{VK_IMAGE_TYPE_BOOT2, "vp-boot2.bin"}
	}
};

/* Location of memory base addresses of interest in BAR1 */
/* Load Boot1 to start of ITCM */
#define BAR1_CODEPUSH_BASE_BOOT1	0x100000

/* Allow minimum 1s for Load Image timeout responses */
#define LOAD_IMAGE_TIMEOUT_MS		(1 * MSEC_PER_SEC)

/* Image startup timeouts */
#define BOOT1_STARTUP_TIMEOUT_MS	(5 * MSEC_PER_SEC)
#define BOOT2_STARTUP_TIMEOUT_MS	(10 * MSEC_PER_SEC)

/* 1ms wait for checking the transfer complete status */
#define TXFR_COMPLETE_TIMEOUT_MS	1

/* MSIX usages */
#define VK_MSIX_MSGQ_MAX		3
#define VK_MSIX_NOTF_MAX		1
#define VK_MSIX_TTY_MAX			BCM_VK_NUM_TTY
#define VK_MSIX_IRQ_MAX			(VK_MSIX_MSGQ_MAX + VK_MSIX_NOTF_MAX + \
					 VK_MSIX_TTY_MAX)

/* Number of bits set in DMA mask*/
#define BCM_VK_DMA_BITS			64

/* Ucode boot wait time */
#define BCM_VK_UCODE_BOOT_US            (100 * USEC_PER_MSEC)
/* 50% margin */
#define BCM_VK_UCODE_BOOT_MAX_US        ((BCM_VK_UCODE_BOOT_US * 3) >> 1)

/* deinit time for the card os after receiving doorbell */
#define BCM_VK_DEINIT_TIME_MS		(2 * MSEC_PER_SEC)

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

/*
 * alerts that could be generated from peer
 */
struct bcm_vk_entry const bcm_vk_peer_err[BCM_VK_PEER_ERR_NUM] = {
	{ERR_LOG_UECC, ERR_LOG_UECC, "uecc"},
	{ERR_LOG_SSIM_BUSY, ERR_LOG_SSIM_BUSY, "ssim_busy"},
	{ERR_LOG_AFBC_BUSY, ERR_LOG_AFBC_BUSY, "afbc_busy"},
	{ERR_LOG_HIGH_TEMP_ERR, ERR_LOG_HIGH_TEMP_ERR, "high_temp"},
	{ERR_LOG_WDOG_TIMEOUT, ERR_LOG_WDOG_TIMEOUT, "wdog_timeout"},
	{ERR_LOG_SYS_FAULT, ERR_LOG_SYS_FAULT, "sys_fault"},
	{ERR_LOG_RAMDUMP, ERR_LOG_RAMDUMP, "ramdump"},
	{ERR_LOG_MEM_ALLOC_FAIL, ERR_LOG_MEM_ALLOC_FAIL, "malloc_fail warn"},
	{ERR_LOG_LOW_TEMP_WARN, ERR_LOG_LOW_TEMP_WARN, "low_temp warn"},
	{ERR_LOG_ECC, ERR_LOG_ECC, "ecc"},
};

/* alerts detected by the host */
struct bcm_vk_entry const bcm_vk_host_err[BCM_VK_HOST_ERR_NUM] = {
	{ERR_LOG_HOST_PCIE_DWN, ERR_LOG_HOST_PCIE_DWN, "PCIe_down"},
	{ERR_LOG_HOST_HB_FAIL, ERR_LOG_HOST_HB_FAIL, "hb_fail"},
};

irqreturn_t bcm_vk_notf_irqhandler(int irq, void *dev_id)
{
	struct bcm_vk *vk = dev_id;

	if (!bcm_vk_drv_access_ok(vk)) {
		dev_err(&vk->pdev->dev,
			"Interrupt %d received when msgq not inited\n", irq);
		goto skip_schedule_work;
	}

	/* if notification is not pending, set bit and schedule work */
	if (test_and_set_bit(BCM_VK_WQ_NOTF_PEND, vk->wq_offload) == 0)
		queue_work(vk->wq_thread, &vk->wq_work);

skip_schedule_work:
	return IRQ_HANDLED;
}

static void bcm_vk_log_notf(struct bcm_vk *vk,
			    struct bcm_vk_alert *alert,
			    struct bcm_vk_entry const *entry_tab,
			    const uint32_t table_size)
{
	uint32_t i;
	uint32_t masked_val, latched_val;
	struct bcm_vk_entry const *entry;
	uint32_t reg;
	uint16_t ecc_mem_err, uecc_mem_err;
	struct device *dev = &vk->pdev->dev;

	for (i = 0; i < table_size; i++) {
		entry = &entry_tab[i];
		masked_val = entry->mask & alert->notfs;
		latched_val = entry->mask & alert->flags;

		if (masked_val == ERR_LOG_UECC) {
			/*
			 * if there is difference between stored cnt and it
			 * is greater than threshold, log it.
			 */
			reg = vkread32(vk, BAR_0, BAR_CARD_ERR_MEM);
			BCM_VK_EXTRACT_FIELD(uecc_mem_err, reg,
					     BCM_VK_MEM_ERR_FIELD_MASK,
					     BCM_VK_UECC_MEM_ERR_SHIFT);
			if ((uecc_mem_err != vk->alert_cnts.uecc) &&
			    (uecc_mem_err >= BCM_VK_UECC_THRESHOLD))
				dev_info(dev,
					 "ALERT! %s.%d uecc RAISED - ErrCnt %d\n",
					 DRV_MODULE_NAME, vk->misc_devid,
					 uecc_mem_err);
			vk->alert_cnts.uecc = uecc_mem_err;
		} else if (masked_val == ERR_LOG_ECC) {
			reg = vkread32(vk, BAR_0, BAR_CARD_ERR_MEM);
			BCM_VK_EXTRACT_FIELD(ecc_mem_err, reg,
					     BCM_VK_MEM_ERR_FIELD_MASK,
					     BCM_VK_ECC_MEM_ERR_SHIFT);
			if ((ecc_mem_err != vk->alert_cnts.ecc) &&
			    (ecc_mem_err >= BCM_VK_ECC_THRESHOLD))
				dev_info(dev, "ALERT! %s.%d ecc RAISED - ErrCnt %d\n",
					 DRV_MODULE_NAME, vk->misc_devid,
					 ecc_mem_err);
			vk->alert_cnts.ecc = ecc_mem_err;
		} else if (masked_val != latched_val) {
			/* print a log as info */
			dev_info(dev, "ALERT! %s.%d %s %s\n",
				 DRV_MODULE_NAME, vk->misc_devid, entry->str,
				 masked_val ? "RAISED" : "CLEARED");
		}
	}
}

static void bcm_vk_dump_peer_log(struct bcm_vk *vk)
{
	struct bcm_vk_peer_log log;
	char loc_buf[BCM_VK_PEER_LOG_LINE_MAX];
	int cnt;
	struct device *dev = &vk->pdev->dev;
	uint data_offset;

	memcpy_fromio(&log, vk->bar[BAR_2] + vk->peerlog_off, sizeof(log));

	dev_dbg(dev, "Peer PANIC: Size 0x%x(0x%x), [Rd Wr] = [%d %d]\n",
		log.buf_size, log.mask, log.rd_idx, log.wr_idx);

	cnt = 0;
	data_offset = vk->peerlog_off + sizeof(struct bcm_vk_peer_log);
	while (log.rd_idx != log.wr_idx) {
		loc_buf[cnt] = vkread8(vk, BAR_2, data_offset + log.rd_idx);

		if ((loc_buf[cnt] == '\0') ||
		    (cnt == (BCM_VK_PEER_LOG_LINE_MAX - 1))) {
			dev_err(dev, "%s", loc_buf);
			cnt = 0;
		} else {
			cnt++;
		}
		log.rd_idx = (log.rd_idx + 1) & log.mask;
	}
	/* update rd idx at the end */
	vkwrite32(vk, log.rd_idx, BAR_2,
		  vk->peerlog_off + offsetof(struct bcm_vk_peer_log, rd_idx));
}

void bcm_vk_handle_notf(struct bcm_vk *vk)
{
	uint32_t reg;
	struct bcm_vk_alert alert;
	bool intf_down;
	unsigned long flags;

	/* handle peer alerts and then locally detected ones */
	reg = vkread32(vk, BAR_0, BAR_CARD_ERR_LOG);
	intf_down = BCM_VK_INTF_IS_DOWN(reg);
	if (!intf_down) {
		vk->peer_alert.notfs = reg;
		bcm_vk_log_notf(vk, &vk->peer_alert, bcm_vk_peer_err,
				ARRAY_SIZE(bcm_vk_peer_err));
		vk->peer_alert.flags = vk->peer_alert.notfs;
	} else {
		/* turn off access */
		bcm_vk_blk_drv_access(vk);
	}

	/* check and make copy of alert with lock and then free lock */
	spin_lock_irqsave(&vk->host_alert_lock, flags);
	if (intf_down)
		vk->host_alert.notfs |= ERR_LOG_HOST_PCIE_DWN;

	alert = vk->host_alert;
	vk->host_alert.flags = vk->host_alert.notfs;
	spin_unlock_irqrestore(&vk->host_alert_lock, flags);

	/* call display with copy */
	bcm_vk_log_notf(vk, &alert, bcm_vk_host_err,
			ARRAY_SIZE(bcm_vk_host_err));

	/*
	 * If it is a sys fault or heartbeat timeout, we would like extract
	 * log msg from the card so that we would know what is the last fault
	 */
	if (!intf_down &&
	    ((vk->host_alert.flags & ERR_LOG_HOST_HB_FAIL) ||
	     (vk->peer_alert.flags & ERR_LOG_SYS_FAULT)))
		bcm_vk_dump_peer_log(vk);
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

	/* get the peer log pointer, only need the offset */
	vk->peerlog_off = offset;
}

static void bcm_vk_get_proc_mon_info(struct bcm_vk *vk)
{
	struct device *dev = &vk->pdev->dev;
	struct bcm_vk_proc_mon_info *mon = &vk->proc_mon_info;
	uint32_t num, entry_size, offset, buf_size;
	uint8_t *dst;

	/* calculate offset which is based on peerlog offset */
	buf_size = vkread32(vk, BAR_2,
			    vk->peerlog_off
			    + offsetof(struct bcm_vk_peer_log, buf_size));
	offset = vk->peerlog_off + sizeof(struct bcm_vk_peer_log)
		 + buf_size;

	/* first read the num and entry size */
	num = vkread32(vk, BAR_2, offset);
	entry_size = vkread32(vk, BAR_2, offset + sizeof(num));

	/* check for max allowed */
	if (num > BCM_VK_PROC_MON_MAX) {
		dev_err(dev, "Processing monitoring entry %d exceeds max %d\n",
			num, BCM_VK_PROC_MON_MAX);
		return;
	}
	mon->num = num;
	mon->entry_size = entry_size;

	vk->proc_mon_off = offset;

	/* read it once that will capture those static info */
	dst = (uint8_t *)&mon->entries[0];
	offset += sizeof(num) + sizeof(entry_size);
	memcpy_fromio(dst, vk->bar[BAR_2] + offset, num * entry_size);
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
		vkwrite32(vk, (uint64_t)vk->tdma_addr >> 32, BAR_1,
			  VK_BAR1_SCRATCH_OFF_HI);
		vkwrite32(vk, (uint32_t)vk->tdma_addr, BAR_1,
			  VK_BAR1_SCRATCH_OFF_LO);
		vkwrite32(vk, nr_scratch_pages * PAGE_SIZE, BAR_1,
			  VK_BAR1_SCRATCH_SZ_ADDR);
	}

	/* get static card info, only need to read once */
	bcm_vk_get_card_info(vk);

	/* get the proc mon info once */
	bcm_vk_get_proc_mon_info(vk);

	return 0;
}

void bcm_vk_blk_drv_access(struct bcm_vk *vk)
{
	int i;

	/*
	 * kill all the apps except for the process that is resetting.
	 * If not called during reset, reset_pid will be 0, and all will be
	 * killed.
	 */
	spin_lock(&vk->ctx_lock);

	/* set msgq_inited to 0 so that all rd/wr will be blocked */
	atomic_set(&vk->msgq_inited, 0);

	for (i = 0; i < VK_PID_HT_SZ; i++) {
		struct bcm_vk_ctx *ctx;

		list_for_each_entry(ctx, &vk->pid_ht[i].head, node) {
			if (ctx->pid != vk->reset_pid) {
				dev_dbg(&vk->pdev->dev,
					"Send kill signal to pid %d\n",
					ctx->pid);
				kill_pid(find_vpid(ctx->pid), SIGKILL, 1);
			}
		}
	}
	bcm_vk_tty_terminate_tty_user(vk);
	spin_unlock(&vk->ctx_lock);
}

static void bcm_vk_buf_notify(struct bcm_vk *vk, void *bufp,
			      dma_addr_t host_buf_addr, uint32_t buf_size)
{
	/* update the dma address to the card */
	vkwrite32(vk, (uint64_t)host_buf_addr >> 32, BAR_1,
		  VK_BAR1_DMA_BUF_OFF_HI);
	vkwrite32(vk, (uint32_t)host_buf_addr, BAR_1,
		  VK_BAR1_DMA_BUF_OFF_LO);
	vkwrite32(vk, buf_size, BAR_1, VK_BAR1_DMA_BUF_SZ);
}

static int bcm_vk_load_image_by_type(struct bcm_vk *vk, u32 load_type,
				     const char *filename)
{
	struct device *dev = &vk->pdev->dev;
	const struct firmware *fw = NULL;
	void *bufp = NULL;
	size_t max_buf;
	int ret;
	uint64_t offset_codepush;
	u32 codepush;
	u32 value;
	dma_addr_t boot_dma_addr;

	if (load_type == VK_IMAGE_TYPE_BOOT1) {
		/*
		 * After POR, enable VK soft BOOTSRC so bootrom do not clear
		 * the pushed image (the TCM memories).
		 */
		value = vkread32(vk, BAR_0, BAR_BOOTSRC_SELECT);
		value |= BOOTSRC_SOFT_ENABLE;
		vkwrite32(vk, value, BAR_0, BAR_BOOTSRC_SELECT);

		codepush = CODEPUSH_BOOTSTART + CODEPUSH_BOOT1_ENTRY;
		offset_codepush = BAR_CODEPUSH_SBL;

		/* Write a 1 to request SRAM open bit */
		vkwrite32(vk, CODEPUSH_BOOTSTART, BAR_0, offset_codepush);

		/* Wait for VK to respond */
		ret = bcm_vk_wait(vk, BAR_0, BAR_BOOT_STATUS, SRAM_OPEN,
				  SRAM_OPEN, LOAD_IMAGE_TIMEOUT_MS);
		if (ret < 0) {
			dev_err(dev, "boot1 timeout\n");
			goto err_buf_out;
		}

		max_buf = SZ_256K;
		bufp = dma_alloc_coherent(dev,
					  max_buf,
					  &boot_dma_addr, GFP_KERNEL);
		if (!bufp) {
			dev_err(dev, "Error allocating 0x%zx\n", max_buf);
			ret = -ENOMEM;
			goto err_buf_out;
		}
	} else if (load_type == VK_IMAGE_TYPE_BOOT2) {
		codepush = CODEPUSH_BOOT2_ENTRY;
		offset_codepush = BAR_CODEPUSH_SBI;

		/* Wait for VK to respond */
		ret = bcm_vk_wait(vk, BAR_0, BAR_BOOT_STATUS, DDR_OPEN,
				  DDR_OPEN, LOAD_IMAGE_TIMEOUT_MS);
		if (ret < 0) {
			dev_err(dev, "boot2 timeout\n");
			goto err_buf_out;
		}

		max_buf = SZ_4M;
		bufp = dma_alloc_coherent(dev,
					  max_buf,
					  &boot_dma_addr, GFP_KERNEL);
		if (!bufp) {
			dev_err(dev, "Error allocating 0x%zx\n", max_buf);
			ret = -ENOMEM;
			goto err_buf_out;
		}

		bcm_vk_buf_notify(vk, bufp, boot_dma_addr, max_buf);
	} else {
		dev_err(dev, "Error invalid image type 0x%x\n", load_type);
		ret = -EINVAL;
		goto err_buf_out;
	}

	ret = REQUEST_FIRMWARE_INTO_BUF(&fw, filename, dev,
					bufp, max_buf, 0,
					KERNEL_PREAD_FLAG_PART);
	if (ret) {
		dev_err(dev, "Error %d requesting firmware file: %s\n",
			ret, filename);
		goto err_firmware_out;
	}
	dev_dbg(dev, "size=0x%zx\n", fw->size);
	if (load_type == VK_IMAGE_TYPE_BOOT1)
		memcpy_toio(vk->bar[BAR_1] + BAR1_CODEPUSH_BASE_BOOT1,
			    bufp,
			    fw->size);

	dev_dbg(dev, "Signaling 0x%x to 0x%llx\n", codepush, offset_codepush);
	vkwrite32(vk, codepush, BAR_0, offset_codepush);

	if (load_type == VK_IMAGE_TYPE_BOOT1) {
		/* wait until done */
		ret = bcm_vk_wait(vk, BAR_0, BAR_BOOT_STATUS,
				  BOOT1_RUNNING,
				  BOOT1_RUNNING,
				  BOOT1_STARTUP_TIMEOUT_MS);
		if (ret) {
			dev_err(dev,
				"Timeout %ld ms waiting for boot1 to come up\n",
				BOOT1_STARTUP_TIMEOUT_MS);
			goto err_firmware_out;
		}
	} else if (load_type == VK_IMAGE_TYPE_BOOT2) {
		unsigned long timeout;

		timeout = jiffies + msecs_to_jiffies(LOAD_IMAGE_TIMEOUT_MS);

		/* To send more data to VK than max_buf allowed at a time */
		do {
			/*
			 * Check for ack from card. when Ack is received,
			 * it means all the data is received by card.
			 * Exit the loop after ack is received.
			 */
			ret = bcm_vk_wait(vk, BAR_0, BAR_BOOT_STATUS,
					  FW_LOADER_ACK_RCVD_ALL_DATA,
					  FW_LOADER_ACK_RCVD_ALL_DATA,
					  TXFR_COMPLETE_TIMEOUT_MS);
			if (ret == 0) {
				dev_info(dev, "Exit boot2 download\n");
				break;
			}

			/* exit the loop, if there is no response from card */
			if (time_after(jiffies, timeout)) {
				dev_err(dev, "Error. No reply from card\n");
				ret = -ETIMEDOUT;
				goto err_firmware_out;
			}

			/* Wait for VK to open BAR space to copy new data */
			ret = bcm_vk_wait(vk, BAR_0, offset_codepush,
					  codepush, 0,
					  TXFR_COMPLETE_TIMEOUT_MS);
			if (ret == 0) {
				ret = REQUEST_FIRMWARE_INTO_BUF
						(&fw,
						 filename,
						 dev, bufp,
						 max_buf,
						 fw->size,
						 KERNEL_PREAD_FLAG_PART);
				if (ret) {
					dev_err(dev,
						"Error %d requesting firmware file: %s offset: 0x%zx\n",
						ret, filename, fw->size);
					goto err_firmware_out;
				}
				dev_dbg(dev, "size=0x%zx\n", fw->size);
				dev_dbg(dev, "Signaling 0x%x to 0x%llx\n",
					codepush, offset_codepush);
				vkwrite32(vk, codepush, BAR_0, offset_codepush);
				/* reload timeout after every codepush */
				timeout = jiffies +
				    msecs_to_jiffies(LOAD_IMAGE_TIMEOUT_MS);
			}
		} while (1);

		/* wait for fw status bits to indicate app ready */
		ret = bcm_vk_wait(vk, BAR_0, VK_BAR_FWSTS,
				  VK_FWSTS_READY,
				  VK_FWSTS_READY,
				  BOOT2_STARTUP_TIMEOUT_MS);
		if (ret < 0) {
			dev_err(dev, "Boot2 not ready - timeout\n");
			goto err_firmware_out;
		}

		/*
		 * Next, initialize Message Q if we are loading boot2.
		 * Do a force sync
		 */
		ret = bcm_vk_sync_msgq(vk, true);
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

err_buf_out:
	if (bufp)
		dma_free_coherent(dev, max_buf, bufp, boot_dma_addr);

	return ret;
}

static u32 bcm_vk_next_boot_image(struct bcm_vk *vk)
{
	uint32_t boot_status;
	uint32_t fw_status;
	u32 load_type = 0;  /* default for unknown */

	boot_status = vkread32(vk, BAR_0, BAR_BOOT_STATUS);
	fw_status = vkread32(vk, BAR_0, VK_BAR_FWSTS);

	if (!BCM_VK_INTF_IS_DOWN(boot_status) && (boot_status & SRAM_OPEN))
		load_type = VK_IMAGE_TYPE_BOOT1;
	else if (boot_status == BOOT1_RUNNING)
		load_type = VK_IMAGE_TYPE_BOOT2;

	/*
	 * TO_FIX: For now, like to know what value we get everytime
	 *         for debugging.
	 */
	dev_info(&vk->pdev->dev,
		 "boot-status value for next image: 0x%x : fw-status 0x%x\n",
		 boot_status, fw_status);

	return load_type;
}

static enum soc_idx get_soc_idx(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	enum soc_idx idx;

	switch (pdev->device) {
	case PCI_DEVICE_ID_VALKYRIE:
		idx = VALKYRIE;
		break;

	case PCI_DEVICE_ID_VIPER:
		idx = VIPER;
		break;

	default:
		idx = VK_IDX_INVALID;
		dev_err(dev, "no images for 0x%x\n", pdev->device);
	}
	return idx;
}

int bcm_vk_auto_load_all_images(struct bcm_vk *vk)
{
	int i, ret = -1;
	enum soc_idx idx;
	struct device *dev = &vk->pdev->dev;
	uint32_t curr_type;
	const char *curr_name;

	idx = get_soc_idx(dev);
	if (idx >= VK_IDX_INVALID)
		goto auto_load_all_exit;

	/* log a message to know the relative loading order */
	dev_info(dev, "Load All for device %d\n", vk->misc_devid);

	for (i = 0; i < NUM_BOOT_STAGES; i++) {
		curr_type = image_tab[idx][i].image_type;
		if (bcm_vk_next_boot_image(vk) == curr_type) {
			curr_name = image_tab[idx][i].image_name;
			ret = bcm_vk_load_image_by_type(vk, curr_type,
							curr_name);
			dev_info(dev, "Auto load %s, ret %d\n",
				 curr_name, ret);

			if (ret) {
				dev_err(dev, "Error loading default %s\n",
					curr_name);
				goto auto_load_all_exit;
			}
		}
	}

auto_load_all_exit:
	return ret;
}

static int bcm_vk_trigger_autoload(struct bcm_vk *vk)
{
	if (test_and_set_bit(BCM_VK_WQ_DWNLD_PEND, vk->wq_offload) != 0)
		return -EPERM;

	set_bit(BCM_VK_WQ_DWNLD_AUTO, vk->wq_offload);
	queue_work(vk->wq_thread, &vk->wq_work);

	return 0;
}

static long bcm_vk_load_image(struct bcm_vk *vk,
			      const struct vk_image __user *arg)
{
	struct device *dev = &vk->pdev->dev;
	const char *image_name;
	struct vk_image image;
	u32 next_loadable;
	enum soc_idx idx;
	int image_idx;
	int ret;

	if (copy_from_user(&image, arg, sizeof(image)))
		return -EACCES;

	if ((image.type != VK_IMAGE_TYPE_BOOT1) &&
	    (image.type != VK_IMAGE_TYPE_BOOT2)) {
		dev_err(dev, "invalid image.type %u\n", image.type);
		return -EPERM;
	}

	next_loadable = bcm_vk_next_boot_image(vk);
	if (next_loadable != image.type) {
		dev_err(dev, "Next expected image %u, Loading %u\n",
			next_loadable, image.type);
		return -EPERM;
	}

	/*
	 * if something is pending download already.  This could only happen
	 * for now when the driver is being loaded, or if someone has issued
	 * another download command in another shell.
	 */
	if (test_and_set_bit(BCM_VK_WQ_DWNLD_PEND, vk->wq_offload) != 0) {
		dev_err(dev, "Download operation already pending.\n");
		return -EPERM;
	}

	image_name = image.filename;
	if (image_name[0] == '\0') {
		/* Use default image name if NULL */
		idx = get_soc_idx(dev);
		if (idx >= VK_IDX_INVALID)
			return -EPERM;

		/* Image idx starts with boot1 */
		image_idx = image.type - VK_IMAGE_TYPE_BOOT1;
		image_name = image_tab[idx][image_idx].image_name;
	} else {
		/* Ensure filename is NULL terminated */
		image.filename[sizeof(image.filename) - 1] = '\0';
	}
	ret = bcm_vk_load_image_by_type(vk, image.type, image_name);
	clear_bit(BCM_VK_WQ_DWNLD_PEND, vk->wq_offload);

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
	fw_status = vkread32(vk, BAR_0, VK_BAR_FWSTS);
	/* immediate exit if interface goes down */
	if (BCM_VK_INTF_IS_DOWN(fw_status)) {
		dev_err(dev, "PCIe Intf Down!\n");
		goto reset_exit;
	}

	reset_reason = (fw_status & VK_FWSTS_RESET_REASON_MASK);
	if ((reset_reason == VK_FWSTS_RESET_MBOX_DB) ||
	    (reset_reason == VK_FWSTS_RESET_UNKNOWN))
		ret = 0;

	/*
	 * if some of the deinit bits are set, but done
	 * bit is not, this is a failure if triggered while boot2 is running
	 */
	if ((fw_status & VK_FWSTS_DEINIT_TRIGGERED) &&
	    !(fw_status & VK_FWSTS_RESET_DONE))
		ret = -EAGAIN;

reset_exit:
	dev_dbg(dev, "FW status = 0x%x ret %d\n", fw_status, ret);

	return ret;
}

static long bcm_vk_reset(struct bcm_vk *vk, const struct vk_reset __user *arg)
{
	struct device *dev = &vk->pdev->dev;
	struct vk_reset reset;
	int ret = 0;

	if (copy_from_user(&reset, arg, sizeof(struct vk_reset)))
		return -EFAULT;

	dev_info(dev, "Issue Reset\n");

	/*
	 * The following is the sequence of reset:
	 * - send card level graceful shut down
	 * - wait enough time for VK to handle its business, stopping DMA etc
	 * - kill host apps
	 * - Trigger interrupt with DB
	 */
	bcm_vk_send_shutdown_msg(vk, VK_SHUTDOWN_GRACEFUL, 0);

	spin_lock(&vk->ctx_lock);
	if (!vk->reset_pid) {
		vk->reset_pid = task_pid_nr(current);
	} else {
		dev_err(dev, "Reset already launched by process pid %d\n",
			vk->reset_pid);
		ret = -EACCES;
	}
	spin_unlock(&vk->ctx_lock);
	if (ret)
		return ret;

	bcm_vk_blk_drv_access(vk);
	bcm_vk_trigger_reset(vk);

	/*
	 * Wait enough time for card os to deinit
	 * and populate the reset reason.
	 */
	msleep(BCM_VK_DEINIT_TIME_MS);

	ret = bcm_vk_reset_successful(vk);

	return ret;
}

static int bcm_vk_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct bcm_vk_ctx *ctx = file->private_data;
	struct bcm_vk *vk = container_of(ctx->miscdev, struct bcm_vk, miscdev);
	unsigned long pg_size;

	/* only BAR2 is mmap possible, which is bar num 4 due to 64bit */
#define VK_MMAPABLE_BAR 4

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
	const void __user *argp = (void __user *)arg;

	dev_dbg(&vk->pdev->dev,
		"ioctl, cmd=0x%02x, arg=0x%02lx\n",
		cmd, arg);

	mutex_lock(&vk->mutex);

	switch (cmd) {
	case VK_IOCTL_LOAD_IMAGE:
		ret = bcm_vk_load_image(vk, argp);
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
	.poll = bcm_vk_poll,
	.release = bcm_vk_release,
	.mmap = bcm_vk_mmap,
	.unlocked_ioctl = bcm_vk_ioctl,
};

static int bcm_vk_on_panic(struct notifier_block *nb,
			   unsigned long e, void *p)
{
	struct bcm_vk *vk = container_of(nb, struct bcm_vk, panic_nb);

	bcm_to_v_doorbell(vk, VK_BAR0_RESET_DB_NUM, VK_BAR0_RESET_DB_HARD);

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
	uint32_t boot_status;

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
		vk->tdma_vaddr = dma_alloc_coherent
					(dev,
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

	for (vk->num_irqs = 0;
	     vk->num_irqs < VK_MSIX_MSGQ_MAX;
	     vk->num_irqs++) {
		err = devm_request_irq(dev, pci_irq_vector(pdev, vk->num_irqs),
				       bcm_vk_msgq_irqhandler,
				       IRQF_SHARED, DRV_MODULE_NAME, vk);
		if (err) {
			dev_err(dev, "failed to request msgq IRQ %d for MSIX %d\n",
				pdev->irq + vk->num_irqs, vk->num_irqs + 1);
			goto err_irq;
		}
	}
	/* one irq for notification from VK */
	err = devm_request_irq(dev, pci_irq_vector(pdev, vk->num_irqs),
			       bcm_vk_notf_irqhandler,
			       IRQF_SHARED, DRV_MODULE_NAME, vk);
	if (err) {
		dev_err(dev, "failed to request notf IRQ %d for MSIX %d\n",
			pdev->irq + vk->num_irqs, vk->num_irqs + 1);
		goto err_irq;
	}
	vk->num_irqs++;

	for (i = 0; i < VK_MSIX_TTY_MAX; i++) {
		err = devm_request_irq(dev, pci_irq_vector(pdev, vk->num_irqs),
				       bcm_vk_tty_irqhandler,
				       IRQF_SHARED, DRV_MODULE_NAME, vk);
		if (err) {
			dev_err(dev, "failed request tty IRQ %d for MSIX %d\n",
				pdev->irq + vk->num_irqs, vk->num_irqs + 1);
			goto err_irq;
		}
		vk->tty[i].irq_enabled = true;
		vk->num_irqs++;
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
		goto err_misc_deregister;
	}

	/* sync other info */
	bcm_vk_sync_card_info(vk);

	err = bcm_vk_sysfs_init(pdev, misc_device);
	if (err)
		goto err_misc_deregister;

	/* register for panic notifier */
	vk->panic_nb.notifier_call = bcm_vk_on_panic;
	atomic_notifier_chain_register(&panic_notifier_list,
				       &vk->panic_nb);

	snprintf(name, sizeof(name), KBUILD_MODNAME ".%d_ttyVK", id);
	err = bcm_vk_tty_init(vk, name);
	if (err)
		goto err_unregister_panic_notifier;

	/*
	 * lets trigger an auto download.  We don't want to do it serially here
	 * because at probing time, it is not supposed to block for a long time.
	 */
	boot_status = vkread32(vk, BAR_0, BAR_BOOT_STATUS);
	if (auto_load) {
		if ((boot_status & BOOT_STATE_MASK) == BROM_RUNNING) {
			if (bcm_vk_trigger_autoload(vk))
				goto err_bcm_vk_tty_exit;
		} else {
			dev_info(dev,
				 "Auto-load skipped - BROM not in proper state (0x%x)\n",
				 boot_status);
		}
	}

	/* enable hb */
	bcm_vk_hb_init(vk);

	dev_info(dev, "BCM-VK:%u created, 0x%p\n", id, vk);

	return 0;

err_bcm_vk_tty_exit:
	bcm_vk_tty_exit(vk);

err_unregister_panic_notifier:
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &vk->panic_nb);

err_misc_deregister:
	misc_deregister(misc_device);

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

	bcm_vk_hb_deinit(vk);

	/*
	 * Trigger a reset to card and wait enough time for UCODE to rerun,
	 * which re-initialize the card into its default state.
	 * This ensures when driver is re-enumerated it will start from
	 * a completely clean state.
	 */
	bcm_vk_trigger_reset(vk);
	usleep_range(BCM_VK_UCODE_BOOT_US, BCM_VK_UCODE_BOOT_MAX_US);

	/* unregister panic notifier */
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &vk->panic_nb);

	bcm_vk_sysfs_exit(pdev, misc_device);

	bcm_vk_msg_remove(vk);
	bcm_vk_tty_exit(vk);

	if (vk->tdma_vaddr)
		dma_free_coherent(&pdev->dev, nr_scratch_pages * PAGE_SIZE,
				  vk->tdma_vaddr, vk->tdma_addr);

	/* remove if name is set which means misc dev registered */
	if (misc_device->name) {
		misc_deregister(misc_device);
		kfree(misc_device->name);
		ida_simple_remove(&bcm_vk_ida, vk->misc_devid);
	}
	for (i = 0; i < vk->num_irqs; i++)
		devm_free_irq(&pdev->dev, pci_irq_vector(pdev, i), vk);

	pci_disable_msix(pdev);
	pci_disable_msi(pdev);

	cancel_work_sync(&vk->wq_work);
	destroy_workqueue(vk->wq_thread);
	cancel_work_sync(&vk->tty_wq_work);
	destroy_workqueue(vk->tty_wq_thread);

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

MODULE_DESCRIPTION("Broadcom VK Host Driver");
MODULE_AUTHOR("Scott Branden <scott.branden@broadcom.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
