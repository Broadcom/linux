// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018-2020 Broadcom.
 */

#include <linux/module.h>
#include <linux/kernel.h>

#include "bcm_vk.h"

#define BCM_VK_BITS_NOT_SET(val, bitmask) \
		(((val) & (bitmask)) != (bitmask))

#define BCM_VK_BUS_SYMLINK_NAME		"pci"

struct bcm_vk_sysfs_reg_list {
	const uint64_t offset;
	struct bcm_vk_entry const *tab;
	const uint32_t size;
	const char *hdr;
};

/*
 * table structure for all shutdown related info in fw status register
 */
static struct bcm_vk_entry const fw_shutdown_reg_tab[] = {
	{VK_FWSTS_APP_DEINIT_START, VK_FWSTS_APP_DEINIT_START,
	 "app_deinit_st"},
	{VK_FWSTS_APP_DEINIT_DONE, VK_FWSTS_APP_DEINIT_DONE,
	 "app_deinited"},
	{VK_FWSTS_DRV_DEINIT_START, VK_FWSTS_DRV_DEINIT_START,
	 "drv_deinit_st"},
	{VK_FWSTS_DRV_DEINIT_DONE, VK_FWSTS_DRV_DEINIT_DONE,
	 "drv_deinited"},
	{VK_FWSTS_RESET_DONE, VK_FWSTS_RESET_DONE,
	 "reset_done"},
	 /* reboot reason */
	{VK_FWSTS_RESET_REASON_MASK, VK_FWSTS_RESET_SYS_PWRUP,
	 "sys_pwrup"},
	{VK_FWSTS_RESET_REASON_MASK, VK_FWSTS_RESET_MBOX_DB,
	 "reset_doorbell"},
	{VK_FWSTS_RESET_REASON_MASK, VK_FWSTS_RESET_M7_WDOG,
	 "wdog"},
	{VK_FWSTS_RESET_REASON_MASK, VK_FWSTS_RESET_TEMP,
	 "overheat"},
	{VK_FWSTS_RESET_REASON_MASK, VK_FWSTS_RESET_PCI_FLR,
	 "pci_flr"},
	{VK_FWSTS_RESET_REASON_MASK, VK_FWSTS_RESET_PCI_HOT,
	 "pci_hot"},
	{VK_FWSTS_RESET_REASON_MASK, VK_FWSTS_RESET_PCI_WARM,
	 "pci_warm" },
	{VK_FWSTS_RESET_REASON_MASK, VK_FWSTS_RESET_PCI_COLD,
	 "pci_cold" },
	{VK_FWSTS_RESET_REASON_MASK, VK_FWSTS_RESET_L1,
	 "L1_reset" },
	{VK_FWSTS_RESET_REASON_MASK, VK_FWSTS_RESET_L0,
	 "L0_reset" },
	{VK_FWSTS_RESET_REASON_MASK, VK_FWSTS_RESET_UNKNOWN,
	 "unknown" },
};

/* define for the start of the reboot reason */
#define FW_STAT_RB_REASON_START 5

/* table for all fast boot register related items */
static struct bcm_vk_entry const boot_reg_tab[] = {
	/* download status */
	{FW_LOADER_ACK_SEND_MORE_DATA, FW_LOADER_ACK_SEND_MORE_DATA,
	 "bt1_needs_data"},
	{FW_LOADER_ACK_IN_PROGRESS, FW_LOADER_ACK_IN_PROGRESS,
	 "bt1_inprog"},
	{FW_LOADER_ACK_RCVD_ALL_DATA, FW_LOADER_ACK_RCVD_ALL_DATA,
	 "bt2_dload_done"},
	/* running state */
	{BOOT_STATE_MASK, BROM_NOT_RUN,  "ucode_not_run"},
	{BOOT_STATE_MASK, BROM_RUNNING,  "wait_boot1"},
	{BOOT_STATE_MASK, BOOT1_RUNNING, "wait_boot2"},
	{BOOT_STATE_MASK, BOOT2_RUNNING, "boot2_running"},
};

/* define for the start of OS state */
#define OS_STATE_START 3

static int bcm_vk_sysfs_dump_reg(uint32_t reg_val,
				 struct bcm_vk_entry const *entry_tab,
				 const uint32_t table_size, char *buf)
{
	uint32_t i, masked_val;
	struct bcm_vk_entry const *entry;
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

static int sysfs_chk_fw_status(struct bcm_vk *vk, uint32_t mask,
			       char *buf, const char *err_log)
{
	uint32_t fw_status;
	int ret = 0;

	/* if card OS is not running, no one will update the value */
	fw_status = vkread32(vk, BAR_0, VK_BAR_FWSTS);
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

	fw_status = vkread32(vk, BAR_0, VK_BAR_FWSTS);

	return sprintf(buf, "0x%x\n", fw_status);
}

static ssize_t boot_status_reg_show(struct device *dev,
				    struct device_attribute *devattr,
				    char *buf)
{
	uint32_t boot_status;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	boot_status = vkread32(vk, BAR_0, BAR_BOOT_STATUS);

	return sprintf(buf, "0x%x\n", boot_status);
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
	ret = sysfs_chk_fw_status(vk, FIRMWARE_STATUS_PRE_INIT_DONE,
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
	ret = sysfs_chk_fw_status(vk, FIRMWARE_STATUS_PRE_INIT_DONE,
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
	static struct bcm_vk_entry const fw_status_reg_tab[] = {
		{VK_FWSTS_RELOCATION_ENTRY, VK_FWSTS_RELOCATION_ENTRY,
		 "relo_entry"},
		{VK_FWSTS_RELOCATION_EXIT, VK_FWSTS_RELOCATION_EXIT,
		 "relo_exit"},
		{VK_FWSTS_INIT_START, VK_FWSTS_INIT_START,
		 "init_st"},
		{VK_FWSTS_ARCH_INIT_DONE, VK_FWSTS_ARCH_INIT_DONE,
		 "arch_inited"},
		{VK_FWSTS_PRE_KNL1_INIT_DONE, VK_FWSTS_PRE_KNL1_INIT_DONE,
		 "pre_kern1_inited"},
		{VK_FWSTS_PRE_KNL2_INIT_DONE, VK_FWSTS_PRE_KNL2_INIT_DONE,
		  "pre_kern2_inited"},
		{VK_FWSTS_POST_KNL_INIT_DONE, VK_FWSTS_POST_KNL_INIT_DONE,
		  "kern_inited"},
		{VK_FWSTS_INIT_DONE, VK_FWSTS_INIT_DONE,
		 "card_os_inited"},
		{VK_FWSTS_APP_INIT_START, VK_FWSTS_APP_INIT_START,
		 "app_init_st"},
		{VK_FWSTS_APP_INIT_DONE, VK_FWSTS_APP_INIT_DONE,
		 "app_inited"},
	};
	/* list of registers */
	static struct bcm_vk_sysfs_reg_list const fw_status_reg_list[] = {
		{VK_BAR_FWSTS, fw_status_reg_tab,
		 ARRAY_SIZE(fw_status_reg_tab),
		 "FW status"},
		{BAR_BOOT_STATUS, boot_reg_tab,
		 ARRAY_SIZE(boot_reg_tab),
		 "Boot status"},
		{VK_BAR_FWSTS, fw_shutdown_reg_tab,
		 ARRAY_SIZE(fw_shutdown_reg_tab),
		 "Last Reset status"},
	};

	reg_status = vkread32(vk, BAR_0, VK_BAR_FWSTS);
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
	static struct bcm_vk_entry const *tab =
		&fw_shutdown_reg_tab[FW_STAT_RB_REASON_START];

	reg = vkread32(vk, BAR_0, VK_BAR_FWSTS);
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

	reg = vkread32(vk, BAR_0, VK_BAR_FWSTS);
	if (BCM_VK_INTF_IS_DOWN(reg))
		return sprintf(buf, "PCIe Intf Down!\n");

	reg = vkread32(vk, BAR_0, BAR_BOOT_STATUS);
	for (i = OS_STATE_START; i < ARRAY_SIZE(boot_reg_tab); i++) {
		if ((boot_reg_tab[i].mask & reg) == boot_reg_tab[i].exp_val)
			return sprintf(buf, "%s\n", boot_reg_tab[i].str);
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
	static const char * const pwr_state_tab[] = {
		"Full", "Reduced", "Lowest"};
	char *pwr_state_str;

	/*
	 * host detected alerts are available even if FW has gone down,
	 * display first.
	 */
	reg = vk->host_alert.flags;
	ret = sprintf(p_buf, "Host Alerts: 0x%08x\n", reg);
	if (ret < 0)
		goto card_state_show_fail;

	dev_dbg(dev, "%s", p_buf);
	p_buf += ret;

	ret = bcm_vk_sysfs_dump_reg(reg,
				    bcm_vk_host_err,
				    ARRAY_SIZE(bcm_vk_host_err),
				    p_buf);
	if (ret < 0)
		goto card_state_show_fail;
	p_buf += ret;

	/* next, see if there is any peer latched alert */
	reg = vk->peer_alert.flags;
	ret = sprintf(p_buf, "Peer Alerts: 0x%08x\n", reg);
	if (ret < 0)
		goto card_state_show_fail;

	dev_dbg(dev, "%s", p_buf);
	p_buf += ret;

	ret = bcm_vk_sysfs_dump_reg(reg,
				    bcm_vk_peer_err,
				    ARRAY_SIZE(bcm_vk_peer_err),
				    p_buf);
	if (ret < 0)
		goto card_state_show_fail;
	p_buf += ret;

	/* if OS is not running, no one will update the value */
	ret = sysfs_chk_fw_status(vk, VK_FWSTS_READY, p_buf,
				  "card_state: n/a (fw not running)\n");
	if (ret) {
		p_buf += ret;
		return (p_buf - buf);
	}

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
			 (char *)pwr_state_tab[pwr_state - 1] : "n/a";
	ret = sprintf(p_buf, _PWR_AND_THRE_FMT, reg, pwr_state, pwr_state_str,
		      low_temp_thre, high_temp_thre);
	if (ret < 0)
		goto card_state_show_fail;
	p_buf += ret;
	dev_dbg(dev, _PWR_AND_THRE_FMT, reg, pwr_state, pwr_state_str,
		low_temp_thre, high_temp_thre);

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

static ssize_t uptime_s_show(struct device *dev,
			     struct device_attribute *devattr, char *buf)
{
	unsigned int uptime_s;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	uptime_s = vkread32(vk, BAR_0, BAR_OS_UPTIME);

	dev_dbg(dev, "up_time : %u s\n", uptime_s);
	return sprintf(buf, "%d\n", uptime_s);
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
	ret = sysfs_chk_fw_status(vk, VK_FWSTS_READY, buf, "0\n");
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
	ret = sysfs_chk_fw_status(vk, VK_FWSTS_READY, buf, "0\n");
	if (ret)
		return ret;

	/* display memory error */
	reg = vkread32(vk, BAR_0, BAR_CARD_ERR_MEM);
	BCM_VK_EXTRACT_FIELD(uecc_mem_err, reg,
			     BCM_VK_MEM_ERR_FIELD_MASK,
			     BCM_VK_UECC_MEM_ERR_SHIFT);

	return sprintf(buf, "%d\n", uecc_mem_err);
}

static ssize_t sysfs_utilization_update(struct device *dev, char *buf,
					enum proc_mon_type type,
					struct bcm_vk_proc_mon_entry_t
						**p_entryp)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	struct bcm_vk_proc_mon_info *mon = &vk->proc_mon_info;
	struct bcm_vk_proc_mon_entry_t *entry;
	uint32_t offset;
	int ret;

	/* if OS is not running, no one will update the value */
	ret = sysfs_chk_fw_status(vk, VK_FWSTS_READY, buf, "n/a\n");
	if (ret)
		return ret;

	if (type >= mon->num)
		return sprintf(buf, "n/a\n");

	offset = vk->proc_mon_off +
		 offsetof(struct bcm_vk_proc_mon_info, entries) +
		 (type * mon->entry_size);

	entry = &mon->entries[type];
	entry->used = vkread32(vk, BAR_2,
			       offset +
			       offsetof(struct bcm_vk_proc_mon_entry_t,
					used));
	*p_entryp = entry;

	return 0;
}

static ssize_t utilization_show(struct device *dev,
				struct device_attribute *devattr, char *buf)
{
	int ret, i;
	ssize_t cnt = 0;
	struct bcm_vk_proc_mon_entry_t *entry;

	for (i = 0; i < PROC_MON_TYPE_MAX; i++) {
		ret = sysfs_utilization_update(dev, &buf[cnt], i, &entry);
		if (ret)
			return (ret + cnt);

		cnt += sprintf(&buf[cnt], "%s (%d/%d) - %ld%%\n",
			       entry->tag,
			       entry->used,
			       entry->max,
			       entry->max ? (entry->used * 100L)
					     / entry->max : 0);
	}

	return cnt;
}

static ssize_t utilization_pix_show(struct device *dev,
				    struct device_attribute *devattr,
				    char *buf)
{
	struct bcm_vk_proc_mon_entry_t *entry;
	int ret;

	ret = sysfs_utilization_update(dev, buf, PROC_MON_PIXELS, &entry);
	if (ret)
		return ret;

	return sprintf(buf, "%ld\n",
		       (entry->max) ? (entry->used * 100L) / entry->max : 0);
}

static ssize_t utilization_pix_used_show(struct device *dev,
					 struct device_attribute *devattr,
					 char *buf)
{
	struct bcm_vk_proc_mon_entry_t *entry;
	int ret;

	ret = sysfs_utilization_update(dev, buf, PROC_MON_PIXELS, &entry);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", entry->used);
}

static ssize_t utilization_pix_max_show(struct device *dev,
					struct device_attribute *devattr,
					char *buf)
{
	struct bcm_vk_proc_mon_entry_t *entry;
	int ret;

	ret = sysfs_utilization_update(dev, buf, PROC_MON_PIXELS, &entry);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", entry->max);
}

static ssize_t utilization_codec_show(struct device *dev,
				      struct device_attribute *devattr,
				      char *buf)
{
	struct bcm_vk_proc_mon_entry_t *entry;
	int ret;

	ret = sysfs_utilization_update(dev, buf, PROC_MON_SESS, &entry);
	if (ret)
		return ret;

	return sprintf(buf, "%ld\n",
		       (entry->max) ? (entry->used * 100L) / entry->max : 0);
}

static ssize_t utilization_codec_used_show(struct device *dev,
					   struct device_attribute *devattr,
					   char *buf)
{
	struct bcm_vk_proc_mon_entry_t *entry;
	int ret;

	ret = sysfs_utilization_update(dev, buf, PROC_MON_SESS, &entry);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", entry->used);
}

static ssize_t utilization_codec_max_show(struct device *dev,
					  struct device_attribute *devattr,
					  char *buf)
{
	struct bcm_vk_proc_mon_entry_t *entry;
	int ret;

	ret = sysfs_utilization_update(dev, buf, PROC_MON_SESS, &entry);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", entry->max);
}

static ssize_t peer_alert_show(struct device *dev, char *buf,
			       const uint16_t flag)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return sprintf(buf, "%d\n", vk->peer_alert.flags & flag ? 1 : 0);
}

static ssize_t alert_ecc_show(struct device *dev,
			      struct device_attribute *devattr, char *buf)
{
	return peer_alert_show(dev, buf, ERR_LOG_UECC);
}

static ssize_t alert_ssim_busy_show(struct device *dev,
				    struct device_attribute *devattr, char *buf)
{
	return peer_alert_show(dev, buf, ERR_LOG_SSIM_BUSY);
}

static ssize_t alert_afbc_busy_show(struct device *dev,
				    struct device_attribute *devattr, char *buf)
{
	return peer_alert_show(dev, buf, ERR_LOG_AFBC_BUSY);
}

static ssize_t alert_high_temp_show(struct device *dev,
				    struct device_attribute *devattr, char *buf)
{
	return peer_alert_show(dev, buf, ERR_LOG_HIGH_TEMP_ERR);
}

static ssize_t alert_wdog_timeout_show(struct device *dev,
				       struct device_attribute *devattr,
				       char *buf)
{
	return peer_alert_show(dev, buf, ERR_LOG_WDOG_TIMEOUT);
}

static ssize_t alert_sys_fault_show(struct device *dev,
				    struct device_attribute *devattr, char *buf)
{
	return peer_alert_show(dev, buf, ERR_LOG_SYS_FAULT);
}

static ssize_t alert_ramdump_show(struct device *dev,
				  struct device_attribute *devattr, char *buf)
{
	return peer_alert_show(dev, buf, ERR_LOG_RAMDUMP);
}

static ssize_t alert_malloc_fail_warn_show(struct device *dev,
					   struct device_attribute *devattr,
					   char *buf)
{
	return peer_alert_show(dev, buf, ERR_LOG_MEM_ALLOC_FAIL);
}

static ssize_t alert_low_temp_warn_show(struct device *dev,
					struct device_attribute *devattr,
					char *buf)
{
	return peer_alert_show(dev, buf, ERR_LOG_LOW_TEMP_WARN);
}

static ssize_t alert_ecc_warn_show(struct device *dev,
				   struct device_attribute *devattr,
				   char *buf)
{
	return peer_alert_show(dev, buf, ERR_LOG_ECC);
}

static ssize_t host_alert_show(struct device *dev, char *buf,
			       const uint16_t flag)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return sprintf(buf, "%d\n", vk->host_alert.flags & flag ? 1 : 0);
}

static ssize_t alert_pcie_down_show(struct device *dev,
				    struct device_attribute *devattr, char *buf)
{
	return host_alert_show(dev, buf, ERR_LOG_HOST_PCIE_DWN);
}

static ssize_t alert_heartbeat_fail_show(struct device *dev,
					 struct device_attribute *devattr,
					 char *buf)
{
	return host_alert_show(dev, buf, ERR_LOG_HOST_HB_FAIL);
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

	ret = sysfs_chk_fw_status(vk, VK_FWSTS_READY, buf, "0\n");
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

	ret = sysfs_chk_fw_status(vk, VK_FWSTS_READY, buf, "0\n");
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

static const char *sotp_dauth_active(struct bcm_vk *vk, uint32_t idx)
{
	int i;
	struct bcm_vk_dauth_info *info = &vk->dauth_info;

	for (i = 0; i < idx; i++)
		/* if anything below is active, this is inactive */
		if (!strcmp(info->keys[i].valid, "1"))
			return "0";
	/*
	 * when reaching here, all belows are non-active, so return
	 * based on current key's valid value
	 */
	if (!strcmp(info->keys[i].valid, "1"))
		return "1";
	else
		return "0";
}

static ssize_t sotp_dauth_1_show(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return sprintf(buf, "%s\n", vk->dauth_info.keys[0].store);
}

static ssize_t sotp_dauth_1_valid_show(struct device *dev,
				       struct device_attribute *devattr,
				       char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return sprintf(buf, "%s\n", vk->dauth_info.keys[0].valid);
}

static ssize_t sotp_dauth_1_active_status_show(struct device *dev,
					       struct device_attribute *devattr,
					       char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return sprintf(buf, "%s\n", sotp_dauth_active(vk, 0));
}

static ssize_t sotp_dauth_2_show(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return sprintf(buf, "%s\n", vk->dauth_info.keys[1].store);
}

static ssize_t sotp_dauth_2_valid_show(struct device *dev,
				       struct device_attribute *devattr,
				       char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return sprintf(buf, "%s\n", vk->dauth_info.keys[1].valid);
}

static ssize_t sotp_dauth_2_active_status_show(struct device *dev,
					       struct device_attribute *devattr,
					       char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return sprintf(buf, "%s\n", sotp_dauth_active(vk, 1));
}

static ssize_t sotp_dauth_3_show(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return sprintf(buf, "%s\n", vk->dauth_info.keys[2].store);
}

static ssize_t sotp_dauth_3_valid_show(struct device *dev,
				       struct device_attribute *devattr,
				       char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return sprintf(buf, "%s\n", vk->dauth_info.keys[2].valid);
}

static ssize_t sotp_dauth_3_active_status_show(struct device *dev,
					       struct device_attribute *devattr,
					       char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return sprintf(buf, "%s\n", sotp_dauth_active(vk, 2));
}

static ssize_t sotp_dauth_4_show(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return sprintf(buf, "%s\n", vk->dauth_info.keys[3].store);
}

static ssize_t sotp_dauth_4_valid_show(struct device *dev,
				       struct device_attribute *devattr,
				       char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return sprintf(buf, "%s\n", vk->dauth_info.keys[3].valid);
}

static ssize_t sotp_dauth_4_active_status_show(struct device *dev,
					       struct device_attribute *devattr,
					       char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return sprintf(buf, "%s\n", sotp_dauth_active(vk, 3));
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

static void bcm_vk_get_dauth_info(struct bcm_vk *vk)
{
	struct device *dev = &vk->pdev->dev;
	struct bcm_vk_dauth_info *info = &vk->dauth_info;
	int i;

	for (i = 0; i < VK_BAR1_DAUTH_MAX; i++) {
		bcm_vk_sysfs_get_tag(vk, BAR_1, VK_BAR1_DAUTH_STORE_ADDR(i),
				     info->keys[i].store, "%s");
		bcm_vk_sysfs_get_tag(vk, BAR_1, VK_BAR1_DAUTH_VALID_ADDR(i),
				     info->keys[i].valid, "%s");
		dev_dbg(dev, "[%d]: %s valid %s\n", i,
			info->keys[i].store, info->keys[i].valid);
	}
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
static DEVICE_ATTR_RO(uptime_s);
static DEVICE_ATTR_RO(mem_ecc);
static DEVICE_ATTR_RO(mem_uecc);
static DEVICE_ATTR_RO(utilization);
static DEVICE_ATTR_RO(utilization_pix);
static DEVICE_ATTR_RO(utilization_pix_used);
static DEVICE_ATTR_RO(utilization_pix_max);
static DEVICE_ATTR_RO(utilization_codec);
static DEVICE_ATTR_RO(utilization_codec_used);
static DEVICE_ATTR_RO(utilization_codec_max);
static DEVICE_ATTR_RO(alert_ecc);
static DEVICE_ATTR_RO(alert_ssim_busy);
static DEVICE_ATTR_RO(alert_afbc_busy);
static DEVICE_ATTR_RO(alert_high_temp);
static DEVICE_ATTR_RO(alert_wdog_timeout);
static DEVICE_ATTR_RO(alert_sys_fault);
static DEVICE_ATTR_RO(alert_ramdump);
static DEVICE_ATTR_RO(alert_malloc_fail_warn);
static DEVICE_ATTR_RO(alert_low_temp_warn);
static DEVICE_ATTR_RO(alert_ecc_warn);
static DEVICE_ATTR_RO(alert_pcie_down);
static DEVICE_ATTR_RO(alert_heartbeat_fail);
static DEVICE_ATTR_RO(temp_threshold_lower_c);
static DEVICE_ATTR_RO(temp_threshold_upper_c);
static DEVICE_ATTR_RO(freq_core_mhz);
static DEVICE_ATTR_RO(freq_mem_mhz);
static DEVICE_ATTR_RO(mem_size_mb);
static DEVICE_ATTR_RO(sotp_dauth_1);
static DEVICE_ATTR_RO(sotp_dauth_1_valid);
static DEVICE_ATTR_RO(sotp_dauth_1_active_status);
static DEVICE_ATTR_RO(sotp_dauth_2);
static DEVICE_ATTR_RO(sotp_dauth_2_valid);
static DEVICE_ATTR_RO(sotp_dauth_2_active_status);
static DEVICE_ATTR_RO(sotp_dauth_3);
static DEVICE_ATTR_RO(sotp_dauth_3_valid);
static DEVICE_ATTR_RO(sotp_dauth_3_active_status);
static DEVICE_ATTR_RO(sotp_dauth_4);
static DEVICE_ATTR_RO(sotp_dauth_4_valid);
static DEVICE_ATTR_RO(sotp_dauth_4_active_status);
static DEVICE_ATTR_RO(sotp_boot1_rev_id);
static DEVICE_ATTR_RO(sotp_boot2_rev_id);
static DEVICE_ATTR_RO(temperature_sensor_1_c);
static DEVICE_ATTR_RO(temperature_sensor_2_c);
static DEVICE_ATTR_RO(temperature_sensor_3_c);
static DEVICE_ATTR_RO(voltage_18_mv);
static DEVICE_ATTR_RO(voltage_33_mv);
static DEVICE_ATTR_RO(chip_id);
static DEVICE_ATTR_RO(firmware_status_reg);
static DEVICE_ATTR_RO(boot_status_reg);
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
	&dev_attr_uptime_s.attr,
	&dev_attr_temp_threshold_lower_c.attr,
	&dev_attr_temp_threshold_upper_c.attr,
	&dev_attr_freq_core_mhz.attr,
	&dev_attr_freq_mem_mhz.attr,
	&dev_attr_mem_size_mb.attr,
	&dev_attr_sotp_dauth_1.attr,
	&dev_attr_sotp_dauth_1_valid.attr,
	&dev_attr_sotp_dauth_1_active_status.attr,
	&dev_attr_sotp_dauth_2.attr,
	&dev_attr_sotp_dauth_2_valid.attr,
	&dev_attr_sotp_dauth_2_active_status.attr,
	&dev_attr_sotp_dauth_3.attr,
	&dev_attr_sotp_dauth_3_valid.attr,
	&dev_attr_sotp_dauth_3_active_status.attr,
	&dev_attr_sotp_dauth_4.attr,
	&dev_attr_sotp_dauth_4_valid.attr,
	&dev_attr_sotp_dauth_4_active_status.attr,
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
	&dev_attr_boot_status_reg.attr,
	&dev_attr_pwr_state.attr,
	&dev_attr_mem_ecc.attr,
	&dev_attr_mem_uecc.attr,
	&dev_attr_utilization.attr,
	&dev_attr_utilization_pix.attr,
	&dev_attr_utilization_pix_used.attr,
	&dev_attr_utilization_pix_max.attr,
	&dev_attr_utilization_codec.attr,
	&dev_attr_utilization_codec_used.attr,
	&dev_attr_utilization_codec_max.attr,
	&dev_attr_alert_ecc.attr,
	&dev_attr_alert_ssim_busy.attr,
	&dev_attr_alert_afbc_busy.attr,
	&dev_attr_alert_high_temp.attr,
	&dev_attr_alert_wdog_timeout.attr,
	&dev_attr_alert_sys_fault.attr,
	&dev_attr_alert_ramdump.attr,
	&dev_attr_alert_malloc_fail_warn.attr,
	&dev_attr_alert_low_temp_warn.attr,
	&dev_attr_alert_ecc_warn.attr,
	&dev_attr_alert_pcie_down.attr,
	&dev_attr_alert_heartbeat_fail.attr,
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

int bcm_vk_sysfs_init(struct pci_dev *pdev, struct miscdevice *misc_device)
{
	struct device *dev = &pdev->dev;
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	int rc;

	dev_info(dev, "create sysfs group for bcm-vk\n");
	rc = sysfs_create_group(&pdev->dev.kobj,
				&bcm_vk_card_stat_attribute_group);
	if (rc < 0) {
		dev_err(dev, "failed to create card status attr\n");
		goto err_sysfs_exit;
	}
	rc = sysfs_create_group(&pdev->dev.kobj,
				&bcm_vk_card_mon_attribute_group);
	if (rc < 0) {
		dev_err(dev, "failed to create card mon attr\n");
		goto err_free_card_stat_group;
	}

	/* create symbolic link from misc device to bus directory */
	rc = sysfs_create_link(&misc_device->this_device->kobj,
			       &pdev->dev.kobj, BCM_VK_BUS_SYMLINK_NAME);
	if (rc < 0) {
		dev_err(dev, "failed to create symlink\n");
		goto err_free_card_mon_group;
	}
	/* create symbolic link from bus to misc device also */
	rc = sysfs_create_link(&pdev->dev.kobj,
			       &misc_device->this_device->kobj,
			       misc_device->name);
	if (rc < 0) {
		dev_err(dev, "failed to create reverse symlink\n");
		goto err_free_sysfs_entry;
	}

	/* read dauth info */
	bcm_vk_get_dauth_info(vk);

	return 0;

err_free_sysfs_entry:
	sysfs_remove_link(&misc_device->this_device->kobj,
			  BCM_VK_BUS_SYMLINK_NAME);

err_free_card_mon_group:
	sysfs_remove_group(&pdev->dev.kobj, &bcm_vk_card_mon_attribute_group);
err_free_card_stat_group:
	sysfs_remove_group(&pdev->dev.kobj, &bcm_vk_card_stat_attribute_group);

err_sysfs_exit:
	return rc;
}

void bcm_vk_sysfs_exit(struct pci_dev *pdev, struct miscdevice *misc_device)
{
	/* remove the sysfs entry and symlinks associated */
	sysfs_remove_link(&pdev->dev.kobj, misc_device->name);
	sysfs_remove_link(&misc_device->this_device->kobj,
			  BCM_VK_BUS_SYMLINK_NAME);
	sysfs_remove_group(&pdev->dev.kobj, &bcm_vk_card_mon_attribute_group);
	sysfs_remove_group(&pdev->dev.kobj, &bcm_vk_card_stat_attribute_group);
}
