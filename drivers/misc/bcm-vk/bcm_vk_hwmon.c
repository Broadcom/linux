// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 Broadcom.
 */
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include "bcm_vk.h"

/* hwmon temp channel number */
enum temp_channel {
	TEMP_ASIC,
	TEMP_DDR0,
	TEMP_DDR1,
	TEMP_CHAN_END,
};

/* hwmon voltage channel number */
enum volt_channel {
	VOLT_1800mV,
	VOLT_3300mV,
	VOLT_CHAN_END,
};

/* only allow read */
#define HWMON_SYSFS_UMODE 0444

/* common scaling for all */
#define HWMON_SCALING_F 1000

/* forward declarations */
static umode_t bcm_vk_hwmon_is_visible(const void *data,
				       enum hwmon_sensor_types type,
				       u32 attr, int channel);
static int bcm_vk_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			     u32 attr, int channel, long *value);
static int bcm_vk_hwmon_read_labels(struct device *dev,
				    enum hwmon_sensor_types type,
				    u32 attr, int channel, const char **str);

/* hwmon attributes used */
static u32 hwmon_temp_config[] = {
	[TEMP_ASIC] = (HWMON_T_INPUT | HWMON_T_MIN | HWMON_T_MAX |
		       HWMON_T_CRIT | HWMON_T_MIN_ALARM | HWMON_T_MAX_ALARM |
		       HWMON_T_CRIT_ALARM | HWMON_T_LABEL),
	[TEMP_DDR0] = (HWMON_T_INPUT | HWMON_T_LABEL),
	[TEMP_DDR1] = (HWMON_T_INPUT | HWMON_T_LABEL),
	[TEMP_CHAN_END] = 0,
};

static u32 hwmon_volt_config[] = {
	[VOLT_1800mV] = (HWMON_I_INPUT | HWMON_I_LABEL),
	[VOLT_3300mV] = (HWMON_I_INPUT | HWMON_I_LABEL),
	[VOLT_CHAN_END] = 0,
};

static const struct hwmon_channel_info hwmon_temp_channel_info = {
	.type = hwmon_temp,
	.config = hwmon_temp_config,
};

static const struct hwmon_channel_info hwmon_volt_channel_info = {
	.type = hwmon_in,
	.config = hwmon_volt_config,
};

static const struct hwmon_channel_info *hwmon_info[] = {
	&hwmon_temp_channel_info,
	&hwmon_volt_channel_info,
	NULL,
};

static const struct hwmon_ops hwmon_ops = {
	.is_visible = bcm_vk_hwmon_is_visible,
	.read = bcm_vk_hwmon_read,
	.read_string = bcm_vk_hwmon_read_labels,
};

static const struct hwmon_chip_info hwmon_chip_info = {
	.ops = &hwmon_ops,
	.info = hwmon_info,
};

static umode_t bcm_vk_hwmon_is_visible(const void *data,
				       enum hwmon_sensor_types type,
				       u32 attr, int channel)
{
	umode_t umode;
	const struct bcm_vk *vk = data;

	dev_dbg(&vk->pdev->dev, "is_visible type %d attr 0x%x channel %d\n",
		type, attr, channel);

	umode = 0;
	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_min:
		case hwmon_temp_min_alarm:
		case hwmon_temp_max:
		case hwmon_temp_max_alarm:
		case hwmon_temp_crit:
		case hwmon_temp_crit_alarm:
			/* only ASIC sensor has level setting */
			if (channel != TEMP_ASIC)
				break;
			umode = HWMON_SYSFS_UMODE;
			break;
		case hwmon_temp_input:
		case hwmon_temp_label:
			/* all temp has measured value and label */
			umode = HWMON_SYSFS_UMODE;
			break;
		default:
			break;
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
		case hwmon_in_label:
			/* all voltages have measured value and label */
			umode = HWMON_SYSFS_UMODE;
			break;
		default:
			break;
		}
	default:
		break;
	}

	return umode;
}

static int bcm_vk_hwmon_temp_read(struct device *dev, u32 attr, int channel,
				  long *value)
{
	u32 reg;
	struct bcm_vk *vk = dev_get_drvdata(dev);

	*value = 0; /* default of 0 */
	if (attr == hwmon_temp_input) {
		static const int temp_shift[] = {
			[TEMP_ASIC] = BCM_VK_CPU_TEMP_SHIFT,
			[TEMP_DDR0] = BCM_VK_DDR0_TEMP_SHIFT,
			[TEMP_DDR1] = BCM_VK_DDR1_TEMP_SHIFT,
		};

		if (channel >= TEMP_CHAN_END)
			return -ERANGE;

		reg = vkread32(vk, BAR_0, BAR_CARD_TEMPERATURE);
		if (BCM_VK_INTF_IS_DOWN(reg))
			return -EPERM;

		*value = sign_extend32(reg >> temp_shift[channel],
				       BCM_VK_TEMP_FIELD_WIDTH - 1);
		*value *= HWMON_SCALING_F;
	} else {
		static const u32 table[] = {
			[hwmon_temp_min] = BCM_VK_LOW_TEMP_THRE_SHIFT,
			[hwmon_temp_max] = BCM_VK_HIGH_TEMP_THRE_SHIFT,
			[hwmon_temp_crit] = BCM_VK_TRAP_TEMP_THRE_SHIFT,
			[hwmon_temp_min_alarm] = ERR_LOG_LOW_TEMP_WARN,
			[hwmon_temp_max_alarm] = ERR_LOG_HIGH_TEMP_ERR,
			[hwmon_temp_crit_alarm] = ERR_LOG_THERMAL_TRAP,
		};

		/* only ASIC sensor supports range */
		if (channel != TEMP_ASIC)
			return -ERANGE;

		switch (attr) {
		case hwmon_temp_min:
		case hwmon_temp_max:
		case hwmon_temp_crit:
			reg = vkread32(vk, BAR_0, BAR_CARD_PWR_AND_THRE);
			if (BCM_VK_INTF_IS_DOWN(reg))
				return -EPERM;
			BCM_VK_EXTRACT_FIELD(*value, reg,
					     BCM_VK_PWR_AND_THRE_FIELD_MASK,
					     table[attr]);
			*value *= HWMON_SCALING_F;
			break;
		case hwmon_temp_min_alarm:
		case hwmon_temp_max_alarm:
		case hwmon_temp_crit_alarm:
			*value = (vk->peer_alert.flags & table[attr]) ? 1 : 0;
			break;
		default:
			return -EINVAL;
		}
	}
	return 0;
}

static int bcm_vk_hwmon_volt_read(struct device *dev, u32 attr, int channel,
				  long *value)
{
	u32 reg;
	static const int volt_shift[] = {
		[VOLT_1800mV] = 0,
		[VOLT_3300mV] = BCM_VK_3P3_VOLT_REG_SHIFT,
	};
	struct bcm_vk *vk = dev_get_drvdata(dev);

	if (channel >= VOLT_CHAN_END)
		return -ERANGE;

	reg = vkread32(vk, BAR_0, BAR_CARD_VOLTAGE);
	if (BCM_VK_INTF_IS_DOWN(reg))
		return -EPERM;

	*value = (reg >> volt_shift[channel]) & BCM_VK_VOLT_RAIL_MASK;
	return 0;
}

static int bcm_vk_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			     u32 attr, int channel, long *value)
{
	int ret = -EINVAL;

	dev_dbg(dev, "read type %d attr 0x%x channel %d\n",
		type, attr, channel);

	if (type == hwmon_temp)
		ret = bcm_vk_hwmon_temp_read(dev, attr, channel, value);
	else if (type == hwmon_in)
		ret = bcm_vk_hwmon_volt_read(dev, attr, channel, value);

	return ret;
}

static int bcm_vk_hwmon_read_labels(struct device *dev,
				    enum hwmon_sensor_types type,
				    u32 attr, int channel, const char **str)
{
	int ret = 0;
	static const char * const temp_labels[] = {
		[TEMP_ASIC] = "asic",
		[TEMP_DDR0] = "ddr0",
		[TEMP_DDR1] = "ddr1",
	};
	static const char * const volt_labels[] = {
		[VOLT_1800mV] = "1.8V",
		[VOLT_3300mV] = "3.3V",
	};

	dev_dbg(dev, "read_labels type %d attr 0x%x channel %d\n",
		type, attr, channel);

	if ((type == hwmon_temp) && (channel < TEMP_CHAN_END)) {
		*str = temp_labels[channel];
	} else if ((type == hwmon_in) && (channel < VOLT_CHAN_END)) {
		*str = volt_labels[channel];
	} else {
		*str = NULL;
		ret = -ERANGE;
	}
	return ret;
}

int bcm_vk_hwmon_init(struct bcm_vk *vk)
{
	int i;
	struct device *dev = &vk->pdev->dev;

	vk->hwmon_name = kstrdup(vk->miscdev.name, GFP_KERNEL);
	if (!vk->hwmon_name)
		return -ENOMEM;

	i = 0;
	while (vk->hwmon_name[i] != '\0') {
		/* replace unsupported chars with '.' */
		if (hwmon_is_bad_char(vk->hwmon_name[i]))
			vk->hwmon_name[i] = '.';
		i++;
	}
	vk->hwmon_dev = hwmon_device_register_with_info(dev,
							vk->hwmon_name,
							vk,
							&hwmon_chip_info,
							NULL);
	if (IS_ERR(vk->hwmon_dev)) {
		vk->hwmon_dev = NULL;
		kfree(vk->hwmon_name);
		dev_err(dev, "failed to register for hwmon\n");
		return -EINVAL;
	}
	return 0;
}

void bcm_vk_hwmon_deinit(struct bcm_vk *vk)
{
	if (vk->hwmon_dev) {
		hwmon_device_unregister(vk->hwmon_dev);
		kfree(vk->hwmon_name);
	}
}
