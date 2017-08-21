/*
 * Copyright (C) 2017 Broadcom
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
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/slab.h>

#define BSTI_CONTROL_OFFSET     0x0
#define BSTI_COMMAND_OFFSET     0x4

#define IHOST_VDDC_MAX_DIFF	40000
#define MAX_VOLTAGE             990000

/* Vsens Voltage divider 1.56 */
#define VOL_DIV(vol)            ((vol)*100/156)
/* Vref value 0.5v */
#define VOL_REF                 500000
/* Step value 3.125mV */
#define ONE_STEP_VALUE          3125

#define BSTI_CMD(sb, op, pa, ra, ta, step) \
	((((sb) & 0x3) << 30) | (((op) & 0x3) << 28) | \
	(((pa) & 0x1F) << 23) | (((ra) & 0x1F) << 18) | \
	(((ta) & 0x3) << 16) | (step))

#define STEP_VALUE(vol)	\
	((((((VOL_DIV(vol)) - VOL_REF) / ONE_STEP_VALUE) & 0xff) << 8) | 4)

#define BSTI_CONTROL_VAL        0x81
#define BSTI_CONTROL_BSY        BIT(8)

#define PHYID_IHOST03   2
#define PHYID_IHOST12   3
#define PHYID_VDDC      6

#define PHY_REG0        0x0
#define PHY_REGC        0xc

#define UPDATE_POS_EDGE(set)	(0x560 | ((set) << 1))
#define BSTI_TA                 2
#define BSTI_WRITE              1

struct bcm_swreg {
	struct dentry *dentry;
	void __iomem    *reg;
};

static int bcm_swreg_poll(void __iomem *off, uint32_t val)
{
	uint32_t data;
	int retry;

	retry = 1000;
	do {
		data = readl(off);
		if ((data & val) != val)
			return 0;
		udelay(10);
	} while (--retry > 0);
	return -ETIMEDOUT;
}

static int bcm_swreg_set_voltage(struct bcm_swreg *bs, uint32_t ihost,
				 uint32_t vddc)
{
	uint32_t step, data;
	int ret;

	step = STEP_VALUE(ihost);

	pr_info("iHost voltage = %d and step ihost = %d\n",
			ihost, step);
	writel(BSTI_CONTROL_VAL, bs->reg + BSTI_CONTROL_OFFSET);
	data = BSTI_CMD(1, BSTI_WRITE, PHYID_IHOST03, PHY_REGC, BSTI_TA, step);
	writel(data, bs->reg + BSTI_COMMAND_OFFSET);
	ret = bcm_swreg_poll(bs->reg + BSTI_CONTROL_OFFSET, BSTI_CONTROL_BSY);
	if (ret)
		return ret;

	data = BSTI_CMD(1, BSTI_WRITE, PHYID_IHOST03, PHY_REG0, BSTI_TA,
			UPDATE_POS_EDGE(1));
	writel(data, bs->reg + BSTI_COMMAND_OFFSET);
	ret = bcm_swreg_poll(bs->reg + BSTI_CONTROL_OFFSET, BSTI_CONTROL_BSY);
	if (ret)
		return ret;

	data = BSTI_CMD(1, BSTI_WRITE, PHYID_IHOST03, PHY_REG0, BSTI_TA,
			UPDATE_POS_EDGE(0));
	writel(data, bs->reg + BSTI_COMMAND_OFFSET);
	ret = bcm_swreg_poll(bs->reg + BSTI_CONTROL_OFFSET, BSTI_CONTROL_BSY);
	if (ret)
		return ret;

	writel(BSTI_CONTROL_VAL, bs->reg + BSTI_CONTROL_OFFSET);
	data = BSTI_CMD(1, BSTI_WRITE, PHYID_IHOST12, PHY_REGC, BSTI_TA, step);
	writel(data, bs->reg + BSTI_COMMAND_OFFSET);
	ret = bcm_swreg_poll(bs->reg + BSTI_CONTROL_OFFSET, BSTI_CONTROL_BSY);
	if (ret)
		return ret;

	data = BSTI_CMD(1, BSTI_WRITE, PHYID_IHOST12, PHY_REG0, BSTI_TA,
			UPDATE_POS_EDGE(1));
	writel(data, bs->reg + BSTI_COMMAND_OFFSET);
	ret = bcm_swreg_poll(bs->reg + BSTI_CONTROL_OFFSET, BSTI_CONTROL_BSY);
	if (ret)
		return ret;

	data = BSTI_CMD(1, BSTI_WRITE, PHYID_IHOST12, PHY_REG0, BSTI_TA,
			UPDATE_POS_EDGE(0));
	writel(data, bs->reg + BSTI_COMMAND_OFFSET);
	ret = bcm_swreg_poll(bs->reg + BSTI_CONTROL_OFFSET, BSTI_CONTROL_BSY);
	if (ret)
		return ret;

	pr_info("iHost voltage update to %d\n", ihost);

	step = STEP_VALUE(vddc);
	pr_info("VDDC voltage = %d and step_vddc = %d\n",
			vddc, step);
	writel(BSTI_CONTROL_VAL, bs->reg + BSTI_CONTROL_OFFSET);
	data = BSTI_CMD(1, BSTI_WRITE, PHYID_VDDC, PHY_REGC, BSTI_TA, step);
	writel(data, bs->reg + BSTI_COMMAND_OFFSET);
	ret = bcm_swreg_poll(bs->reg + BSTI_CONTROL_OFFSET, BSTI_CONTROL_BSY);
	if (ret)
		return ret;

	data = BSTI_CMD(1, BSTI_WRITE, PHYID_VDDC, PHY_REG0, BSTI_TA,
			UPDATE_POS_EDGE(1));
	writel(data, bs->reg + BSTI_COMMAND_OFFSET);
	ret = bcm_swreg_poll(bs->reg + BSTI_CONTROL_OFFSET, BSTI_CONTROL_BSY);
	if (ret)
		return ret;

	data = BSTI_CMD(1, BSTI_WRITE, PHYID_VDDC, PHY_REG0, BSTI_TA,
			UPDATE_POS_EDGE(0));
	writel(data, bs->reg + BSTI_COMMAND_OFFSET);
	ret = bcm_swreg_poll(bs->reg + BSTI_CONTROL_OFFSET, BSTI_CONTROL_BSY);
	if (ret)
		return ret;

	pr_info("VDDC voltage update to %d\n", ihost);

	return 0;
}
static ssize_t bcm_swreg_write(struct file *file, const char __user *user_buf,
			 size_t count, loff_t *ppos)
{
	u32 ihost_val;
	char *start, *s, buf[32];
	u32 buf_size, vddc_val;
	int ret;

	struct bcm_swreg *bs = file->f_inode->i_private;

	buf_size = min(count, (size_t)(sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size)) {
		pr_err("Failed to copy from user\n");
		return -EFAULT;
	}

	buf[buf_size] = 0;
	pr_info("buffer :%s\n", buf);
	start = buf;

	s = strsep(&start, " ");
	if (!s || !*s)
		return -EINVAL;
	ret = kstrtouint(s, 0, &ihost_val);
	if (ret)
		return -EINVAL;
	pr_info("ihost_val: %u\n", ihost_val);

	s = strsep(&start, " ");
	if (!s || !*s)
		return -EINVAL;
	ret = kstrtouint(s, 0, &vddc_val);
	if (ret)
		return -EINVAL;
	pr_info("vddc_val: %u\n", vddc_val);

	if (ihost_val == 0 || vddc_val == 0 ||
		abs(ihost_val - vddc_val) > IHOST_VDDC_MAX_DIFF) {
		pr_err("iHost and VDDC voltages should be non-zero\n");
		pr_err("iHost, VDDC diff should be less than 40000uV\n");
		return -EINVAL;
	}

	if (ihost_val > MAX_VOLTAGE || vddc_val > MAX_VOLTAGE) {
		pr_err("Voltage should be less than 990000\n");
		return -EINVAL;
	}

	ret = bcm_swreg_set_voltage(bs, ihost_val, vddc_val);
	if (ret)
		return ret;

	return buf_size;
}

static ssize_t
bcm_swreg_read(struct file *file, char __user *user_buf,
		size_t count, loff_t *ppos)
{
	char *buff;
	int desc = 0;
	ssize_t ret;

	buff = kzalloc(1024, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	desc += sprintf(buff + desc, "command to change iHost vddc voltage\n");
	desc += sprintf(buff + desc,
			"echo \"<iHost_uV> <vddc_uV>\" > bcmswreg\n");
	desc += sprintf(buff + desc,
			"voltage should be less than 990000uV\n");
	desc += sprintf(buff + desc,
			"iHost and vddc diff should be less than 40000uV\n");
	ret = simple_read_from_buffer(user_buf, count, ppos, buff, desc);
	kfree(buff);

	return ret;
}

static const struct file_operations bcm_swreg_fops = {
	.write = bcm_swreg_write,
	.read = bcm_swreg_read,
};

static int bcm_swreg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct bcm_swreg *bs;

	bs = devm_kzalloc(dev, sizeof(*bs), GFP_KERNEL);
	if (!bs)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	bs->reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(bs->reg)) {
		dev_err(&pdev->dev, "%s IO remap failed\n", __func__);
		return PTR_ERR(bs->reg);
	}

	bs->dentry = debugfs_create_file("bcmswreg", 0644, NULL,
							bs, &bcm_swreg_fops);
	if (!bs->dentry) {
		dev_err(&pdev->dev, "Failed to create debugfs swreg file\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, bs);
	dev_info(&pdev->dev, "bcm swreg init done\n");
	return 0;
}

static int bcm_swreg_remove(struct platform_device *pdev)
{
	struct bcm_swreg *bs = platform_get_drvdata(pdev);

	debugfs_remove(bs->dentry);
	platform_set_drvdata(pdev, NULL);
	dev_info(&pdev->dev, "bcm swreg removed\n");

	return 0;
}

static const struct of_device_id bcm_swreg_of_match[] = {
	{ .compatible = "brcm,swreg", },
	{},
};
MODULE_DEVICE_TABLE(of, bcm_swreg_of_match);

static struct platform_driver bcm_swreg_driver = {
	.driver = {
		.name = "bcm-swreg",
		.of_match_table = bcm_swreg_of_match,
	},
	.probe    = bcm_swreg_probe,
	.remove   = bcm_swreg_remove,
};
module_platform_driver(bcm_swreg_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BCM SWREG Driver");
