/*
 * Copyright (C) 2016 Broadcom
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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

/*
 * # of tries for OTP Status. The time to execute a command varies. The slowest
 * commands are writes which also vary based on the # of bits turned on. Writing
 * 0xffffffff takes ~3800 us.
 */
#define OTPC_RETRIES                 5000

/* Sequence to enable OTP program */
#define OTPC_PROG_EN_SEQ             { 0xf, 0x4, 0x8, 0xd }

/* OTPC Commands */
#define OTPC_CMD_READ                0x0
#define OTPC_CMD_OTP_PROG_ENABLE     0x2
#define OTPC_CMD_OTP_PROG_DISABLE    0x3
#define OTPC_CMD_PROGRAM             0xA

/* OTPC Status Bits */
#define OTPC_STAT_CMD_DONE           BIT(1)
#define OTPC_STAT_PROG_OK            BIT(2)

/* OTPC register definition */
#define OTPC_MODE_REG_OFFSET         0x0
#define OTPC_MODE_REG_OTPC_MODE      0
#define OTPC_COMMAND_OFFSET          0x4
#define OTPC_COMMAND_COMMAND_WIDTH   6
#define OTPC_CMD_START_OFFSET        0x8
#define OTPC_CMD_START_START         0
#define OTPC_CPU_STATUS_OFFSET       0xc
#define OTPC_CPU_DATA_OFFSET         0x10
#define OTPC_CPUADDR_REG_OFFSET      0x28
#define OTPC_CPUADDR_REG_OTPC_CPU_ADDRESS_WIDTH 16
#define OTPC_CPU_WRITE_REG_OFFSET    0x2c

#define OTPC_CMD_MASK  (BIT(OTPC_COMMAND_COMMAND_WIDTH) - 1)
#define OTPC_ADDR_MASK (BIT(OTPC_CPUADDR_REG_OTPC_CPU_ADDRESS_WIDTH) - 1)

struct otpc_priv {
	struct device       *dev;
	void __iomem        *base;
	struct nvmem_config *config;
};

static inline void set_command(void __iomem *base, u32 command)
{
	writel(command & OTPC_CMD_MASK, base + OTPC_COMMAND_OFFSET);
}

static inline void set_cpu_address(void __iomem *base, u32 addr)
{
	writel(addr & OTPC_ADDR_MASK, base + OTPC_CPUADDR_REG_OFFSET);
}

static inline void set_start_bit(void __iomem *base)
{
	writel(1 << OTPC_CMD_START_START, base + OTPC_CMD_START_OFFSET);
}

static inline void reset_start_bit(void __iomem *base)
{
	writel(0, base + OTPC_CMD_START_OFFSET);
}

static inline void write_cpu_data(void __iomem *base, u32 value)
{
	writel(value, base + OTPC_CPU_WRITE_REG_OFFSET);
}

static int poll_cpu_status(void __iomem *base, u32 value)
{
	u32 status;
	u32 retries;

	for (retries = 0; retries < OTPC_RETRIES; retries++) {
		status = readl(base + OTPC_CPU_STATUS_OFFSET);
		if (status & value)
			break;
		udelay(1);
	}
	if (retries == OTPC_RETRIES)
		return -EAGAIN;

	return 0;
}

static int enable_ocotp_program(void __iomem *base)
{
	static const u32 vals[] = OTPC_PROG_EN_SEQ;
	int i;
	int ret;

	/* Write the magic sequence to enable programming */
	set_command(base, OTPC_CMD_OTP_PROG_ENABLE);
	for (i = 0; i < ARRAY_SIZE(vals); i++) {
		write_cpu_data(base, vals[i]);
		set_start_bit(base);
		ret = poll_cpu_status(base, OTPC_STAT_CMD_DONE);
		reset_start_bit(base);
		if (ret)
			return ret;
	}

	return poll_cpu_status(base, OTPC_STAT_PROG_OK);
}

static int disable_ocotp_program(void __iomem *base)
{
	int ret;

	set_command(base, OTPC_CMD_OTP_PROG_DISABLE);
	set_start_bit(base);
	ret = poll_cpu_status(base, OTPC_STAT_PROG_OK);
	reset_start_bit(base);

	return ret;
}

static int bcm_28nm_otpc_read(void *context, unsigned int offset, void *val,
	size_t bytes)
{
	struct otpc_priv *priv = context;
	u32 *buf = val;
	u32 bytes_read;
	u32 address = offset / priv->config->word_size;
	int ret;

	for (bytes_read = 0; bytes_read < bytes; bytes_read += sizeof(*buf)) {
		set_command(priv->base, OTPC_CMD_READ);
		set_cpu_address(priv->base, address++);
		set_start_bit(priv->base);
		ret = poll_cpu_status(priv->base, OTPC_STAT_CMD_DONE);
		if (ret) {
			dev_err(priv->dev, "otp read error: 0x%x", ret);
			return -EIO;
		}
		*buf++ = readl(priv->base + OTPC_CPU_DATA_OFFSET);
		reset_start_bit(priv->base);
	}

	return 0;
}

static int bcm_28nm_otpc_write(void *context, unsigned int offset, void *val,
	size_t bytes)
{
	struct otpc_priv *priv = context;
	u32 *buf = val;
	u32 bytes_written;
	u32 address = offset / priv->config->word_size;
	int ret;

	if (offset % priv->config->word_size)
		return -EINVAL;

	ret = enable_ocotp_program(priv->base);
	if (ret)
		return -EIO;

	for (bytes_written = 0; bytes_written < bytes;
		bytes_written += sizeof(*buf)) {
		set_command(priv->base, OTPC_CMD_PROGRAM);
		set_cpu_address(priv->base, address++);
		write_cpu_data(priv->base, *buf++);
		set_start_bit(priv->base);
		ret = poll_cpu_status(priv->base, OTPC_STAT_CMD_DONE);
		reset_start_bit(priv->base);
		if (ret) {
			dev_err(priv->dev, "otp write error: 0x%x", ret);
			return -EIO;
		}
	}

	disable_ocotp_program(priv->base);

	return 0;
}

static struct nvmem_config bcm_28nm_otpc_nvmem_config = {
	.name = "bcm-28nm-ocotp",
	.read_only = false,
	.word_size = 4,
	.stride = 4,
	.owner = THIS_MODULE,
	.reg_read = bcm_28nm_otpc_read,
	.reg_write = bcm_28nm_otpc_write,
};

static const struct of_device_id bcm_28nm_otpc_dt_ids[] = {
	{ .compatible = "brcm,28nm-ocotp" },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm_28nm_otpc_dt_ids);

static int bcm_28nm_otpc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn = dev->of_node;
	struct resource *res;
	struct otpc_priv *priv;
	struct nvmem_device *nvmem;
	int err;
	u32 num_words;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Get OTP base address register. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base)) {
		dev_err(dev, "unable to map I/O memory\n");
		return PTR_ERR(priv->base);
	}

	/* Enable CPU access to OTPC. */
	writel(readl(priv->base + OTPC_MODE_REG_OFFSET) |
		BIT(OTPC_MODE_REG_OTPC_MODE),
		priv->base + OTPC_MODE_REG_OFFSET);
	reset_start_bit(priv->base);

	/* Read size of memory in words. */
	err = of_property_read_u32(dn, "brcm,ocotp-size", &num_words);
	if (err) {
		dev_err(dev, "size parameter not specified\n");
		return -EINVAL;
	} else if (num_words == 0) {
		dev_err(dev, "size must be > 0\n");
		return -EINVAL;
	}

	bcm_28nm_otpc_nvmem_config.size = 4 * num_words;
	bcm_28nm_otpc_nvmem_config.dev = dev;
	bcm_28nm_otpc_nvmem_config.priv = priv;
	priv->config = &bcm_28nm_otpc_nvmem_config;
	nvmem = nvmem_register(&bcm_28nm_otpc_nvmem_config);
	if (IS_ERR(nvmem)) {
		dev_err(dev, "error registering nvmem config\n");
		return PTR_ERR(nvmem);
	}

	platform_set_drvdata(pdev, nvmem);

	return 0;
}

static int bcm_28nm_otpc_remove(struct platform_device *pdev)
{
	struct nvmem_device *nvmem = platform_get_drvdata(pdev);

	return nvmem_unregister(nvmem);
}

static struct platform_driver bcm_28nm_otpc_driver = {
	.probe	= bcm_28nm_otpc_probe,
	.remove	= bcm_28nm_otpc_remove,
	.driver = {
		.name	= "brcm-28nm-otpc",
		.of_match_table = bcm_28nm_otpc_dt_ids,
	},
};
module_platform_driver(bcm_28nm_otpc_driver);

MODULE_DESCRIPTION("Broadcom 28NM OTPC driver");
MODULE_LICENSE("GPL v2");
