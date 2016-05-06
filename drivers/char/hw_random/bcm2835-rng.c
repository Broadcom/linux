/**
 * Copyright (c) 2010-2012 Broadcom. All rights reserved.
 * Copyright (c) 2013 Lubomir Rintel
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License ("GPL")
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/printk.h>

#define RNG_CTRL	0x0
#define RNG_STATUS	0x4
#define RNG_DATA	0x8
#define RNG_INT_MASK	0x10

/* enable rng */
#define RNG_RBGEN	0x1

/* the initial numbers generated are "less random" so will be discarded */
#define RNG_WARMUP_COUNT 0x40000

#define RNG_VALID_SHIFT		24
#define RNG_INT_OFF		0x1

static int bcm2835_rng_read(struct hwrng *rng, void *buf, size_t max,
			       bool wait)
{
	void __iomem *rng_base = (void __iomem *)rng->priv;
	u32 max_words = max/sizeof(u32);
	u32 num_words, count;

	while ((readl(rng_base + RNG_STATUS) >> RNG_VALID_SHIFT) == 0) {
		if (!wait)
			return 0;
		cpu_relax();
	}

	num_words = (readl(rng_base + RNG_STATUS) >> RNG_VALID_SHIFT);
	if (num_words > max_words)
		num_words = max_words;

	for (count = 0; count < num_words; count++) {
		*(u32 *)buf = readl(rng_base + RNG_DATA);
		buf += sizeof(u32);
	}

	return (num_words * sizeof(u32));
}

static struct hwrng bcm2835_rng_ops = {
	.name	= "bcm2835",
	.read	= bcm2835_rng_read,
};

static int bcm2835_rng_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	void __iomem *rng_base;
	int err;
	u32 val = 0;

	/* map peripheral */
	rng_base = of_iomap(np, 0);
	if (!rng_base) {
		dev_err(dev, "failed to remap rng regs");
		return -ENODEV;
	}
	bcm2835_rng_ops.priv = (unsigned long)rng_base;

	/* mask the interrupt */
	val = readl(rng_base + RNG_INT_MASK);
	val |= RNG_INT_OFF;
	writel(val, rng_base + RNG_INT_MASK);

	/* set warm-up count & enable */
	writel(RNG_WARMUP_COUNT, rng_base + RNG_STATUS);
	writel(RNG_RBGEN, rng_base + RNG_CTRL);

	/* register driver */
	err = hwrng_register(&bcm2835_rng_ops);
	if (err) {
		dev_err(dev, "hwrng registration failed\n");
		iounmap(rng_base);
	} else
		dev_info(dev, "hwrng registered\n");

	return err;
}

static int bcm2835_rng_remove(struct platform_device *pdev)
{
	void __iomem *rng_base = (void __iomem *)bcm2835_rng_ops.priv;

	/* disable rng hardware */
	writel(0, rng_base + RNG_CTRL);

	/* unregister driver */
	hwrng_unregister(&bcm2835_rng_ops);
	iounmap(rng_base);

	return 0;
}

static const struct of_device_id bcm2835_rng_of_match[] = {
	{ .compatible = "brcm,bcm2835-rng", },
	{},
};
MODULE_DEVICE_TABLE(of, bcm2835_rng_of_match);

static struct platform_driver bcm2835_rng_driver = {
	.driver = {
		.name = "bcm2835-rng",
		.of_match_table = bcm2835_rng_of_match,
	},
	.probe		= bcm2835_rng_probe,
	.remove		= bcm2835_rng_remove,
};
module_platform_driver(bcm2835_rng_driver);

MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_DESCRIPTION("BCM2835 Random Number Generator (RNG) driver");
MODULE_LICENSE("GPL v2");
