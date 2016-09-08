/*
* Copyright (C) 2016 Broadcom
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License as
* published by the Free Software Foundation version 2.
*
* This program is distributed "as is" WITHOUT ANY WARRANTY of any
* kind, whether express or implied; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include "bcm_iproc_ftm.h"

static void iproc_ftm_fifo_clear(struct iproc_ftm *ftm)
{
	u32 entries;
	int i, j;
	u32 dummy;

	for (i = 0; i < FTM_MAX_CHNL; i++) {
		regmap_read(ftm->regmap, FTM_STATUS0 + i * 4, &entries);
		entries &= FTM_STATUS0_ENTRIES_MASK;

		for (j = 0; j < entries; j++)
			regmap_read(ftm->regmap, FTM_DATA_REG0 + i * 4, &dummy);
	}
}

static void iproc_ftm_ena_intr(struct iproc_ftm *ftm, bool enable)
{

	regmap_write(ftm->regmap, FTM_INTR_STATUS, INTR_MASK);

	if (enable) {
		iproc_ftm_fifo_clear(ftm);

		/* unmask flex timer interrupts */
		regmap_write(ftm->regmap, FTM_INTR_MASK, INTR_MASK);

		/* unmask ftm modules interrupt from TSC controller*/
		regmap_update_bits(ftm->regmap, TSC_INTR_MASK_REG,
				TSC_FTM_INTR_MASK, TSC_FTM_INTR_MASK);
	} else {
		/* mask ftm modules interrupt from TSC controller*/
		regmap_update_bits(ftm->regmap, TSC_INTR_MASK_REG,
				TSC_FTM_INTR_MASK, 0);

		/* mask flex timer interrupts */
		regmap_write(ftm->regmap, FTM_INTR_MASK, 0);
	}
}

static void iproc_ftm_ena_timeout(struct iproc_ftm *ftm, bool enable)
{
	if (enable)
		regmap_update_bits(ftm->regmap, FLEX_TIMER_CTRL1,
						TIMEOUT_EN, TIMEOUT_EN);
	else
		regmap_update_bits(ftm->regmap, FLEX_TIMER_CTRL1,
							TIMEOUT_EN, 0);
}

static void iproc_ftm_init(struct iproc_ftm *ftm)
{
	u32 value;

	/* div 100M/8 = 12.5M */
	regmap_write(ftm->regmap, CLK_PRESCALE, CLK_PRESCALE_DIVIDER);

	/* both edge & timeout */
	value = FTM_EDGE_HL | FTM_EDGE_LH;
	value &= ~CONFIG_TIMEOUT_MASK;
	value |= TIMEOUT_VALUE << CONFIG_TIMEOUT;
	regmap_write(ftm->regmap, FLEX_TIMER_CTRL1, value);

	/* water mark set to 32 and enable glitch filter */
	value = WATERMARK_VALUE |
		(WATERMARK_VALUE << WATERMARK_CHAN2) |
		(WATERMARK_VALUE << WATERMARK_CHAN3);
	regmap_write(ftm->regmap, FTM_CTRL2, value);

		/* enable ftm */
	regmap_update_bits(ftm->regmap, TSC_REG_CTRL2,
				TSC_FTM_ENA, TSC_FTM_ENA);

	iproc_ftm_ena_timeout(ftm, true);
}

static irqreturn_t iproc_ftm_isr(int irq, void *drv_ctx)
{
	int i, ret;
	u32 status;
	u32 entries;
	u32 v;
	struct ftm_chnl_desc *ch;
	struct iproc_ftm *ftm = drv_ctx;

	regmap_read(ftm->regmap, FTM_INTR_STATUS, &status);
	regmap_write(ftm->regmap, FTM_INTR_STATUS, status);

	if ((status & INTR_MASK) == 0)
		return IRQ_NONE;

	if (status & INTR_SOS)
		ftm->sosc += 1;

	if (status & INTR_TIMEOUT)
		ftm->timeoutc += 1;

	for (i = 0; i < FTM_MAX_CHNL; i++) {
		ch = &ftm->ch[i];
		regmap_read(ftm->regmap, FTM_STATUS0 + i * 4, &entries);
		entries &= FTM_STATUS0_ENTRIES_MASK;
		dev_dbg(ftm->dev, "Avail data word entries:%X\n", entries);

		if (ch->ts_cnt >= MAX_TS_ENTRIES) {
			dev_err(ftm->dev, "Dumped %d ts for ch :%d\n",
							ch->ts_cnt, i);
			print_hex_dump(KERN_ERR, "ftm ts:", DUMP_PREFIX_NONE,
				16, 2, ch->ts_buf, ch->ts_cnt * 2, false);
			memset(ch->ts_buf, 0, MAX_TS_BUFF_SIZE);
			ch->ts_cnt = 0;
		}

		while (entries-- && (ch->ts_cnt < MAX_TS_ENTRIES)) {
			regmap_read(ftm->regmap, FTM_DATA_REG0 + i * 4, &v);
			ch->ts_buf[ch->ts_cnt] = (u16)v;
			ch->ts_cnt++;
		}

		if ((ftm->sosc == ftm->timeoutc) && ch->ts_cnt) {
			/* decode ts for track data */
			ret = ftm_decode(ch->ts_buf, ch->ts_cnt, i,
						ch->track_data, &ch->data_cnt);
			if (ret) {
				dev_err(ftm->dev, "TS decoding failed\n");
			} else {
				ch->drdy = true;
				wake_up_interruptible(&ch->wq);
			}
		}
	}
	return IRQ_HANDLED;
}

static int iproc_ftm_open(struct inode *inode, struct file *filp)
{
	struct iproc_ftm *ftm = container_of(inode->i_cdev,
		struct iproc_ftm, cdev);
	int minor = iminor(inode);

	dev_dbg(ftm->dev, "%s: minor: %d\n", __func__, minor);

	mutex_lock(&ftm->ulock);
	if (!(ftm->users & FTM_USER_MASK(minor))) {
		ftm->users |= FTM_USER_MASK(minor);
	} else {
		dev_info(ftm->dev, "ftm%d is already opened\n", minor);
		mutex_unlock(&ftm->ulock);
		return -EBUSY;
	}
	filp->private_data = ftm;
	mutex_unlock(&ftm->ulock);

	return 0;
}

static int iproc_ftm_release(struct inode *inode, struct file *filp)
{
	struct iproc_ftm *ftm = filp->private_data;
	int minor = iminor(inode);

	dev_dbg(ftm->dev, "%s: minor: %d\n", __func__, minor);

	mutex_lock(&ftm->ulock);
	ftm->users &= ~(FTM_USER_MASK(minor));
	mutex_unlock(&ftm->ulock);

	return 0;
}

static ssize_t iproc_ftm_read(struct file *filp, char __user *buf,
					size_t count, loff_t *offp)
{
	int minor;
	int ret = 0;
	struct iproc_ftm *ftm = filp->private_data;

	minor = iminor(filp->f_path.dentry->d_inode);

	if (wait_event_interruptible(ftm->ch[minor].wq,
					ftm->ch[minor].drdy != true))
		return -EFAULT;

	if (copy_to_user(buf, ftm->ch[minor].track_data,
					ftm->ch[minor].data_cnt))
		return -EFAULT;

	dev_dbg(ftm->dev, "copied bytes: %d\n", ftm->ch[minor].data_cnt);

	ret = ftm->ch[minor].data_cnt;

	ftm->ch[minor].data_cnt = 0;
	ftm->ch[minor].drdy = false;

	return ret;
}

static const struct file_operations iproc_ftm_cdev_fops = {
	.open = iproc_ftm_open,
	.release = iproc_ftm_release,
	.read = iproc_ftm_read,
};

static int iproc_ftm_probe(struct platform_device *pdev)
{
	struct iproc_ftm *ftm;
	int ret = 0;
	int i = 0;
	struct ftm_chnl_desc *ch;
	struct device *dev = &pdev->dev;

	ftm = devm_kzalloc(dev, sizeof(*ftm), GFP_KERNEL);
	if (ftm == NULL)
		return -ENOMEM;

	mutex_init(&ftm->ulock);
	ftm->dev = dev;
	platform_set_drvdata(pdev, ftm);

	/* obtain regmap handle */
	ftm->regmap = syscon_regmap_lookup_by_phandle(dev->of_node,
						"flextimer_syscon");
	if (IS_ERR(ftm->regmap)) {
		dev_err(dev, "failed to get handle for tsc syscon\n");
		return PTR_ERR(ftm->regmap);
	}

	ftm->irq = platform_get_irq(pdev, 0);
	if (ftm->irq < 0) {
		dev_err(dev, "Failed to parse interrupt\n");
		ret = -ENODEV;
	}

	iproc_ftm_ena_intr(ftm, false);
	/* init irq */
	ret = devm_request_irq(dev, ftm->irq, iproc_ftm_isr, IRQF_SHARED,
							pdev->name, ftm);
	if (ret) {
		dev_err(dev, "Failed to register interrupt\n");
		return ret;
	}

	ftm->ftm_clk = devm_clk_get(dev, "ftm_clk");
	if (IS_ERR(ftm->ftm_clk)) {
		dev_err(dev, "%s Failed getting clock ftm_clk\n", __func__);
		return PTR_ERR(ftm->ftm_clk);
	}

	/* alloc ts buff space */
	for (i = 0; i < FTM_MAX_CHNL; i++) {
		ch = &ftm->ch[i];
		ch->ts_buf = devm_kzalloc(dev, MAX_TS_BUFF_SIZE, GFP_KERNEL);
		if (ch->ts_buf == NULL)
			return -ENOMEM;

		ch->track_data = devm_kzalloc(dev, MAX_TRACK_DATA_SIZE,
								GFP_KERNEL);
		if (ch->track_data == NULL)
			return -ENOMEM;

		init_waitqueue_head(&ch->wq);
	}

	/* create device */
	ret = alloc_chrdev_region(&ftm->devno, 0, FTM_MAX_CHNL, "ft");
	if (ret) {
		dev_err(dev, "failed to alloc chardev region\n");
		return ret;
	}
	cdev_init(&ftm->cdev, &iproc_ftm_cdev_fops);
	ret = cdev_add(&ftm->cdev, ftm->devno, FTM_MAX_CHNL);
	if (ret) {
		dev_err(dev, "Failed adding ft cdev file\n");
		goto free_char_region;
	}

	/* create class for mdev auto create node /dev/ftm */
	ftm->class = class_create(THIS_MODULE, "ftm");
	if (IS_ERR(ftm->class)) {
		dev_err(dev, "Failed creating class\n");
		ret = PTR_ERR(ftm->class);
		goto del_cdev;
	}
	for (i = 0; i < FTM_MAX_CHNL; i++)
		device_create(ftm->class, dev,
			MKDEV(MAJOR(ftm->devno), i), ftm, "ftm%d", i);

	ret = clk_prepare_enable(ftm->ftm_clk);
	if (ret) {
		dev_err(dev, "clk_prepare_enable failed %d\n", ret);
		goto rm_class;
	}

	/* hw init */
	iproc_ftm_init(ftm);
	iproc_ftm_ena_intr(ftm, true);

	dev_info(dev, "ftm probe done successfully\n");
	return ret;

rm_class:
	for (i = 0; i < FTM_MAX_CHNL; i++)
		device_destroy(ftm->class, MKDEV(MAJOR(ftm->devno), i));

	class_destroy(ftm->class);

del_cdev:
	cdev_del(&ftm->cdev);
free_char_region:
	unregister_chrdev_region(ftm->devno, FTM_MAX_CHNL);
	return ret;
}

static const struct of_device_id iproc_ftm_of_match[] = {
	{.compatible = "brcm,iproc-ftm", },
	{ },
};
MODULE_DEVICE_TABLE(of, iproc_ftm_of_match);

static struct platform_driver brcm_iproc_ftm_drv = {
	.driver = {
		.name = "brcmftm",
		.of_match_table = iproc_ftm_of_match,
	},
	.probe	= iproc_ftm_probe,
};
module_platform_driver(brcm_iproc_ftm_drv);

MODULE_DESCRIPTION("Broadcom Flextimer HW driver");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
