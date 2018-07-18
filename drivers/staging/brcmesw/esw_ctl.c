// SPDX-License-Identifier: (GPL-2.0 or BSD-3-Clause)
/*
 *  Copyright(c) 2018 Broadcom
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <uapi/linux/bcm_esw_ioctl.h>

#define ESW_CTL_DEVICE_NAME   "esw-ctl"

/* command and status register of the SRAB */
#define SRAB_CMDSTAT			0x2c
#define  SRAB_CMDSTAT_RST		BIT(2)
#define  SRAB_CMDSTAT_WRITE		BIT(1)
#define  SRAB_CMDSTAT_GORDYN	BIT(0)
#define  SRAB_CMDSTAT_PAGE		24
#define  SRAB_CMDSTAT_REG		16

/* high order word of write data to switch register */
#define SRAB_WD_H			0x30

/* low order word of write data to switch register */
#define SRAB_WD_L			0x34

/* high order word of read data from switch register */
#define SRAB_RD_H			0x38

/* low order word of read data from switch register */
#define SRAB_RD_L			0x3c

/* command and status register of the SRAB */
#define SRAB_CTRLS			0x40
#define  SRAB_CTRLS_RCAREQ		BIT(3)
#define  SRAB_CTRLS_RCAGNT		BIT(4)
#define  SRAB_CTRLS_SW_INIT_DONE	BIT(6)

#define SRAB_REQ_WAIT_LOOP		20
#define SRAB_OP_WAIT_LOOP		5

struct esw_reg_ops {
	int (*read)(u8 __iomem *regs, u8 page, u8 offset, u8 size, u64 *val);
	int (*write)(u8 __iomem *regs, u8 page, u8 offset, u8 size, u64 val);
};

struct esw_ctl_priv {
	struct platform_device *pdev;
	struct cdev c_dev;
	struct class *dev_class;
	dev_t devt;
	void __iomem *reg;
	struct esw_reg_ops *ops;
	/* locking ioctl access */
	struct mutex lock;
};

static int esw_srab_request_grant(u8 __iomem *regs)
{
	u32 ctrls;
	int i;

	ctrls = readl(regs + SRAB_CTRLS);
	ctrls |= SRAB_CTRLS_RCAREQ;
	writel(ctrls, regs + SRAB_CTRLS);

	for (i = 0; i < SRAB_REQ_WAIT_LOOP; i++) {
		ctrls = readl(regs + SRAB_CTRLS);
		if (ctrls & SRAB_CTRLS_RCAGNT)
			break;
		usleep_range(10, 100);
	}
	if (WARN_ON(i == SRAB_REQ_WAIT_LOOP))
		return -EIO;

	return 0;
}

static void esw_srab_release_grant(u8 __iomem *regs)
{
	u32 ctrls;

	ctrls = readl(regs + SRAB_CTRLS);
	ctrls &= ~SRAB_CTRLS_RCAREQ;
	writel(ctrls, regs + SRAB_CTRLS);
}

static int esw_srab_op(u8 __iomem *regs, u8 page, u8 reg, u32 op)
{
	int i;
	u32 cmdstat;

	/* set register address */
	cmdstat = (page << SRAB_CMDSTAT_PAGE) |
		  (reg << SRAB_CMDSTAT_REG) |
		  SRAB_CMDSTAT_GORDYN |
		  op;
	writel(cmdstat, regs + SRAB_CMDSTAT);

	/* check if operation completed */
	for (i = 0; i < SRAB_OP_WAIT_LOOP; ++i) {
		cmdstat = readl(regs + SRAB_CMDSTAT);
		if (!(cmdstat & SRAB_CMDSTAT_GORDYN))
			break;
		usleep_range(10, 100);
	}

	if (WARN_ON(i == SRAB_OP_WAIT_LOOP))
		return -EIO;

	return 0;
}

static int esw_ctl_open(struct inode *pnode, struct file *fp)
{
	/* look up device info for this device file */
	struct esw_ctl_priv *esw_ctl = container_of(pnode->i_cdev,
			struct esw_ctl_priv, c_dev);

	fp->private_data = esw_ctl;

	return 0;
}

static int esw_ctl_srab_read_reg(u8 __iomem *regs,
				 u8 page, u8 offset,
				 u8 size, u64 *val)
{
	int ret;

	ret = esw_srab_request_grant(regs);
	if (ret)
		goto err;

	ret = esw_srab_op(regs, page, offset, 0);
	if (ret)
		goto err;

	switch (size) {
	case 8:
	case 16:
	case 32:
		*val = readl(regs + SRAB_RD_L) & (BIT(size) - 1);
		break;
	case 64:
		*val = readl(regs + SRAB_RD_L);
		*val += (u64)readl(regs + SRAB_RD_H) << 32;
		break;
	default:
		ret = -EINVAL;
		break;
	}

err:
	esw_srab_release_grant(regs);

	return ret;
}

static int esw_ctl_srab_write_reg(u8 __iomem *regs,
				  u8 page, u8 reg,
				  u8 size, u64 val)
{
	int ret;

	ret = esw_srab_request_grant(regs);
	if (ret)
		goto err;

	switch (size) {
	case 8:
	case 16:
	case 32:
		writel((u32)val, regs + SRAB_WD_L);
		break;
	case 64:
		writel((u32)val, regs + SRAB_WD_L);
		writel((u32)(val >> 32), regs + SRAB_WD_H);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (!ret)
		ret = esw_srab_op(regs, page, reg, SRAB_CMDSTAT_WRITE);

err:
	esw_srab_release_grant(regs);

	return ret;
}

static int esw_ctl_read_reg(struct esw_ctl_priv *esw_ctl,
			    u8 page, u8 reg, u64 *data, u8 len)
{
	if (esw_ctl->ops->read)
		return esw_ctl->ops->read(esw_ctl->reg, page, reg, len, data);
	else
		return -EOPNOTSUPP;
}

static int esw_ctl_write_reg(struct esw_ctl_priv *esw_ctl,
			     u8 page, u8 reg, u64 data, u8 len)
{
	if (esw_ctl->ops->write)
		return esw_ctl->ops->write(esw_ctl->reg, page, reg, len, data);
	else
		return -EOPNOTSUPP;
}

static long esw_ctl_ioctl(struct file *filep,
			  unsigned int cmd,
			  unsigned long arg)
{
	struct esw_ctl_priv *esw_ctl = filep->private_data;
	struct esw_reg_data reg;
	int rc = 0;

	mutex_lock(&esw_ctl->lock);

	switch (cmd) {
	case SIOCESW_REG_READ:
		if (copy_from_user(&reg, (struct esw_reg_data *)arg,
				   sizeof(struct esw_reg_data))) {
			rc = -EFAULT;
			break;
		}

		/* Clear the data */
		reg.data = 0;
		rc = esw_ctl_read_reg(esw_ctl, reg.page, reg.offset,
				      &reg.data, reg.len);

		if (!rc)
			if (copy_to_user((struct esw_reg_data *)arg, &reg,
					 sizeof(struct esw_reg_data)))
				rc = -EFAULT;
		break;

	case SIOCESW_REG_WRITE:
		if (copy_from_user(&reg, (struct esw_reg_data *)arg,
				   sizeof(reg))) {
			rc = -EFAULT;
			break;
		}
		rc = esw_ctl_write_reg(esw_ctl, reg.page, reg.offset,
				       reg.data, reg.len);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&esw_ctl->lock);

	return rc;
}

#ifdef CONFIG_COMPAT
static long esw_ctl_compat_ioctl(struct file *file, unsigned int cmd,
				 unsigned long arg)
{
	return esw_ctl_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations esw_ctl_fops = {
	.owner = THIS_MODULE,
	.open			= esw_ctl_open,
	.unlocked_ioctl = esw_ctl_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= esw_ctl_compat_ioctl,
#endif
};

/* SRAB reg ops */
const struct esw_reg_ops esw_srab_ops = {
	.read = esw_ctl_srab_read_reg,
	.write = esw_ctl_srab_write_reg
};

static int esw_ctl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct esw_ctl_priv *esw_ctl;
	int ret = -ENOMEM;
	dev_t devt;
	struct device *retdev;

	esw_ctl = devm_kzalloc(dev, sizeof(*esw_ctl), GFP_KERNEL);
	if (!esw_ctl)
		return -ENOMEM;

	/* ESW memory mapped registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	esw_ctl->reg = ioremap(res->start, resource_size(res));
	if (IS_ERR(esw_ctl->reg)) {
		dev_err(&pdev->dev, "%s IO remap audioeav failed\n", __func__);
		return PTR_ERR(esw_ctl->reg);
	}

	/* create class for mdev auto create /dev node */
	esw_ctl->dev_class = class_create(THIS_MODULE, ESW_CTL_DEVICE_NAME);
	if (IS_ERR(esw_ctl->dev_class)) {
		ret = PTR_ERR(esw_ctl->dev_class);
		dev_err(&pdev->dev, "can't register static esw ctl class\n");
		goto err_iounmap;
	}

	ret = alloc_chrdev_region(&devt, 0, 1, ESW_CTL_DEVICE_NAME);
	if (ret) {
		dev_err(&pdev->dev,
			"%s Alloc char device region failed\n", __func__);
		goto err_class_delete;
	}

	/* create device */
	cdev_init(&esw_ctl->c_dev, &esw_ctl_fops);

	ret = cdev_add(&esw_ctl->c_dev, devt, 1);
	if (ret) {
		dev_err(dev, "Failed adding esw cdev file\n");
		goto err_unreg_chardev;
	}

	retdev = device_create(esw_ctl->dev_class, NULL,
			       devt, NULL, ESW_CTL_DEVICE_NAME);
	if (IS_ERR(retdev)) {
		dev_err(&pdev->dev, "can't create esw device\n ");
		ret = PTR_ERR(retdev);
		goto err_cdev_delete;
	}

	mutex_init(&esw_ctl->lock);
	esw_ctl->pdev = pdev;
	esw_ctl->devt = devt;
	esw_ctl->ops = (struct esw_reg_ops *)&esw_srab_ops;
	platform_set_drvdata(pdev, esw_ctl);

	return 0;

err_cdev_delete:
	cdev_del(&esw_ctl->c_dev);

err_unreg_chardev:
	unregister_chrdev_region(devt, 1);

err_class_delete:
	device_destroy(esw_ctl->dev_class, esw_ctl->devt);

err_iounmap:
	iounmap(esw_ctl->reg);

	return ret;
}

static int esw_ctl_remove(struct platform_device *pdev)
{
	struct esw_ctl_priv *esw_ctl = platform_get_drvdata(pdev);

	device_destroy(esw_ctl->dev_class, esw_ctl->devt);
	cdev_del(&esw_ctl->c_dev);
	unregister_chrdev_region(esw_ctl->devt, 1);
	class_destroy(esw_ctl->dev_class);
	platform_set_drvdata(pdev, NULL);
	iounmap(esw_ctl->reg);
	mutex_destroy(&esw_ctl->lock);

	return 0;
}

static const struct of_device_id esw_ctl_of_match[] = {
	{ .compatible = "brcm,esw-ctl", },
	{},
};
MODULE_DEVICE_TABLE(of, esw_ctl_of_match);

static struct platform_driver esw_ctl_driver = {
	.driver = {
		.name = "esw-ctl",
		.of_match_table = esw_ctl_of_match,
	},
	.probe    = esw_ctl_probe,
	.remove   = esw_ctl_remove,
};
module_platform_driver(esw_ctl_driver);

MODULE_AUTHOR("Arun Parameswaran <arun.parameswaran@broadcom.com>");
MODULE_DESCRIPTION("Broadcom Ethernet Switch Control Interface driver");
MODULE_LICENSE("GPL v2");
