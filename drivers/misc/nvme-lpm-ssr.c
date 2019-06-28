// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2018 Broadcom
 */
#include <asm/barrier.h>
#include <asm/spinlock.h>
#include <linux/bcm_iproc_mailbox.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/nvme-lpm-ssr.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/smp.h>
#include <linux/uaccess.h>
#include <uapi/asm-generic/errno.h>

/*
 * Pass SSR command to CRMU
 * Populate SSR Structure in shared memory which would be flashed to
 * NVM memory via CRMU
 * Response - none
 */
#define M0_IPC_M0_CMD_SSR_MSG 0x82

struct nvme_lpm {
	struct device *dev;
	struct mbox_client client;
	struct mbox_chan *mbox_chan;
	void *shared_mem;
	spinlock_t shmlock;
	void *shared_nvme_data;
	struct gpio_desc *gpiod;
	int irq;
	struct class *class;
	struct cdev cdev;
	struct device *class_dev;
	dev_t devt;
	unsigned int ssr_state_armed;
	spinlock_t live_backup_lock;
	uint8_t live_backup_state;
};

static struct nvme_lpm_drv_ops *nvme_drv_ops;

static int iproc_mbox_send_msg(struct nvme_lpm *nvme_lpm,
				struct ssr_wrapper *lwrap)
{
	int ret, i;
	struct iproc_mbox_msg msg;
	struct ssr_wrapper *wrap = nvme_lpm->shared_mem;

	spin_lock(&nvme_lpm->shmlock);
	memcpy(wrap, lwrap, sizeof(*lwrap));
	msg.cmd = M0_IPC_M0_CMD_SSR_MSG;
	msg.param = 0;
	msg.wait_ack = false;

	wrap->crmu_maia_valid_response = 0x0;
	wrap->maia_crmu_valid_request = MAIA_CRMU_VALID_REQUEST;
	dsb(sy);
	ret = mbox_send_message(nvme_lpm->mbox_chan, &msg);
	if (ret < 0) {
		dev_err(nvme_lpm->dev, "Failed to send mbox msg ret:%d\n", ret);
		goto fail;
	}
	ret = 0;

	mbox_client_txdone(nvme_lpm->mbox_chan, 0);

	/* Wait for completion */
	for (i = 0; i < MAX_CRMU_RESPONSE_TIMEOUT; i++)
		if (wrap->ssr_cmd_id != 0)
			udelay(100);
		else
			break;

	if (i == MAX_CRMU_RESPONSE_TIMEOUT) {
		dev_err(nvme_lpm->dev, "CRMU response timed out\n");
		ret = -ETIMEDOUT;
		goto fail;
	}

	if (wrap->crmu_maia_valid_response != CRMU_MAIA_VALID_RESPONSE) {
		ret = -ENODATA;
		goto fail;
	}

	/*
	 * Data needs to be written back from shared memory to local SSR
	 * in GET_SSR case
	 */
	if (lwrap->ssr_cmd_id == NVME_LPM_CMD_GET_SSR)
		memcpy(&lwrap->ssr, &wrap->ssr, SSR_SIZE);

fail:
	spin_unlock(&nvme_lpm->shmlock);
	return ret;
}

static int nvme_lpm_get_ssr(struct nvme_lpm *nvme_lpm, void __user *argp)
{
	int ret = 0;
	struct ssr_wrapper wrap = {0,};

	/* send msg to CRMU */
	wrap.ssr_cmd_id = NVME_LPM_CMD_GET_SSR;
	ret = iproc_mbox_send_msg(nvme_lpm, &wrap);
	if (ret)
		goto fail;

	/* copy SSR */
	if (copy_to_user(argp, &wrap.ssr, SSR_SIZE)) {
		dev_err(nvme_lpm->dev, "Failed to copy to user space\n");
		ret = -EFAULT;
	}
fail:
	return ret;
}

static int nvme_lpm_erase_ssr(struct nvme_lpm *nvme_lpm, void __user *argp)
{
	struct ssr_wrapper wrap = {0,};

	wrap.ssr_cmd_id = NVME_LPM_CMD_ERASE_SSR;
	wrap.ssr.state = SSR_STATE_INIT;

	/* send msg to CRMU */
	return iproc_mbox_send_msg(nvme_lpm, &wrap);
}

static inline void update_live_backup_state(struct nvme_lpm *nvme_lpm,
					    enum live_backup_state state)
{
	/* Saving the live backup state for local usage */
	nvme_lpm->live_backup_state = state;

	/*
	 * Continuously share the live backup state with CRMU without writing
	 * it to SSR, this is done to avoid LPM switch delay
	 */
	nvme_drv_ops->update_live_backup_state(nvme_drv_ops->ctxt, state);
}

static int nvme_lpm_arm_ssr(struct nvme_lpm *nvme_lpm, void __user *argp)
{
	struct ssr_wrapper wrap = {0,};
	struct armed_ssr armed_ssr;
	int ret = -EINVAL;

#ifndef CONFIG_LPM_SSR_DISABLE_NVME
	if (!nvme_drv_ops)
		goto out;

	ret = nvme_drv_ops->nvme_destroy_backup_io_queues(nvme_drv_ops->ctxt);
	if (ret) {
		dev_err(nvme_lpm->dev, "Failed to prepare nvme for backup\n");
		goto out;
	}
#endif

	if (copy_from_user(&armed_ssr, argp, sizeof(struct armed_ssr))) {
		dev_err(nvme_lpm->dev, "Failed to copy armed ssr from user\n");
		ret = -EFAULT;
		goto out;
	}

#ifndef CONFIG_LPM_SSR_DISABLE_NVME
	ret = nvme_drv_ops->nvme_build_backup_io_queues(nvme_drv_ops->ctxt,
						    armed_ssr.memory_address,
						    armed_ssr.disk_address,
						    armed_ssr.length, true,
						    nvme_lpm->shared_nvme_data);

	if (ret) {
		dev_err(nvme_lpm->dev, "Failed to build backup io queues\n");
		goto out;
	}
#endif
	update_live_backup_state(nvme_lpm, LIVE_BACKUP_NOT_ACTIVE);

	wrap.ssr_cmd_id = NVME_LPM_CMD_ARM_SSR;
	wrap.ssr.state = SSR_STATE_ARM;
	/* initialize live backup state */
	wrap.ssr.live_backup_state = LIVE_BACKUP_NOT_ACTIVE;
	wrap.ssr.sequence = armed_ssr.sequence;
	wrap.ssr.nvme_backup_offset = armed_ssr.disk_address;
	wrap.ssr.nvme_backup_length = armed_ssr.length;

	/* send msg to CRMU */
	ret = iproc_mbox_send_msg(nvme_lpm, &wrap);
	if (ret)
		nvme_lpm->ssr_state_armed = false;
	else
		nvme_lpm->ssr_state_armed = true;

out:
	return ret;
}

static int nvme_lpm_disarm_cmd(struct nvme_lpm *nvme_lpm, void __user *argp)
{
	struct ssr_wrapper wrap = {0,};
	struct disarmed_ssr disarmed_ssr;
	int ret = -EINVAL;

#ifndef CONFIG_LPM_SSR_DISABLE_NVME
	if (!nvme_drv_ops)
		goto out;

	ret = nvme_drv_ops->nvme_destroy_backup_io_queues(nvme_drv_ops->ctxt);
	if (ret) {
		dev_err(nvme_lpm->dev, "Failed to destroy nvme io queues\n");
		goto out;
	}
#endif

	ret = copy_from_user(&disarmed_ssr, argp, sizeof(struct disarmed_ssr));
	if (ret) {
		dev_err(nvme_lpm->dev, "Failed to copy disarmed info\n");
		goto out;
	}
	wrap.ssr_cmd_id = NVME_LPM_CMD_DISARM_SSR;
	wrap.ssr.state = SSR_STATE_DISARM;
	wrap.ssr.sequence = disarmed_ssr.sequence;

	/* send msg to CRMU */
	ret = iproc_mbox_send_msg(nvme_lpm, &wrap);
	if (!ret)
		nvme_lpm->ssr_state_armed = false;
out:
	return ret;
}

static int nvme_lpm_trigger_ssr(struct nvme_lpm *nvme_lpm)
{
	uint8_t live_backup_state = nvme_lpm->live_backup_state;

	console_silent();

#ifndef CONFIG_LPM_SSR_DISABLE_NVME
	if (nvme_lpm->ssr_state_armed == true) {
		/* Do not initiate the data transfer as already in progress */
		if (nvme_drv_ops && live_backup_state == LIVE_BACKUP_NOT_ACTIVE)
			nvme_drv_ops->nvme_initiate_xfers(nvme_drv_ops->ctxt);
	}
	else
		machine_halt();
#endif

	smp_send_stop();
	set_cpu_online(smp_processor_id(), false);
	cpu_die();

	return 0;
}

static int nvme_live_backup_state_ssr(struct nvme_lpm *nvme_lpm,
				      enum live_backup_state new_state)
{
	struct ssr_wrapper wrap = {0,};

	nvme_lpm->live_backup_state = new_state;
	/* Only live backup state can be updated using following command */
	wrap.ssr_cmd_id = NVME_LPM_CMD_LIVE_BACKUP_SSR;
	wrap.ssr.live_backup_state = new_state;

	/* send msg to CRMU */
	return iproc_mbox_send_msg(nvme_lpm, &wrap);
}

#ifndef CONFIG_LPM_SSR_DISABLE_NVME
static int nvme_lpm_read_test(struct nvme_lpm *nvme_lpm, void __user *argp)
{
	struct armed_ssr armed_ssr;
	int ret = -EINVAL;

	if (!nvme_drv_ops)
		goto out;

	ret = nvme_drv_ops->nvme_destroy_backup_io_queues(nvme_drv_ops->ctxt);
	if (ret) {
		dev_err(nvme_lpm->dev, "Failed to prepare nvme for backup\n");
		goto out;
	}

	if (copy_from_user(&armed_ssr, argp, sizeof(struct armed_ssr))) {
		dev_err(nvme_lpm->dev, "Failed to copy armed ssr from user\n");
		ret = -EFAULT;
		goto out;
	}

	/*
	 * Read back case is for testing purpose only, SSR does not get
	 * accessed for this case
	 */
	ret = nvme_drv_ops->nvme_build_backup_io_queues(nvme_drv_ops->ctxt,
						    armed_ssr.memory_address,
						    armed_ssr.disk_address,
						    armed_ssr.length, false,
						    nvme_lpm->shared_nvme_data);

	if (ret) {
		nvme_lpm->ssr_state_armed = false;
		dev_err(nvme_lpm->dev, "Failed to build read-back io queues\n");
	} else
		nvme_lpm->ssr_state_armed = true;

out:
	return ret;
}

static int nvme_lpm_poll_xfers_from_ap(struct nvme_lpm *nvme_lpm)
{
	int ret = -EINVAL;
	unsigned long flags;
	uint8_t state;

	if (!nvme_drv_ops || nvme_lpm->ssr_state_armed != true)
		goto out;

	/* Initiate transfers(live backup) */
	spin_lock_irqsave(&nvme_lpm->live_backup_lock, flags);

	update_live_backup_state(nvme_lpm, LIVE_BACKUP_XFER_INIT);
	nvme_drv_ops->nvme_initiate_xfers(nvme_drv_ops->ctxt);
	update_live_backup_state(nvme_lpm, LIVE_BACKUP_IN_PROGRESS);

	spin_unlock_irqrestore(&nvme_lpm->live_backup_lock, flags);

	/*
	 * During polling, no need to explicitly take a lock here as each queue
	 * gets polled with a queue lock for given polling loop
	 */
	ret = nvme_drv_ops->nvme_poll_xfers(nvme_drv_ops->ctxt);
	if (ret) {
		/* We do not want interrupt while updating the backup state */
		spin_lock_irqsave(&nvme_lpm->live_backup_lock, flags);
		state = LIVE_BACKUP_FAIL;
		update_live_backup_state(nvme_lpm, state);
		spin_unlock_irqrestore(&nvme_lpm->live_backup_lock, flags);
		dev_err(nvme_lpm->dev, "Transfer not completed");
		goto update_ssr;
	}
	dev_err(nvme_lpm->dev, "Transfer done");

	/* During flush command, interrupt cannot be served */
	spin_lock_irqsave(&nvme_lpm->live_backup_lock, flags);

	update_live_backup_state(nvme_lpm, LIVE_BACKUP_FLUSH);
	ret = nvme_drv_ops->nvme_send_flush_cmd(nvme_drv_ops->ctxt);
	if (ret) {
		dev_err(nvme_lpm->dev, "Flush not completed");
		state = LIVE_BACKUP_FAIL;
	} else {
		dev_info(nvme_lpm->dev, "Flush done");
		state = LIVE_BACKUP_DONE;
	}
	update_live_backup_state(nvme_lpm, state);

	spin_unlock_irqrestore(&nvme_lpm->live_backup_lock, flags);

update_ssr:
	ret = nvme_live_backup_state_ssr(nvme_lpm, state);
out:
	return ret;
}
#endif

/* IOCTL interface through character device */
static int nvme_dev_open(struct inode *inode, struct file *file)
{
	file->private_data = container_of(inode->i_cdev,
					  struct nvme_lpm, cdev);

	return 0;
}

static long nvme_dev_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	struct nvme_lpm *nvme_lpm = file->private_data;
	void __user *argp = (void __user *)arg;
	int ret = -1;

	switch (cmd) {
	case NVME_LPM_IOCTL_GET_SSR:
		ret = nvme_lpm_get_ssr(nvme_lpm, argp);
		break;
	case NVME_LPM_IOCTL_ERASE_SSR:
		ret = nvme_lpm_erase_ssr(nvme_lpm, argp);
		break;
	case NVME_LPM_IOCTL_ARM_SSR:
		ret = nvme_lpm_arm_ssr(nvme_lpm, argp);
		break;
	case NVME_LPM_IOCTL_DISARM_SSR:
		ret = nvme_lpm_disarm_cmd(nvme_lpm, argp);
		break;
	case NVME_LPM_IOCTL_TRIGGER_SSR:
		ret = nvme_lpm_trigger_ssr(nvme_lpm);
		break;
#ifndef CONFIG_LPM_SSR_DISABLE_NVME
	case NVME_LPM_IOCTL_READ:
		ret = nvme_lpm_read_test(nvme_lpm, argp);
		break;
	case NVME_LPM_IOCTL_AP_POLL:
		ret = nvme_lpm_poll_xfers_from_ap(nvme_lpm);
		break;
#endif
	default:
		return -ENOENT;
	}
	return ret;
}

static irqreturn_t iproc_gpio_isr(int irq, void *drv_ctx)
{
	struct nvme_lpm *nvme_lpm = (struct nvme_lpm *)drv_ctx;

	nvme_lpm_trigger_ssr(nvme_lpm);

	return IRQ_HANDLED;
}

int register_nvme_lpm_ops(void *nvme_lpm_drv_ops)
{
	int ret = -EINVAL;

	if (!nvme_drv_ops) {
		nvme_drv_ops = nvme_lpm_drv_ops;
		ret = 0;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(register_nvme_lpm_ops);

void unregister_nvme_lpm_ops(void)
{
	nvme_drv_ops = NULL;
}
EXPORT_SYMBOL_GPL(unregister_nvme_lpm_ops);

static const struct file_operations nvme_dev_fops = {
	.owner		= THIS_MODULE,
	.open		= nvme_dev_open,
	.unlocked_ioctl	= nvme_dev_ioctl,
	.compat_ioctl	= nvme_dev_ioctl,
};

static int nvme_lpm_probe(struct platform_device *pdev)
{
	int ret;
	struct nvme_lpm *nvme_lpm;
	struct device *dev = &pdev->dev;
	struct resource *res, reg;
	struct device_node *np;

	nvme_lpm = devm_kzalloc(dev, sizeof(struct nvme_lpm), GFP_KERNEL);
	if (!nvme_lpm) {
		ret = -ENOMEM;
		goto out;
	}

	nvme_lpm->dev = dev;
	spin_lock_init(&nvme_lpm->shmlock);
	spin_lock_init(&nvme_lpm->live_backup_lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nvme_lpm->shared_mem = devm_memremap(dev, res->start,
					resource_size(res), MEMREMAP_WB);
	if (IS_ERR(nvme_lpm->shared_mem)) {
		dev_err(dev, "failed to get io address\n");
		ret = PTR_ERR(nvme_lpm->shared_mem);
		goto out;
	}

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		dev_err(dev, "No memory-region specified\n");
		ret = -ENOMEM;
		goto out;
	}

	ret = of_address_to_resource(np, 0, &reg);
	if (ret) {
		dev_err(dev, "No memory address assigned to the region\n");
		ret = -ENOMEM;
		goto out_put_node;
	}

	nvme_lpm->shared_nvme_data = devm_memremap(dev, reg.start,
					resource_size(&reg), MEMREMAP_WB);
	if (IS_ERR(nvme_lpm->shared_nvme_data)) {
		dev_err(dev, "Failed to remap pci_lpm_mem region\n");
		ret = PTR_ERR(nvme_lpm->shared_nvme_data);
		goto out_put_node;
	}

	nvme_lpm->gpiod = devm_gpiod_get(dev, "intrpt", GPIOD_IN);
	if (IS_ERR(nvme_lpm->gpiod)) {
		dev_err(dev, "Failed to get interrupt gpio\n");
		ret = PTR_ERR(nvme_lpm->gpiod);
		goto out_put_node;
	}

	nvme_lpm->irq = gpiod_to_irq(nvme_lpm->gpiod);
	if (nvme_lpm->irq < 0) {
		dev_err(dev, "Failed to get interrupt gpio IRQ\n");
		ret = nvme_lpm->irq;
		goto out_put_node;
	}

	/* init irq */
	ret = devm_request_irq(dev, nvme_lpm->irq, iproc_gpio_isr,
			       IRQF_TRIGGER_RISING,
			       dev_name(dev), nvme_lpm);
	if (ret) {
		dev_err(dev, "Failed to register interrupt\n");
		goto out_put_node;
	}

	/* Request mailbox channel. */
	nvme_lpm->client.dev          = &pdev->dev;
	nvme_lpm->client.tx_block     = false;
	nvme_lpm->client.tx_tout      = 1;
	nvme_lpm->client.knows_txdone = true;
	nvme_lpm->mbox_chan = mbox_request_channel(&nvme_lpm->client, 0);
	if (IS_ERR(nvme_lpm->mbox_chan)) {
		dev_err(dev, "unable to get mbox channel\n");
		ret = PTR_ERR(nvme_lpm->mbox_chan);
		goto out_put_node;
	}
	dev_dbg(dev, "Mailbox registration done\n");
	ret = alloc_chrdev_region(&nvme_lpm->devt, 0, 1, "nvme-lpm");
	if (ret) {
		dev_err(dev, "cannot get add chrdev, err:%d\n", ret);
		goto out_mbox;
	}
	cdev_init(&nvme_lpm->cdev, &nvme_dev_fops);
	ret = cdev_add(&nvme_lpm->cdev, nvme_lpm->devt, 1);
	if (ret) {
		dev_err(dev, "cannot get add chrdev, err:%d\n", ret);
		goto out_region;
	}
	dev_dbg(dev, "Character device created\n");

	nvme_lpm->class = class_create(THIS_MODULE, "nvme-lpm");
	if (IS_ERR(nvme_lpm->class)) {
		ret = PTR_ERR(nvme_lpm->class);
		goto out_cdev;
	}
	dev_dbg(dev, "Class creation done\n");

	nvme_lpm->class_dev = device_create(nvme_lpm->class, nvme_lpm->dev,
				  nvme_lpm->devt, nvme_lpm, "nvme-lpm");
	if (IS_ERR(nvme_lpm->class_dev)) {
		dev_err(dev, "failed to create device\n");
		ret = PTR_ERR(nvme_lpm->class_dev);
		goto out_class;
	}
	dev_info(dev, "device got added\n");
	return ret;
out_class:
	class_destroy(nvme_lpm->class);
out_cdev:
	cdev_del(&nvme_lpm->cdev);
out_region:
	unregister_chrdev_region(nvme_lpm->devt, 1);
out_mbox:
	mbox_free_channel(nvme_lpm->mbox_chan);
out_put_node:
	of_node_put(np);
out:
	return ret;
}

static int nvme_lpm_remove(struct platform_device *pdev)
{
	struct nvme_lpm *nvme_lpm = platform_get_drvdata(pdev);

	class_destroy(nvme_lpm->class);
	cdev_del(&nvme_lpm->cdev);
	unregister_chrdev_region(nvme_lpm->devt, 1);
	mbox_free_channel(nvme_lpm->mbox_chan);

	return 0;
}

static const struct of_device_id nvme_lpm_of_match[] = {
	{ .compatible = "brcm,nvme-lpm-data-backup", },
	{}
};
MODULE_DEVICE_TABLE(of, nvme_lpm_of_match);

struct platform_driver nvme_lpm_driver = {
	.driver = {
		.name = "nvme-lpm-data-backup",
		.of_match_table = nvme_lpm_of_match,
	},
	.probe = nvme_lpm_probe,
	.remove = nvme_lpm_remove,
};
module_platform_driver(nvme_lpm_driver);
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom nvme lpm driver");
MODULE_LICENSE("GPL v2");
