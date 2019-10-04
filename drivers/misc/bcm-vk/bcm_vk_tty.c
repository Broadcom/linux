// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018-2019 Broadcom.
 */

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include "bcm_vk.h"

/* TTYVK base offset is 0x30000 into BAR1 */
#define BAR1_TTYVK_BASE_OFFSET	0x300000
/* Each TTYVK channel (TO or FROM) is 0x10000 */
#define BAR1_TTYVK_CHAN_OFFSET	0x100000
/* Each TTYVK channel has TO and FROM, hence the * 2 */
#define BAR1_TTYVK_BASE(index)	(BAR1_TTYVK_BASE_OFFSET + \
				 ((index) * BAR1_TTYVK_CHAN_OFFSET * 2))
/* TO TTYVK channel base comes before FROM for each index */
#define TO_TTYK_BASE(index)	BAR1_TTYVK_BASE(index)
#define FROM_TTYK_BASE(index)	(BAR1_TTYVK_BASE(index) + \
				 BAR1_TTYVK_CHAN_OFFSET)

struct bcm_vk_tty_chan {
	uint32_t reserved;
	uint32_t size;
	uint32_t wr;
	uint32_t rd;
	uint32_t *data;
};

#define VK_BAR_CHAN(v, DIR, e)	((v)->DIR##_offset \
				 + offsetof(struct bcm_vk_tty_chan, e))
#define VK_BAR_CHAN_SIZE(v, DIR)	VK_BAR_CHAN(v, DIR, size)
#define VK_BAR_CHAN_WR(v, DIR)		VK_BAR_CHAN(v, DIR, wr)
#define VK_BAR_CHAN_RD(v, DIR)		VK_BAR_CHAN(v, DIR, rd)
#define VK_BAR_CHAN_DATA(v, DIR, off)	(VK_BAR_CHAN(v, DIR, data) + off)

/* Poll every 1/10 of second - temp hack till we use MSI interrupt */
#define SERIAL_TIMER_VALUE (HZ / 10)

static void bcm_vk_tty_poll(struct timer_list *t)
{
	struct bcm_vk *vk = from_timer(vk, t, serial_timer);
	struct bcm_vk_tty *vktty;
	int card_status;
	int ready_mask;
	int count = 0;
	unsigned char c;
	int i;
	int wr;

	spin_lock(&vk->timer_lock);

	card_status = vkread32(vk, BAR_0, BAR_CARD_STATUS);

	for (i = 0; i < BCM_VK_NUM_TTY; i++) {
		/* Check the card status that the tty channel is ready */
		ready_mask = BIT(i);
		if ((card_status & ready_mask) == 0)
			continue;

		vktty = &vk->tty[i];

		/* Fetch the wr offset in buffer from VK */
		wr = vkread32(vk, BAR_1, VK_BAR_CHAN_WR(vktty, from));
		if (wr >= vktty->from_size) {
			dev_err(&vk->pdev->dev,
				"ERROR: poll ttyVK%d wr:0x%x > 0x%x\n",
				i, wr, vktty->from_size);
			/* Need to signal and close device in this case */
			goto err_exit;
		}

		/*
		 * Simple read of circular buffer and
		 * insert into tty flip buffer
		 */
		while (vk->tty[i].rd != wr) {
			c = vkread8(vk, BAR_1,
				    VK_BAR_CHAN_DATA(vktty, from, vktty->rd));
			vktty->rd++;
			if (vktty->rd >= vktty->from_size)
				vktty->rd = 0;
			tty_insert_flip_char(&vktty->port, c, TTY_NORMAL);
			count++;
		}
	}

	if (count) {
		tty_flip_buffer_push(&vktty->port);

		/* Update read offset from shadow register to card */
		vkwrite32(vk, vktty->rd, BAR_1, VK_BAR_CHAN_RD(vktty, from));
	}

	mod_timer(&vk->serial_timer, jiffies + SERIAL_TIMER_VALUE);
err_exit:
	spin_unlock(&vk->timer_lock);
}

static int bcm_vk_tty_open(struct tty_struct *tty, struct file *file)
{
	int card_status;
	int ready_mask;
	struct bcm_vk *vk;
	struct bcm_vk_tty *vktty;
	int index;

	/* initialize the pointer in case something fails */
	tty->driver_data = NULL;

	vk = (struct bcm_vk *)dev_get_drvdata(tty->dev);
	index = tty->index;

	if (index > BCM_VK_NUM_TTY)
		return -EINVAL;

	vktty = &vk->tty[index];

	vktty->to_offset = TO_TTYK_BASE(index);
	vktty->from_offset = FROM_TTYK_BASE(index);

	/* Do not allow tty device to be opened if tty on card not ready */
	card_status = vkread32(vk, BAR_0, BAR_CARD_STATUS);
	if (card_status == -1)
		return -1;

	ready_mask = BIT(index);
	if ((card_status & ready_mask) == 0)
		return -1;

	/*
	 * Get shadow registers of the buffer sizes and the "to" write offset
	 * and "from" read offset
	 */
	vktty->to_size = vkread32(vk, BAR_1, VK_BAR_CHAN_SIZE(vktty, to));
	vktty->wr = vkread32(vk, BAR_1,  VK_BAR_CHAN_WR(vktty, to));
	vktty->from_size = vkread32(vk, BAR_1, VK_BAR_CHAN_SIZE(vktty, from));
	vktty->rd = vkread32(vk, BAR_1,  VK_BAR_CHAN_RD(vktty, from));

	spin_lock_bh(&vk->timer_lock);
	if (tty->count == 1) {
		timer_setup(&vk->serial_timer, bcm_vk_tty_poll, 0);
		mod_timer(&vk->serial_timer, jiffies + SERIAL_TIMER_VALUE);
	}
	spin_unlock_bh(&vk->timer_lock);

	return 0;
}

static void bcm_vk_tty_close(struct tty_struct *tty, struct file *file)
{
	struct bcm_vk *vk;

	vk = (struct bcm_vk *)dev_get_drvdata(tty->dev);

	spin_lock_bh(&vk->timer_lock);
	if (tty->count == 1)
		del_timer_sync(&vk->serial_timer);
	spin_unlock_bh(&vk->timer_lock);
}

static int bcm_vk_tty_write(struct tty_struct *tty,
			    const unsigned char *buffer,
			    int count)
{
	int index;
	struct bcm_vk *vk;
	struct bcm_vk_tty *vktty;
	int i;
	int retval;

	index = tty->index;
	vk = (struct bcm_vk *)dev_get_drvdata(tty->dev);
	vktty = &vk->tty[index];

	/* Simple write each byte to circular buffer */
	for (i = 0; i < count; i++) {
		vkwrite8(vk,
			 buffer[i],
			 BAR_1,
			 VK_BAR_CHAN_DATA(vktty, to, vktty->wr));
		vktty->wr++;
		if (vktty->wr >= vktty->to_size)
			vktty->wr = 0;
	}
	/* Update write offset from shadow register to card */
	/* TODO: Need to add write to doorbell here */
	vkwrite32(vk, vktty->wr, BAR_1, VK_BAR_CHAN_WR(vktty, to));

	retval = count;

	return retval;
}

static int bcm_vk_tty_write_room(struct tty_struct *tty)
{
	int room;
	struct bcm_vk *vk;
	struct bcm_vk_tty *vktty;

	vk = (struct bcm_vk *)dev_get_drvdata(tty->dev);
	vktty = &vk->tty[tty->index];

	/*
	 * Calculate how much room is left in the device
	 * Just return the size -1 of buffer.  We could care about
	 * overflow but don't at this point.
	 */
	room = vktty->to_size - 1;

	return room;
}

static const struct tty_operations serial_ops = {
	.open = bcm_vk_tty_open,
	.close = bcm_vk_tty_close,
	.write = bcm_vk_tty_write,
	.write_room = bcm_vk_tty_write_room,
};

int bcm_vk_tty_init(struct bcm_vk *vk, char *name)
{
	int i;
	int err;
	unsigned long flags;
	struct tty_driver *tty_drv;
	struct device *dev = &vk->pdev->dev;

	/* allocate the tty driver */
	flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_drv = tty_alloc_driver(BCM_VK_NUM_TTY, flags);

	if (IS_ERR(tty_drv)) {
		err = PTR_ERR(tty_drv);
		goto err_exit;
	}
	/* Save struct tty_driver for uninstalling the device */
	vk->tty_drv = tty_drv;

	/* initialize the tty driver */
	tty_drv->driver_name = KBUILD_MODNAME;
	tty_drv->name = kstrdup(name, GFP_KERNEL);
	if (!tty_drv->name) {
		err = -ENOMEM;
		goto err_put_tty_driver;
	}
	tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
	tty_drv->subtype = SERIAL_TYPE_NORMAL;
	tty_drv->init_termios = tty_std_termios;
	tty_set_operations(tty_drv, &serial_ops);

	/* register the tty driver */
	err = tty_register_driver(tty_drv);
	if (err) {
		dev_err(dev, "tty_register_driver failed\n");
		goto err_kfree_tty_name;
	}

	spin_lock_init(&vk->timer_lock);

	for (i = 0; i < BCM_VK_NUM_TTY; i++) {
		struct device *tty_dev;

		tty_port_init(&vk->tty[i].port);
		tty_dev = tty_port_register_device(&vk->tty[i].port,
						   tty_drv,
						   i,
						   dev);
		dev_set_drvdata(tty_dev, vk);

		if (IS_ERR(tty_dev)) {
			int j;

			for (j = 0; j > i; j++)
				tty_port_unregister_device(&vk->tty[j].port,
							   tty_drv,
							   j);
			goto err_tty_unregister_driver;
		}
	}

	return 0;

err_tty_unregister_driver:
	tty_unregister_driver(tty_drv);

err_kfree_tty_name:
	kfree(tty_drv->name);
	tty_drv->name = NULL;

err_put_tty_driver:
	put_tty_driver(tty_drv);

err_exit:
	return err;
}

void bcm_vk_tty_exit(struct bcm_vk *vk)
{
	int i;

	for (i = 0; i < BCM_VK_NUM_TTY; ++i) {
		tty_port_unregister_device(&vk->tty[i].port,
					   vk->tty_drv,
					   i);
		tty_port_destroy(&vk->tty[i].port);
	}
	tty_unregister_driver(vk->tty_drv);
	put_tty_driver(vk->tty_drv);

	kfree(vk->tty_drv->name);
	vk->tty_drv->name = NULL;
}
