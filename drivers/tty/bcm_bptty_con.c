// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 Broadcom.
 */

#include <linux/circ_buf.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#define BPTTY_CHANNELS		2	/* receive & xmit channels per port */
#define BPTTY_XMIT_CH		0	/* output: send to memory */
#define BPTTY_RECV_CH		1	/* input: read from memory */
#define BPTTY_CH_ADDR(a)	(offsetof(struct bptty_chnl, a))
#define BPTTY_PORT_NUM		1	/* support for one tty device only */
#define BPTTY_PORT_NAME		"ttyBP"	/* Broadcom PCIe console */

struct bptty_chnl {
	unsigned int reserved;
	unsigned int size;
	unsigned int wr;
	unsigned int rd;
	unsigned int *data;
};

struct bptty_state {
	struct tty_struct *tty;
	struct tty_port port;
	struct semaphore sem;
	struct timer_list timer;
	struct circ_buf xmits[BPTTY_CHANNELS];
	struct bptty_chnl *ch[BPTTY_CHANNELS];
	struct resource *resources[BPTTY_CHANNELS];
	int open_count;
};

static struct bptty_state state_info;
static struct tty_driver *bptty_tty_driver;

static const struct of_device_id bptty_of_match[] = {
	{ .compatible = "brcm,bcm-bptty-console" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bptty_of_match);

/* Poll every 1/10 of second - temp hack till MSI-X/mailbox interrupts */
#define DELAY_TIME		(HZ / 10)

static void receive_poll(struct timer_list *unused)
{
	struct bptty_state *state = &state_info;
	struct tty_port *port = &state->port;
	struct bptty_chnl *recv_ch = state->ch[BPTTY_RECV_CH];
	struct circ_buf *recv = &state->xmits[BPTTY_RECV_CH];
	unsigned char c;
	unsigned int count, i;

	if (!recv->buf)
		return;

	/* reset head to new write index */
	recv->head = recv_ch->wr;
	count = CIRC_CNT(recv->head, recv->tail, recv_ch->size);

	for (i = 0; i < count; i++) {
		c = readb(recv->buf + recv->tail);

		recv->tail++;

		tty_insert_flip_char(port, c, TTY_NORMAL);

		if (recv->tail >= recv_ch->size)
			recv->tail = 0;
	}
	recv_ch->rd = recv->tail;

	if (i)
		tty_flip_buffer_push(port);

	mod_timer(&state->timer, jiffies + DELAY_TIME);
}

static int bptty_state_init(struct device_node *np, struct bptty_state *state)
{
	struct resource *resource;
	void __iomem *ioptr;
	u64 addr[BPTTY_CHANNELS];
	unsigned int size;
	struct bptty_chnl *ch;
	static int init_done;
	int i, ret;

	if (!state)
		return -ENODEV;

	if (init_done)
		return 0;

	sema_init(&state->sem, 1);

	ret = of_property_read_u64(np, "xmit-addr", &addr[BPTTY_XMIT_CH]);
	if (ret) {
		pr_err("bptty: failed no xmit-addr found\n");
		goto exit;
	}

	ret = of_property_read_u64(np, "recv-addr", &addr[BPTTY_RECV_CH]);
	if (ret) {
		pr_err("bptty: failed no recv-addr found\n");
		goto exit;
	}

	for (i = 0; i < BPTTY_CHANNELS; i++) {
		if (state->ch[i])
			iounmap((void __iomem *)state->ch[i]);

		/*
		 * Read the channel struct to obtain data size to remap to
		 */
		ioptr = ioremap(addr[i], sizeof(struct bptty_chnl));
		if (IS_ERR(ioptr)) {
			pr_err("bptty: failed to map addr space at offset 0x%llx\n",
			       addr[i]);
			ret = PTR_ERR(ioptr);
			goto exit;
		}

		ch = (struct bptty_chnl *)ioptr;
		/* contains data size only */
		size = ch->size;
		iounmap(ioptr);

		/* adjust size to include the channel struct to map to */
		size += BPTTY_CH_ADDR(data);
		resource = request_mem_region(addr[i], size, "bcm-bptty");
		if (!resource) {
			pr_err("bptty: failed to request memory region for offset 0x%llx\n",
			       addr[i]);
			ret = -EBUSY;
			goto exit;
		}
		state->resources[i] = resource;

		ioptr = ioremap(addr[i], size);
		if (IS_ERR(ioptr)) {
			pr_err("bptty: failed to map buffer at offset 0x%llx\n",
			       addr[i]);
			ret = PTR_ERR(ioptr);
			goto exit;
		}
		state->ch[i] = (struct bptty_chnl *)ioptr;
		state->xmits[i].buf = (unsigned char *)(ioptr +
							BPTTY_CH_ADDR(data));
		state->xmits[i].head = state->xmits[i].tail = 0;

		/* check read/write offset starts at zero */
		if (state->ch[i]->rd || state->ch[i]->wr) {
			pr_err("invalid read/write start address offsets 0x%x/0x%x\n",
				state->ch[i]->rd, state->ch[i]->wr);
			ret = -EINVAL;
			goto exit;
		}
	}

	init_done = 1;
	return 0;

exit:
	for (i = 0; i < BPTTY_CHANNELS; i++) {
		if (state->ch[i])
			iounmap(state->ch[i]);

		state->ch[i] = NULL;

		if (state->resources[i])
			release_resource(state->resources[i]);

		state->resources[i] = NULL;
	}
	return ret;
}

static int bptty_open(struct tty_struct *tty, struct file *file)
{
	struct bptty_state *state = &state_info;
	struct tty_port *port;

	if (!state)
		return -ENODEV;

	port = &state->port;

	if (tty->index < 0 || tty->index >= BPTTY_PORT_NUM)
		return -EINVAL;

	down(&state->sem);

	port->client_data = state;
	tty->driver_data = state;
	tty->port = port;

	state->tty = tty;

	state->open_count++;

	tty_port_tty_set(port, tty);

	up(&state->sem);
	return 0;
}

static void do_close(struct bptty_state *state)
{
	struct tty_port *port = &state->port;

	down(&state->sem);

	if (!state->open_count)
		goto exit;

	state->open_count--;
	if (state->open_count <= 0) {
		del_timer(&state->timer);
		tty_port_tty_set(port, NULL);
	}
exit:
	up(&state->sem);
}

static void bptty_close(struct tty_struct *tty, struct file *file)
{
	struct bptty_state *state = tty->driver_data;

	if (state)
		do_close(state);
}

/* Return no of bytes successfully transferred to memory */
static int do_write(struct bptty_state *state, const unsigned char *buf,
		    int count)
{
	int i;

	if (!state->xmits[BPTTY_XMIT_CH].buf)
		return 0;

	for (i = 0; i < count; i++) {
		writeb(buf[i], state->xmits[BPTTY_XMIT_CH].buf +
		       state->xmits[BPTTY_XMIT_CH].head);

		state->xmits[BPTTY_XMIT_CH].head++;

		if (state->xmits[BPTTY_XMIT_CH].head >=
		    state->ch[BPTTY_XMIT_CH]->size)
			state->xmits[BPTTY_XMIT_CH].head = 0;
	}
	state->ch[BPTTY_XMIT_CH]->wr = state->xmits[BPTTY_XMIT_CH].head;
	return count;
}

static int bptty_write(struct tty_struct *tty, const unsigned char *buffer,
		       int count)
{
	struct bptty_state *state = tty->driver_data;
	int wrote = 0;

	if (!state)
		return -ENODEV;

	if (!state->open_count)
		goto exit;

	wrote = do_write(state, buffer, count);
exit:
	return wrote;
}

static int update_xmit_rd_index(struct bptty_state *state, unsigned int count)
{
	int space_left;
	struct bptty_chnl *xmit_ch = state->ch[BPTTY_XMIT_CH];
	struct circ_buf *xmit = &state->xmits[BPTTY_XMIT_CH];

	xmit->tail = xmit_ch->rd;

	space_left = CIRC_SPACE(xmit->head, xmit->tail, xmit_ch->size);

	/*
	 * In case where read not performed by remote host, tail will not
	 * get incremented. Thus move tail (overwrite) to make space.
	 */
	while (space_left <= count) {
		xmit->tail++;

		if (xmit->tail >= xmit_ch->size)
			xmit->tail = 0;

		xmit_ch->rd = xmit->tail;

		/* recalculate with adjusted tail */
		space_left = CIRC_SPACE(xmit->head, xmit->tail, xmit_ch->size);
	}
	return space_left;
}

static int bptty_write_room(struct tty_struct *tty)
{
	struct bptty_state *state = tty->driver_data;
	int room = -EINVAL;

	if (!state)
		return -ENODEV;

	if (!state->open_count)
		goto exit;

	/*
	 * Check available space in send memory
	 * and ensure at least space for one character
	 */
	room = update_xmit_rd_index(state, 1);

exit:
	return room;
}

static const struct tty_operations tty_ops = {
	.open = bptty_open,
	.close = bptty_close,
	.write = bptty_write,
	.write_room = bptty_write_room,
};

static void bptty_console_write(struct console *co, const char *buf,
				unsigned int count)
{
	struct bptty_state *state = &state_info;

	/*
	 * Ensure read index is incremented enough to write
	 * the whole buffer content.
	 */
	update_xmit_rd_index(state, count);

	do_write(state, buf, count);
}

static struct tty_driver *bptty_console_device(struct console *co, int *index)
{
	*index = co->index;
	return bptty_tty_driver;
}

static int bptty_console_setup(struct console *co, char *options)
{
	struct bptty_state *state = &state_info;
	struct device_node *np;

	np = of_find_matching_node(NULL, bptty_of_match);
	if (IS_ERR_OR_NULL(np)) {
		pr_err("failed to find device node for bptty\n");
		return -EINVAL;
	}
	return bptty_state_init(np, state);
}

static struct console bptty_console = {
	.name = BPTTY_PORT_NAME,
	.write = bptty_console_write,
	.device = bptty_console_device,
	.setup = bptty_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
};

/*
 * Register console.
 */
static int __init bptty_console_init(void)
{
	register_console(&bptty_console);
	return 0;
}
console_initcall(bptty_console_init);

static int bptty_probe(struct platform_device *pdev)
{
	struct bptty_state *state = &state_info;
	struct tty_port *port = &state->port;
	int retval;

	retval = bptty_state_init(pdev->dev.of_node, state);
	if (retval) {
		dev_err(&pdev->dev, "failed to init bptty console\n");
		return retval;
	}

	bptty_tty_driver = alloc_tty_driver(BPTTY_PORT_NUM);
	if (!bptty_tty_driver) {
		dev_err(&pdev->dev, "failed to alloc tty driver\n");
		return -ENOMEM;
	}

	/* initialize the tty driver */
	bptty_tty_driver->owner = THIS_MODULE;
	bptty_tty_driver->driver_name = "bcm_bptty";
	bptty_tty_driver->name = BPTTY_PORT_NAME;
	bptty_tty_driver->num = BPTTY_PORT_NUM;
	bptty_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	bptty_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	bptty_tty_driver->init_termios = tty_std_termios;
	bptty_tty_driver->init_termios.c_iflag = IGNBRK | IGNPAR;
	bptty_tty_driver->init_termios.c_oflag = ONLCR;
	bptty_tty_driver->init_termios.c_lflag = ISIG | ECHO;
	bptty_tty_driver->flags = TTY_DRIVER_REAL_RAW;
	bptty_tty_driver->init_termios.c_cflag = B57600 | CS8 |
		CREAD | HUPCL | CLOCAL;
	tty_set_operations(bptty_tty_driver, &tty_ops);

	tty_port_init(port);
	tty_port_link_device(port, bptty_tty_driver, 0);

	retval = tty_register_driver(bptty_tty_driver);
	if (retval) {
		dev_err(&pdev->dev, "failed to register bcm_bptty driver");
		put_tty_driver(bptty_tty_driver);
		tty_port_destroy(port);
		return retval;
	}

	timer_setup(&state->timer, receive_poll, 0);
	mod_timer(&state->timer, jiffies + DELAY_TIME);

	platform_set_drvdata(pdev, state);

	return retval;
}

static int bptty_remove(struct platform_device *pdev)
{
	struct bptty_state *state;
	int i;

	state = platform_get_drvdata(pdev);
	unregister_console(&bptty_console);
	tty_unregister_driver(bptty_tty_driver);
	put_tty_driver(bptty_tty_driver);
	tty_port_destroy(&state->port);

	for (i = 0; i < BPTTY_CHANNELS; i++) {
		if (state->ch[i])
			iounmap(state->ch[i]);

		state->ch[i] = NULL;

		if (state->resources[i])
			release_resource(state->resources[i]);

		state->resources[i] = NULL;
	}

	/* shut down our timer and free the memory */
	del_timer(&state->timer);
	return 0;
}

static struct platform_driver bptty_platform_driver = {
	.probe = bptty_probe,
	.remove = bptty_remove,
	.driver = {
		.name = "bcm_bptty",
		.of_match_table = bptty_of_match,
	},
};

static int __init bptty_init(void)
{
	return platform_driver_register(&bptty_platform_driver);
}

static void __exit bptty_exit(void)
{
	platform_driver_unregister(&bptty_platform_driver);
}

module_init(bptty_init);
module_exit(bptty_exit);

MODULE_DESCRIPTION("Broadcom PCIe tty console driver");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL");
