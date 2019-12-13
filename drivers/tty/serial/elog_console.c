// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Broadcom
 */

/*
 * The ELOG console implements both a bootconsole (earlycon) and a console that
 * writes the kernel log buffer to volatile memory. It's configured with a
 * kernel boot parameter (earlylog=address,size). Both an earlycon and earlylog
 * can be configured at the same time. Any output to earlycon will also be sent
 * to earlylog. earlylog is disabled when the "normal" console is available and
 * initialized by the kernel.
 */
#include <asm/early_ioremap.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/serial_core.h>
#include <linux/types.h>

/* logging signature */
#define ELOG_SIG_OFFSET       0x0000
#define ELOG_SIG_VAL          0x75767971

/* current logging offset that points to where new logs should be added */
#define ELOG_OFF_OFFSET       0x0004

/* current logging length (excluding header) */
#define ELOG_LEN_OFFSET       0x0008

#define ELOG_HEADER_LEN       12

/*
 * @base: CPU virtual address of the memory where log is saved
 * @is_initialized: flag that indicates logging has been initialized
 * @addr: physical address of the memory where log is saved
 * @max_size: maximum log buffer size
 */
struct console_elog_data {
	void *base;
	bool is_initialized;
	phys_addr_t addr;
	u32 max_size;
};

static struct console_elog_data elog_data;

static struct console earlycon_elog = {
	.name = "earlycon_elog",
	/* Not specifying CON_PRINTBUFFER to avoid bug in printk. */
	.flags = /*CON_PRINTBUFFER |*/ CON_BOOT,
	.data = &elog_data,
};

static void elog_putc(struct console *console, int c)
{
	struct console_elog_data *elog = console->data;
	u32 offset, len;

	if (!elog->base)
		panic("base address uninitialized\n");

	offset = readl(elog->base + ELOG_OFF_OFFSET);
	len = readl(elog->base + ELOG_LEN_OFFSET);
	writeb(c, elog->base + offset);
	offset++;

	/* elog buffer is now full and need to wrap around */
	if (offset >= elog->max_size)
		offset = ELOG_HEADER_LEN;

	/* only increment length when elog buffer is not full */
	if (len < elog->max_size - ELOG_HEADER_LEN)
		len++;

	writel(offset, elog->base + ELOG_OFF_OFFSET);
	writel(len, elog->base + ELOG_LEN_OFFSET);
}

static void elog_write(struct console *console, const char *s,
		       unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++, s++) {
		if (*s == '\n')
			elog_putc(console, '\r');
		elog_putc(console, *s);
	}
}

static int early_elog_init(struct console *console)
{
	struct console_elog_data *elog = console->data;
	u32 val;

	/* Bail out if invalid parameters are seen */
	if (elog->is_initialized || !elog->addr || !elog->max_size)
		return -EINVAL;

	elog->base = early_memremap(elog->addr, elog->max_size);
	if (!elog->base)
		return -ENOMEM;

	/* Bail out if no header signature can be found */
	val = readl(elog->base + ELOG_SIG_OFFSET);
	if (val != ELOG_SIG_VAL) {
		early_memunmap(elog->base, elog->max_size);
		return -EINVAL;
	}

	/* Register the boot console and ensure it was enabled. */
	console->write = elog_write;
	register_console(console);
	if (!(console->flags & CON_ENABLED)) {
		early_memunmap(elog->base, elog->max_size);
		return -EINVAL;
	}

	elog->is_initialized = true;

	return 0;
}

static int __init param_setup_earlyelog(char *buf)
{
	struct console_elog_data *elog = &elog_data;
	phys_addr_t addr;
	u32 size;
	char *end;
	int err = -EINVAL;

	if (!buf || !buf[0])
		return err;

	/* Parse boot param (eg. earlylog=address,size) */
	addr = memparse(buf, &end);
	if (*end == ',') {
		size = memparse(end + 1, NULL);

		if (addr && size) {
			elog->addr = addr;
			elog->max_size = size;
		} else
			return err;
	}

	err = early_elog_init(&earlycon_elog);
	if (err < 0)
		return err;

	return 0;
}
early_param("earlyelog", param_setup_earlyelog);

/*
 * The console is enabled by default because it doesn't need to be selected on
 * the kernel command line (eg- console=) and therefore wouldn't be enabled
 * automatically when registered.
 */
static struct console console_elog = {
	.name = "console_elog",
	/* Not specifying CON_PRINTBUFFER to avoid bug in printk. */
	.flags = /*CON_PRINTBUFFER |*/ CON_ENABLED,
	.write = elog_write,
	.data = &elog_data,
};

static int __init elog_console_init(void)
{
	struct console_elog_data *elog = console_elog.data;

	/* console_elog depends on earlycon_elog being initialized. */
	if (!elog->is_initialized || !elog->addr || !elog->max_size)
		return -EINVAL;

	/* Clean up earlycon_elog. */
	unregister_console(&earlycon_elog);
	early_memunmap(elog->base, elog->max_size);

	/*
	 * The early base address is no longer usable and must be mapped with
	 * memremap now that it's available.
	 */
	elog->base = memremap(elog->addr, elog->max_size, MEMREMAP_WB);
	if (!elog->base)
		return -ENOMEM;

	register_console(&console_elog);

	return 0;
}
console_initcall(elog_console_init);
