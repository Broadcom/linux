/*
 * Copyright 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 (GPLv2) for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 (GPLv2) along with this source code.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>

#include "pdc.h"

/* top level debug FS directory for PDC driver */
static struct dentry *debugfs_dir;

static ssize_t pdc_debugfs_read(struct file *filp, char __user *ubuf,
				size_t count, loff_t *offp)
{
	struct pdc_state *pdcs;
	char *buf;
	ssize_t ret, out_offset, out_count;

	out_count = 512;

	buf = kmalloc(out_count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	pdcs = filp->private_data;
	out_offset = 0;
	out_offset += snprintf(buf + out_offset, out_count - out_offset,
			       "SPU %u stats:\n", pdcs->pdc_idx);
	out_offset += snprintf(buf + out_offset, out_count - out_offset,
			       "PDC requests............%u\n",
			       pdcs->pdc_requests);
	out_offset += snprintf(buf + out_offset, out_count - out_offset,
			       "PDC responses...........%u\n",
			       pdcs->pdc_replies);
	out_offset += snprintf(buf + out_offset, out_count - out_offset,
			       "Tx err ring full........%u\n",
			       pdcs->txnobuf);
	out_offset += snprintf(buf + out_offset, out_count - out_offset,
			       "Rx err ring full........%u\n",
			       pdcs->rxnobuf);
	out_offset += snprintf(buf + out_offset, out_count - out_offset,
			       "Receive overflow........%u\n",
			       pdcs->rx_oflow);
	out_offset += snprintf(buf + out_offset, out_count - out_offset,
			       "Missing rx init call....%u\n",
			       pdcs->no_rx_init);

	if (out_offset > out_count)
		out_offset = out_count;

	ret = simple_read_from_buffer(ubuf, count, offp, buf, out_offset);
	kfree(buf);
	return ret;
}

static const struct file_operations pdc_debugfs_stats = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = pdc_debugfs_read,
};

/*
 * Create the debug FS directories. If the top-level directory has not yet
 * been created, create it now. Create a stats file in this directory for
 * a SPU.
 */
void pdc_setup_debugfs(struct pdc_state *pdcs)
{
	char spu_stats_name[16];

	if (!debugfs_initialized())
		return;

	snprintf(spu_stats_name, 16, "pdc%d_stats", pdcs->pdc_idx);
	if (!debugfs_dir)
		debugfs_dir = debugfs_create_dir(KBUILD_MODNAME, NULL);

	pdcs->debugfs_stats = debugfs_create_file(spu_stats_name, S_IRUSR,
						  debugfs_dir, pdcs,
						  &pdc_debugfs_stats);
}

void pdc_free_debugfs(void)
{
	if (debugfs_dir && simple_empty(debugfs_dir)) {
		debugfs_remove(debugfs_dir);
		debugfs_dir = NULL;
	}
}

void pdc_free_debugfs_stats(struct pdc_state *pdcs)
{
	debugfs_remove(pdcs->debugfs_stats);
}
