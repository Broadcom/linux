// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018-2019 Broadcom.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/hash.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>

#include "bcm_vk.h"
#include "bcm_vk_msg.h"
#include "bcm_vk_sg.h"

/*
 * allocate a ctx per file struct
 */
static struct bcm_vk_ctx *bcm_vk_get_ctx(struct bcm_vk *vk,
					 struct task_struct *ppid)
{
	uint32_t i;
	struct bcm_vk_ctx *ctx = NULL;
	const pid_t pid = task_pid_nr(ppid);
	uint32_t hash_idx = hash_32(pid, VK_PID_HT_SHIFT_BIT);

	spin_lock(&vk->ctx_lock);

	/* check if it is in reset, if so, don't allow */
	if (vk->reset_ppid) {
		dev_err(&vk->pdev->dev,
			"No context allowed during reset by pid %d\n",
			task_pid_nr(vk->reset_ppid));

		goto in_reset_exit;
	}

	for (i = 0; i < VK_CMPT_CTX_MAX; i++) {
		if (!vk->ctx[i].in_use) {
			vk->ctx[i].in_use = true;
			ctx = &vk->ctx[i];
			break;
		}
	}

	if (!ctx) {
		dev_err(&vk->pdev->dev, "All context in use\n");

		goto all_in_use_exit;
	}

	/* set the pid and insert it to hash table */
	ctx->ppid = ppid;
	ctx->hash_idx = hash_idx;
	list_add_tail(&ctx->node, &vk->pid_ht[hash_idx].head);

	/* increase kref */
	kref_get(&vk->kref);

all_in_use_exit:
in_reset_exit:
	spin_unlock(&vk->ctx_lock);

	return ctx;
}

static uint16_t bcm_vk_get_msg_id(struct bcm_vk *vk)
{
	uint16_t rc = VK_MSG_ID_OVERFLOW;
	uint16_t test_bit_count = 0;

	spin_lock(&vk->msg_id_lock);
	while (test_bit_count < VK_MSG_ID_BITMAP_SIZE) {
		vk->msg_id++;
		vk->msg_id = vk->msg_id & VK_MSG_ID_BITMAP_MASK;
		if (test_bit(vk->msg_id, vk->bmap)) {
			test_bit_count++;
			continue;
		}
		rc = vk->msg_id;
		bitmap_set(vk->bmap, vk->msg_id, 1);
		break;
	}
	spin_unlock(&vk->msg_id_lock);

	return rc;
}

static int bcm_vk_free_ctx(struct bcm_vk *vk, struct bcm_vk_ctx *ctx)
{
	uint32_t idx;
	uint32_t hash_idx;
	pid_t pid;
	struct bcm_vk_ctx *entry;
	int count = 0;

	if (ctx == NULL) {
		dev_err(&vk->pdev->dev, "NULL context detected\n");
		return -EINVAL;
	}
	idx = ctx->idx;
	pid = task_pid_nr(ctx->ppid);

	spin_lock(&vk->ctx_lock);

	if (!vk->ctx[idx].in_use) {
		dev_err(&vk->pdev->dev, "context[%d] not in use!\n", idx);
	} else {
		vk->ctx[idx].in_use = false;
		vk->ctx[idx].miscdev = NULL;

		/* Remove it from hash list and see if it is the last one. */
		list_del(&ctx->node);
		hash_idx = ctx->hash_idx;
		list_for_each_entry(entry, &vk->pid_ht[hash_idx].head, node) {
			if (task_pid_nr(entry->ppid) == pid)
				count++;
		}
	}

	spin_unlock(&vk->ctx_lock);

	return count;
}

static void bcm_vk_free_wkent(struct device *dev, struct bcm_vk_wkent *entry)
{
	bcm_vk_sg_free(dev, entry->dma, VK_DMA_MAX_ADDRS);

	kfree(entry->vk2h_msg);
	kfree(entry);
}

static void bcm_vk_drain_all_pend(struct device *dev,
				  struct bcm_vk_msg_chan *chan,
				  struct bcm_vk_ctx *ctx)
{
	uint32_t num;
	struct bcm_vk_wkent *entry, *tmp;

	spin_lock(&chan->pendq_lock);
	for (num = 0; num < chan->q_nr; num++) {
		list_for_each_entry_safe(entry, tmp, &chan->pendq[num], node) {
			if (ctx == NULL) {
				list_del(&entry->node);
				bcm_vk_free_wkent(dev, entry);
			} else if (entry->ctx->idx == ctx->idx) {
				struct vk_msg_blk *msg;

				/* if it is specific ctx, log for any stuck */
				msg = entry->h2vk_msg;
				dev_err(dev,
					"Drained: fid %u size %u msg 0x%x(seq-%x) ctx 0x%x[fd-%d] args:[0x%x 0x%x] resp %s",
					msg->function_id, msg->size,
					msg->msg_id, entry->seq_num,
					msg->context_id, entry->ctx->idx,
					msg->args[0], msg->args[1],
					entry->vk2h_msg ? "T" : "F");
				list_del(&entry->node);
				bcm_vk_free_wkent(dev, entry);
			}
		}
	}
	spin_unlock(&chan->pendq_lock);
}

bool bcm_vk_msgq_marker_valid(struct bcm_vk *vk)
{
	uint32_t rdy_marker = 0;
	uint32_t fw_status;

	fw_status = vkread32(vk, BAR_0, BAR_FW_STATUS);

	if ((fw_status & FW_STATUS_READY) == FW_STATUS_READY)
		rdy_marker = vkread32(vk, BAR_1, VK_BAR1_MSGQ_DEF_RDY);

	return (rdy_marker == VK_BAR1_MSGQ_RDY_MARKER);
}

/*
 * Function to sync up the messages queue info that is provided by BAR1
 */
int bcm_vk_sync_msgq(struct bcm_vk *vk)
{
	struct bcm_vk_msgq *msgq = NULL;
	struct device *dev = &vk->pdev->dev;
	uint32_t msgq_off;
	uint32_t num_q;
	struct bcm_vk_msg_chan *chan_list[] = {&vk->h2vk_msg_chan,
					       &vk->vk2h_msg_chan};
	struct bcm_vk_msg_chan *chan = NULL;
	int i, j;

	/*
	 * if this function is called when it is already inited,
	 * something is wrong
	 */
	if (vk->msgq_inited) {
		dev_err(dev, "Msgq info already in sync\n");
		return -EPERM;
	}

	/*
	 * If the driver is loaded at startup where vk OS is not up yet,
	 * the msgq-info may not be available until a later time.  In
	 * this case, we skip and the sync function is supposed to be
	 * called again.
	 */
	if (!bcm_vk_msgq_marker_valid(vk)) {
		dev_info(dev, "BAR1 msgq marker not initialized.\n");
		return 0;
	}

	msgq_off = vkread32(vk, BAR_1, VK_BAR1_MSGQ_CTRL_OFF);

	/* each side is always half the total  */
	num_q = vk->h2vk_msg_chan.q_nr = vk->vk2h_msg_chan.q_nr =
		vkread32(vk, BAR_1, VK_BAR1_MSGQ_NR) / 2;

	/* first msgq location */
	msgq = (struct bcm_vk_msgq *)(vk->bar[BAR_1] + msgq_off);

	for (i = 0; i < ARRAY_SIZE(chan_list); i++) {
		chan = chan_list[i];
		for (j = 0; j < num_q; j++) {
			chan->msgq[j] = msgq;

			dev_info(dev,
				 "MsgQ[%d] type %d num %d, @ 0x%x, rd_idx %d wr_idx %d, size %d, nxt 0x%x\n",
				 j,
				 chan->msgq[j]->type,
				 chan->msgq[j]->num,
				 chan->msgq[j]->start,
				 chan->msgq[j]->rd_idx,
				 chan->msgq[j]->wr_idx,
				 chan->msgq[j]->size,
				 chan->msgq[j]->nxt);

			msgq = (struct bcm_vk_msgq *)
				((char *)msgq + sizeof(*msgq) + msgq->nxt);

			rmb(); /* do a read mb to guarantee */
		}
	}

	vk->msgq_inited = true;

	return 0;
}

static int bcm_vk_msg_chan_init(struct bcm_vk_msg_chan *chan)
{
	int rc = 0;
	uint32_t i;

	mutex_init(&chan->msgq_mutex);
	spin_lock_init(&chan->pendq_lock);
	for (i = 0; i < VK_MSGQ_MAX_NR; i++)
		INIT_LIST_HEAD(&chan->pendq[i]);

	return rc;
}

static void bcm_vk_append_pendq(struct bcm_vk_msg_chan *chan, uint16_t q_num,
				struct bcm_vk_wkent *entry)
{
	spin_lock(&chan->pendq_lock);
	list_add_tail(&entry->node, &chan->pendq[q_num]);
	spin_unlock(&chan->pendq_lock);
}

static uint32_t bcm_vk_append_ib_sgl(struct bcm_vk *vk,
				     struct bcm_vk_wkent *entry,
				     struct _vk_data *data,
				     unsigned int num_planes)
{
	unsigned int i;
	unsigned int item_cnt = 0;
	struct device *dev = &vk->pdev->dev;
	uint32_t ib_sgl_size = 0;
	uint8_t *buf = (uint8_t *)&entry->h2vk_msg[entry->h2vk_blks];

	for (i = 0; i < num_planes; i++) {
		if (data[i].address &&
		    (ib_sgl_size + data[i].size) <= vk->ib_sgl_size) {

			item_cnt++;
			memcpy(buf, entry->dma[i].sglist, data[i].size);
			ib_sgl_size += data[i].size;
			buf += data[i].size;
		}
	}

	dev_dbg(dev, "Num %u sgl items appended, size 0x%x, room 0x%x\n",
		item_cnt, ib_sgl_size, vk->ib_sgl_size);

	/* round up size */
	ib_sgl_size = (ib_sgl_size + VK_MSGQ_BLK_SIZE - 1)
		       >> VK_MSGQ_BLK_SZ_SHIFT;

	return ib_sgl_size;
}

void bcm_h2vk_doorbell(struct bcm_vk *vk, uint32_t q_num,
			      uint32_t db_val)
{
	/* press door bell based on q_num */
	vkwrite32(vk,
		  db_val,
		  BAR_0,
		  VK_BAR0_REGSEG_DB_BASE + q_num * VK_BAR0_REGSEG_DB_REG_GAP);
}

static int bcm_h2vk_msg_enqueue(struct bcm_vk *vk, struct bcm_vk_wkent *entry)
{
	static uint32_t seq_num;
	struct bcm_vk_msg_chan *chan = &vk->h2vk_msg_chan;
	struct device *dev = &vk->pdev->dev;
	struct vk_msg_blk *src = &entry->h2vk_msg[0];

	volatile struct vk_msg_blk *dst;
	struct bcm_vk_msgq *msgq;
	uint32_t q_num = src->queue_id;
	uint32_t wr_idx; /* local copy */
	uint32_t i;

	if (entry->h2vk_blks != src->size + 1) {
		dev_err(dev, "number of blks %d not matching %d MsgId[0x%x]: func %d ctx 0x%x\n",
			entry->h2vk_blks,
			src->size + 1,
			src->msg_id,
			src->function_id,
			src->context_id);
		return -EMSGSIZE;
	}

	msgq = chan->msgq[q_num];

	rmb(); /* start with a read barrier */
	mutex_lock(&chan->msgq_mutex);

	/* if not enough space, return EAGAIN and let app handles it */
	if (VK_MSGQ_AVAIL_SPACE(msgq) < entry->h2vk_blks) {
		mutex_unlock(&chan->msgq_mutex);
		return -EAGAIN;
	}

	/* at this point, mutex is taken and there is enough space */
	entry->seq_num = seq_num++; /* update debug seq number */
	wr_idx = msgq->wr_idx;

	dst = VK_MSGQ_BLK_ADDR(vk->bar[BAR_1], msgq, wr_idx);
	for (i = 0; i < entry->h2vk_blks; i++) {
		*dst = *src;

		src++;
		wr_idx = VK_MSGQ_INC(msgq, wr_idx, 1);
		dst = VK_MSGQ_BLK_ADDR(vk->bar[BAR_1], msgq, wr_idx);
	}

	/* flush the write pointer */
	msgq->wr_idx = wr_idx;
	wmb(); /* flush */

	/* log new info for debugging */
	dev_dbg(dev,
		"MsgQ[%d] [Rd Wr] = [%d %d] blks inserted %d - Q = [u-%d a-%d]/%d\n",
		msgq->num,
		msgq->rd_idx, msgq->wr_idx, entry->h2vk_blks,
		VK_MSGQ_OCCUPIED(msgq),
		VK_MSGQ_AVAIL_SPACE(msgq),
		msgq->size);

	mutex_unlock(&chan->msgq_mutex);

	/*
	 * press door bell based on queue number. 1 is added to the wr_idx
	 * to avoid the value of 0 appearing on the VK side to distinguish
	 * from initial value.
	 */
	bcm_h2vk_doorbell(vk, q_num, wr_idx + 1);

	return 0;
}

int bcm_vk_send_shutdown_msg(struct bcm_vk *vk, uint32_t shut_type,
			     pid_t pid)
{
	int rc = 0;
	struct bcm_vk_wkent *entry;
	struct device *dev = &vk->pdev->dev;

	/*
	 * check if the marker is still good.  Sometimes, the PCIe interface may
	 * have gone done, and if so and we ship down thing based on broken
	 * values, kernel may panic.
	 */
	if (!bcm_vk_msgq_marker_valid(vk)) {
		dev_err(dev, "Marker invalid - PCIe interface potentially done!\n");
		return -EINVAL;
	}

	entry = kzalloc(sizeof(struct bcm_vk_wkent) +
			sizeof(struct vk_msg_blk), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	/* just fill up non-zero data */
	entry->h2vk_msg[0].function_id = VK_FID_SHUTDOWN;
	entry->h2vk_msg[0].queue_id = 0; /* use highest queue */
	entry->h2vk_blks = 1; /* always 1 block */

	entry->h2vk_msg[0].args[0] = shut_type;
	entry->h2vk_msg[0].args[1] = pid;

	rc = bcm_h2vk_msg_enqueue(vk, entry);
	if (rc)
		dev_err(dev,
			"Sending shutdown message to q %d for pid %d fails.\n",
			entry->h2vk_msg[0].queue_id, pid);

	kfree(entry);

	return rc;
}

int bcm_vk_handle_last_sess(struct bcm_vk *vk, struct task_struct *ppid)
{
	int rc = 0;
	pid_t pid = task_pid_nr(ppid);
	struct device *dev = &vk->pdev->dev;

	/*
	 * don't send down or do anything if message queue is not initialized
	 * and if it is the reset session, clear it.
	 */
	if (!vk->msgq_inited) {

		if (vk->reset_ppid == ppid)
			vk->reset_ppid = NULL;
		return -EPERM;
	}

	dev_info(dev, "No more sessions, shut down pid %d\n", pid);

	/* only need to do it if it is not the reset process */
	if (vk->reset_ppid != ppid)
		rc = bcm_vk_send_shutdown_msg(vk, VK_SHUTDOWN_PID, pid);
	else
		/* reset the pointer if it is exiting last session */
		vk->reset_ppid = NULL;

	return rc;
}

static struct bcm_vk_wkent *bcm_vk_find_pending(struct bcm_vk_msg_chan *chan,
						uint16_t q_num,
						uint16_t msg_id,
						unsigned long *map)
{
	bool found = false;
	struct bcm_vk_wkent *entry;

	spin_lock(&chan->pendq_lock);
	list_for_each_entry(entry, &chan->pendq[q_num], node) {

		if (entry->h2vk_msg[0].msg_id == msg_id) {
			list_del(&entry->node);
			found = true;
			bitmap_clear(map, msg_id, 1);
			break;
		}
	}
	spin_unlock(&chan->pendq_lock);
	return ((found) ? entry : NULL);
}

static uint32_t bcm_vk2h_msg_dequeue(struct bcm_vk *vk)
{
	struct device *dev = &vk->pdev->dev;
	struct bcm_vk_msg_chan *chan = &vk->vk2h_msg_chan;
	struct vk_msg_blk *data;
	volatile struct vk_msg_blk *src;
	struct vk_msg_blk *dst;
	struct bcm_vk_msgq *msgq;
	struct bcm_vk_wkent *entry;
	uint32_t rd_idx;
	uint32_t q_num, j;
	uint32_t num_blks;
	uint32_t total = 0;

	/*
	 * drain all the messages from the queues, and find its pending
	 * entry in the h2vk queue, based on msg_id & q_num, and move the
	 * entry to the vk2h pending queue, waiting for user space
	 * program to extract
	 */
	mutex_lock(&chan->msgq_mutex);
	rmb(); /* start with a read barrier */
	for (q_num = 0; q_num < chan->q_nr; q_num++) {
		msgq = chan->msgq[q_num];

		while (!VK_MSGQ_EMPTY(msgq)) {

			/* make a local copy */
			rd_idx = msgq->rd_idx;

			/* look at the first block and decide the size */
			src = VK_MSGQ_BLK_ADDR(vk->bar[BAR_1], msgq, rd_idx);

			num_blks = src->size + 1;

			data = kzalloc(num_blks * VK_MSGQ_BLK_SIZE, GFP_KERNEL);

			if (data) {
				/* copy messages and linearize it */
				dst = data;
				for (j = 0; j < num_blks; j++) {
					*dst = *src;

					dst++;
					rd_idx = VK_MSGQ_INC(msgq, rd_idx, 1);
					src = VK_MSGQ_BLK_ADDR(vk->bar[BAR_1],
							       msgq,
							       rd_idx);
				}
				total++;
			} else {
				dev_crit(dev, "Error allocating memory\n");
				/* just keep draining..... */
				rd_idx = VK_MSGQ_INC(msgq, rd_idx, num_blks);
			}

			/* flush rd pointer after a message is dequeued */
			msgq->rd_idx = rd_idx;
			mb(); /* do both rd/wr as we are extracting data out */

			/* log new info for debugging */
			dev_dbg(dev,
				"MsgQ[%d] [Rd Wr] = [%d %d] blks extracted %d - Q = [u-%d a-%d]/%d\n",
				msgq->num,
				msgq->rd_idx, msgq->wr_idx, num_blks,
				VK_MSGQ_OCCUPIED(msgq),
				VK_MSGQ_AVAIL_SPACE(msgq),
				msgq->size);

			/*
			 * No need to search if it is an autonomous one-way
			 * message from driver, as these messages do not bear
			 * a h2vk pending item. Currently, only the shutdown
			 * message falls into this category.
			 */
			if (data->function_id == VK_FID_SHUTDOWN) {
				kfree(data);
				continue;
			}

			/* lookup original message in h2vk direction */
			entry = bcm_vk_find_pending(&vk->h2vk_msg_chan,
						    q_num,
						    data->msg_id,
						    vk->bmap);

			/*
			 * if there is message to does not have prior send,
			 * this is the location to add here
			 */
			if (entry) {
				entry->vk2h_blks = num_blks;
				entry->vk2h_msg = data;
				bcm_vk_append_pendq(&vk->vk2h_msg_chan,
						    q_num, entry);

			} else {
				dev_crit(dev,
					 "Could not find MsgId[0x%x] for resp func %d\n",
					 data->msg_id, data->function_id);
				kfree(data);
			}

		}
	}
	mutex_unlock(&chan->msgq_mutex);
	dev_dbg(dev, "total %d drained from queues\n", total);

	return total;
}

/*
 * deferred work queue for draining and auto download.
 */
static void bcm_vk_wq_handler(struct work_struct *work)
{
	struct bcm_vk *vk = container_of(work, struct bcm_vk, wq_work);
	struct device *dev = &vk->pdev->dev;
	uint32_t tot;

	/* check wq offload bit map and see if auto download is requested */
	if (test_bit(BCM_VK_WQ_DWNLD_AUTO, &vk->wq_offload)) {
		bcm_vk_auto_load_all_images(vk);

		/* at the end of operation, clear AUTO bit and pending bit */
		clear_bit(BCM_VK_WQ_DWNLD_AUTO, &vk->wq_offload);
		clear_bit(BCM_VK_WQ_DWNLD_PEND, &vk->wq_offload);
	}

	/* next, try to drain */
	tot = bcm_vk2h_msg_dequeue(vk);

	if (tot == 0)
		dev_dbg(dev, "Spurious trigger for workqueue\n");
}

/*
 * init routine for all required data structures
 */
static int bcm_vk_data_init(struct bcm_vk *vk)
{
	int rc = 0;
	int i;

	spin_lock_init(&vk->ctx_lock);
	for (i = 0; i < VK_CMPT_CTX_MAX; i++) {
		vk->ctx[i].in_use = false;
		vk->ctx[i].idx = i;	/* self identity */
		vk->ctx[i].miscdev = NULL;
	}
	spin_lock_init(&vk->msg_id_lock);
	vk->msg_id = 0;

	/* initialize hash table */
	for (i = 0; i < VK_PID_HT_SZ; i++)
		INIT_LIST_HEAD(&vk->pid_ht[i].head);

	INIT_WORK(&vk->wq_work, bcm_vk_wq_handler);
	return rc;
}

irqreturn_t bcm_vk_irqhandler(int irq, void *dev_id)
{
	struct bcm_vk *vk = dev_id;

	if (!vk->msgq_inited) {
		dev_err(&vk->pdev->dev,
			"Interrupt %d received when msgq not inited\n", irq);
		goto skip_schedule_work;
	}

	queue_work(vk->wq_thread, &vk->wq_work);

skip_schedule_work:
	return IRQ_HANDLED;
}

int bcm_vk_open(struct inode *inode, struct file *p_file)
{
	struct bcm_vk_ctx *ctx;
	struct miscdevice *miscdev = (struct miscdevice *)p_file->private_data;
	struct bcm_vk *vk = container_of(miscdev, struct bcm_vk, miscdev);
	struct device *dev = &vk->pdev->dev;
	int    rc = 0;

	/* get a context and set it up for file */
	ctx = bcm_vk_get_ctx(vk, current);
	if (!ctx) {
		dev_err(dev, "Error allocating context\n");
		rc = -ENOMEM;
	} else {

		/*
		 * set up context and replace private data with context for
		 * other methods to use.  Reason for the context is because
		 * it is allowed for multiple sessions to open the sysfs, and
		 * for each file open, when upper layer query the response,
		 * only those that are tied to a specific open should be
		 * returned.  The context->idx will be used for such binding
		 */
		ctx->miscdev = miscdev;
		p_file->private_data = ctx;
		dev_dbg(dev, "ctx_returned with idx %d, pid %d\n",
			ctx->idx, task_pid_nr(ctx->ppid));
	}
	return rc;
}

ssize_t bcm_vk_read(struct file *p_file, char __user *buf, size_t count,
			   loff_t *f_pos)
{
	ssize_t rc = -ENOMSG;
	struct bcm_vk_ctx *ctx = p_file->private_data;
	struct bcm_vk *vk = container_of(ctx->miscdev, struct bcm_vk,
					 miscdev);
	struct device *dev = &vk->pdev->dev;
	struct bcm_vk_msg_chan *chan = &vk->vk2h_msg_chan;
	struct bcm_vk_wkent *entry = NULL;
	uint32_t q_num;
	uint32_t rsp_length;
	bool found = false;

	dev_dbg(dev, "Buf count %ld, msgq_inited %d\n", count, vk->msgq_inited);

	if (!vk->msgq_inited)
		return -EPERM;

	found = false;

	/*
	 * search through the pendq on the vk2h chan, and return only those
	 * that belongs to the same context.  Search is always from the high to
	 * the low priority queues
	 */
	spin_lock(&chan->pendq_lock);
	for (q_num = 0; q_num < chan->q_nr; q_num++) {
		list_for_each_entry(entry, &chan->pendq[q_num], node) {
			if (entry->ctx->idx == ctx->idx) {
				if (count >=
				    (entry->vk2h_blks * VK_MSGQ_BLK_SIZE)) {
					list_del(&entry->node);
					found = true;
				} else {
					/* buffer not big enough */
					rc = -EMSGSIZE;
				}
				goto bcm_vk_read_loop_exit;
			}
		}
	}
 bcm_vk_read_loop_exit:
	spin_unlock(&chan->pendq_lock);

	if (found) {
		/* retrieve the passed down msg_id */
		entry->vk2h_msg[0].msg_id = entry->usr_msg_id;
		rsp_length = entry->vk2h_blks * VK_MSGQ_BLK_SIZE;
		if (copy_to_user(buf, entry->vk2h_msg, rsp_length) == 0)
			rc = rsp_length;

		bcm_vk_free_wkent(dev, entry);
	} else if (rc == -EMSGSIZE) {
		struct vk_msg_blk tmp_msg = entry->vk2h_msg[0];

		/*
		 * in this case, return just the first block, so
		 * that app knows what size it is looking for.
		 */
		tmp_msg.msg_id = entry->usr_msg_id;
		tmp_msg.size = entry->vk2h_blks - 1;
		if (copy_to_user(buf, &tmp_msg, VK_MSGQ_BLK_SIZE) != 0) {
			dev_err(dev, "Error return 1st block in -EMSGSIZE\n");
			rc = -EFAULT;
		}
	}
	return rc;
}

ssize_t bcm_vk_write(struct file *p_file, const char __user *buf,
			    size_t count, loff_t *f_pos)
{
	ssize_t rc = -EPERM;
	struct bcm_vk_ctx *ctx = p_file->private_data;
	struct bcm_vk *vk = container_of(ctx->miscdev, struct bcm_vk,
					 miscdev);
	struct bcm_vk_msgq *msgq;
	struct device *dev = &vk->pdev->dev;
	struct bcm_vk_wkent *entry;
	uint32_t sgl_extra_blks;

	dev_dbg(dev, "Msg count %ld, msg_inited %d\n", count, vk->msgq_inited);

	if (!vk->msgq_inited)
		return -EPERM;

	/* first, do sanity check where count should be multiple of basic blk */
	if (count & (VK_MSGQ_BLK_SIZE - 1)) {
		dev_err(dev, "Failure with size %ld not multiple of %ld\n",
			count, VK_MSGQ_BLK_SIZE);
		rc = -EBADR;
		goto bcm_vk_write_err;
	}

	/* allocate the work entry + buffer for size count and inband sgl */
	entry = kzalloc(sizeof(struct bcm_vk_wkent) + count + vk->ib_sgl_size,
			GFP_KERNEL);
	if (!entry) {
		rc = -ENOMEM;
		goto bcm_vk_write_err;
	}

	/* now copy msg from user space, and then formulate the wk ent */
	if (copy_from_user(&entry->h2vk_msg[0], buf, count))
		goto bcm_vk_write_free_ent;

	entry->h2vk_blks = count >> VK_MSGQ_BLK_SZ_SHIFT;
	entry->ctx = ctx;

	/* do a check on the blk size which could not exceed queue space */
	msgq = vk->h2vk_msg_chan.msgq[entry->h2vk_msg[0].queue_id];
	if (entry->h2vk_blks + (vk->ib_sgl_size >> VK_MSGQ_BLK_SZ_SHIFT)
	    > (msgq->size - 1)) {
		dev_err(dev, "Blk size %d exceed max queue size allowed %d\n",
			entry->h2vk_blks, msgq->size - 1);
		rc = -EOVERFLOW;
		goto bcm_vk_write_free_ent;
	}

	/* Use internal message id */
	entry->usr_msg_id = entry->h2vk_msg[0].msg_id;
	rc = bcm_vk_get_msg_id(vk);
	if (rc == VK_MSG_ID_OVERFLOW) {
		dev_err(dev, "msg_id overflow\n");
		rc = -EOVERFLOW;
		goto bcm_vk_write_free_ent;
	}
	entry->h2vk_msg[0].msg_id = rc;

	dev_dbg(dev,
		"Message ctx id %d, usr_msg_id 0x%x sent msg_id 0x%x\n",
		ctx->idx, entry->usr_msg_id,
		entry->h2vk_msg[0].msg_id);

	/* Convert any pointers to sg list */
	if (entry->h2vk_msg[0].function_id == VK_FID_TRANS_BUF) {
		unsigned int num_planes;
		int dir;
		struct _vk_data *data;

		/*
		 * check if we are in reset, if so, no buffer transfer is
		 * allowed and return error.
		 */
		if (vk->reset_ppid) {
			dev_dbg(dev, "No Transfer allowed during reset, pid %d.\n",
				task_pid_nr(ctx->ppid));
			rc = -EACCES;
			goto bcm_vk_write_free_msgid;
		}

		num_planes = entry->h2vk_msg[0].args[0] & VK_CMD_PLANES_MASK;
		if ((entry->h2vk_msg[0].args[0] & VK_CMD_MASK)
		    == VK_CMD_DOWNLOAD) {
			/* Memory transfer from vk device */
			dir = DMA_FROM_DEVICE;
		} else {
			/* Memory transfer to vk device */
			dir = DMA_TO_DEVICE;
		}

		/* Calculate vk_data location */
		/* Go to end of the message */
		data = (struct _vk_data *)
			&(entry->h2vk_msg[entry->h2vk_msg[0].size + 1]);
		/* Now back up to the start of the pointers */
		data -= num_planes;

		/* Convert user addresses to DMA SG List */
		rc = bcm_vk_sg_alloc(dev, entry->dma, dir, data, num_planes);
		if (rc)
			goto bcm_vk_write_free_msgid;

		/* try to embed inband sgl */
		sgl_extra_blks = bcm_vk_append_ib_sgl(vk, entry, data,
						      num_planes);
		entry->h2vk_blks += sgl_extra_blks;
		entry->h2vk_msg[0].size += sgl_extra_blks;
	}

	/*
	 * store wk ent to pending queue until a response is got. This needs to
	 * be done before enqueuing the message
	 */
	bcm_vk_append_pendq(&vk->h2vk_msg_chan, entry->h2vk_msg[0].queue_id,
			    entry);

	rc = bcm_h2vk_msg_enqueue(vk, entry);
	if (rc) {
		dev_err(dev, "Fail to enqueue msg to h2vk queue\n");

		/* remove message from pending list */
		entry = bcm_vk_find_pending(&vk->h2vk_msg_chan,
					    entry->h2vk_msg[0].queue_id,
					    entry->h2vk_msg[0].msg_id,
					    vk->bmap);
		goto bcm_vk_write_free_msgid;
	}

	return count;

bcm_vk_write_free_msgid:
	bitmap_clear(vk->bmap, entry->h2vk_msg[0].msg_id, 1);
bcm_vk_write_free_ent:
	kfree(entry);
bcm_vk_write_err:
	return rc;
}

int bcm_vk_release(struct inode *inode, struct file *p_file)
{
	int ret;
	struct bcm_vk_ctx *ctx = p_file->private_data;
	struct bcm_vk *vk = container_of(ctx->miscdev, struct bcm_vk, miscdev);
	struct device *dev = &vk->pdev->dev;
	struct task_struct *ppid = ctx->ppid;
	pid_t pid = task_pid_nr(ppid);

	dev_dbg(dev, "Draining with context idx %d pid %d\n",
		ctx->idx, pid);

	bcm_vk_drain_all_pend(&vk->pdev->dev, &vk->h2vk_msg_chan, ctx);
	bcm_vk_drain_all_pend(&vk->pdev->dev, &vk->vk2h_msg_chan, ctx);

	ret = bcm_vk_free_ctx(vk, ctx);
	if (ret == 0)
		ret = bcm_vk_handle_last_sess(vk, ppid);

	/* free memory if it is the last reference */
	kref_put(&vk->kref, bcm_vk_release_data);

	return ret;
}

int bcm_vk_msg_init(struct bcm_vk *vk)
{
	struct device *dev = &vk->pdev->dev;
	int err = 0;

	if (bcm_vk_data_init(vk)) {
		dev_err(dev, "Error initializing internal data structures\n");
		err = -EINVAL;
		goto err_out;
	}

	if (bcm_vk_msg_chan_init(&vk->h2vk_msg_chan) ||
	    bcm_vk_msg_chan_init(&vk->vk2h_msg_chan)) {
		dev_err(dev, "Error initializing communication channel\n");
		err = -EIO;
		goto err_out;
	}

	/* create dedicated workqueue */
	vk->wq_thread = create_singlethread_workqueue(vk->miscdev.name);
	if (!vk->wq_thread) {
		dev_err(dev, "Fail to create workqueue thread\n");
		err = -ENOMEM;
		goto err_out;
	}

	/* read msgq info */
	if (bcm_vk_sync_msgq(vk)) {
		dev_err(dev, "Error reading comm msg Q info\n");
		err = -EIO;
		goto err_out;
	}

err_out:
	return err;
}

void bcm_vk_msg_remove(struct bcm_vk *vk)
{
	destroy_workqueue(vk->wq_thread);

	/* drain all pending items */
	bcm_vk_drain_all_pend(&vk->pdev->dev, &vk->h2vk_msg_chan, NULL);
	bcm_vk_drain_all_pend(&vk->pdev->dev, &vk->vk2h_msg_chan, NULL);
	vk->msgq_inited = false;
}

void bcm_vk_trigger_reset(struct bcm_vk *vk)
{
	uint32_t i;
	u32 value;

	/* clean up before pressing the door bell */
	bcm_vk_drain_all_pend(&vk->pdev->dev, &vk->h2vk_msg_chan, NULL);
	bcm_vk_drain_all_pend(&vk->pdev->dev, &vk->vk2h_msg_chan, NULL);
	vk->msgq_inited = false;
	vkwrite32(vk, 0, BAR_1, VK_BAR1_MSGQ_DEF_RDY);
	/* make tag '\0' terminated */
	vkwrite32(vk, 0, BAR_1, VK_BAR1_BOOT1_VER_TAG);

	for (i = 0; i < VK_BAR1_DAUTH_MAX; i++) {
		vkwrite32(vk, 0, BAR_1, VK_BAR1_DAUTH_STORE_ADDR(i));
		vkwrite32(vk, 0, BAR_1, VK_BAR1_DAUTH_VALID_ADDR(i));
	}
	for (i = 0; i < VK_BAR1_SOTP_REVID_MAX; i++)
		vkwrite32(vk, 0, BAR_1, VK_BAR1_SOTP_REVID_ADDR(i));

	memset(&vk->card_info, 0, sizeof(vk->card_info));

	/*
	 * When fastboot fails, the CODE_PUSH_OFFSET stays persistent.
	 * Allowing us to debug the failure. When we call reset,
	 * we should clear CODE_PUSH_OFFSET so ROM does not execute
	 * fastboot again (and fails again) and instead waits for a new
	 * codepush.
	 */
	value = vkread32(vk, BAR_0, BAR_CODEPUSH_SBL);
	value &= ~CODEPUSH_MASK;
	vkwrite32(vk, value, BAR_0, BAR_CODEPUSH_SBL);

	/* reset fw_status with proper reason, and press db */
	vkwrite32(vk, FW_STATUS_RESET_MBOX_DB, BAR_0, BAR_FW_STATUS);
	bcm_h2vk_doorbell(vk, VK_BAR0_RESET_DB_NUM, VK_BAR0_RESET_DB_SOFT);

	/* clear 4096 bits of bitmap */
	bitmap_clear(vk->bmap, 0, VK_MSG_ID_BITMAP_SIZE);
}
