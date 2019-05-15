// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2018 Broadcom
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
 * Turn on the following to verify the data passed down to VK is good, and
 * if not, do retry.  This is a debug/workaround on FPGA PCIe issue.
 * WARN: need to revisit after discussion with HW/ASIC
 */
#define  VK_H2VK_VERIFY_AND_RETRY       1

#if VK_H2VK_VERIFY_AND_RETRY

static inline void bcm_vk_h2vk_verify_idx(struct device *dev,
					  const char *tag,
					  volatile uint32_t *p_idx,
					  const uint32_t expected_val)
{
	volatile uint32_t *p_rd_bck_idx = p_idx;
	uint32_t count = 0;

	while (*p_rd_bck_idx != expected_val) {

		count++;
		dev_err(dev, "[%d] %s exp %d rd_bck_idx %d\n",
			count, tag, expected_val, *p_rd_bck_idx);

		/* write again */
		*p_idx = expected_val;
	}
}

static inline void bcm_vk_h2vk_verify_blk(struct device *dev,
					  const struct vk_msg_blk *p_src_blk,
					  volatile struct vk_msg_blk *p_dst_blk)

{
	struct vk_msg_blk rd_bck_blk;
	uint32_t count = 0;

	rd_bck_blk = *p_dst_blk;
	while (memcmp(&rd_bck_blk,
		      p_src_blk,
		      sizeof(rd_bck_blk)) != 0) {

		count++;
		dev_err(dev,
			"[%d]Src Blk: [0x%x 0x%x 0x%x 0x%x]\n",
			count, *(uint32_t *)p_src_blk,
			p_src_blk->context_id,
			p_src_blk->args[0],
			p_src_blk->args[1]);
		dev_err(dev,
			"[%d]Rdb Blk: [0x%x 0x%x 0x%x 0x%x]\n",
			count, *(uint32_t *)(&rd_bck_blk),
			rd_bck_blk.context_id,
			rd_bck_blk.args[0],
			rd_bck_blk.args[1]);

		*p_dst_blk = *p_src_blk;
		rd_bck_blk = *p_dst_blk;
	}
}

#endif /* VK_H2VK_VERIFY_AND_RETRY */

/*
 * allocate a ctx per file struct
 */
static struct bcm_vk_ctx *bcm_vk_get_ctx(struct bcm_vk *vk,
					 struct task_struct *p_pid)
{
	struct bcm_vk_ctx *p_ctx = NULL;
	uint32_t i;
	const pid_t pid = task_pid_nr(p_pid);
	uint32_t hash_idx = hash_32(pid, VK_PID_HT_SHIFT_BIT);

	spin_lock(&vk->ctx_lock);

	/* check if it is in reset, if so, don't allow */
	if (vk->reset_ppid) {
		dev_err(&vk->pdev->dev, "No context allowed during reset by pid %d\n",
			task_pid_nr(vk->reset_ppid));
		goto in_reset_exit;
	}

	for (i = 0; i < VK_CMPT_CTX_MAX; i++) {
		if (!vk->op_ctx[i].in_use) {
			vk->op_ctx[i].in_use = true;
			p_ctx = &vk->op_ctx[i];
			break;
		}
	}

	/* set the pid and insert it to hash table */
	p_ctx->p_pid = p_pid;
	p_ctx->hash_idx = hash_idx;
	list_add_tail(&p_ctx->list_node, &vk->pid_ht[hash_idx].fd_head);

	/* increase kref */
	kref_get(&vk->kref);

in_reset_exit:
	spin_unlock(&vk->ctx_lock);

	return p_ctx;
}

static uint16_t bcm_vk_get_msg_id(struct bcm_vk *vk)
{
	uint16_t rc = VK_MSG_ID_OVERFLOW;
	uint16_t test_bit_count = 0;

	spin_lock(&vk->msg_id_lock);
	while (test_bit_count < VK_MSG_ID_BITMAP_SIZE) {
		vk->msg_id++;
		vk->msg_id = (vk->msg_id & 0x0FFF);
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

static int bcm_vk_free_ctx(struct bcm_vk *vk, struct bcm_vk_ctx *p_ctx)
{
	uint32_t idx;
	uint32_t hash_idx;
	pid_t pid;
	struct bcm_vk_ctx *p_ent;
	int count = 0;

	if (p_ctx == NULL) {
		dev_err(&vk->pdev->dev, "NULL context detected\n");
		return -EINVAL;
	}
	idx = p_ctx->idx;
	pid = task_pid_nr(p_ctx->p_pid);

	spin_lock(&vk->ctx_lock);

	if (!vk->op_ctx[idx].in_use) {
		dev_err(&vk->pdev->dev, "Freeing context id[%d] not in use!\n",
			idx);
	} else {
		vk->op_ctx[idx].in_use = false;
		vk->op_ctx[idx].p_miscdev = NULL;

		/*
		 * remove it from hash list, and do a search to see if it is
		 * the last one
		 */
		list_del(&p_ctx->list_node);
		hash_idx = p_ctx->hash_idx;
		list_for_each_entry(p_ent,
				    &vk->pid_ht[hash_idx].fd_head,
				    list_node)
			if (task_pid_nr(p_ent->p_pid) == pid)
				count++;
	}

	spin_unlock(&vk->ctx_lock);

	return count;
}

static void bcm_vk_free_wkent(struct device *dev, struct bcm_vk_wkent *p_ent)
{
	bcm_vk_sg_free(dev, p_ent->dma, VK_DMA_MAX_ADDRS);

	kfree(p_ent->p_vk2h_msg);
	kfree(p_ent);
}

static void bcm_vk_drain_all_pend(struct device *dev,
				  struct bcm_vk_msg_chan *p_chan,
				  struct bcm_vk_ctx *p_ctx)
{
	uint32_t q_num;
	struct bcm_vk_wkent *p_ent, *p_tmp;

	spin_lock(&p_chan->pendq_lock);
	for (q_num = 0; q_num < p_chan->q_nr; q_num++) {
		list_for_each_entry_safe(p_ent, p_tmp,
					 &p_chan->pendq_head[q_num],
					 list_node) {

			if ((p_ctx == NULL) ||
			    (p_ent->p_ctx->idx == p_ctx->idx)) {
				list_del(&p_ent->list_node);
				bcm_vk_free_wkent(dev, p_ent);
			}
		}
	}
	spin_unlock(&p_chan->pendq_lock);
}

bool bcm_vk_msgq_marker_valid(struct bcm_vk *vk)
{
	uint32_t rdy_marker = 0;
	uint32_t fw_status;

	fw_status = vkread32(vk, BAR_0, BAR_FW_STATUS);

	if ((fw_status & FW_STATUS_ZEPHYR_READY) == FW_STATUS_ZEPHYR_READY)
		rdy_marker = vkread32(vk, BAR_1, VK_BAR1_MSGQ_DEF_RDY);

	return (rdy_marker == VK_BAR1_MSGQ_RDY_MARKER);
}

/*
 * Function to sync up the messages queue info that is provided by BAR0 and BAR1
 */
int bcm_vk_sync_msgq(struct bcm_vk *vk)
{
	struct bcm_vk_msgq *p_msgq = NULL;
	struct device *dev = &vk->pdev->dev;
	uint32_t msgq_off;
	uint32_t num_q;
	struct bcm_vk_msg_chan *chan_list[] = {&vk->h2vk_msg_chan,
					       &vk->vk2h_msg_chan};
	struct bcm_vk_msg_chan *p_chan = NULL;
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
	p_msgq = (struct bcm_vk_msgq *)(vk->bar[1] + msgq_off);

	for (i = 0; i < ARRAY_SIZE(chan_list); i++) {
		p_chan = chan_list[i];
		for (j = 0; j < num_q; j++) {

			p_chan->msgq[j] = p_msgq;

			dev_info(dev,
				 "MsgQ[%d] info - type %d num %d, @ 0x%x, rd_idx %d wr_idx %d, size %d, off 0x%x\n",
				 j,
				 p_chan->msgq[j]->q_type,
				 p_chan->msgq[j]->q_num,
				 p_chan->msgq[j]->q_start_loc,
				 p_chan->msgq[j]->rd_idx,
				 p_chan->msgq[j]->wr_idx,
				 p_chan->msgq[j]->size,
				 p_chan->msgq[j]->next_off);

			p_msgq = (struct bcm_vk_msgq *)
				 ((char *)p_msgq + sizeof(*p_msgq) +
				  p_msgq->next_off);

			rmb(); /* do a read mb to guarantee */
		}
	}

	/* at the end, pass down scratch mem info */
	vkwrite32(vk, vk->tdma_addr >> 32, BAR_1,
		  VK_BAR1_SCRATCH_OFF);
	vkwrite32(vk, (uint32_t)vk->tdma_addr, BAR_1,
		  VK_BAR1_SCRATCH_OFF + 4);
	vkwrite32(vk, PAGE_SIZE, BAR_1, VK_BAR1_SCRATCH_OFF + 8);

	vk->msgq_inited = true;

	return 0;
}

static int bcm_vk_msg_chan_init(struct bcm_vk_msg_chan *p_chan)
{
	int rc = 0;
	uint32_t i;

	mutex_init(&p_chan->msgq_mutex);
	spin_lock_init(&p_chan->pendq_lock);
	for (i = 0; i < VK_MSGQ_MAX_NR; i++)
		INIT_LIST_HEAD(&p_chan->pendq_head[i]);

	return rc;
}

static void bcm_vk_append_pendq(struct bcm_vk_msg_chan *p_chan, uint16_t q_num,
				struct bcm_vk_wkent *p_ent)
{
	spin_lock(&p_chan->pendq_lock);
	list_add_tail(&p_ent->list_node, &p_chan->pendq_head[q_num]);
	spin_unlock(&p_chan->pendq_lock);
}

static void bcm_h2vk_doorbell(struct bcm_vk *vk, uint32_t q_num,
			      uint32_t db_val)
{
	/* press door bell based on q_num */
	vkwrite32(vk,
		  db_val,
		  BAR_0,
		  VK_BAR0_REGSEG_DB_BASE + q_num * VK_BAR0_REGSEG_DB_REG_GAP);
}

static int bcm_h2vk_msg_enqueue(struct bcm_vk *vk, struct bcm_vk_wkent *p_ent)
{
	struct bcm_vk_msg_chan *p_chan = &vk->h2vk_msg_chan;
	struct device *dev = &vk->pdev->dev;
	struct vk_msg_blk *p_src_blk = &p_ent->p_h2vk_msg[0];

	volatile struct vk_msg_blk *p_dst_blk;
	struct bcm_vk_msgq *p_msgq;
	uint32_t q_num = p_src_blk->queue_id;
	uint32_t wr_idx; /* local copy */
	uint32_t i;

	/*
	 * NOTE: DMA portion will be added later.  For now, simply enqueue what
	 * is passed down
	 */

	if (p_ent->h2vk_blks != p_src_blk->size + 1) {
		dev_err(dev, "ent number of blks %d not matching data's %d MsgId[0x%x]: func %d ctx 0x%x\n",
			p_ent->h2vk_blks,
			p_src_blk->size + 1,
			p_src_blk->msg_id,
			p_src_blk->function_id,
			p_src_blk->context_id);
		return -EMSGSIZE;
	}

	p_msgq = p_chan->msgq[q_num];

	rmb(); /* start with a read barrier */
	mutex_lock(&p_chan->msgq_mutex);

	/* if not enough space, return EAGAIN and let app handles it */
	if (VK_MSGQ_AVAIL_SPACE(p_msgq) < p_ent->h2vk_blks) {
		mutex_unlock(&p_chan->msgq_mutex);
		return -EAGAIN;
	}

	/* at this point, mutex is got and it is sure there is enough space */

	wr_idx = p_msgq->wr_idx;

	p_dst_blk = VK_MSGQ_BLK_ADDR(vk->bar[1], p_msgq, wr_idx);
	for (i = 0; i < p_ent->h2vk_blks; i++) {
		*p_dst_blk = *p_src_blk;

#if VK_H2VK_VERIFY_AND_RETRY
		bcm_vk_h2vk_verify_blk(dev, p_src_blk, p_dst_blk);
#endif
		p_src_blk++;
		wr_idx = VK_MSGQ_INC(p_msgq, wr_idx, 1);
		p_dst_blk = VK_MSGQ_BLK_ADDR(vk->bar[1],
					     p_msgq,
					     wr_idx);
	}

	/* flush the write pointer */
	p_msgq->wr_idx = wr_idx;
	wmb(); /* flush */

#if VK_H2VK_VERIFY_AND_RETRY
	bcm_vk_h2vk_verify_idx(dev, "wr_idx", &p_msgq->wr_idx, wr_idx);
#endif

	/* log new info for debugging */
	dev_dbg(dev,
		"MsgQ[%d] [Rd Wr] = [%d %d] blks inserted %d - Q = [u-%d a-%d]/%d\n",
		p_msgq->q_num,
		p_msgq->rd_idx, p_msgq->wr_idx, p_ent->h2vk_blks,
		VK_MSGQ_OCCUPIED(p_msgq),
		VK_MSGQ_AVAIL_SPACE(p_msgq),
		p_msgq->size);

	mutex_unlock(&p_chan->msgq_mutex);

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
	struct bcm_vk_wkent *p_ent;
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

	p_ent = kzalloc(sizeof(struct bcm_vk_wkent) +
			sizeof(struct vk_msg_blk), GFP_KERNEL);
	if (!p_ent)
		return -ENOMEM;

	/* just fill up non-zero data */
	p_ent->p_h2vk_msg[0].function_id = VK_FID_SHUTDOWN;
	p_ent->p_h2vk_msg[0].queue_id = 2; /* use queue 2 */
	p_ent->h2vk_blks = 1; /* always 1 block */

	p_ent->p_h2vk_msg[0].args[0] = shut_type;
	p_ent->p_h2vk_msg[0].args[1] = pid;

	rc = bcm_h2vk_msg_enqueue(vk, p_ent);
	if (rc)
		dev_err(dev,
			"Sending shutdown message to q %d for pid %d fails.\n",
			p_ent->p_h2vk_msg[0].queue_id, pid);

	kfree(p_ent);

	return rc;
}

int bcm_vk_handle_last_sess(struct bcm_vk *vk, struct task_struct *p_pid)
{
	int rc = 0;
	pid_t pid = task_pid_nr(p_pid);
	struct device *dev = &vk->pdev->dev;

	/*
	 * don't send down or do anything if message queue is not initialized
	 * and if it is the reset session, clear it.
	 */
	if (!vk->msgq_inited) {

		if (vk->reset_ppid == p_pid)
			vk->reset_ppid = NULL;
		return -EPERM;
	}

	dev_info(dev, "No more sessions, shut down pid %d\n", pid);

	/* only need to do it if it is not the reset process */
	if (vk->reset_ppid != p_pid)
		rc = bcm_vk_send_shutdown_msg(vk, VK_SHUTDOWN_PID, pid);
	else
		/* reset the pointer if it is exiting last session */
		vk->reset_ppid = NULL;

	return rc;
}

static struct bcm_vk_wkent *bcm_vk_find_pending(struct bcm_vk_msg_chan *p_chan,
						uint16_t q_num,
						uint16_t msg_id,
						unsigned long *map)
{
	bool found = false;
	struct bcm_vk_wkent *p_ent;

	spin_lock(&p_chan->pendq_lock);
	list_for_each_entry(p_ent, &p_chan->pendq_head[q_num], list_node) {

		if (p_ent->p_h2vk_msg[0].msg_id == msg_id) {
			list_del(&p_ent->list_node);
			found = true;
			bitmap_clear(map, msg_id, 1);
			break;
		}
	}
	spin_unlock(&p_chan->pendq_lock);
	return ((found) ? p_ent : NULL);
}

static uint32_t bcm_vk2h_msg_dequeue(struct bcm_vk *vk)
{
	struct device *dev = &vk->pdev->dev;
	struct bcm_vk_msg_chan *p_chan = &vk->vk2h_msg_chan;
	struct vk_msg_blk *p_data;
	volatile struct vk_msg_blk *p_src_blk;
	struct vk_msg_blk *p_dst_blk;
	struct bcm_vk_msgq *p_msgq;
	struct bcm_vk_wkent *p_ent;
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
	mutex_lock(&p_chan->msgq_mutex);
	rmb(); /* start with a read barrier */
	for (q_num = 0; q_num < p_chan->q_nr; q_num++) {
		p_msgq = p_chan->msgq[q_num];

		while (!VK_MSGQ_EMPTY(p_msgq)) {

			/* make a local copy */
			rd_idx = p_msgq->rd_idx;

			/* look at the first block and decide the size */
			p_src_blk = VK_MSGQ_BLK_ADDR(vk->bar[1],
						     p_msgq,
						     rd_idx);

			num_blks = p_src_blk->size + 1;

			p_data = kzalloc(num_blks * VK_MSGQ_BLK_SIZE,
					 GFP_KERNEL);

			if (p_data) {

				/* copy messages and linearize it */
				p_dst_blk = p_data;
				for (j = 0; j < num_blks; j++) {

					*p_dst_blk = *p_src_blk;

					p_dst_blk++;
					rd_idx = VK_MSGQ_INC
							(p_msgq,
							 rd_idx,
							 1);
					p_src_blk = VK_MSGQ_BLK_ADDR
							(vk->bar[1],
							 p_msgq,
							 rd_idx);
				}
				total++;
			} else {
				dev_crit(dev, "Error allocating memory\n");
				/* just keep draining..... */
				rd_idx = VK_MSGQ_INC(p_msgq, rd_idx, num_blks);
			}

			/* flush rd pointer after a message is dequeued */
			p_msgq->rd_idx = rd_idx;
			mb(); /* do both rd/wr as we are extracting data out */

#if VK_H2VK_VERIFY_AND_RETRY
			bcm_vk_h2vk_verify_idx(dev, "rd_idx",
					       &p_msgq->rd_idx, rd_idx);
#endif

			/* log new info for debugging */
			dev_dbg(dev,
				"MsgQ[%d] [Rd Wr] = [%d %d] blks extracted %d - Q = [u-%d a-%d]/%d\n",
				p_msgq->q_num,
				p_msgq->rd_idx, p_msgq->wr_idx, num_blks,
				VK_MSGQ_OCCUPIED(p_msgq),
				VK_MSGQ_AVAIL_SPACE(p_msgq),
				p_msgq->size);

			/* lookup original message in h2vk direction */
			p_ent = bcm_vk_find_pending(&vk->h2vk_msg_chan, q_num,
						    p_data->msg_id,
						    vk->bmap);

			/*
			 * if there is message to does not have prior send,
			 * this is the location to add here
			 */
			if (p_ent) {
				p_ent->vk2h_blks = num_blks;
				p_ent->p_vk2h_msg = p_data;
				bcm_vk_append_pendq(&vk->vk2h_msg_chan,
						    q_num, p_ent);

			} else {
				dev_crit(dev, "Could not find MsgId[0x%x] for resp func %d\n",
					 p_data->msg_id, p_data->function_id);
				kfree(p_data);
			}

		}
	}
	mutex_unlock(&p_chan->msgq_mutex);
	dev_dbg(dev, "total %d drained from queues\n", total);

	return total;
}

/*
 * deferred work queue for draining.  This function is created purely for
 * letting the work done in the work queue
 */
static void bcm_vk2h_wq_handler(struct work_struct *work)
{
	struct bcm_vk *vk = container_of(work, struct bcm_vk, vk2h_wq);
	struct device *dev = &vk->pdev->dev;
	uint32_t tot;

	tot = bcm_vk2h_msg_dequeue(vk);

	if (tot == 0)
		dev_err(dev, "Spurious trigger for workqueue\n");
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
		vk->op_ctx[i].in_use = false;
		vk->op_ctx[i].idx = i;	/* self identity */
		vk->op_ctx[i].p_miscdev = NULL;
	}
	spin_lock_init(&vk->msg_id_lock);
	vk->msg_id = 0;

	/* initialize hash table */
	for (i = 0; i < VK_PID_HT_SZ; i++)
		INIT_LIST_HEAD(&vk->pid_ht[i].fd_head);

	INIT_WORK(&vk->vk2h_wq, bcm_vk2h_wq_handler);
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

	queue_work(vk->vk2h_wq_thread, &vk->vk2h_wq);

skip_schedule_work:
	return IRQ_HANDLED;
}

int bcm_vk_open(struct inode *inode, struct file *p_file)
{
	struct bcm_vk_ctx *p_ctx;
	struct miscdevice *p_miscdev = (struct miscdevice *)p_file->private_data;
	struct bcm_vk *vk = container_of(p_miscdev, struct bcm_vk, miscdev);
	struct device *dev = &vk->pdev->dev;
	int    rc = 0;

	/* get a context and set it up for file */
	p_ctx = bcm_vk_get_ctx(vk, current);
	if (!p_ctx) {
		dev_err(dev, "Error allocating context\n");
		rc = -ENOMEM;
	} else {

		/*
		 * set up context and replace private data with context for
		 * other methods to use.  Reason for the context is because
		 * it is allowed for multiple sessions to open the sysfs, and
		 * for each file open, when upper layer query the response,
		 * only those that are tied to a specific open should be
		 * returened.  The context->idx will be used for such binding
		 */
		p_ctx->p_miscdev = p_miscdev;
		p_file->private_data = p_ctx;
		dev_dbg(dev, "ctx_returned with idx %d, pid %d\n",
			p_ctx->idx, task_pid_nr(p_ctx->p_pid));
	}
	return rc;
}

ssize_t bcm_vk_read(struct file *p_file, char __user *buf, size_t count,
			   loff_t *f_pos)
{
	ssize_t rc = -ENOMSG;
	struct bcm_vk_ctx *p_ctx = p_file->private_data;
	struct bcm_vk *vk = container_of(p_ctx->p_miscdev, struct bcm_vk,
					 miscdev);
	struct device *dev = &vk->pdev->dev;
	struct bcm_vk_msg_chan *p_chan = &vk->vk2h_msg_chan;
	struct bcm_vk_wkent *p_ent = NULL;
	uint32_t q_num;
	uint32_t rsp_length;
	bool found = false;

	dev_dbg(dev, "Buf count %ld, msgq_inited %d\n",
		count, vk->msgq_inited);

	if (!vk->msgq_inited)
		return -EPERM;

	found = false;

	/*
	 * search through the pendq on the vk2h chan, and return only those
	 * that belongs to the same context.  Search is always from the high to
	 * the low priority queues
	 */
	spin_lock(&p_chan->pendq_lock);
	for (q_num = 0; q_num < p_chan->q_nr; q_num++) {
		list_for_each_entry(p_ent, &p_chan->pendq_head[q_num],
				    list_node) {
			if (p_ent->p_ctx->idx == p_ctx->idx) {
				if (count >= p_ent->vk2h_blks *
					     VK_MSGQ_BLK_SIZE) {
					list_del(&p_ent->list_node);
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
	spin_unlock(&p_chan->pendq_lock);

	if (found) {
		/* retrieve the passed down msg_id */
		p_ent->p_vk2h_msg[0].msg_id = p_ent->usr_msg_id;
		rsp_length = p_ent->vk2h_blks * VK_MSGQ_BLK_SIZE;
		if (copy_to_user(buf, p_ent->p_vk2h_msg, rsp_length) == 0)
			rc = rsp_length;

		bcm_vk_free_wkent(dev, p_ent);
	} else if (rc == -EMSGSIZE) {
		struct vk_msg_blk tmp_msg = p_ent->p_vk2h_msg[0];

		/*
		 * in this case, return just the first block, so
		 * that app knows what size it is looking for.
		 */
		tmp_msg.msg_id = p_ent->usr_msg_id;
		tmp_msg.size = p_ent->vk2h_blks - 1;
		if (copy_to_user(buf, &tmp_msg, VK_MSGQ_BLK_SIZE) != 0) {
			dev_err(dev,
				"Error returning first block in -EMSGSIZE case\n");
			rc = -EFAULT;
		}
	}
	return rc;
}

ssize_t bcm_vk_write(struct file *p_file, const char __user *buf,
			    size_t count, loff_t *f_pos)
{
	ssize_t rc = -EPERM;
	struct bcm_vk_ctx *p_ctx = p_file->private_data;
	struct bcm_vk *vk = container_of(p_ctx->p_miscdev, struct bcm_vk,
					 miscdev);
	struct bcm_vk_msgq *p_msgq;
	struct device *dev = &vk->pdev->dev;
	struct bcm_vk_wkent *p_ent;

	dev_dbg(dev, "Msg count %ld, msg_inited %d\n",
		 count, vk->msgq_inited);

	if (!vk->msgq_inited)
		return -EPERM;

	/* first, do sanity check where count should be multiple of basic blk */
	if ((count % VK_MSGQ_BLK_SIZE) != 0) {
		dev_err(dev, "Failure with size %ld not multiple of %ld\n",
			count, VK_MSGQ_BLK_SIZE);
		rc = -EBADR;
		goto bcm_vk_write_err;
	}

	/* allocate the work entry and the buffer */
	p_ent = kzalloc(sizeof(struct bcm_vk_wkent) + count, GFP_KERNEL);
	if (!p_ent) {
		rc = -ENOMEM;
		goto bcm_vk_write_err;
	}

	/* now copy msg from user space, and then formulate the wk ent */
	if (copy_from_user(&p_ent->p_h2vk_msg[0], buf, count))
		goto bcm_vk_write_free_ent;

	p_ent->h2vk_blks = count / VK_MSGQ_BLK_SIZE;
	p_ent->p_ctx = p_ctx;

	/* do a check on the blk size which could not exceed queue space */
	p_msgq = vk->h2vk_msg_chan.msgq[p_ent->p_h2vk_msg[0].queue_id];
	if (p_ent->h2vk_blks > (p_msgq->size - 1)) {
		dev_err(dev, "Blk size %d exceed max queue size allowed %d\n",
			p_ent->h2vk_blks, p_msgq->size - 1);
		rc = -EOVERFLOW;
		goto bcm_vk_write_free_ent;
	}

	/* Use internal message id */
	p_ent->usr_msg_id = p_ent->p_h2vk_msg[0].msg_id;
	rc = bcm_vk_get_msg_id(vk);
	if (rc == VK_MSG_ID_OVERFLOW) {
		dev_err(dev, "msg_id overflow\n");
		rc = -EOVERFLOW;
		goto bcm_vk_write_free_ent;
	}
	p_ent->p_h2vk_msg[0].msg_id = rc;

	dev_dbg(dev,
		"Message ctx id %d, usr_msg_id 0x%x sent msg_id 0x%x\n",
		p_ctx->idx, p_ent->usr_msg_id,
		p_ent->p_h2vk_msg[0].msg_id);

	/* Convert any pointers to sg list */
	if (p_ent->p_h2vk_msg[0].function_id == VK_FID_TRANS_BUF) {
		int num_planes;
		int dir;
		struct _vk_data *data;

		num_planes = p_ent->p_h2vk_msg[0].args[0] & VK_CMD_PLANES_MASK;
		if ((p_ent->p_h2vk_msg[0].args[0] & VK_CMD_MASK) ==
		    VK_CMD_DOWNLOAD) {
			/* Memory transfer from vk device */
			dir = DMA_FROM_DEVICE;
		} else {
			/* Memory transfer to vk device */
			dir = DMA_TO_DEVICE;
		}

		/* Calculate vk_data location */
		/* Go to end of the message */
		data = (struct _vk_data *)
			&(p_ent->p_h2vk_msg[p_ent->p_h2vk_msg[0].size + 1]);
		/* Now back up to the start of the pointers */
		data -= num_planes;

		/* Convert user addresses to DMA SG List */
		rc = bcm_vk_sg_alloc(dev, p_ent->dma, dir, data, num_planes);
		if (rc)
			goto bcm_vk_write_free_ent;
	}

	/*
	 * store wk ent to pending queue until a response is got. This needs to
	 * be done before enqueuing the message
	 */
	bcm_vk_append_pendq(&vk->h2vk_msg_chan, p_ent->p_h2vk_msg[0].queue_id,
			    p_ent);

	rc = bcm_h2vk_msg_enqueue(vk, p_ent);
	if (rc) {
		dev_err(dev, "Fail to enqueue msg to h2vk queue\n");

		/* remove message from pending list */
		p_ent = bcm_vk_find_pending(&vk->h2vk_msg_chan,
					    p_ent->p_h2vk_msg[0].queue_id,
					    p_ent->p_h2vk_msg[0].msg_id,
					    vk->bmap);
		goto bcm_vk_write_free_ent;
	}

	return count;

 bcm_vk_write_free_ent:
	kfree(p_ent);
 bcm_vk_write_err:
	return rc;
}

int bcm_vk_release(struct inode *inode, struct file *p_file)
{
	int ret;
	struct bcm_vk_ctx *p_ctx = p_file->private_data;
	struct bcm_vk *vk = container_of(p_ctx->p_miscdev, struct bcm_vk,
					 miscdev);
	struct device *dev = &vk->pdev->dev;
	struct task_struct *p_pid = p_ctx->p_pid;
	pid_t pid = task_pid_nr(p_pid);

	dev_dbg(dev, "Draining with context idx %d pid %d\n",
		p_ctx->idx, pid);

	bcm_vk_drain_all_pend(&vk->pdev->dev, &vk->h2vk_msg_chan, p_ctx);
	bcm_vk_drain_all_pend(&vk->pdev->dev, &vk->vk2h_msg_chan, p_ctx);

	ret = bcm_vk_free_ctx(vk, p_ctx);
	if (ret == 0)
		ret = bcm_vk_handle_last_sess(vk, p_pid);

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
	vk->vk2h_wq_thread = create_singlethread_workqueue(vk->miscdev.name);
	if (!vk->vk2h_wq_thread) {
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
	destroy_workqueue(vk->vk2h_wq_thread);

	/* drain all pending items */
	bcm_vk_drain_all_pend(&vk->pdev->dev, &vk->h2vk_msg_chan, NULL);
	bcm_vk_drain_all_pend(&vk->pdev->dev, &vk->vk2h_msg_chan, NULL);
	vk->msgq_inited = false;
}

void bcm_vk_trigger_reset(struct bcm_vk *vk)
{
	/* clean up before pressing the door bell */
	bcm_vk_drain_all_pend(&vk->pdev->dev, &vk->h2vk_msg_chan, NULL);
	bcm_vk_drain_all_pend(&vk->pdev->dev, &vk->vk2h_msg_chan, NULL);
	vk->msgq_inited = false;
	vkwrite32(vk, 0, BAR_1, VK_BAR1_MSGQ_DEF_RDY);

	bcm_h2vk_doorbell(vk, VK_BAR0_RESET_DB_NUM, VK_BAR0_RESET_DB_VAL);

	/* clear 4096 bits of bitmap */
	bitmap_clear(vk->bmap, 0, VK_MSG_ID_BITMAP_SIZE);
}
