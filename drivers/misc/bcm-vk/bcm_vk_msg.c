// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2018 Broadcom
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>

#include "bcm_vk.h"
#include "bcm_vk_msg.h"

/*
 * allocate a ctx per file struct
 */
static struct bcm_vk_ctx *bcm_vk_get_ctx(struct bcm_vk *vk)
{
	struct bcm_vk_ctx *p_ctx = NULL;
	uint32_t i;

	spin_lock(&vk->ctx_lock);

	for (i = 0; i < VK_CMPT_CTX_MAX; i++) {
		if (!vk->op_ctx[i].in_use) {
			vk->op_ctx[i].in_use = true;
			p_ctx = &vk->op_ctx[i];
			break;
		}
	}

	spin_unlock(&vk->ctx_lock);

	return p_ctx;
}

static uint16_t bcm_vk_get_msg_id(struct bcm_vk *vk)
{
	uint16_t rc;

	spin_lock(&vk->msg_id_lock);
	vk->msg_id = (++vk->msg_id & 0x0FFF);
	rc = vk->msg_id;
	spin_unlock(&vk->msg_id_lock);

	return rc;
}

static void bcm_vk_free_ctx(struct bcm_vk *vk, struct bcm_vk_ctx *p_ctx)
{
	uint32_t idx;

	if (p_ctx == NULL) {
		dev_err(&vk->pdev->dev, "NULL context detected\n");
		return;
	}
	idx = p_ctx->idx;

	spin_lock(&vk->ctx_lock);

	if (!vk->op_ctx[idx].in_use) {
		dev_err(&vk->pdev->dev, "Freeing context id[%d] not in use!\n",
			idx);
	} else {
		vk->op_ctx[idx].in_use = false;
		vk->op_ctx[idx].p_miscdev = NULL;
	}

	spin_unlock(&vk->ctx_lock);
}

static void bcm_vk_free_wkent(struct bcm_vk_wkent *p_ent)
{
	kfree(p_ent->p_vk2h_msg);
	kfree(p_ent);

	/* TO_DO: add DMA buffer related cleaning */
}

static void bcm_vk_drain_all_pend(struct bcm_vk_msg_chan *p_chan,
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
				bcm_vk_free_wkent(p_ent);
			}
		}
	}
	spin_unlock(&p_chan->pendq_lock);
}

/*
 * Function to sync up the messages queue info that is provided by BAR0 and BAR1
 * TO_DO: This function is based on the current valkyrie-emu definition and imp,
 *	  which properly need final adjustments.
 */
static int bcm_vk_sync_msgq(struct bcm_vk *vk)
{
	struct bcm_vk_msgq *p_msgq = NULL;
	struct device *dev = &vk->pdev->dev;
	uint32_t msgq_off;
	uint32_t num_q;
	struct bcm_vk_msg_chan *chan_list[] = {&vk->h2vk_msg_chan,
					       &vk->vk2h_msg_chan};
	struct bcm_vk_msg_chan *p_chan = NULL;
	int i, j;

	if (!vkread32(vk, BAR_1, VK_BAR1_DDRSEG_MSGQ_DEF_RDY)) {
		dev_err(dev, "BAR1 not initialized.\n");
		return -EPERM;
	}

	msgq_off = vkread32(vk, BAR_1, VK_BAR1_DDRSEG_MSGQ_CTRL_OFF);

	/* each side is always half the total  */
	num_q = vk->h2vk_msg_chan.q_nr = vk->vk2h_msg_chan.q_nr =
		vkread32(vk, BAR_1, VK_BAR1_DDRSEG_MSGQ_NR) / 2;

	/* first msgq location */
	p_msgq = (struct bcm_vk_msgq *)(vk->bar[1] + msgq_off);

	for (i = 0; i < ARRAY_SIZE(chan_list); i++) {
		p_chan = chan_list[i];
		for (j = 0; j < num_q; j++) {

			p_chan->msgq[j] = p_msgq;

			dev_info(dev, "MsgQ[%d] info - type %d num %d, @ 0x%x, rd_idx %d wr_idx %d, size %d, off 0x%x\n",
				 j, p_chan->msgq[j]->q_type,
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

static void bcm_h2vk_doorbell(struct bcm_vk *vk, uint32_t q_num)
{
	/* assume door bell to be adjacent for now? */
	vkwrite32(vk,
		  1,
		  BAR_0,
		  VK_BAR0_REGSEG_DB_BASE + (q_num * sizeof(uint32_t)));
}

static int bcm_h2vk_msg_enqueue(struct bcm_vk *vk, struct bcm_vk_wkent *p_ent)
{
	struct bcm_vk_msg_chan *p_chan = &vk->h2vk_msg_chan;
	struct device *dev = &vk->pdev->dev;
	struct bcm_vk_msg_blk *p_src_blk = &p_ent->p_h2vk_msg[0];

	volatile struct bcm_vk_msg_blk *p_dst_blk;
	struct bcm_vk_msgq *p_msgq;
	uint32_t q_num = p_src_blk->queue_id;
	uint32_t wr_idx; /* local copy */
	uint32_t delay_cnt = 0;
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
	/* just wait for enough space... this may need some revision later */
	mutex_lock(&p_chan->msgq_mutex);
	while (VK_MSGQ_AVAIL_SPACE(p_msgq) < p_ent->h2vk_blks) {
		mutex_unlock(&p_chan->msgq_mutex);
		msleep(1000);
		delay_cnt++;
		mutex_lock(&p_chan->msgq_mutex);
	}

	if (delay_cnt >= 2)
		dev_err(dev, "Waiting for room exceeds 1s\n");

	/* at this point, mutex is got and it is sure there is enough space */

	wr_idx = p_msgq->wr_idx;

	p_dst_blk = VK_MSGQ_BLK_ADDR(vk->bar[1], p_msgq, wr_idx);
	for (i = 0; i < p_ent->h2vk_blks; i++) {
		*p_dst_blk = *p_src_blk;

		p_src_blk++;
		wr_idx = VK_MSGQ_INC(p_msgq, wr_idx, 1);
		p_dst_blk = VK_MSGQ_BLK_ADDR(vk->bar[1],
					     p_msgq,
					     wr_idx);
	}

	/* flush the write pointer */
	p_msgq->wr_idx = wr_idx;
	wmb(); /* flush */

	/* log new info for debugging */
	dev_info(dev, "MsgQ[%d] [Rd Wr] = [%d %d] blks inserted %d - Q = [u-%d a-%d]/%d\n",
		 p_msgq->q_num,
		 p_msgq->rd_idx, p_msgq->wr_idx, p_ent->h2vk_blks,
		 VK_MSGQ_OCCUPIED(p_msgq),
		 VK_MSGQ_AVAIL_SPACE(p_msgq),
		 p_msgq->size);

	mutex_unlock(&p_chan->msgq_mutex);

	/* press door bell based on queue number */
	bcm_h2vk_doorbell(vk, q_num);

	return 0;
}

static struct bcm_vk_wkent *bcm_vk_find_pending(struct bcm_vk_msg_chan *p_chan,
						uint16_t q_num,
						uint16_t msg_id)
{
	bool found = false;
	struct bcm_vk_wkent *p_ent;

	spin_lock(&p_chan->pendq_lock);
	list_for_each_entry(p_ent, &p_chan->pendq_head[q_num], list_node) {

		if (p_ent->p_h2vk_msg[0].msg_id == msg_id) {
			list_del(&p_ent->list_node);
			found = true;
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
	struct bcm_vk_msg_blk *p_data;
	volatile struct bcm_vk_msg_blk *p_src_blk;
	struct bcm_vk_msg_blk *p_dst_blk;
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

			/* log new info for debugging */
			dev_info(dev, "MsgQ[%d] [Rd Wr] = [%d %d] blks extracted %d - Q = [u-%d a-%d]/%d\n",
				 p_msgq->q_num,
				 p_msgq->rd_idx, p_msgq->wr_idx, num_blks,
				 VK_MSGQ_OCCUPIED(p_msgq),
				 VK_MSGQ_AVAIL_SPACE(p_msgq),
				 p_msgq->size);

			/* lookup original message in h2vk direction */
			p_ent = bcm_vk_find_pending(&vk->h2vk_msg_chan, q_num,
						    p_data->msg_id);

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
	dev_info(dev, "total %d drained from queues\n", total);

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

	INIT_WORK(&vk->vk2h_wq, bcm_vk2h_wq_handler);
	return rc;
}

irqreturn_t bcm_vk_irqhandler(int irq, void *dev_id)
{
	struct bcm_vk *vk = dev_id;

	schedule_work(&vk->vk2h_wq);

	return IRQ_HANDLED;
}

int bcm_vk_open(struct inode *inode, struct file *p_file)
{
	struct bcm_vk_ctx *p_ctx;
	struct miscdevice *p_miscdev = (struct miscdevice *)p_file->private_data;
	struct bcm_vk *vk = container_of(p_miscdev, struct bcm_vk, miscdev);
	struct device *dev = &vk->pdev->dev;
	int    rc = 0;

	dev_info(dev, "%s\n", __func__);

	/* get a context and set it up for file */
	p_ctx = bcm_vk_get_ctx(vk);
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
		dev_info(dev, "ctx_returned with idx %d\n", p_ctx->idx);
	}
	return rc;
}

ssize_t bcm_vk_read(struct file *p_file, char __user *buf, size_t count,
			   loff_t *f_pos)
{
	ssize_t rc = 0;
	struct bcm_vk_ctx *p_ctx = p_file->private_data;
	struct bcm_vk *vk = container_of(p_ctx->p_miscdev, struct bcm_vk,
					 miscdev);
	struct device *dev = &vk->pdev->dev;
	struct bcm_vk_msg_chan *p_chan = &vk->vk2h_msg_chan;
	struct bcm_vk_wkent *p_ent;
	uint32_t q_num;
	uint32_t rsp_length;
	bool found = false;

	dev_info(dev, "%s(): called with buf count %ld\n", __func__, count);

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
			if ((p_ent->p_ctx->idx == p_ctx->idx) &&
			    (count >= p_ent->vk2h_blks * VK_MSGQ_BLK_SIZE)) {
				list_del(&p_ent->list_node);
				found = true;
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

		bcm_vk_free_wkent(p_ent);
	}
	return rc;
}

ssize_t bcm_vk_write(struct file *p_file, const char __user *buf,
			    size_t count, loff_t *f_pos)
{
	ssize_t rc = 0;
	struct bcm_vk_ctx *p_ctx = p_file->private_data;
	struct bcm_vk *vk = container_of(p_ctx->p_miscdev, struct bcm_vk,
					 miscdev);
	struct bcm_vk_msgq *p_msgq;
	struct device *dev = &vk->pdev->dev;
	struct bcm_vk_wkent *p_ent;

	dev_info(dev, "%s() called with msg count %ld\n", __func__, count);

	/* first, do sanity check where count should be multiple of basic blk */
	if ((count % VK_MSGQ_BLK_SIZE) != 0) {
		dev_err(dev, "Failure with size %ld not multiple of %ld\n",
			count, VK_MSGQ_BLK_SIZE);
		goto bcm_vk_write_err;
	}

	/* allocate the work entry and the buffer */
	p_ent = kzalloc(sizeof(struct bcm_vk_wkent) + count, GFP_KERNEL);
	if (!p_ent)
		goto bcm_vk_write_err;


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
		goto bcm_vk_write_free_ent;
	}

	/* Use internal message id */
	p_ent->usr_msg_id = p_ent->p_h2vk_msg[0].msg_id;
	p_ent->p_h2vk_msg[0].msg_id = bcm_vk_get_msg_id(vk);

	dev_info(dev, "Message ctx id %d, usr_msg_id 0x%x sent msg_id 0x%x\n",
		 p_ctx->idx, p_ent->usr_msg_id,
		 p_ent->p_h2vk_msg[0].msg_id);

	/*
	 * store wk ent to pending queue until a response is got. This needs to
	 * be done before enqueuing the message
	 */
	bcm_vk_append_pendq(&vk->h2vk_msg_chan, p_ent->p_h2vk_msg[0].queue_id,
			    p_ent);

	if (bcm_h2vk_msg_enqueue(vk, p_ent)) {
		dev_err(dev, "Fail to enqueue msg to h2vk queue\n");

		/* remove message from pending list */
		p_ent = bcm_vk_find_pending(&vk->h2vk_msg_chan,
					    p_ent->p_h2vk_msg[0].queue_id,
					    p_ent->p_h2vk_msg[0].msg_id);
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
	struct bcm_vk_ctx *p_ctx = p_file->private_data;
	struct bcm_vk *vk = container_of(p_ctx->p_miscdev, struct bcm_vk,
					 miscdev);
	struct device *dev = &vk->pdev->dev;

	dev_info(dev, "Draining with context idx %d\n", p_ctx->idx);

	bcm_vk_drain_all_pend(&vk->h2vk_msg_chan, p_ctx);
	bcm_vk_drain_all_pend(&vk->vk2h_msg_chan, p_ctx);

	bcm_vk_free_ctx(vk, p_ctx);
	return 0;
}

int bcm_vk_msg_init(struct bcm_vk *vk)
{
	struct device *dev = &vk->pdev->dev;
	int err = 0;

	if (bcm_vk_data_init(vk)) {
		dev_err(dev, "Error initializing internal data structures");
		err = -EINVAL;
		goto err_out;
	}

	if (bcm_vk_msg_chan_init(&vk->h2vk_msg_chan) ||
	    bcm_vk_msg_chan_init(&vk->vk2h_msg_chan)) {
		dev_err(dev, "Error initializing communication channel");
		err = -EIO;
		goto err_out;
	}

	/* read msgq info */
	if (bcm_vk_sync_msgq(vk)) {
		dev_err(dev, "Error reading comm msg Q info");
		err = -EIO;
		goto err_out;
	}

err_out:
	return err;
}

void bcm_vk_msg_remove(struct bcm_vk *vk)
{
	/* drain all pending items */
	bcm_vk_drain_all_pend(&vk->h2vk_msg_chan, NULL);
	bcm_vk_drain_all_pend(&vk->vk2h_msg_chan, NULL);
}
