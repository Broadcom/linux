// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2018 Broadcom
 */

#include <linux/aer.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvme-lpm-ssr.h>
#include <linux/sizes.h>
#include <linux/types.h>

#include "nvme-lpm.h"

#define ADMIN_CMD_TIMEOUT_MS    50  /* Admin command timeout(ms) */
#define IO_CMD_TIMEOUT_MS       2   /* IO command timeout(ms) */
#define SHUTDOWN_TIMEOUT_SEC    5   /* controller shutdown timeout(seconds) */
#define MIN_PAGE_SHIFT_CTRL     12  /* Minimum page shift supported by NVMe */
#define MAX_IO_QPAIR            32

/* Structure shared with CRMU for completion polling */
struct nvme_lpm_data {
	unsigned int used_queues;
	unsigned int q_depth;
	unsigned int sq_addr[MAX_IO_QPAIR + 1];
	unsigned int cq_addr[MAX_IO_QPAIR + 1];
	unsigned int last_cmd_id[MAX_IO_QPAIR + 1];
	unsigned int cqe_seen[MAX_IO_QPAIR + 1];
	unsigned int bar;
	uint8_t live_backup_state;
};

struct memory_window {
	u64 start_addr;
	u64 offset;
};

/* FIXME: have to get memory map from kernel to make following table generic */
static const struct memory_window memory_map[] = {
	{
		.start_addr = 0x80000000,
		.offset = 0x80000000,
	},
	{
		.start_addr = 0x880000000,
		.offset = 0x800000000,
	},
	{
		.start_addr = 0x9000000000,
		.offset = 0x400000000,
	},
	{
		.start_addr = 0xa000000000,
		.offset = 0x400000000,
	},
};

static int get_current_memmap(phys_addr_t memaddr)
{
	int i;
	u64 start_addr, end_addr;

	for (i = 0; i < ARRAY_SIZE(memory_map); i++) {
		start_addr = memory_map[i].start_addr;
		end_addr = memory_map[i].start_addr + memory_map[i].offset - 1;
		if (memaddr >= start_addr && memaddr <= end_addr)
			return i;
	}

	return -EINVAL;
}

static void check_memaddr(phys_addr_t *memaddr, unsigned int *current_memmap)
{
	int ret;

	ret = get_current_memmap(*memaddr);
	if (ret < 0 && *current_memmap < (ARRAY_SIZE(memory_map) - 1)) {
		(*current_memmap)++;
		*memaddr = memory_map[*current_memmap].start_addr;
	}
}

static void nvme_init_queue(struct nvme_queue *nvmeq, u16 qid)
{
	struct nvme_dev *dev = nvmeq->dev;
	unsigned long flags;

	spin_lock_irqsave(&nvmeq->q_lock, flags);
	nvmeq->sq_tail = 0;
	nvmeq->cq_head = 0;
	nvmeq->cq_phase = 1;
	nvmeq->poll_done = false;
	nvmeq->cmd_id = 0;
	nvmeq->q_db = (u32 *)dev->dbs + (qid * 2 * dev->db_stride);
	memset((void *)nvmeq->cqes, 0, CQ_SIZE(nvmeq->q_depth));
	dev->online_queues++;
	dev_dbg(dev->dev, "QP-%d init'ed", qid);
	spin_unlock_irqrestore(&nvmeq->q_lock, flags);
}

/* We read the CQE phase first to check if the rest of the entry is valid */
static inline bool nvme_cqe_valid(struct nvme_queue *nvmeq, u16 head,
				  u16 phase)
{
	return (le16_to_cpu(nvmeq->cqes[head].status) & 1) == phase;
}

static inline void nvme_ring_cq_doorbell(struct nvme_queue *nvmeq)
{
	u16 head = nvmeq->cq_head;

	if (likely(nvmeq->cq_vector >= 0))
		writel(head, (u32 *)nvmeq->q_db + nvmeq->dev->db_stride);
}

static inline void nvme_handle_cqe(struct nvme_queue *nvmeq,
				   struct nvme_completion *cqe)
{
	if (unlikely(cqe->command_id >= nvmeq->q_depth)) {
		dev_warn(nvmeq->dev->dev,
			"invalid id %d completed on queue %d\n",
			cqe->command_id, le16_to_cpu(cqe->sq_id));
		return;
	}

	if (unlikely(nvmeq->qid == 0 &&
			cqe->command_id >= NVME_AQ_BLK_MQ_DEPTH)) {
		dev_warn(nvmeq->dev->dev,
			"invalid id %d completed on queue %d\n",
			cqe->command_id, le16_to_cpu(cqe->sq_id));
		return;
	}

	if ((le16_to_cpu(cqe->status) & 0xfe))
		dev_warn(nvmeq->dev->dev,
			"Error %#x seen for id %d on queue %d\n",
			le16_to_cpu(cqe->status), cqe->command_id,
			le16_to_cpu(cqe->sq_id));

	nvmeq->cqe_seen++;
}

static inline bool nvme_read_cqe(struct nvme_queue *nvmeq,
				 struct nvme_completion *cqe)
{
	if (nvme_cqe_valid(nvmeq, nvmeq->cq_head, nvmeq->cq_phase)) {
		*cqe = nvmeq->cqes[nvmeq->cq_head];

		if (++nvmeq->cq_head == nvmeq->q_depth) {
			nvmeq->cq_head = 0;
			nvmeq->cq_phase = !nvmeq->cq_phase;
		}
		return true;
	}
	return false;
}

static int nvme_poll(struct nvme_queue *nvmeq, unsigned int tag)
{
	struct nvme_dev *dev = nvmeq->dev;
	struct nvme_lpm_data *data = dev->shared_data;
	bool found = false;
	unsigned int consumed = 0;
	unsigned long flags;
	struct nvme_completion cqe;

	if (!nvme_cqe_valid(nvmeq, nvmeq->cq_head, nvmeq->cq_phase))
		return 0;

	spin_lock_irqsave(&nvmeq->q_lock, flags);

	while (nvme_read_cqe(nvmeq, &cqe)) {
		nvme_handle_cqe(nvmeq, &cqe);
		consumed++;

		if (tag == cqe.command_id) {
			found = true;
			break;
		}
	}

	if (consumed)
		nvme_ring_cq_doorbell(nvmeq);

	/* Keep on updating number of polled IO CQ entries */
	if (nvmeq->qid)
		data->cqe_seen[nvmeq->qid] = nvmeq->cqe_seen;

	spin_unlock_irqrestore(&nvmeq->q_lock, flags);

	return found;
}

static int __nvme_submit_cmd_sync(struct nvme_command *cmd,
				  struct nvme_queue *q,
				  union nvme_result *result,
				  unsigned int timeout)
{
	struct nvme_completion *cqe;
	int cqe_num, ret = 0;
	u16 tail = q->sq_tail;
	unsigned int cmd_id = q->cmd_id;
	unsigned int cmd_timeout = timeout ? timeout : ADMIN_CMD_TIMEOUT_MS;
	unsigned long flags;
	unsigned int max_cmd_id = (q->qid) ? (q->q_depth)
					   : NVME_AQ_BLK_MQ_DEPTH;

	cmd->common.command_id = (++cmd_id == max_cmd_id) ?
				 (q->cmd_id = 1) : (q->cmd_id = cmd_id);

	spin_lock_irqsave(&q->q_lock, flags);
	memcpy(&q->sq_cmds[tail], cmd, sizeof(*cmd));

	if (++tail == q->q_depth)
		tail = 0;

	/* ring SQ tail doorbell */
	writel(tail, q->q_db);
	q->sq_tail = tail;
	spin_unlock_irqrestore(&q->q_lock, flags);

	while (!ret && cmd_timeout--) {
		ret = nvme_poll(q, cmd->identify.command_id);
		usleep_range(1000, 1010);
	}

	if (!ret) {
		dev_err(q->dev->dev, "nvme SQ%d command-%d timed out: %d!",
			q->qid, cmd->identify.command_id, ret);
		return -ETIMEDOUT;
	}
	dev_dbg(q->dev->dev, "nvme SQ%d command-%d done!: %d",
		q->qid, cmd->identify.command_id, cmd_timeout);

	if (result) {
		cqe_num = (q->cq_head == 0) ? max_cmd_id : (q->cq_head - 1);
		cqe = &q->cqes[cqe_num];
		memcpy(result, &cqe->result, sizeof(*result));
	}

	return 0;
}

static int nvme_submit_cmd_sync(struct nvme_command *cmd, struct nvme_queue *q,
				union nvme_result *result)
{
	return __nvme_submit_cmd_sync(cmd, q, result, 0);
}

static int nvme_submit_cmd(struct nvme_command *cmd, struct nvme_queue *q)
{
	u16 tail = q->sq_tail;
	unsigned int cmd_id = q->cmd_id;

	cmd->common.command_id = (++cmd_id == q->dev->q_depth) ?
				 (q->cmd_id = 1) : (q->cmd_id = cmd_id);

	memcpy(&q->sq_cmds[tail], cmd, sizeof(*cmd));

	if (++tail == q->q_depth) {
		dev_info(q->dev->dev, "nvme IO SQ%d command-%d done!",
					q->qid, cmd->rw.command_id);
		dev_info(q->dev->dev, "Queue %d is full\n", q->qid);
		tail = 0;
		return -EBUSY;
	}
	q->sq_tail = tail;
	return 0;
}

static int nvme_identify_controller(struct nvme_ctrl *ctrl,
				    struct nvme_id_ctrl **id)
{
	struct nvme_dev *dev = to_nvme_dev(ctrl);
	struct nvme_queue *q =  dev->queues[0];
	struct nvme_command c = { };
	dma_addr_t id_phys;

	memset(&c, 0, sizeof(c));
	c.identify.opcode = nvme_admin_identify;
	c.identify.cns = NVME_ID_CNS_CTRL;

	*id = dmam_alloc_coherent(dev->dev, sizeof(struct nvme_id_ctrl),
				&id_phys, GFP_KERNEL);
	if (!*id)
		return -ENOMEM;
	c.identify.dptr.prp1 = id_phys;

	return nvme_submit_cmd_sync(&c, q, NULL);
}

static int nvme_set_features(struct nvme_ctrl *ctrl, unsigned int fid,
			     unsigned int dword11, u32 *result)
{
	struct nvme_command c;
	struct nvme_dev *dev = to_nvme_dev(ctrl);
	struct nvme_queue *q =  dev->queues[0];
	union nvme_result res;
	int ret;

	memset(&c, 0, sizeof(c));
	c.features.opcode = nvme_admin_set_features;
	c.features.fid = cpu_to_le32(fid);
	c.features.dword11 = cpu_to_le32(dword11);

	ret = nvme_submit_cmd_sync(&c, q, &res);
	if (ret >= 0 && result)
		*result = le32_to_cpu(res.u32);

	return ret;
}

static int nvme_set_qcount(struct nvme_ctrl *ctrl, int *count)
{
	u32 q_count = (*count - 1) | ((*count - 1) << 16);
	u32 result = 0, ret;
	int nr_io_queues;

	dev_info(ctrl->dev, "requested Qs: %d", *count);
	ret = nvme_set_features(ctrl, NVME_FEAT_NUM_QUEUES, q_count, &result);

	if (ret < 0) {
		dev_err(ctrl->dev, "Could not set queue count (%d)\n", *count);
		*count = 0;
	} else {
		nr_io_queues = min(result & 0xffff, result >> 16) + 1;
		*count = min(*count, nr_io_queues);
	}

	return 0;
}

static int adapter_alloc_cq(struct nvme_dev *dev, u16 qid,
			    struct nvme_queue *nvmeq)
{
	struct nvme_command c;
	int flags = NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED;
	struct nvme_queue *q = dev->queues[0];

	/*
	 * Note: we (ab)use the fact that the prp fields survive if no data
	 * is attached to the request.
	 */
	memset(&c, 0, sizeof(c));
	c.create_cq.opcode = nvme_admin_create_cq;
	c.create_cq.prp1 = cpu_to_le64(nvmeq->cq_dma_addr);
	c.create_cq.cqid = cpu_to_le16(qid);
	c.create_cq.qsize = cpu_to_le16(nvmeq->q_depth - 1);
	c.create_cq.cq_flags = cpu_to_le16(flags);
	c.create_cq.irq_vector = cpu_to_le16(nvmeq->cq_vector);

	return nvme_submit_cmd_sync(&c, q, NULL);
}

static int adapter_alloc_sq(struct nvme_dev *dev, u16 qid,
			    struct nvme_queue *nvmeq)
{
	struct nvme_command c;
	int flags = NVME_QUEUE_PHYS_CONTIG;
	struct nvme_queue *q = dev->queues[0];

	/*
	 * Note: we (ab)use the fact that the prp fields survive if no data
	 * is attached to the request.
	 */
	memset(&c, 0, sizeof(c));
	c.create_sq.opcode = nvme_admin_create_sq;
	c.create_sq.prp1 = cpu_to_le64(nvmeq->sq_dma_addr);
	c.create_sq.sqid = cpu_to_le16(qid);
	c.create_sq.qsize = cpu_to_le16(nvmeq->q_depth - 1);
	c.create_sq.sq_flags = cpu_to_le16(flags);
	c.create_sq.cqid = cpu_to_le16(qid);

	return nvme_submit_cmd_sync(&c, q, NULL);
}

static int adapter_delete_queue(struct nvme_dev *dev, u8 opcode, u16 id)
{
	struct nvme_command c;
	struct nvme_queue *q =  dev->queues[0];

	memset(&c, 0, sizeof(c));
	c.delete_queue.opcode = opcode;
	c.delete_queue.qid = cpu_to_le16(id);

	return nvme_submit_cmd_sync(&c, q, NULL);
}

static int adapter_delete_cq(struct nvme_dev *dev, u16 cqid)
{
	return adapter_delete_queue(dev, nvme_admin_delete_cq, cqid);
}

static int adapter_delete_sq(struct nvme_dev *dev, u16 sqid)
{
	return adapter_delete_queue(dev, nvme_admin_delete_sq, sqid);
}

static unsigned long db_bar_size(struct nvme_dev *dev,
				 unsigned int nr_io_queues)
{
	return NVME_REG_DBS + ((nr_io_queues + 1) * 8 * dev->db_stride);
}

static int nvme_remap_bar(struct nvme_dev *dev, unsigned long size)
{
	struct pci_dev *pdev = to_pci_dev(dev->dev);

	if (size <= dev->bar_mapped_size)
		return 0;
	if (size > pci_resource_len(pdev, 0))
		return -ENOMEM;
	if (dev->bar)
		iounmap(dev->bar);
	dev->bar = ioremap(pci_resource_start(pdev, 0), size);
	if (!dev->bar) {
		dev->bar_mapped_size = 0;
		return -ENOMEM;
	}
	dev->bar_mapped_size = size;
	dev->dbs = dev->bar + NVME_REG_DBS;

	return 0;
}

static struct nvme_queue *nvme_alloc_queue(struct nvme_dev *dev, int qid,
					   int depth)
{
	struct nvme_queue *nvmeq = devm_kzalloc(dev->dev,
						sizeof(*nvmeq), GFP_KERNEL);

	if (!nvmeq)
		return NULL;

	/* Admin/IO completion queue allocation */
	nvmeq->cqes = dma_alloc_coherent(dev->dev, CQ_SIZE(depth),
					 &nvmeq->cq_dma_addr, GFP_KERNEL);
	dev_dbg(dev->dev, "CQ%d @ 0x%llx\n", qid, nvmeq->cq_dma_addr);
	if (!nvmeq->cqes)
		goto free_nvmeq;

	/* Admin/IO submission command queues allocation */
	nvmeq->sq_cmds = dma_alloc_coherent(dev->dev, SQ_SIZE(depth),
					    &nvmeq->sq_dma_addr, GFP_KERNEL);
	if (!nvmeq->sq_cmds)
		goto free_nvme_cq;
	dev_dbg(dev->dev, "SQ%d @ 0x%llx\n", qid, nvmeq->sq_dma_addr);

	nvmeq->dev = dev;
	spin_lock_init(&nvmeq->q_lock);
	nvmeq->cq_head = 0;
	nvmeq->cq_phase = 1;
	nvmeq->q_db = (u32 *)dev->dbs + (qid * 2 * dev->db_stride);
	nvmeq->q_depth = depth;
	nvmeq->qid = qid;
	nvmeq->cq_vector = -1;
	dev->queues[qid] = nvmeq;
	dev->ctrl.queue_count++;

	return nvmeq;

 free_nvme_cq:
	dma_free_coherent(dev->dev, CQ_SIZE(depth), (void *)nvmeq->cqes,
			  nvmeq->cq_dma_addr);
 free_nvmeq:
	return NULL;
}

static int nvme_create_queue(struct nvme_queue *nvmeq, int qid)
{
	struct nvme_dev *dev = nvmeq->dev;
	int result;

	nvmeq->cq_vector = qid - 1;
	result = adapter_alloc_cq(dev, qid, nvmeq);
	if (result < 0)
		return result;

	result = adapter_alloc_sq(dev, qid, nvmeq);
	if (result < 0)
		goto release_cq;

	nvme_init_queue(nvmeq, qid);

	return result;

 release_cq:
	adapter_delete_cq(dev, qid);
	return result;
}

static int nvme_create_io_queues(struct nvme_dev *dev)
{
	unsigned int i, max;
	int ret = 0;

	for (i = dev->ctrl.queue_count; i <= dev->max_qid; i++) {
		/* vector == qid - 1, match nvme_create_queue */
		if (!nvme_alloc_queue(dev, i, dev->q_depth)) {
			ret = -ENOMEM;
			break;
		}
	}

	max = min(dev->max_qid, dev->ctrl.queue_count - 1);
	for (i = dev->online_queues; i <= max; i++) {
		ret = nvme_create_queue(dev->queues[i], i);
		if (ret)
			break;
	}

	/*
	 * Ignore failing Create SQ/CQ commands, we can continue with less
	 * than the desired amount of queues, and even a controller without
	 * I/O queues can still be used to issue admin commands.  This might
	 * be useful to upgrade a buggy firmware for example.
	 */
	return ret >= 0 ? 0 : ret;
}

static int nvme_setup_io_queues(struct nvme_dev *dev)
{
	struct nvme_queue *adminq = dev->queues[0];
	int result, nr_io_queues;
	unsigned long size;

	nr_io_queues = MAX_IO_QPAIR;
	result = nvme_set_qcount(&dev->ctrl, &nr_io_queues);
	if (result < 0) {
		dev_err(dev->dev, "nvme_set_queue_count failed\n");
		return result;
	}

	if (nr_io_queues == 0)
		return 0;

	do {
		size = db_bar_size(dev, nr_io_queues);
		result = nvme_remap_bar(dev, size);
		if (!result)
			break;
		if (!--nr_io_queues)
			return -ENOMEM;
	} while (1);
	adminq->q_db = dev->dbs;

	dev->max_qid = nr_io_queues;

	return nvme_create_io_queues(dev);
}

static int nvme_ctrl_init_identify(struct nvme_ctrl *ctrl,
				   struct nvme_id_ctrl **id)
{
	u64 cap;
	int ret, page_shift;

	ret = ctrl->ops->reg_read32(ctrl, NVME_REG_VS, &ctrl->vs);
	if (ret) {
		dev_err(ctrl->dev, "Reading VS failed (%d)\n", ret);
		return ret;
	}

	if (ctrl->vs >= NVME_VS(1, 1, 0))
		ctrl->subsystem = NVME_CAP_NSSRC(cap);

	ret = ctrl->ops->reg_read64(ctrl, NVME_REG_CAP, &cap);
	if (ret) {
		dev_err(ctrl->dev, "Reading CAP failed (%d)\n", ret);
		return ret;
	}
	page_shift = NVME_CAP_MPSMIN(cap) + 12;

	ret = nvme_identify_controller(ctrl, id);
	if (ret) {
		dev_err(ctrl->dev, "Identify Controller failed (%d)\n", ret);
		return -EIO;
	}

	return 0;
}

static int nvme_pci_configure_admin_queue(struct nvme_dev *dev)
{
	int result;
	u32 aqa;
	struct nvme_queue *nvmeq;

	result = nvme_remap_bar(dev, db_bar_size(dev, 0));
	if (result < 0)
		return result;

	dev->subsystem = readl(dev->bar + NVME_REG_VS) >= NVME_VS(1, 1, 0) ?
				NVME_CAP_NSSRC(dev->ctrl.cap) : 0;

	if (dev->subsystem &&
	    (readl(dev->bar + NVME_REG_CSTS) & NVME_CSTS_NSSRO))
		writel(NVME_CSTS_NSSRO, dev->bar + NVME_REG_CSTS);

	result = nvme_disable_ctrl(&dev->ctrl, dev->ctrl.cap);
	if (result < 0)
		return result;

	nvmeq = dev->queues[0];
	if (!nvmeq) {
		nvmeq = nvme_alloc_queue(dev, 0, NVME_AQ_DEPTH);
		if (!nvmeq)
			return -ENOMEM;
	}

	aqa = nvmeq->q_depth - 1;
	aqa |= aqa << 16;

	writel(aqa, dev->bar + NVME_REG_AQA);
	lo_hi_writeq(nvmeq->sq_dma_addr, dev->bar + NVME_REG_ASQ);
	lo_hi_writeq(nvmeq->cq_dma_addr, dev->bar + NVME_REG_ACQ);

	result = nvme_enable_ctrl(&dev->ctrl, dev->ctrl.cap);
	if (result)
		return result;

	nvmeq->cq_vector = 0;
	nvme_init_queue(nvmeq, 0);

	return result;
}

static int nvme_pci_enable(struct nvme_dev *dev)
{
	int result = -ENOMEM;
	struct pci_dev *pdev = to_pci_dev(dev->dev);

	if (pci_enable_device_mem(pdev))
		return result;

	pci_set_master(pdev);

	/* Restricting CQ/SQ allocation below 4 GB */
	if (dma_set_mask_and_coherent(dev->dev, DMA_BIT_MASK(32)))
		goto disable;

	if (readl(dev->bar + NVME_REG_CSTS) == -1) {
		result = -ENODEV;
		goto disable;
	}

	/*
	 * Some devices and/or platforms don't advertise or work with INTx
	 * interrupts. Pre-enable a single MSIX or MSI vec for setup. We'll
	 * adjust this later.
	 */
	result = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_ALL_TYPES);
	if (result < 0)
		return result;

	dev->ctrl.cap = lo_hi_readq(dev->bar + NVME_REG_CAP);

	/* configuring maximum queue depth */
	dev->q_depth = NVME_CAP_MQES(dev->ctrl.cap);
	dev->db_stride = 1 << NVME_CAP_STRIDE(dev->ctrl.cap);
	dev->dbs = dev->bar + NVME_REG_DBS;

	dev_info(dev->dev, "CAP:%#llx, queue depth=%u, doorbell stride=%d\n",
		 dev->ctrl.cap, dev->q_depth, dev->db_stride);

	pci_enable_pcie_error_reporting(pdev);
	pci_save_state(pdev);
	return 0;

 disable:
	pci_disable_device(pdev);
	return result;
}

static void nvme_dev_unmap(struct nvme_dev *dev)
{
	if (dev->bar)
		iounmap(dev->bar);

	pci_release_mem_regions(to_pci_dev(dev->dev));
}

static void nvme_pci_disable(struct nvme_dev *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev->dev);

	pci_free_irq_vectors(pdev);

	if (pci_is_enabled(pdev)) {
		pci_disable_pcie_error_reporting(pdev);
		pci_disable_device(pdev);
	}
}

static void nvme_dev_disable(struct nvme_dev *dev, bool shutdown)
{
	if (shutdown)
		nvme_shutdown_ctrl(&dev->ctrl);
	else
		nvme_disable_ctrl(&dev->ctrl, dev->ctrl.cap);

	nvme_pci_disable(dev);
}

static int nvme_pci_reg_read32(struct nvme_ctrl *ctrl, u32 off, u32 *val)
{
	*val = readl(to_nvme_dev(ctrl)->bar + off);
	return 0;
}

static int nvme_pci_reg_write32(struct nvme_ctrl *ctrl, u32 off, u32 val)
{
	writel(val, to_nvme_dev(ctrl)->bar + off);
	return 0;
}

static int nvme_pci_reg_read64(struct nvme_ctrl *ctrl, u32 off, u64 *val)
{
	*val = readq(to_nvme_dev(ctrl)->bar + off);
	return 0;
}

static const struct nvme_ctrl_ops nvme_pci_ctrl_ops = {
	.name			= "pcie-lpm",
	.module			= THIS_MODULE,
	.reg_read32		= nvme_pci_reg_read32,
	.reg_write32		= nvme_pci_reg_write32,
	.reg_read64		= nvme_pci_reg_read64,
};

static int nvme_dev_map(struct nvme_dev *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev->dev);

	if (pci_request_mem_regions(pdev, "nvme"))
		return -ENODEV;

	dev_dbg(dev->dev, "BAR0 allocated @%llx of size 0x%llx\n",
		pci_resource_start(pdev, 0), pci_resource_len(pdev, 0));
	if (nvme_remap_bar(dev, NVME_REG_DBS + 4096))
		goto release;

	return 0;
 release:
	pci_release_mem_regions(pdev);
	return -ENODEV;
}

/* Read Write Ops */
int nvme_used_io_queues(struct nvme_dev *dev)
{
	return dev->lpm_dev.used_io_queues;
}

static struct nvme_id_ns *nvme_identify_ns(struct nvme_ctrl *ctrl,
					   unsigned int nsid)
{
	struct nvme_id_ns *id;
	struct nvme_command c = { };
	struct nvme_dev *dev = to_nvme_dev(ctrl);
	struct nvme_queue *q =  dev->queues[0];
	dma_addr_t id_phys;
	int error;

	memset(&c, 0, sizeof(c));
	c.identify.opcode = nvme_admin_identify;
	c.identify.nsid = cpu_to_le32(nsid);
	c.identify.cns = NVME_ID_CNS_NS;

	id = dmam_alloc_coherent(dev->dev, sizeof(*id),
				&id_phys, GFP_KERNEL);
	if (!id)
		return NULL;
	c.identify.dptr.prp1 = id_phys;

	error = nvme_submit_cmd_sync(&c, q, NULL);
	if (error) {
		dev_warn(ctrl->dev, "Identify namespace failed\n");
		return NULL;
	}

	return id;
}

static void nvme_pci_free_prps(struct nvme_dev *dev)
{
	unsigned int i;
	int page_size = dev->ctrl.page_size;

	for (i = 0; i < dev->num_prp_pages; i++)
		dma_free_coherent(dev->dev, page_size,
				  dev->prp_addr[i].vaddr,
				  dev->prp_addr[i].paddr);

	vfree(dev->prp_addr);
}

static int nvme_pci_setup_prps(struct nvme_dev *dev, phys_addr_t *mem_addr,
			       u64 *prp2, u16 lbas)
{
	struct nvme_lpm_dev *lpm_dev = &dev->lpm_dev;
	unsigned int page_size = dev->ctrl.page_size;
	unsigned int offset = *mem_addr & (page_size - 1);
	int xfer_length = lbas << lpm_dev->lba_shift;
	unsigned int nprps, i;
	int current_memmap;
	dma_addr_t *prp_list;
	dma_addr_t prp_dma_addr;

	xfer_length -= page_size - offset;
	if (xfer_length <= 0) {
		*prp2 = 0;
		return 0;
	}

	current_memmap = get_current_memmap(*mem_addr);
	check_memaddr(mem_addr, &current_memmap);

	if (xfer_length) {
		*mem_addr += page_size - offset;
		check_memaddr(mem_addr, &current_memmap);
	}

	if (xfer_length <= page_size) {
		*prp2 = *mem_addr;
		return 0;
	}

	prp_list = dma_alloc_coherent(dev->dev, page_size,
				      &prp_dma_addr, GFP_KERNEL);
	if (!prp_list)
		return -ENOMEM;

	dev->prp_addr[dev->num_prp_pages].vaddr = prp_list;
	dev->prp_addr[dev->num_prp_pages].paddr = prp_dma_addr;
	dev->num_prp_pages++;

	nprps = DIV_ROUND_UP(xfer_length, page_size);

	for (i = 0; i < nprps; i++) {
		prp_list[i] = cpu_to_le64(*mem_addr);
		*mem_addr += page_size;
		check_memaddr(mem_addr, &current_memmap);
	}

	*prp2 = prp_dma_addr;

	return 0;
}

static int nvme_send_flush_cmd(void *ndev_cntxt)
{
	struct nvme_dev *dev = ndev_cntxt;
	struct nvme_command cmnd = { };
	unsigned int qid = nvme_used_io_queues(dev);
	struct nvme_queue *q =  dev->queues[qid];
	unsigned int timeout = dev->num_prp_pages * IO_CMD_TIMEOUT_MS;

	memset(&cmnd, 0, sizeof(cmnd));

	cmnd.common.opcode = nvme_cmd_flush;
	cmnd.common.nsid = cpu_to_le32(1);

	return __nvme_submit_cmd_sync(&cmnd, q, NULL, timeout);
}

static int nvme_build_io_queue(struct nvme_dev *ndev, int qid,
			       phys_addr_t *mem_addr,
			       int nsid, u64 slba, u16 lbas, bool write)
{
	struct nvme_command cmnd = { };
	struct nvme_queue *q =  ndev->queues[qid];
	struct device *dev = ndev->dev;
	int error;
	u64 prp2;

	memset(&cmnd, 0, sizeof(cmnd));

	if (write)
		cmnd.rw.opcode = nvme_cmd_write;
	else
		cmnd.rw.opcode = nvme_cmd_read;

	cmnd.rw.nsid = cpu_to_le32(nsid);

	cmnd.rw.slba = cpu_to_le64(slba);
	cmnd.rw.length = cpu_to_le16(lbas - 1);

	cmnd.rw.dptr.prp1 = cpu_to_le64(*mem_addr);

	error = nvme_pci_setup_prps(ndev, mem_addr, &prp2, lbas);
	if (error) {
		dev_warn(dev, "PRP setup for SQ%d entry-%d failed\n",
				q->qid, cmnd.rw.command_id);
		return -EIO;
	}

	cmnd.rw.dptr.prp2 = cpu_to_le64(prp2);

	error = nvme_submit_cmd(&cmnd, q);
	if (error) {
		dev_warn(dev, "SQ%d entry-%d failed\n", q->qid, q->cmd_id);
		return -EIO;
	}
	dev_dbg(dev, "SQ%d entry-%d done\n", q->qid, q->cmd_id);

	return 0;
}

static int nvme_alloc_ns(struct nvme_ctrl *ctrl, unsigned int nsid)
{
	struct nvme_id_ns *id;
	int lba_shift;
	struct nvme_dev *dev = to_nvme_dev(ctrl);

	id = nvme_identify_ns(ctrl, nsid);
	if (!id)
		return -EINVAL;

	if (id->ncap == 0)
		return -EINVAL;

	lba_shift = id->lbaf[id->flbas & NVME_NS_FLBAS_LBA_MASK].ds;
	dev->lpm_dev.lba_shift = lba_shift;

	return 0;
}

static int nvme_initiate_xfers(void *ndev_cntxt)
{
	struct nvme_dev *ndev = ndev_cntxt;
	unsigned int qid, used_queues = nvme_used_io_queues(ndev);
	struct nvme_queue *q;

	/* Ringing SQDB for all Qs */
	for (qid = 1; qid <= used_queues; qid++) {
		q = ndev->queues[qid];
		dev_info(ndev->dev, "qid:%d q->sq_tail:%d", q->qid, q->sq_tail);
		writel(q->sq_tail, q->q_db);
	}

	return 0;
}

static void nvme_poll_xfers_on_queue(struct nvme_dev *dev, int qid)
{
	struct nvme_queue *q = dev->queues[qid];
	int consumed;

	if (q->poll_done)
		return;

	dev_dbg(dev->dev, "polling on CQ%d, entries seen till now: %d",
				qid, q->cqe_seen);
	nvme_poll(q, q->cmd_id);
	consumed = q->cqe_seen;
	if (consumed == q->cmd_id) {
		q->poll_done = true;
		dev_info(dev->dev, "CQ%d poll done", qid);
	}
}

static int nvme_poll_xfers(void *ndev_cntxt)
{
	struct nvme_dev *dev = ndev_cntxt;
	unsigned int qid, polled_cqs;
	unsigned int used_queues = nvme_used_io_queues(dev);
	int timeout = dev->num_prp_pages * IO_CMD_TIMEOUT_MS;
	struct nvme_queue *q;

	while (timeout--) {
		polled_cqs = 0;

		for (qid = 1; qid <= used_queues; qid++) {
			q = dev->queues[qid];

			if (q->poll_done)
				polled_cqs++;

			nvme_poll_xfers_on_queue(dev, qid);
		}

		if (polled_cqs == used_queues)
			break;

		usleep_range(1000, 1010);
	}

	if (timeout < 0)
		return -ETIMEDOUT;
	else
		return 0;
}

/**
 * nvme_destroy_backup_io_queues: Destroy any previously created command Qs
 */
static int nvme_destroy_backup_io_queues(void *ndev_cntxt)
{
	int i;
	struct nvme_dev *dev = ndev_cntxt;
	struct nvme_queue *nvmeq;

	if (dev->online_queues == 1)
		return 0;

	dev_info(dev->dev, "online Qs: %d", dev->online_queues);
	/* reset the IO Queues */
	for (i = dev->ctrl.queue_count - 1; i >= 1; i--) {
		nvmeq = dev->queues[i];
		if (adapter_delete_sq(dev, i)) {
			dev_err(dev->dev, "SQ-%d deletion failed", i);
			return -EIO;
		}
		if (adapter_delete_cq(dev, i)) {
			dev_err(dev->dev, "CQ-%d deletion failed", i);
			return -EIO;
		}
		dev->online_queues--;

		dma_free_coherent(dev->dev, SQ_SIZE(dev->q_depth),
				  (void *)nvmeq->sq_cmds, nvmeq->sq_dma_addr);
		dma_free_coherent(dev->dev, CQ_SIZE(dev->q_depth),
				  (void *)nvmeq->cqes, nvmeq->cq_dma_addr);
		dev->ctrl.queue_count--;
		dev_dbg(dev->dev, "QP-%d deletion done", i);
	}
	dev_info(dev->dev, "online Qs: %d", dev->online_queues);

	nvme_pci_free_prps(dev);

	return 0;
}

static void nvme_update_live_backup_state(void *ndev_cntxt,
					  enum live_backup_state new_state)
{
	struct nvme_dev *ndev = ndev_cntxt;
	struct nvme_lpm_data *data = ndev->shared_data;

	data->live_backup_state = new_state;
}

static void nvme_put_shared_data(struct nvme_dev *ndev, void *shared_data)
{
	int i;
	struct nvme_lpm_data *data = shared_data;
	struct pci_dev *pdev = to_pci_dev(ndev->dev);

	data->used_queues = nvme_used_io_queues(ndev);
	data->q_depth = ndev->q_depth;

	for (i = 0; i < ndev->online_queues; i++) {
		/* put SQ addresses */
		data->sq_addr[i] = ndev->queues[i]->sq_dma_addr;
		/* put CQ addresses */
		data->cq_addr[i] = ndev->queues[i]->cq_dma_addr;
		/* Live backup status for each QP can be known by polled CQEs */
		data->cqe_seen[i] = 0;
		/* put last cmd_id for each used QP */
		data->last_cmd_id[i] = ndev->queues[i]->cmd_id;
	}

	/* put nvme registers(BAR0) address */
	data->bar = pci_resource_start(pdev, 0);

	/*
	 * save reference to shared structure with CRMU for later use in
	 * Live backup case
	 */
	ndev->shared_data = data;
}

/**
 * nvme_build_backup_io_queues - Build NVMe command queue as per memory backup
 *				 requirement
 * @ blknr: starting backup NVMe logical block address (SLBA)
 * @ mem_addr : starting backup memory address
 * @ xfer_length: data transfer length in bytes, corresponding required logical
 *		  block count is stoted in blkcnt
 * @ write: when set, nvme write is done; when unset, nvme read is done
 * @ data: pointer to memory region shared with CRMU for CQ polling,
 *	   this region must be mapped prior to using it
 */
static int nvme_build_backup_io_queues(void *ndev_cntxt, u64 mem_addr,
				       u64 blknr, u64 xfer_length, bool write,
				       void *data)
{
	struct nvme_dev *dev = ndev_cntxt;
	struct nvme_lpm_dev *lpm_dev = &dev->lpm_dev;
	unsigned int lba_shift, i = 0, qid = 1;
	u64 blkcnt;
	u16 max_lbas_per_xfer;
	int ret;

	if (get_current_memmap(mem_addr) < 0) {
		dev_err(dev->dev, "Invalid memory address!\n");
		return -EINVAL;
	}

	ret = nvme_setup_io_queues(dev);
	if (ret) {
		dev_err(dev->dev, "Failed to setup io queues!\n");
		return -EIO;
	}
	dev_info(dev->dev, "online Qs: %d", dev->online_queues);

	dev->prp_addr = vmalloc((dev->max_qid) * (dev->q_depth)
				* sizeof(struct prp_list_addr));
	if (!dev->prp_addr)
		return -ENOMEM;
	dev->num_prp_pages = 0;

	/* At least 1 IO Queue will be used */
	lpm_dev->used_io_queues = 1;

	lba_shift = lpm_dev->lba_shift;
	blkcnt = DIV_ROUND_UP(xfer_length, 1 << lba_shift);
	max_lbas_per_xfer = 1 << (lpm_dev->max_transfer_shift - lba_shift);

	dev_info(dev->dev, "requested xfer size:%#llx->lbas:%#llx;"
		 "max_lbas_per_xfer:%#x start mem_addr:%#llx start lba: %#llx",
		 xfer_length, blkcnt, max_lbas_per_xfer, mem_addr, blknr);

	do {
		if (blkcnt < max_lbas_per_xfer) {
			max_lbas_per_xfer = blkcnt;
			blkcnt = 0;
		} else
			blkcnt -= max_lbas_per_xfer;

		if (i == dev->q_depth - 1) {
			dev_info(dev->dev, "Q%d is full now", qid);
			qid++;
			lpm_dev->used_io_queues++;
			i = 0;
		}

		ret = nvme_build_io_queue(dev, qid, &mem_addr, 1, blknr,
					  max_lbas_per_xfer, write);
		if (ret < 0)
			return ret;

		blknr += max_lbas_per_xfer;

		i++;
	} while (blkcnt);

	dev_info(dev->dev, "command queues(%d) built", qid);
	dev_info(dev->dev, "last lba:%#llx last mem_addr:%#llx",
		 blknr, mem_addr);

	/* share NVMe device queues data with CRMU */
	if (data) {
		nvme_put_shared_data(dev, data);
		dev_info(dev->dev, "NVMe data shared with CRMU\n");
	}

	return 0;
}

struct nvme_lpm_drv_ops nvme_lpm_driver_ops = {
	.nvme_destroy_backup_io_queues = nvme_destroy_backup_io_queues,
	.nvme_build_backup_io_queues = nvme_build_backup_io_queues,
	.nvme_initiate_xfers = nvme_initiate_xfers,
	.nvme_poll_xfers = nvme_poll_xfers,
	.nvme_send_flush_cmd = nvme_send_flush_cmd,
	.update_live_backup_state = nvme_update_live_backup_state,
};

static int nvme_lpm_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret = -ENOMEM;
	struct nvme_dev *dev;
	struct nvme_ctrl *ctrl;
	struct nvme_lpm_dev *lpm_dev;
	struct nvme_id_ctrl *id_ctrl;
	unsigned int nn;
	int min_page_shift;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->queues = devm_kzalloc(&pdev->dev,
			(MAX_IO_QPAIR + 1) * sizeof(void *), GFP_KERNEL);
	if (!dev->queues)
		return -ENOMEM;

	dev->dev = get_device(&pdev->dev);
	pci_set_drvdata(pdev, dev);
	lpm_dev = &dev->lpm_dev;

	/* ioremap the nvme bar registers(IORESOURCE_MEM) to outbound memory */
	ret = nvme_dev_map(dev);
	if (ret)
		goto put_pci;

	/* nvme controller regs read-write callbacks */
	ctrl = &dev->ctrl;
	ctrl->dev = dev->dev;
	ctrl->ops = &nvme_pci_ctrl_ops;

	ret = register_nvme_lpm_ops(&nvme_lpm_driver_ops);
	if (ret) {
		dev_err(dev->dev, "Failed to register LPM ioctls\n");
		goto put_pci;
	}
	nvme_lpm_driver_ops.ctxt = dev;

	ret = nvme_pci_enable(dev);
	if (ret)
		goto put_pci;

	ret = nvme_pci_configure_admin_queue(dev);
	if (ret)
		goto put_pci;

	/* nvme_init_identify: identify controller */
	ret = nvme_ctrl_init_identify(ctrl, &id_ctrl);
	if (ret)
		goto put_pci;

	dev_info(dev->dev, "pci-lpm function %s\n", dev_name(&pdev->dev));

	/* Honor RTD3 entry latency if provided */
	if (id_ctrl->rtd3e) {
		/* us -> s */
		u32 transition_time = le32_to_cpu(id_ctrl->rtd3e) / 1000000;

		/* Latency value clamped to a range of 5 to 60 seconds */
		ctrl->shutdown_timeout = clamp_t(unsigned int, transition_time,
						 SHUTDOWN_TIMEOUT_SEC, 60);

		if (ctrl->shutdown_timeout != SHUTDOWN_TIMEOUT_SEC)
			dev_warn(dev->dev,
				 "Shutdown timeout set to %u seconds\n",
				 dev->ctrl.shutdown_timeout);
	} else
		ctrl->shutdown_timeout = SHUTDOWN_TIMEOUT_SEC;

	min_page_shift = NVME_CAP_MPSMIN(ctrl->cap) + MIN_PAGE_SHIFT_CTRL;
	if (!id_ctrl->mdts)
		/**
		 * Maximum Data Transfer Size (MDTS) field indicates the maximum
		 * data transfer size between the host and the controller.
		 * The Spec says value of 0h indicates no restrictions on
		 * transfer size. We default to 2 MB in such case.
		 */
		lpm_dev->max_transfer_shift = 21;
	else
		lpm_dev->max_transfer_shift = (id_ctrl->mdts + min_page_shift);

	nn = le32_to_cpu(id_ctrl->nn);
	if (nn > 1) {
		dev_err(dev->dev, "Multiple namespaces not supported");
		ret = -ENOTSUPP;
		goto put_pci;
	}

	ret = nvme_alloc_ns(ctrl, nn);
	if (ret) {
		dev_err(dev->dev, "Invalid namespace");
		goto put_pci;
	}

	return 0;

put_pci:
	put_device(dev->dev);
	return ret;
}

static const struct pci_device_id nvme_lpm_id_table[] = {
	{ PCI_VDEVICE(INTEL, 0x0953), },    /* Intel 750 SSD */
	{ PCI_DEVICE(0x144d, 0xa804), },    /* Samsung SM961/PM961 SSD */
	{ PCI_DEVICE(0x126f, 0x2263), },    /* Silicon Motion SSD */
};

static void nvme_lpm_remove(struct pci_dev *pdev)
{
	struct nvme_dev *dev = pci_get_drvdata(pdev);

	unregister_nvme_lpm_ops();
	nvme_destroy_backup_io_queues(dev);

	if (!pci_device_is_present(pdev))
		nvme_dev_disable(dev, false);

	nvme_dev_disable(dev, true);
	nvme_dev_unmap(dev);
}

static struct pci_driver nvme_driver = {
	.name		= "nvme-lpm",
	.id_table	= nvme_lpm_id_table,
	.probe		= nvme_lpm_probe,
	.remove		= nvme_lpm_remove,
};

module_pci_driver(nvme_driver);

MODULE_AUTHOR("Abhishek Shah <abhishek.shah@broadcom.com>");
MODULE_DESCRIPTION("Low Power Mode Memory backup driver");
MODULE_LICENSE("GPL v2");
