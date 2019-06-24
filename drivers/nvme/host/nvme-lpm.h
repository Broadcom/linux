// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2018 Broadcom
 */

#ifndef _NVME_LPM_H
#define _NVME_LPM_H

#include "nvme.h"

#define SQ_SIZE(depth)    ((depth) * sizeof(struct nvme_command))
#define CQ_SIZE(depth)    ((depth) * sizeof(struct nvme_completion))

struct nvme_lpm_dev {
	unsigned int lba_shift;
	unsigned int max_transfer_shift;
	unsigned int used_io_queues;
	phys_addr_t mem_addr;
	u64 slba;
	u64 xfer_length;
	bool nvme_write;
};

struct prp_list_addr {
	void *vaddr;
	dma_addr_t paddr;
};

/*
 * Represents an NVM Express device.  Each nvme_dev is a PCI function.
 */
struct nvme_dev {
	struct nvme_queue **queues;
	void __iomem *dbs;
	struct device *dev;
	unsigned int online_queues;
	unsigned int max_qid;
	unsigned int q_depth;
	u32 db_stride;
	void __iomem *bar;
	unsigned long bar_mapped_size;
	bool subsystem;
	struct nvme_ctrl ctrl;
	struct nvme_lpm_dev lpm_dev;
	struct prp_list_addr *prp_addr;
	unsigned int num_prp_pages;
	void *shared_data;
};

static inline struct nvme_dev *to_nvme_dev(struct nvme_ctrl *ctrl)
{
	return container_of(ctrl, struct nvme_dev, ctrl);
}

/*
 * An NVM Express queue.  Each device has at least two (one for admin
 * commands and one for I/O commands).
 */
struct nvme_queue {
	struct nvme_dev *dev;
	spinlock_t q_lock;
	struct nvme_command *sq_cmds;
	struct nvme_completion *cqes;
	dma_addr_t sq_dma_addr;
	dma_addr_t cq_dma_addr;
	void __iomem *q_db;
	u16 q_depth;
	s16 cq_vector;
	u16 sq_tail;
	u16 cq_head;
	u16 qid;
	u8 cq_phase;
	unsigned int cqe_seen;
	bool poll_done;
	unsigned int cmd_id;
};

#endif /* _NVME_LPM_H */
