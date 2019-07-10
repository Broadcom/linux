/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright(c) 2019 Broadcom
 */

#ifndef __UAPI_NVME_LPM_SSR_H__
#define __UAPI_NVME_LPM_SSR_H__

#include <stddef.h>

enum ssr_states {
	SSR_STATE_INVALID,
	SSR_STATE_INIT,
	SSR_STATE_DISARM,
	SSR_STATE_ARM,
	SSR_STATE_POWERLOSS,
	SSR_STATE_INPROGRESS,
	SSR_STATE_COMPLETE,
	SSR_STATE_ERROR,
};

enum live_backup_state {
	LIVE_BACKUP_NOT_ACTIVE,
	LIVE_BACKUP_XFER_INIT,
	LIVE_BACKUP_IN_PROGRESS,
	LIVE_BACKUP_FLUSH,
	LIVE_BACKUP_DONE,
	LIVE_BACKUP_FAIL,
};

struct state_save_checkpoint {
	uint32_t voltage;
	uint64_t timestamp;
} __attribute__((packed));

struct ssr {
	uint8_t state;
	uint8_t live_backup_state;
	uint32_t sequence;
	uint64_t nvme_backup_length;
	uint64_t nvme_backup_offset;
	uint64_t nvme_error_code;
	uint64_t ddr0_ecc_code;
	uint64_t ddr1_ecc_code;
	struct state_save_checkpoint power_loss;
	struct state_save_checkpoint in_progress;
	struct state_save_checkpoint complete;
} __attribute__((packed));
#define SSR_SIZE sizeof(struct ssr)

struct armed_ssr {
	uint32_t sequence;
	uint64_t memory_address;
	uint64_t disk_address;
	uint64_t length;
} __attribute__((packed));

struct disarmed_ssr {
	uint32_t sequence;
} __attribute__((packed));

#define NVME_LPM_IOCTL_MAGIC		'N'
#define NVME_LPM_IOCTL_GET_SSR		_IOR(NVME_LPM_IOCTL_MAGIC, 1, \
						struct ssr)
#define NVME_LPM_IOCTL_ERASE_SSR	_IO(NVME_LPM_IOCTL_MAGIC, 2)
#define NVME_LPM_IOCTL_ARM_SSR		_IOW(NVME_LPM_IOCTL_MAGIC, 3, \
						struct armed_ssr)
#define NVME_LPM_IOCTL_DISARM_SSR	_IOW(NVME_LPM_IOCTL_MAGIC, 4, \
						struct disarmed_ssr)
#define NVME_LPM_IOCTL_TRIGGER_SSR	_IO(NVME_LPM_IOCTL_MAGIC, 5)
#define NVME_LPM_IOCTL_READ		_IOW(NVME_LPM_IOCTL_MAGIC, 6, \
						struct armed_ssr)
#define NVME_LPM_IOCTL_AP_POLL		_IO(NVME_LPM_IOCTL_MAGIC, 7)

#endif
