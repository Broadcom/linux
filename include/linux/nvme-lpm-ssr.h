// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2018 Broadcom
 */

#ifndef __NVME_LPM_SSR_H__
#define __NVME_LPM_SSR_H__

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
};

struct armed_ssr {
	uint32_t sequence;
	uint64_t memory_address;
	uint64_t disk_address;
	uint64_t length;
} __attribute__((packed));

struct disarmed_ssr {
	uint32_t sequence;
} __attribute__((packed));

struct state_save_checkpoint {
	uint32_t voltage;
	uint64_t timestamp;
} __attribute__((packed));

struct ssr {
	uint8_t state;
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

enum {
	NVME_LPM_CMD_GET_SSR = 0x53535201, /* cmd format in ASCII SSR<x>*/
	NVME_LPM_CMD_ERASE_SSR,
	NVME_LPM_CMD_ARM_SSR,
	NVME_LPM_CMD_DISARM_SSR,
	NVME_LPM_CMD_TRIGGER_SSR,
};

#define NVM_SSR_OFFSET	0x0
#define OS_CRMU_SSRCMD_MEM 0x8f100100

struct ssr_wrapper {
	/*
	 * MAIA must populate this signature indicating validity of cmd
	 */
#define MAIA_CRMU_VALID_REQUEST		0xbabebabe
	uint32_t maia_crmu_valid_request;
	/*
	 * CRMU must populate this signature indicating status of command.
	 * if done succussfully populate with CRMU_MAIA_VALID_RESPONSE or
	 * 0x0 as erroneous condition.
	 */
#define MAX_CRMU_RESPONSE_TIMEOUT 1000 /* 100 msec */
#define CRMU_MAIA_VALID_RESPONSE	0xfaceface
	uint32_t crmu_maia_valid_response;
	/*
	 * MAIA uses this to send the command to MCU. MCU should clear this
	 * indicating that command has been processed.
	 */
	uint32_t ssr_cmd_id;

	/*
	 * SSR struture. It should be populated with relevant field in case
	 * other side expects anything i.e  sequence number
	 */
	struct ssr ssr;
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

struct nvme_lpm_drv_ops {
	void *ctxt;
	int (*nvme_destroy_backup_io_queues)(void *ctxt);
	int (*nvme_build_backup_io_queues)(void *ctxt, uint64_t memory_address,
					   uint64_t disk_address,
					   uint64_t length,
					   bool write_to_nvme,
					   void *shared_nvme_data);
	int (*nvme_initiate_xfers)(void *ctxt);
	int (*nvme_poll_xfers)(void *ctxt);
	int (*nvme_send_flush_cmd)(void *ctxt);
	void (*update_live_backup_state)(void *ctxt,
					 enum live_backup_state new_state);
};

int register_nvme_lpm_ops(void *nvme_drv_ops);
void unregister_nvme_lpm_ops(void);

#endif
