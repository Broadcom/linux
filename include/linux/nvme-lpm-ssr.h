// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2019 Broadcom
 */

#ifndef __NVME_LPM_SSR_H__
#define __NVME_LPM_SSR_H__

#include <stddef.h>
#include <uapi/linux/nvme-lpm-ssr.h>

enum {
	NVME_LPM_CMD_GET_SSR = 0x53535201, /* cmd format in ASCII SSR<x>*/
	NVME_LPM_CMD_ERASE_SSR,
	NVME_LPM_CMD_ARM_SSR,
	NVME_LPM_CMD_DISARM_SSR,
	NVME_LPM_CMD_TRIGGER_SSR,
	NVME_LPM_CMD_LIVE_BACKUP_SSR,
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
