/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2019 Broadcom.
 */

#ifndef _BROADCOM_TEE_BNXT_FW_H
#define _BROADCOM_TEE_BNXT_FW_H

#include <linux/types.h>

int tee_bnxt_fw_load(void);
int tee_bnxt_health_status(u32 *status);
int tee_bnxt_handshake(u32 timeout, u32 *status);
int tee_bnxt_fw_update(void *buf, u32 size);
int tee_bnxt_copy_coredump(void *buf, u32 offset, u32 size);

#endif /* _BROADCOM_TEE_BNXT_FW_H */
