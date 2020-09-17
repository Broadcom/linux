/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2020 Broadcom.
 */
#ifndef _BCM_IPROC_MAILBOX_V2_H_
#define _BCM_IPROC_MAILBOX_V2_H_

#include <linux/types.h>

/*
 * iProc mailbox message format.
 * @cmd: Mailbox command.
 * @param: Mailbox command parameters.
 */
struct iproc_mbox_msg {
	u32       cmd;
	u32       param[4];
} __packed;

#endif
