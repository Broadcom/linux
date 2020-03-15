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
 * @payload_phys_addr: When the payload (parameter) is bigger than 8 bytes,
 *  instead of embedding within the mailbox message, it needs to be sent
 *  using a separate buffer. In this case 'payload_phys_addr' specifies the
 *  physical address of this buffer.
 * @param: Mailbox command extra parameters.
 */
struct iproc_mbox_msg {
	u32       cmd;
	union {
		u32       param[2];
		u64       payload_phys_addr;
	};
	u32       extra_param[2];
} __packed;

#endif
