/*
 * Copyright (C) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ARCH_CYGNUS_PM_H
#define __ARCH_CYGNUS_PM_H

#include <linux/mailbox_client.h>
#include <linux/bcm_iproc_mailbox.h>

#ifdef CONFIG_PM
void cygnus_pm_init(void);
#else
#define cygnus_pm_init NULL
#endif

struct cygnus_pm {
	struct device      *dev;
	struct mbox_client mbox_client;
	struct mbox_chan   *mbox_chan;

	void __iomem  *scu;
	void __iomem  *crmu;
	void __iomem  *cru;
};

struct cygnus_pm * __init cygnus_pm_device_init(void);

void cygnus_pm_cpu_resume(void);

/*
 * Sends message to M0 using mailbox framework.
 *
 * @cmd The command to send.
 * @param The parameter corresponding to the command or 0 if n/a.
 * @wait_ack true to wait for M0 to send a reply to this command, false
 *   to return immediately and not wait for a reply.
 *
 * @return A negative value if sending the message failed, otherwise the reply
 *   code from the M0 indicating success or failure of executing the command: 0
 *   indicates success.
 */
static inline int cygnus_mbox_send_msg(struct mbox_chan *chan, u32 cmd,
	u32 param, bool wait_ack)
{
	int ret;
	struct iproc_mbox_msg msg;

	if (!chan)
		return -ENODEV;

	msg.cmd = cmd;
	msg.param = param;
	msg.wait_ack = wait_ack;
	ret = mbox_send_message(chan, &msg);
	mbox_client_txdone(chan, 0);

	return ret < 0 ? ret : msg.reply_code;
}

#endif
