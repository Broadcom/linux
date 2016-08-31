/*
 * Copyright (C) 2016 Broadcom.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/mailbox_client.h>
#include <linux/bcm_iproc_mailbox.h>
#include <asm/system_misc.h>

struct iproc_reset {
	struct device       *dev;
	struct mbox_client  client;
	struct mbox_chan    *mbox_chan;
};

enum iproc_reset_m0_cmd_param {
	/*
	 * Reset request
	 * Param - must be 0xffffffff
	 * Response - return code
	 */
	M0_IPC_M0_CMD_IPROC_RESET = 0,
	M0_IPC_M0_CMD_IPROC_RESET_PARAM = 0xffffffff,

	/*
	 * Issue chip Warm reset
	 * Param - none
	 * Response - none (A9 will reset as a result of this command)
	 */
	M0_IPC_M0_CMD_WARM_RESET = 0xd,

	/*
	 * Enter OFF state
	 * Param - physical address to resume execution from upon power saving
	 *   mode exit.
	 * Response - none (M0 will power the system off as a result of this
	 *   command)
	 */
	M0_IPC_M0_CMD_ENTER_OFF = 0xa,
};

static struct iproc_reset *iproc_reset;

static inline void iproc_reset_print_mbox_err(int mbox_err, int reply_code)
{
	if (mbox_err < 0)
		dev_err(iproc_reset->dev,
			"mbox_send_message failed: %d\n", mbox_err);
	else if (reply_code)
		dev_err(iproc_reset->dev,
			"M0 command failed: 0x%x\n", reply_code);
}

static inline void iproc_mbox_send_msg(struct iproc_mbox_msg *msg)
{
	int err;

	err = mbox_send_message(iproc_reset->mbox_chan, msg);
	iproc_reset_print_mbox_err(err, msg->reply_code);
	mbox_client_txdone(iproc_reset->mbox_chan, 0);
}
/*
 * Perform cold (L0) reset.
 */
static inline void iproc_cold_reset(void)
{
	struct iproc_mbox_msg msg;

	msg.cmd = M0_IPC_M0_CMD_IPROC_RESET;
	msg.param = M0_IPC_M0_CMD_IPROC_RESET_PARAM;
	msg.wait_ack = true;
	iproc_mbox_send_msg(&msg);
}

/*
 * Perform warm reset (keep ethernet switch active through the reset)
 */
static inline void iproc_warm_reset(void)
{
	struct iproc_mbox_msg msg;

	msg.cmd = M0_IPC_M0_CMD_WARM_RESET;
	msg.param = 0;
	msg.wait_ack = false;
	iproc_mbox_send_msg(&msg);
}

/*
 * Notifies M0 to power off.
 */
static inline void iproc_power_off(void)
{
	struct iproc_mbox_msg msg;

	msg.cmd = M0_IPC_M0_CMD_ENTER_OFF;
	msg.param = 0;
	msg.wait_ack = false;
	iproc_mbox_send_msg(&msg);
}

/*
 * Handles rebooting CPU.
 */
static void iproc_reboot(enum reboot_mode mode, const char *cmd)
{
	if (mode == REBOOT_COLD)
		iproc_cold_reset();
	else if (mode == REBOOT_WARM)
		iproc_warm_reset();
}

static int iproc_reset_probe(struct platform_device *pdev)
{

	iproc_reset = devm_kzalloc(&pdev->dev,
		sizeof(struct iproc_reset), GFP_KERNEL);
	if (!iproc_reset)
		return -ENOMEM;

	iproc_reset->dev = &pdev->dev;

	/* Request mailbox channel. */
	iproc_reset->client.dev          = &pdev->dev;
	iproc_reset->client.tx_block     = false;
	iproc_reset->client.tx_tout      = 1;
	iproc_reset->client.knows_txdone = true;
	iproc_reset->mbox_chan = mbox_request_channel(&iproc_reset->client, 0);
	if (IS_ERR(iproc_reset->mbox_chan)) {
		dev_err(&pdev->dev, "unable to get mbox channel\n");
		return PTR_ERR(iproc_reset->mbox_chan);
	}

	/* Set the machine restart handler. */
	arm_pm_restart = iproc_reboot;

	/* Set the power off handler. */
	pm_power_off = iproc_power_off;

	return 0;
}

static const struct of_device_id iproc_reset_of_match[] = {
	{ .compatible = "brcm,iproc-reset", },
	{}
};
MODULE_DEVICE_TABLE(of, iproc_reset_of_match);

struct platform_driver iproc_reset_driver = {
	.driver = {
		.name = "brcm,iproc-reset",
		.of_match_table = iproc_reset_of_match,
	},
	.probe = iproc_reset_probe,
};
module_platform_driver(iproc_reset_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom iproc Reset Driver");
MODULE_LICENSE("GPL v2");
