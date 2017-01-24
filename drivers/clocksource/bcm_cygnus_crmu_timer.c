/*
 * Copyright (C) 2017 Broadcom
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
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/mailbox_client.h>
#include <linux/bcm_iproc_mailbox.h>
#include <asm/mach/time.h>

/*
 * Get M0 uptime
 *
 * Param - physical address of m0_ipc_m0_cmd_get_uptime struct,
 *         where m0 will store uptime value
 */
#define M0_IPC_M0_CMD_GET_UPTIME    0xf

struct m0_ipc_m0_cmd_get_uptime {
	uint32_t sec;
	uint32_t nsec;
};

struct cygnus_crmu_timer {
	struct device       *dev;
	struct mbox_client  client;
	struct mbox_chan    *mbox_chan;
	struct m0_ipc_m0_cmd_get_uptime m0_uptime;
};
struct cygnus_crmu_timer *cygnus_crmu_timer;

static int cygnus_crmu_timer_mbox_send_msg(uint32_t cmd, uint32_t param,
	bool wait_ack)
{
	int ret;
	struct iproc_mbox_msg msg;

	msg.cmd = cmd;
	msg.param = param;
	msg.wait_ack = wait_ack;
	ret = mbox_send_message(cygnus_crmu_timer->mbox_chan, &msg);
	if (ret < 0)
		dev_err(cygnus_crmu_timer->dev,
			"mbox_send_message failed: %d\n", ret);
	else if (msg.reply_code)
		dev_err(cygnus_crmu_timer->dev,
			"M0 command failed: 0x%x\n", msg.reply_code);

	mbox_client_txdone(cygnus_crmu_timer->mbox_chan, 0);

	return ret < 0 ? ret : msg.reply_code;
}

static int cygnus_crmu_timer_mbox_send_msg_with_struct(uint32_t cmd,
	void *param, size_t size, bool wait_ack)
{
	int ret;
	dma_addr_t dma_handle;

	dma_handle = dma_map_single(cygnus_crmu_timer->dev, param, size,
		DMA_BIDIRECTIONAL);
	if (dma_mapping_error(cygnus_crmu_timer->dev, dma_handle)) {
		dev_err(cygnus_crmu_timer->dev,
			"Failed to dma map param of command 0x%x\n", cmd);
		return -ENOMEM;
	}

	ret = cygnus_crmu_timer_mbox_send_msg(cmd, dma_handle, wait_ack);

	dma_unmap_single(cygnus_crmu_timer->dev, dma_handle,
			size, DMA_BIDIRECTIONAL);

	return ret;
}

static void cygnus_crmu_timer_read_persistent_clock(struct timespec64 *ts)
{
	int err;

	err = cygnus_crmu_timer_mbox_send_msg_with_struct(
		M0_IPC_M0_CMD_GET_UPTIME, &cygnus_crmu_timer->m0_uptime,
		sizeof(cygnus_crmu_timer->m0_uptime), true);
	if (err) {
		dev_err(cygnus_crmu_timer->dev,
			"Failed to get M0 uptime\n");
	} else {
		ts->tv_sec = cygnus_crmu_timer->m0_uptime.sec;
		ts->tv_nsec = cygnus_crmu_timer->m0_uptime.nsec;
	}

	dev_dbg(cygnus_crmu_timer->dev, "persistent_clock %llu.%09lu\n",
		ts->tv_sec, ts->tv_nsec);
}

static int __init cygnus_crmu_timer_probe(struct platform_device *pdev)
{
	int ret;

	if (cygnus_crmu_timer)
		dev_err(&pdev->dev, "cygnus crmu timer already set. not setting\n");

	cygnus_crmu_timer = devm_kzalloc(&pdev->dev,
		sizeof(*cygnus_crmu_timer), GFP_KERNEL);
	if (!cygnus_crmu_timer)
		return -ENOMEM;

	cygnus_crmu_timer->dev = &pdev->dev;
	platform_set_drvdata(pdev, cygnus_crmu_timer);

	/* Request mailbox channel. */
	cygnus_crmu_timer->client.dev          = &pdev->dev;
	cygnus_crmu_timer->client.rx_callback  = NULL;
	cygnus_crmu_timer->client.tx_done      = NULL;
	cygnus_crmu_timer->client.tx_block     = false;
	cygnus_crmu_timer->client.tx_tout      = 1;
	cygnus_crmu_timer->client.knows_txdone = true;
	cygnus_crmu_timer->mbox_chan = mbox_request_channel(
		&cygnus_crmu_timer->client, 0);
	if (IS_ERR(cygnus_crmu_timer->mbox_chan)) {
		dev_err(cygnus_crmu_timer->dev,
			"Unable to get mbox channel\n");
		return PTR_ERR(cygnus_crmu_timer->mbox_chan);
	}

	ret = register_persistent_clock(
		NULL, cygnus_crmu_timer_read_persistent_clock);
	if (ret) {
		dev_err(cygnus_crmu_timer->dev,
			"Failed to register_persistent_clock\n");
		goto err_free_channel;
	}

	dev_info(cygnus_crmu_timer->dev, "persistent clock registered\n");

	return 0;

err_free_channel:
	mbox_free_channel(cygnus_crmu_timer->mbox_chan);

	return ret;
}

static const struct of_device_id cygnus_crmu_timer_of_match[] = {
	{ .compatible = "brcm,cygnus-crmu-timer", },
	{}
};
MODULE_DEVICE_TABLE(of, cygnus_crmu_timer_of_match);

struct platform_driver __refdata cygnus_crmu_timer_driver = {
	.driver = {
		.name = "cygnus-crmu-timer",
		.of_match_table = cygnus_crmu_timer_of_match,
	},
	.probe = cygnus_crmu_timer_probe,
};
module_platform_driver(cygnus_crmu_timer_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Cygnus CRMU Persistent Timer Driver");
MODULE_LICENSE("GPL v2");
