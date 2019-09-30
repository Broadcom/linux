/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 Broadcom.
 *
 */
#ifndef BCM_VK_MAILBOX_H
#define BCM_VK_MAILBOX_H

#include <linux/types.h>
/*
 * Valkyrie mailbox message, send/recv 64-bit data
 *
 * @cmd command to send.
 * @arg argunment for the command.
 *
 * message format:
 *  ------------------------------   --------------------------
 * |  31  | 30         4 | 3    0 | | 31                    0  |
 * | ACK  |    COMMAND   | ARG(MS)| |        ARG(LS)           |
 *  ------------------------------   --------------------------
 */
struct vk_mbox_msg {
	uint32_t cmd;
	uint32_t arg;
};

#endif
