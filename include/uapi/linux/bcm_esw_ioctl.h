/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *  Copyright(c) 2018 Broadcom
 */

#ifndef __BCM_ESW_IOCTL_H
#define __BCM_ESW_IOCTL_H

#include <linux/ioctl.h>
#include <linux/sockios.h>
#include <linux/types.h>

struct esw_reg_data {
	/* Switch access using 64-bit only */
	uint64_t data;
	/* Actually register length, in bits */
	size_t len;
	uint8_t page;
	uint8_t offset;
};

/* Data structure for entering Wake-up On Lan (WOL) mode
 * Only the port specified in the data structure will be enabled
 * during the WOL mode. Other Ports will be disabled.
 */
struct esw_wol_data {
	unsigned int port; /* The port to be left enabled */
	unsigned int port_speed; /* speed of the port in WOL mode */
};

#define SIOCESW_REG_READ         (SIOCDEVPRIVATE + 1)
#define SIOCESW_REG_WRITE        (SIOCDEVPRIVATE + 2)
#define SIOCESW_ENTER_WOL        (SIOCDEVPRIVATE + 3)
#define SIOCESW_EXIT_WOL         (SIOCDEVPRIVATE + 4)
#define SIOCESW_GET_EEE          (SIOCDEVPRIVATE + 5)
#define SIOCESW_SET_EEE          (SIOCDEVPRIVATE + 6)

#endif /* __BCM_ESW_IOCTL_H */
