/*
 * Copyright (C) 2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef IPROC_SCI_IOCTL_H
#define IPROC_SCI_IOCTL_H

/* ioctl commands */
#define SC_IOCTL_MAGIC     'S'
#define SC_IOCTL_INIT      _IO(SC_IOCTL_MAGIC, 0x01)
#define SC_IOCTL_POWERUP   _IO(SC_IOCTL_MAGIC, 0x02)
#define SC_IOCTL_POWERDOWN _IO(SC_IOCTL_MAGIC, 0x03)
#define SC_IOCTL_TRANSMIT  _IOWR(SC_IOCTL_MAGIC, 0x04, __u32)
#define SC_IOCTL_RESET     _IO(SC_IOCTL_MAGIC, 0x05)
#define SC_IOCTL_ICCSTATUS _IOR(SC_IOCTL_MAGIC, 0x06, __u32)
#define SC_IOCTL_DOPPS     _IO(SC_IOCTL_MAGIC, 0x07)

#define MAX_INTERFACES  2

struct atr_parm {
	char atr[BSCD_MAX_ATR_SIZE];
	int atr_len;
};

struct sc {
	int id;
	int handle;
	struct atr_parm atr;
} sc_port[MAX_INTERFACES];

struct apdu_args {
	unsigned char *txbuf;
	int txlen;
	unsigned char *rxbuf;
	int *rxlen;
};

#endif /* IPROC_SCI_IOCTL_H */
