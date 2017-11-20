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

#ifndef BSCD_T1_H__
#define BSCD_T1_H__

#include "bscd.h"

#define SC_T1_OFFSET_NAD     0
#define SC_T1_OFFSET_PCB     1
#define SC_T1_OFFSET_LEN     2
#define SC_T1_OFFSET_INF     3

#define SC_T1_I_BLK          0x0
/* I block sequence number. b7 of b8-b1 */
#define SC_T1_I_BLK_N_SHIFT  6
/* I block More bit. b6 of b8-b1 */
#define SC_T1_I_BLK_MORE     0x20
#define SC_T1_I_BLK_N(pcb)   ((pcb >> SC_T1_I_BLK_N_SHIFT) & 0x01)

#define SC_T1_R_BLK          0x80
/* R block sequence number. b5 of b8-b1 */
#define SC_T1_R_BLK_N_SHIFT  4
/* b2-b1 are err indication bits */
#define SC_T1_R_BLK_OK(pcb)  ((pcb & 0x03) == 0)
#define SC_T1_R_BLK_N(pcb)   ((pcb >> SC_T1_R_BLK_N_SHIFT) & 0x01)
#define SC_T1_R_BLK_ERR_PARITY  0x01
#define SC_T1_R_BLK_ERR_OTHER   0x02


#define SC_T1_S_BLK(pcb)     0xC0

#define SC_T1_S_RESYNC_REQ   (0xC0) /* 0b11000000 */
#define SC_T1_S_RESYNC_REP   (0xE0) /* 0b11100000 */
#define SC_T1_S_IFS_REQ      (0xC1) /* 0b11000001 */
#define SC_T1_S_IFS_REP      (0xE1) /* 0b11100001 */
#define SC_T1_S_ABORT_REQ    (0xC2) /* 0b11000010 */
#define SC_T1_S_ABORT_REP    (0xE2) /* 0b11100010 */
#define SC_T1_S_WTX_REQ      (0xC3) /* 0b11000011 */
#define SC_T1_S_WTX_REP      (0xE3) /* 0b11100011 */

#define SC_T1_GET_BLK_TYPE(pcb) (((pcb & 0x80) == 0 ? SC_T1_I_BLK : \
		((pcb & 0xC0) == 0x80 ? SC_T1_R_BLK : pcb)))

#define SC_T1_ERR_COUNT_RETRANSMIT    3     /* Rule 7.4.2 */
#define SC_T1_ERR_COUNT_RESYNCH       3     /* Rule 6.4 */

enum sc_state {
	SC_STATE_DATASEND,
	SC_STATE_DATARCV,
};

enum sc_action {
	SC_ACTION_TX_I,
	SC_ACTION_TX_R,
	SC_ACTION_TX_RESYNC,
	SC_ACTION_TX_WTX,
	SC_ACTION_TX_IFS,
	SC_ACTION_TX_ABT
};

struct bscd_tpdu_t1 {
	enum sc_state scstate;
	enum sc_action scaction;
	unsigned char ndevice;
	unsigned char ncard;
	unsigned int  nerr;
};

int bscd_chnl_tpdu_transceive_t1(
	struct p_chnl_hdl *in_channelhandle,
	const struct bscd_chnl_settings *inp_channeldefsettings,
	uint8_t *inp_ucxmitdata,
	unsigned long in_ulnumxmitbytes,
	uint8_t *outp_ucrcvdata,
	unsigned long *outp_ulnumrcvbytes,
	unsigned long in_ulmaxreadbytes
);

#endif /* BSCD_T1_H__ */
