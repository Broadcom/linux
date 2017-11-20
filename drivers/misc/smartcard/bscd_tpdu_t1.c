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

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/wait.h>

#include "bscd.h"
#include "bscd_datatypes.h"
#include "bscd_priv.h"

uint8_t bscd_calculate_lrc(uint8_t *buf, uint8_t len)
{
	int i;
	uint8_t nlrc = 0;

	/* calculate lrc or crc */
	for (i = 0; i < len; i++)
		nlrc ^= buf[i];

	return nlrc;
}

int bscd_process_err(struct bscd_tpdu_t1 *ptpdu_t1)
{
	ptpdu_t1->nerr++;

	if (ptpdu_t1->scaction == SC_ACTION_TX_I)
		ptpdu_t1->scaction = SC_ACTION_TX_R;
	if (ptpdu_t1->nerr <= SC_T1_ERR_COUNT_RETRANSMIT)
		return 0;
	if (ptpdu_t1->nerr <= SC_T1_ERR_COUNT_RESYNCH) {
		ptpdu_t1->scaction = SC_ACTION_TX_RESYNC;
		return 0;
	}

	return BSCD_STATUS_FAILED;
}

int bscd_chnl_tpdu_transceive_t1(
		struct p_chnl_hdl         *in_channelhandle,
		const struct bscd_chnl_settings *inp_channeldefsettings,
		uint8_t                    *inp_ucxmitdata,
		unsigned long              in_ulnumxmitbytes,
		uint8_t                    *outp_ucrcvdata,
		unsigned long              *outp_ulnumrcvbytes,
		unsigned long              in_ulmaxreadbytes
)
{
	int errcode = BERR_SUCCESS;
	uint32_t ntpdulen;
	uint8_t  nsentlen = 0;
	uint8_t  nrxlen = 0;
	uint32_t ulactualrxlen = 0;
	uint8_t  nwtx = 0, nifs = 0;
	uint8_t ntxpcb = 0, nrxpcb = 0, nrerr = 0 /* r blk err bits */;
	uint8_t uctxrxbuf[T1_MAX_BUF_SIZE];
	struct bscd_tpdu_t1 *ptpdu_t1 = &(in_channelhandle->sc_tpdu_t1);
	uint32_t berr = 0, babort = 0;
	struct bscd_timer_value timevalue = {
		BSCD_DEFAULT_BLOCK_WAITING_TIME, unit_etu};

	ptpdu_t1->scstate  = SC_STATE_DATASEND;
	ptpdu_t1->scaction = SC_ACTION_TX_I;
	ptpdu_t1->nerr     = 0;
	*outp_ulnumrcvbytes = 0; /* clear it because so far no rx */

	while (1) {
		switch (ptpdu_t1->scaction) {

		case SC_ACTION_TX_I: /* send out the application data */
			pr_devel("bscd_channel_tpdu_transceive_t1: tx i blk\n");
			ntxpcb = SC_T1_I_BLK |
			(ptpdu_t1->ndevice << SC_T1_I_BLK_N_SHIFT);
			nsentlen = in_ulnumxmitbytes;
			if (in_ulnumxmitbytes >
				inp_channeldefsettings->curr_ifsd) {
				ntxpcb |= SC_T1_I_BLK_MORE;
				nsentlen = inp_channeldefsettings->curr_ifsd;
			}
			uctxrxbuf[SC_T1_OFFSET_NAD] = 0;
			uctxrxbuf[SC_T1_OFFSET_PCB] = ntxpcb;
			uctxrxbuf[SC_T1_OFFSET_LEN] = nsentlen;
			memcpy(&uctxrxbuf[SC_T1_OFFSET_INF],
						inp_ucxmitdata, nsentlen);
			ntpdulen = nsentlen + 3;
			ptpdu_t1->ndevice = (ptpdu_t1->ndevice == 0) ? 1 : 0;
			break;

		case SC_ACTION_TX_R:
			pr_devel("bscd_channel_tpdu_transceive_t1: tx r blk\n");
			uctxrxbuf[SC_T1_OFFSET_NAD] = 0;
			uctxrxbuf[SC_T1_OFFSET_PCB] = SC_T1_R_BLK |
		(ptpdu_t1->ncard << SC_T1_R_BLK_N_SHIFT) | nrerr;
			uctxrxbuf[SC_T1_OFFSET_LEN] = 0;
			ntpdulen = 3;
			break;

		case SC_ACTION_TX_RESYNC:
			pr_devel("bscd_channel_tpdu_transceive_t1: tx resync\n");
			uctxrxbuf[SC_T1_OFFSET_NAD] = 0;
			uctxrxbuf[SC_T1_OFFSET_PCB] = SC_T1_S_RESYNC_REQ;
			uctxrxbuf[SC_T1_OFFSET_LEN] = 0;
			ntpdulen = 3;
			break;

		case SC_ACTION_TX_WTX: /* wtx response */
			pr_devel("bscd_channel_tpdu_transceive_t1: tx wtx\n");
			uctxrxbuf[SC_T1_OFFSET_NAD] = 0;
			uctxrxbuf[SC_T1_OFFSET_PCB] = SC_T1_S_WTX_REP;
			uctxrxbuf[SC_T1_OFFSET_LEN] = 1;
			uctxrxbuf[SC_T1_OFFSET_INF] = nwtx;
			ntpdulen = 4;
			break;

		case SC_ACTION_TX_IFS: /* ifs response */
			pr_devel("bscd_channel_tpdu_transceive_t1: tx ifs\n");
			uctxrxbuf[SC_T1_OFFSET_NAD] = 0;
			uctxrxbuf[SC_T1_OFFSET_PCB] = SC_T1_S_IFS_REP;
			uctxrxbuf[SC_T1_OFFSET_LEN] = 1;
			uctxrxbuf[SC_T1_OFFSET_INF] = nifs;
			ntpdulen = 4;
			break;

		case SC_ACTION_TX_ABT: /* abort response */
			pr_devel("bscd_channel_tpdu_transceive_t1: tx ifs\n");
			uctxrxbuf[SC_T1_OFFSET_NAD] = 0;
			uctxrxbuf[SC_T1_OFFSET_PCB] = SC_T1_S_ABORT_REP;
			uctxrxbuf[SC_T1_OFFSET_LEN] = 0;
			ntpdulen = 3;
			break;

		default:
			pr_devel("invalid action code 0x%x,\n",
				ptpdu_t1->scaction);
			return BSCD_STATUS_FAILED;
		}

		uctxrxbuf[ntpdulen++] = bscd_calculate_lrc(uctxrxbuf, ntpdulen);

		berr = 0;
		errcode = chnl_xmit(in_channelhandle, uctxrxbuf, ntpdulen);
		if (errcode != BERR_SUCCESS)
			return errcode;

		errcode = chnl_rcv(in_channelhandle, uctxrxbuf,
			(unsigned long *)&ulactualrxlen, T1_MAX_BUF_SIZE);
		if (errcode != BERR_SUCCESS || uctxrxbuf[SC_T1_OFFSET_NAD] != 0)
			berr = 1;

		if (bscd_calculate_lrc(uctxrxbuf, ulactualrxlen) != 0)
			berr = 1;

		/* minus lrc */
		ulactualrxlen--;

		if (berr == 0) {
			nrxpcb = uctxrxbuf[SC_T1_OFFSET_PCB];
			nrxlen = uctxrxbuf[SC_T1_OFFSET_LEN];

			switch (SC_T1_GET_BLK_TYPE(nrxpcb)) {
			case SC_T1_S_WTX_REQ:
				if (nrxlen != 1) {
					berr = 1;
					ptpdu_t1->scaction = SC_ACTION_TX_R;
					break;
				}
				nwtx = uctxrxbuf[SC_T1_OFFSET_INF];
				timevalue.value = nwtx *
				in_channelhandle->cur_chs.blk_wait_time.value;
				errcode = chnl_set_blk_wait_time_ext(
				in_channelhandle, timevalue.value);
				ptpdu_t1->scaction = SC_ACTION_TX_WTX;
				break;

			case SC_T1_S_ABORT_REQ:
				if (nrxlen != 0) {
					berr = 1;
					ptpdu_t1->scaction = SC_ACTION_TX_R;
					break;
				}
				babort = 1;
				ptpdu_t1->scaction = SC_ACTION_TX_ABT;
				break;

			case SC_T1_S_IFS_REQ:
				if (nrxlen != 1) {
					berr = 1;
					ptpdu_t1->scaction = SC_ACTION_TX_R;
					break;
				}
				nifs = uctxrxbuf[SC_T1_OFFSET_INF];
				ptpdu_t1->scaction = SC_ACTION_TX_IFS;
				break;

			case SC_T1_I_BLK:
				if ((ulactualrxlen < SC_T1_OFFSET_INF) ||
					((ulactualrxlen - 3) !=
					uctxrxbuf[SC_T1_OFFSET_LEN])) {
					berr = 1;
					break;
				}

				if (ptpdu_t1->scstate == SC_STATE_DATASEND)
					berr = (ntxpcb & SC_T1_I_BLK_MORE) ?
								1 : berr;

				berr = (ptpdu_t1->ncard !=
					SC_T1_I_BLK_N(nrxpcb)) ? 1 : berr;

				if (berr != 0)
					break;

				memcpy(outp_ucrcvdata,
					&uctxrxbuf[SC_T1_OFFSET_INF], nrxlen);
				outp_ucrcvdata += nrxlen;
				*outp_ulnumrcvbytes += nrxlen;

				ptpdu_t1->ncard = SC_T1_I_BLK_N(nrxpcb);
				if (ptpdu_t1->ncard == 0)
					ptpdu_t1->ncard = 1;
				else
					ptpdu_t1->ncard = 0;

				if (nrxpcb & SC_T1_I_BLK_MORE) {
					ptpdu_t1->scstate = SC_STATE_DATARCV;
					ptpdu_t1->scaction = SC_ACTION_TX_R;
					nrerr = 0;
				} else {
					return (babort == 0) ? BERR_SUCCESS :
							BSCD_STATUS_FAILED;
				}
				break;
			case SC_T1_R_BLK:
				if (!SC_T1_R_BLK_OK(nrxpcb) || (nrxlen != 0)) {
					berr = 1;
					break;
				}
				ptpdu_t1->scaction = SC_ACTION_TX_I;

				if (ptpdu_t1->ndevice ==
						SC_T1_R_BLK_N(nrxpcb)) {
					if (in_ulnumxmitbytes >= nsentlen) {
						inp_ucxmitdata    += nsentlen;
						in_ulnumxmitbytes -= nsentlen;
					} else {
						berr = 1;
					}
					break;
				}
				ptpdu_t1->ndevice = (ptpdu_t1->ndevice == 0) ?
								1 : 0;
				break;
			default:
				berr = 1;
			}
		}

		if (berr) {
			if (bscd_process_err(ptpdu_t1))
				return BSCD_STATUS_FAILED;
			nrerr = SC_T1_R_BLK_ERR_OTHER;
		}
	}
}
