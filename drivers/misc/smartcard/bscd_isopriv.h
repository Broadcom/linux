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

#ifndef BSCD_ISO_PRIV_H__
#define BSCD_ISO_PRIV_H__

#include "bscd.h"

int chnl_p_iso_atr_rdy_next_byte(struct p_chnl_hdl *hdl, unsigned char *data);

int chnl_p_iso_atr_chk_for_additional_bytes(struct p_chnl_hdl *hdl);

int chnl_p_iso_atr_rcv_and_decode(struct p_chnl_hdl *hdl);

int chnl_p_iso_validate_ts_byte(unsigned char in_ts_byte);

int chnl_p_iso_validate_t0_byte(unsigned char t0_byte,
				unsigned int  *num_historical_byte);

int chnl_p_iso_validate_ta1_byte(struct p_chnl_hdl *hdl, unsigned char ta1byte);

int chnl_p_iso_validate_tb1_byte(struct p_chnl_hdl *hdl, unsigned char tb1byte);

int chnl_p_iso_validate_tc1_byte(struct p_chnl_hdl *hdl, unsigned char tc1byte);

int chnl_p_iso_validate_td1_byte(struct p_chnl_hdl *hdl,
				 unsigned char in_td1_byte,
				 bool *out_b_tck_required);

int chnl_p_iso_validate_ta2_byte(struct p_chnl_hdl *hdl, unsigned char ta2byte);

int chnl_p_iso_validate_tb2_byte(unsigned char in_tb2_byte);

int chnl_p_iso_validate_tc2_byte(struct p_chnl_hdl *hdl, unsigned char tc2byte);

int chnl_p_iso_validate_td2_byte(struct p_chnl_hdl *hdl, unsigned char td2byte,
				 unsigned char td1byte,
				 bool *out_b_tck_required);

int chnl_p_iso_validate_ta3_byte(struct p_chnl_hdl *hdl, unsigned char ta3byte);

int chnl_p_iso_validate_tb3_byte(struct p_chnl_hdl *hdl, unsigned char tb3byte,
				 unsigned char in_ffactor,
				 unsigned char in_dfactor);

int chnl_p_iso_validate_tc3_byte(unsigned char tc3_byte);

int chnl_p_iso_validate_tck_byte(struct p_chnl_hdl *hdl, unsigned char *in_atr,
				 unsigned int in_atr_len);

int chnl_p_iso_atr_byte_read(struct p_chnl_hdl *hdl, unsigned char *inoutp_data,
			     unsigned long in_max_atr_byte_time_in_etu,
			     long in_max_total_atr_byte_time_in_etu,
			     unsigned long *inoutp_total_atr_byte_time_in_etu);
#endif /* BSCD_PRIV_H__ */
