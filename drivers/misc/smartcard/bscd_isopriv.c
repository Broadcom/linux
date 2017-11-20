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
#include <linux/io.h>
#include <linux/printk.h>

#include "bscd_datatypes.h"
#include "bscd_isopriv.h"
#include "bscd_priv.h"
#include "sci_regs.h"

#define BSTD_UNUSED(x)
int chnl_p_iso_atr_rdy_next_byte(struct p_chnl_hdl *hdl, unsigned char *data)
{
	int err = BERR_SUCCESS;
	unsigned long rxlen = 0;

	if (hdl->rxlen  ==  BSCD_MAX_ATR_SIZE) {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
	err = chnl_rcv_atr(hdl, data, &rxlen, 1);
	if (err != BERR_SUCCESS)
		goto bscd_p_done_label;

bscd_p_done_label:
	pr_devel("Leave  ISOATRReadNextByte err = 0x%x\n", err);
	return err;
}


int chnl_p_iso_atr_chk_for_additional_bytes(struct p_chnl_hdl *hdl)
{
	int err = BERR_SUCCESS;
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
	unsigned int val;
#endif
	unsigned char bval;
	unsigned long total_atr_byte_time_in_etu = 0;
	unsigned int  additional_byte_cnt = 0;

	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}

	pr_devel("%s\n", "In  chnl_p_iso_atr_chk_for_additional_bytes");
	while (err == BERR_SUCCESS) {

		/*
		 * Since GP is used for total ATR time and WWT is used for WWT,
		 * we use event2_intr for this 200 ETU checking
		 */

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
		/* 200 ETU */
		writel(hdl->baddr + BSCD_P_EVENT2_CMP, 200);

		/* start event src */
		val = BSCD_P_RX_ETU_TICK_EVENT_SRC;
		writel(hdl->baddr + BSCD_P_EVENT2_CMD_3, val);

		/* increment event src */
		val = BSCD_P_RX_ETU_TICK_EVENT_SRC;
		writel(hdl->baddr + BSCD_P_EVENT2_CMD_2, val);

		/* reset event src */
		val = BSCD_P_NO_EVENT_EVENT_SRC;
		writel(hdl->baddr + BSCD_P_EVENT2_CMD_1, val);

		/* event_en, intr_mode, run_after_reset and run_after_compare*/
		val = BCHP_SCA_SC_EVENT2_CMD_4_event_en_MASK |
			BCHP_SCA_SC_EVENT2_CMD_4_intr_after_compare_MASK |
			BCHP_SCA_SC_EVENT2_CMD_4_run_after_reset_MASK |
			BCHP_SCA_SC_EVENT2_CMD_4_run_after_compare_MASK;

		val &= ~(BCHP_SCA_SC_EVENT2_CMD_4_intr_after_reset_MASK);
		writel(hdl->baddr + BSCD_P_EVENT2_CMD_4, val);

		err = chnl_ena_int_cbisr(hdl, int_event2, chnl_p_event2_cbisr);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
#endif
		err = chnl_p_iso_atr_byte_read(hdl, &bval, 200,
					BSCD_MAX_EMV_ETU_FOR_ALL_ATR_BYTES,
					&total_atr_byte_time_in_etu);

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
			/* Disable event2 */
			val = readl(hdl->baddr + BSCD_P_EVENT2_CMD_4);
			val &= ~(BCHP_SCA_SC_EVENT2_CMD_4_event_en_MASK);
			writel(hdl->baddr + BSCD_P_EVENT2_CMD_4, val);
#endif

		if (err == BERR_SUCCESS) {
			pr_devel("In SmartCardATRCheckForAdditionalATRBytes:");
			pr_devel(" Extra Byte Detected\n");
			additional_byte_cnt++;
		}
	}

	if (additional_byte_cnt)
		err = BSCD_STATUS_FAILED;

bscd_p_done_label:
	return err;
}

int chnl_p_iso_atr_rcv_and_decode(struct p_chnl_hdl *hdl)
{
	int err = BERR_SUCCESS;
	unsigned int num_historical_byte = 0;
	unsigned char t0_byte = 0, td1_byte = 0, td2_byte = 0, td3_byte = 0;
	unsigned char tc1_byte = 0;
	unsigned int i;
	bool tck_required = false;
	unsigned char historical_bytes[50];

	/* assign negotiate settings to default */
	get_chnl_default_settings(NULL, 0, &(hdl->negotiated_chs));

	/* TS */
	hdl->rxlen = 0;
	err = chnl_p_iso_atr_rdy_next_byte(hdl, &(hdl->rxbuf[hdl->rxlen]));
	if (err != BERR_SUCCESS)
		goto bscd_p_done_label;

	err = chnl_p_iso_validate_ts_byte(hdl->rxbuf[(hdl->rxlen)++]);
	if (err != BERR_SUCCESS)
		goto bscd_p_done_label;

	/* T0 */
	err = chnl_p_iso_atr_rdy_next_byte(hdl, &(hdl->rxbuf[hdl->rxlen]));
	if (err != BERR_SUCCESS)
		goto bscd_p_done_label;

	err = chnl_p_iso_validate_t0_byte(hdl->rxbuf[hdl->rxlen],
					  &num_historical_byte);
	if (err != BERR_SUCCESS)
		goto bscd_p_done_label;

	t0_byte = hdl->rxbuf[(hdl->rxlen)++];

	hdl->is_pps_needed = false; /* default to NOT need PPS */

	if (t0_byte  & 0x10) { /* TA1 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
					&(hdl->rxbuf[hdl->rxlen]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		pr_devel("After T0 ISOATRReadNextByte err = 0x%x\n", err);

		err = chnl_p_iso_validate_ta1_byte(hdl,
				hdl->rxbuf[(hdl->rxlen)++]);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

	/*
	 * if TA1 is absent, no need to do since already set to default in
	 * get_chnl_default_settings()
	 */

	if (t0_byte & 0x20) {  /* TB1 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
				&(hdl->rxbuf[hdl->rxlen]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		err = chnl_p_iso_validate_tb1_byte(hdl,
					hdl->rxbuf[(hdl->rxlen)++]);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

	if (t0_byte & 0x40) {  /* TC1 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
					&(hdl->rxbuf[hdl->rxlen]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		tc1_byte =  hdl->rxbuf[(hdl->rxlen)++];
		err = chnl_p_iso_validate_tc1_byte(hdl, tc1_byte);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}
	/*
	 * if TC1 is absent from the ATR, default is 2etu
	 * (BSCD_DEFAULT_EXTRA_GUARD_TIME). Before we assign to 0 here
	 */

	if (t0_byte & 0x80) {  /* TD1 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
					&(hdl->rxbuf[hdl->rxlen]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		td1_byte = hdl->rxbuf[(hdl->rxlen)++];  /* Store td1_byte */
		err = chnl_p_iso_validate_td1_byte(hdl,
					td1_byte, &tck_required);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

	if (td1_byte & 0x10) {  /* TA2 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
					&(hdl->rxbuf[hdl->rxlen]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		err = chnl_p_iso_validate_ta2_byte(hdl,
				hdl->rxbuf[(hdl->rxlen)++]);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

	if (td1_byte & 0x20) {  /* TB2 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
				&(hdl->rxbuf[hdl->rxlen]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		err = chnl_p_iso_validate_tb2_byte(hdl->rxbuf[(hdl->rxlen)++]);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

	/*
	 * do not check err return, otherwise we would fail for the
	 * OberthurPIVv1.08 card which used RFU value of D=7
	 */
	p_fd_adjust_without_reg_update(&(hdl->negotiated_chs),
					hdl->negotiated_chs.ffactor,
					hdl->negotiated_chs.dfactor);

	hdl->negotiated_chs.curr_baud_rate =
			hdl->mod_hdl->cur_settings.mod_clk_freq.clk_freq /
				hdl->negotiated_chs.etu_clk_div /
				(hdl->negotiated_chs.prescale+1) /
				hdl->negotiated_chs.baud_div;

	if (td1_byte & 0x40) { /* TC2 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
				&(hdl->rxbuf[hdl->rxlen]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		err = chnl_p_iso_validate_tc2_byte(hdl,
					hdl->rxbuf[(hdl->rxlen)++]);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

	if (td1_byte & 0x80) {  /* TD2 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
					&(hdl->rxbuf[hdl->rxlen]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		td2_byte = hdl->rxbuf[(hdl->rxlen)++];  /* Store td1_byte */
		err = chnl_p_iso_validate_td2_byte(hdl, td2_byte,
						td1_byte, &tck_required);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

	if (td2_byte & 0x10) {  /* TA3 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
					&(hdl->rxbuf[hdl->rxlen]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		err = chnl_p_iso_validate_ta3_byte(hdl,
					hdl->rxbuf[(hdl->rxlen)++]);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

	if (td2_byte & 0x20) { /* TB3 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
					&(hdl->rxbuf[hdl->rxlen]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		err = chnl_p_iso_validate_tb3_byte(hdl,
					hdl->rxbuf[(hdl->rxlen)++],
					hdl->negotiated_chs.ffactor,
					hdl->negotiated_chs.dfactor);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

	if (td2_byte & 0x40) { /* TC3 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
					&(hdl->rxbuf[hdl->rxlen]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		err = chnl_p_iso_validate_tc3_byte(hdl->rxbuf[(hdl->rxlen)++]);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

	if (td2_byte & 0x80) {  /* TD3 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
					&(hdl->rxbuf[hdl->rxlen]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		td3_byte = hdl->rxbuf[(hdl->rxlen)++];  /* Store td1_byte */
	}

	if (td3_byte & 0x10) {  /* TA4 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
				&(hdl->rxbuf[(hdl->rxlen)++]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

	if (td3_byte & 0x20) { /* TB4 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
				&(hdl->rxbuf[(hdl->rxlen)++]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

	}

	if (td3_byte & 0x40) { /* TC4 is present */
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
				&(hdl->rxbuf[(hdl->rxlen)++]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

	}

	if (num_historical_byte) {
		for (i = 0; i < num_historical_byte; i++) {
			err = chnl_p_iso_atr_rdy_next_byte(hdl,
					&(hdl->rxbuf[(hdl->rxlen)]));
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;

			historical_bytes[i] = hdl->rxbuf[(hdl->rxlen)++];
		}
	}

	if (tck_required) {
		pr_devel("Checking for TCK Byte");
		err = chnl_p_iso_atr_rdy_next_byte(hdl,
				&(hdl->rxbuf[hdl->rxlen]));
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		err = chnl_p_iso_validate_tck_byte(hdl,
				hdl->rxbuf,  (hdl->rxlen)++);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

bscd_p_done_label:
	pr_devel("Leave ISOATRReceiveAndDecode  err = 0x%x\n", err);
	return err;
}

int chnl_p_iso_validate_ts_byte(unsigned char in_ts_byte)
{
	int err = BERR_SUCCESS;

	if (in_ts_byte == 0x3f) {
		pr_devel("TS = %02x, Inverse Convention\n", in_ts_byte);
		return err;
	} else if (in_ts_byte == 0x3b) {
		pr_devel("TS = %02x, Direct Convention\n", in_ts_byte);
		return err;
	}
	{
		pr_err("TS = %02x, Unknown Convention\n", in_ts_byte);
		err = BERR_TRACE(BSCD_STATUS_FAILED);
		return BSCD_STATUS_FAILED;
	}
}

int chnl_p_iso_validate_t0_byte(unsigned char t0_byte,
				unsigned int *num_historical_byte)
{
	*num_historical_byte = (unsigned int)(t0_byte & 0x0F);
	return BERR_SUCCESS;
}

int chnl_p_iso_validate_ta1_byte(struct p_chnl_hdl *hdl,
				 unsigned char in_ta1_byte)
{
	int err = BERR_SUCCESS;
	unsigned char ffactor, dfactor;

	pr_devel("in_ta1_byte = %d\n", in_ta1_byte);
	/* Decode TA1 (F and D adjustment). */
	ffactor = (in_ta1_byte >> 4) & 0x0f;
	dfactor = in_ta1_byte & 0x0f;

	hdl->negotiated_chs.ffactor = ffactor;
	hdl->negotiated_chs.dfactor = dfactor;

	/*
	 * For Negotiable mode, if F/D are not default values, need to do PPS.
	 * For Specific mode there will be TA2 byte, is_pps_needed can be
	 * overwritten there depends on TA2
	 */
	if ((ffactor != 1) || (dfactor != 1))
		hdl->is_pps_needed = true;

	return err;
}

int chnl_p_iso_validate_tb1_byte(struct p_chnl_hdl *hdl,
				 unsigned char in_tb1_byte)
{
	int err = BERR_SUCCESS;

	/* Decode TB1 (programming voltage) */
	if (in_tb1_byte == 0x00) {
		/*
		 * pr_devel("VPP is not connected in the ICC\n");
		 * pr_devel("No programming current\n");
		 */
	} else {
		/*
		 * According to EMV ATR spec, in response to a warm reset,
		 * the terminal shall accept an ATR containing TB1 of any value
		 */
		if (hdl->reset_type == rst_warm) {
			/*
			 * pr_err(("VPP is not connected in the ICC\n"));
			 * pr_err(("No programming current\n"));
			 */
		} else {
			pr_err("Non-Zero TB1 = %02x during", in_tb1_byte);
			pr_err("code reset. Not acceptable for EMV.\n");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
	}
bscd_p_done_label:
	return err;
}


int chnl_p_iso_validate_tc1_byte(struct p_chnl_hdl *hdl,
				 unsigned char in_tc1_byte)
{
	int val = 0;

	pr_devel("In  SmartCardValidateTC1Byte\n");
	/* Decode TC1 (guard time) */
	if ((hdl->negotiated_chs.proto_type == async_proto_e0) &&
		(in_tc1_byte == 0xff))  {

		/*
		 * When in T = 0 mode and tc1_byte == 0xff, simply set
		 * additional guardtime to zero ETUs.  This is to pass the
		 * test and it is different from EMV 96.
		 */
		hdl->negotiated_chs.extra_guard_time.value = 0;
	} else {
		/*
		 * use value of tc1_byte for additional guardtime,
		 * regardless of T = 0 or T = 1 mode
		 */
		hdl->negotiated_chs.extra_guard_time.value = in_tc1_byte;
	}

	hdl->negotiated_chs.extra_guard_time.unit = unit_etu;
	val = hdl->negotiated_chs.extra_guard_time.value;
	pr_devel("\nSmartCardValidateTC1Byte: ulGuardTime = 0x%x\n", val);

	return BERR_SUCCESS;
}

int chnl_p_iso_validate_td1_byte(struct p_chnl_hdl *hdl,
				 unsigned char in_td1_byte,
				 bool *out_b_tck_required)
{
	int err = BERR_SUCCESS;

	if  ((in_td1_byte & 0x0f) > 0x01) {
		/*
		 * If the lower nibble of td1_byte is not equal to either 0
		 * or 1, then return fail, otherwise return success
		 */
		pr_err("Erroneous TD1 l.s. nibble:");
		pr_err("%02x, should be either 0x00 or 0x01\n", in_td1_byte);

		*out_b_tck_required = true;
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	} else if ((in_td1_byte & 0x0f) == 0x01) {
		/* set protocol type to T = 1 */
		hdl->negotiated_chs.proto_type = async_proto_e1;/*added by fye*/
		*out_b_tck_required = true;
	}

bscd_p_done_label:
	return err;
}

int chnl_p_iso_validate_ta2_byte(struct p_chnl_hdl *hdl,
				 unsigned char in_ta2_byte)
{
	int err = BERR_SUCCESS;

	/* check bit5, e.g. 0x10 */
	if (in_ta2_byte & 0x10) {
		pr_err("Inval TA2:%02x,this uses implicit val\n", in_ta2_byte);
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	} else {
		pr_devel("TA2 present with b5=0\n");
		hdl->is_pps_needed = false;
	}
bscd_p_done_label:
	return err;
}

int chnl_p_iso_validate_tb2_byte(unsigned char in_tb2_byte)
{
	BSTD_UNUSED(in_tb2_byte);
	/* TB2 is not supported by EMV, therefore return failed */
	pr_err("TB2 is present,but not reqd for Europay std.Invalid ATR\n");
	return BSCD_STATUS_FAILED;
}

int chnl_p_iso_validate_tc2_byte(struct p_chnl_hdl *hdl,
				 unsigned char in_tc2_byte)
{
	int err = BERR_SUCCESS;

	pr_devel("%s\n", "In  chnl_p_iso_validate_tc2_byte");

	/*
	 * fye: enable this.java card sends us Tc2=255 so to have a
	 * longer WWT.
	 *
	 * Decode TC2.  NOTE: TC2 is specific to protocol type T = 0
	 */
	if ((in_tc2_byte == 0x00)) {
		/* Reject ATR if TC2 is equal to '0x00' or greater than '0x0A'*/
		pr_err("Invalid TC2 = %02x\n", in_tc2_byte);
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	} else {
		/* Reset work waiting time, using valid TC2 Value */
		/* Specify work wait time used for T = 0 protocol */
		err = p_adjust_wwt(&(hdl->negotiated_chs),
				   in_tc2_byte);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}
bscd_p_done_label:
	return err;
}

int chnl_p_iso_validate_td2_byte(struct p_chnl_hdl *hdl,
				 unsigned char in_td2_byte,
				 unsigned char in_td1_byte,
				 bool *out_b_tck_required)
{
	int err = BERR_SUCCESS;
	unsigned char t1, t2;

	BSTD_UNUSED(hdl);

	t1 = in_td1_byte & 0x0F;
	t2 = in_td2_byte & 0x0F;

	if (t1 == 0x00) {
		/*
		 * fye: we removed the error check here for TD2. If TD1 is
		 * T=0, TD2 can be any value
		 */
		/* this is to fix the Oberthur ID One V5.2 used by DoD */

		if (t2 != 0x00) {
			/* If TD1 and TD2 are not same, TCK is needed */
			*out_b_tck_required = true;
		}
	} else if (t1 == 0x01) {
		if (t2 == 0x00) {
			/*
			 * If l.s. nibble of TD1 is '1', then l.s. nibble of
			 * TD2 must not be 0 according to standard
			 */
			pr_devel("Fail inSmartCardValidateTD2Byte:TD2==0x00\n");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
	}

	/* if two protocols offered (T=0 to 14), then need PPS */
	/* No need to compare >=0 since t1/2 are unsigned char */
	if ((t1 != t2) && (t1 <= 14) && (t2 <= 14))
		hdl->is_pps_needed = true;

bscd_p_done_label:
	return err;
}

int chnl_p_iso_validate_ta3_byte(struct p_chnl_hdl *hdl,
				unsigned char in_ta3_byte)
{
	int err = BERR_SUCCESS;

	/* Decode TA3 according to the protocol */
	if (hdl->negotiated_chs.proto_type == async_proto_e0) {
		/*
		 * TA3 codes UI(card supports clock stop or not)
		 * and XI(class indicator)
		 * We dont hanlding this case for now
		 */
	} else if (hdl->negotiated_chs.proto_type == async_proto_e1) {
			/* TA3 codes IFSC */
		if ((in_ta3_byte <= 0x0f) || (in_ta3_byte == 0xff))  {
			pr_err("Invalid ISO TA3 = %02x\n", in_ta3_byte);
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		} else {
			hdl->negotiated_chs.curr_ifsc = in_ta3_byte;
		}
	}
bscd_p_done_label:
	return err;
}

int chnl_p_iso_validate_tb3_byte(struct p_chnl_hdl *hdl,
				 unsigned char in_tb3_byte,
				 unsigned char in_ffactor,
				 unsigned char in_dfactor)
{
	int err = BERR_SUCCESS;
	unsigned int  cwt_val;
	unsigned char cwi_val, bwi_val;

	if (hdl->negotiated_chs.proto_type == async_proto_e1) {
		/* Decode TB3. */
		cwi_val = in_tb3_byte & 0x0f;
		bwi_val = (in_tb3_byte >> 4) & 0x0f;

		/*
		 * Obtain the power(2,CWI) factor from the value of
		 * CWI - see TB3 in EMV'96 spec for more
		 */
		/* Set CWT */
		cwt_val = readl(hdl->baddr + BSCD_P_PROTO_CMD);

		/*
		 * and with 0xf0 to remove the original 0x0f inserted
		 * into this register
		 */
		cwt_val &= 0xf0;
		cwt_val |= cwi_val;
		hdl->negotiated_chs.char_wait_time_integer = cwi_val;

		pr_devel("cwt_val = 0x%x\n", cwt_val);

		/* set BWT */
		chnl_set_blk_wait_time_integer(&(hdl->negotiated_chs), bwi_val);
		pr_devel("TB3, blk_wait_time = %u\n",
			 hdl->negotiated_chs.blk_wait_time.value);

	}
	return err;
}

int chnl_p_iso_validate_tc3_byte(unsigned char in_tc3_byte)
{
	int err = BERR_SUCCESS;

	pr_devel("In BSCD_Channel_P_EMVValidateTC3Byte: 0x%02x\n", in_tc3_byte);

	/* Terminal shall reject ATR containing non-zero TC3 Byte */
	if (in_tc3_byte != 0) {
		pr_err("Failing in SmartCardValidateTC3Byte\n");
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
bscd_p_done_label:
	return err;
}

int chnl_p_iso_validate_tck_byte(struct p_chnl_hdl *hdl,
				 unsigned char *in_atr,
				 unsigned int in_atr_len)
{
	int err = BERR_SUCCESS;
	unsigned char tck_cmp = 0;
	unsigned int i;
	int val = 0;

	BSTD_UNUSED(hdl);

	/* Start from T0 to TCK.  Including historical bytes if they exist */
	for (i = 1; i <= in_atr_len; i++) {
		tck_cmp = tck_cmp ^ in_atr[i];
		val = in_atr[i];
		pr_devel("SmartCardValidateTCKByte in_atr[%d]:%02x\n", i, val);
	}
	if (tck_cmp != 0) {
		pr_err("Invalid TCK.\n");
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}

	/* TCK validated successfully */
bscd_p_done_label:
	return err;
}

int chnl_p_iso_atr_byte_read(struct p_chnl_hdl *hdl,
			     unsigned char *indata,
			     unsigned long in_max_atr_byte_time_in_etu,
			     long in_max_total_atr_byte_time_in_etu,
			     unsigned long *inoutp_total_atr_byte_time_in_etu)
{
	unsigned long flag;
	int err = BERR_SUCCESS;
	unsigned int timer_cnt_val1, timer_cnt_val2, timer_cnt_val;
	unsigned int status;
	struct bscd_timer timer = {timer_gpt, {gpt_timer_mode_immediate},
								false, false};
	struct bscd_timer wwt_timer = {timer_wait, {gpt_timer_mode_immediate},
								false, false};

	BSTD_UNUSED(in_max_atr_byte_time_in_etu);
	spin_lock_irqsave(&hdl->lock, flag);
	status = hdl->status2 = readl(hdl->baddr + BSCD_P_STATUS_2);
	pr_devel("Status2 = 0x%x\n", status);
	spin_unlock_irqrestore(&hdl->lock, flag);
	if ((status & STATUS2_REMPTY_MASK) == STATUS2_REMPTY_MASK) {

		/* Do not have any byte in SC_RECEIVE */
		err = chnl_p_wait_for_rcv(hdl);
		if (err != BERR_SUCCESS) {
			/*
			 * disable the timer, always return the prev error.
			 * Disable timer, which was enable upon receiving
			 * atr_intr.
			 */
			chnl_ena_dis_timer_isr(hdl, &timer);

			/* disable WWT.  This was enabled in activating time */
			wwt_timer.timer_mode.wait_time_mode = wtm_work;
			chnl_ena_dis_timer_isr(hdl, &wwt_timer);

			/*
			 * Read timer counter and accumulate it to
			 * inoutp_total_atr_byte_time_in_etu
			 */
			timer_cnt_val2 = readl(hdl->baddr + BSCD_P_TIMER_CNT_2);
			timer_cnt_val1 = readl(hdl->baddr + BSCD_P_TIMER_CNT_1);
			timer_cnt_val = (((unsigned int) timer_cnt_val2) << 8)
							| timer_cnt_val1;

			*inoutp_total_atr_byte_time_in_etu += timer_cnt_val;
			if (*inoutp_total_atr_byte_time_in_etu >
			(unsigned long)in_max_total_atr_byte_time_in_etu) {
				err = BSCD_STATUS_FAILED;
				goto bscd_p_done_label;
			}

			pr_devel("AfterWaitForRcvinISOATRByteRead:0x%x\n", err);
			return err;
		}
	} else {
		spin_lock_irqsave(&hdl->lock, flag);
		hdl->intr_status2 &= ~INTSTAT2_RCV_INTR_MASK;
		spin_unlock_irqrestore(&hdl->lock, flag);
		pr_devel("Cancel out RCV_INTR, pSc_intrStatus2 = 0x%x\n",
			hdl->intr_status2);
	}

	*indata = (unsigned char) readl(hdl->baddr + BSCD_P_RECEIVE);
	pr_devel("atr = 0x%x\n", *indata);
bscd_p_done_label:
	return err;
}
