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
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include "bscd_datatypes.h"
#include "bscd_priv.h"
#include "sci_regs.h"

/*
 * Default Module and Channel Settings.  Note that we could only modify
 * Module settings during open.
 */
static const struct bscd_settings bscd_def_scd_settings = {
	/* This attribute indicates the source of clock and the value */
	{clkfreq_src_internal_clk, BSCD_INTERNAL_CLOCK_FREQ},

	/* maximum SCD channels supported */
	BSCD_MAX_SUPPORTED_CHANNELS,
};

static const struct bscd_chnl_settings bscd_def_scd_chs = {
	/* Smart Card Standard */
	.sc_std = std_iso,

	/* Asynchronous Protocol Types. */
	.proto_type = async_proto_e0,

	/* Smart Card Types. */
	.ctx_card_type = {card_unknown, vcc_level_5V},

	/*
	 * This read-only attribute specifies the default
	 * source clock frequency in Hz.
	 */
	.src_clk_freq_hz = BSCD_INTERNAL_CLOCK_FREQ,

	/*
	 * ICC CLK frequency in Hz which is
	 * source freq / SC_CLK_CMD[etu_clk_div] / SC_CLK_CMD[sc_clk_div]
	 */
	.curr_icc_clk_freq = BSCD_INTERNAL_CLOCK_FREQ /
			     BSCD_DEFAULT_ETU_CLK_DIV /
			     BSCD_DEFAULT_SC_CLK_DIV,

	/* ETU in microseconds which is source freq / SC_CLK_CMD[etu_clk_div] */
	/* (SC_PRESCALE * external_clock_div + (external_clock_div - 1))  */
	.curr_baud_rate = BSCD_INTERNAL_CLOCK_FREQ / BSCD_DEFAULT_ETU_CLK_DIV /
			  (BSCD_DEFAULT_PRESCALE + 1) / BSCD_DEFAULT_BAUD_DIV,

	/* This read-only attribute specifies the maximum IFSD.Should be 264. */
	.max_ifsd = BSCD_MAX_TX_SIZE,

	/* This attribute indicates the current IFSD */
	.curr_ifsd = BSCD_DEFAULT_EMV_INFORMATION_FIELD_SIZE,

	/*
	 * Clock Rate Conversion Factor,
	 * F in 1,2,3,4,5,6,9, 10, 11, 12 or 13. Default is 1.
	 */
	.ffactor = BSCD_DEFAULT_F,

	/* Baud Rate Adjustment Factor, D in 1,2,3,4,5,6,8 or 9.Default is 1.*/
	.dfactor = BSCD_DEFAULT_D,

	/*
	 * ETU Clock Divider in
	 * SC_CLK_CMD register. Valid value is
	 * from 1 to 8. Default is 6.
	 */
	.etu_clk_div = BSCD_DEFAULT_ETU_CLK_DIV,

	/*
	 * SC Clock Divider in
	 * SC_CLK_CMD register. Valid value is
	 * 1,2,3,4,5,8,10,16. Default is 1.
	 */
	.sc_clk_div = BSCD_DEFAULT_SC_CLK_DIV,

	/* Prescale Value */
	.prescale = BSCD_DEFAULT_PRESCALE,

	/* external clock divisor */
	.external_clk_div = BSCD_DEFAULT_EXTERNAL_CLOCK_DIVISOR,

	/* Baud Divisor */
	.baud_div = BSCD_DEFAULT_BAUD_DIV,

	/*
	 * Number of transmit parity retries per character in
	 * SC_UART_CMD_2 register. Default is 4 and max is 6.
	 * 7 indicates infinite retries
	 */
	.tx_retries = BSCD_DEFAULT_TX_PARITY_RETRIES,

	/*
	 * Number of receive parity retries per character in
	 * SC_UART_CMD_2 register. Default is 4 and max is 6.
	 * 7 indicates infinite retries
	 */
	.rx_retries = BSCD_DEFAULT_RX_PARITY_RETRIES,

	/*
	 * work waiting time in SC_TIME_CMD register. Other than EMV
	 * standard, only valid if current protocol is T=0.
	 */
	.work_wait_time = {BSCD_DEFAULT_WORK_WAITING_TIME, unit_etu},

	/*
	 * block Wait time in SC_TIME_CMD register. Only valid if
	 * current protocol is T=1.
	 */
	.blk_wait_time = {BSCD_DEFAULT_BLOCK_WAITING_TIME, unit_etu},

	/* Extra Guard Time in SC_TGUARD register. */
	.extra_guard_time = {BSCD_DEFAULT_EXTRA_GUARD_TIME, unit_etu},

	/*
	 * block Guard time in SC_BGT register.Other than EMV
	 * standard, only valid if current protocol is T=1.
	 */
	.blk_guard_time = {BSCD_DEFAULT_BLOCK_GUARD_TIME, unit_etu},

	/*
	 * character Wait time in SC_PROTO_CMD register. Only valid
	 * if current protocol is T=1.
	 */
	.char_wait_time_integer = BSCD_DEFAULT_CHARACTER_WAIT_TIME_INTEGER,

	/* EDC encoding. Only valid if current protocol is T=1. */
	.edc_setting = {edc_encode_lrc, false},

	/* arbitrary Time Out value for any synchronous transaction. */
	.timeout = {BSCD_DEFAULT_TIME_OUT, unit_ms},

	/* Specify if we need auto deactivation sequence */
	.auto_deactive_req = false,

	/*
	 * True if we receive 0x60 in T=0, we will ignore it.
	 * Otherwise, we treat 0x60 as a valid data byte
	 */
	.null_filter = false,

	/* Debounce info for IF_CMD_2 */
	.pres_dbinfo = {mode_mask, true, BSCD_DEFAULT_DB_WIDTH},

	/* Tell driver whether to read/decode/program registers or not*/
	.rst_card_act = rst_card_act_rcv_decode,

	.blk_wait_time_ext = {0, unit_etu},  /* block wait time extension */

	.tpdu = false,                       /* IS packet in TPDU? */

	.curr_ifsc = BSCD_DEFAULT_IFSC
};

/* Default historical bytes for various card types */
#define COUNTOF(ary)   ((int)(sizeof(ary) / sizeof((ary)[0])))
static const struct bscd_card_type_settings bscd_def_card_type_settings[] = {
	{card_java, 5, {0xae, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
	},

	{card_java, 13, {0x80, 0x31, 0x80, 0x65, 0xb0, 0x83, 0x02, 0x04,
				0x7e, 0x83, 0x00, 0x90, 0x00, 0x00, 0x00}
	},

	{card_pki, 10, {0x4a, 0x43, 0x4f, 0x50, 0x34, 0x31, 0x56, 0x32,
				0x32, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00}
	},

	{card_acos, 3,
	/*
	 *  T1    T2    T3    T4    T5    T6    T7    T8    T9   T10   T11
	 *  T12   T13   T14   T15
	 *
	 * ACOS |Ver. |Rev. |Option Registers|Personalization File bytes |
	 * Stage | --  | --  | N/A
	 */
	 {0x41, 0x01, 0x38, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
						0x02, 0x90, 0x00, 0x00}
	}
};

/* Public Module Functions */
int bscd_chnl_apdu_transceive(struct p_chnl_hdl *hdl, unsigned char *xmit_data,
			      unsigned long xmit_bytes, unsigned char *rcv_data,
			      unsigned long *rcv_bytes,
			      unsigned long max_read_bytes)
{
	int err = BERR_SUCCESS;
#ifdef SCI_DEBUG
	unsigned int ii;
#endif
	struct apdu_s apdu;
	unsigned char *data = xmit_data;
	unsigned short cse;
	unsigned char cmd[5], ack;
	unsigned long actual_rxlen;
	unsigned int len = 0;
	unsigned int zero256 = 0; /*flag indicate if this is Le=0 => 256 case*/

#ifdef SCI_DEBUG
	pr_devel("BSCD TX %ld bytes: ", xmit_bytes);
	for (ii = 0; ii < xmit_bytes; ii++)
		pr_devel("%x ", xmit_data[ii]);
	pr_devel("\n");
#endif
	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	*rcv_bytes = 0;

	/* Check minimum APDU command/response size */
	if ((xmit_bytes < 4) || (xmit_bytes < 2))
		return -EINVAL;

	if (hdl->cur_chs.proto_type == async_proto_e0) { /* APDU through T= 0 */
		/* Parse APDU command buffer */
		memset(&apdu, 0, sizeof(struct apdu_s));
		apdu.data = rcv_data;
		apdu.cla = *data++;
		apdu.ins = *data++;
		apdu.p1 = *data++;
		apdu.p2 = *data++;

		/* Check the APDU command cases */
		if (xmit_bytes < 5) { /* case 1: Header only */
			cse = 1;
		} else if (xmit_bytes == 5) { /* case 2: Header + Le */
			cse = 2;
			apdu.le = (xmit_data[4]);
			if (apdu.le == 0) {
				/*
				 * jTOP PIV T=0 card sends "0 c0 0 0 0",
				 * we will fail this frame
				 */
				zero256 = 1;
			}
		} else {
			apdu.lc = xmit_data[4];
			if (xmit_bytes == (apdu.lc + 5)) {
				/* case 3: Header + Lc + data */
				cse = 3;
			} else if (xmit_bytes == (apdu.lc + 6)) {
				/* case 4: Header + Lc + data + Le */
				cse = 4;
				apdu.le = xmit_data[xmit_bytes - 1];
				if (apdu.le == 0)
					zero256 = 1;
			} else {
				return -EINVAL; /* invalid APDU command */
			}
		}

		/* Check if receive buffer is big enough */
		if (max_read_bytes < apdu.le + 2)
			return -EINVAL;

		/*
		 * clear return data (le bytes) in case of card response with
		 * last command result, in case it didn't finish last command
		 * (due to us timed out)
		 */
		memset(apdu.data, 0, apdu.le);

		/* Send 5-byte command header */
		if (cse == 1)
			memset(cmd, 0, 5);
		memcpy(cmd, xmit_data,  (cse == 1) ? 4 : 5);
		err = chnl_xmit(hdl, cmd, 5);
		if (err != BERR_SUCCESS)
			goto out;

		while (hdl->chnl_status.card_present == true) {
			err = chnl_rcv(hdl, &ack, &actual_rxlen, 1);
			if (err != BERR_SUCCESS)
				goto out;
			/*
			 * If receive null value '60' - wait a little.
			 * Send WTX indication to pc host.
			 * Do not need this now since there's no PC in 5892
			 * ccidctSendWTXIndication();
			 */
			if (ack == 0x60)
				continue;

			/* Check if it's SW1 || SW2 */
			if ((ack & 0xf0) == 0x60 || (ack & 0xf0) == 0x90) {
				apdu.sw[0] = ack;
				err = chnl_rcv(hdl, &ack, &actual_rxlen, 1);
				if (err != BERR_SUCCESS)
					goto out;
				apdu.sw[1] = ack;

				memcpy(apdu.data + len, apdu.sw, 2);
				len += 2;
				break;
			}

			/* Receive ACK (PB) - go ahead and send data */
			if ((ack == apdu.ins) && (apdu.lc)) {
				err = chnl_xmit(hdl, ++data, apdu.lc);
				if (err != BERR_SUCCESS)
					goto out;
				continue;
			}

			/* Receive ~ACK, send single byte only */
			if ((ack == (unsigned char)~apdu.ins) && (apdu.lc)) {
				err = chnl_xmit(hdl, ++data, 1);
				if (err != BERR_SUCCESS)
					goto out;
				apdu.lc--;
				continue;
			}

			/* 0 means 256 for this case */
			if (zero256 && (apdu.le == 0))
				apdu.le = 256;

			err = chnl_rcv(hdl, (apdu.data + len),
					&actual_rxlen, apdu.le);
			if (err != BERR_SUCCESS)
				goto out;
			len += actual_rxlen;
		}
		*rcv_bytes = len;
	} else if (hdl->cur_chs.proto_type == async_proto_e1) { /* APDU T= 1 */
		err = bscd_chnl_tpdu_transceive_t1(hdl, &(hdl->negotiated_chs),
				xmit_data, xmit_bytes, rcv_data,
				rcv_bytes, max_read_bytes);
		if (err)
			pr_err("%s err = %d\n", __func__, err);
	} else {
		err = BERR_NOT_SUPPORTED;
	}
out:
#ifdef SCI_DEBUG
	pr_devel("BSCD RX %ld bytes: ", *rcv_bytes);
	for (ii = 0; ii < *rcv_bytes; ii++)
		pr_err("%x ", rcv_data[ii]);
	pr_err("\n");
#endif
	return err;
}

int get_default_settings(struct bscd_settings *outp_settings)
{
	int err = BERR_SUCCESS;

	*outp_settings = bscd_def_scd_settings;

	return err;
}

int open(struct bscd_p_handle **outp_handle,
		const struct bscd_settings *inp_settings,
		enum phx_rfid_system coupler_type)
{
	int err = BERR_SUCCESS;
	struct bscd_p_handle *mod_hdl;
	unsigned int i;

	mod_hdl = kzalloc(sizeof(struct bscd_p_handle), GFP_KERNEL);
	if (mod_hdl == NULL) {
		/* pr_err("%s:%s\n", __func__, "open FAIL no memory"); */
		err = -ENOMEM;
		goto out;
	}

	mod_hdl->magic_number = BSCD_P_HANDLE_MAGIC_NUMBER;
	spin_lock_init(&mod_hdl->lock);
	if (inp_settings == NULL)
		mod_hdl->cur_settings = bscd_def_scd_settings;
	else
		mod_hdl->cur_settings = *inp_settings;

	mod_hdl->coupler_type = coupler_type;

	/* Set ICC CLK Freq */
	if ((mod_hdl->cur_settings.mod_clk_freq.freq_src ==
	     clkfreq_src_internal_clk) ||
	    (mod_hdl->cur_settings.mod_clk_freq.freq_src ==
	     clkfreq_src_unknown)) {
		mod_hdl->cur_settings.mod_clk_freq.freq_src =
			clkfreq_src_internal_clk;
		mod_hdl->cur_settings.mod_clk_freq.clk_freq =
			BSCD_INTERNAL_CLOCK_FREQ;
	} else if ((mod_hdl->cur_settings.mod_clk_freq.freq_src >
		    clkfreq_src_external_clk) ||
		   ((mod_hdl->cur_settings.mod_clk_freq.freq_src ==
		     clkfreq_src_external_clk) &&
		    (mod_hdl->cur_settings.mod_clk_freq.clk_freq == 0)
		   )
		  ) {
		err = BSCD_STATUS_FAILED;
		goto out;
	} else if ((mod_hdl->cur_settings.mod_clk_freq.freq_src ==
		    clkfreq_src_external_clk) &&
		   (mod_hdl->cur_settings.mod_clk_freq.clk_freq != 0)
		  ) {
	}

	/*
	 * If inp_settings->maxChannels == 0,
	 * set it to BSCD_MAX_SUPPORTED_CHANNELS
	 */
	if (mod_hdl->cur_settings.max_chan == 0)
		mod_hdl->cur_settings.max_chan = BSCD_MAX_SUPPORTED_CHANNELS;

	for (i = 0; i < mod_hdl->cur_settings.max_chan; i++)
		mod_hdl->chnl_hdls[i] = NULL;
	*outp_handle = mod_hdl;
out:
	return err;
}

int close(struct bscd_p_handle *inout_hdl)
{
	unsigned long flag;
	int err = BERR_SUCCESS;

	WARN_ON(!inout_hdl);

	spin_lock_irqsave(&inout_hdl->lock, flag);
	if (inout_hdl ==  NULL) {
		err = BSCD_STATUS_FAILED;
		spin_unlock_irqrestore(&inout_hdl->lock, flag);
		goto out;
	}
	spin_unlock_irqrestore(&inout_hdl->lock, flag);

	if (inout_hdl->magic_number != BSCD_P_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

out:

	spin_lock_irqsave(&inout_hdl->lock, flag);
	inout_hdl->magic_number = 0;
	spin_unlock_irqrestore(&inout_hdl->lock, flag);
	kfree(inout_hdl);
	inout_hdl = NULL;

	return err;
}


int get_chnl_default_settings(struct bscd_p_handle *in_hdl,
			      unsigned int in_chnl_num,
			      struct bscd_chnl_settings *outp_settings)
{
	int err = BERR_SUCCESS;

	*outp_settings = bscd_def_scd_chs;

	return err;
}

void bscd_remove_reset(struct p_chnl_hdl *chnl_hdl)
{
	unsigned int  value;

	value = IFCMD1_RST_MASK | readb(chnl_hdl->baddr + BSCD_P_IF_CMD_1);
	pr_devel("Enable sc rst signal to enable sci coupler writing:");
	pr_devel("0x%x to %p\n", value, (chnl_hdl->baddr + BSCD_P_IF_CMD_1));
	writeb(value, chnl_hdl->baddr + BSCD_P_IF_CMD_1);

	/* Wait 20ms for it to stabilize */
	msleep(20);
}

int chnl_open(struct bscd_p_handle *in_hdl, struct p_chnl_hdl **outp_chnl_hdl,
	      unsigned int in_chnl_num,
	      const struct bscd_chnl_settings *in_chnl_def_settings,
	      void __iomem *in_reg_base)
{
	unsigned long flag;
	int err = BERR_SUCCESS;
	struct p_chnl_hdl *chnl_hdl = NULL;

	WARN_ON(!in_hdl);

	if (in_hdl->magic_number != BSCD_P_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	if (in_chnl_num >= in_hdl->cur_settings.max_chan) {
		err = -EINVAL;
		goto out;
	}

	/* channel handle must be NULL.  */
	if (in_hdl->chnl_hdls[in_chnl_num] != NULL) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	*outp_chnl_hdl = NULL;

	chnl_hdl = kzalloc(sizeof(struct p_chnl_hdl), GFP_KERNEL);
	if (chnl_hdl == NULL) {
		err = -ENOMEM;
		goto out;
	}

	chnl_hdl->magic_number = BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER;
	chnl_hdl->mod_hdl = in_hdl;

	chnl_hdl->chnl_number = in_chnl_num;

	chnl_hdl->baddr = in_reg_base;
	spin_lock_init(&chnl_hdl->lock);
#ifdef BSCD_EMV2000_CWT_PLUS_4
	chnl_hdl->is_recv = false;
#endif
	spin_lock_irqsave(&chnl_hdl->lock, flag);
	writeb(0, chnl_hdl->baddr + BSCD_P_INTR_EN_1);
	writeb(0, chnl_hdl->baddr + BSCD_P_INTR_EN_2);
	chnl_hdl->status1 = 0x00;
	chnl_hdl->status2 = 0x00;
	chnl_hdl->intr_status1 = 0x00;
	chnl_hdl->intr_status2 = 0x00;
	spin_unlock_irqrestore(&chnl_hdl->lock, flag);

	pr_devel("in_chnl_num = %d\n", in_chnl_num);
	pr_devel("chnl_hdl->baddr = %p\n", chnl_hdl->baddr);

	if (in_chnl_def_settings != NULL) {
		pr_devel("Using input settings for this channel\n");
		err = chnl_set_params(chnl_hdl, in_chnl_def_settings);
	} else {
		pr_devel("Using bscd_def_scd_chs for this channel\n");
		err = chnl_set_params(chnl_hdl, &bscd_def_scd_chs);
	}
	if (err != BERR_SUCCESS) {
		pr_err("sci: failed to get chnl setting\n");
		goto out;
	}

	/* Set VCC level */
	chnl_set_vcc_level(chnl_hdl, chnl_hdl->cur_chs.ctx_card_type.vcc_level);

	/*
	 * we don't need to call chnl_p_ena_intr_isr() here since is_open is
	 * not set
	 */

	err = create_event(&(chnl_hdl->chnl_wait_event.card_wait));
	if (err != BERR_SUCCESS)
		goto out;
	err = create_event(&(chnl_hdl->chnl_wait_event.tdone_wait));
	if (err != BERR_SUCCESS)
		goto out;
	err = create_event(&(chnl_hdl->chnl_wait_event.rcv_wait));
	if (err != BERR_SUCCESS)
		goto out;
	err = create_event(&(chnl_hdl->chnl_wait_event.atr_start));
	if (err != BERR_SUCCESS)
		goto out;
	err = create_event(&(chnl_hdl->chnl_wait_event.timer_wait));
	if (err != BERR_SUCCESS)
		goto out;

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
	err = create_event(&(chnl_hdl->chnl_wait_event.event1_wait));
	if (err != BERR_SUCCESS)
		goto out;
#endif
	err = create_event(&(chnl_hdl->chnl_wait_event.event2_wait));
	if (err != BERR_SUCCESS)
		goto out;
	err = create_event(&(chnl_hdl->chnl_wait_event.bh_pres));
	if (err != BERR_SUCCESS)
		goto out;

	in_hdl->chnl_hdls[in_chnl_num] = chnl_hdl;

	*outp_chnl_hdl = chnl_hdl;
	spin_lock_irqsave(&chnl_hdl->lock, flag);
	chnl_hdl->is_open = true;
	spin_unlock_irqrestore(&chnl_hdl->lock, flag);
out:
	if (err != BERR_SUCCESS)
		if (chnl_hdl != NULL) {
			if (chnl_hdl->chnl_wait_event.card_wait != NULL)
				kfree(chnl_hdl->chnl_wait_event.card_wait);
			if (chnl_hdl->chnl_wait_event.tdone_wait != NULL)
				kfree(chnl_hdl->chnl_wait_event.tdone_wait);
			if (chnl_hdl->chnl_wait_event.rcv_wait != NULL)
				kfree(chnl_hdl->chnl_wait_event.rcv_wait);
			if (chnl_hdl->chnl_wait_event.atr_start != NULL)
				kfree(chnl_hdl->chnl_wait_event.atr_start);
			if (chnl_hdl->chnl_wait_event.timer_wait != NULL)
				kfree(chnl_hdl->chnl_wait_event.timer_wait);
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
			if (chnl_hdl->chnl_wait_event.atr_start != NULL)
				kfree(chnl_hdl->chnl_wait_event.event1_wait);
#endif
			if (chnl_hdl->chnl_wait_event.timer_wait != NULL)
				kfree(chnl_hdl->chnl_wait_event.event2_wait);
			if (chnl_hdl->chnl_wait_event.bh_pres != NULL)
				kfree(chnl_hdl->chnl_wait_event.bh_pres);

			kfree(chnl_hdl);
		}
	return err;
}

int chnl_close(struct bscd_p_handle *in_hdl, unsigned int in_chnl_num)
{
	unsigned long flag;
	struct p_chnl_hdl *chhdl = in_hdl->chnl_hdls[in_chnl_num];
	int err = BERR_SUCCESS;
	struct bscd_p_handle *mod_hdl;

	WARN_ON(!chhdl);

	if (chhdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	if (chhdl->is_open ==  false) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}
out:
	chhdl->is_open = false;
	pr_devel("DestroyEvents...\n");
	kfree(chhdl->chnl_wait_event.card_wait);
	kfree(chhdl->chnl_wait_event.tdone_wait);
	kfree(chhdl->chnl_wait_event.rcv_wait);
	kfree(chhdl->chnl_wait_event.atr_start);
	kfree(chhdl->chnl_wait_event.timer_wait);
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
	kfree(chhdl->chnl_wait_event.event1_wait);
#endif
	kfree(chhdl->chnl_wait_event.event2_wait);
	kfree(chhdl->chnl_wait_event.bh_pres);

	pr_devel("Channel_Deactivate...\n");
	chnl_deactivate(chhdl);
	mod_hdl = chhdl->mod_hdl;
	mod_hdl->chnl_hdls[chhdl->chnl_number] = NULL;

	pr_devel("Free chhdl ...\n");
	local_irq_save(flag);
	chhdl->magic_number = 0;
	kfree(chhdl);
	chhdl = NULL;
	local_irq_restore(flag);

	return err;
}

int get_chnl(struct bscd_p_handle *in_hdl, unsigned int in_chnl_num,
	     struct p_chnl_hdl **outp_chnl_hdl)
{
	int err = BERR_SUCCESS;
	struct p_chnl_hdl *chnl_hdl = NULL;

	WARN_ON(!in_hdl);

	*outp_chnl_hdl = NULL;
	if (in_hdl->magic_number != BSCD_P_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	if (in_chnl_num >= in_hdl->cur_settings.max_chan) {
		err = -EINVAL;
		goto out;
	}

	chnl_hdl = in_hdl->chnl_hdls[in_chnl_num];

	if (chnl_hdl == NULL) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	if (chnl_hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	if (chnl_hdl->is_open ==  false) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	*outp_chnl_hdl = chnl_hdl;
out:
	return err;
}

bool chnl_is_card_activated(struct p_chnl_hdl *hdl)
{
	int err = BERR_SUCCESS;

	WARN_ON(!hdl);
	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	if (hdl->is_open ==  false) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	return hdl->chnl_status.card_activate;
out:
	return false;
}

bool bscd_chnl_is_pps_done(struct p_chnl_hdl *hdl)
{
	int err = BERR_SUCCESS;

	WARN_ON(!hdl);
	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	if (chnl_is_card_activated(hdl))
		return hdl->chnl_status.pps_done;
out:
	return false;
}

int chnl_detect_card_non_blk(struct p_chnl_hdl *hdl,
			enum bscd_card_present in_ecard_present)
{
	unsigned long flag;
	int err = BERR_SUCCESS;

	WARN_ON(!hdl);

	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	if (hdl->is_open ==  false) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	hdl->status1 = readb(hdl->baddr + BSCD_P_STATUS_1);

	if (!(hdl->chnl_status.card_present)) {
		pr_devel("WaitEvent 50ms for bh_pres\n");
		wait_for_event(hdl->chnl_wait_event.bh_pres, 50);
		pr_devel("End of Wait\n");
	}
	switch (in_ecard_present) {
	case card_inserted:
		spin_lock_irqsave(&hdl->lock, flag);
		if (hdl->status1 & STATUS1_CARD_PRES_MASK) {
			hdl->chnl_status.card_present = true;
			spin_unlock_irqrestore(&hdl->lock, flag);
			goto out;
		} else {
			err = BERR_UNKNOWN;
			pr_devel("SC Not Present, Please insert the SC\n");
		}
		spin_unlock_irqrestore(&hdl->lock, flag);
	break;
	case card_removed:
		spin_lock_irqsave(&hdl->lock, flag);
		if (!(hdl->status1 & STATUS1_CARD_PRES_MASK)) {
			hdl->chnl_status.card_present = false;
			spin_unlock_irqrestore(&hdl->lock, flag);
			goto out;
		} else {
			err = BERR_UNKNOWN;
			pr_devel("SC Present, Please remove the SmartCard\n");
		}
		spin_unlock_irqrestore(&hdl->lock, flag);
	break;
	default:
		break;
	}
out:
	return err;
}

int bscd_chnl_set_detect_card_cb(struct p_chnl_hdl *hdl,
				 enum bscd_card_present in_ecard_present,
				 isrcb in_callback)
{
	int err = BERR_SUCCESS;

	WARN_ON(!hdl);
	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	if (hdl->is_open == false) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	switch (in_ecard_present) {
	case card_inserted:
		err = chnl_ena_int_cbisr(hdl, int_card_insert, in_callback);
	break;
	case card_removed:
		err = chnl_ena_int_cbisr(hdl, int_card_remove, in_callback);
	break;
	}
out:
	return err;
}


int chnl_set_params(struct p_chnl_hdl *hdl,
			const struct bscd_chnl_settings *inp_settings)
{
	int err = BERR_SUCCESS;
	unsigned int  value = 0;

	WARN_ON(!hdl);
	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	/*  Smart Card Standard */
	if ((inp_settings->sc_std <= std_unknown) ||
	    (inp_settings->sc_std > std_es)) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	hdl->cur_chs.sc_std = inp_settings->sc_std;
	pr_devel("sc_std = %d\n", hdl->cur_chs.sc_std);

	err = chnp_p_set_standard(hdl, inp_settings);
	if (err != BERR_SUCCESS) {
		pr_err("sci: chnl std setting failed\n");
		goto out;
	}

	err =  chnp_p_set_freq(hdl, inp_settings);
	if (err != BERR_SUCCESS) {
		pr_err("sci: chnl freq setting failed\n");
		goto out;
	}

	/* Set Vcc level used */
	hdl->cur_chs.ctx_card_type.vcc_level =
			inp_settings->ctx_card_type.vcc_level;
	hdl->cur_chs.ctx_card_type.inited = inp_settings->ctx_card_type.inited;

	/* Set maximum IFSD */
	hdl->cur_chs.max_ifsd = BSCD_MAX_TX_SIZE;
	pr_devel("max_ifsd = %ld\n", hdl->cur_chs.max_ifsd);

	/* Set current IFSD */
	if (inp_settings->curr_ifsd > BSCD_MAX_TX_SIZE) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	if (inp_settings->max_ifsd == 0)
		hdl->cur_chs.curr_ifsd = BSCD_MAX_TX_SIZE;
	else
		hdl->cur_chs.curr_ifsd = inp_settings->curr_ifsd;

	pr_devel("curr_ifsd = %ld\n", hdl->cur_chs.curr_ifsd);

	/* Set current IFSC */
	hdl->cur_chs.curr_ifsc = inp_settings->curr_ifsc;
	pr_devel("curr_ifsc = %d\n", hdl->cur_chs.curr_ifsc);

	err = chnp_p_set_edc_parity(hdl, inp_settings);
	if (err != BERR_SUCCESS)
		goto out;

	err = chnp_p_set_wait_time(hdl, inp_settings);
	if (err != BERR_SUCCESS)
		goto out;
	err =  chnp_p_set_guard_time(hdl, inp_settings);
	if (err != BERR_SUCCESS)
		goto out;

	/* Set transaction time out */
	err = chnp_p_set_transmission_time(hdl, inp_settings);
	if (err != BERR_SUCCESS)
		goto out;
	/* auto deactivation sequence */
	hdl->cur_chs.auto_deactive_req = inp_settings->auto_deactive_req;
	pr_devel("auto_deactive_req = %d\n", hdl->cur_chs.auto_deactive_req);

	/* nullFilter */
	hdl->cur_chs.null_filter = inp_settings->null_filter;
	pr_devel("null_filter = %d\n", hdl->cur_chs.null_filter);

	/* debounce info */
	if (inp_settings->pres_dbinfo.db_width > BSCD_MAX_DB_WIDTH) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	hdl->cur_chs.pres_dbinfo = inp_settings->pres_dbinfo;
	pr_devel("is_enabled = %d\n", hdl->cur_chs.pres_dbinfo.is_enabled);
	pr_devel("db_width = %d\n", hdl->cur_chs.pres_dbinfo.db_width);
	pr_devel("sc_pres_mode = %d\n", hdl->cur_chs.pres_dbinfo.sc_pres_mode);

	/*
	 * Specify if we want the driver to read, decode and program
	 * registers
	 */
	hdl->cur_chs.rst_card_act = inp_settings->rst_card_act;
	pr_devel("rst_card_act = %d\n", hdl->cur_chs.rst_card_act);


	/* Update the BSCD_P_PRESCALE */
	value = readb(hdl->baddr + BSCD_P_PRESCALE);
	pr_devel("orig BSCD_P_PRESCALE = 0x%x\n", value);


	writeb(hdl->cur_chs.prescale, hdl->baddr + BSCD_P_PRESCALE);
	pr_devel("New BSCD_P_PRESCALE = 0x%lx\n", hdl->cur_chs.prescale);

	/*
	 * Don't enable clock here since auto_clk need to be set first in
	 * ResetIFD before clock enabling for auto_deactivation
	 */
	value = readb(hdl->baddr + BSCD_P_CLK_CMD);
	pr_devel("orig ucClkCmd = 0x%x\n", value);

	/* If enabled before, change the the value.Otherwise leave it intact. */
	value = value & CLKCMD_CLK_ENA_MASK;
	if (value == CLKCMD_CLK_ENA_MASK) {
		value = value |
			(p_map_scclk_div_to_mask_value(
				hdl->cur_chs.sc_clk_div)) |
			((hdl->cur_chs.etu_clk_div - 1) << 1) |
			((hdl->cur_chs.baud_div == 31) ? 0 : 1);

		writeb(value, hdl->baddr + BSCD_P_CLK_CMD);
		if (hdl->cur_chs.baud_div == 25) {
			value = readb(hdl->baddr + BSCD_P_FLOW_CMD);
			value = 0x80 | value;
			writeb(value, hdl->baddr + BSCD_P_FLOW_CMD);
		}
		pr_devel("New SC_CLK_CMD = 0x%x\n", value);
	}

	pr_devel("UART_CMD_2addr :%p\n", (hdl->baddr + BSCD_P_UART_CMD_2));
	/* Update the BSCD_P_UART_CMD_2 */
	value = readb(hdl->baddr + BSCD_P_UART_CMD_2);
	pr_devel("orig BSCD_P_UART_CMD_2 = 0x%x\n", value);

	value &= (UARTCMD2_CONVENTION_MASK);
	if (inp_settings->proto_type == async_proto_e0)
		value |= (hdl->cur_chs.rx_retries << UARTCMD2_RPAR_RETRY_SHIFT)
				|(hdl->cur_chs.tx_retries);

	writeb(value, hdl->baddr + BSCD_P_UART_CMD_2);
	pr_devel("BSCD_P_UART_CMD_2 = 0x%x\n", value);

	/* Update the BSCD_P_PROTO_CMD */
	value =  readb(hdl->baddr + BSCD_P_PROTO_CMD);
	if ((inp_settings->proto_type == async_proto_e1) &&
	    (hdl->cur_chs.edc_setting.is_enabled)) {
		value =  PROTOCMD_EDC_ENA_MASK;
		if (hdl->cur_chs.edc_setting.edc_encode == edc_encode_lrc)
			value &=  ~PROTOCMD_CRC_LRC_MASK;
		else if (hdl->cur_chs.edc_setting.edc_encode == edc_encode_crc)
			value |= PROTOCMD_CRC_LRC_MASK;
	} else {
		value &= ~PROTOCMD_EDC_ENA_MASK;
	}

	value |= hdl->cur_chs.char_wait_time_integer;
	writeb(value, hdl->baddr + BSCD_P_PROTO_CMD);

	/* Update the BSCD_P_FLOW_CMD */
	value = 0;
	if (hdl->cur_chs.sc_std == std_nds)
		value =  FLOWCMD_FLOW_ENA_MASK;
	else
		value &=  ~FLOWCMD_FLOW_ENA_MASK;
	writeb(value, hdl->baddr + BSCD_P_FLOW_CMD);

	/* Update the BSCD_P_IF_CMD_2 */
	value = 0;
	if (hdl->cur_chs.pres_dbinfo.is_enabled == true)
		value =  IFCMD2_DB_EN_MASK;
	else
		value &=  ~IFCMD2_DB_EN_MASK;

	if (hdl->cur_chs.pres_dbinfo.sc_pres_mode == mode_mask)
		value |= IFCMD2_DB_MASK_MASK;
	else if (hdl->cur_chs.pres_dbinfo.sc_pres_mode == mode_debounce)
		value &= ~IFCMD2_DB_MASK_MASK;

	value |= hdl->cur_chs.pres_dbinfo.db_width;
	writeb(value, hdl->baddr + BSCD_P_IF_CMD_2);

	/* Update the BSCD_P_TGUARD */
	writeb(hdl->cur_chs.extra_guard_time.value, hdl->baddr + BSCD_P_TGUARD);

	/* for T=1 we always use TPDU, e.g. we do not build T=1 APDU */
	hdl->cur_chs.tpdu = true;
out:
	return err;
}

int chnl_get_params(struct p_chnl_hdl *hdl,
		    struct bscd_chnl_settings *outp_settings)
{
	int err = BERR_SUCCESS;

	WARN_ON(!hdl);
	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	*outp_settings = hdl->cur_chs;
out:
	return err;
}

int chnl_get_negotiate_params_ptr(struct p_chnl_hdl *hdl,
				  struct bscd_chnl_settings **outp_settings)
{
	int err = BERR_SUCCESS;

	WARN_ON(!hdl);
	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	*outp_settings = &(hdl->negotiated_chs);
out:
	return err;
}


int chnl_get_chnl_num(struct p_chnl_hdl *hdl)
{
	int err = BERR_SUCCESS;

	WARN_ON(!hdl);
	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}
out:
	if (err == BERR_SUCCESS)
		return hdl->chnl_number;
	else
		return  -1;
}


int chnl_deactivate(struct p_chnl_hdl *hdl)
{
	int err = BERR_SUCCESS;
	unsigned int value;

	WARN_ON(!hdl);
	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	/* Disable all interrupts */
	writeb(0, hdl->baddr + BSCD_P_INTR_EN_1);
	writeb(0, hdl->baddr + BSCD_P_INTR_EN_2);

	/* Turn off VCC */
	chnl_reset_power_icc(hdl, pwricc_pwrdwn);

	/* Set RST = 0.     */
	chnl_reset_signal(hdl, 0);

	/* Set CLK = 0.      */
	writeb(0, hdl->baddr + BSCD_P_CLK_CMD);

	/* Set IO = 0.      */
	value = ~IFCMD1_IO_MASK &
		 readb(hdl->baddr + BSCD_P_IF_CMD_1);
	writeb(value, hdl->baddr + BSCD_P_IF_CMD_1);

	/* Reset Tx & Rx buffers.   */
	writeb(~UARTCMD1_IO_ENA_MASK, hdl->baddr + BSCD_P_UART_CMD_1);
	value = PROTOCMD_RBUF_RST_MASK | PROTOCMD_TBUF_RST_MASK;
	writeb(value, hdl->baddr + BSCD_P_PROTO_CMD);
out:
	return err;
}

int chnl_reset_ifd(struct p_chnl_hdl *hdl, enum bscd_rst_type in_reset_type)
{
	int err = BERR_SUCCESS, val = 0;
	unsigned int if_cmd_val = 0, value;
	struct bscd_timer timer = {timer_gpt,
				   {gpt_timer_mode_immediate},
				   true, true};
	struct bscd_timer_value time_val = {2, unit_etu};

	WARN_ON(!hdl);
	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	/* Reset all status */
	hdl->status1 = 0;
	hdl->status2 = 0;
	hdl->intr_status1 = 0;
	hdl->intr_status2 = 0;
	hdl->chnl_status.status1 = 0;
	if (in_reset_type == rst_cold) {
		hdl->chnl_status.card_present = false;
		/*
		 * 09/20/05,Allen.C, reset is_card_removed after card removed
		 * and reinitialize
		 */
		hdl->is_card_removed = false;
	}
	/* Reset some critical registers */
	writeb(0, hdl->baddr + BSCD_P_TIMER_CMD);
	writeb(0, hdl->baddr + BSCD_P_INTR_EN_1);
	writeb(0, hdl->baddr + BSCD_P_INTR_EN_2);
	writeb(0, hdl->baddr + BSCD_P_UART_CMD_1);
	writeb(0, hdl->baddr + BSCD_P_UART_CMD_2);

	/* Set up debounce filter */
	if (hdl->cur_chs.pres_dbinfo.is_enabled == true) {
		value = IFCMD2_DB_EN_MASK;
		if (hdl->cur_chs.pres_dbinfo.sc_pres_mode == mode_mask)
			value |= IFCMD2_DB_MASK_MASK;
		value |= hdl->cur_chs.pres_dbinfo.db_width;
		writeb(value, hdl->baddr + BSCD_P_IF_CMD_2);
	} else {
		writeb(0, hdl->baddr + BSCD_P_IF_CMD_2);
	}
	val = readb(hdl->baddr + BSCD_P_IF_CMD_2);
	pr_devel("%s: debounce info BSCD_P_IF_CMD_2 = 0x%x\n", __func__, val);

	/* Cold Reset or Warm Reset */
	if_cmd_val = readb(hdl->baddr + BSCD_P_IF_CMD_1);
	pr_devel("%s: Before Cold Reset", __func__);
	pr_devel("BSCD_P_IF_CMD_1 = 0x%x\n", if_cmd_val);
	if (in_reset_type == rst_cold) {
		pr_devel("Cold Reset\n");
		hdl->reset_type = rst_cold;  /* Cold Reset */
		chnl_reset_power_icc(hdl, pwricc_pwrdwn);
		if_cmd_val |= IFCMD1_PRES_POL_MASK;
		writeb(if_cmd_val, hdl->baddr + BSCD_P_IF_CMD_1);
	} else {
		pr_devel("Warm Reset\n");
		hdl->reset_type = rst_warm;  /* Warm Reset */
	}
	pr_devel("%s: After Cold Reset-", __func__);
	pr_devel("BSCD_P_IF_CMD_1 = 0x%x\n", if_cmd_val);

	/* Use Auto Deactivation instead of TDA8004 */
	if (hdl->cur_chs.auto_deactive_req == true) {
		val = readb(hdl->baddr + BSCD_P_CLK_CMD);
		pr_devel("Before auto clk BSCD_P_CLK_CMD 0x%x\n", val);
		if_cmd_val |= IFCMD1_AUTO_CLK_MASK;
		writeb(if_cmd_val, hdl->baddr + BSCD_P_IF_CMD_1);
	}
	/* Set Clk cmd */
	value = CLKCMD_CLK_ENA_MASK |
		(p_map_scclk_div_to_mask_value(hdl->cur_chs.sc_clk_div)) |
		((hdl->cur_chs.etu_clk_div - 1) << 1)  |
		((hdl->cur_chs.baud_div == 31) ? 0 : 1);

	pr_devel("Reset: BCM_SC_CLK_CMD = 0x%x\n", (unsigned int)value);

	writeb(value, hdl->baddr + BSCD_P_CLK_CMD);
	writeb(hdl->cur_chs.prescale, hdl->baddr + BSCD_P_PRESCALE);

	pr_devel("Reset: BSCD_P_PRESCALE = 0x%lx\n", hdl->cur_chs.prescale);

	/* Use Auto Deactivation instead of TDA8004 */
	if (hdl->cur_chs.auto_deactive_req == true) {
		pr_devel("Before auto io if_cmd_val = 0x%x\n", if_cmd_val);
		if_cmd_val |= IFCMD1_AUTO_IO_MASK;
		writeb(if_cmd_val, hdl->baddr + BSCD_P_IF_CMD_1);
		pr_devel("After auto io if_cmd_val = 0x%x\n", if_cmd_val);
	}

	chnl_reset_signal(hdl, 1);
	value = 0;
	writeb(value, hdl->baddr + BSCD_P_UART_CMD_1);
	pr_devel("%s:Before SmartCardEnableInt\n", __func__);

	/* Enable 2 interrupts with callback */
	err = chnl_ena_int_cbisr(hdl, int_card_insert,
				 chnl_p_card_insert_cbisr);
	if (err != BERR_SUCCESS) {
		pr_err("sci:Failed to enable card insertion intr\n");
		goto out;
	}
	err = chnl_ena_int_cbisr(hdl, int_card_remove,
				 chnl_p_card_remove_cbisr);
	if (err != BERR_SUCCESS) {
		pr_err("sci: Railed to enable card remove intr\n");
		goto out;
	}
	value = readb(hdl->baddr + BSCD_P_INTR_EN_1);
	pr_devel("%s: After ena insert/remove intr,", __func__);
	pr_devel("BSCD_P_INTR_EN_1:0x%x\n", value);

	writeb(UARTCMD1_UART_RST_MASK, hdl->baddr + BSCD_P_UART_CMD_1);
	udelay(100);
	/*
	 * UART Reset should be set within 1 ETU (however, we are generous
	 * to give it 2 etus.
	 */
	err = chnl_config_timer(hdl, &timer, &time_val);
	if (err != BERR_SUCCESS) {
		pr_err("sci: failed to configure gpt for 2 etu\n");
		goto out;
	}

	/*
	 * do not check for card removal, since it has nothing
	 * to do with the card
	 */
	err = chnl_p_wait_for_timer_event(hdl, 0);
	if (err != BERR_SUCCESS)
		goto out;
	/*
	 * this timer is setup to wait for UART reset done, after UART reset
	 * done, disable timer ????? if time out, isr event is called.
	 */

	/* Disable timer */
	timer.is_timer_intr_ena = false;
	timer.is_timer_ena = false;
	err = chnl_ena_dis_timer_isr(hdl, &timer);
	if (err != BERR_SUCCESS)
		goto out;

	value = readb(hdl->baddr + BSCD_P_UART_CMD_1);

	/* If equal to zero, then UART reset has gone low, so return success */
	if ((value & UARTCMD1_UART_RST_MASK) == 0) {
		pr_devel("Reset UART Success\n");

		/*
		 * INITIAL_CWI_SC_PROTO_CMD = 0x0f is required so that
		 * CWI does not remain equal to zero, which causes an
		 * erroneous timeout, the CWI is set correctly in the
		 * SmartCardEMVATRDecode procedure
		 */
		pr_devel("Reset tx/rx buffer\n");
		value = PROTOCMD_TBUF_RST_MASK | PROTOCMD_RBUF_RST_MASK;
		writeb(value, hdl->baddr + BSCD_P_PROTO_CMD);
	}
out:
	return err;
}

int chnl_reset_power_icc(struct p_chnl_hdl *hdl,
			 enum bscd_power_icc in_icc_action)
{
	int err = BERR_SUCCESS;
	unsigned int value;

	WARN_ON(!hdl);
	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	switch (in_icc_action) {
	case pwricc_pwrup:
		pr_devel("To power up NXP8024 coupler");
		value = ~IFCMD1_VCC_MASK & readb(hdl->baddr + BSCD_P_IF_CMD_1);
		writeb(value, hdl->baddr + BSCD_P_IF_CMD_1);
		break;
	case pwricc_pwrdwn:
		pr_devel("To power down NXP8024 coupler");
		value = IFCMD1_VCC_MASK | readb(hdl->baddr + BSCD_P_IF_CMD_1);
		writeb(value, hdl->baddr + BSCD_P_IF_CMD_1);
		break;
	default:
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	value = readb(hdl->baddr + BSCD_P_IF_CMD_1);
	pr_devel("After PowerICC, BSCD_P_IF_CMD_1 = 0x%x\n", value);
	/* both power On/Off ccid message should lead to reactivate card */
	hdl->chnl_status.card_activate = false;
	hdl->chnl_status.pps_done = false;
out:
	return err;
}

/* This handles the RST signal (high/low, e.g. true/false) to the ICC */
int chnl_reset_signal(struct p_chnl_hdl *hdl, bool in_rst)
{
	int err = BERR_SUCCESS;
	unsigned int value;

	WARN_ON(!hdl);
	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	if (in_rst) {
		value = IFCMD1_RST_MASK | readb(hdl->baddr + BSCD_P_IF_CMD_1);
		writeb(value, hdl->baddr + BSCD_P_IF_CMD_1);
	} else {
		value = ~IFCMD1_RST_MASK & readb(hdl->baddr + BSCD_P_IF_CMD_1);
		writeb(value, hdl->baddr + BSCD_P_IF_CMD_1);
	}
out:
	return err;
}

/* This sets the vcc level */
int chnl_set_vcc_level(struct p_chnl_hdl *hdl, enum bscd_vcc_level in_vcc_level)
{
	int err = BERR_SUCCESS;

	WARN_ON(!hdl);
	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	switch (in_vcc_level) {
	case vcc_level_3V:
		break;
	case vcc_level_5V:
		/*
		 * For NXP8024:
		 * GPB13 for SC0 (select 5V/3V); GPB15 for SC1 (select 5V/3V)
		 */

		/*
		 * Set GPB13 (Group1 pin 13) high for 5V- fye: removed temporary
		 * reg_gpio_iotr_set_pin_type(HW_GPIO0_PIN_MAX+13,
		 * GPIO_PIN_TYPE_OUTPUT);
		 */
		break;
	case vcc_level_18V:
		break;
	default:
		err = BSCD_STATUS_FAILED;
		pr_err("Do not support VCC Level switch:0x%x\n", in_vcc_level);
		goto out;
	}
out:
	return err;
}

int chnl_rst_card(struct p_chnl_hdl *hdl,
		  enum bscd_rst_card_action in_icc_action)
{
	int err = BERR_SUCCESS;

	WARN_ON(!hdl);

	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	switch (in_icc_action) {
	case rst_card_act_no_aciton:
		bscd_chnl_p_activating(hdl);
		break;
	case rst_card_act_rcv_decode:
		err = bscd_chnl_p_activating(hdl);
		if (err != BERR_SUCCESS) {
			err = BERR_TRACE(BSCD_STATUS_DEACTIVATE);
			goto out;
		}
		pr_devel("Activating OK\n");
		err = chnp_p_rcv_and_decode(hdl);
		if (err != BERR_SUCCESS) {
			err = BERR_TRACE(err);
			goto out;
		}
		pr_devel("ReceiveAndDecode OK\n");
		break;
	default:
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	/* Clear internal states */
	memset(&(hdl->sc_tpdu_t1), 0, sizeof(struct bscd_tpdu_t1));

out:
	pr_devel("Leave ResetCard erroCode = 0x%x\n", err);
	return err;
}

int bscd_chnl_get_atr(struct p_chnl_hdl *hdl, unsigned char *rcv_data,
			unsigned long *rcv_bytes)
{
	int err = BERR_SUCCESS;
	unsigned char bindex;

	for (bindex = 0; bindex < hdl->rxlen; bindex++)
		*rcv_data++ = hdl->rxbuf[bindex];

	*rcv_bytes = hdl->rxlen;
	return err;
}

int bscd_chnl_pps(struct p_chnl_hdl *hdl)
{
	int err = BERR_SUCCESS;
	int ret = 0;
	unsigned char txbuf[4]; /* size of minimum PPS packet */
	unsigned char rxbuf[4] = {0}; /* size of minimum PPS packet */
	unsigned int len = 0;
	/* ACOS5 32G card workaroud: skip PPS */
	char acos5_32g[] = {0x3B, 0xBE, 0x18, 0x0, 0x0, 0x41, 0x5, 0x10, 0x0,
			    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x90, 0x0};

	WARN_ON(!hdl);

	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	if (hdl->is_pps_needed == false) {
		pr_alert("sc: PPS not needed\n");
		hdl->chnl_status.pps_done = true;
		goto out;
	}

	/* ACOS5 32G card workaroud: skip PPS */
	if (hdl->rxlen == sizeof(acos5_32g)) {
		ret = memcmp(hdl->rxbuf, acos5_32g, sizeof(acos5_32g));
		if (ret == 0) {
			pr_alert("This is a ACOS 32G card.\n");
			err = BERR_STATUS_FAKE_FAILED;
			goto out;
		}
	}

	/* assign PPS */
	txbuf[0] = 0xFF; /* PPSS */
	txbuf[1] = 0x10 | (hdl->negotiated_chs.proto_type ==
			   async_proto_e1 ? 1 : 0); /* PPS0:PPS1 exist, plus T*/
	txbuf[2] = (hdl->negotiated_chs.ffactor << 4) |
		   (hdl->negotiated_chs.dfactor & 0x0f); /* PPS1 */
	txbuf[3] = (txbuf[0] ^ txbuf[1] ^ txbuf[2]); /* PCK */

	/* send and receive PPS */
	err = chnl_xmit(hdl, txbuf, sizeof(txbuf));
	if (err == BERR_SUCCESS)
		err = chnl_rcv(hdl, rxbuf, (unsigned long *)&len,
			       sizeof(rxbuf));
	hdl->chnl_status.pps_done = true;
	if (err) {
		char set_cos43[] = {0x3B, 0x9F, 0x94, 0x40, 0x1E, 0x00, 0x67,
				0x11, 0x43, 0x46, 0x49, 0x53, 0x45, 0x10,
				0x52, 0x66, 0xFF, 0x81, 0x90, 0x00};

		pr_alert("ct: pps tx/rx error\n");
		if (hdl->rxlen == sizeof(set_cos43)) {
			ret = memcmp(hdl->rxbuf, set_cos43, sizeof(set_cos43));
			if (ret == 0) {
				pr_alert("This is a SetCos4.3 card.\n");
				err = BERR_SUCCESS;
				goto out;
			}
		}
		err = BSCD_STATUS_FAILED;
		goto out;
	} else {
		/* check if response is same as request */
		if (memcmp(txbuf, rxbuf, sizeof(rxbuf))) {
			/*
			 * We treat all non-matching cases as failure, althoug
			 * the spec says we need to look byte by byte.
			 * From spec:
			 * PPSS should be echoed
			 * PPS0 b1-b4 should be echoed
			 * PPS0 b5 should be either echoed or not
			 * When echoed, PPS1 must echoed;
			 * When not echoed, PPS1(e.g. F/D) should not be used,
			 * we should regard it as PPS fail so our parameter is
			 * not changed. So in all cases, the response must
			 * exactly same as request.
			 */

			pr_alert("ct: PPS response error\n");
			err = BSCD_STATUS_FAILED;
			goto out;
		}
	}
out:
	pr_devel("%s%x\n", "Leave bscd_chnl_pps erroCode = 0x", err);
	return err;
}

int chnl_get_status(struct p_chnl_hdl *hdl, struct bscd_status *outp_status)
{
	int err = BERR_SUCCESS;

	WARN_ON(!hdl);

	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	*outp_status = hdl->chnl_status;
out:
	return err;
}

int chnl_xmit(struct p_chnl_hdl *hdl, unsigned char *xmit_data,
			unsigned long xmit_bytes)
{
	int err = BERR_SUCCESS;

	WARN_ON(!hdl);

	err =  chnl_ena_intr(hdl);
	if (err != BERR_SUCCESS)
		goto out;

	if (hdl->cur_chs.sc_std == std_irdeto) {
		err = BSCD_STATUS_FAILED;
		goto out;
	} else if ((hdl->cur_chs.proto_type == async_proto_e0_sync) ||
		   (hdl->cur_chs.proto_type == async_proto_e1_sync)) {
		err = BSCD_STATUS_FAILED;
		goto out;
	} else {
		return chnl_p_t0t1_transmit(hdl, xmit_data, xmit_bytes);
	}
out:
	return err;
}

int chnl_rcv(struct p_chnl_hdl *hdl, unsigned char *rcv_data,
	     unsigned long *rcv_bytes, unsigned long max_read_bytes)
{
	int err = BSCD_STATUS_READ_SUCCESS;
#ifndef BSCD_DSS_ICAM
	struct bscd_timer timer = {timer_wait, {wtm_work}, false, false};
#endif

	WARN_ON(!hdl);

	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	err = chnl_ena_intr(hdl);
	if (err != BERR_SUCCESS)
		goto out;
	*rcv_bytes = 0;
	if ((hdl->cur_chs.proto_type == async_proto_e0) ||
	    (hdl->cur_chs.proto_type == async_proto_e14_irdeto)) {

		err = chnl_p_t0_read_data(hdl, rcv_data, rcv_bytes,
					  max_read_bytes);
		if (err != BERR_SUCCESS)
			goto out;

		/*
		 * The Work Wait Timer is enabled in chnl_p_t0t1_transmit.
		 * We cannot disable it in chnl_p_t0_read_data since
		 * chnl_p_t0_read_data is also used by reading ATR, which is one
		 * byte at a time
		 */

#ifndef BSCD_DSS_ICAM
		/*
		 * BSYT leave this WWT enabled. We only disable WWT in transmit.
		 * I assume all standards, other than EMV, will read all the
		 * bytes in chnl_p_t0_read_data, therefore we couold safely
		 * disable the WWT here.EMV only read 1 bytes at a time,
		 * therefore we have to disable WWT in the application.
		 *
		 * fye: this channel receive can be called multiple times
		 * without tx in between, in the case of rx 0x60 only. this
		 * will result the WWT being disabled here for the 2nd rx and
		 * so on
		 */
		if ((hdl->cur_chs.sc_std != std_emv1996) &&
		    (hdl->cur_chs.sc_std != std_emv2000)) {
			err = chnl_ena_dis_timer_isr(hdl, &timer);
			if (err != BERR_SUCCESS) {
				err = BSCD_STATUS_READ_FAILED;
				goto out;
			}
		}
#endif
	} else if (hdl->cur_chs.proto_type == async_proto_e1) {
		err =  chnl_p_t1_read_data(hdl, rcv_data,
					   rcv_bytes, max_read_bytes);
		if (err != BERR_SUCCESS)
			goto out;
	} /* async_proto_e1 */

	if (*rcv_bytes > 0) {
		/* Ignore the Readtimeout error returned by SmartCardByteRead */
		/* printk(KERN_ERR "success in SmartCardReadCmd\n"); */
	} else {
		pr_devel("No Response detected.deactivating,scerr:%02x\n", err);
		err = BSCD_STATUS_FAILED;
		goto out;
	}
out:
	return err;
}

int chnl_rcv_atr(struct p_chnl_hdl *hdl, unsigned char *rcv_data,
		 unsigned long *rcv_bytes, unsigned long max_read_bytes)
{
	int err = BSCD_STATUS_READ_SUCCESS;

	WARN_ON(!hdl);

	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	*rcv_bytes = 0;
	if ((hdl->cur_chs.proto_type == async_proto_e0_sync) ||
	    (hdl->cur_chs.proto_type == async_proto_e1_sync)) {
		err = BSCD_STATUS_FAILED;
		goto out;
	} else {
		err = chnl_p_t0_read_data(hdl, rcv_data, rcv_bytes,
					  max_read_bytes);
		if (err != BERR_SUCCESS)
			goto out;
	}
	if (*rcv_bytes > 0) {
		/*
		 * For T=0, we depend on timeout to
		 * identify that there is no more byte to be received
		 */

		/* Ignore the Readtimeout error returned by SmartCardByteRead */
	} else {
		pr_devel("No Response detected.deactivating,s cerr:%x\n", err);
		err = BSCD_STATUS_FAILED;
		goto out;
	}
out:
	return err;
}

int chnl_config_timer(struct p_chnl_hdl *hdl, struct bscd_timer *inp_timer,
		      struct bscd_timer_value *inp_cnt)
{
	unsigned long flag;
	unsigned int timer_cmd, timer_cmp_val;
	int err = BERR_SUCCESS;
	int val = 0;

	WARN_ON(!hdl);
	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	if (inp_timer->timer_type == timer_gpt) {
		/* add check */
		if (inp_cnt->value >> 16) {
			pr_alert("Warning: SC set GT timer more than 16 bits,");
			pr_alert("change it back to 0xFFFF\n");
			inp_cnt->value = 0xFFFF;
		}

		/* Always disbale timer first before we change timer_cmd */
		timer_cmd = readb(hdl->baddr + BSCD_P_TIMER_CMD);
		timer_cmd &= (~TIMERCMD_TIMER_ENA_MASK);
		writeb(timer_cmd, hdl->baddr + BSCD_P_TIMER_CMD);
		val = readb(hdl->baddr + BSCD_P_TIMER_CMD);
		pr_devel("before we change timer_cmd:");
		pr_devel("BSCD_P_TIMER_CMD = 0x%x\n", val);
		spin_lock_irqsave(&hdl->lock, flag);
		hdl->intr_status1  &= ~INTSTAT1_TIMER_INTR_MASK;
		spin_unlock_irqrestore(&hdl->lock, flag);

		/* Set timer_cmp registers */

		timer_cmp_val = ((inp_cnt->value & 0xFF00) >> 8);
		writeb(timer_cmp_val, hdl->baddr + BSCD_P_TIMER_CMP_2);

		timer_cmp_val = inp_cnt->value & 0x00FF;
		writeb(timer_cmp_val, hdl->baddr + BSCD_P_TIMER_CMP_1);

		/* Set the timer unit and mode */
		if (inp_cnt->unit == unit_clk) {
			timer_cmd |= TIMERCMD_TIMER_SRC_MASK;
		} else if (inp_cnt->unit  == unit_etu) {
			timer_cmd &= (~TIMERCMD_TIMER_SRC_MASK);
		} else {
			err = BSCD_STATUS_FAILED;
			pr_err("sci: unknown timer unit :%d", inp_cnt->unit);
			goto out;
		}

		if (inp_timer->timer_mode.gpt_timer_mode ==
					gpt_timer_mode_nxt_start_bit)
			timer_cmd |= TIMERCMD_TIMER_MODE_MASK;
		else
			timer_cmd &= (~TIMERCMD_TIMER_MODE_MASK);

		/*Check if we need to raise an interrupt when the time expires*/
		if (inp_timer->is_timer_intr_ena == true) {
			err = chnl_ena_int_cbisr(hdl, int_timer,
						 chnl_p_timer_cbisr);
			if (err != BERR_SUCCESS) {
				pr_err("sci: failed to raise timer intr\n");
				goto out;
			}
		} else {
			err = chnl_dis_intr_cbisr(hdl, int_timer);
			if (err != BERR_SUCCESS) {
				pr_err("SCI: failed to dis timer intr");
				goto out;
			}
		}

		if (inp_timer->is_timer_ena == true)
			timer_cmd |= TIMERCMD_TIMER_ENA_MASK;
		else
			timer_cmd &= ~TIMERCMD_TIMER_ENA_MASK;

	} else {  /* timer_wait */

		/* add check */
		if (inp_cnt->value >> 24)
			pr_alert("Err: SC set GT timer more than 16 bits\n");

		/* Always disable timer first before we change timer_cmd */
		timer_cmd = readb(hdl->baddr + BSCD_P_TIMER_CMD);
		timer_cmd &= (~TIMERCMD_WAIT_ENA_MASK);
		writeb(timer_cmd, hdl->baddr + BSCD_P_TIMER_CMD);

		spin_lock_irqsave(&hdl->lock, flag);
		hdl->intr_status2  &= ~INTSTAT2_WAIT_INTR_MASK;
		spin_unlock_irqrestore(&hdl->lock, flag);

		/* Set sc_wait registers */
		timer_cmp_val = ((inp_cnt->value  & 0xFF0000) >> 16);
		writeb(timer_cmp_val, hdl->baddr + BSCD_P_WAIT_3);

		timer_cmp_val = ((inp_cnt->value & 0x00FF00) >> 8);
		writeb(timer_cmp_val, hdl->baddr + BSCD_P_WAIT_2);

		timer_cmp_val = (inp_cnt->value & 0x0000FF);
		writeb(timer_cmp_val, hdl->baddr + BSCD_P_WAIT_1);

		/*
		 * Check if we need to invoke an interrupt when the time
		 * expires
		 */
		if (inp_timer->is_timer_intr_ena == true) {
			err = chnl_ena_int_cbisr(hdl, int_wait,
						 chnl_p_wait_cbisr);
			if (err != BERR_SUCCESS) {
				pr_err("SCI: failed to ena wait timer intr\n");
				goto out;
			}
		} else {
			err = chnl_dis_intr_cbisr(hdl, int_wait);
			if (err != BERR_SUCCESS) {
				pr_err("SCI: failed to dis wait timer intr\n");
				goto out;
			}
		}
		if (inp_timer->is_timer_ena == true) {
			/* Set the wait mode */
			if (inp_timer->timer_type == timer_wait) {
				if (inp_timer->timer_mode.wait_time_mode ==
								wtm_blk)
					timer_cmd |= TIMERCMD_WAIT_MODE_MASK;
				else
					timer_cmd &= ~TIMERCMD_WAIT_MODE_MASK;
				timer_cmd |= TIMERCMD_WAIT_ENA_MASK;
			}

		} else {
			timer_cmd &= ~TIMERCMD_WAIT_ENA_MASK;
		}
	}
	writeb(timer_cmd, hdl->baddr + BSCD_P_TIMER_CMD);
	udelay(150);
	pr_devel("%s%x\n", "chnl_config_timer: BSCD_P_TIMER_CMD = 0x",
		 timer_cmd);
out:
	return err;
}

bool chnl_is_timer_ena(struct p_chnl_hdl *hdl, enum bscd_timer_type timer_type)
{
	int err = BERR_SUCCESS;
	unsigned int timer_cmd;

	WARN_ON(!hdl);

	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	timer_cmd = readb(hdl->baddr + BSCD_P_TIMER_CMD);
	if (timer_type == timer_gpt) {
		return timer_cmd & TIMERCMD_TIMER_ENA_MASK;
	} else {  /* timer_wait */
		return timer_cmd & TIMERCMD_WAIT_ENA_MASK;
	}
out:
	return 0;
}

int chnl_ena_dis_timer_isr(struct p_chnl_hdl *hdl, struct bscd_timer *inp_timer)
{
	int err = BERR_SUCCESS;
	unsigned int timer_cmd;

	WARN_ON(!hdl);

	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	timer_cmd = readb(hdl->baddr + BSCD_P_TIMER_CMD);
	if (inp_timer->timer_type == timer_gpt) {
		hdl->intr_status1 &= ~INTSTAT1_TIMER_INTR_MASK;
		if (inp_timer->is_timer_intr_ena == true) {
			err = chnl_ena_int_cbisr(hdl, int_timer,
						 chnl_p_timer_cbisr);
			if (err != BERR_SUCCESS)
				goto out;
		} else {
			err = chnl_dis_intr_cbisr(hdl, int_timer);
			if (err != BERR_SUCCESS)
				goto out;
		}
		if (inp_timer->is_timer_ena == true) {
			timer_cmd |= TIMERCMD_TIMER_ENA_MASK;
		} else { /* inp_timer->is_timer_ena == false && timer_gpt */
			timer_cmd &= ~TIMERCMD_TIMER_ENA_MASK;
		}
	} else {  /* timer_wait */
		hdl->intr_status1 &= ~INTSTAT1_TIMER_INTR_MASK;
		if (inp_timer->is_timer_intr_ena == true) {
			err = chnl_ena_int_cbisr(hdl, int_wait,
						 chnl_p_wait_cbisr);
			if (err != BERR_SUCCESS)
				goto out;
		} else {
			err = chnl_dis_intr_cbisr(hdl, int_wait);
			if (err != BERR_SUCCESS)
				goto out;
		}
		if (inp_timer->is_timer_ena == true) {
			/* Set the wait mode */
			if (inp_timer->timer_type == timer_wait) {
				if (inp_timer->timer_mode.wait_time_mode ==
								wtm_blk) {
					timer_cmd |= TIMERCMD_WAIT_MODE_MASK;
				} else { /* wtm_work */
					timer_cmd &= ~TIMERCMD_WAIT_MODE_MASK;
				}
				timer_cmd |= TIMERCMD_WAIT_ENA_MASK;
			}
		} else { /* inp_timer->is_timer_ena == false && timer_wait */
			timer_cmd &= ~TIMERCMD_WAIT_ENA_MASK;
		}
	}
	writeb(timer_cmd, hdl->baddr + BSCD_P_TIMER_CMD);
	pr_devel("%s%08x\n", "chnl_ena_dis_timer_isr: Timer cmd = 0x",
		 timer_cmd);
out:
	return err;
}

int chnl_ena_int_cbisr(struct p_chnl_hdl *hdl, enum bscd_int_type in_int_type,
		       isrcb  in_callback)
{
	unsigned int  val, old_val;
	unsigned int  reg = BSCD_P_INTR_EN_1, i;
	int err = BERR_SUCCESS;

	if ((in_int_type == int_tparity) ||
	    (in_int_type == int_timer) ||
	    (in_int_type == int_card_insert) ||
	    (in_int_type == int_card_remove) ||
	    (in_int_type == int_bg) ||
	    (in_int_type == int_tdone) ||
	    (in_int_type == int_retry) ||
	    (in_int_type == int_tempty) ||
	    (in_int_type == int_event1)) {
		reg = BSCD_P_INTR_EN_1;
	} else if ((in_int_type == int_rparity) ||
		   (in_int_type == int_atr) ||
		   (in_int_type == int_cw) ||
		   (in_int_type == int_rlen) ||
		   (in_int_type == int_wait) ||
		   (in_int_type == int_rcv) ||
		   (in_int_type == int_rready) ||
		   (in_int_type == int_event2)) {
		reg = BSCD_P_INTR_EN_2;
	} else if (in_int_type == int_edc) {
		reg = BSCD_P_PROTO_CMD;
	} else {
		pr_err("Interrupt not supported, int_type = %d\n", in_int_type);
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	val = readb(hdl->baddr + reg);
	old_val = val;

	switch (in_int_type) {
	case int_tparity:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.tparity[i] == NULL) {
				hdl->callback.tparity[i] = in_callback;
				break;
			} else if ((hdl->callback.tparity[i] != NULL) &&
				   (hdl->callback.tparity[i] == in_callback)) {
				break;
			}
		}
		val |=  INTSTAT1_TPAR_INTR_MASK;
		break;
	case int_timer:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.timer[i] == NULL) {
				hdl->callback.timer[i] = in_callback;
				break;
			} else if ((hdl->callback.timer[i] != NULL) &&
				   (hdl->callback.timer[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT1_TIMER_INTR_MASK;
		break;
	case int_card_insert:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.card_insert[i] == NULL) {
				hdl->callback.card_insert[i] = in_callback;
				pr_devel("new int_card_insert  callback\n");
				break;
			} else if ((hdl->callback.card_insert[i] != NULL) &&
				(hdl->callback.card_insert[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT1_PRES_INTR_MASK;
		break;
	case int_card_remove:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.card_rm[i] == NULL) {
				hdl->callback.card_rm[i] = in_callback;
				pr_devel("new int_card_remove  callback\n");
				break;
			} else if ((hdl->callback.card_rm[i] != NULL) &&
				(hdl->callback.card_rm[i] == in_callback)) {
				pr_devel("int_card_remove same callback\n");
				break;
			}
		}
		val |= INTSTAT1_PRES_INTR_MASK;
		break;
	case int_bg:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.bgt[i] == NULL) {
				hdl->callback.bgt[i] = in_callback;
				break;
			} else if ((hdl->callback.bgt[i] != NULL) &&
				(hdl->callback.bgt[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT1_BGT_INTR_MASK;
		break;
	case int_tdone:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.tdone[i] == NULL) {
				hdl->callback.tdone[i] = in_callback;
				break;
			} else if ((hdl->callback.tdone[i] != NULL) &&
				(hdl->callback.tdone[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT1_TDONE_INTR_MASK;
		break;
	case int_retry:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.retry[i] == NULL) {
				hdl->callback.retry[i] = in_callback;
				break;
			} else if ((hdl->callback.retry[i] != NULL) &&
				(hdl->callback.retry[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT1_RETRY_INTR_MASK;
		break;
	case int_tempty:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.tempty[i] == NULL) {
				hdl->callback.tempty[i] = in_callback;
				break;
			} else if ((hdl->callback.tempty[i] != NULL) &&
				(hdl->callback.tempty[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT1_TEMP_INTR_MASK;
		break;
	case int_rparity:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.rparity[i] == NULL) {
				hdl->callback.rparity[i] = in_callback;
				break;
			} else if ((hdl->callback.rparity[i] != NULL) &&
				(hdl->callback.rparity[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT2_RPAR_INTR_MASK;
		break;
	case int_atr:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.atr[i] == NULL) {
				hdl->callback.atr[i] = in_callback;
				break;
			} else if ((hdl->callback.atr[i] != NULL) &&
				(hdl->callback.atr[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT2_ATR_INTR_MASK;
		break;
	case int_cw:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.cwt[i] == NULL) {
				hdl->callback.cwt[i] = in_callback;
				break;
			} else if ((hdl->callback.cwt[i] != NULL) &&
				(hdl->callback.cwt[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT2_CWT_INTR_MASK;
		break;
	case int_rlen:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.rlen[i] == NULL) {
				hdl->callback.rlen[i] = in_callback;
				break;
			} else if ((hdl->callback.rlen[i] != NULL) &&
				(hdl->callback.rlen[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT2_RLEN_INTR_MASK;
		break;
	case int_wait:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.wait[i] == NULL) {
				hdl->callback.wait[i] = in_callback;
				break;
			} else if ((hdl->callback.wait[i] != NULL) &&
				(hdl->callback.wait[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT2_WAIT_INTR_MASK;
		break;
	case int_rcv:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.rcv[i] == NULL) {
				hdl->callback.rcv[i] = in_callback;
				break;
			} else if ((hdl->callback.rcv[i] != NULL) &&
				(hdl->callback.rcv[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT2_RCV_INTR_MASK;
		break;
	case int_rready:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.rrdy[i] == NULL) {
				hdl->callback.rrdy[i] = in_callback;
				break;
			} else if ((hdl->callback.rrdy[i] != NULL) &&
				(hdl->callback.rrdy[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT2_RRDY_INTR_MASK;
		break;
	case int_edc:
		if (hdl->cur_chs.proto_type == async_proto_e0) {
			for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (hdl->callback.edc[i] == NULL) {
					hdl->callback.edc[i] = in_callback;
					break;
				} else if ((hdl->callback.edc[i] != NULL) &&
					(hdl->callback.edc[i] == in_callback)) {
					break;
				}
			}
			val |= PROTOCMD_EDC_ENA_MASK;
		}
		break;
	case int_event1:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.event1[i] == NULL) {
				hdl->callback.event1[i] = in_callback;
				break;
			} else if ((hdl->callback.event1[i] != NULL) &&
				(hdl->callback.event1[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT1_EV1_INTR_MASK;
		break;
	case int_event2:
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.event2[i] == NULL) {
				hdl->callback.event2[i] = in_callback;
				break;
			} else if ((hdl->callback.event2[i] != NULL) &&
				(hdl->callback.event2[i] == in_callback)) {
				break;
			}
		}
		val |= INTSTAT2_EV2_INTR_MASK;
		break;
	default:
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	if (val != old_val)
		writeb(val, hdl->baddr + reg);

out:
	return err;
}

int chnl_dis_intr_cbisr(struct p_chnl_hdl *hdl, enum bscd_int_type in_int_type)
{
	unsigned int val;
	unsigned int  reg = BSCD_P_INTR_EN_1;
	int err = BERR_SUCCESS;

	pr_devel("%s: in_int_type=%d\n", __func__, in_int_type);
	if ((in_int_type == int_tparity) ||
	    (in_int_type == int_timer) ||
	    (in_int_type == int_card_insert) ||
	    (in_int_type == int_card_remove) ||
	    (in_int_type == int_bg) ||
	    (in_int_type == int_tdone) ||
	    (in_int_type == int_retry) ||
	    (in_int_type == int_tempty) ||
	    (in_int_type == int_event1)) {
		reg = BSCD_P_INTR_EN_1;
	} else if ((in_int_type == int_rparity) ||
		   (in_int_type == int_atr) ||
		   (in_int_type == int_cw) ||
		   (in_int_type == int_rlen) ||
		   (in_int_type == int_wait) ||
		   (in_int_type == int_rcv) ||
		   (in_int_type == int_rready) ||
		   (in_int_type == int_event2)) {
		reg = BSCD_P_INTR_EN_2;
	} else if (in_int_type == int_edc) {
		reg = BSCD_P_PROTO_CMD;
	} else {
		pr_devel("Interrupt not supported\n");
		err = BSCD_STATUS_FAILED;
		goto out;
	}

	val = readb(hdl->baddr + reg);
	pr_devel("val = 0x%x", val);
	switch (in_int_type) {
	case int_tparity:
		val &= ~INTSTAT1_TPAR_INTR_MASK;
		break;
	case int_timer:
		val &= ~INTSTAT1_TIMER_INTR_MASK;
		break;
	case int_card_insert:
		val &= ~INTSTAT1_PRES_INTR_MASK;
		break;
	case int_card_remove:
		val &= ~INTSTAT1_PRES_INTR_MASK;
		break;
	case int_bg:
		val &= ~INTSTAT1_BGT_INTR_MASK;
		break;
	case int_tdone:
		val &= ~INTSTAT1_TDONE_INTR_MASK;
		break;
	case int_retry:
		val &= ~INTSTAT1_RETRY_INTR_MASK;
		break;
	case int_tempty:
		val &= ~INTSTAT1_TEMP_INTR_MASK;
		break;
	case int_rparity:
		val &= ~INTSTAT2_RPAR_INTR_MASK;
		break;
	case int_atr:
		val &= ~INTSTAT2_ATR_INTR_MASK;
		break;
	case int_cw:
		val &= ~INTSTAT2_CWT_INTR_MASK;
		break;
	case int_rlen:
		val &= ~INTSTAT2_RLEN_INTR_MASK;
		break;
	case int_wait:
		val &= ~INTSTAT2_WAIT_INTR_MASK;
		break;
	case int_rcv:
		val &= ~INTSTAT2_RCV_INTR_MASK;
		break;
	case int_rready:
		val &= ~INTSTAT2_RRDY_INTR_MASK;
		break;
	case int_edc:
		val &= ~PROTOCMD_EDC_ENA_MASK;
		break;
	case int_event1:
		val &= ~INTSTAT1_EV1_INTR_MASK;
		break;
	case int_event2:
		val &=  ~INTSTAT2_EV2_INTR_MASK;
		break;
	default:
		err = BSCD_STATUS_FAILED;
		goto out;
	}
	writeb(val, hdl->baddr + reg);
out:
	return err;
}

int chnl_ena_intr(struct p_chnl_hdl *hdl)
{
	unsigned long flag;
	int err = BERR_SUCCESS;

	WARN_ON(!hdl);
	spin_lock_irqsave(&hdl->lock, flag);
	err = chnl_p_ena_intr_isr(hdl);
	if (err != BERR_SUCCESS) {
		err = BERR_TRACE(err);
		goto out;
	}
out:
	spin_unlock_irqrestore(&hdl->lock, flag);
	return err;
}

int chnl_rst_blk_wait_timer(struct p_chnl_hdl *hdl)
{
	int err = BERR_SUCCESS;
	struct bscd_timer timer = {timer_wait, {gpt_timer_mode_immediate},
				   false, true};
	struct bscd_timer_value  time_val = {BSCD_DEFAULT_BLOCK_WAITING_TIME,
					     unit_etu};
	WARN_ON(!hdl);

	/* Need this for MetroWerks */
	timer.timer_type = timer_wait;
	timer.timer_mode.wait_time_mode = wtm_blk;

	time_val.value = hdl->cur_chs.blk_wait_time.value;
	err = chnl_config_timer(hdl, &timer, &time_val);
	if (err != BERR_SUCCESS)
		goto out;

	hdl->cur_chs.blk_wait_time_ext.value = 0;
out:
	return err;
}

/* Get BWT (Note: Not extension) */
int chnl_get_blk_wait_time(struct p_chnl_hdl *hdl,
			   unsigned int *pin_blk_wait_time_in_etu)
{
	WARN_ON(!hdl);
	*pin_blk_wait_time_in_etu = hdl->cur_chs.blk_wait_time.value;

	return BERR_SUCCESS;
}

/* Set BWT Extension */
int chnl_set_blk_wait_time_ext(struct p_chnl_hdl *hdl,
			       unsigned int blk_wait_time_ext_in_etu)
{
	WARN_ON(!hdl);
	hdl->cur_chs.blk_wait_time_ext.value = blk_wait_time_ext_in_etu;
	return BERR_SUCCESS;
}

/* Set BWI */
int chnl_set_blk_wait_time_integer(struct bscd_chnl_settings *chs,
				   unsigned char bwi_val)
{
	int err = BERR_SUCCESS;
	unsigned char baud_div, clk_div;
	unsigned int  blk_wait_time;

	/*
	 * The block waiting time is encoded as described in ISO 7816-3,
	 * repeated here in the following equation:
	 * BWT = [11 + 2 bwi x 960 x 372 x D/F] etu
	 * e.g If bwi = 4 and F/D = 372 then BWT = 15,371 etu.
	 * The minimum and maximum BWT are ~186 and 15,728,651 etu.
	 */
	chs->prescale = p_get_prescale(chs->dfactor, chs->ffactor) *
				       chs->external_clk_div +
				       (chs->external_clk_div - 1);

	baud_div = p_get_baud_div(chs->dfactor, chs->ffactor);
	clk_div = p_get_clk_div(chs->dfactor, chs->ffactor);

	if (bwi_val == 0x00)
		blk_wait_time = 960 * 372 * clk_div / (chs->prescale+1) /
				baud_div + 11;
	else
		blk_wait_time = (2 << (bwi_val-1)) * 960 *  372 * clk_div /
				(chs->prescale + 1) / baud_div + 11;


	/* Change timer to equal calculated BWT */
	chs->blk_wait_time.value = blk_wait_time;
	chs->blk_wait_time.unit = unit_etu;

	return err;
}
