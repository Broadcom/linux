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

#ifndef BSCD_PRIV_H__
#define BSCD_PRIV_H__
#include <linux/io.h>
#include <linux/spinlock.h>
#include "bscd.h"
#include "bscd_tpdu_t1.h"

/*
 * Smart card D and F Table according to SC_PRESCALE Table, where F is Clock
 * Rate Conversion Factor and D is the Baud Rate Adjustment Factor.
 * This table contains the recommended set of values for the SC_PRESCALE and
 * SC_CLK_CMD based on the ATR TA1 byte. The resulting clock rate and baud
 * rate will comply to ISO 7816-3.
 */

struct p_df_sc_struct {
	unsigned char sc_clk_div;     /* clk divider in SC_CLK_CMD register */
	unsigned char sc_prescale;    /* prescale in SC_PRESCALE register */
	unsigned char sc_baud_div;/* baud rate divider in SC_CLK_CMD register */
	unsigned char sc_etuclk_div;/* etu clk divider in SC_CLK_CMD register */
};

/* Smart Card Module magic number used to check if opaque handle is corrupt */
#define BSCD_P_HANDLE_MAGIC_NUMBER           0xdeadbeef

/* Smart Card Channel magic number used to check if opaque handle is corrupt */
#define BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER   0xfeedbabe

/* Smart Card Private Data Structures */

/*
 * Summary:
 * Smart card event handle.
 *
 * Description:
 * Upon receiving an interrupt, ISR shall signal this event to wake up the
 * thread/task that is waiting for this event.
 *
 * See Also:
 * bscd_chnl_settings, open()
 */
struct p_wait_event {
	struct bscd_event_obj *card_wait;   /* card detection */
	struct bscd_event_obj *tdone_wait;  /* transmit done */
	struct bscd_event_obj *rcv_wait;    /* receive at least one byte */
	struct bscd_event_obj *atr_start;   /* receive start bit of the ATR */
	struct bscd_event_obj *timer_wait;  /* timer expired */
	struct bscd_event_obj *event1_wait; /* timer expired */
	struct bscd_event_obj *event2_wait; /* timer expired */
	struct bscd_event_obj *bh_pres;     /*
					     * card insert/remove bottom half
					     * finished or not
					     */
};

/*
 * Summary:
 * Structure that defines Smart card Baud Rate and Clock divisor.
 *
 * Description:
 * Smart card D and F Table according to SC_PRESCALE Table, where F is Clock
 * Rate Conversion Factor and D is the Baud Rate Adjustment Factor.
 * This table contains the recommended set of values for the SC_PRESCALE and
 * SC_CLK_CMD based  on the ATR TA1 byte. The resulting clock rate and baud
 * rate will comply to ISO 7816-3.
 *
 * See Also:
 * openChannel()
 */
struct p_df_struct {
	unsigned char clk_div;      /* clk divider in SC_CLK_CMD register */
	unsigned char prescale;    /* prescale in SC_PRESCALE register */
	unsigned char baud_div;   /* baud rate divider in SC_CLK_CMD register */
	unsigned char etuclk_div;   /* etu clk divider in SC_CLK_CMD register */
};

/*
 * Summary:
 * Structure that defines Smart card interrupt calback function.
 *
 * Description:
 * Structure that defines Smart card calback function.
 */
struct p_isrcb {
	isrcb tparity[BSCD_MAX_NUM_CALLBACK_FUNC];  /* for Transmit Parity */
	isrcb timer[BSCD_MAX_NUM_CALLBACK_FUNC]; /*for eneral Purpose Timer*/
	isrcb card_insert[BSCD_MAX_NUM_CALLBACK_FUNC];/* for Card Insertion */
	isrcb card_rm[BSCD_MAX_NUM_CALLBACK_FUNC];/* for Card Removal */
	isrcb bgt[BSCD_MAX_NUM_CALLBACK_FUNC];    /* for Block Guard Time */
	isrcb tdone[BSCD_MAX_NUM_CALLBACK_FUNC];  /* for Transmit Done */
	isrcb retry[BSCD_MAX_NUM_CALLBACK_FUNC]; /*xmit/rcv retry Overflow*/
	isrcb tempty[BSCD_MAX_NUM_CALLBACK_FUNC]; /* for Transmit Empty */
	isrcb rparity[BSCD_MAX_NUM_CALLBACK_FUNC];/* for Receive Parity */
	isrcb atr[BSCD_MAX_NUM_CALLBACK_FUNC];    /* for ATR Start */
	isrcb cwt[BSCD_MAX_NUM_CALLBACK_FUNC];    /* for CWT */
	isrcb rlen[BSCD_MAX_NUM_CALLBACK_FUNC]; /* for Receive Length Error */
	isrcb wait[BSCD_MAX_NUM_CALLBACK_FUNC];/*for Blk/Work Waiting Time*/
	isrcb rcv[BSCD_MAX_NUM_CALLBACK_FUNC];    /* for Receive Character */
	isrcb rrdy[BSCD_MAX_NUM_CALLBACK_FUNC];   /* for Receive Ready */
	isrcb edc[BSCD_MAX_NUM_CALLBACK_FUNC];    /* for EDC Error */
	isrcb event1[BSCD_MAX_NUM_CALLBACK_FUNC]; /* for Event1 inerrupt */
	isrcb event2[BSCD_MAX_NUM_CALLBACK_FUNC]; /* for Event2 inerrupt */
};

/*
 * Summary:
 * Structure that defines Smart card channel  handle.
 *
 * Description:
 * Structure that defines Smart card channel  handle.
 * This P_Handle is actually BSCD_ChannelHandle, defined in bscd.h as follows:
 * typedef struct p_chnl_hdl     *BSCD_ChannelHandle;
 *
 * See Also:
 * openChannel()
 */
struct p_chnl_hdl {
	unsigned long magic_number; /* BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER */

	struct bscd_p_handle *mod_hdl;   /* Module handle */

	struct bscd_chnl_settings cur_chs;   /* current channel settings */

	/* negotiating channel settings from ATR */
	struct bscd_chnl_settings negotiated_chs;

	unsigned char chnl_number;     /* channel number */

	unsigned int status1;           /* value of SC_STATUS_1 */

	unsigned int status2;           /* value of SC_STATUS_2 */

	unsigned int intr_status1;       /* value of SC_INTR_STATUS_1 */

	unsigned int intr_status2;       /* value of SC_INTR_STATUS_2 */

	struct p_wait_event chnl_wait_event;    /* Wait event */

	/*Need this for EMV ATR to determine if this is warmRst or coldRst */
	enum bscd_rst_type reset_type;

	/* Channel status that returned by chnl_get_status */
	struct bscd_status chnl_status;

	bool is_open;    /* Is channel opened */

	unsigned char rxbuf[BSCD_MAX_RX_SIZE]; /*receiving bytes */

	unsigned long rxlen;                   /* number receiving bytes */

	void __iomem *baddr;

	struct p_isrcb callback;             /* interrupt Callback functions */

	struct bscd_tpdu_t1        sc_tpdu_t1;

#ifdef BSCD_EMV2000_CWT_PLUS_4
	bool is_recv;    /* Is channel opened */
#endif

	bool is_card_removed;    /* Is the Card removed ? */

	bool is_pps_needed;      /* Is PPS needed */

#ifdef BSCD_EMV2000_FIME
	/*FIME EMV2000 for 1732 xy=20/21/22 patch */
	unsigned char parity_error_cnt;
#endif
	void *priv; /*handle private data structure */

	spinlock_t lock;
};
/* End of Smart Card Private Data Structures */


/* Private Function */
unsigned char p_get_clk_div(unsigned char in_dfactor, unsigned char in_ffactor);

unsigned char p_get_etuclk_div(unsigned char in_dfactor,
			       unsigned char in_ffactor);

unsigned char p_get_iso_baud_rate_adjustor(unsigned char in_dfactor);

unsigned int p_get_iso_clkrate_conversion_factor(unsigned char in_ffactor);

unsigned char p_map_scclk_div_to_mask_value(unsigned char in_clk_div);

unsigned char p_get_prescale(unsigned char dfactor, unsigned char ffactor);

unsigned char p_get_baud_div(unsigned char in_dfactor,
			     unsigned char in_ffactor);

int p_fd_adjust(struct p_chnl_hdl *hdl, unsigned char in_ffactor,
		unsigned char in_dfactor);

int p_fd_adjust_without_reg_update(struct bscd_chnl_settings *chnl_settings,
				   unsigned char in_ffactor,
				   unsigned char in_dfactor);

int p_adjust_wwt(struct bscd_chnl_settings *chnl_settings,
		 unsigned char in_work_wait_time_integer);

void chnl_p_card_insert_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

void  chnl_p_card_remove_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

void chnl_p_rcv_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

void chnl_p_atr_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

void chnl_p_wait_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

void chnl_p_timer_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

void chnl_p_retry_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

void chnl_p_timer_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

void chnl_p_rparity_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

void chnl_p_tparity_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

void chnl_p_cwt_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

void chnl_p_bgtcb_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

void chnl_p_rlen_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

void chnl_p_rready_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

void chnl_p_tdone_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
void chnl_p_event1_cbisr(struct p_chnl_hdl *hdl, void *inp_data);
#endif

void chnl_p_event2_cbisr(struct p_chnl_hdl *hdl, void *inp_data);

int chnl_p_wait_for_card_insertion(struct p_chnl_hdl *hdl);

int chnl_p_wait_for_card_remove(struct p_chnl_hdl *hdl);

int chnl_p_wait_for_timer_event(struct p_chnl_hdl *hdl,
				unsigned int  in_chk_card_removal);

int chnl_p_wait_for_atr_start(struct p_chnl_hdl *hdl);

int chnl_p_wait_for_tdone(struct p_chnl_hdl *hdl);

int chnl_p_wait_for_rcv(struct p_chnl_hdl *hdl);

int chnl_p_wait_for_rready(struct p_chnl_hdl *hdl);

int bscd_chnl_p_activating(struct p_chnl_hdl *hdl);

void p_hex_dump(char *inp_title, unsigned char *inp_buf, unsigned int len);

int chnl_p_t0_read_data(struct p_chnl_hdl *hdl, unsigned char *rcv_data,
			unsigned long *rcv_bytes, unsigned long max_read_bytes);

int chnl_p_t1_read_data(struct p_chnl_hdl *hdl, unsigned char *rcv_data,
			unsigned long *rcv_bytes, unsigned long max_read_bytes);

int chnl_p_byte_read(struct p_chnl_hdl *hdl, unsigned char *outp_data);

int chnl_p_ena_intr_isr(struct p_chnl_hdl *hdl);

int chnl_p_intr_handler_isr(struct p_chnl_hdl *chnl_hdl, int in_chnl_number);

void chnl_p_intr_handler_bh(struct p_chnl_hdl *chnl_hdl, int in_chnl_number);

int chnl_p_t0t1_transmit(struct p_chnl_hdl *hdl, unsigned char *xmit_data,
			 unsigned long xmit_bytes);

/*
 * Summary:
 * This function creates the events that associated with the channel.
 *
 * Description:
 * This function creates the events that associated with the channel.
 *
 * Calling Context:
 * The function shall be called in BSCD_ChannelOpen .
 *
 * Performance and Timing:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl  - BSCD_ChannelHandle, a smart card channel handle.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 */
int p_create_wait_event(struct p_chnl_hdl *hdl);

/*
 * Summary:
 * This function destroys the events that associated with the channel.
 *
 * Description:
 * This function destroys the events that associated with the channel.
 *
 * Calling Context:
 * The function shall be called in BSCD_ChannelClose .
 *
 * Performance and Timing:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl  - BSCD_ChannelHandle, a smart card
 * channel handle.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 */
int p_destroy_wait_event(struct p_chnl_hdl *hdl);

/*
 * Summary:
 * This function control the state transition of smart card channel.
 *
 * Description:
 * This function control the state transition of smart card channel.
 *
 * Calling Context:
 *
 *
 * Performance and Timing:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * in_event - BSCD_P_Event, state transition event.
 *
 * Input/Output:
 * hdl  - BSCD_ChannelHandle, a smart card channel handle.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 */
int chnp_p_rcv_and_decode(struct p_chnl_hdl *hdl);

int chnp_p_set_standard(struct p_chnl_hdl *hdl,
			const struct bscd_chnl_settings *inp_settings);

int chnp_p_set_freq(struct p_chnl_hdl *hdl,
		    const struct bscd_chnl_settings *inp_settings);

int chnp_p_set_wait_time(struct p_chnl_hdl *hdl,
			 const struct bscd_chnl_settings *inp_settings);

int chnp_p_set_guard_time(struct p_chnl_hdl *hdl,
			  const struct bscd_chnl_settings *inp_settings);

int chnp_p_set_transmission_time(struct p_chnl_hdl *hdl,
				 const struct bscd_chnl_settings *inp_settings);

int chnp_p_set_edc_parity(struct p_chnl_hdl *hdl,
			  const struct bscd_chnl_settings *inp_settings);

/* End of Private Function */

#endif /* BSCD_PRIV_H__ */
