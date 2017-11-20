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

#ifndef BSCD_DATATYPES_H__
#define BSCD_DATATYPES_H__

#include <linux/wait.h>

#define BERR_TRACE(err) err
struct bscd_event_obj {
	wait_queue_head_t wq;
	int eventset;
};

/* standard error codes */

#define BERR_SUCCESS              0   /* success (always zero) */
#define BERR_TIMEOUT              5   /* reached timeout limit */
#define BERR_NOT_SUPPORTED        8   /* requested feature is not supported */
#define BERR_UNKNOWN              9   /* unknown */
#define BERR_STATUS_FAKE_FAILED   11  /*
				       * special fake failed, treat as success,
				       * added for PPS
				       */
#define BKNI_INFINITE -1

extern int create_event(struct bscd_event_obj **p_event);
extern int wait_for_event(struct bscd_event_obj *event, int timeout_msec);
extern void set_event(struct bscd_event_obj *event);

/*
 * Summary:
 * This enum represents all the Smartcard Interrupt Types.
 *
 * Description:
 * This enumeration defines all the Smartcard Interrupt Types.
 */
enum bscd_int_type {
	int_tparity,     /* Transmit Parity Interrupt. Only for T=0 */
	int_timer,       /* General-purpose Timer Interrupt */
	int_card_insert, /* Card Insertion Interrupt */
	int_card_remove, /* Card Removal Interrupt */
	int_bg,          /* Block Guard Time Interrupt */
	int_tdone,       /* Transmit Done Interrupt */
	int_retry,       /* Transmit or Receive Retry Overflow Interrupt */
	int_tempty,      /* Transmit Empty Interrupt. Only used for debugging */
	int_rparity,     /* Receive Parity Interrupt. Only for T=0 */
	int_atr,         /* ATR Start Interrupt */
	int_cw,          /* CWT Interrupt */
	int_rlen,        /* Receive Length Error Interrupt. Only for T=1 */
	int_wait,        /* Block or Work Waiting Time Interrupt */
	int_rcv,         /* Receive Character Interrupt */
	int_rready,      /* Receive Ready Interrupt. Only for T=1 */
	int_edc,         /* EDC Error. Yet to be implemented. */
	int_event1,      /* Event1 Interrupt.  */
	int_event2       /* event2 Interrupt. */
};

/*
 * ISO Work Waiting Time factor
 */
#define BSCD_ISO_WORK_WAIT_TIME_DEFAULT_FACTOR 960
/* #define BSCD_EMV2000_EXTRA_WORK_WAIT_TIME_DEFAULT_FACTOR 480 */

/*
 * EMV 2000 Work Waiting Time factor in etu
 */
#define BSCD_EMV2000_WORK_WAIT_TIME_DELTA 1

/*
 * DEfault ISO Work Waiting Time Integer (WI)
 */
#define BSCD_ISO_DEFAULT_WORK_WAIT_TIME_INTEGER 10

/*
 * Mask to combine SC_RLEN1 and SC_RLEN1 into rlen. But
 * we only want the 9 bits of those 2 registers
 */
#define BSCD_RLEN_9_BIT_MASK 0x01ff

/*
 * For EMV and ISO T=1, the minimum interval btw the leading
 * edges of the start bits of 2 consecutive characters sent
 * in opposite directions shall be 22.
 */
#define BSCD_BLOCK_GUARD_TIME 22

/*
 * For EMV T=0 only, the minimum interval btw the leading
 * edges of the start bits of 2 consecutive characters sent
 * in opposite directions shall be 16.
 */
#define BSCD_MIN_DELAY_BEFORE_TZERO_SEND 16

/*
 * Maximum ETU allowed per ATR byte
 */
#define BSCD_MAX_ETU_PER_ATR_BYTE 9600
#define BSCD_MAX_ETU_PER_ATR_BYTE_EMV2000 10081

/*
 * Maximum ETU allowed for whole EMV ATR package
 */
#define BSCD_MAX_EMV_ETU_FOR_ALL_ATR_BYTES 19200
#define MAX_EMV_ETU_FOR_ALL_ATR_BYTES_EMV2000 20160

/*
 * Default EMV Information field size for T=1, used for IFSD defalut
 */
#define BSCD_DEFAULT_EMV_INFORMATION_FIELD_SIZE 0x20

/*
 * Default IFSC
 */
#define BSCD_DEFAULT_IFSC 0x20


/*
 * Reset Period
 */
#define BSCD_MAX_RESET_IN_CLK_CYCLES 42000

#define BSCD_MAX_ATR_SIZE 33
#define BSCD_MAX_ATR_HISTORICAL_SIZE 15
#define BSCD_MAX_APDU_SIZE 500

/*
 * ATR timing
 */
#define BSCD_MAX_ATR_START_IN_CLK_CYCLES 40000
#define BSCD_EMV2000_MAX_ATR_START_IN_CLK_CYCLES 42000
#define BSCD_MIN_ATR_START_IN_CLK_CYCLES 400

/*
 * Delay between when ATR is sent from SmartCard
 * and when ATR_START bit get set. In micro secs
 */

#ifdef BESPVR
#define BSCD_ATR_START_BIT_DELAY_IN_CLK_CYCLES      200 /* TDA8004 clk_div =1*/
#else
#define BSCD_ATR_START_BIT_DELAY_IN_CLK_CYCLES      250
#endif

#ifdef FPGA_CM3
#define BSCD_DELAY_MSEC_COUNTS                       525
#else
#define BSCD_DELAY_MSEC_COUNTS                       230 /* external RAM */
#endif

/* Internal Clock Frequency in Hz */
#define BSCD_DEFAULT_EXTERNAL_CLOCK_DIVISOR          1

/* Internal Clock Frequency in Hz */
#define BSCD_INTERNAL_CLOCK_FREQ_27MHz  27000000

#define BSCD_INTERNAL_CLOCK_FREQ_40MHz  40000000
/* Cygnus use fixed 40MHz clk source */
#define BSCD_INTERNAL_CLOCK_FREQ BSCD_INTERNAL_CLOCK_FREQ_40MHz

/* Default SC_CLK_CMD[etu_clk_div] */
#define BSCD_DEFAULT_ETU_CLK_DIV        1

/* Default SC_CLK_CMD[sc_clk_div] */
#define BSCD_DEFAULT_SC_CLK_DIV         10

/* Default SC_PRESCALE */
#define BSCD_DEFAULT_PRESCALE           119

/* Max SC_PRESCALE */
#define BSCD_MAX_PRESCALE               0xFF

/* Default Baud Divisor */
#define BSCD_DEFAULT_BAUD_DIV           31

/* Default Clock Rate Conversion Factor */
#define BSCD_DEFAULT_F                  1

/* Default Baud Rate Adjustment Factor */
#define BSCD_DEFAULT_D                  1

/* Default number of transmit parity */
#define BSCD_DEFAULT_TX_PARITY_RETRIES  5

/* Default EMV 2000 number of transmit parity */
#define BSCD_EMV2000_DEFAULT_TX_PARITY_RETRIES  4

/* maximum number of transmit parity */
#define BSCD_MAX_TX_PARITY_RETRIES              7

/* Default number of receiving parity */
#define BSCD_DEFAULT_RX_PARITY_RETRIES          5

/* Default EMV 2000 number of receiving parity */
#define BSCD_EMV2000_DEFAULT_RX_PARITY_RETRIES  4

/* maximum number of receiving parity */
#define BSCD_MAX_RX_PARITY_RETRIES              7

/* Default work waiting time in ETUs */
#define BSCD_DEFAULT_WORK_WAITING_TIME          9600

#define BSCD_DEFAULT_EXTRA_WORK_WAITING_TIME_EMV2000  480

/* Default block waiting time in ETUs = 11 + 960 etus, where bwi = 0*/
#define BSCD_DEFAULT_BLOCK_WAITING_TIME               971

#define BSCD_DEFAULT_EXTRA_BLOCK_WAITING_TIME_EMV2000 960

/* Default Extra Guard Time in ETUs */
#define BSCD_DEFAULT_EXTRA_GUARD_TIME                 2

/*
 * For EMV and ISO T=1, the minimum interval btw the leading edges of the start
 * bits of 2 consecutive characters sent in opposite directions shall be 22
 * ETUs.
 */
#define BSCD_DEFAULT_BLOCK_GUARD_TIME          22

/* Min Block Guard Time */
#define BSCD_MIN_BLOCK_GUARD_TIME              12

/* Max Block Guard Time*/
#define BSCD_MAX_BLOCK_GUARD_TIME              31

/* Default character wait time in ETUs */
#define BSCD_DEFAULT_CHARACTER_WAIT_TIME_INTEGER            0

#define BSCD_CHARACTER_WAIT_TIME_GRACE_PERIOD               1

/* Maximum character wait time in ETUs */
#define BSCD_MAX_CHARACTER_WAIT_TIME_INTEGER               15

/* Default timeout period for each transaction in milli seconds. */
#define BSCD_DEFAULT_TIME_OUT                              13000


/*
 * T14 Irdeto clock Rate Conversion factor.Spec says 625, but card prefers
 * 600
 */
#define BSCD_T14_IRDETO_CONSTANT_CLOCK_RATE_CONV_FACTOR    600

/* Irdeto needs to wait for minimum of 1250 from last RX to the next TX */
#define BSCD_T14_IRDETO_MIN_DELAY_RX2TX                    1250

/* Maximum supported Channels in BSCD module. */
#define BSCD_MAX_SUPPORTED_CHANNELS    2

/* Maximum supported callback functions per interrupt. */
#define BSCD_MAX_NUM_CALLBACK_FUNC    2

/*
 * The maximum number of bytes that system can write into the hardware transmit
 * buffer
 */
#define BSCD_MAX_TX_SIZE          264

/*
 * The maximum number of bytes that system can write into the hardware receive
 * buffer.
 */
#define BSCD_MAX_RX_SIZE          264

/* The maximum number of bytes that EMV package */
#define BSCD_MAX_EMV_BUFFER_SIZE          258

/* The default db width in IF_CMD_2 */
#define BSCD_DEFAULT_DB_WIDTH             0x07   /* around 10 ms */

/* The maximum db width in IF_CMD_2 */
#define BSCD_MAX_DB_WIDTH                 0x3F

/*
 * Definition to be used in member status1 of struct bscd_status
 * Either one of the following defined values shall be set to status1.
 * Use OR if you have more than one.
 */
#define BSCD_RX_PARITY                0x0800
#define BSCD_TX_PARITY                0x0400
#define BSCD_RX_TIMEOUT               0x0200
#define BSCD_TX_TIMEOUT               0x0100
#define BSCD_EDC_ERROR                0x0008
#define BSCD_HARDWARE_FAILURE         0x0004
#define BSCD_RST_CHNL_REQD   0x0002

#define BERR_SCD_ID     0x0C
#define BERR_MAKE_CODE(id, num) ((id << 16) | num)
/* Smart card module status codes */
#define BSCD_STATUS_ATR_SUCCESS      BERR_MAKE_CODE(BERR_SCD_ID, 0)
#define BSCD_STATUS_ATR_FAILED       BERR_MAKE_CODE(BERR_SCD_ID, 1)
#define BSCD_STATUS_READ_SUCCESS     BERR_MAKE_CODE(BERR_SCD_ID, 2)
#define BSCD_STATUS_READ_FAILED      BERR_MAKE_CODE(BERR_SCD_ID, 3)
#define BSCD_STATUS_SEND_SUCCESS     BERR_MAKE_CODE(BERR_SCD_ID, 4)
#define BSCD_STATUS_SEND_FAILED      BERR_MAKE_CODE(BERR_SCD_ID, 5)
#define BSCD_STATUS_WARM_RESET       BERR_MAKE_CODE(BERR_SCD_ID, 6)
#define BSCD_STATUS_DEACTIVATE       BERR_MAKE_CODE(BERR_SCD_ID, 7)
#define BSCD_STATUS_OTHER_ERROR      BERR_MAKE_CODE(BERR_SCD_ID, 8)
#define BSCD_STATUS_LOOPBACK_DONE    BERR_MAKE_CODE(BERR_SCD_ID, 9)
#define BSCD_STATUS_TIME_OUT         BERR_MAKE_CODE(BERR_SCD_ID, 10)
#define BSCD_STATUS_PARITY_EDC_ERR   BERR_MAKE_CODE(BERR_SCD_ID, 11)
#define BSCD_STATUS_NO_SC_RESPONSE   BERR_MAKE_CODE(BERR_SCD_ID, 12)
#define BSCD_STATUS_FAILED           BERR_MAKE_CODE(BERR_SCD_ID, 13)

/*
 * Summary:
 * This enum is to identify the smart card T=0 or T=1 protocol.
 *
 * Description:
 * This enumeration defines the smart card asynchronous and synchronous
 * transmission protocol.
 */
enum bscd_async_proto_type {
	async_proto_unknown = 0, /* Initial value */
	async_proto_e0,/*
			* Character-oriented
			* asynchronous transmission protocol
			*/
	async_proto_e1, /* Block-oriented asynchronous transmission protocol */
	async_proto_e0_sync, /* Type 1 synchronous transmission protocol */
	async_proto_e1_sync, /* Type 2 synchronous transmission protocol */
	async_proto_e14_irdeto /*
				* Irdeto T=14 proprietary transmission
				* protocol
				*/
};

/*
 * Summary:
 * This enum is to identify the smart card type.
 *
 * Description:
 * This enumeration defines the smart card type supported.
 */
enum bscd_card_type {
	card_unknown = 0, /* Initial value */
	card_acos,        /* ACOS smart card by Advanced Card Systems */
	card_java,        /* Java card */
	card_pki          /* PKI card */
};

/*
 * Summary:
 * This enum is to identify the source of the smartcard clock frequency.
 *
 * Description:
 * This enumeration defines the smart card external or internal clock frequency.
 */
enum bscd_clk_freq_src {
	clkfreq_src_unknown = 0,      /* Initial Value */
	clkfreq_src_internal_clk = 1, /* Internal Clock. 40MHz on Cygnus*/
	clkfreq_src_external_clk = 2  /*
				       * External Clock. Must further
				       * specify the clock frequency in Hz
				       */
};

/*
 * Summary:
 * This enum is to identify the smart card standards.
 *
 * Description:
 * This enumeration defines the supported smart card standards.
 */
enum bscd_standard {
	std_unknown = 0,/* Initial Value */
	std_nds,        /* NDS. T=0 with flow control. */
	std_iso,        /* ISO 7816. T=0 or T=1*/
	std_emv1996,    /* EMV. T=0 or T=1 */
	std_emv2000,    /* EMV. T=0 or T=1 */
	std_irdeto,     /*
			 * Irdeto. T=14. Need Major software workarouond to
			 * support this
			 */
	std_arib,       /* ARIB. T=1, Obsolete. Use ISO */
	std_mt,         /* MT, T=??. Obsolete. Use ISO */
	std_conax,      /* Conax, T=??. Obsolete. Use ISO */
	std_es          /* ES, T=1. Obsolete. Use ISO */
};

/*
 * Summary:
 * This enum is to identify error detect code (EDC) encoding.
 *
 * Description:
 * This enumeration defines the supported error detect code (EDC) encoding .
 */
enum bscd_edc_encode {
	edc_encode_lrc = 0, /* 1-byte LRC */
	edc_encode_crc      /* 2-byte CRC */
};

/*
 * Summary:
 * This enum is to identify action for chnl_reset_ifd function.
 *
 * Description:
 * This enumeration defines the supported action for chnl_reset_ifd function.
 *
 * See Also:
 * chnl_reset_power_icc
 */
enum bscd_rst_type {
	rst_cold = 0,     /* Cold Reset */
	rst_warm = 1      /* Warm Reset */
};

/*
 * Summary:
 * This enum is to identify action for chnl_reset_power_icc function.
 *
 * Description:
 * This enumeration defines the supported action for chnl_reset_power_icc
 * function.
 *
 * See Also:
 * chnl_reset_power_icc
 */
enum bscd_power_icc {
	pwricc_pwrup = 0,   /*
			     * power up the ICC and request
			     * activation of the contact
			     */
	pwricc_pwrdwn = 1   /*
			     * power down the ICC and request
			     * deactivation of the contact
			     */
};

/*
 * Summary:
 * This enum is to identify voltage level for chnl_set_vcc_level function.
 *
 * Description:
 * This enumeration defines the supported action for chnl_set_vcc_level
 * function.
 *
 * See Also:
 * chnl_set_vcc_level
 */
enum bscd_vcc_level {
	vcc_level_5V  = 0,  /* 5v is default value */
	vcc_level_3V  = 1,  /* 3v */
	vcc_level_18V = 2   /* 1.8v */
};

/*
 * Summary:
 * This enum is to identify action for chnl_rst_card function.
 *
 * Description:
 * This enumeration defines the supported action for chnl_rst_card function.
 *
 * See Also:
 * chnl_rst_card
 */
enum bscd_rst_card_action {
	rst_card_act_no_aciton = 0, /*
				     * Only Reset the card, do not return
				     * ATR data.Use chnl_rcv to read the ATR
				     * data
				     */
	rst_card_act_rcv_decode = 1 /*
				     * Reset the card , read and decode ATR and
				     * program the registers accordingly. Caller
				     * still has to call chnl_rcv to read the
				     * ATR data
				     */
};


/*
 * Summary:
 * This enum is to identify the unit of timer value.
 *
 * Description:
 * This enumeration defines the supported unit of timer value.
 */
enum bscd_timer_unit {
	unit_etu = 0, /* in Elementary Time Units */
	unit_clk,     /* in raw clock cycles that smart card receives */
	unit_ms       /* in milli seconds */
};

/*
 * Summary:
 * This enum is to identify which timer register we are accessing.
 *
 * Description:
 * This enumeration defines the supported timer register we are accessing.
 */
enum bscd_timer_type {
	timer_gpt = 0,  /* General Purpose Timer */
	timer_wait      /* Wait Timer */
};

/*
 * Summary:
 * This enum is to identify the General-purpose Timer Start Mode in SC_TIMER_CMD
 * register.
 *
 * Description:
 * This enumeration defines the General-purpose Timer Start Mode in SC_TIMER_CMD
 * register.
 */
enum bscd_gpt_time_mode {
	gpt_timer_mode_immediate = 0,  /* Start Timer Immediate */
	gpt_timer_mode_nxt_start_bit   /* Start Timer on Next Start Bit */
};


/* This enum is used to identify */
/*
 * Summary:
 * This enum is to identify the Waiting Timer Mode in SC_TIMER_CMD register.
 *
 * Description:
 * This enumeration defines the Waiting Timer Mode in SC_TIMER_CMD register.
 */
enum bscd_wait_time_mode {
	wtm_work = 0,   /* Work Waiting Time Mode */
	wtm_blk         /* Block Waiting Time Mode */
};


/*
 * Summary:
 * This enum is to identify the card is inserted or removed.
 *
 * Description:
 * This enumeration defines the insertion or removal of the card.
 */
enum bscd_card_present {
	card_removed = 0,   /* card removed */
	card_inserted       /* card inserted */
};


/*
 * Summary:
 * This enum is to identify read or write a specific register.
 *
 * Description:
 * This enumeration defines read or write of a smart card register.
 */
enum bscd_sc_pres_mode {
	mode_debounce = 0,
	mode_mask
};

/* Smart Card Public Data Structures */
/*
 * Summary:
 * This struct specifies if we are intereseted in General-Purpose timer
 * or Wait Timer.
 *
 * Description:
 * This structure defines if we are intereseted in General-Purpose timer
 * or Wait Timer.
 */
struct bscd_timer {
	enum bscd_timer_type timer_type;   /* timer type */
	union {
		enum bscd_gpt_time_mode gpt_timer_mode;  /*
							  * General-purpose
							  * Timer Start Mode
							  */
		enum bscd_wait_time_mode wait_time_mode;
	} timer_mode;             /* timer mode */
	bool is_timer_ena;        /* enable or disable */
	bool is_timer_intr_ena;   /* enbale or disable timer interrupt */
};

/*
 * Summary:
 * The timer value that application set to or get from the device.
 *
 * Description:
 * This structure defines timer value that application set to or get from the
 * device.
 */
struct bscd_timer_value {
	unsigned int      value;       /* timer value */
	enum bscd_timer_unit   unit;   /* units */
};

/*
 * Summary:
 * The source of smartcard clock frequency and number of clock
 * frequency in Hz if external clock is used.
 *
 * Description:
 * The source of smartcard clock frequency and number of clock
 * frequency in Hz if external clock is used.
 */
struct bscd_clock_freq {
	enum bscd_clk_freq_src   freq_src;   /* source of clock frequency */
	unsigned int clk_freq;  /*
				 * Clock frequency in Hz. Only specified
				 * this when external clock is used. For
				 * Internal clock, this is always
				 * 27000000 Hz.
				 */
};

/*
 * Summary:
 * The configuration of EDC setting for T=1 protocol only.
 *
 * Description:
 * This structure specifies what EDC enconding shall be used and whether
 * hardware shall handle the EDC automatically.
 */
struct bscd_edc_settings {
	enum bscd_edc_encode edc_encode;   /* EDC encoding */

	bool is_enabled;  /*
			   * True if enabled and
			   * hardware shall compute
			   * the EDC byte(s) and append
			   * it as the last byte when transmitting.
			   * Hardware shall verify EDC byte(s)
			   * when receiving.
			   * Otherwise, it is false and application
			   * shall compute and append the EDC
			   * byte(s) when transmitting and verify
			   * EDC byte(s) when receiving.
			   */
};

/*
 * Summary:
 * The configuration of SC_Pres Debounce.
 *
 * Description:
 * This structure specifies how to configure SC_Pres Debounce in IF_CMD_2.
 */
struct bscd_sc_pres_dbinfo {
	enum bscd_sc_pres_mode sc_pres_mode;   /* EDC encoding */
	bool is_enabled;   /* True if enabled. Otherwise it should be false*/
	unsigned char db_width;    /* db_width[5:0] */
};

/*
 * Summary:
 * The status of the device that driver will return to the application.
 *
 * Description:
 * This structure defines the status of the device that driver will return
 * to the application.
 */
struct bscd_status {
	bool card_present;  /* true if card present. Otherwise it is false.*/
	bool card_activate; /* Has card ever been reset (e.g. get ATR) */
	bool pps_done;      /* if PPS done or not */
	unsigned int status1;     /* status reported back to application */
	unsigned int status2;     /* reserved */
};

/*
 * Summary:
 * This struct specifies the card characterics.
 *
 * Description:
 * This structure defines various card characterics supported.
 */
/* This is JAVA card specific */
struct bscd_java_ctx {
	unsigned short pvt_data_size;    /* Private data size */
	unsigned short public_data_size; /* Public data size */
};

struct bscd_acos_user_file {
	unsigned char  record_len;
	unsigned char  record_no;
	unsigned char  read_attr;
	unsigned char  write_attr;
	unsigned short file_id;
};

/* This is ACOS card specific */
enum phx_sc_card {
	SC_ACOS2,
	SC_ACOS3,
	SC_NON_ACOS
};

struct bscd_acos_ctx {
	enum          phx_sc_card card;
	unsigned int  card_stage;
	unsigned char reg_option;
	unsigned char reg_security;
	unsigned char num_of_files;
	unsigned char personalized;/* if the card has personalization done */
	unsigned char serial_num_string[8];   /* card serial number */
	unsigned int  record_max_len;         /* user file max record length */
	unsigned int  record_max_num;         /* user file max record number */
	unsigned char code_num;               /* Code reference */
	struct bscd_acos_user_file user_file; /* User data file */
};

#define BSCD_MAX_OBJ_ID_STRING 20
struct bscd_card_object_id {
	unsigned char id_num;
	unsigned char id_string[BSCD_MAX_OBJ_ID_STRING];
};

#define BSCD_MAX_OBJ_ID_SIZE 5
struct bscd_card_type_ctx {
	enum bscd_card_type card_type;   /* Type of card */
	enum bscd_vcc_level vcc_level;   /* Vcc voltage level */
	unsigned int inited;
		union {
		   struct bscd_java_ctx java_ctx;
		   struct bscd_acos_ctx acos_ctx;
		} card_char;        /* Card characterics */
	unsigned char current_id;
	struct bscd_card_object_id obj_id[BSCD_MAX_OBJ_ID_SIZE];
	unsigned char hist_bytes[BSCD_MAX_ATR_HISTORICAL_SIZE];
};

/*
 * Summary:
 * Required default settings structure for smart card module.
 *
 * Description:
 * The default setting structure defines the default configure of
 * smart card. Since BSCD has multiple channels, it also has
 * default settings for a channel.
 *
 * See Also:
 * bscd_chnl_settings, open()
 */
struct bscd_settings {
	struct bscd_clock_freq mod_clk_freq;   /*
						* This attribute indicates
						* the source of clock and the
						* value
						*/
	unsigned char max_chan;   /* maximum SCD channels supported */
};

/* Required default settings structure for smart card type. */
struct bscd_card_type_settings {
	enum bscd_card_type card_type;   /* Type of card */
	unsigned int card_id_length;    /* Length used to identify card */
	/* Historical bytes */
	unsigned char hist_bytes[BSCD_MAX_ATR_HISTORICAL_SIZE];

};

/*
 * Summary:
 * Required default settings structure for smart card channel.
 *
 * Description:
 * The default setting structure defines the default configure of
 * smart card.
 *
 * See Also:
 * openChannel()
 */
struct bscd_chnl_settings {
	enum bscd_standard sc_std;                /* Smart Card Standard */
	enum bscd_async_proto_type proto_type;    /* Async Protocol Types. */

	struct bscd_card_type_ctx ctx_card_type;  /* Card characteristics */

	unsigned long src_clk_freq_hz;

	/*
	 * This read-only current ICC CLK frequency in Hz which is
	 * source freq / SC_CLK_CMD[etu_clk_div] /SC_CLK_CMD[sc_clk_div] /
	 * external_clock_div
	 */
	unsigned long curr_icc_clk_freq;

	/*
	 * This read-only current baudrate which is
	 * source freq / SC_CLK_CMD[etu_clk_div] / (
	 * SC_PRESCALE * external_clock_div +
	 * (external_clock_div - 1)) /SC_CLK_CMD[baud_div]
	 */
	unsigned long curr_baud_rate;

	/* This read-only attribute specifies the maximum IFSD. Should be 264.*/
	unsigned long max_ifsd;

	unsigned long curr_ifsd;

	unsigned char ffactor;     /*
				    * Clock Rate Conversion Factor,
				    * F in 1,2,3,4,5,6,9, 10, 11, 12 or 13.
				    * Default is 1.
				    */

	unsigned char dfactor;     /*
				    * Baud Rate Adjustment Factor,
				    * D in 1,2,3,4,5,6,8 or 9. Default is 1.
				    */

	unsigned char etu_clk_div; /*
				    * ETU Clock Divider in SC_CLK_CMD reg.
				    * Valid value is from 1 to 8.
				    * Default is 6.
				    */

	unsigned char sc_clk_div;  /*
				    * SC Clock Divider in SC_CLK_CMD reg.
				    * Valid value is 1,2,3,4,5,8,10,16.
				    * Default is 1.
				    */
	unsigned long prescale;    /*
				    * Prescale Value. This value is the final
				    * value that already takes external clock
				    * divisor into account
				    */

	unsigned char external_clk_div;  /*
					  * external clock divisor,
					  * could 1, 2, 4, 8
					  */
	unsigned char baud_div;   /* baud divisor , could be 31 or 32 */

	unsigned char tx_retries; /*
				   * Number of transmit parity retries per
				   * character in SC_UART_CMD_2 register.
				   * Default is 4 and max is 6. 7 indicates
				   * infinite retries
				   */
	unsigned char rx_retries; /*
				   * Number of receive parity retries per
				   * character in SC_UART_CMD_2 register.
				   * Default is 4 and max is 6.
				   * 7 indicates infinite retries
				   */
	struct bscd_timer_value work_wait_time; /*
						 * work waiting time in
						 * SC_TIME_CMD register.
						 * Other than EMV standard,
						 * only valid if current
						 * protocol is T=0.
						 */

	/* blk Wait time in SC_TIME_CMD reg.Only valid if cur proto is T=1.*/
	struct bscd_timer_value blk_wait_time;

	/* Extra Guard Time in SC_TGUARD register. */
	struct bscd_timer_value extra_guard_time;

	/*
	 * block Guard time in SC_BGT register. Other than EMV standard,
	 * only valid if current protocol is T=1.
	 */
	struct bscd_timer_value blk_guard_time;

	/*
	 * character Wait time Integer in SC_PROTO_CMD register.
	 * Only valid if current protocol is T=1.
	 */
	unsigned long char_wait_time_integer;

	/* EDC encoding. Only valid if current protocol is T=1. */
	struct bscd_edc_settings edc_setting;

	/*
	 * arbitrary Time Out value in milli seconds for any syn
	 * transaction.
	 */
	struct bscd_timer_value timeout;

	/* Specify if we need auto deactivation sequence */
	bool auto_deactive_req;

	bool null_filter;       /*
				 * True if we receive 0x60 in T=0, we will
				 * ignore it. Otherwise, we treat 0x60 as a
				 * valid data byte
				 */
	/* Debounce info for IF_CMD_2 */
	struct bscd_sc_pres_dbinfo pres_dbinfo;

	/* Specify if we want the drv to read, decode and program registers */
	enum bscd_rst_card_action rst_card_act;

	struct bscd_timer_value blk_wait_time_ext;  /*
						     * Specify the block wait
						     * time extension
						     */
	bool tpdu; /* True if packet is in TPDU rather than APDU */

	unsigned char curr_ifsc;     /* current IFSC size (T=1 only) */
};
#endif /* BSCD_DATATYPES_H__ */
