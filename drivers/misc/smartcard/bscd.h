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

/*
 * ================== Module Overview =====================================
 * Module Overview
 * The Smart Card (SCD) API is a porting interface(PI) module which
 * Allows status/control of smart card communication.
 *
 * Usage
 * Since most of the chips and reference designs can support multiple identical
 * And completely indepedent smart card interfaces, SCD shall control multiple
 * Smartcard channels with different settings concurrently. However, the whole
 * System should have only one module handle.
 *
 * During system initialization, the system may first call get_default_settings
 * To acquire the default settings for the smart card module. The system shall
 * Then call open to create a module handle. It may call
 * Get_chnl_default_settings to retrieve the default settings for the
 * Specific smart card channel and chnl_open to create a channel handle.
 * The system shall reset and configure the channel and detect the smart card.
 * Upon receiving ATR (Answer To Reset) data after the system reset the
 * Smart card, the system can then communcate with smart card associated with
 * This channel. All the I/O data shall be automically processed according to
 * the Inverse or direct convention. During system shutdown, the system shall
 * call Chnl_close to close this channel and call close to close the Smart card
 * module.
 *
 * Each channel can support ISO 7816 asynchronous T=0 and T=1 modes with
 * 264-byte UART receive and transmit buffers. Each channel can have different
 * Baud rate but all channels have to use either internal clock or external
 * Clock. SCD could defines various smart card timing and error management
 * Registers to conform to different standards with minimal CPU overhead and
 * Interrupt latency.
 * SCD should be able to support NDS, EMV 1996, ARIB and ISO smart card
 * Standards.
 *
 * SCD is expected to be able to support various smart card standards, therefore
 * It is designed to be as much standard independent as possible. Application is
 * Expected to be standard awareness and to handle all the standard comformance
 * Requirements.
 *
 * The caller usually first calls SCD to setup the related registers to receive
 * ATR. The caller then interprets ATR and setup the registers again for
 * Transmitting and receiving data. Since all these scenarioes occur
 * sequentially And synchronously, there is no reason for the caller to call
 * more than one SCD functions to access one channel concurrently with multiple
 * threads.
 */

#ifndef BSCD_H__
#define BSCD_H__
#include <linux/spinlock.h>
#include "bscd_datatypes.h"
/*
 * Summary:
 * SCD channel context handle.
 *
 * Description:
 * Opaque handle that is created in chnl_open.
 * BSCD_ChannelHandle holds the context of the smart card channel. The system
 * could have more than one BSCD_ChannelHandle if the chip can support multiple
 * smartcard interfaces/channels. Caller of chnl_open is responsible
 * to store this BSCD_ChannelHandle and uses it for the future function call
 * after chnl_open function returns successfully.
 *
 * See Also:
 * open, chnl_open
 *
 * struct p_chnl_hdl;
 */

/*
 * Summary:
 * Required default settings structure for smart card module.
 *
 * Description:
 * The default setting structure that defines the default configure of
 * smart card module when the module is initialized.
 *
 * See Also:
 * get_default_settings, open.
 *
 * struct bscd_settings
 */

/*
 * Summary:
 * Smart card coupler enum.
 *
 * Description:
 * The supported couplers
 */
enum phx_rfid_system {
	COUPLER_NXP8024,
	COUPLER_NXP8026
};


/*
 * Summary:
 * Smart Card (SCD) module context handle.
 *
 * Description:
 * Opaque handle that is created in open.
 * BSCD_Handle holds the context of the smart card module. The system
 * should have only one BSCD_Handle. Caller of open is responsible to store
 * this BSCD_Handle and uses it for the future function call after open function
 * returns successfully.
 *
 * See Also:
 * open, chnl_open
 */
struct bscd_p_handle {
	unsigned long        magic_number;/*Must be BSCD_P_HANDLE_MAGIC_NUMBER*/
	struct p_chnl_hdl    *chnl_hdls[BSCD_MAX_SUPPORTED_CHANNELS];
	struct bscd_settings cur_settings;   /* current settings */
	enum phx_rfid_system coupler_type;
	spinlock_t           lock;
};

/* Smart Card Interrupt Callback function. */
typedef void (*isrcb)(struct p_chnl_hdl *hdl, void *inp_data);

/*
 * Summary:
 * Required default settings structure for smart card channel.
 *
 * Description:
 * The default setting structure defines the default configure of
 * smart card interface/channel when the interface is open. Since SCD
 * could support multiple smart card interfaces/channels, system may have
 * more than one default channel settings that each channel may have
 * different default channel settings.
 *
 * See Also:
 * get_chnl_default_settings, chnl_open
 *
 * struct bscd_chnl_settings
 */

/*
 * Summary:
 * Smart Card Interrupt Callback function.
 *
 * Description:
 * Caller of BSCD shall call chnl_ena_int_cbisr
 * to register and enable the callback. BSCD shall call
 *
 * See Also:
 * chnl_ena_int_cbisr
 */
typedef void (*bicm_cb)(void *inp_parm1, int in_parm2);


/* Basic Module Functions */
/*
 * Summary:
 * This function shall return a recommended default settings for SCD module.
 *
 * Description:
 * This function shall return a recommended default settings for SCD module.
 * This function shall be called before open
 * and the caller can then over-ride any of the default settings
 * required for the build and configuration by calling open.
 *
 * These default settings are always the same regardless of how
 * many times this function is called or what other functions have
 * been called in the porting interface.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_GET_DEFAULT_SETTINGS or during insmod)
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 *
 * Output:
 * outp_settings - bscd_settings, a ref/pointer to the default setting.
 *
 * Returns:
 * BERR_SUCCESS - Always return success.
 *
 * See Also:
 * open
 */
int get_default_settings(struct bscd_settings *outp_settings);

/*
 * Summary:
 * This function creates the smart card module handle.
 *
 * Description:
 * This function shall create the smart card module handle.
 * It also initializes the smart card module
 * and hardware using settings stored in the p_Settings pointer.
 * All the associated channels are not ready to be access until
 * BSCD_ChannelOpen is called and returns successfully.
 *
 * The caller can pass a NULL pointer for inp_settings. If the
 * p_Settings pointer is NULL, default settings should be used.
 *
 * It is the caller responsibility to store the outp_handle and uses
 * it for the future function call after this function returns
 * successfully. If this function returns successfully, outp_handle shall
 * not be NULL.
 *
 * Before calling this function, the only function that the caller
 * can call is get_default_settings. System shall not call
 * any other smart card functions prior to this function.
 *
 * System shall not call this function more than once without calling close
 * previously.
 *
 * If illegal settings are passed in an error should be
 * returned and the hardware state should not be modified.
 *
 * The BINT_Handle is only required if this module needs to
 * associate ISR callback routines with L2 interrupts.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * during insmod )
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * inp_settings - bscd_settings, the settings that apply to smart card module
 * If NULL, a default setting shall be used.
 * BSCD_COUPLER_TYPE coupler_type
 *
 * Output:
 * outp_handle - BSCD_Handle, a ref/pointer to the smart card module handle.
 * It shall not be NULL if this function returns successfully.
 *
 * Returns:
 * BERR_SUCCESS - success
 * BERR_OUT_OF_SYSTEM_MEMORY - out of system memory
 *
 * See Also:
 * get_default_settings
 * close
 */
int open(struct bscd_p_handle **outp_handle,
	 const struct bscd_settings *inp_settings,
	 enum phx_rfid_system coupler_type);

/*
 * Summary:
 * This function frees the main handle and any resources contained
 * in the main handle.
 *
 * Description:
 * This function shall free the main handle and any resources contained
 * in the main handle. This function shall try to free any resources associated
 * with sub handles created from the main handle. However, this function
 * does not free any resources associated with channel handle.
 *
 * Regardless of the return value, this function always attempts to free all
 * the allocated resources and inout_hdl shall be NULL.
 *
 * Other than get_default_settings, system shall not call any other smart
 * card functions after this function returns, regardless of the return result.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * during rmmod)
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input/Output:
 * inout_hdl - BSCD_Handle, smart card module handle.
 *
 * Returns:
 * BERR_SUCCESS - success
 * BSCD_STATUS_FAILED - failed.
 *
 * See Also:
 * open
 * get_default_settings
 */
int close(struct bscd_p_handle *inout_hdl);


/*
 * Summary:
 * This function shall return a recommended default settings for SCD channel.
 *
 * Description:
 * This function shall return a recommended default settings for SCD channel.
 * This function shall be called before chnl_open
 * and the caller can then over-ride any of the default settings
 * required for the build and configuration by calling chnl_open
 * or chnl_reset_ifd.
 *
 * The caller shall pass in_chnl_num that is smaller outp_totalChannels in
 * BSCD_GetTotalChannels. The in_chnl_num for the first channel shall be zero.
 *
 * These default settings are always the same regardless of how
 * many times this function is called or what other functions have
 * been called in the porting interface.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_GET_CHANNEL_DEFAULT_SETTINGS
 * or device open )
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * in_hdl - BSCD_Handle, smart card module handle.
 * in_chnl_num - unsigned int, an index that indicates which channel or smart
 * card interface that the caller want to access.
 * Output:
 * outp_settings - bscd_chnl_settings, a ref/pointer to the default channel
 * setting.
 *
 * Returns:
 * BERR_SUCCESS - success
 * BSCD_STATUS_FAILED - failed.
 * BERR_INVALID_PARAMETER - in_chnl_num is invalid.
 *
 * See Also:
 * open
 * BSCD_GetTotalChannels
 * chnl_open
 */
int get_chnl_default_settings(struct bscd_p_handle *in_hdl,
			      unsigned int in_chnl_num,
			      struct bscd_chnl_settings *outp_settings);


/*
 * Summary:
 * This function creates the smart card channel handle.
 *
 * Description:
 * This function shall create the smart card channel handle.
 * It also initializes the specified smart card interface, all the associated
 * channels and hardware using settings stored in the in_chnl_def_settings
 * pointer.
 *
 * The caller shall pass in_chnl_num that is smaller outp_totalChannels in
 * BSCD_GetTotalChannels. The in_chnl_num for the first channel shall be zero.
 *
 * The caller can pass a NULL pointer for in_chnl_def_settings. If the
 * in_chnl_def_settings pointer is NULL, default settings should be used.
 *
 * It is the caller responsibility to store the outp_chnl_hdl and uses
 * it for the future function call after this function returns
 * successfully. If this function returns successfully, outp_handle shall
 * not be NULL.
 *
 * Before calling this function, the only channel related functions that
 * the system can call are BSCD_GetTotalChannels and
 * get_chnl_default_settings. System shall not call any other channel
 * related functions prior to this function.
 *
 * System shall not call this function more than once without calling
 * chnl_close previously.
 *
 * If illegal settings are passed in an error should be
 * returned and the hardware state should not be modified.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * device open)
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * in_hdl - BSCD_Handle, smart card module handle.
 * in_chnl_num - unsigned int, an index that indicates which channel or smart
 * card interface that the caller want to access.
 * in_chnl_def_settings - bscd_chnl_settings, the channel settings that
 * apply to this specific channel. If NULL, a default
 * channel setting shall be used.
 *
 * Output:
 * outp_chnl_hdl - BSCD_ChannelHandle, a ref/pointer to the smart
 * card channel handle.
 *
 * Returns:
 * BERR_SUCCESS - success
 * BSCD_STATUS_FAILED - failed.
 * BERR_INVALID_PARAMETER - in_chnl_num or part of in_chnl_def_settings
 * is invalid.
 *
 * See Also:
 * open
 * BSCD_GetTotalChannels
 * chnl_close
 */
int chnl_open(struct bscd_p_handle *in_hdl, struct p_chnl_hdl **outp_chnl_hdl,
	      unsigned int in_chnl_num,
	      const struct bscd_chnl_settings *in_chnl_def_settings,
	      void __iomem *in_reg_base);

/*
 * Summary:
 * This function frees the channel handle and any resources contained
 * in the channel handle.
 *
 * Description:
 * This function shall free the channel handle and any resources contained
 * in the channel handle.
 *
 * Regardless of the return value, this function always attempts to free all
 * the allocated resources and inout_chnl_hdl shall be NULL.
 *
 * Other than BSCD_GetTotalChannels and get_chnl_default_settings, system
 * shall not call any other channel related functions after this function
 * returns,regardless of the return result.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * device close)
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Returns:
 * BERR_SUCCESS - success
 * BSCD_STATUS_FAILED - failed.
 *
 * See Also:
 *chnl_open
 */
int chnl_close(struct bscd_p_handle *in_hdl, unsigned int in_chnl_num);

/* End of Basic Module Functions */

/* Module Specific Functions */
/*
 * Summary:
 * This function returns a specific smart card channel handle.
 *
 * Description:
 * This function returns a specific smart card channel handle.
 * The caller shall pass in_chnl_num that is smaller outp_totalChannels in
 * BSCD_GetTotalChannels. The in_chnl_num for the first channel shall be zero.
 * If this function returns successfully, outp_chnl_hdl shall not be NULL.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_GET_CHANNEL )
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * in_hdl - BSCD_Handle, smart card module handle.
 * in_chnl_num - unsigned int, an index that indicate which channel or
 * smart card inerface that the caller want to access.
 *
 * Output:
 * outp_chnl_hdl - BSCD_ChannelHandle, a ref/pointer to the smart card
 * channel handle.
 *
 * Returns:
 * BERR_SUCCESS - success
 * BSCD_STATUS_FAILED - failed.
 * BERR_INVALID_PARAMETER - in_chnl_num is invalid.
 *
 * See Also:
 * open
 * BSCD_GetTotalChannels
 * chnl_open
 */
int get_chnl(struct bscd_p_handle *in_hdl, unsigned int in_chnl_num,
	     struct p_chnl_hdl **outp_chnl_hdl);

/*
 * Summary:
 * This function shall wait until the card is inserted or removed.
 *
 * Description:
 * This function shall wait until the card is inserted or removed.
 *
 * This function shall returns immediately either
 * 1) the card is removed and in_ecard_present is card_removed, or
 * 2) the card is inserted and in_ecard_present is card_inserted.
 *
 * This function shall be blocked until the card is inserted if the card is
 * currently removed and in_ecard_present is card_inserted.
 *
 * This function shall be blocked until the card is removed if the card is
 * currently inserted and in_ecard_present is card_removed.
 *
 * If the caller does not want to be blocked, it could use
 * chnl_get_status to check the card presence status.
 *
 * Note:
 * The application may seem to be hang since this function may be blocked until
 * user inserting or removing the card.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_DETECT_CARD )
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 * This function may be
 * blocked until user inserts or removes the card. Since TDA8004 does not have
 * debounce feature for the presence switches, this function may
 * wait for extra 10ms for the presence switches to be stabilized.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a specific smart
 * card channel handle.
 * in_ecard_present - bscd_card_present, indicate if the caller wants to wait
 * until the card is inserted or removed.
 *
 * Returns:
 * BERR_SUCCESS - success
 * BSCD_STATUS_FAILED - failed.
 *
 * See Also:
 * chnl_get_status
 */
int chnl_detect_card(struct p_chnl_hdl *hdl,
		     enum bscd_card_present in_ecard_present);

/*
 * Summary:
 * This function shall modify the current smart card channel setting.
 *
 * Description:
 * This function shall modify the current smart card channel setting. For
 * better performance that this function can modify only a small set of changes,
 * it is strongly recommended that the caller should call
 * chnl_get_params to retrieve the current channel setting.
 * The caller should only update the modified members in bscd_settings and call
 * chnl_set_params to set the current setting.
 *
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_SET_PARAMETERS )
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a specific smart
 * card channel handle.
 * inp_settings - bscd_settings, the settings that apply to a specific
 * channel. If NULL, the function shall return an error.
 *
 * Returns:
 * BERR_SUCCESS - success
 * bxerr_InvalidArgument - inp_settings is NULL.
 * BSCD_STATUS_FAILED - failed.
 *
 * See Also:
 * chnl_get_params
 */
int chnl_set_params(struct p_chnl_hdl *hdl,
		    const struct bscd_chnl_settings *settings);

/*
 * Summary:
 * This function retrieves the current smart card channel setting.
 *
 * Description:
 * This function shall retrieve the current smart card channel setting. If
 * necessary, the caller can call chnl_set_params to modify the
 * current channel setting.
 *
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_GET_PARAMETERS )
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a ref/pointer to a specific smart
 * card channel handle.
 *
 * Output:
 * outp_settings - bscd_settings, the settings that apply to a specific
 * channel.
 *
 * Returns:
 * BERR_SUCCESS - success
 * BSCD_STATUS_FAILED - failed.
 *
 * See Also:
 * chnl_set_params
 */
int chnl_get_params(struct p_chnl_hdl *hdl,
		    struct bscd_chnl_settings *outp_settings);

/*
 * Summary:
 * This function retrieves the negotiate smart card channel setting pointer.
 *
 * Returns:
 * BERR_SUCCESS - success
 * BSCD_STATUS_FAILED - failed.
 *
 * See Also:
 * chnl_set_params
 */
int chnl_get_negotiate_params_ptr(struct p_chnl_hdl *hdl,
				  struct bscd_chnl_settings **outp_settings);

/*
 * Summary:
 * This function retrieves the current smart card channel number.
 *
 * Description:
 * This function shall retrieve the current smart card channel number.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_GET_CHANNEL_NUMBER )
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a ref/pointer to a specific smart
 * card channel handle.
 *
 * Output:
 * outp_ucchnl_number - unsigned char, the settings that apply to a specific.
 *
 * Returns:
 * Channel number, where 0 is the first channel. If it fails, the return
 * values is -1.
 */
int chnl_get_chnl_num(struct p_chnl_hdl *hdl);

/*
 * Summary:
 * This function deativates the specific smart card interface.
 *
 * Description:
 * This function shall deativate the specific smart card interface. The
 * deactivation sequence shall be:
 *
 * o Set SC_VCC high
 * o Set SC_RST low
 * o Disable SC_CLK
 * o IO is unconditionally driver low.
 *
 * The caller shall call chnl_reset_ifd to reset a specific smart card
 * channel.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_DEACTIVATE )
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a specific smart
 * card channel handle.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 *
 * See Also:
 * chnl_reset_power_icc
 * chnl_reset_ifd
 */
int chnl_deactivate(struct p_chnl_hdl *hdl);

/*
 * Summary:
 * This function reset a specific smart card interface (IFD).
 *
 * Description:
 * This function shall reset a specific smart card interface (IFD). It shall
 * not reset the smart card (ICC). This function shall reset the channel and
 * apply the settings stored in the p_Settings pointer to the channel and
 * hardware.
 *
 * This function shall reset the smart card interface in the following sequence:
 * Set SC_VCC high (Only Cold Reset)
 * Enable SC_CLK
 * Set SC_RST high
 * Reset UART transmit and receive buffer.
 *
 * No ATR data shall be received after this call since IO is yet to be
 * enabled. chnl_reset_power_icc with ResetICC option shall activate
 * the card so that the interface is ready to receive ATR.
 *
 * For TDA8004 emergency deactivation, we could use this function or
 * chnl_reset_power_icc with BSCD_ICCAction_ePowerUp
 * to set SC_VCC high after we realize the smart card is no longer present.
 * This will set the SC_VCC high and the next chnl_get_status will show the
 * correct presence of the card or next chnl_detect_card will response
 * correctly. The presence of card is unknown if this function is not called
 * after TDA8004 emergency deactivation.
 *
 * The caller shall call chnl_deactivate to deactivate a specific smart
 * card channel. The caller shall call BSCD_Reset to reset all the channels.
 *
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_RESET_CHANNEL )
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a ref/pointer to the smart card
 * channel handle.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 *
 *
 * See Also:
 * chnl_reset_power_icc
 * chnl_deactivate
 */
int chnl_reset_ifd(struct p_chnl_hdl *hdl,
		   enum bscd_rst_type in_rst_type);

/*
 * Summary:
 * This function shall set SC_VCC high, low or reset the smart card.
 *
 * Description:
 * There are 3 options to be selected in this function.
 *
 * If in_icc_action is BSCD_ICCAction_ePowerUp, this function shall set SC_VCC
 * high. System should call this function so that the next chnl_get_status
 * will show the correct presence of the card or next chnl_detect_card
 * will response correctly after TDA8004 emergency deactivation.
 *
 * If in_icc_action is BSCD_ICCAction_ePowerDown, this function shall set SC_VCC
 * low. The next chnl_get_status may not show the correct presence of the
 * card or next chnl_detect_card may not response correctly after
 * TDA8004 emergency deactivation.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_Power_ICC )
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, smart card channel handle.
 * in_icc_action - BSCD_ICCAction, the settings that apply to a specific
 * channel. If NULL, the interface shall be reset with the
 * current setting.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 *
 * See Also:
 * chnl_reset_power_icc
 * chnl_deactivate
 * chnl_rcv
 */
int chnl_reset_power_icc(struct p_chnl_hdl *hdl,
			 enum bscd_power_icc in_icc_action);

/*
 * Summary:
 * This function shall set SC_RST high or low
 */
int chnl_reset_signal(struct p_chnl_hdl *hdl, bool in_rst);

/*
 * Summary:
 * This function set voltage level for smart card interface.
 *
 * Description:
 * We have to modify the board to use VPP pin of smartcard and connect it to
 * pin 3 (3V/5V) of TDA chip and run this function. Make sure to disconnect
 * your QAM or QPSK connection before calling this function or your smartcard
 * will be damaged.
 *
 * We also have to use proper voltage card for test when we change smart card
 * interface voltage.
 * (3v card for 3v Smart Card interface, 5v card for 5v Smart Card interface ).
 * By default the Smart Card iterface is set 5V.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, smart card channel handle.
 * in_vcc_level - Smart Card interface voltage level ( 5V and 3V )
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 */
int chnl_set_vcc_level(struct p_chnl_hdl *hdl,
		       enum bscd_vcc_level in_vcc_level);

/*
 * Summary:
 * This function enable or disable "Insert Card Hardware Reset" feature in smart
 * card.
 *
 * Description:
 * When "Insert Card Hardware Reset" is enabled, the hardware will reset when
 * smart card is inserted.This is triggered by SC_STATUS_1 bit
 * card_pres = 0 -> 1
 * Make sure the SC_IF_CMD_1 bit pres_pol = 1, otherwise this trigger can not
 * happen. After hardware reset, by default this feature is dabled.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, smart card channel handle.
 * in_ena_hw_rst - true: Enable this feature
 * false: disable this feature
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 */
int chnl_ins_card_hw_reset(struct p_chnl_hdl *hdl, bool in_ena_hw_rst);

/*
 * Summary:
 * This function shall reset the smart card. Depends on the option, this
 * function could further read and interpret the ATR data and program the
 * registers accordingly.
 *
 * Description:
 * There are 2 options to be selected in this function.
 *
 * If in_icc_action is rst_card_act_no_aciton, this function shall only reset
 * the card. It shall not read ATR data. Caller has to call chnl_rcv to
 * receive the ATR data, decode it and then call chnl_set_params to program the
 * registers.
 *
 * If in_icc_action is rst_card_act_rcv_decode, this function shall reset
 * the card.
 * It then reads and decodes ATR data and programs the registers accordingly.
 * This option is required to support smart card standard, like EMV that has
 * stringent timing requirements on ATR error handling. Caller still has to call
 * chnl_rcv to receive the ATR data. The caller has the option if it wants to
 * decode it and then call chnl_set_params to program the registers.
 *
 * This function shall reset the smart card (ICC) in the following sequence:
 *
 * o SC_VCC low
 * o SC_RST high
 * o Wait for 42000 clock cycles.
 * o IO is ready to receive ATR.
 * o SC_RST low
 * o ICC must send ATR per standard requirements (For example between 400 and
 * 40,000 clock cycles).
 *
 *
 * Unless specify, the rest of this section describes the scenarioes if
 * in_icc_action is rst_card_act_rcv_decode:
 * This function shall be blocked until either
 *
 * 1) All ATR data is received.
 * 2) one of the timer expired.
 *
 * After ATR data are received correctly, the hardware shall determine if this
 * card support direct (TS, first character is 0x3B) or inverse convention
 * (TS is 0x3F). The caller shall call chnl_rcv to receive the ATR data.
 *
 *
 * This function shall parse the ATR data and modify the following settings
 * according to the received ATR data:
 *
 * o ProtocolType (T=0 or T=1)
 * o Clock Rate Conversion Factor
 * o Baud Rate Adjustment Factor
 * o Extra Guard Time
 * o Work Waiting time (For T=0 only. For EMV standard, it applies to T=1 too)
 * o Block Wait time (For T=1 only)
 * o Character Wait time (For T=1 only)
 *
 * This function shall enable certain interrupts for T=0 or T=1 and set
 * the parity retrial number.
 *
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_RESET_CARD )
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done. The maximum
 * number of ATR data is 32 and the default work waiting time is 9600 ETUs,
 * therefore the application can wait for 5 seconds, if each ETU is 165us,
 * before all the ATR bytes are received.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, smart card channel handle.
 * in_icc_action - bscd_rst_card_action, the settings that apply to a specific
 * channel. If NULL, the interface shall be reset with the current setting.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 *
 *
 * See Also:
 * chnl_reset_power_icc
 * chnl_deactivate
 * chnl_rcv
 */
int chnl_rst_card(struct p_chnl_hdl *hdl,
		  enum bscd_rst_card_action in_icc_action);

/*
 * Summary:
 * This function retrieves the smart card channel status and current software
 * state.
 *
 * Description:
 * This function shall retrieve the smart card status and current software
 * state.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_GET_STATUS )
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done. Since
 * TDA8004 does not have debounce feature for the presence switches, this
 * function may wait for extra 10ms for the presence switches to be stabilized.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, smart card channel handle.
 *
 * Output:
 * outp_status - bscd_status, a ref/pointer that indicates the currect status
 * and software state of the smart card module.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 *
 *
 * See Also:
 * chnl_detect_card
 */
int chnl_get_status(struct p_chnl_hdl *hdl,
		    struct bscd_status *outp_status);

/*
 * Summary:
 * This function transmits data to the smart card.
 *
 * Description:
 * This function shall be blocked until it has transmitted in_unNumXmitBytes
 * number of bytes in xmit_data to the card.
 *
 * This function shall be blocked until either all the bytes has been
 * transmitted.
 *
 * For NDS standard, this function shall set the SC_FLOW_CMD[flow_en] to 1 and
 * the hardware transmitter shall waits until the NDS flow control is deasserted
 * before transmitting the next byte.
 *
 * This function shall not interpret the transmitting data.
 *
 * For any parity error, the hardware shall retry the transmission for the
 * number of times specify in the setting.If ICC still reports transmission
 * parity error after that, SC_INTR_STAT_1[retry_intr] will set to 1 and caller
 * can call chnl_get_status to check if this is a transmission parity error.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * driver write)
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 * This function is blocked until it has transmitted in_unNumXmitBytes number
 * of bytes in xmit_data.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, smart card channel handle.
 * xmit_data - unsigned char, a ref/pointer to the buffer for transmitting data
 * in_unNumXmitBytes - unsigned long, a ref/pointer to the number of
 * transmitting bytes.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 *
 * See Also:
 * chnl_xmit
 * chnl_rcv
 * chnl_get_status
 */
int chnl_xmit(struct p_chnl_hdl *hdl, unsigned char *xmit_data,
	      unsigned long xmit_bytes);

/*
 * Summary:
 * This function receives data from the smart card after IFD transmits data to
 * the smart card.
 *
 * Description:
 * This function shall receive data in the rcv_data from the smart card,
 * after IFD transmits data to the smart card.
 *
 * This function is NOT recommended to be used to read ATR data. Use
 * chnl_rcv_atr to read ATR data since it handles the time out more
 * accurately. If the system does not care about the accurate timeout,
 * this function can be used to read the ATR data.
 *
 * For better performance, the caller is recommended to set max_read_bytes
 * equal to the number of expected receiving bytes. If the caller set
 * max_read_bytes to a number that is greater than the rcv_bytes
 * (for example, MAX_ATR_SIZE), this function shall return rcv_bytes
 * number of receiving bytes with an error. It is the responsibility of the
 * application to determine if this operation is succeed or not.
 *
 * This function shall be blocked until either
 *
 * 1) The number of receiving data is equal to max_read_bytes.
 * 2) An error occurs.
 *
 * This function shall return an error either
 *
 * 1) The number of receiving bytes is greater than max_read_bytes.
 * 2) One of the timer expired before max_read_bytes of bytes
 *   has received.
 *
 * This function shall not interpret receiving data.
 *
 * For any parity error, the hardware shall retry the receiving for the
 * number of times specify in the setting. If IFD still reports receiving parity
 * error after that, SC_INTR_STAT_1[retry_intr] will set to 1 and caller can
 * call chnl_get_status to check if this is a receiving parity error.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * driver read )
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a ref/pointer to the sc channel handle.
 * max_read_bytes - a ref/pointer to the number of maximum receiving bytes
 *
 * Output:
 * rcv_data - unsigned char, a ref/pointer to the buffer for receive data
 * rcv_bytes - a ref/pointer to the number of receiving bytes
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 *
 *
 * See Also:
 * chnl_xmit
 * chnl_rcv_atr
 * chnl_get_status
 */
int chnl_rcv(struct p_chnl_hdl *hdl, unsigned char *rcv_data,
	     unsigned long *rcv_bytes, unsigned long max_read_bytes);

/*
 * Summary:
 * This function receives Answer To Reset (ATR) data from the smart card.
 *
 * Description:
 * This function shall receive ATR data in the rcv_data from the smart card.
 * ATR data shall actually be received after calling chnl_reset_power_icc. This
 * function only retrieve ATR data from the hardware receiving buffer.
 *
 * This function is recommended to be used to return ATR data since the time out
 * period is more accurate. Use chnl_rcv
 * to read the data from the smart card after IFD transmits data to the
 * smart card.
 *
 * For better performance, the caller is recommended to set max_read_bytes
 * equal to the number of expected receiving bytes. If the caller set
 * max_read_bytes to a number that is greater than the rcv_bytes
 * (for example, MAX_ATR_SIZE), this function shall return rcv_bytes
 * number of receiving bytes with an error. It is the responsibility of the
 * application to determine if this operation is succeed or not.
 *
 * This function shall be blocked until either
 *
 * 1) The number of receiving data is equal to max_read_bytes.
 * 2) An error occurs.
 *
 * This function shall return an error either
 *
 * 1) The number of receiving bytes is greater than max_read_bytes.
 * 2) One of the timer expired before max_read_bytes of bytes
 *    has received.
 *
 * This function shall not interpret receiving data.
 *
 * For any parity error, the hardware shall retry the receiving for the
 * number of times specify in the setting. If IFD still reports receiving parity
 * error after that, SC_INTR_STAT_1[retry_intr] will set to 1 and caller can
 * call chnl_get_status to check if this is a receiving parity error.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * driver read )
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a ref/pointer to the sc channel handle.
 * max_read_bytes - a ref/pointer to the number of maximum receiving
 * bytes
 *
 * Output:
 * rcv_data - a ref/pointer to the buffer for receive data
 * rcv_bytes - a ref/pointer to the number of receiving bytes
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 *
 *
 * See Also:
 * chnl_xmit
 * chnl_rcv
 * chnl_get_status
 */
int chnl_rcv_atr(struct p_chnl_hdl *hdl, unsigned char *rcv_data,
		 unsigned long *rcv_bytes, unsigned long max_read_bytes);

/*
 * Summary:
 * This function configures the Waiting Timer or General Purpose Timer.
 *
 * Description:
 * This function shall configure the Waiting Timer or General Purpose Timer.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_CONFIG_TIMER).
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, smart card channel handle.
 * inp_timer - bscd_timer, a ref/pointer to the smart card timer structure.
 * inp_cnt - bscd_timer_value, a ref/pointer to the smart card timer value
 * and unit.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 */
int chnl_config_timer(struct p_chnl_hdl *hdl, struct bscd_timer *inp_timer,
		      struct bscd_timer_value *inp_cnt);

/*
 * Summary:
 * This function check to see if the Waiting Timer or General Purpose
 * Timer is enabled or not.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, smart card channel handle.
 * timer_type - bscd_timer_type
 *
 * Returns:
 * bool
 */
bool chnl_is_timer_ena(struct p_chnl_hdl *hdl,
		       enum bscd_timer_type timer_type);

/*
 * Summary:
 * This function enables or disables the Waiting Timer or General Purpose Timer.
 *
 * Description:
 * This function shall either
 *
 * 1) Disable Waiting Timer or General Purpose Timer.
 * 2) Enable Waiting Timer in Work Waiting Time Mode or Block Waiting Time Mode.
 * 3) Enable General Purppose Timer in Start Timer Immediate mode or Start Timer
 *    on Next Start Bit mode.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_ENABLE_DISABLE_TIMER).
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a ref/pointer to the smart card
 * channel handle.
 * inp_timer - bscd_timer, a ref/pointer to the smart card timer structure.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 */
int chnl_ena_dis_timer_isr(struct p_chnl_hdl *hdl,
			   struct bscd_timer *inp_timer);

/*
 * Summary:
 * This function enables a specific smart card interrupt.
 *
 * Description:
 * This function enables a specific smart card interrupt.
 *
 * There are 2 callback functions that can be registered with a specific
 * interrupt. One of them must be reserved for default callback function
 * provided by this module. The other callback function is opt to use for
 * customized callback function.Therefore this function can only register 2
 * different callback functions per specific interrupt.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_ENABLE_INTR_CALLBACK).
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a ref/pointer to the smart card
 * channel handle.
 * intr_type - BICM_IntrType, Interrupt type.
 * cbhdl - bicm_cb, callback function.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 */
int chnl_ena_int_cbisr(struct p_chnl_hdl *hdl, enum bscd_int_type intr_type,
		       isrcb cbhdl);

/*
 * Summary:
 * This function disables a specific smart card interrupt.
 *
 * Description:
 * This function disables a specific smart card interrupt.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_DISABLE_INTR_CALLBACK).
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a ref/pointer to the smart card
 * channel handle.
 * intr_type - BICM_IntrType, Interrupt type.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 */
int chnl_dis_intr_cbisr(struct p_chnl_hdl *hdl, enum bscd_int_type intr_type);

/*
 * Summary:
 * This function enable set of related smart card interrupts for T=0, T=1 or
 * T=14.
 *
 * Description:
 * This function enable set of related smart card interrupts for T=0, T=1 or
 * T=14.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_ENABLE_INTERRUPTS).
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a ref/pointer to the smart card
 * channel handle.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 */
int chnl_ena_intr(struct p_chnl_hdl *hdl);

/*
 * Summary:
 * This function will reset the Block wait time back to whatever current channel
 * settings which was either set through ATR or chnl_set_params.
 *
 * Description:
 * This function will reset the Block wait time back to whatever current channel
 * settings which was either set through ATR or chnl_set_params.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_RESET_BLOCK_WAIT_TIMER).
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a ref/pointer to the smart card
 * channel handle.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 */

int chnl_rst_blk_wait_timer(struct p_chnl_hdl *hdl);

/*
 * Summary:
 * This function will set the Block wait time extension.
 *
 * Description:
 * This function will set the Block wait time extension.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_SET_BLOCK_WAIT_TIME_EXT).
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a ref/pointer to the smart card
 * channel handle.
 * blk_wait_time_ext_in_etu - unsigned int, block wait time extension in ETU.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 */

int chnl_set_blk_wait_time_ext(struct p_chnl_hdl *hdl,
			       unsigned int blk_wait_time_ext_in_etu);

/*
 * Summary:
 * This function will get the Block wait time.
 *
 * Description:
 * This function will get the Block wait time.
 *
 * Calling Context:
 * The function shall be called from application level (for example in
 * VxWorks or no-os) or from driver level (for example in Linux,
 * recommended ioctl: BSCD_IOCTL_GET_BLOCK_WAIT_TIME).
 *
 * Performance and Synchronization:
 * This is a synchronous function that will return when it is done.
 *
 * Input:
 * hdl - BSCD_ChannelHandle, a ref/pointer to the smart card
 * channel handle.
 * pin_blk_wait_time_in_etu -unsigned int *, place to store block wait time in
 * ETU.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 */

int chnl_get_blk_wait_time(struct p_chnl_hdl *hdl,
			   unsigned int *pin_blk_wait_time_in_etu);

/*
 * Summary:
 * This function will set the Block wait time through the passed in BWI.
 *
 * Input:
 * chnl_settings- bscd_chnl_settings, a ref/pointer to the sc settings handle
 * bwi_val - unsigned char, BWI value.
 *
 * Returns:
 * BERR_SUCCESS - success
 * To Do: Need more error code
 */

int chnl_set_blk_wait_time_integer(struct bscd_chnl_settings *chnl_settings,
				   unsigned char bwi_val);

int bscd_chnl_set_card_type_char(struct p_chnl_hdl *hdl,
				 unsigned char *historical_bytes,
				 unsigned int num_historical_bytes);

#define T1_MAX_BUF_SIZE         258 /* 3 (prologue) + 255 (max len) = 258 */

/* PCB block identifier */
#define T1_I_BLOCK              0x00
#define T1_R_BLOCK              0x80
#define T1_S_BLOCK              0xC0
#define T1_I_MORE_BLOCKS        0x20

/* Block sequence number */
#define T1_I_SEQ_SHIFT          6
#define T1_R_SEQ_SHIFT          4

/* R-block defines */
#define T1_R_IS_ERROR(pcb)      ((pcb) & 0x0F)
#define T1_R_EDC_ERROR          0x01
#define T1_R_OTHER_ERROR        0x02

/* S-block defines */
#define T1_S_RESPONSE           0x20
#define T1_S_RESYNC             0x00
#define T1_S_IFS                0x01
#define T1_S_ABORT              0x02
#define T1_S_WTX                0x03

enum {TX, RX, RESYNC};

#define BSCD_IS_APDU_RESPONSE_STATUS_FAIL(_status) (((_status) == 0x90) ? 0 : 1)

#define BSCD_APDU_PIN_VERIFY        0x0001
#define BSCD_APDU_GET_CHALLENGE     0x0002
#define BSCD_APDU_GET_RESPONSE      0x0004
#define BSCD_APDU_SELECT_FILE       0x0008
#define BSCD_APDU_READ_DATA         0x0010
#define BSCD_APDU_WRITE_DATA        0x0020
#define BSCD_APDU_DELETE_FILE       0x0040

struct apdu_s {
	unsigned char   cla;          /* Instruction class */
	unsigned char   ins;          /* Instruction code */
	unsigned char   p1;           /* Parameter 1 */
	unsigned char   p2;           /* Parameter 2 */
	unsigned short  lc;           /* Length of command data */
	unsigned short  le;           /* Length of response data */
	unsigned char   *data;        /* Response data buffer */
	unsigned char   sw[2];        /* Execution status of the command */
};


int bscd_chnl_apdu_transceive(struct p_chnl_hdl *hdl,
			      unsigned char *xmit_data,
			      unsigned long xmit_bytes,
			      unsigned char *rcv_data,
			      unsigned long *rcv_bytes,
			      unsigned long max_read_bytes
);

int chnl_detect_card_non_blk(struct p_chnl_hdl *hdl,
			     enum bscd_card_present in_ecard_present
);

int bscd_chnl_set_detect_card_cb(struct p_chnl_hdl *hdl,
				 enum bscd_card_present in_ecard_present,
				 isrcb in_callback);


/* perform PPS transaction */
int bscd_chnl_pps(struct p_chnl_hdl *hdl);

int bscd_chnl_get_atr(struct p_chnl_hdl *hdl,
		      unsigned char *rcv_data,
		      unsigned long *rcv_bytes);
#endif /* BSCD_H__ */
