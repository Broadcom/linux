/*
 * Copyright 2018 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 (GPLv2) for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 (GPLv2) along with this source code.
 */
#include <linux/delay.h>
#include <sound/initval.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "bcm-audioh.h"

#define AUDIOH_DAI_ID_TDM    0
#define AUDIOH_DAI_ID_FIFO   1

/*
 * 500 us has tested successully 10/10 times on a system that was
 * previously failing almost every time.
 */
#define NOISE_WORKAROUND_DELAY_US  500

struct audioh_priv {
	struct device *dev;

	struct regmap *regmap;
	struct regmap *crmu_regmap;

	bool analog_init_once;

	int tonegen_common_enabled_flag;
	int tonegen_freqA;
	int tonegen_freqB;
};

#define CRMU_POLL_TIMEOUT   20000  /* timeout after 20ms */
#define CRMU_POLL_PAUSE       10  /* wait 10 us betwen reads */
static int crmu_wait_for_ready(struct snd_soc_component *component,
		   unsigned int offset, u32 mask, u32 value)
{
	struct audioh_priv *audioh = snd_soc_component_get_drvdata(component);
	unsigned int temp = 0;
	int ret;

	ret = regmap_read_poll_timeout(audioh->crmu_regmap, offset,
				       temp, ((temp & mask) == value),
				       CRMU_POLL_PAUSE, CRMU_POLL_TIMEOUT);
	if (ret)
		dev_err(component->dev,
			"Timeout polling crmu reg 0x%x\n", offset);

	return ret;
}

static void crmu_update_bits(struct snd_soc_component *component,
		      unsigned int offset, u32 mask, u32 new_val)
{
	struct audioh_priv *audioh = snd_soc_component_get_drvdata(component);

	regmap_update_bits(audioh->crmu_regmap, offset, mask, new_val);
}

static void audioh_update_bits(struct snd_soc_component *component,
		      unsigned int offset, u32 mask, u32 new_val)
{
	struct audioh_priv *audioh = snd_soc_component_get_drvdata(component);

	regmap_update_bits(audioh->regmap, offset, mask, new_val);
}

static void tdm_mode_enable(struct snd_soc_component *component)
{
	u32 mask, val;
	unsigned int bits_per_frame = 128;
	unsigned int bits_per_sample = 0;  /* bits_per_sample = 0 for 32 bit */
	unsigned int valid_slots = (4 / 2);  /* divide actual in half */

	/* Configure for TDM Master mode 128 bits per frame */
	mask = CODEC_TDM_MODE_SELECT_BITCLK_MULT_MASK |
	       BIT(CODEC_TDM_MODE_SELECT_TDM) |
	       BIT(CODEC_TDM_MODE_SELECT_MASTER);
	val = (bits_per_frame << CODEC_TDM_MODE_SELECT_BITCLK_MULT) |
	      BIT(CODEC_TDM_MODE_SELECT_TDM) |
	      BIT(CODEC_TDM_MODE_SELECT_MASTER);
	audioh_update_bits(component, CODEC_TDM_MODE_SELECT, mask, val);

	/*
	 * Slave mode must always be set, also invert LR Clock to create
	 * a "normal" frame sync signal (i.e., pulse high).
	 */
	mask = BIT(CODEC_I2S_IN_CFG_SLAVE_MODE) |
	       BIT(CODEC_I2S_IN_CFG_TDM_MODE)   |
	       CODEC_I2S_IN_CFG_VALID_SLOT_MASK |
	       BIT(CODEC_I2S_IN_CFG_BITS_PER_SLOT) |
	       CODEC_I2S_IN_CFG_BITS_PER_SAMPLE_MASK |
	       BIT(CODEC_I2S_OUT_CFG_LRCK_POLARITY);

	val = BIT(CODEC_I2S_IN_CFG_SLAVE_MODE) |
	      BIT(CODEC_I2S_IN_CFG_TDM_MODE)   |
	      (valid_slots << CODEC_I2S_IN_CFG_VALID_SLOT) |
	      (0 << CODEC_I2S_IN_CFG_BITS_PER_SLOT) |
	      (bits_per_sample << CODEC_I2S_IN_CFG_BITS_PER_SAMPLE) |
	      BIT(CODEC_I2S_OUT_CFG_LRCK_POLARITY);
	audioh_update_bits(component, CODEC_IOP_IN_I2S_CFG, mask, val);
	audioh_update_bits(component, CODEC_IOP_OUT_I2S_CFG, mask, val);
}

static void tdm_mode_disable(struct snd_soc_component *component)
{
	u32 mask;

	mask = BIT(CODEC_TDM_MODE_SELECT_TDM);
	audioh_update_bits(component, CODEC_TDM_MODE_SELECT, mask, 0);
}

static void codec_fll_init(struct snd_soc_component *component)
{
	u32 mask, val;

	/* Toggle reset */
	mask = BIT(CRMU_RESCAL_CFG_RESCAL_RSTB);
	crmu_update_bits(component, CRMU_CODEC_RASCAL_CONTROL0, mask, 0);
	udelay(1);
	crmu_update_bits(component, CRMU_CODEC_RASCAL_CONTROL0, mask, mask);

	udelay(1);

	mask = BIT(CRMU_RESCAL_CFG_RESCAL_PWRDN);
	crmu_update_bits(component, CRMU_CODEC_RASCAL_CONTROL0, mask, 0);

	mask = BIT(STAT_RESCAL_DONE);
	crmu_wait_for_ready(component, CDRU_AUDIO_ANA_TEST_BUS, mask, mask);

	/* configure FLL */

	/* Step 1 : Full FLL reset */
	mask = BIT(OSC_RESET);
	crmu_update_bits(component, CRMU_CODEC_OSC_CONTROL1, mask, mask);
	udelay(10);
	crmu_update_bits(component, CRMU_CODEC_OSC_CONTROL1, mask, 0);

	/* Step 2 : reset dig FLL, set osc_powerup and vco_en to 1 */
	// Lower 8 bits = 0
	mask = BIT(OSC_PWRUP) | BIT(OSC_VCO_EN) | 0xFF;
	val  = BIT(OSC_PWRUP) | BIT(OSC_VCO_EN);
	crmu_update_bits(component, CRMU_CODEC_OSC_CONTROL1, mask, val);

	/* Step 3 : set 26M clk source as ref clk. */
	mask = BIT(OSC_EN_26M_IN);
	crmu_update_bits(component, CRMU_CODEC_OSC_CONTROL2, mask, mask);

	/* Step 4 : wait for ~200us for LDO to stabilize */
	/*  (This is nominal delay, max can be 400us) */
	udelay(600);

	mask = BIT(OSC_RESET_DIG_FLL);
	crmu_update_bits(component, CRMU_CODEC_OSC_CONTROL1, mask, mask);
	udelay(1);
	crmu_update_bits(component, CRMU_CODEC_OSC_CONTROL1, mask, 0);

	/* Step 5 :  set osc_cal_start */
	mask = BIT(OSC_CAL_START);
	crmu_update_bits(component, CRMU_CODEC_OSC_CONTROL0, mask, mask);

	/* Step 6 : wait for osc_cal_done, takes ~700us */
	mask = BIT(STAT_OSC_CAL_DONE);
	crmu_wait_for_ready(component, CRMU_CODEC_OSC_CONTROL4, mask, mask);

	/* Step 7 : wait for osc_lock, should happen shortly after cal done */
	mask = BIT(STAT_OSC_LOCK);
	crmu_wait_for_ready(component, CRMU_CODEC_OSC_CONTROL4, mask, mask);

	/* Step 8 : initiate glitchless transtion from xtal clk to VCO clk */
	mask = BIT(OSC_CLKGEN_BYP_AUD) |
	       BIT(OSC_VCO_DIGCLKOUT_EN) |
	       BIT(OSC_CLKGEN_DONT_BYP_DIG) |
	       BIT(OSC_XTAL_DIGCLKOUT_DISABLE);
	val = mask & ~BIT(OSC_CLKGEN_BYP_AUD);
	crmu_update_bits(component, CRMU_CODEC_OSC_CONTROL2, mask, val);

	/* Step 9 : clear osc_cal_start */
	mask = BIT(OSC_CAL_START);
	crmu_update_bits(component, CRMU_CODEC_OSC_CONTROL0, mask, 0);

	/* gate ref clk input to DFE */
	mask = BIT(CRMU_CODEC_CLK_GATE);
	crmu_update_bits(component, CRMU_CLK_GATE_CTRL, mask, mask);
}

static void codec_mic_bias_pwrup(struct snd_soc_component *component)
{
	u32 mask;

	/* Power up micbias for mic 1 */
	mask = BIT(MICBIAS_PWRUP);
	crmu_update_bits(component, CRMU_AUXMIC_DET_MICBIAS1_CTL, mask, mask);

	mask = BIT(MICBIAS_PWRUP);
	crmu_update_bits(component, CRMU_AUXMIC_DET_MICBIAS2_CTL, mask, mask);

	mask = BIT(0) | BIT(1);   //auxen   (2 bits wide?)
	crmu_update_bits(component,
		CRMU_AUXMIC_DET_AUXMIC_AUXMIC6, mask, BIT(0));

	/* Power up micbias for aux mic */
	mask = BIT(AUXMIC_MICBIAS_PWRUP);
	crmu_update_bits(component,
		CRMU_AUXMIC_DET_AUXMIC_AUXMIC43, mask, mask);
}

static void analog_global_init(struct snd_soc_component *component)
{
	struct audioh_priv *audioh = snd_soc_component_get_drvdata(component);
	u32 mask;

	 /* Assume codec is release from reset. CRDU_MISC_REGS (0x1100) */
	if (audioh->analog_init_once)
		return;

	audioh->analog_init_once = true;

	/* Remove the isolation for clock.  Needed for AFE to work */
	mask = BIT(CODEC_ISO_EN);
	crmu_update_bits(component, CRMU_AUD_LOGIC_POWER_CONTROL, mask, 0);

	mask = BIT(AUDIOH_GLOBAL_CLK_EN_ALIGN_SOFT_RESET);
	audioh_update_bits(component, AUDIOH_GLOBAL_CLK_EN_ALIGN, mask, mask);
	udelay(200);
	audioh_update_bits(component, AUDIOH_GLOBAL_CLK_EN_ALIGN, mask, 0);

	/* Toggle clkreset for AFE, synchronizes clks in AFE and DFE */
	mask = BIT(ANA_CTL_CLKRESET);
	audioh_update_bits(component, AUDIOH_AUDIO_ANA_CTL, mask, mask);
	audioh_update_bits(component, AUDIOH_AUDIO_ANA_CTL, mask, 0);

	/* Power up global bias */
	mask = BIT(EP_DAC_CTL_0_PUP_V21);
	audioh_update_bits(component, AUDIOH_EP_DAC_CTL_0, mask, mask);

	codec_fll_init(component);
}

static void analog_mic_enable(struct snd_soc_component *component)
{
	u32 mask;

	/* Toggle Aux Mic Reset */
	mask = BIT(CODEC_CLK_CTRL_AUX_MICRESETN);
	crmu_update_bits(component, CRMU_CODEC_CLK_CTRL, mask, mask);
	crmu_update_bits(component, CRMU_CODEC_CLK_CTRL, mask, 0);
	crmu_update_bits(component, CRMU_CODEC_CLK_CTRL, mask, mask);

	codec_mic_bias_pwrup(component);

	/* Power up ADC and PGA, release reset */
	mask = BIT(ADC_CFG2_ADC_PD) |
		BIT(ADC_CFG2_PGA_PD) |
		BIT(ADC_CFG2_ADC_RESET);
	audioh_update_bits(component, AUDIOH_ADC1_CFG2, mask, 0);
	audioh_update_bits(component, AUDIOH_ADC2_CFG2, mask, 0);
}

static void ep_afe_powerup(struct snd_soc_component *component)
{
	u32 mask, val;

	/* Toggle dac_reset and set pop-click enable */
	mask = BIT(EP_DAC_CTL_1_RESET) | BIT(EP_DAC_CTL_1_POPCLICK_CTRL_EN);
	val = BIT(EP_DAC_CTL_1_POPCLICK_CTRL_EN);
	audioh_update_bits(component, AUDIOH_EP_DAC_CTL_1, mask, val);
	udelay(10);
	audioh_update_bits(component, AUDIOH_EP_DAC_CTL_1, mask, mask);
	udelay(10);
	/* clear dac_reset and dac power down*/
	mask = BIT(EP_DAC_CTL_1_RESET) | BIT(EP_DAC_CTL_1_POWERDOWN);
	audioh_update_bits(component, AUDIOH_EP_DAC_CTL_1, mask, 0);

	udelay(20);

	mask = BIT(EP_DAC_CTL_1_PUP_DRV) | BIT(EP_DAC_CTL_1_PUP_EXT_EN);
	audioh_update_bits(component, AUDIOH_EP_DAC_CTL_1, mask, mask);

	/* Clear pup_en_ext, set final */
	mask = BIT(EP_DAC_CTL_1_FINAL_EXT) | BIT(EP_DAC_CTL_1_PUP_EXT_EN);
	val = BIT(EP_DAC_CTL_1_FINAL_EXT);
	audioh_update_bits(component, AUDIOH_EP_DAC_CTL_1, mask, val);

	udelay(50);
}

static void ep_afe_powerdown(struct snd_soc_component *component)
{
	u32 mask;

	/* set dac power down*/
	mask = BIT(EP_DAC_CTL_1_POWERDOWN);
	audioh_update_bits(component, AUDIOH_EP_DAC_CTL_1, mask, mask);
}

static void ep_afe_init(struct snd_soc_component *component)
{
	u32 mask;

	/*
	 * Disable during configuration
	 * Toggling DAC enable "cleans" up signal if DAC was previously enabled
	 * even if there is not any configuration.
	 */
	mask = BIT(AUDIOH_DAC_CTL_EP_ENABLE);
	audioh_update_bits(component, AUDIOH_DAC_CTL, mask, 0);

	/* Configuration this audio path would go here */

	/* Enable Handset (Earpiece) path */
	mask = BIT(AUDIOH_DAC_CTL_EP_ENABLE);
	audioh_update_bits(component, AUDIOH_DAC_CTL, mask, mask);

	/*
	 * Toggle the enable a 2nd time.
	 * This is used to clear up a loud noise that occasionally occurs
	 * on the interface.
	 */
	udelay(NOISE_WORKAROUND_DELAY_US);
	audioh_update_bits(component, AUDIOH_DAC_CTL, mask, 0);
	audioh_update_bits(component, AUDIOH_DAC_CTL, mask, mask);
}

static void mono_voip_headset_powerup(struct snd_soc_component *component)
{
	u32 mask, val;

	/* Enable pop-click enable */
	mask = BIT(EP_DAC2_CTL_1_POPCLICK_CTRL_EN);
	audioh_update_bits(component, AUDIOH_EP_DAC2_CTL_1, mask, mask);

	/* clear power down*/
	mask = BIT(EP_DAC2_CTL_1_POWERDOWN);
	audioh_update_bits(component, AUDIOH_EP_DAC2_CTL_1, mask, 0);

	udelay(20);

	mask = BIT(EP_DAC2_CTL_1_PUP_DRV) | BIT(EP_DAC2_CTL_1_PUP_EXT_EN);
	audioh_update_bits(component, AUDIOH_EP_DAC2_CTL_1, mask, mask);

	/* Clear pup_en_ext, set final */
	mask = BIT(EP_DAC2_CTL_1_FINAL_EXT) | BIT(EP_DAC2_CTL_1_PUP_EXT_EN);
	val = BIT(EP_DAC2_CTL_1_FINAL_EXT);
	audioh_update_bits(component, AUDIOH_EP_DAC2_CTL_1, mask, val);

	udelay(50);
}

static void mono_voip_headset_powerdown(struct snd_soc_component *component)
{
	u32 mask;

	/* set dac power down*/
	mask = BIT(EP_DAC2_CTL_1_POWERDOWN);
	audioh_update_bits(component, AUDIOH_EP_DAC2_CTL_1, mask, mask);
}

static void ihf_afe_powerup(struct snd_soc_component *component)
{
	u32 mask;

	/* Toggle reset */
	mask = BIT(AUDIOH_IHF_CTL_CLKRESET);
	audioh_update_bits(component, AUDIOH_IHF_CTL, mask, 0);
	udelay(10);
	audioh_update_bits(component, AUDIOH_IHF_CTL, mask, mask);
	udelay(10);
	audioh_update_bits(component, AUDIOH_IHF_CTL, mask, 0);

	mask = BIT(AUDIOH_IHF_CTL_PUP_L) |
	       BIT(AUDIOH_IHF_CTL_DRV_BIAS_PUP_L) |
	       BIT(AUDIOH_IHF_CTL_DAC_PU_1P8_L);
	audioh_update_bits(component, AUDIOH_IHF_CTL, mask, mask);
}

static void ihf_afe_enable(struct snd_soc_component *component)
{
	u32 mask;

	ihf_afe_powerup(component);

	/*
	 * Disable during configuration
	 * Toggling DAC enable "cleans" up signal if DAC was previously enabled
	 * even if there is not any configuration.
	 */
	mask = BIT(AUDIOH_DAC_CTL_IHF_ENABLE);
	audioh_update_bits(component, AUDIOH_DAC_CTL, mask, 0);

	/*
	 * There is currently not any configuration for this analog block,
	 * this comment is a placehold to indicate that the config should be
	 * done between the the disable and enable stages.
	 */

	/* Enable after config is done */
	mask = BIT(AUDIOH_DAC_CTL_IHF_ENABLE);
	audioh_update_bits(component, AUDIOH_DAC_CTL, mask, mask);

	/*
	 * Toggle the enable a 2nd time.
	 * This is used to clear up a loud noise that occasionally occurs
	 * on the interface.
	 */
	udelay(NOISE_WORKAROUND_DELAY_US);
	audioh_update_bits(component, AUDIOH_DAC_CTL, mask, 0);
	audioh_update_bits(component, AUDIOH_DAC_CTL, mask, mask);
}

static void headset_afe_powerup(struct snd_soc_component *component)
{
	u32 mask, cp_mask;

	mask = BIT(HS_DAC_CTL_CLKRESET);
	audioh_update_bits(component, AUDIOH_HS_DAC_CTL, mask, 0);

	/* Set for max current */
	mask = HS_DRV_TRIM_DRV_AMP_MASK;
	audioh_update_bits(component, AUDIOH_HS_DRV_TRIM, mask, mask);

	/* Set charge pump for max voltage */
	cp_mask = HS_CP_CTRL_VNC_MASK | HS_CP_CTRL_VPC_MASK;
	audioh_update_bits(component, AUDIOH_HS_CP_CTRL, cp_mask, cp_mask);

	/* Power up */
	mask = BIT(HS_DRV_PUP1_PUP_L) |
	       BIT(HS_DRV_PUP1_PUP_R) |
	       BIT(HS_DRV_PUP1_SC_PUP_L) |
	       BIT(HS_DRV_PUP1_SC_PUP_R);
	audioh_update_bits(component, AUDIOH_HS_DRV_PUP1, mask, mask);

	msleep(30);

	/* Set charge pump for min voltage */
	audioh_update_bits(component, AUDIOH_HS_CP_CTRL, cp_mask, 0);

	udelay(50);
}

static void headset_afe_enable(struct snd_soc_component *component)
{
	u32 mask;

	headset_afe_powerup(component);

	/*
	 * Disable during configuration
	 * Toggling DAC enable "cleans" up signal if DAC was previously enabled
	 * even if there is not any configuration.
	 */
	mask = BIT(AUDIOH_DAC_CTL_HSR_ENABLE) | BIT(AUDIOH_DAC_CTL_HSL_ENABLE);
	audioh_update_bits(component, AUDIOH_DAC_CTL, mask, 0);

	/* Configuration of Headset path would go here (DEM stuff?) */

	/* enable Stereo Headset Left and Right */
	mask = BIT(AUDIOH_DAC_CTL_HSR_ENABLE) | BIT(AUDIOH_DAC_CTL_HSL_ENABLE);
	audioh_update_bits(component, AUDIOH_DAC_CTL, mask, mask);

	/*
	 * Toggle the enable a 2nd time.
	 * This is used to clear up a loud noise that occasionally occurs
	 * on the interface.
	 */
	udelay(NOISE_WORKAROUND_DELAY_US);
	audioh_update_bits(component, AUDIOH_DAC_CTL, mask, 0);
	audioh_update_bits(component, AUDIOH_DAC_CTL, mask, mask);
}

static void clear_capture_fifos(struct snd_soc_component *component)
{
	u32 mask1, mask2;

	mask1 = BIT(CAPTURE_FIFO1_CLEAR) | BIT(CAPTURE_FIFO2_CLEAR);
	mask2 = BIT(CAPTURE_FIFO3_CLEAR) | BIT(CAPTURE_FIFO4_CLEAR);

	audioh_update_bits(component, AUDIOH_MIC12_FIFO_CTRL, mask1, mask1);
	audioh_update_bits(component, AUDIOH_MIC34_FIFO_CTRL, mask2, mask2);
	udelay(100);
	audioh_update_bits(component, AUDIOH_MIC12_FIFO_CTRL, mask1, 0);
	audioh_update_bits(component, AUDIOH_MIC34_FIFO_CTRL, mask2, 0);
}

#define TDM_PLAYBACK_PATH_MASK  (BIT(CODEC_TDM_PLAYOUT_PATH_EP_ENABLE) | \
				BIT(CODEC_TDM_PLAYOUT_PATH_IHF_ENABLE) | \
				BIT(CODEC_TDM_PLAYOUT_PATH_STEREO_HS_ENABLE) | \
				BIT(CODEC_TDM_PLAYOUT_PATH_GLOBAL_ENABLE))

static void enable_tx_path(struct snd_soc_component *component, bool use_tdm)
{
	u32 mask, val;

	if (use_tdm) {
		tdm_mode_enable(component);

		mask = TDM_PLAYBACK_PATH_MASK;
		val = mask;
		audioh_update_bits(component,
			CODEC_TDM_PLAYOUT_PATH_ENABLE, mask, val);
	}

	mask = BIT(AUDIOH_DAC_CTL_HSR_ENABLE) |
		BIT(AUDIOH_DAC_CTL_HSL_ENABLE) |
		BIT(AUDIOH_DAC_CTL_IHF_ENABLE) |
		BIT(AUDIOH_DAC_CTL_EP_ENABLE);
	val = mask; /* Enable All channels */
	audioh_update_bits(component, AUDIOH_DAC_CTL, mask, val);

	if (use_tdm) {
		mask = BIT(IOP_IN_I2S_STREAM_CFG_ENABLE);
		audioh_update_bits(component,
			CODEC_IOP_IN_I2S_STREAM_CFG, mask, mask);
	}
}

static void disable_tx_path(struct snd_soc_component *component, bool use_tdm)
{
	u32 mask;

	if (use_tdm) {
		mask = BIT(IOP_IN_I2S_STREAM_CFG_ENABLE);
		audioh_update_bits(component,
			CODEC_IOP_IN_I2S_STREAM_CFG, mask, 0);

		mask = TDM_PLAYBACK_PATH_MASK;
		audioh_update_bits(component,
			CODEC_TDM_PLAYOUT_PATH_ENABLE, mask, 0);

		audioh_update_bits(component,
			CODEC_TDM_PLAYOUT_PATH_SW_RESET, 1, 1);
		audioh_update_bits(component,
			CODEC_TDM_PLAYOUT_PATH_SW_RESET, 1, 0);
	}
}

#define TDM_CAPTURE_SLOT_MASK  (BIT(CODEC_TDM_CAPTURE_SLOT0_ACTIVE) | \
				BIT(CODEC_TDM_CAPTURE_SLOT1_ACTIVE) | \
				BIT(CODEC_TDM_CAPTURE_SLOT2_ACTIVE) | \
				BIT(CODEC_TDM_CAPTURE_SLOT3_ACTIVE))

#define TDM_CAPTURE_PATH_MASK  (BIT(CODEC_TDM_CAPTURE_PATH_ENABLE_GLOBAL) | \
				BIT(CODEC_TDM_CAPTURE_PATH_ENABLE_FIFO1)  | \
				BIT(CODEC_TDM_CAPTURE_PATH_ENABLE_FIFO2)  | \
				BIT(CODEC_TDM_CAPTURE_PATH_ENABLE_FIFO3)  | \
				BIT(CODEC_TDM_CAPTURE_PATH_ENABLE_FIFO4))

static void enable_rx_path(struct snd_soc_component *component, bool use_tdm)
{
	u32 mask, val;

	if (use_tdm) {
		tdm_mode_enable(component);

		mask = TDM_CAPTURE_SLOT_MASK;
		val = mask;
		audioh_update_bits(component,
			CODEC_TDM_CAPTURE_SLOTS_ACTIVE, mask, val);

		mask = TDM_CAPTURE_PATH_MASK;
		val = mask;
		audioh_update_bits(component,
			CODEC_TDM_CAPTURE_PATH_ENABLE, mask, val);
	}

	mask = BIT(AUDIOH_ADC_CTL_MIC1_ENABLE) |
		BIT(AUDIOH_ADC_CTL_MIC2_ENABLE) |
		BIT(AUDIOH_ADC_CTL_MIC3_ENABLE) |
		BIT(AUDIOH_ADC_CTL_MIC4_ENABLE);
	val = mask;  /* Enable All channels */
	audioh_update_bits(component, AUDIOH_ADC_CTL, mask, val);

	if (use_tdm) {
		mask = BIT(IOP_OUT_I2S_STREAM_CFG_ENABLE);
		audioh_update_bits(component,
			CODEC_IOP_OUT_I2S_STREAM_CFG, mask, mask);
	}
}

static void disable_rx_path(struct snd_soc_component *component, bool use_tdm)
{
	u32 mask;

	if (use_tdm) {
		mask = BIT(IOP_OUT_I2S_STREAM_CFG_ENABLE);
		audioh_update_bits(component,
			CODEC_IOP_OUT_I2S_STREAM_CFG, mask, 0);

		mask = TDM_CAPTURE_PATH_MASK;
		audioh_update_bits(component,
			CODEC_TDM_CAPTURE_PATH_ENABLE, mask, 0);

		mask = TDM_CAPTURE_SLOT_MASK;
		audioh_update_bits(component,
			CODEC_TDM_CAPTURE_SLOTS_ACTIVE, mask, 0);

		audioh_update_bits(component,
			CODEC_TDM_CAPTURE_PATH_SW_RESET, 1, 1);
		audioh_update_bits(component,
			CODEC_TDM_CAPTURE_PATH_SW_RESET, 1, 0);
	}
}

static int audioh_trigger(struct snd_pcm_substream *substream,
		int cmd,
		struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	bool playback;
	bool use_tdm;

	dev_dbg(dai->dev, "%s: dai: %d Enter.\n", __func__, dai->id);

	playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	use_tdm = false;
	if (dai->id == AUDIOH_DAI_ID_TDM)
		use_tdm = true;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (playback) {
			enable_tx_path(component, use_tdm);
		} else {
			clear_capture_fifos(component);
			enable_rx_path(component, use_tdm);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (playback)
			disable_tx_path(component, use_tdm);
		else
			disable_rx_path(component, use_tdm);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int audioh_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	dev_dbg(dai->dev, "%s: dai: %d Enter.\n", __func__, dai->id);

	/* AudioH tdm interface only supports DSP_A mode */
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_DSP_A) {
		dev_err(dai->dev, "%s: only support DSP_A mode.\n", __func__);
		return -EINVAL;
	}

	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBM_CFM) {
		dev_err(dai->dev, "%s: only support master mode\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int audioh_dai_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;

	dev_dbg(dai->dev, "%s: dai: %d Enter.\n", __func__, dai->id);

	/* These modes are mutually exclusive */
	if (dai->id == AUDIOH_DAI_ID_TDM)
		tdm_mode_enable(component);
	else
		tdm_mode_disable(component);

	return 0;
}

static void audioh_dai_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	dev_dbg(dai->dev, "%s: dai: %d Enter.\n", __func__, dai->id);
}

static const struct snd_soc_dai_ops audioh_dai_ops = {
	.startup  = audioh_dai_startup,
	.shutdown = audioh_dai_shutdown,
	.set_fmt = audioh_dai_set_fmt,
	.trigger = audioh_trigger,
};

static struct snd_soc_dai_driver audioh_dais[] = {
{
	.name = "bcm-audioh-tdm-dai",
	.id = AUDIOH_DAI_ID_TDM,
	.playback = {
		.stream_name = "TDM Playback",
		.channels_min = 4,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.stream_name = "TDM Capture",
		.channels_min = 4,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &audioh_dai_ops,
},
{
	.name = "bcm-audioh-fifo-dai",
	.id = AUDIOH_DAI_ID_FIFO,
	.playback = {
		.stream_name = "FIFO Playback",
		.channels_min = 4,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.stream_name = "FIFO Capture",
		.channels_min = 4,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &audioh_dai_ops,
}
};

/* Define a pair of muxes for selecting the source for each ADC */
static const char * const amicsel_texts[] = { "main mic", "aux mic" };

static const struct soc_enum audioh_amicsel_enum[] = {
	SOC_ENUM_SINGLE(AUDIOH_ADC1_CFG, ADC_CFG_MIC_SEL,
			ARRAY_SIZE(amicsel_texts), amicsel_texts),
	SOC_ENUM_SINGLE(AUDIOH_ADC2_CFG, ADC_CFG_MIC_SEL,
			ARRAY_SIZE(amicsel_texts), amicsel_texts)
};

static const struct snd_kcontrol_new amic1_mux_controls =
	SOC_DAPM_ENUM("Route", audioh_amicsel_enum[0]);

static const struct snd_kcontrol_new amic2_mux_controls =
	SOC_DAPM_ENUM("Route", audioh_amicsel_enum[1]);

/* Define muxes for selecting the source for each capture path */
static const char * const audioh_micpath_texts[] = {
	"None", "adc1", "adc2", "dmic1", "dmic2", "dmic3", "dmic4", "Loopback"
};

static const struct soc_enum audioh_micpath_enum[] = {
	SOC_ENUM_SINGLE(AUDIOH_MIC_SELECT, AUDIOH_MIC_SELECT_FIFO1,
		ARRAY_SIZE(audioh_micpath_texts), audioh_micpath_texts),
	SOC_ENUM_SINGLE(AUDIOH_MIC_SELECT, AUDIOH_MIC_SELECT_FIFO2,
		ARRAY_SIZE(audioh_micpath_texts), audioh_micpath_texts),
	SOC_ENUM_SINGLE(AUDIOH_MIC_SELECT, AUDIOH_MIC_SELECT_FIFO3,
		ARRAY_SIZE(audioh_micpath_texts), audioh_micpath_texts),
	SOC_ENUM_SINGLE(AUDIOH_MIC_SELECT, AUDIOH_MIC_SELECT_FIFO4,
		ARRAY_SIZE(audioh_micpath_texts), audioh_micpath_texts)
};

static const struct snd_kcontrol_new micpath1_mux_controls =
	SOC_DAPM_ENUM("Route", audioh_micpath_enum[0]);

static const struct snd_kcontrol_new micpath2_mux_controls =
	SOC_DAPM_ENUM("Route", audioh_micpath_enum[1]);

static const struct snd_kcontrol_new micpath3_mux_controls =
	SOC_DAPM_ENUM("Route", audioh_micpath_enum[2]);

static const struct snd_kcontrol_new micpath4_mux_controls =
	SOC_DAPM_ENUM("Route", audioh_micpath_enum[3]);

/* Define demux for selecting the EP output path*/
static const char * const mono_ep_path_texts[] = {
	"None", "Handset", "Headset"
};
static SOC_ENUM_SINGLE_VIRT_DECL(ep_path_enum, mono_ep_path_texts);

static const struct snd_kcontrol_new ep_path_ctrl =
	SOC_DAPM_ENUM("EP Path Select", ep_path_enum);

/* Sidetone */
static const char * const audioh_sidetone_src[] = {
	"None", "Capture1", "Capture2", "Capture3", "Capture4"
};

static SOC_ENUM_SINGLE_DECL(audioh_sidetone_enum,
	AUDIOH_MIC_SELECT, AUDIOH_MIC_SELECT_SDT, audioh_sidetone_src);

static const struct snd_kcontrol_new audioh_sidetone_mux_controls =
	SOC_DAPM_ENUM("Route", audioh_sidetone_enum);

static int dmic_1and2_clk_set(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component;
	u32 mask;

	component = snd_soc_dapm_to_component(w->dapm);

	/* Clear "use_dved1_clk_for_dmic1" */
	mask = BIT(USE_DVED_CLK_FOR_DMIC1);
	audioh_update_bits(component, AUDIOH_DVED_CTL1, mask, 0);

	audioh_update_bits(component, AUDIOH_MIC_CLOCK_SELECT,
		DMIC12_CLK_SELECT_MASK,
		(DMIC_CLK_SELECT_3250 << DMIC12_CLK_SELECT));

	return 0;
}

static int dmic_3and4_clk_set(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component;
	u32 mask;

	component = snd_soc_dapm_to_component(w->dapm);

	/* Clear "use_dved2_clk_for_dmic2" */
	mask = BIT(USE_DVED_CLK_FOR_DMIC2);
	audioh_update_bits(component, AUDIOH_DVED_CTL1, mask, 0);

	audioh_update_bits(component, AUDIOH_MIC_CLOCK_SELECT,
		DMIC34_CLK_SELECT_MASK,
		(DMIC_CLK_SELECT_3250 << DMIC34_CLK_SELECT));

	return 0;
}

static int ep_dac1_ev(struct snd_soc_dapm_widget *w,
		      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component;

	component = snd_soc_dapm_to_component(w->dapm);

	/* Handle the two power events we care about, ignore others */
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		ep_afe_powerup(component);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		ep_afe_powerdown(component);
		break;
	default:
		break;
	}

	return 0;
}

static int ep_dac2_ev(struct snd_soc_dapm_widget *w,
		      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component;

	component = snd_soc_dapm_to_component(w->dapm);

	/* Handle the two power events we care about, ignore others */
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mono_voip_headset_powerup(component);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		mono_voip_headset_powerdown(component);
		break;
	default:
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget audioh_dapm_widgets[] = {
	/* Microphone Inputs */
	SND_SOC_DAPM_INPUT("AMIC1"),
	SND_SOC_DAPM_INPUT("AUXMIC1"),
	SND_SOC_DAPM_INPUT("AMIC3"),
	SND_SOC_DAPM_INPUT("AUXMIC2_UnusedMaybe"),
	SND_SOC_DAPM_INPUT("DMIC1"),
	SND_SOC_DAPM_INPUT("DMIC2"),
	SND_SOC_DAPM_INPUT("DMIC3"),
	SND_SOC_DAPM_INPUT("DMIC4"),

	SND_SOC_DAPM_SUPPLY("DMIC_1_2 Power", AUDIOH_DMIC_CLK_GATE,
		DMIC_CLK_GATE_1AND2, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DMIC_3_4 Power", AUDIOH_DMIC_CLK_GATE,
		DMIC_CLK_GATE_3AND4, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("DMIC_1_2 Clock", SND_SOC_NOPM, 0, 0,
			dmic_1and2_clk_set, SND_SOC_DAPM_PRE_PMU),
	SND_SOC_DAPM_SUPPLY("DMIC_3_4 Clock", SND_SOC_NOPM, 0, 0,
			dmic_3and4_clk_set, SND_SOC_DAPM_PRE_PMU),

	/* Analog Mic Muxes */
	SND_SOC_DAPM_MUX("ADC1 Mux", SND_SOC_NOPM,
			 0, 0, &amic1_mux_controls),
	SND_SOC_DAPM_MUX("ADC2 Mux", SND_SOC_NOPM,
			 0, 0, &amic2_mux_controls),

	/* Capture Path Muxes */
	SND_SOC_DAPM_MUX("Capture1 Mux", SND_SOC_NOPM,
			 0, 0, &micpath1_mux_controls),
	SND_SOC_DAPM_MUX("Capture2 Mux", SND_SOC_NOPM,
			 0, 0, &micpath2_mux_controls),
	SND_SOC_DAPM_MUX("Capture3 Mux", SND_SOC_NOPM,
			 0, 0, &micpath3_mux_controls),
	SND_SOC_DAPM_MUX("Capture4 Mux", SND_SOC_NOPM,
			 0, 0, &micpath4_mux_controls),

	/* Sidetone */
	SND_SOC_DAPM_MUX("Sidetone Mux", SND_SOC_NOPM, 0, 0,
			&audioh_sidetone_mux_controls),

	/* Digital inputs (TDM) */
	SND_SOC_DAPM_AIF_IN("TDM_PLAY_Slot0", "TDM Playback", 0,
				SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("TDM_PLAY_Slot1", "TDM Playback", 1,
				SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("TDM_PLAY_Slot2", "TDM Playback", 2,
				SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("TDM_PLAY_Slot3", "TDM Playback", 3,
				SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_IN("FIFO_PLAY_Slot0", "FIFO Playback", 0,
				SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("FIFO_PLAY_Slot1", "FIFO Playback", 1,
				SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("FIFO_PLAY_Slot2", "FIFO Playback", 2,
				SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("FIFO_PLAY_Slot3", "FIFO Playback", 3,
				SND_SOC_NOPM, 0, 0),

	/* Digital output (TDM) */
	SND_SOC_DAPM_AIF_OUT("TDM_CAP_Slot0", "TDM Capture", 0,
				SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TDM_CAP_Slot1", "TDM Capture", 1,
				SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TDM_CAP_Slot2", "TDM Capture", 2,
				SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TDM_CAP_Slot3", "TDM Capture", 3,
				SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_OUT("FIFO_CAP_Slot0", "FIFO Capture", 0,
				SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("FIFO_CAP_Slot1", "FIFO Capture", 1,
				SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("FIFO_CAP_Slot2", "FIFO Capture", 2,
				SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("FIFO_CAP_Slot3", "FIFO Capture", 3,
				SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_ADC("ADC1", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC2", NULL, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_PGA("ADC1 PGA", SND_SOC_NOPM, 0, 1, NULL, 0),
	SND_SOC_DAPM_PGA("ADC2 PGA", SND_SOC_NOPM, 0, 1, NULL, 0),

	SND_SOC_DAPM_DAC("Headset DAC Left", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("Headset DAC Right", NULL, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_DAC_E("Earpiece DAC1", NULL, SND_SOC_NOPM, 0, 0,
			   ep_dac1_ev,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_DAC_E("Earpiece DAC2", NULL, SND_SOC_NOPM, 0, 0,
			   ep_dac2_ev,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_DAC("IHF DAC1", NULL, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_DEMUX("EP Output Select", SND_SOC_NOPM, 0, 0,
			   &ep_path_ctrl),

	/* Analog outputs */
	SND_SOC_DAPM_OUTPUT("HS_OUT_L"),
	SND_SOC_DAPM_OUTPUT("HS_OUT_R"),
	SND_SOC_DAPM_OUTPUT("LSPK_OUT"),
	SND_SOC_DAPM_OUTPUT("EP_OUT"),
	SND_SOC_DAPM_OUTPUT("EP_OUT2"),
};

static const struct snd_soc_dapm_route audioh_dapm_routes[] = {
	/* -----  Playback Path  ------------------------------------------ */
	{"HS_OUT_L", NULL, "Headset DAC Left"},
	{"Headset DAC Left", NULL, "TDM_PLAY_Slot0"},
	{"Headset DAC Left", NULL, "FIFO_PLAY_Slot0"},

	{"HS_OUT_R", NULL, "Headset DAC Right"},
	{"Headset DAC Right", NULL, "TDM_PLAY_Slot1"},
	{"Headset DAC Right", NULL, "FIFO_PLAY_Slot1"},

	{"LSPK_OUT", NULL, "IHF DAC1"},
	{"IHF DAC1", NULL, "TDM_PLAY_Slot2"},

	{"EP_OUT",  NULL, "Earpiece DAC1"},
	{"EP_OUT2", NULL, "Earpiece DAC2"},
	{"Earpiece DAC1", "Handset", "EP Output Select"},
	{"Earpiece DAC2", "Headset", "EP Output Select"},
	{"EP Output Select", NULL, "TDM_PLAY_Slot3"},
	{"EP Output Select", NULL, "FIFO_PLAY_Slot3"},

	/* -----  Sidetone Path  ------------------------------------------ */
	{"Sidetone Mux", "Capture1", "Capture1 Mux"},
	{"Sidetone Mux", "Capture2", "Capture2 Mux"},
	{"Sidetone Mux", "Capture3", "Capture3 Mux"},
	{"Sidetone Mux", "Capture4", "Capture4 Mux"},

	/* -----  Capture Path  ------------------------------------------ */
	{"ADC1 Mux", "main mic", "AMIC1"},
	{"ADC1 Mux", "aux mic", "AUXMIC1"},

	{"ADC2 Mux", "main mic", "AMIC3"},
	{"ADC2 Mux", "aux mic", "AUXMIC2_UnusedMaybe"},

	{"ADC1 PGA", NULL, "ADC1 Mux"},
	{"ADC2 PGA", NULL, "ADC2 Mux"},

	{"ADC1", NULL, "ADC1 PGA"},
	{"ADC2", NULL, "ADC2 PGA"},

	{"Capture1 Mux", "adc1", "ADC1"},
	{"Capture1 Mux", "adc2", "ADC2"},
	{"Capture1 Mux", "dmic1", "DMIC1"},
	{"Capture1 Mux", "dmic2", "DMIC2"},
	{"Capture1 Mux", "dmic3", "DMIC3"},
	{"Capture1 Mux", "dmic4", "DMIC4"},

	{"Capture2 Mux", "adc1", "ADC1"},
	{"Capture2 Mux", "adc2", "ADC2"},
	{"Capture2 Mux", "dmic1", "DMIC1"},
	{"Capture2 Mux", "dmic2", "DMIC2"},
	{"Capture2 Mux", "dmic3", "DMIC3"},
	{"Capture2 Mux", "dmic4", "DMIC4"},

	{"Capture3 Mux", "adc1", "ADC1"},
	{"Capture3 Mux", "adc2", "ADC2"},
	{"Capture3 Mux", "dmic1", "DMIC1"},
	{"Capture3 Mux", "dmic2", "DMIC2"},
	{"Capture3 Mux", "dmic3", "DMIC3"},
	{"Capture3 Mux", "dmic4", "DMIC4"},

	{"Capture4 Mux", "adc1", "ADC1"},
	{"Capture4 Mux", "adc2", "ADC2"},
	{"Capture4 Mux", "dmic1", "DMIC1"},
	{"Capture4 Mux", "dmic2", "DMIC2"},
	{"Capture4 Mux", "dmic3", "DMIC3"},
	{"Capture4 Mux", "dmic4", "DMIC4"},

	{"TDM_CAP_Slot0", NULL, "Capture1 Mux"},
	{"TDM_CAP_Slot1", NULL, "Capture2 Mux"},
	{"TDM_CAP_Slot2", NULL, "Capture3 Mux"},
	{"TDM_CAP_Slot3", NULL, "Capture4 Mux"},

	{"FIFO_CAP_Slot0", NULL, "Capture1 Mux"},
	{"FIFO_CAP_Slot1", NULL, "Capture2 Mux"},
	{"FIFO_CAP_Slot2", NULL, "Capture3 Mux"},
	{"FIFO_CAP_Slot3", NULL, "Capture4 Mux"},

	{ "DMIC1", NULL, "DMIC_1_2 Power" },
	{ "DMIC2", NULL, "DMIC_1_2 Power" },
	{ "DMIC3", NULL, "DMIC_3_4 Power" },
	{ "DMIC4", NULL, "DMIC_3_4 Power" },

	{ "DMIC1", NULL, "DMIC_1_2 Clock" },
	{ "DMIC2", NULL, "DMIC_1_2 Clock" },
	{ "DMIC3", NULL, "DMIC_3_4 Clock" },
	{ "DMIC4", NULL, "DMIC_3_4 Clock" },
};

#define TONEGEN_SAMPLE_RATE  48000
/*
 * (Freq/48000) * 2^26
 * (Freq/375) * 2^19
 * (Freq/375) * 2^16 * 2*3
 */
static int tonegen_calc_coef(unsigned int freq)
{
	unsigned int coef;

	if (freq > (TONEGEN_SAMPLE_RATE/2))
		return -EINVAL;

	coef = ((freq << 16) / 375) * 8;
	return coef;
}

static int tonegen_common_enable_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component;
	struct audioh_priv *audioh;

	component = snd_soc_kcontrol_component(kcontrol);
	audioh = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = audioh->tonegen_common_enabled_flag;
	return 0;
}

static int tonegen_common_enable_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component;
	struct audioh_priv *audioh;
	u32 mask;
	unsigned int coef;

	component = snd_soc_kcontrol_component(kcontrol);
	audioh = snd_soc_component_get_drvdata(component);

	audioh->tonegen_common_enabled_flag = ucontrol->value.integer.value[0];

	if (audioh->tonegen_common_enabled_flag) {
		/* Enable both tone generator */
		mask = BIT(DAC_TONEGEN_Enable0) | BIT(DAC_TONEGEN_Enable1);
		audioh_update_bits(component,
			AUDIOH_DAC_TONEGEN_CTRL, mask, mask);

		coef = tonegen_calc_coef(audioh->tonegen_freqA);
		if (coef < 0)
			return coef;
		mask = TONEGEN_PHASE_STEP_MASK;
		audioh_update_bits(component,
			AUDIOH_DAC_TONEGEN_PHASE_STEP_0, mask, coef);

		coef = tonegen_calc_coef(audioh->tonegen_freqB);
		if (coef < 0)
			return coef;
		audioh_update_bits(component,
			AUDIOH_DAC_TONEGEN_PHASE_STEP_1, mask, coef);

		mask = GENMASK(23, 0);
		audioh_update_bits(component,
		    AUDIOH_DAC_TONEGEN_X_INITIAL_0, mask, 3565787);
		audioh_update_bits(component,
		    AUDIOH_DAC_TONEGEN_X_INITIAL_1, mask, 3565787);
	} else {
		mask = BIT(DAC_TONEGEN_Enable0) | BIT(DAC_TONEGEN_Enable1);
		audioh_update_bits(component, AUDIOH_DAC_TONEGEN_CTRL, mask, 0);
	}

	return 0;
}

static int tonegen_freqA_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component;
	struct audioh_priv *audioh;

	component = snd_soc_kcontrol_component(kcontrol);
	audioh = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = audioh->tonegen_freqA;
	return 0;
}

static int tonegen_freqA_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component;
	struct audioh_priv *audioh;

	component = snd_soc_kcontrol_component(kcontrol);
	audioh = snd_soc_component_get_drvdata(component);

	audioh->tonegen_freqA = ucontrol->value.integer.value[0];
	return 0;
}

static int tonegen_freqB_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component;
	struct audioh_priv *audioh;

	component = snd_soc_kcontrol_component(kcontrol);
	audioh = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = audioh->tonegen_freqB;
	return 0;
}

static int tonegen_freqB_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component;
	struct audioh_priv *audioh;

	component = snd_soc_kcontrol_component(kcontrol);
	audioh = snd_soc_component_get_drvdata(component);

	audioh->tonegen_freqB = ucontrol->value.integer.value[0];
	return 0;
}

static void tonegen_set_scale(struct snd_soc_component *component,
				u32 reg, unsigned int val)
{
	u32 enable_mask;

	enable_mask = BIT(TONEGEN_CTRL_ENABLE);
	audioh_update_bits(component, reg, enable_mask, 0);  /* Disable first */
	audioh_update_bits(component, reg,
		TONEGEN_CTRL_SCALE_MASK, (val << TONEGEN_CTRL_SCALE));
	audioh_update_bits(component, reg, enable_mask, enable_mask);
}

static int tonegen_scale_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component;
	struct soc_mixer_control *mixer_ctrl =
		(struct soc_mixer_control *) kcontrol->private_value;
	unsigned int reg = mixer_ctrl->reg;
	unsigned int val;

	component = snd_soc_kcontrol_component(kcontrol);

	val = ucontrol->value.integer.value[0];
	tonegen_set_scale(component, reg, val);
	return 0;
}

static int tonegen_scale_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component;
	struct soc_mixer_control *mixer_ctrl =
		(struct soc_mixer_control *) kcontrol->private_value;
	unsigned int reg = mixer_ctrl->reg;
	unsigned int shift = mixer_ctrl->shift;
	unsigned int val;

	component = snd_soc_kcontrol_component(kcontrol);
	val = snd_soc_component_read32(component, reg) >> shift;
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static const DECLARE_TLV_DB_SCALE(adcpga_tlv, 0, 300, 0);

/*
 * The AudioH ADC PGA gain control is unusual.
 * Value of 6 and 7 are not used.
 *    0x0 : 0dB
 *    0x1 : 3dB
 *    0x2 : 6dB
 *    0x3 : 9dB
 *    0x4 : 12dB
 *    0x5 : 15dB
 *    0x6 : Undef
 *    0x7 : Undef
 *    0x8 : 18dB
 *    0x9 : 21dB
 *    0xA : 24dB
 *    0xB : 27dB
 *    0xC : 30dB
 *    0xD : 33dB
 *    0xE : 36dB
 *    0xF : 39dB
 * Use the normal soc volume macros and function, except put a wrapper around
 * the controls to handle the discontinuity in the range.
 */
static int custom_put_volsw(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	if (ucontrol->value.integer.value[0] >= 6)
		ucontrol->value.integer.value[0] += 2;

	return snd_soc_put_volsw(kcontrol, ucontrol);
}

static int custom_get_volsw(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	int ret;

	ret = snd_soc_get_volsw(kcontrol, ucontrol);
	if (ret)
		return ret;

	if (ucontrol->value.integer.value[0] >= 8)
		ucontrol->value.integer.value[0] -= 2;

	return 0;
}

#define AUDIOH_SINGLE_TLV(xname, reg, shift, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .get = custom_get_volsw,\
	.put = custom_put_volsw, \
	.private_value = SOC_SINGLE_VALUE(reg, shift, max, invert, 0) }

static const struct snd_kcontrol_new audioh_control[] = {
	SOC_SINGLE("Stereo Headset Mute Switch",
			AUDIOH_DAC_CTL, AUDIOH_DAC_CTL_HS_MUTE, 1, 0),
	SOC_SINGLE("IHF Mute Switch",
			AUDIOH_DAC_CTL, AUDIOH_DAC_CTL_IHF_MUTE, 1, 0),
	SOC_SINGLE("EP Mute Switch",
			AUDIOH_DAC_CTL, AUDIOH_DAC_CTL_EP_MUTE, 1, 0),

	SOC_SINGLE("Stereo Headset Sidetone Switch",
		   AUDIOH_DAC_CTL, AUDIOH_DAC_CTL_HS_SIDETONE_EN, 1, 0),
	SOC_SINGLE("IHF Sidetone Switch",
		   AUDIOH_DAC_CTL, AUDIOH_DAC_CTL_IHF_SIDETONE_EN, 1, 0),
	SOC_SINGLE("EP Sidetone Switch",
		   AUDIOH_DAC_CTL, AUDIOH_DAC_CTL_EP_SIDETONE_EN, 1, 0),
	SOC_SINGLE("Sidetone Switch",
		   AUDIOH_ADC_CTL, AUDIOH_ADC_CTL_SIDETONE_EN, 1, 0),

	SOC_SINGLE_EXT("Common ToneGen", 0, 0, 1, 0,
			tonegen_common_enable_get, tonegen_common_enable_put),
	SOC_SINGLE_EXT("ToneGen FreqA", 0, 0, 16000, 0,
			tonegen_freqA_get, tonegen_freqA_put),
	SOC_SINGLE_EXT("ToneGen FreqB", 0, 0, 16000, 0,
			tonegen_freqB_get, tonegen_freqB_put),

	SOC_SINGLE("EP ToneGen Switch",
			AUDIOH_EP_TONEGEN_CTRL, TONEGEN_CTRL_OUTSEL, 1, 0),
	SOC_SINGLE_EXT("EP Tone Scale",
			AUDIOH_EP_TONEGEN_CTRL, TONEGEN_CTRL_SCALE, 0xFFFF, 0,
			tonegen_scale_get, tonegen_scale_put),

	SOC_SINGLE("HS ToneGen Switch",
			AUDIOH_HS_TONEGEN_CTRL, TONEGEN_CTRL_OUTSEL, 1, 0),
	SOC_SINGLE_EXT("HS Tone Scale",
			AUDIOH_HS_TONEGEN_CTRL, TONEGEN_CTRL_SCALE, 0xFFFF, 0,
			tonegen_scale_get, tonegen_scale_put),

	SOC_SINGLE("IHF ToneGen Switch",
			AUDIOH_IHF_TONEGEN_CTRL, TONEGEN_CTRL_OUTSEL, 1, 0),
	SOC_SINGLE_EXT("IHF Tone Scale",
			AUDIOH_IHF_TONEGEN_CTRL, TONEGEN_CTRL_SCALE, 0xFFFF, 0,
			tonegen_scale_get, tonegen_scale_put),

	/* Ranges from 0 to 39 dB, step of 3 dB */
	AUDIOH_SINGLE_TLV("ADC1 PGA",
			AUDIOH_ADC1_CFG, ADC_CFG_PGA_GAIN, 13, 0, adcpga_tlv),
	AUDIOH_SINGLE_TLV("ADC2 PGA",
			AUDIOH_ADC2_CFG, ADC_CFG_PGA_GAIN, 13, 0, adcpga_tlv),

	SOC_SINGLE_RANGE("Sidetone Gain", AUDIOH_SDT_CTRL,
			AUDIOH_SDT_CTRL_TARGET_GAIN, 0, 0x7FFF, 1),
};


static void codec_reset_release(struct snd_soc_component *component)
{
	u32 mask;

	mask = BIT(CDRU_MISC_CODEC_RESETN) |
	       BIT(CDRU_MISC_CODEC_CEC_APB_RESETN);
	crmu_update_bits(component, CDRU_MISC_REG, mask, mask);
}

static void capture_path_filter_disable(struct snd_soc_component *component)
{
	u32 mask, val;

	mask = MIC_FILTER_ORDER_MASK << MIC_FILTER_ORDER;

	/* Set to 8 to bypass */
	val = (MIC_FILTER_ORDER_BYPASS << MIC_FILTER_ORDER);
	audioh_update_bits(component, AUDIOH_CAPTURE_FILT_MIC1_CFG, mask, val);
	audioh_update_bits(component, AUDIOH_CAPTURE_FILT_MIC2_CFG, mask, val);
	audioh_update_bits(component, AUDIOH_CAPTURE_FILT_MIC3_CFG, mask, val);
	audioh_update_bits(component, AUDIOH_CAPTURE_FILT_MIC4_CFG, mask, val);
}

static int audioh_codec_probe(struct snd_soc_component *component)
{
	u32 mask, val;

	codec_reset_release(component);

	capture_path_filter_disable(component);

	/*
	 * Configure all playback FIFOs the same:
	 *   - 24 bit (max resolution), unpacked
	 */
	mask = BIT(PLAYBACK_FIFO_24BIT);
	audioh_update_bits(component, AUDIOH_VOUT_FIFO_CTRL, mask, mask);
	audioh_update_bits(component, AUDIOH_IHF_FIFO_CTRL, mask, mask);
	audioh_update_bits(component, AUDIOH_STEREO_FIFO_CTRL, mask, mask);

	/*
	 * Configure all capture FIFOs the same:
	 *   - 24 bit (max resolution), unpacked
	 */
	mask = BIT(CAPTURE_FIFO1_24BIT) | BIT(CAPTURE_FIFO2_24BIT);
	audioh_update_bits(component, AUDIOH_MIC12_FIFO_CTRL, mask, mask);

	mask = BIT(CAPTURE_FIFO3_24BIT) | BIT(CAPTURE_FIFO4_24BIT);
	audioh_update_bits(component, AUDIOH_MIC34_FIFO_CTRL, mask, mask);

	/* Enable Sidetone FIFO in 24 bit mode*/
	mask = BIT(AUDIOH_SDT_CTRL_3_MODE16BIT) |
		BIT(AUDIOH_SDT_CTRL_3_FIFO_ENABLE);
	val = BIT(AUDIOH_SDT_CTRL_3_FIFO_ENABLE);
	audioh_update_bits(component, AUDIOH_SDT_CTRL_3, mask, val);

	/* Enable sidetone fir filter */
	mask = BIT(AUDIOH_SDT_CTRL_FIR_FILTER_DISABLE);
	audioh_update_bits(component, AUDIOH_SDT_CTRL, mask, 0);

	/* Bypass sidetone filters */
	mask = BIT(AUDIOH_SDT_CTRL_4_FILTER_BYPASS);
	audioh_update_bits(component, AUDIOH_SDT_CTRL_4, mask, mask);

	/* Allow sidetone gain to be updated */
	mask = BIT(AUDIOH_SDT_CTRL_TARGET_GAIN_LOAD);
	audioh_update_bits(component, AUDIOH_SDT_CTRL, mask, mask);

	/* power up the analog interfaces */
	analog_global_init(component);
	ihf_afe_enable(component);
	headset_afe_enable(component);
	ep_afe_init(component);
	analog_mic_enable(component);

	return 0;
}

static const struct snd_soc_component_driver soc_codec_audioh = {
	.probe			= audioh_codec_probe,

	.controls		= audioh_control,
	.num_controls		= ARRAY_SIZE(audioh_control),
	.dapm_widgets		= audioh_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(audioh_dapm_widgets),
	.dapm_routes		= audioh_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(audioh_dapm_routes),

	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static const struct regmap_config audioh_codec_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,

	.max_register = AUDIOH_MAX_REGMAP_REG,
	.cache_type = REGCACHE_NONE,
};

static int audioh_probe(struct platform_device *pdev)
{
	struct audioh_priv *audioh;
	struct resource *mem_res;
	void __iomem *base;
	int ret;

	audioh = devm_kzalloc(&pdev->dev, sizeof(*audioh), GFP_KERNEL);
	if (audioh == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, audioh);

	audioh->dev = &pdev->dev;

	audioh->tonegen_freqA = 400;
	audioh->tonegen_freqB = 400;

	mem_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "audioh");
	base = devm_ioremap_resource(&pdev->dev, mem_res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	audioh->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					    &audioh_codec_regmap_config);
	if (IS_ERR(audioh->regmap))
		return PTR_ERR(audioh->regmap);

	audioh->crmu_regmap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						"brcm,audioh-crmu-syscon");
	if (IS_ERR(audioh->crmu_regmap)) {
		dev_err(&pdev->dev, "crmu regmap failed\n");
		return PTR_ERR(audioh->crmu_regmap);
	}

	ret =  devm_snd_soc_register_component(&pdev->dev, &soc_codec_audioh,
				      audioh_dais, ARRAY_SIZE(audioh_dais));
	if (ret) {
		dev_err(&pdev->dev, "Failed to register component\n");
		return -EINVAL;
	}

	return ret;
}

static int audioh_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id audioh_of_match[] = {
	{.compatible = "brcm,audioh",},
	{}
};

MODULE_DEVICE_TABLE(of, audioh_of_match);

static struct platform_driver bcm_audioh_driver = {
	.driver = {
		   .name = "bcm-audioh",
		   .of_match_table = audioh_of_match,
	},
	.probe = audioh_probe,
	.remove = audioh_remove
};

module_platform_driver(bcm_audioh_driver);

MODULE_DESCRIPTION("Broadcom AudioH codec driver");
MODULE_AUTHOR("Broadcom <bcm-kernel-feedback-list@broadcom.com>");
MODULE_LICENSE("GPL v2");

