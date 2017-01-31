/*
 * Copyright (C) 2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __BCM_IPROC_DSI_LOWLEVEL_H__
#define __BCM_IPROC_DSI_LOWLEVEL_H__

#include <linux/delay.h>

#define DSI_HW_CMND_FIFO_SIZE_B		16
#define DSI_HW_PIXEL_FIFO_SIZE_B	1024
#define DSI_HW_TX_MSG_MAX	(DSI_HW_PIXEL_FIFO_SIZE_B + \
					DSI_HW_CMND_FIFO_SIZE_B)
#define DSI_HW_CMND_WHEN_BEST_EFFORT	0

#define	DSI_HW_CTRL_CLR_LANED_FIFO	BIT(7)
#define	DSI_HW_CTRL_CLR_RXPKT_FIFO	BIT(6)
#define DSI_HW_CTRL_CLR_PIX_DATA_FIFO	BIT(5)
#define	DSI_HW_CTRL_CLR_CMD_DATA_FIFO	BIT(4)

#define	DSI_HW_ISTAT_TXPKT1_DONE	BIT(1)
#define	DSI_HW_STAT_TXPKT1_DONE		BIT(1)

#define	DSI1_PHYC_OFFSET			0x0000004C
#define	DSI1_PHYC_TYPE				uint32_t
#define	DSI1_PHYC_FORCE_TXSTOP_0_SHIFT		2
#define	DSI1_PHYC_FORCE_TXSTOP_0_MASK		BIT(2)
#define	DSI1_PHYC_TXULPSESC_0_SHIFT		1
#define	DSI1_PHYC_TXULPSESC_0_MASK		BIT(1)
#define	DSI1_PHYC_TX_HSCLK_CONT_SHIFT		18
#define	DSI1_PHYC_TX_HSCLK_CONT_MASK		BIT(18)
#define	DSI1_PHYC_TXULPSESC_1_SHIFT		5
#define	DSI1_PHYC_TXULPSESC_1_MASK		BIT(5)
#define	DSI1_PHYC_PHY_DLANE1_EN_MASK		0x00000010
#define	DSI1_PHYC_RESERVED_MASK			0xFC08CCC8
#define	DSI1_PHYC_ESC_CLK_LPDT_SHIFT		20
#define	DSI1_PHYC_ESC_CLK_LPDT_MASK		0x03F00000
#define	DSI1_PHYC_PHY_CLANE_EN_MASK		0x00010000

#define	DSI1_PHYC_PHY_DLANE0_EN_MASK		0x00000001
#define	DSI1_PHY_AFEC0_PD_MASK			0x00001000
#define	DSI1_PHY_AFEC0_OFFSET			0x00000070
#define	DSI1_PHY_AFEC0_RESERVED_MASK		0x00004000
#define	DSI1_PHY_AFEC0_PD_BG_SHIFT		11
#define	DSI1_PHY_AFEC0_PD_BG_MASK		0x00000800

#define	DSI1_LP_DLT7_OFFSET			0x0000006C
#define	DSI1_LP_DLT7_RESERVED_MASK		0xFF000000
#define	DSI1_LP_DLT7_LP_WUP_SHIFT               0
#define	DSI1_LP_DLT7_LP_WUP_MASK		0x00FFFFFF
#define	DSI1_LP_DLT6_OFFSET			0x00000068
#define	DSI1_LP_DLT6_RESERVED_MASK		0x00000000
#define	DSI1_LP_DLT6_LPX_SHIFT                  0
#define	DSI1_LP_DLT6_LPX_MASK			0x000000FF
#define	DSI1_LP_DLT6_TA_GET_SHIFT               24
#define	DSI1_LP_DLT6_TA_GET_MASK		0xFF000000
#define	DSI1_LP_DLT6_TA_SURE_SHIFT		16
#define	DSI1_LP_DLT6_TA_SURE_MASK		0x00FF0000
#define	DSI1_LP_DLT6_TA_GO_SHIFT                8
#define	DSI1_LP_DLT6_TA_GO_MASK			0x0000FF00
#define	DSI1_HS_CLT0_OFFSET			0x00000050
#define	DSI1_HS_CLT0_RESERVED_MASK		0xF8000000
#define	DSI1_HS_CLT0_HS_CZERO_SHIFT		18
#define	DSI1_HS_CLT0_HS_CZERO_MASK		0x07FC0000
#define	DSI1_HS_CLT0_HS_CPRE_SHIFT              9
#define	DSI1_HS_CLT0_HS_CPRE_MASK		0x0003FE00
#define	DSI1_HS_CLT0_HS_CPREP_SHIFT             0
#define	DSI1_HS_CLT0_HS_CPREP_MASK		0x000001FF
#define	DSI1_HS_CLT1_OFFSET			0x00000054
#define	DSI1_HS_CLT1_RESERVED_MASK		0xFFFC0000
#define	DSI1_HS_CLT1_HS_CTRAIL_SHIFT            9
#define	DSI1_HS_CLT1_HS_CTRAIL_MASK		0x0003FE00
#define	DSI1_HS_CLT1_HS_CPOST_SHIFT             0
#define	DSI1_HS_CLT1_HS_CPOST_MASK		0x000001FF
#define	DSI1_HS_CLT2_OFFSET			0x00000058
#define	DSI1_HS_CLT2_RESERVED_MASK		0xFF000000
#define	DSI1_HS_CLT2_HS_WUP_SHIFT               0
#define	DSI1_HS_CLT2_HS_WUP_MASK		0x00FFFFFF
#define	DSI1_HS_DLT3_OFFSET			0x0000005C
#define	DSI1_HS_DLT3_RESERVED_MASK		0xF8000000
#define	DSI1_HS_DLT3_HS_EXIT_SHIFT		18
#define	DSI1_HS_DLT3_HS_EXIT_MASK		0x07FC0000
#define	DSI1_HS_DLT3_HS_ZERO_SHIFT              9
#define	DSI1_HS_DLT3_HS_ZERO_MASK		0x0003FE00
#define	DSI1_HS_DLT3_HS_PRE_SHIFT               0
#define	DSI1_HS_DLT3_HS_PRE_MASK		0x000001FF
#define	DSI1_HS_DLT4_OFFSET			0x00000060
#define	DSI1_HS_DLT4_RESERVED_MASK		0xFF800000
#define	DSI1_HS_DLT4_HS_ANLAT_SHIFT		18
#define	DSI1_HS_DLT4_HS_ANLAT_MASK		0x007C0000
#define	DSI1_HS_DLT4_HS_TRAIL_SHIFT             9
#define	DSI1_HS_DLT4_HS_TRAIL_MASK		0x0003FE00
#define	DSI1_HS_DLT4_HS_LPX_SHIFT               0
#define	DSI1_HS_DLT4_HS_LPX_MASK		0x000001FF
#define	DSI1_HS_DLT5_OFFSET			0x00000064
#define	DSI1_HS_DLT5_RESERVED_MASK		0xFF000000
#define	DSI1_HS_DLT5_HS_INIT_SHIFT              0
#define	DSI1_HS_DLT5_HS_INIT_MASK		0x00FFFFFF

#define DSI1_CTRL_OFFSET			0x00000000
#define DSI1_CTRL_RESERVED_MASK			0xFFFC0008
#define	DSI1_CTRL_HSDT_EOT_EN_SHIFT		11

#define DSI1_INT_EN_OFFSET			0x00000034
#define DSI1_INT_EN_RESERVED_MASK		0x80000000
#define DSI1_INT_STAT_OFFSET			0x00000030
#define DSI1_INT_STAT_RESERVED_MASK		0x80000000
#define DSI1_STAT_OFFSET			0x00000038
#define DSI1_STAT_RESERVED_MASK			0x00000000

#define DSI1_TXPKT_CMD_FIFO_OFFSET		0x0000001C
#define DSI1_TXPKT_CMD_FIFO_RESERVED_MASK	0xFFFFFF00
#define DSI1_TXPKT_PIXD_FIFO_OFFSET		0x00000020
#define DSI1_TXPKT_PIXD_FIFO_RESERVED_MASK      0x00000000
#define	DSI1_TXPKT1_C_CMD_CTRL_SHIFT            4
#define	DSI1_TXPKT1_C_CMD_CTRL_MASK		0x00000030
#define	DSI1_TXPKT1_C_CMD_MODE_SHIFT            3
#define	DSI1_TXPKT1_C_CMD_MODE_MASK		0x00000008
#define	DSI1_TXPKT1_C_CMD_TYPE_SHIFT            2
#define	DSI1_TXPKT1_C_CMD_TYPE_MASK		0x00000004
#define	DSI1_TXPKT1_C_CMD_TE_EN_SHIFT           1
#define	DSI1_TXPKT1_C_CMD_TE_EN_MASK		0x00000002
#define	DSI1_TXPKT1_C_CMD_EN_SHIFT              0
#define	DSI1_TXPKT1_C_CMD_EN_MASK		0x00000001
#define	DSI1_TXPKT1_C_CMD_TX_TIME_SHIFT         6
#define	DSI1_TXPKT1_C_CMD_TX_TIME_MASK		0x000000C0
#define	DSI1_TXPKT1_C_TRIG_CMD_SHIFT            24
#define	DSI1_TXPKT1_C_TRIG_CMD_MASK		0xFF000000
#define DSI1_TXPKT1_C_TYPE                      uint32_t
#define	DSI1_TXPKT1_C_DISPLAY_NO_SHIFT          8
#define	DSI1_TXPKT1_C_DISPLAY_NO_MASK		0x00000300
#define DSI1_TXPKT1_H_OFFSET			0x00000008
#define DSI1_TXPKT1_H_TYPE                      uint32_t
#define DSI1_TXPKT1_H_RESERVED_MASK		0x00000000
#define DSI1_TXPKT1_C_OFFSET			0x00000004
#define DSI1_TXPKT1_C_RESERVED_MASK		0x00000000

#define DSI1_TXPKT2_C_OFFSET			0x0000000C
#define DSI1_TXPKT2_C_RESERVED_MASK		0x00000000
#define DSI1_TXPKT2_H_OFFSET			0x00000010
#define DSI1_TXPKT2_H_RESERVED_MASK		0x00000000
#define	DSI1_TXPKT2_C_CMD_EN_SHIFT              0
#define	DSI1_TXPKT2_C_CMD_EN_MASK		0x00000001

#define	DSI1_PHYC_TXULPSCLK_SHIFT		17
#define	DSI1_PHYC_TXULPSCLK_MASK		0x00020000

#define DSI1_PHY_AFEC0_TYPE                     uint32_t
#define	DSI1_PHY_AFEC0_PD_SHIFT			12

#define	DSI1_CTRL_DISP_CRCC_SHIFT               2
#define	DSI1_CTRL_DISP_ECCC_SHIFT		1
#define	DSI1_CTRL_DSI_EN_SHIFT                  0
#define	DSI1_CTRL_DSI_EN_MASK			0x00000001
#define	DSI1_TXPKT1_C_CMD_REPEAT_SHIFT          10
#define	DSI1_TXPKT1_C_CMD_REPEAT_MASK		0x00FFFC00
#define	DSI1_TXPKT1_H_WC_CDFIFO_SHIFT           24
#define	DSI1_TXPKT1_H_WC_CDFIFO_MASK		0xFF000000
#define	DSI1_TXPKT1_H_WC_PARAM_SHIFT            8
#define	DSI1_TXPKT1_H_WC_PARAM_MASK		0x00FFFF00

#define DSI1_DISP0_CTRL_OFFSET			0x00000028
#define DSI1_DISP0_CTRL_RESERVED_MASK		0xFFC00000
#define DSI1_DISP0_CTRL_PFORMAT_SHIFT		2
#define DSI1_DISP0_CTRL_PFORMAT_MASK		0x0000000C
#define DSI1_DISP0_CTRL_MODE_SHIFT		1
#define DSI1_DISP0_CTRL_MODE_MASK		0x00000002
#define DSI1_DISP0_CTRL_EN_SHIFT		0
#define DSI1_DISP0_CTRL_EN_MASK			0x00000001
#define DSI1_DISP0_CTRL_ST_END_SHIFT		4
#define DSI1_DISP0_CTRL_ST_END_MASK		0x00000010
#define DSI1_DISP0_CTRL_PIX_CLK_DIV_SHIFT	13
#define DSI1_DISP0_CTRL_PIX_CLK_DIV_MASK	0x003FE000

#define DSI1_DISP1_CTRL_OFFSET			0x0000002C
#define DSI1_DISP1_CTRL_RESERVED_MASK		0xFFFFE008
#define DSI1_DISP1_CTRL_DMA_THRESH_SHIFT	4
#define DSI1_DISP1_CTRL_DMA_THRESH_MASK		0x00001FF0
#define DSI1_DISP1_CTRL_PFORMAT_SHIFT		1
#define DSI1_DISP1_CTRL_PFORMAT_MASK		0x00000006
#define DSI1_DISP1_CTRL_EN_SHIFT		0
#define DSI1_DISP1_CTRL_EN_MASK			0x00000001

#define MIPI_DSI_DPHY_CTRL_OFFSET		0x2c
#define MIPI_DSI_MISC_CTRL_OFFSET		0x40
#define DSI1_PHY_AFEC1_OFFSET			0x74
#define DSI_HSTX_TO_CNT_OFFSET			0x3c
#define DSI_LPRX_TO_CNT_OFFSET			0x40
#define DSI_TA_TO_CNT_OFFSET			0x44
#define DSI_PR_TO_CNT_OFFSET			0x48

#define DSI_DPHY_LDOCONTROL_RESET		0xe1ac8488
#define DSI_DPHY_LDOCONTROL_RESET_OUT		0xc1ac8488
#define DSI_DPHY_MISC_CMDINTF_DMA_EN		0x00000001
#define DSI_AFEC0_RESET				0x036da077
#define DSI_AFEC0_RESET_OUT			0x036d8077
#define DSI_APHY1_CLKCNTRL			0x00000000
#define DSI_DISP1_DMA_THRESHOLD			0xc00
#define DSI_HSTX_TO_EN				0x0
#define DSI_LPRX_TO_EN				0x0
#define DSI_TA_TO_EN				0x0
#define DSI_PR_TO_EN				0x0

enum dsi_hw_phy_state {
	PHY_TXSTOP = 1,	/* PHY FORCE TX-STOP  */
	PHY_ULPS = 2,	/* PHY FORCE ULPS  */
	PHY_CORE = 3,	/* PHY under Core Control(remove any forced state) */
};

enum dsi_de1_col_mod {
	DE1_CM_565 = 0,		/* out -> B2 B1 B3 B2 */
	DE1_CM_888U = 1,	/* out -> B2,B1,B0 (B3 ignored) */
	DE1_CM_LE = 2,		/* out -> B0,B1,B2,B3 */
	DE1_CM_BE = 3,		/* out -> B3,B2,B1,B0 */
};

enum dsi_de0_col_mod {
	DE0_CM_565P = 0,
	DE0_CM_666P_VID = 1,
	DE0_CM_666 = 2,
	DE0_CM_888U = 3,
};

enum dsi_de0_mode {
	DE0_MODE_VID = 0,
	DE0_MODE_CMD = 1,
};

struct dsi_tx_cfg {
	unsigned int vc;	/* DSI VC, destination VC */
	bool is_lp;		/* Low Power | High Speed */
	unsigned int vm_when;	/* if Video Mode active, when */
	unsigned int dsi_cmnd;	/* DSI DT  */
	uint8_t *msg;		/* N A to LONG  Packet byte buffer */
	unsigned int msg_len;	/* packet len in bytes */
	unsigned int msg_len_cfifo; /* N A to SHORT packets */
	bool start;		/* start transmission */
	unsigned int repeat;	/* packet repeat count */
	bool end_with_bta;	/* end with BTA, USE ONLY WHEN REPEAT==1 */
	unsigned int disp_engine; /* Display Engine: source of pixel data */
};

enum dsi_clk_sel {
	DSI_HW_BIT_CLK_DIV_BY_8 = 0,	/* BYTE CLOCK */
	DSI_HW_BIT_CLK_DIV_BY_4 = 1,	/* DDR2 */
	DSI_HW_BIT_CLK_DIV_BY_2 = 2,	/* DDR */
};

struct dsi_hw_init {
	unsigned int dlcount;	/* DL count */
};

struct dsi_hw_mode {
	enum dsi_clk_sel clksel;	/* DSI mode */
};

struct dsi_afe_cfg {
	unsigned int afe_cta_adj;	/* DSI-PHY-AFE-C */
	unsigned int afe_pta_adj;	/* DSI-PHY-AFE-C */
	bool afe_bandgap_on;		/* DSI-PHY-AFE-C */
	bool afeds2xclk_ena;		/* DSI-PHY-AFE-C */
	unsigned int afeclk_idr;	/* DSI-PHY-AFE-C */
	unsigned int afedl_idr;		/* DSI-PHY-AFE-C */
};

enum dsi_hw_res {
	DSI_HW_OK,		/* OK */
	DSI_HW_MSG_SIZE,	/* Unsupported msg size */
};

void dsi_hw_phy_state(void *handle, enum dsi_hw_phy_state state);
void dsi_hw_ena_int(void *handle, uint32_t intmask);
uint32_t dsi_hw_get_ena_int(void *handle);
uint32_t dsi_hw_get_int(void *handle);
void dsi_hw_clr_int(void *handle, uint32_t intmask);
void dsi_hw_clr_fifo(void *handle, uint32_t fifomask);
uint32_t dsi_hw_get_status(void *handle);
void dsi_hw_clr_status(void *handle, uint32_t statmask);
void dsi_hw_tx_start(void *handle, uint8_t tx_eng, bool start);
void dsi_hw_de1_set_dma_thresh(void *handle, uint32_t thresh);
void __iomem *dsi_hw_de1_get_dma_address(void *handle);
void dsi_hw_de1_set_cm(void *handle, enum dsi_de1_col_mod cm);
void dsi_hw_de1_enable(void *handle, bool ena);
void dsi_hw_de0_set_cm(void *handle, enum dsi_de0_col_mod cm);
void dsi_hw_de0_set_pix_clk_div(void *handle, unsigned int div);
void dsi_hw_de0_enable(void *handle, bool ena);
void dsi_hw_de0_set_mode(void *handle, enum dsi_de0_mode mode);
void dsi_hw_de0_st_end(void *handle, bool ena);
void dsi_hw_tx_bta(void *handle, uint8_t tx_eng);
void dsi_hw_tx_trig(void *handle, uint8_t tx_eng, uint8_t trig);
enum dsi_hw_res dsi_hw_wr_cfifo(void *handle,
				 uint8_t *buff_ptr, unsigned int byte_count);
enum dsi_hw_res dsi_hw_wr_pfifo_be(void *handle,
				    uint8_t *buff_ptr, unsigned int byte_count);
enum dsi_hw_res dsi_hw_tx_long(void *handle,
				uint8_t tx_eng, struct dsi_tx_cfg *tx_cfg);
enum dsi_hw_res dsi_hw_tx_short(void *handle,
				 uint8_t tx_cfg, struct dsi_tx_cfg *cmnd);
void dsi_hw_on(void *handle, void __iomem *dsi_genpll_base);
void dsi_hw_off(void *handle);
void dsi_reset(void __iomem *dsi_reset_base, int reset_active, bool on);
void dsi_hw_phy_afe_off(void *handle);
bool dsi_hw_set_timing(void *handle, void *phy_timing,
		enum dsi_clk_sel coreclksel, unsigned int esc_clk_mhz,
		unsigned int hs_bitrate_mbps, unsigned int lpbitrate_mbps);
void *dsi_hw_init(void __iomem *base_address, struct dsi_hw_init *dsi_init);
void dsi_errata_init(void __iomem *base_address,
				int size, bool has_mipi_errata);

#endif		/* __BCM_IPROC_DSI_LOWLEVEL_H__ */
