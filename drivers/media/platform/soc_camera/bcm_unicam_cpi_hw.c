/*
 * Copyright (C) 2016 Broadcom
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

/*
 * Unicam camera host hw exports
 */

#include <linux/bitops.h>
#include <linux/mfd/syscon.h>

#include "bcm_unicam_cpi.h"

/* Camera Pad Values */
#define CAMPAD_CFG_REG		0x0
#define CAMPAD_CFG_MASK		BIT(8)
#define CAMPAD_CFG_VAL		0x00000100

/* Camera Values */
#define CAMCPIS_DEFAULT		0x00000000
#define CAMCPIS_SOFTRESET	0x00000020
#define CAMCPIS_ENB		0x00000001
#define CAMCPIS_CPEMEM_DIS_MASK	0xfffffffc

#define CAMCPIS_CFG_MASK	0x7ffffffc
#define CAMCPIS_CFG_VAL		0x80008003

#define CAMCPIR_DEFAULT		0x00000000
#define CAMCPIR_2_8BIT		0x00000002
#define CAMCPIR_2_10BIT		0x00000082
#define CAMCPIR_3_8BIT		0x00000003
#define CAMCPIR_3_10BIT		0x00000083
#define CAMCPIR_DWID_8		0x00000000
#define CAMCPIR_DWID_10		0x00000080
/* DRSYNC, HSRM, VSRM Negative edge */
#define CAMCPIR_DHV_SYNC_NEGE	0x0000001c
/* HSRM-nagative, VSRM-negative, HSAL-high, VSAL-high */
#define CAMCPIR_HV_MODELEVEL	0x0000000f

#define CAMCPIF_MODE0		0x00000000

#define CAMCPIW_DIS_WIN		0x00000000
#define CAMCPIW_ENB_WIN		0x00000001
#define CAMCPIWHC		0x08000000
#define CAMCPIWVC		0x00100000

#define CAMCPIF_DEFAULT		0x00000035
#define CAMCPIF_SHIFT_SYNC_ENB	0x1
#define CAMCPIF_SHIFT_SYNC_0	0x0
#define CAMCPIF_SHIFT_SYNC_1	0x1
#define CAMCPIF_VSYNC_BIT7	0x7
#define CAMCPIF_HSYNC_BIT6	0x6
#define CAMCPIF_FIELD_BIT8	0x8
#define CAMCPIF_VSYNC_BIT5	0x5
#define CAMCPIF_HSYNC_BIT4	0x4
#define CAMCPIF_FIELD_BIT6	0x6

#define CAMCPIF_SCMODE_EN	0x00000008

#define CAMCPIF_HVSYNC_SETTING	0x0000000A

#define CAMIDI0_0		0x00000000
#define CAMIDI0_1		0x00000080

#define CAMIBLS_DEFAULT		0x00000500
#define CAMIBLS_RESET_ZERO	0x00000000

#define CAMIBSA0_RESET		0x00000000
#define CAMIBEA0_RESET		0x00000000

#define CAMDCS_IMG_DEFAULT	0x00000000
#define CAMDCS_IMG_JPEG		0x00010021

#define CAMFIX0_DEFAULT		0x00000000
#define CAMFIX0_CPI_SELECT	0x1
#define CAMFIX0_CPI_DIS_DB	0x1

#define FSP_FLUSH_RES_LS	0x1
#define FSP_FLUSH_RES_FE	0x2

#define CAMCTL_DEFAULT		0x0002050b

#define CAMDBCTL_DB_EN		0x1

#define CAMCTL_OET_MASK		0xFFF00FFF
#define CAMCTL_OET_VAL		0xFF

#define CAMISTA_BUF0_RDY_SHIFT	20
#define CAMISTA_BUF1_RDY_SHIFT	22

#define CAMIPIPE_UNONE_P10	0x00000100
#define CAMIPIPE_UNONE_P8	0x00000080
#define CAMIPIPE_UNONE_PNONE	0x00000000
#define CAMIPIPE_U10_PNONE	0x00000004
#define CAMIPIPE_U8_PNONE	0x00000003
#define CAMIPIPE_U10_P10	0x00000104
#define CAMIPIPE_U8_P8		0x00000083

/* LIP:1, TFC:1, FCM:1, FEIE:1, FSIE:1 */
#define CAMICTL_MASK		0xffffffC4
#define CAMICTL_VAL		0x0000003B
#define CAMICTL_MASK_CONT	0xffffff98
#define CAMICTL_VAL_CONT	0x00000067
#define CAMICTL_BUF1_VAL	0x0000007b
#define CAMICTL_BUF0_VAL	0x0000003b
#define CAMICTL_BUF1_VAL_CONT	0x00000067
#define CAMICTL_BUF0_VAL_CONT	0x00000027

/* 8 = 512 cycle delay */
#define CAMPRI_BS		0x00008000
#define CAMSTA_OES_SDWN		0x00000100

/* CPI Mode */
#define CPI_MODE_0		0
#define CPI_MODE_1		1
#define CPI_MODE_2		2
#define CPI_MODE_3		3

/* CPI Data width */
#define CPI_DW_8		8
#define CPI_DW_10		10

/* CAMERA registers */
#define CAM_TOP_REGS_CAMCTL_OFFSET 0x000
#define CAM_TOP_REGS_CAMCTL_STANDBY_SHIFT 21
#define CAM_TOP_REGS_CAMCTL_RST_SHIFT 20
#define CAM_TOP_REGS_CAMCTL_OET_R 12
#define CAM_TOP_REGS_CAMCTL_SOE_SHIFT 4
#define CAM_TOP_REGS_CAMCTL_CPR_SHIFT 2
#define CAM_TOP_REGS_CAMCTL_CPE_SHIFT 0
#define CAM_TOP_REGS_CAMSTA_OFFSET 0x004
#define CAM_TOP_REGS_CAMSTA_BUF1_NO_SHIFT 23
#define CAM_TOP_REGS_CAMSTA_BUF0_NO_SHIFT 21
#define CAM_TOP_REGS_CAMSTA_IS_SHIFT 14
#define CAM_TOP_REGS_CAMSTA_PS_SHIFT 13
#define CAM_TOP_REGS_CAMSTA_DL_SHIFT 12
#define CAM_TOP_REGS_CAMSTA_BFO_SHIFT 11
#define CAM_TOP_REGS_CAMSTA_OFO_SHIFT 10
#define CAM_TOP_REGS_CAMSTA_IFO_SHIFT 9
#define CAM_TOP_REGS_CAMSTA_OES_SHIFT 8
#define CAM_TOP_REGS_CAMSTA_CS_SHIFT 1
#define CAM_TOP_REGS_CAMSTA_SYN_SHIFT 0
#define CAM_TOP_REGS_CAMANA_OFFSET 0x008
#define CAM_TOP_REGS_CAMPRI_OFFSET 0x00c
#define CAM_TOP_REGS_CAMCLK_OFFSET 0x010
#define CAM_TOP_REGS_CAMCLT_OFFSET 0x014
#define CAM_TOP_REGS_CAMDAT0_OFFSET 0x018
#define CAM_TOP_REGS_CAMDAT1_OFFSET 0x01c
#define CAM_TOP_REGS_CAMDLT_OFFSET 0x028
#define CAM_TOP_REGS_CAMICTL_OFFSET 0x100
#define CAM_TOP_REGS_CAMICTL_TFC_SHIFT 4
#define CAM_TOP_REGS_CAMISTA_OFFSET 0x104
#define CAM_TOP_REGS_CAMIDI0_OFFSET 0x108
#define CAM_TOP_REGS_CAMIPIPE_OFFSET 0x10c
#define CAM_TOP_REGS_CAMIBSA0_OFFSET 0x110
#define CAM_TOP_REGS_CAMIBEA0_OFFSET 0x114
#define CAM_TOP_REGS_CAMIBLS_OFFSET 0x118
#define CAM_TOP_REGS_CAMIBWP_OFFSET 0x11c
#define CAM_TOP_REGS_CAMDCS_OFFSET 0x200
#define CAM_TOP_REGS_CAMDBCTL_OFFSET 0x300
#define CAM_TOP_REGS_CAMIBSA1_OFFSET 0x304
#define CAM_TOP_REGS_CAMIBEA1_OFFSET 0x308
#define CAM_TOP_REGS_CAMFIX0_OFFSET 0x400
#define CAM_TOP_REGS_CAMFIX0_CPI_SELECT 17
#define CAM_TOP_REGS_CAMFIX0_FSP_FLUSH_R 10
#define CAM_TOP_REGS_CAMFIX0_DIS_DB_IE 4
#define CAM_TOP_REGS_CAMCPIS_OFFSET 0x500
#define CAM_TOP_REGS_CAMCPIR_OFFSET 0x504
#define CAM_TOP_REGS_CAMCPIF_OFFSET 0x508
#define CAM_TOP_REGS_CAMCPIF_FIELD_BIT_R 24
#define CAM_TOP_REGS_CAMCPIF_HSYNC_BIT_R 20
#define CAM_TOP_REGS_CAMCPIF_VSYNC_BIT_R 16
#define CAM_TOP_REGS_CAMCPIF_SHIFT_SYNC_R 13
#define CAM_TOP_REGS_CAMCPIF_EMB_SYNCSHIFT_EN 12
#define CAM_TOP_REGS_CAMCPIW_OFFSET 0x50c
#define CAM_TOP_REGS_CAMCPIWVC_OFFSET 0x510
#define CAM_TOP_REGS_CAMCPIWHC_OFFSET 0x518

/*
 ** HELPER MACROS
 ** These are used to combine names for the actual read and write macros.
 */
#define CPI_READ_REG(r)		readl(r)
#define CPI_WRITE_REG(b, r)	writel(b, r)

struct config_param {
	bool seq_asiu_cam_soft_reset_enable;
	uint32_t seq_asiu_cam_cpi_data_width;
	uint32_t seq_asiu_cam_cpi_mode;
	bool seq_asiu_cam_unpack_option;
	bool seq_asiu_cam_pack_option;
	bool seq_asiu_horz_window_enable;
	uint32_t cfg_asiu_horz_window_value;
	uint32_t seq_asiu_verti_window_enable;
	uint32_t cfg_asiu_verti_window_value;
	bool seq_asiu_multiple_frames_en;
	bool cpi_clk_neg_edg_en;
	bool cpi_burst_space_enable;
};

struct unicam_camera_config {
	void __iomem *base;
	struct config_param cam_config;
};

static void asiu_cam_cpi_seq(struct unicam_camera_config *config)
{

	void __iomem *base = config->base;
	struct config_param *cam_config = &config->cam_config;

	if (cam_config->seq_asiu_cam_soft_reset_enable == false)
		CPI_WRITE_REG(CAMCPIS_DEFAULT,
				base + CAM_TOP_REGS_CAMCPIS_OFFSET);
	else
		CPI_WRITE_REG(CAMCPIS_SOFTRESET,
				base + CAM_TOP_REGS_CAMCPIS_OFFSET);

	/* Enable CPI Pheripheral ENB */
	CPI_WRITE_REG(CAMCPIS_ENB, base + CAM_TOP_REGS_CAMCPIS_OFFSET);

	/* Data width : DWID[8:7] */
	if (cam_config->seq_asiu_cam_cpi_data_width == CPI_DW_10)
		CPI_WRITE_REG(CAMCPIR_DWID_10,
					base + CAM_TOP_REGS_CAMCPIR_OFFSET);
	else
		CPI_WRITE_REG(CAMCPIR_DWID_8,
					base + CAM_TOP_REGS_CAMCPIR_OFFSET);

	if (cam_config->seq_asiu_cam_cpi_mode == CPI_MODE_0) {
		/* MODE=HV */
		CPI_WRITE_REG(CAMCPIF_MODE0,
					base + CAM_TOP_REGS_CAMCPIF_OFFSET);
	} else if (cam_config->seq_asiu_cam_cpi_mode == CPI_MODE_1) {
		/* Embedded Mode */
		CPI_WRITE_REG(CAMCPIW_ENB_WIN,
				base + CAM_TOP_REGS_CAMCPIW_OFFSET);
		CPI_WRITE_REG(CAMCPIWHC,
				base + CAM_TOP_REGS_CAMCPIWHC_OFFSET);
		CPI_WRITE_REG(CAMCPIWVC,
				base + CAM_TOP_REGS_CAMCPIWVC_OFFSET);

		if (cam_config->seq_asiu_cam_cpi_data_width == CPI_DW_10) {
			/* Horizontal Sync Mode=HSAL,Vertical Sync Mode=VSAL,
			*  Sync Mode=Embedded
			*/
			CPI_WRITE_REG((CAMCPIF_DEFAULT |
				CAMCPIF_SHIFT_SYNC_ENB <<
					CAM_TOP_REGS_CAMCPIF_EMB_SYNCSHIFT_EN |
				CAMCPIF_SHIFT_SYNC_0 <<
					CAM_TOP_REGS_CAMCPIF_SHIFT_SYNC_R |
				CAMCPIF_VSYNC_BIT7 <<
					CAM_TOP_REGS_CAMCPIF_VSYNC_BIT_R |
				CAMCPIF_HSYNC_BIT6 <<
					CAM_TOP_REGS_CAMCPIF_HSYNC_BIT_R |
				CAMCPIF_FIELD_BIT8 <<
					CAM_TOP_REGS_CAMCPIF_FIELD_BIT_R),
				base + CAM_TOP_REGS_CAMCPIF_OFFSET);
		} else {
			/* Horizontal Sync Mode=HSAL,Vertical Sync Mode=VSAL,
			*  Sync Mode=Embedded
			*/
			CPI_WRITE_REG((CAMCPIF_DEFAULT |
				CAMCPIF_SHIFT_SYNC_ENB <<
					CAM_TOP_REGS_CAMCPIF_EMB_SYNCSHIFT_EN |
				CAMCPIF_SHIFT_SYNC_1 <<
					CAM_TOP_REGS_CAMCPIF_SHIFT_SYNC_R |
				CAMCPIF_VSYNC_BIT5 <<
					CAM_TOP_REGS_CAMCPIF_VSYNC_BIT_R |
				CAMCPIF_HSYNC_BIT4 <<
					CAM_TOP_REGS_CAMCPIF_HSYNC_BIT_R |
				CAMCPIF_FIELD_BIT6 <<
					CAM_TOP_REGS_CAMCPIF_FIELD_BIT_R),
				base + CAM_TOP_REGS_CAMCPIF_OFFSET);
		}
	} else if (cam_config->seq_asiu_cam_cpi_mode == CPI_MODE_2) {
		if (cam_config->seq_asiu_cam_cpi_data_width == CPI_DW_8)
			CPI_WRITE_REG(CAMCPIR_2_8BIT,
				base + CAM_TOP_REGS_CAMCPIR_OFFSET);
		else if (cam_config->seq_asiu_cam_cpi_data_width == CPI_DW_10)
			CPI_WRITE_REG(CAMCPIR_2_10BIT,
				base + CAM_TOP_REGS_CAMCPIR_OFFSET);
		else
			CPI_WRITE_REG(CAMCPIR_DEFAULT,
				base + CAM_TOP_REGS_CAMCPIR_OFFSET);

		CPI_WRITE_REG(CAMCPIF_SCMODE_EN,
					base + CAM_TOP_REGS_CAMCPIF_OFFSET);
		CPI_WRITE_REG(CAMCPIW_ENB_WIN,
					base + CAM_TOP_REGS_CAMCPIW_OFFSET);
		CPI_WRITE_REG(cam_config->cfg_asiu_horz_window_value,
			base + CAM_TOP_REGS_CAMCPIWHC_OFFSET);
		CPI_WRITE_REG(cam_config->cfg_asiu_verti_window_value,
			base + CAM_TOP_REGS_CAMCPIWVC_OFFSET);
	} else {
		CPI_WRITE_REG(CAMCPIF_MODE0,
					base + CAM_TOP_REGS_CAMCPIF_OFFSET);
		CPI_WRITE_REG(CAMCPIW_ENB_WIN,
					base + CAM_TOP_REGS_CAMCPIW_OFFSET);
		CPI_WRITE_REG(cam_config->cfg_asiu_horz_window_value,
				base + CAM_TOP_REGS_CAMCPIWHC_OFFSET);
		CPI_WRITE_REG(cam_config->cfg_asiu_verti_window_value,
				base + CAM_TOP_REGS_CAMCPIWVC_OFFSET);
	}

	CPI_WRITE_REG(CAMCPIW_DIS_WIN, base + CAM_TOP_REGS_CAMCPIW_OFFSET);

	/* Image data Identifier */
	CPI_WRITE_REG(CAMIDI0_0, base + CAM_TOP_REGS_CAMIDI0_OFFSET);

	if (cam_config->seq_asiu_cam_cpi_mode == CPI_MODE_3)
		CPI_WRITE_REG(CAMIDI0_0, base + CAM_TOP_REGS_CAMIDI0_OFFSET);
	else
		CPI_WRITE_REG(CAMIDI0_1, base + CAM_TOP_REGS_CAMIDI0_OFFSET);

	/* Set Buffer Line Stride */
	CPI_WRITE_REG(CAMIBLS_DEFAULT, base + CAM_TOP_REGS_CAMIBLS_OFFSET);
	/* Data Control & Status */
	CPI_WRITE_REG(CAMDCS_IMG_DEFAULT, base + CAM_TOP_REGS_CAMDCS_OFFSET);
	/* CPI Select and Disable buffer ready interrupts */
	CPI_WRITE_REG((CAMFIX0_DEFAULT |
			CAMFIX0_CPI_SELECT <<
					CAM_TOP_REGS_CAMFIX0_CPI_SELECT |
			(FSP_FLUSH_RES_LS | FSP_FLUSH_RES_FE) <<
				CAM_TOP_REGS_CAMFIX0_FSP_FLUSH_R |
			CAMFIX0_CPI_DIS_DB <<
				CAM_TOP_REGS_CAMFIX0_DIS_DB_IE),
			base + CAM_TOP_REGS_CAMFIX0_OFFSET);
	CPI_WRITE_REG(CAMCTL_DEFAULT, base + CAM_TOP_REGS_CAMCTL_OFFSET);
}

void unicam_camera_cpi_config(struct unicam_camera_dev *dev)
{
	void __iomem *base = dev->base;
	unsigned int reg_data;
	struct unicam_camera_config config;
	struct config_param *cam_config = &config.cam_config;

	memset(cam_config, 0x0, sizeof(struct config_param));

	if (dev->data_width == 10)
		cam_config->seq_asiu_cam_cpi_data_width = CPI_DW_10;
	else
		cam_config->seq_asiu_cam_cpi_data_width = CPI_DW_8;
	cam_config->seq_asiu_cam_cpi_mode = CPI_MODE_0;
	cam_config->seq_asiu_cam_pack_option = true;
	cam_config->seq_asiu_multiple_frames_en = false;

	/* Reset & re-initialise */
	reg_data = CPI_READ_REG(base + CAM_TOP_REGS_CAMCTL_OFFSET);
	reg_data |= BIT(CAM_TOP_REGS_CAMCTL_CPR_SHIFT);
	CPI_WRITE_REG(reg_data, base + CAM_TOP_REGS_CAMCTL_OFFSET);
	reg_data &= ~BIT(CAM_TOP_REGS_CAMCTL_CPR_SHIFT);
	CPI_WRITE_REG(reg_data, base + CAM_TOP_REGS_CAMCTL_OFFSET);

	if (cam_config->seq_asiu_multiple_frames_en)
		CPI_WRITE_REG(CAMDBCTL_DB_EN,
				base + CAM_TOP_REGS_CAMDBCTL_OFFSET);

	config.base = base;
	asiu_cam_cpi_seq(&config);

	/* Extend timeout */
	reg_data = CPI_READ_REG(base + CAM_TOP_REGS_CAMCTL_OFFSET);
	reg_data &= CAMCTL_OET_MASK;
	reg_data |= (CAMCTL_OET_VAL << CAM_TOP_REGS_CAMCTL_OET_R);
	CPI_WRITE_REG(reg_data, base + CAM_TOP_REGS_CAMCTL_OFFSET);

	/* Set Pixel packing/Unpacking Modes */
	if (!cam_config->seq_asiu_cam_unpack_option &&
					cam_config->seq_asiu_cam_pack_option) {
		if (cam_config->seq_asiu_cam_cpi_data_width == CPI_DW_10)
			/* Unpack: None , Pack: PACK10 */
			CPI_WRITE_REG(CAMIPIPE_UNONE_P10,
				base + CAM_TOP_REGS_CAMIPIPE_OFFSET);
		else
			/* Unpack: None , Pack: PACK8 */
			CPI_WRITE_REG(CAMIPIPE_UNONE_P8,
				base + CAM_TOP_REGS_CAMIPIPE_OFFSET);
	} else if (!cam_config->seq_asiu_cam_unpack_option &&
				!cam_config->seq_asiu_cam_pack_option)
		/* Unpack: None , Pack: None */
		CPI_WRITE_REG(CAMIPIPE_UNONE_PNONE,
					base + CAM_TOP_REGS_CAMIPIPE_OFFSET);
	else if (cam_config->seq_asiu_cam_unpack_option &&
				!cam_config->seq_asiu_cam_pack_option) {
		if (cam_config->seq_asiu_cam_cpi_data_width == CPI_DW_10)
			/* Unpack: RAW10 , Pack: None */
			CPI_WRITE_REG(CAMIPIPE_U10_PNONE,
				base + CAM_TOP_REGS_CAMIPIPE_OFFSET);
		else
			/* Unpack: RAW8 , Pack: None */
			CPI_WRITE_REG(CAMIPIPE_U8_PNONE,
				base + CAM_TOP_REGS_CAMIPIPE_OFFSET);
	} else if (cam_config->seq_asiu_cam_unpack_option &&
				cam_config->seq_asiu_cam_pack_option) {
		if (cam_config->seq_asiu_cam_cpi_data_width == CPI_DW_10)
			/* Unpack: RAW10 , Pack: PACK10 */
			CPI_WRITE_REG(CAMIPIPE_U10_P10,
				base + CAM_TOP_REGS_CAMIPIPE_OFFSET);
		else
			/* Unpack: RAW8 , Pack: PACK8 */
			CPI_WRITE_REG(CAMIPIPE_U8_P8,
				base + CAM_TOP_REGS_CAMIPIPE_OFFSET);
	}

	if (cam_config->seq_asiu_horz_window_enable ||
			cam_config->seq_asiu_verti_window_enable) {
		CPI_WRITE_REG(CAMCPIW_ENB_WIN,
					base + CAM_TOP_REGS_CAMCPIW_OFFSET);
		CPI_WRITE_REG(CAMCPIF_HVSYNC_SETTING,
					base + CAM_TOP_REGS_CAMCPIF_OFFSET);
		if (cam_config->seq_asiu_cam_cpi_data_width == CPI_DW_8)
			CPI_WRITE_REG(CAMCPIR_3_8BIT,
				base + CAM_TOP_REGS_CAMCPIR_OFFSET);
		else if (cam_config->seq_asiu_cam_cpi_data_width == CPI_DW_10)
			CPI_WRITE_REG(CAMCPIR_3_10BIT,
				base + CAM_TOP_REGS_CAMCPIR_OFFSET);
		else
			CPI_WRITE_REG(CAMCPIR_3_8BIT,
				base + CAM_TOP_REGS_CAMCPIR_OFFSET);
		/* Vertical Windowing */
		if (cam_config->seq_asiu_verti_window_enable) {
			CPI_WRITE_REG(cam_config->cfg_asiu_verti_window_value,
					base + CAM_TOP_REGS_CAMCPIWVC_OFFSET);
		} else {
			/* Horizontal Windowing */
			CPI_WRITE_REG(cam_config->cfg_asiu_horz_window_value,
					base + CAM_TOP_REGS_CAMCPIWHC_OFFSET);
		}
	}
	if (cam_config->seq_asiu_cam_cpi_mode == CPI_MODE_3) {
		dev_dbg(dev->icd->parent, "JPEG Mode\n");
		CPI_WRITE_REG(CAMDCS_IMG_JPEG,
					base + CAM_TOP_REGS_CAMDCS_OFFSET);
	} else
		CPI_WRITE_REG(CAMDCS_IMG_DEFAULT,
					base + CAM_TOP_REGS_CAMDCS_OFFSET);
	if (cam_config->cpi_clk_neg_edg_en) {
		reg_data = CPI_READ_REG(base + CAM_TOP_REGS_CAMCPIR_OFFSET);
		reg_data |= CAMCPIR_DHV_SYNC_NEGE;
		CPI_WRITE_REG(reg_data, base + CAM_TOP_REGS_CAMCPIR_OFFSET);
	}
	if (cam_config->cpi_burst_space_enable) {
		/* Burst Space Configuration : 512 cycle delay */
		reg_data = CPI_READ_REG(base + CAM_TOP_REGS_CAMPRI_OFFSET);
		reg_data |= CAMPRI_BS;
		CPI_WRITE_REG(reg_data, base + CAM_TOP_REGS_CAMPRI_OFFSET);
	}

	reg_data = CPI_READ_REG(base + CAM_TOP_REGS_CAMCPIR_OFFSET);
	reg_data |= CAMCPIR_HV_MODELEVEL;
	CPI_WRITE_REG(reg_data, base + CAM_TOP_REGS_CAMCPIR_OFFSET);

	/* Enable FRAME Start and End Interrupts and mode */
	/* LIP:1, TFC:1, FCM:1, FEIE:1, FSIE:1 */
	reg_data = CPI_READ_REG(base + CAM_TOP_REGS_CAMICTL_OFFSET);
	reg_data = (reg_data & CAMICTL_MASK) | CAMICTL_VAL;
	CPI_WRITE_REG(reg_data, base + CAM_TOP_REGS_CAMICTL_OFFSET);

	/* Enable Interrupt -- Reg. config. during CPI Capture */
	reg_data = CPI_READ_REG(base + CAM_TOP_REGS_CAMCPIS_OFFSET);
	reg_data = (reg_data & CAMCPIS_CFG_MASK) | CAMCPIS_CFG_VAL;
	CPI_WRITE_REG(reg_data, base + CAM_TOP_REGS_CAMCPIS_OFFSET);

	/* Program camera pads -- Reg. config. during CPI Capture */
	regmap_update_bits(dev->asiu_pad_ctrl, CAMPAD_CFG_REG,
			   CAMPAD_CFG_MASK, CAMPAD_CFG_VAL);
}

void unicam_camera_reset_standby(struct unicam_camera_dev *dev)
{
	unsigned int reg_value;
	void __iomem *base = dev->base;

	reg_value = CPI_READ_REG(base + CAM_TOP_REGS_CAMCTL_OFFSET);
	reg_value &= ~BIT(CAM_TOP_REGS_CAMCTL_STANDBY_SHIFT);
	reg_value |= BIT(CAM_TOP_REGS_CAMCTL_RST_SHIFT);
	reg_value |= BIT(CAM_TOP_REGS_CAMCTL_CPE_SHIFT);
	CPI_WRITE_REG(reg_value, base + CAM_TOP_REGS_CAMCTL_OFFSET);
}

void disable_cpi_interface(struct unicam_camera_dev *dev)
{
	unsigned int read_data, val = 0, i = 0;
	void __iomem *base = dev->base;
	/* shut down */
	read_data = CPI_READ_REG(base + CAM_TOP_REGS_CAMCTL_OFFSET);
	read_data |= BIT(CAM_TOP_REGS_CAMCTL_SOE_SHIFT);
	CPI_WRITE_REG(read_data, base + CAM_TOP_REGS_CAMCTL_OFFSET);

	while ((val & CAMSTA_OES_SDWN) == 0) {
		val = CPI_READ_REG(base + CAM_TOP_REGS_CAMSTA_OFFSET);
		/* wait for OES = 1 */
		/* delay */
		if (++i  == 10) {
			dev_err(dev->icd->parent,
				"\n Err in unicam shutdown:STA = 0x%x\n", val);
			return;
		}
	}

	read_data = CPI_READ_REG(base + CAM_TOP_REGS_CAMCTL_OFFSET);
	read_data &= ~BIT(CAM_TOP_REGS_CAMCTL_SOE_SHIFT);
	CPI_WRITE_REG(read_data, base + CAM_TOP_REGS_CAMCTL_OFFSET);

	/* peripheral soft reset enable */
	read_data = CPI_READ_REG(base + CAM_TOP_REGS_CAMCTL_OFFSET);
	read_data |= BIT(CAM_TOP_REGS_CAMCTL_CPR_SHIFT);
	CPI_WRITE_REG(read_data, base + CAM_TOP_REGS_CAMCTL_OFFSET);

	/* peripheral soft reset disable */
	read_data = CPI_READ_REG(base + CAM_TOP_REGS_CAMCTL_OFFSET);
	read_data &= ~BIT(CAM_TOP_REGS_CAMCTL_CPR_SHIFT);
	CPI_WRITE_REG(read_data, base + CAM_TOP_REGS_CAMCTL_OFFSET);

	CPI_WRITE_REG(CAMIBLS_RESET_ZERO, base + CAM_TOP_REGS_CAMIBLS_OFFSET);
	CPI_WRITE_REG(CAMIBSA0_RESET, base + CAM_TOP_REGS_CAMIBSA0_OFFSET);
	CPI_WRITE_REG(CAMIBEA0_RESET, base + CAM_TOP_REGS_CAMIBEA0_OFFSET);

	/* peripheral disable */
	read_data = CPI_READ_REG(base + CAM_TOP_REGS_CAMCTL_OFFSET);
	read_data &= ~BIT(CAM_TOP_REGS_CAMCTL_CPE_SHIFT);
	CPI_WRITE_REG(read_data, base + CAM_TOP_REGS_CAMCTL_OFFSET);

	/* cpi disable */
	read_data = CPI_READ_REG(base + CAM_TOP_REGS_CAMCPIS_OFFSET);
	read_data &= CAMCPIS_CPEMEM_DIS_MASK;
	CPI_WRITE_REG(read_data, base + CAM_TOP_REGS_CAMCPIS_OFFSET);
}

void unicam_reg_dump(struct unicam_camera_dev *dev)
{
	void __iomem *base = dev->base;

	dev_dbg(dev->icd->parent, "CAM_CTL 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMCTL_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_STA 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMSTA_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_ANA 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMANA_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_PRI 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMPRI_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_CLK 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMCLK_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_CLT 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMCLT_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_DAT0 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMDAT0_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_DAT1 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMDAT1_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_DLT 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMDLT_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_ICTL 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMICTL_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_ISTA 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMISTA_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_IDI 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMIDI0_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_IPIPE 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMIPIPE_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_IBSA 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMIBSA0_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_IBEA 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMIBEA0_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_IBLS 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMIBLS_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_DCS 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMDCS_OFFSET));
	dev_dbg(dev->icd->parent, "CAM_TOP_REGS_CAMFIX0_OFFSET 0x%x\n",
			CPI_READ_REG(base + CAM_TOP_REGS_CAMFIX0_OFFSET));
}

int unicam_cpi_get_int_stat(struct unicam_camera_dev *dev,
		int rx_stat, int ack)
{
	void __iomem *base = dev->base;
	unsigned int reg, ctrl;

	reg = CPI_READ_REG(base + CAM_TOP_REGS_CAMISTA_OFFSET);

	if (ack) {
		if (rx_stat & BIT(CAMISTA_BUF0_RDY_SHIFT)) {/*BUF0_RDY*/
			if (dev->camera_mode_continuous)
				CPI_WRITE_REG(CAMICTL_BUF1_VAL_CONT,
					base + CAM_TOP_REGS_CAMICTL_OFFSET);
			else
				CPI_WRITE_REG(CAMICTL_BUF1_VAL,
					base + CAM_TOP_REGS_CAMICTL_OFFSET);
		} else if (rx_stat & BIT(CAMISTA_BUF1_RDY_SHIFT)) {/*BUF1_RDY*/
			if (dev->camera_mode_continuous)
				CPI_WRITE_REG(CAMICTL_BUF0_VAL_CONT,
					base + CAM_TOP_REGS_CAMICTL_OFFSET);
			else
				CPI_WRITE_REG(CAMICTL_BUF0_VAL,
					base + CAM_TOP_REGS_CAMICTL_OFFSET);
		}
	}

	CPI_WRITE_REG(reg, base + CAM_TOP_REGS_CAMISTA_OFFSET);

	ctrl = CPI_READ_REG(base + CAM_TOP_REGS_CAMICTL_OFFSET);
	dev_dbg(dev->icd->parent, "CAMICTL = 0x%x\n", ctrl);

	ctrl = CPI_READ_REG(base + CAM_TOP_REGS_CAMIBWP_OFFSET);
	dev_dbg(dev->icd->parent, "Image buffer write pointer  0x%x\n", ctrl);

	return reg;
}

unsigned int unicam_cpi_get_rx_stat(struct unicam_camera_dev *dev, int ack)
{
	void __iomem *base = dev->base;
	unsigned int reg;

	reg = CPI_READ_REG(base + CAM_TOP_REGS_CAMSTA_OFFSET);

	if (ack) {
		/*should not touch the RO bits*/
		reg &= ~(BIT(CAM_TOP_REGS_CAMSTA_SYN_SHIFT) |
			BIT(CAM_TOP_REGS_CAMSTA_OES_SHIFT) |
			BIT(CAM_TOP_REGS_CAMSTA_IS_SHIFT) |
			BIT(CAM_TOP_REGS_CAMSTA_BUF0_NO_SHIFT) |
			BIT(CAM_TOP_REGS_CAMSTA_BUF1_NO_SHIFT));

		CPI_WRITE_REG(reg, base +  CAM_TOP_REGS_CAMSTA_OFFSET);
	}
	dev_dbg(dev->icd->parent, "CAMSTA: 0x%x\n", reg);

	return reg;
}

void unicam_cpi_update_addr(struct unicam_camera_dev *dev,
		struct buffer_desc *im0,
		struct buffer_desc *im1, struct buffer_desc *dat0,
		struct buffer_desc *dat1)
{
	void __iomem *base = dev->base;

	if (im0 == NULL && im1 == NULL)
		return;

	if (im0) {
		CPI_WRITE_REG(im0->ls, base + CAM_TOP_REGS_CAMIBLS_OFFSET);
		CPI_WRITE_REG(im0->start, base + CAM_TOP_REGS_CAMIBSA0_OFFSET);
		CPI_WRITE_REG((im0->start + im0->size),
					base + CAM_TOP_REGS_CAMIBEA0_OFFSET);

		if (dev->camera_mode_continuous)
			CPI_WRITE_REG(CAMICTL_BUF0_VAL_CONT,
					base + CAM_TOP_REGS_CAMICTL_OFFSET);
		else
			CPI_WRITE_REG(CAMICTL_BUF0_VAL,
					base + CAM_TOP_REGS_CAMICTL_OFFSET);

		 dev_dbg(dev->icd->parent, "set buf0\n");
	}
	if (im1) {
		CPI_WRITE_REG(im1->ls, base + CAM_TOP_REGS_CAMIBLS_OFFSET);
		CPI_WRITE_REG(im1->start, base + CAM_TOP_REGS_CAMIBSA1_OFFSET);
		CPI_WRITE_REG((im1->start + im1->size),
					base + CAM_TOP_REGS_CAMIBEA1_OFFSET);

		if (dev->camera_mode_continuous)
			CPI_WRITE_REG(CAMICTL_BUF1_VAL_CONT,
					base + CAM_TOP_REGS_CAMICTL_OFFSET);
		else
			CPI_WRITE_REG(CAMICTL_BUF1_VAL,
					base + CAM_TOP_REGS_CAMICTL_OFFSET);

		 dev_dbg(dev->icd->parent, "set buf1\n");
	}
}

void unicam_cpi_trigger_cap(struct unicam_camera_dev *dev)
{
	void __iomem *base = dev->base;
	unsigned int read_data;

	if (dev->cam_state.trigger) {
		read_data = CPI_READ_REG(base + CAM_TOP_REGS_CAMICTL_OFFSET);
		read_data = read_data | BIT(CAM_TOP_REGS_CAMICTL_TFC_SHIFT);
		CPI_WRITE_REG(read_data, base + CAM_TOP_REGS_CAMICTL_OFFSET);
	}
}
