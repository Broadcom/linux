/*
 * Copyright (C) 2015-2017 Broadcom
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

#ifndef __BCM_AMAC_REGS_H__
#define __BCM_AMAC_REGS_H__

#include <linux/types.h>

#define NICPM_ROOT			0x61030000
#define NICPM_PADRING_CFG		0x00000004
#define NICPM_IOMUX_CTRL		0x00000008

#define NICPM_PADRING_CFG_INIT_VAL	0x74000000
#define NICPM_IOMUX_CTRL_INIT_VAL	0x21880000
#define NICPM_IOMUX_CTRL_INIT_VAL_BX	0x3196e800

/* Offsets from GMAC_DEVCONTROL */
#define GMAC_INT_STATUS_REG      0x020
#define GMAC_INT_MASK_REG        0x024

#define GMAC_INTR_RX_LAZY_REG    0x100
#define GMAC_PHY_CTRL_REG        0x188

#define GMAC_DMA_TX_CTRL_REG     0x200
#define GMAC_DMA_TX_PTR_REG      0x204
#define GMAC_DMA_TX_ADDR_LO_REG  0x208
#define GMAC_DMA_TX_ADDR_HI_REG  0x20c
#define GMAC_DMA_TX_STATUS0_REG  0x210
#define GMAC_DMA_TX_STATUS1_REG  0x214

#define GMAC_DMA_RX_CTRL_REG     0x220
#define GMAC_DMA_RX_PTR_REG      0x224
#define GMAC_DMA_RX_ADDR_LO_REG  0x228
#define GMAC_DMA_RX_ADDR_HI_REG  0x22c
#define GMAC_DMA_RX_STATUS0_REG  0x230
#define GMAC_DMA_RX_STATUS1_REG  0x234

#define AMAC_TX_GOOD_OCTETS	 0x300
#define AMAC_TX_GOOD_OCTETS_HIGH 0x304
#define AMAC_TX_GOOD_PKTS	 0x308
#define AMAC_TX_OCTETS		 0x30c
#define AMAC_TX_OCTETS_HIGH	 0x310
#define AMAC_TX_PKTS		 0x314
#define AMAC_TX_BROADCAST_PKTS	 0x318
#define AMAC_TX_MULTICAST_PKTS	 0x31c
#define AMAC_TX_LEN_64		 0x320
#define AMAC_TX_LEN_65_TO_127	 0x324
#define AMAC_TX_LEN_128_TO_255	 0x328
#define AMAC_TX_LEN_256_TO_511	 0x32c
#define AMAC_TX_LEN_512_TO_1023	 0x330
#define AMAC_TX_LEN_1024_TO_1522 0x334
#define AMAC_TX_LEN_1523_TO_2047 0x338
#define AMAC_TX_LEN_2048_TO_4095 0x33c
#define AMAC_TX_LEN_4096_TO_8191 0x340
#define AMAC_TX_LEN_8192_TO_MAX	 0x344
#define AMAC_TX_JABBER_PKTS	 0x348		/* Error */
#define AMAC_TX_OVERSIZE_PKTS	 0x34c		/* Error */
#define AMAC_TX_FRAGMENT_PKTS	 0x350
#define AMAC_TX_UNDERRUNS	 0x354		/* Error */
#define AMAC_TX_TOTAL_COLS	 0x358
#define AMAC_TX_SINGLE_COLS	 0x35c
#define AMAC_TX_MULTIPLE_COLS	 0x360
#define AMAC_TX_EXCESSIVE_COLS	 0x364		/* Error */
#define AMAC_TX_LATE_COLS	 0x368		/* Error */
#define AMAC_TX_DEFERED		 0x36c
#define AMAC_TX_CARRIER_LOST	 0x370
#define AMAC_TX_PAUSE_PKTS	 0x374
#define AMAC_TX_UNI_PKTS	 0x378
#define AMAC_TX_Q0_PKTS		 0x37c
#define AMAC_TX_Q0_OCTETS	 0x380
#define AMAC_TX_Q0_OCTETS_HIGH	 0x384
#define AMAC_TX_Q1_PKTS		 0x388
#define AMAC_TX_Q1_OCTETS	 0x38c
#define AMAC_TX_Q1_OCTETS_HIGH	 0x390
#define AMAC_TX_Q2_PKTS		 0x394
#define AMAC_TX_Q2_OCTETS	 0x398
#define AMAC_TX_Q2_OCTETS_HIGH	 0x39c
#define AMAC_TX_Q3_PKTS		 0x3a0
#define AMAC_TX_Q3_OCTETS	 0x3a4
#define AMAC_TX_Q3_OCTETS_HIGH	 0x3a8
#define AMAC_RX_GOOD_OCTETS	 0x3b0
#define AMAC_RX_GOOD_OCTETS_HIGH 0x3b4
#define AMAC_RX_GOOD_PKTS	 0x3b8
#define AMAC_RX_OCTETS		 0x3bc
#define AMAC_RX_OCTETS_HIGH	 0x3c0
#define AMAC_RX_PKTS		 0x3c4
#define AMAC_RX_BROADCAST_PKTS	 0x3c8
#define AMAC_RX_MULTICAST_PKTS	 0x3cc
#define AMAC_RX_LEN_64		 0x3d0
#define AMAC_RX_LEN_65_TO_127	 0x3d4
#define AMAC_RX_LEN_128_TO_255	 0x3d8
#define AMAC_RX_LEN_256_TO_511	 0x3dc
#define AMAC_RX_LEN_512_TO_1023	 0x3e0
#define AMAC_RX_LEN_1024_TO_1522 0x3e4
#define AMAC_RX_LEN_1523_TO_2047 0x3e8
#define AMAC_RX_LEN_2048_TO_4095 0x3ec
#define AMAC_RX_LEN_4096_TO_8191 0x3f0
#define AMAC_RX_LEN_8192_TO_MAX	 0x3f4
#define AMAC_RX_JABBER_PKTS	 0x3f8		/* Error */
#define AMAC_RX_OVERSIZE_PKTS	 0x3fc		/* Error */
#define AMAC_RX_FRAGMENT_PKTS	 0x400
#define AMAC_RX_MISSED_PKTS	 0x404		/* Error */
#define AMAC_RX_CRC_ALIGN_ERRS	 0x408		/* Error */
#define AMAC_RX_UNDERSIZE	 0x40c		/* Error */
#define AMAC_RX_CRC_ERRS	 0x410		/* Error */
#define AMAC_RX_ALIGN_ERRS	 0x414		/* Error */
#define AMAC_RX_SYMBOL_ERRS	 0x418		/* Error */
#define AMAC_RX_PAUSE_PKTS	 0x41c
#define AMAC_RX_NONPAUSE_PKTS	 0x420
#define AMAC_RX_SACHANGES	 0x424
#define AMAC_RX_UNI_PKTS	 0x428
#define AMAC_NUM_MIB_TX_REGS	\
		(((AMAC_TX_Q3_OCTETS_HIGH - AMAC_TX_GOOD_OCTETS) / 4) + 1)
#define AMAC_NUM_MIB_RX_REGS	\
		(((AMAC_RX_UNI_PKTS - AMAC_RX_GOOD_OCTETS) / 4) + 1)

#define UNIMAC_CMD_CFG_REG       0x808

#define UNIMAC_MACADDR_HIGH	 0x80c	/* High 4 octets of own mac address */
#define UNIMAC_MACADDR_LOW	 0x810	/* Low 2 octets of own mac address */

#define GMAC0_IRL_FRAMECOUNT_SHIFT  24

/* PHY registers */
#define GPHY_MII_CTRL_REG           0x00
#define GPHY_MII_CTRL_REG_PWR_MASK  0x800
#define GPHY_MII_CTRL_REG_RST_MASK  0x8000

#define GPHY_EXP_DATA_REG           0x15
#define GPHY_EXP_SELECT_REG         0x17
#define GPHY_MISC_CTRL_REG          0x18  /* shadow 7 */
#define GPHY_CLK_ALIGNCTRL_REG      0x1C  /* Shadow 3 */

/* Initialization values of above PHY registers */
#define GPHY_EXP_DATA_REG_VAL                  0x11B
#define GPHY_EXP_SELECT_REG_VAL_LANE_SWAP      0x0F09
#define GPHY_EXP_SELECT_REG_VAL_BROADREACH_OFF 0x0F90
#define GPHY_MISC_CTRL_REG_SKEW_DISABLE_VAL    0xF0E7
#define GPHY_CLK_GTX_DELAY_DISALE_WR_VAL       0x8c00
#define GPHY_MISC_CTRL_REG_DELAY_DISABLE_VAL   0x7007
#define GPHY_CLK_GTX_DELAY_DISALE_RD_VAL       0x0c00

/* AMAC IDM Registers */
#define AMAC_IDM0_IO_CTRL_REG 0x8
#define AMAC_IDM0_IO_CTRL_DEST_SYNC_MODE_EN_BIT 3
#define AMAC_IDM0_IO_CTRL_GMII_MODE_BIT 5
#define AMAC_IDM0_IO_CTRL_CLK_250_SEL_BIT 6

#define AMAC_IDM_RST_CTRL_REG 0x400

/* Offsets from Switch Global Config registers */
#define CDRU_SWITCH_CFG_BYPASS_SWITCH 0xD

/* Offsets from CRMU Chip IO Pad Control */
#define CRMU_CHIP_IO_PAD_CONTROL__CDRU_IOMUX_FORCE_PAD_IN 0

/* register-specific flag definitions */
/* device control */
#define DC_TSM          0x00000002
#define DC_CFCO         0x00000004
#define DC_RLSS         0x00000008
#define DC_MROR         0x00000010
#define DC_FCM_MASK	    0x00000060
#define DC_FCM_SHIFT    5
#define DC_NAE          0x00000080
#define DC_TF           0x00000100
#define DC_RDS_MASK     0x00030000
#define DC_RDS_SHIFT    16
#define DC_TDS_MASK     0x000c0000
#define DC_TDS_SHIFT    18

/* interrupt status and mask registers */
#define I_MRO      0x00000001
#define I_MTO      0x00000002
#define I_TFD      0x00000004
#define I_LS       0x00000008
#define I_MDIO     0x00000010
#define I_MR       0x00000020
#define I_MT       0x00000040
#define I_TO       0x00000080
#define I_PDEE     0x00000400
#define I_PDE      0x00000800
#define I_DE       0x00001000
#define I_RDU      0x00002000
#define I_RFO      0x00004000
#define I_XFU      0x00008000
#define I_RI       0x00010000
#define I_XI0      0x01000000
#define I_XI1      0x02000000
#define I_XI2      0x04000000
#define I_XI3      0x08000000
#define I_INTMASK  0x0f01fcff
#define I_ERRMASK  0x0000fc00
#define I_XI_ALL   (I_XI0 | I_XI1 | I_XI2 | I_XI3)

/* phy control */
#define PC_EPA_MASK   0x0000001f
#define PC_MCT_MASK   0x007f0000
#define PC_MCT_SHIFT  16
#define PC_MTE        0x00800000
/* command config */
#define CC_TE        0x00000001
#define CC_RE        0x00000002
#define CC_ES_MASK   0x0000000c
#define CC_ES_SHIFT  2
#define CC_PROM      0x00000010
#define CC_PAD_EN    0x00000020
#define CC_CF        0x00000040
#define CC_PF        0x00000080
#define CC_RPI       0x00000100
#define CC_TAI       0x00000200
#define CC_HD        0x00000400
#define CC_HD_SHIFT  10
#define CC_SR        0x00002000
#define CC_ML        0x00008000
#define CC_AE        0x00400000
#define CC_CFE       0x00800000
#define CC_NLC       0x01000000
#define CC_RL        0x02000000
#define CC_RED       0x04000000
#define CC_PE        0x08000000
#define CC_TPI       0x10000000

/* DMA specific bits */

/* transmit channel control */
#define D64_XC_XE       0x00000001 /* transmit enable */
#define D64_XC_SE       0x00000002 /* transmit suspend request */
#define D64_XC_LE       0x00000004 /* loopback enable */
#define D64_XC_FL       0x00000010 /* flush request */
#define D64_XC_MR_MASK  0x000000C0 /* Multiple outstanding reads */
#define D64_XC_MR_SHIFT 6
#define D64_XC_PD       0x00000800 /* parity check disable */
#define D64_XC_AE       0x00030000 /* address extension bits */
#define D64_XC_AE_SHIFT 16
#define D64_XC_BL_MASK  0x001C0000 /* BurstLen bits */
#define D64_XC_BL_SHIFT 18
#define D64_XC_PC_MASK  0x00E00000 /* Prefetch control */
#define D64_XC_PC_SHIFT 21
#define D64_XC_PT_MASK  0x03000000 /* Prefetch threshold */
#define D64_XC_PT_SHIFT 24

/* transmit descriptor table pointer */
#define D64_XP_LD_MASK  0x00001fff /* last valid descriptor */

/* transmit channel status */
#define D64_XS0_CD_MASK     0x00001fff /* current descriptor pointer */
#define D64_XS0_XS_MASK     0xf0000000 /* transmit state */
#define D64_XS0_XS_SHIFT    28
#define D64_XS0_XS_DISABLED 0x00000000 /* disabled */
#define D64_XS0_XS_ACTIVE   0x10000000 /* active */
#define D64_XS0_XS_IDLE     0x20000000 /* idle wait */
#define D64_XS0_XS_STOPPED  0x30000000 /* stopped */
#define D64_XS0_XS_SUSP     0x40000000 /* suspend pending */

#define D64_XS1_AD_MASK     0x00001fff /* active descriptor */
#define D64_XS1_XE_MASK	    0xf0000000 /* transmit errors */
#define D64_XS1_XE_SHIFT    28
#define D64_XS1_XE_NOERR    0x00000000 /* no error */
#define D64_XS1_XE_DPE      0x10000000 /* descriptor protocol error */
#define D64_XS1_XE_DFU      0x20000000 /* data fifo underrun */
#define D64_XS1_XE_DTE      0x30000000 /* data transfer error */
#define D64_XS1_XE_DESRE    0x40000000 /* descriptor read error */
#define D64_XS1_XE_COREE    0x50000000 /* core error */

/* receive channel control */
#define D64_RC_RE       0x00000001 /* receive enable */
#define D64_RC_RO_MASK  0x000000fe /* receive frame offset */
#define D64_RC_RO_SHIFT 1
#define D64_RC_FM 0x00000100 /* direct fifo receive (pio) mode */
#define D64_RC_SH 0x00000200 /* separate rx header descriptor enable */
#define D64_RC_OC 0x00000400 /* overflow continue */
#define D64_RC_PD 0x00000800 /* parity check disable */
#define D64_RC_GE 0x00004000 /* Glom enable */
#define D64_RC_AE 0x00030000 /* address extension bits */
#define D64_RC_AE_SHIFT 16
#define D64_RC_BL_MASK  0x001C0000 /* BurstLen bits */
#define D64_RC_BL_SHIFT 18
#define D64_RC_PC_16_DESC 0x3
#define D64_RC_PC_8_DESC 0x2
#define D64_RC_PC_4_DESC 0x1
#define D64_RC_PC_MASK  0x00E00000 /* Prefetch control */
#define D64_RC_PC_SHIFT 21
#define D64_RC_PT_MASK  0x03000000 /* Prefetch threshold */
#define D64_RC_PT_SHIFT 24

/* flags for dma controller */
#define DMA_CTRL_RX_PEN BIT(0) /* rx partity enable */
#define DMA_CTRL_TX_PEN BIT(1) /* tx partity enable */
#define DMA_CTRL_RX_ROC BIT(2) /* rx overflow continue */
#define DMA_CTRL_RX_PC  BIT(3) /* RX Prefetch Control */
#define DMA_CTRL_TX_PC  BIT(4) /* TX Prefetch Control */

/* receive channel status */
#define D64_RS0_CD_MASK  0x00001fff /* current descriptor pointer */
#define D64_RS0_RS_MASK  0xf0000000 /* receive state */
#define D64_RS0_RS_SHIFT    28
#define D64_RS0_RS_DISABLED 0x00000000 /* disabled */
#define D64_RS0_RS_ACTIVE   0x10000000 /* active */
#define D64_RS0_RS_IDLE     0x20000000 /* idle wait */
#define D64_RS0_RS_STOPPED  0x30000000 /* stopped */
#define D64_RS0_RS_SUSP     0x40000000 /* suspend pending */

#define D64_RS1_AD_MASK   0x0001ffff /* active descriptor */
#define D64_RS1_RE_MASK   0xf0000000 /* receive errors */
#define D64_RS1_RE_SHIFT  28
#define D64_RS1_RE_NOERR  0x00000000 /* no error */
#define D64_RS1_RE_DPO    0x10000000 /* descriptor protocol error */
#define D64_RS1_RE_DFU    0x20000000 /* data fifo overflow */
#define D64_RS1_RE_DTE    0x30000000 /* data transfer error */
#define D64_RS1_RE_DESRE  0x40000000 /* descriptor read error */
#define D64_RS1_RE_COREE  0x50000000 /* core error */

/* descriptor control flags 1 */
#define D64_CTRL_COREFLAGS 0x0ff00000 /* core specific */
#define D64_CTRL1_EOT ((unsigned int)BIT(28)) /* end of descriptor table */
#define D64_CTRL1_IOC ((unsigned int)BIT(29)) /* interrupt on completion */
#define D64_CTRL1_EOF ((unsigned int)BIT(30)) /* end of frame */
#define D64_CTRL1_SOF ((unsigned int)BIT(31)) /* start of frame */

/* descriptor control flags 2 */
#define D64_CTRL2_BC_MASK  0x00007fff /* buff byte cnt.real data len <= 16KB */
#define D64_CTRL2_AE       0x00030000 /* address extension bits */
#define D64_CTRL2_AE_SHIFT 16
#define D64_CTRL2_PARITY   0x00040000      /* parity bit */

/* control flags in the range [27:20] are core-specific and not defined here */
#define D64_CTRL_CORE_MASK  0x0ff00000

#define D64_RX_FRM_STS_LEN  0x0000ffff /* frame length mask */
#define D64_RX_FRM_STS_OVFL 0x00800000 /* RxOverFlow */

/* no. of descp used - 1, d11corerev >= 22 */
#define D64_RX_FRM_STS_DSCRCNT 0x0f000000

#define D64_RX_FRM_STS_DATATYPE 0xf0000000 /* core-dependent data type */

#define HWRXOFF    30

#endif /*__BCM_AMAC_REGS_H__ */

