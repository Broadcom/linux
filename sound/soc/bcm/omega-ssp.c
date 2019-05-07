// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Broadcom
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#include "omega-ssp.h"
#include "iproc-pcm.h"

/* Identifies which io map to the reg offsets is to be used with */
#define MAP1_FLAG  0x10000
#define MAP2_FLAG  0x20000
#define MAP3_FLAG  0x30000

/* Begin register offset defines */
#define IOMAP1_BASE_OFFSET      0x000

#define AUD_MISC_SEROUT_OE_REG_BASE     (0x01c | MAP1_FLAG)
#define AUD_MISC_SEROUT_SPDIF_OE  12
#define AUD_MISC_SEROUT_MCLK_OE    3
#define AUD_MISC_SEROUT_LRCK_OE    2
#define AUD_MISC_SEROUT_SCLK_OE    1
#define AUD_MISC_SEROUT_SDAT_OE    0

/* AUD_FMM_BF_CTRL_xxx regs */
#define BF_DST_CFG0_OFFSET              (0x100 | MAP1_FLAG)
#define BF_DST_CFG1_OFFSET              (0x104 | MAP1_FLAG)
#define BF_DST_CFG2_OFFSET              (0x108 | MAP1_FLAG)

#define BF_DST_CTRL0_OFFSET             (0x130 | MAP1_FLAG)
#define BF_DST_CTRL1_OFFSET             (0x134 | MAP1_FLAG)
#define BF_DST_CTRL2_OFFSET             (0x138 | MAP1_FLAG)

#define BF_SRC_CFG0_OFFSET              (0x148 | MAP1_FLAG)
#define BF_SRC_CFG1_OFFSET              (0x14c | MAP1_FLAG)
#define BF_SRC_CFG2_OFFSET              (0x150 | MAP1_FLAG)
#define BF_SRC_CFG3_OFFSET              (0x154 | MAP1_FLAG)

#define BF_SRC_CTRL0_OFFSET             (0x1c0 | MAP1_FLAG)
#define BF_SRC_CTRL1_OFFSET             (0x1c4 | MAP1_FLAG)
#define BF_SRC_CTRL2_OFFSET             (0x1c8 | MAP1_FLAG)
#define BF_SRC_CTRL3_OFFSET             (0x1cc | MAP1_FLAG)

#define BF_SRC_GRP0_OFFSET              (0x1fc | MAP1_FLAG)
#define BF_SRC_GRP1_OFFSET              (0x200 | MAP1_FLAG)
#define BF_SRC_GRP2_OFFSET              (0x204 | MAP1_FLAG)
#define BF_SRC_GRP3_OFFSET              (0x208 | MAP1_FLAG)

#define BF_SRC_GRP_EN_OFFSET            (0x320 | MAP1_FLAG)
#define BF_SRC_GRP_FLOWON_OFFSET        (0x324 | MAP1_FLAG)
#define BF_SRC_GRP_SYNC_DIS_OFFSET      (0x328 | MAP1_FLAG)

/*--------------------------------------------
 * Register offsets shared io space
 */
#define IOMAP2_BASE_OFFSET      0xa00

/* AUD_FMM_IOP_OUT_I2S_xxx regs */
#define OUT_I2S_0_STREAM_CFG_OFFSET     (0xa00 | MAP2_FLAG)
#define OUT_I2S_0_CFG_OFFSET            (0xa04 | MAP2_FLAG)
#define OUT_I2S_0_MCLK_CFG_OFFSET       (0xa14 | MAP2_FLAG)

#define OUT_I2S_1_STREAM_CFG_OFFSET     (0xa50 | MAP2_FLAG)
#define OUT_I2S_1_CFG_OFFSET            (0xa54 | MAP2_FLAG)
#define OUT_I2S_1_MCLK_CFG_OFFSET       (0xa64 | MAP2_FLAG)

#define OUT_I2S_2_STREAM_CFG_OFFSET     (0xaa0 | MAP2_FLAG)
#define OUT_I2S_2_CFG_OFFSET            (0xaa4 | MAP2_FLAG)
#define OUT_I2S_2_MCLK_CFG_OFFSET       (0xab4 | MAP2_FLAG)

/* AUD_FMM_IOP_OUT_SPDIF_xxx regs */
#define SPDIF_STREAM_CFG_OFFSET         (0xaf0 | MAP2_FLAG)
#define SPDIF_CTRL_OFFSET               (0xaf4 | MAP2_FLAG)
#define SPDIF_FORMAT_CFG_OFFSET         (0xb10 | MAP2_FLAG)
#define SPDIF_MCLK_CFG_OFFSET           (0xb14 | MAP2_FLAG)

/*--------------------------------------------
 * Register offsets for i2s_in io space
 */
/* AUD_FMM_IOP_IN_I2S_xxx regs */
#define IOMAP3_BASE_OFFSET      0xd70

#define IN_I2S_0_STREAM_CFG_OFFSET      (0xd70 | MAP3_FLAG)
#define IN_I2S_0_CFG_OFFSET             (0xd74 | MAP3_FLAG)
#define IN_I2S_0_MCLK_CFG_OFFSET        (0xd78 | MAP3_FLAG)
#define IN_I2S_0_STATUS                 (0xd90 | MAP3_FLAG)

#define IN_I2S_1_STREAM_CFG_OFFSET      (0xdb0 | MAP3_FLAG)
#define IN_I2S_1_CFG_OFFSET             (0xdb4 | MAP3_FLAG)
#define IN_I2S_1_MCLK_CFG_OFFSET        (0xdb8 | MAP3_FLAG)
#define IN_I2S_1_STATUS                 (0xdd0 | MAP3_FLAG)

#define IN_I2S_2_STREAM_CFG_OFFSET      (0xdf0 | MAP3_FLAG)
#define IN_I2S_2_CFG_OFFSET             (0xdf4 | MAP3_FLAG)
#define IN_I2S_2_MCLK_CFG_OFFSET        (0xdf8 | MAP3_FLAG)
#define IN_I2S_2_STATUS                 (0xe10 | MAP3_FLAG)

/* AUD_FMM_IOP_MISC_xxx regs */
#define IOP_SW_INIT_LOGIC               (0xf80 | MAP3_FLAG)

/* End register offset defines */


/* AUD_FMM_IOP_OUT_I2S_x_MCLK_CFG_0_REG */
#define I2S_OUT_MCLK_DIV1      24
#define I2S_OUT_MCLKRATE_SHIFT 16  /* 8 bits, 16-23 */
#define I2S_OUT_MCLKRATE_MASK \
	GENMASK(I2S_OUT_MCLKRATE_SHIFT + 7, I2S_OUT_MCLKRATE_SHIFT)

#define I2S_IN_MCLKRATE_SHIFT 16
#define I2S_IN_MCLKRATE_MASK \
	GENMASK(I2S_IN_MCLKRATE_SHIFT + 7, I2S_IN_MCLKRATE_SHIFT)

/* AUD_FMM_IOP_OUT_I2S_x_MCLK_CFG_REG */
#define I2S_OUT_PLLCLKSEL_SHIFT  0
#define I2S_IN_PLLCLKSEL_SHIFT  0

/* AUD_FMM_IOP_OUT_I2S_x_STREAM_CFG */
#define I2S_OUT_STREAM_CFG_ENA           31
#define I2S_OUT_STREAM_CFG_CH_GROUP      25  /* 4 bits, 25-28 */
#define I2S_OUT_STREAM_CFG_CH_GROUP_MASK \
	GENMASK(I2S_OUT_STREAM_CFG_CH_GROUP + 3, I2S_OUT_STREAM_CFG_CH_GROUP)
#define I2S_OUT_STREAM_CFG_GROUP_ID      21  /* 4 bits, 21-24 */
#define I2S_OUT_STREAM_CFG_GROUP_ID_MASK \
	GENMASK(I2S_OUT_STREAM_CFG_GROUP_ID + 3, I2S_OUT_STREAM_CFG_GROUP_ID)
#define I2S_OUT_STREAM_CFG_FCI_ID         0  /* 10 bits, 0-9 */
#define I2S_OUT_STREAM_CFG_FCI_ID_MASK \
	GENMASK(I2S_OUT_STREAM_CFG_FCI_ID + 9, I2S_OUT_STREAM_CFG_FCI_ID)

/* AUD_FMM_IOP_IN_I2S_x_CAP */
#define I2S_IN_STREAM_CFG_CAP_ENA   31
#define I2S_IN_STREAM_CFG_GROUP_ID   4
#define I2S_IN_STREAM_CFG_GROUP_ID_MASK  \
	GENMASK(I2S_IN_STREAM_CFG_GROUP_ID + 3, I2S_IN_STREAM_CFG_GROUP_ID)

/* AUD_FMM_IOP_OUT_I2S_x_I2S_CFG_REG */
#define I2S_OUT_CFGX_LRCK_POLARITY   0
#define I2S_OUT_CFGX_SCLK_POLARITY   1
#define I2S_OUT_CFGX_DATA_ALIGNMENT  2
#define I2S_OUT_CFGX_DATA_ALIGNMENT_MASK  (BIT(2) | BIT(3))
#define I2S_OUT_CFGX_DATA_JUSTIFICATION  4
#define I2S_OUT_CFGX_BITS_PER_SAMPLE 5  /* 5 bits, 5-9 */
#define I2S_OUT_CFGX_BIT_PER_SAMPLE_MASK \
	GENMASK(I2S_OUT_CFGX_BITS_PER_SAMPLE + 4, I2S_OUT_CFGX_BITS_PER_SAMPLE)
#define I2S_OUT_CFGX_BITS_PER_SLOT  10
#define I2S_OUT_CFGX_VALID_SLOT     12  /* 4 bits, 12-15 */
#define I2S_OUT_CFGX_VALID_SLOT_MASK \
	GENMASK(I2S_OUT_CFGX_VALID_SLOT + 3, I2S_OUT_CFGX_VALID_SLOT)
#define I2S_OUT_CFGX_FSYNC_WIDTH    16  /* 9 bits, 16-24 */
#define I2S_OUT_CFGX_FSYNC_WIDTH_MASK \
	GENMASK(I2S_OUT_CFGX_FSYNC_WIDTH + 8, I2S_OUT_CFGX_FSYNC_WIDTH)
#define I2S_OUT_CFGX_SCLKS_PER_1FS_DIV32 25  /* 4 bits, 25-28 */
#define I2S_OUT_CFGX_SCLKS_PER_1FS_DIV32_MASK \
	GENMASK(I2S_OUT_CFGX_SCLKS_PER_1FS_DIV32 + 3, \
		I2S_OUT_CFGX_SCLKS_PER_1FS_DIV32)
#define I2S_OUT_CFGX_SLAVE_MODE     30
#define I2S_OUT_CFGX_TDM_MODE       31

#define I2S_IN_CFGX_DATA_ALIGNMENT   2
#define I2S_IN_CFGX_BITS_PER_SAMPLE  5  /* 5 bits, 5-9 */
#define I2S_IN_CFGX_BIT_PER_SAMPLE_MASK \
	GENMASK(I2S_IN_CFGX_BITS_PER_SAMPLE + 4, I2S_IN_CFGX_BITS_PER_SAMPLE)
#define I2S_IN_CFGX_BITS_PER_SLOT   10
#define I2S_IN_CFGX_VALID_SLOT      12  /* 4 bits, 12-15 */
#define I2S_IN_CFGX_VALID_SLOT_MASK \
	GENMASK(I2S_IN_CFGX_VALID_SLOT + 3, I2S_IN_CFGX_VALID_SLOT)
#define I2S_IN_CFGX_SLAVE_MODE      30
#define I2S_IN_CFGX_TDM_MODE        31

/* AUD_FMM_BF_CTRL_SOURCECH_CFGx_REG */
#define BF_SRC_CFGX_SFIFO_ENA              0
#define BF_SRC_CFGX_BUFFER_PAIR_ENABLE     1
#define BF_SRC_CFGX_SAMPLE_CH_MODE         2
#define BF_SRC_CFGX_SFIFO_SZ_DOUBLE        5
#define BF_SRC_CFGX_NOT_PAUSE_WHEN_EMPTY   9
#define BF_SRC_CFGX_SAMPLE_REPEAT_ENABLE  10
#define BF_SRC_CFGX_BIT_RES               20  /* 5 bits, 20-24 */
#define BF_SRC_CFGX_BITRES_MASK  \
	GENMASK(BF_SRC_CFGX_BIT_RES + 4, BF_SRC_CFGX_BIT_RES)
#define BF_SRC_CFGX_PROCESS_SEQ_ID_VALID  31

/* AUD_FMM_BF_CTRL_SOURCECH_CTRLx_REG */
#define BF_SOURCECH_CTRL_PLAY_RUN   0

/* AUD_FMM_BF_CTRL_SOURCECH_GRPX */
#define BF_SRC_GRPX_GROUP_ID 0
#define BF_SRC_GRPX_GROUP_ID_MASK \
	GENMASK(BF_SRC_GRPX_GROUP_ID + 2, BF_SRC_GRPX_GROUP_ID)

/* AUD_FMM_BF_CTRL_DESTCH_CFGx_REG */
#define BF_DST_CFGX_CAP_ENA              0
#define BF_DST_CFGX_BUFFER_PAIR_ENABLE   1
#define BF_DST_CFGX_DFIFO_SZ_DOUBLE      2
#define BF_DST_CFGX_NOT_PAUSE_WHEN_FULL 11
#define BF_DST_CFGX_FCI_ID              12  /* 10 bits, 12-21 */
#define BF_DST_CFGX_FCI_ID_MASK \
	GENMASK(BF_DST_CFGX_FCI_ID + 9, BF_DST_CFGX_FCI_ID)
#define BF_DST_CFGX_CAP_MODE            23
#define BF_DST_CFGX_PROC_SEQ_ID_VALID   31

/* AUD_FMM_BF_CTRL_DESTCH_CTRLX */
#define BF_DESTCH_CTRLX_CAP_RUN  0x1

/* AUD_FMM_IOP_OUT_SPDIF_STREAM_CFG */
#define SPDIF_STREAM_CFG_ENA            31
#define SPDIF_STREAM_CFG_FCI_ID_MASK    GENMASK(9, 0)

/* AUD_FMM_IOP_OUT_SPDIF_CTRL */
#define SPDIF_CTRL_DITHER_ENA     3

#define INIT_SSP_REGS(num) (struct aio_ssp_regs){ \
		.i2s_stream_cfg = OUT_I2S_ ##num## _STREAM_CFG_OFFSET, \
		.i2s_cap_stream_cfg = IN_I2S_ ##num## _STREAM_CFG_OFFSET, \
		.i2s_cfg = OUT_I2S_ ##num## _CFG_OFFSET, \
		.i2s_cap_cfg = IN_I2S_ ##num## _CFG_OFFSET, \
		.i2s_mclk_cfg = OUT_I2S_ ##num## _MCLK_CFG_OFFSET, \
		.i2s_cap_status = IN_I2S_ ##num## _STATUS, \
		.bf_destch_ctrl = BF_DST_CTRL ##num## _OFFSET, \
		.bf_destch_cfg = BF_DST_CFG ##num## _OFFSET, \
		.bf_sourcech_ctrl = BF_SRC_CTRL ##num## _OFFSET, \
		.bf_sourcech_cfg = BF_SRC_CFG ##num## _OFFSET \
}


#define CAPTURE_FCI_ID_BASE 0x181
#define SSP_TRISTATE_MASK 0x001fff

/* Used with stream_on field to indicate which streams are active */
#define PLAYBACK_STREAM_MASK   BIT(0)
#define CAPTURE_STREAM_MASK    BIT(1)

#define CH_GRP_STEREO            0x1

#define SSP_RATE_MIN     8000
#define SSP_RATE_MAX   384000

#define AIO_MAX_PLAYBACK_PORTS	4
#define AIO_MAX_CAPTURE_PORTS	3
#define AIO_MAX_I2S_PORTS	3
#define AIO_MAX_PORTS		AIO_MAX_PLAYBACK_PORTS

#define SSPMODE_I2S 0
#define SSPMODE_TDM 1
#define SSPMODE_UNKNOWN -1


enum audio_port_type {
	PORT_TDM,
	PORT_SPDIF,
};

struct aio_ssp_regs {
	u32 i2s_stream_cfg;
	u32 i2s_cfg;
	u32 i2s_mclk_cfg;

	u32 i2s_cap_stream_cfg;
	u32 i2s_cap_cfg;
	u32 i2s_cap_mclk_cfg;
	u32 i2s_cap_status;

	u32 bf_destch_ctrl;
	u32 bf_destch_cfg;
	u32 bf_sourcech_ctrl;
	u32 bf_sourcech_cfg;
};

struct audio_io {
	struct regmap *audio;
	struct regmap *cmn_io;
	struct regmap *i2s_in;
};

struct audio_clkinfo {
	struct clk *audio_clk;
};

struct aio_port {
	struct device *dev;

	int portnum;
	int mode;
	bool is_slave;
	int streams_on;   /* will be 0 if both capture and play are off */
	int port_type;

	unsigned int fsync_width;
	unsigned int fs_delay;
	bool invert_bclk;
	bool invert_fs;

	u32 mclk;
	u32 lrclk;

	unsigned int slot_width;
	unsigned int slots_per_frame;
	unsigned int active_slots;

	struct audio_io *io;
	struct aio_ssp_regs regs;
	struct audio_clkinfo clk_info;
};

struct omega_audio {
	struct aio_port		portinfo[AIO_MAX_PORTS];
	struct iproc_rb_info	rb_info;
	struct iproc_pcm_dma_info   dma_info_play[AIO_MAX_PLAYBACK_PORTS];
	struct iproc_pcm_dma_info   dma_info_cap[AIO_MAX_CAPTURE_PORTS];

	struct audio_io io;
	struct device *dev;

	u32 oe_reg_context;
};

/* List of valid frame sizes for tdm mode */
static const int ssp_valid_tdm_framesize[] = {32, 64, 128, 256, 512};

static const unsigned int ssp_rates[] = {
	 8000, 11025,  16000,  22050,  32000,  44100, 48000,
	88200, 96000, 176400, 192000, 352800, 384000
};

static const struct snd_pcm_hw_constraint_list ssp_rate_constraint = {
	.count = ARRAY_SIZE(ssp_rates),
	.list = ssp_rates,
};

static void update_ssp_cfg(struct aio_port *aio);


struct reg_desc {
	struct regmap *iomap;
	unsigned int io_offset;
};

static int audio_get_iomap(struct audio_io *io,
			    unsigned int reg_code, struct reg_desc *desc)
{
	unsigned int  map_idx;
	unsigned int  full_offset;

	map_idx     = reg_code & 0xFFFF0000;
	full_offset = reg_code & 0x0000FFFF;

	switch (map_idx) {
	case MAP1_FLAG:
		desc->iomap = io->audio;
		desc->io_offset = full_offset - IOMAP1_BASE_OFFSET;
		break;
	case MAP2_FLAG:
		desc->iomap = io->cmn_io;
		desc->io_offset = full_offset - IOMAP2_BASE_OFFSET;
		break;
	case MAP3_FLAG:
		desc->iomap = io->i2s_in;
		desc->io_offset = full_offset - IOMAP3_BASE_OFFSET;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sspreg_update(struct audio_io *io, unsigned int offset,
			 u32 mask, u32 new_val)
{
	struct reg_desc desc;
	int ret;

	ret = audio_get_iomap(io, offset, &desc);
	if (ret)
		return ret;

	regmap_update_bits(desc.iomap, desc.io_offset, mask, new_val);

	return 0;
}

/* sspreg_clr_bits will set all the bits specified by "mask" to 0 */
static int sspreg_clr_bits(struct audio_io *io, unsigned int offset, u32 mask)
{
	return sspreg_update(io, offset, mask, 0);
}


/* sspreg_set_bits will set all the bits specified by "mask" to 1 */
static int sspreg_set_bits(struct audio_io *io, unsigned int offset, u32 mask)
{
	return sspreg_update(io, offset, mask, mask);
}

static int sspreg_read(struct audio_io *io, unsigned int offset, u32 *val)
{
	struct reg_desc desc;
	int ret;

	ret = audio_get_iomap(io, offset, &desc);
	if (ret)
		return ret;

	regmap_read(desc.iomap, desc.io_offset, val);
	return 0;
}

static int sspreg_write(struct audio_io *io, unsigned int offset, u32 val)
{
	struct reg_desc desc;
	int ret;

	ret = audio_get_iomap(io, offset, &desc);
	if (ret)
		return ret;

	regmap_write(desc.iomap, desc.io_offset, val);
	return 0;
}

static struct aio_port *aio_dai_get_portinfo(struct snd_soc_dai *dai)
{
	struct omega_audio *aiotop = snd_soc_dai_get_drvdata(dai);

	return &aiotop->portinfo[dai->id];
}

static int audio_ssp_init_portregs(struct aio_port *aio)
{
	u32 val, fci_id, mask;
	int status = 0;

	/* Set Group ID */
	mask = BF_SRC_GRPX_GROUP_ID_MASK;
	sspreg_update(aio->io, BF_SRC_GRP0_OFFSET, mask, 0);
	sspreg_update(aio->io, BF_SRC_GRP1_OFFSET, mask, 1);
	sspreg_update(aio->io, BF_SRC_GRP2_OFFSET, mask, 2);
	sspreg_update(aio->io, BF_SRC_GRP3_OFFSET, mask, 3);

	switch (aio->port_type) {
	case PORT_TDM:
		/* Configure the AUD_FMM_IOP_OUT_I2S_x_STREAM_CFG reg */
		mask = I2S_OUT_STREAM_CFG_GROUP_ID_MASK;
		mask |= I2S_OUT_STREAM_CFG_FCI_ID_MASK;
		mask |= I2S_OUT_STREAM_CFG_CH_GROUP_MASK;
		val = aio->portnum << I2S_OUT_STREAM_CFG_GROUP_ID;
		/* FCI ID is the port num */
		val |= (aio->portnum << I2S_OUT_STREAM_CFG_FCI_ID);
		val |= (CH_GRP_STEREO << I2S_OUT_STREAM_CFG_CH_GROUP);
		sspreg_update(aio->io, aio->regs.i2s_stream_cfg, mask, val);

		/* Configure the AUD_FMM_BF_CTRL_SOURCECH_CFGX reg */
		mask = BIT(BF_SRC_CFGX_NOT_PAUSE_WHEN_EMPTY);
		mask |=	BIT(BF_SRC_CFGX_SAMPLE_REPEAT_ENABLE);
		mask |= BIT(BF_SRC_CFGX_SFIFO_SZ_DOUBLE);
		mask |= BIT(BF_SRC_CFGX_PROCESS_SEQ_ID_VALID);
		val = BIT(BF_SRC_CFGX_SFIFO_SZ_DOUBLE);
		val |= BIT(BF_SRC_CFGX_PROCESS_SEQ_ID_VALID);
		sspreg_update(aio->io, aio->regs.bf_sourcech_cfg, mask, val);

		/* Configure the AUD_FMM_IOP_IN_I2S_x_CAP_STREAM_CFG_0 reg */
		mask = I2S_IN_STREAM_CFG_GROUP_ID_MASK;
		val = aio->portnum << I2S_IN_STREAM_CFG_GROUP_ID;
		sspreg_update(aio->io, aio->regs.i2s_cap_stream_cfg, mask, val);

		/* Configure the AUD_FMM_BF_CTRL_DESTCH_CFGX_REG_BASE reg */
		mask = BF_DST_CFGX_FCI_ID_MASK;
		mask |= BIT(BF_DST_CFGX_DFIFO_SZ_DOUBLE);
		mask |= BIT(BF_DST_CFGX_NOT_PAUSE_WHEN_FULL);
		mask |= BIT(BF_DST_CFGX_PROC_SEQ_ID_VALID);

		fci_id = CAPTURE_FCI_ID_BASE + aio->portnum;
		val = BIT(BF_DST_CFGX_DFIFO_SZ_DOUBLE);
		val |= (fci_id << BF_DST_CFGX_FCI_ID);
		val |= BIT(BF_DST_CFGX_PROC_SEQ_ID_VALID);
		sspreg_update(aio->io, aio->regs.bf_destch_cfg, mask, val);

		/* Enable the transmit pin for this port, logic high */
		mask = BIT((aio->portnum * 4) + AUD_MISC_SEROUT_SDAT_OE);
		sspreg_set_bits(aio->io, AUD_MISC_SEROUT_OE_REG_BASE, mask);
		break;
	case PORT_SPDIF:
		mask = BIT(SPDIF_CTRL_DITHER_ENA);
		sspreg_set_bits(aio->io, SPDIF_CTRL_OFFSET, mask);

		/* Enable and set the FCI ID for the SPDIF channel */
		mask = SPDIF_STREAM_CFG_FCI_ID_MASK;
		mask |= BIT(SPDIF_STREAM_CFG_ENA);
		val = aio->portnum; /* FCI ID is the port num */
		val |= BIT(SPDIF_STREAM_CFG_ENA);
		sspreg_update(aio->io, SPDIF_STREAM_CFG_OFFSET, mask, val);

		mask = BIT(BF_SRC_CFGX_NOT_PAUSE_WHEN_EMPTY);
		mask |= BIT(BF_SRC_CFGX_SFIFO_SZ_DOUBLE);
		mask |= BIT(BF_SRC_CFGX_PROCESS_SEQ_ID_VALID);
		val = BIT(BF_SRC_CFGX_SFIFO_SZ_DOUBLE) |
			BIT(BF_SRC_CFGX_PROCESS_SEQ_ID_VALID);
		sspreg_update(aio->io, aio->regs.bf_sourcech_cfg, mask, val);

		/* Enable the spdif output pin */
		mask = BIT(AUD_MISC_SEROUT_SPDIF_OE);
		sspreg_set_bits(aio->io, AUD_MISC_SEROUT_OE_REG_BASE, mask);
		break;
	default:
		dev_err(aio->dev, "Port not supported\n");
		status = -EINVAL;
	}

	return status;
}

static void audio_ssp_in_enable(struct aio_port *aio)
{
	u32 mask;

	mask = BIT(BF_DST_CFGX_CAP_ENA);
	sspreg_set_bits(aio->io, aio->regs.bf_destch_cfg, mask);

	mask = BF_DESTCH_CTRLX_CAP_RUN;
	sspreg_set_bits(aio->io, aio->regs.bf_destch_ctrl, mask);

	mask = BIT(I2S_IN_STREAM_CFG_CAP_ENA);
	sspreg_set_bits(aio->io, aio->regs.i2s_cap_stream_cfg, mask);

	aio->streams_on |= CAPTURE_STREAM_MASK;
}

#define RX_STATUS_DONE_TIMEOUT_US  200  /* 200 us */
static int wait_for_rx_done(struct aio_port *aio)
{
	u32 mask, val, done_val;
	int ret;

	mask = BIT(0);
	done_val = 0x0;

	ret = readl_poll_timeout_atomic(aio->io + aio->regs.i2s_cap_status,
					val, ((val & mask) == done_val),
					10, RX_STATUS_DONE_TIMEOUT_US);
	if (ret)
		dev_err(aio->dev, "Timeout polling Rx status done. Port %u\n",
			aio->portnum);

	return ret;
}

static void audio_ssp_in_disable(struct aio_port *aio)
{
	u32 mask;

	mask = BIT(I2S_IN_STREAM_CFG_CAP_ENA);
	sspreg_clr_bits(aio->io, aio->regs.i2s_cap_stream_cfg, mask);

	if (!aio->is_slave)
		wait_for_rx_done(aio);

	mask = BIT(BF_DST_CFGX_CAP_ENA);
	sspreg_clr_bits(aio->io, aio->regs.bf_destch_cfg, mask);

	mask = BF_DESTCH_CTRLX_CAP_RUN;
	sspreg_clr_bits(aio->io, aio->regs.bf_destch_ctrl, mask);

	aio->streams_on &= ~CAPTURE_STREAM_MASK;
}

static int audio_ssp_out_enable(struct aio_port *aio)
{
	u32 value, mask;
	int status = 0;

	switch (aio->port_type) {
	case PORT_TDM:
		mask = I2S_OUT_STREAM_CFG_FCI_ID_MASK;
		value = aio->portnum;
		sspreg_update(aio->io, aio->regs.i2s_stream_cfg, mask, value);

		mask = BIT(BF_SRC_CFGX_SFIFO_ENA);
		sspreg_set_bits(aio->io, aio->regs.bf_sourcech_cfg, mask);

		mask = BIT(BF_SOURCECH_CTRL_PLAY_RUN);
		sspreg_set_bits(aio->io, aio->regs.bf_sourcech_ctrl, mask);

		mask = BIT(I2S_OUT_STREAM_CFG_ENA);
		sspreg_set_bits(aio->io, aio->regs.i2s_stream_cfg, mask);

		aio->streams_on |= PLAYBACK_STREAM_MASK;
		break;
	case PORT_SPDIF:
		mask = 0x3;
		sspreg_set_bits(aio->io, SPDIF_FORMAT_CFG_OFFSET, mask);

		mask = BIT(BF_SOURCECH_CTRL_PLAY_RUN);
		sspreg_set_bits(aio->io, aio->regs.bf_sourcech_ctrl, mask);

		mask = BIT(BF_SRC_CFGX_SFIFO_ENA);
		sspreg_set_bits(aio->io, aio->regs.bf_sourcech_cfg, mask);
		break;
	default:
		dev_err(aio->dev, "Port not supported %d\n", aio->portnum);
		status = -EINVAL;
	}

	return status;
}

static int audio_ssp_out_disable(struct aio_port *aio)
{
	u32 mask;
	int status = 0;

	switch (aio->port_type) {
	case PORT_TDM:
		aio->streams_on &= ~PLAYBACK_STREAM_MASK;

		mask = BIT(I2S_OUT_STREAM_CFG_ENA);
		sspreg_clr_bits(aio->io, aio->regs.i2s_stream_cfg, mask);

		/* set group_sync_dis = 1 */
		mask = BIT(aio->portnum);
		sspreg_set_bits(aio->io, BF_SRC_GRP_SYNC_DIS_OFFSET, mask);

		mask = BIT(BF_SOURCECH_CTRL_PLAY_RUN);
		sspreg_clr_bits(aio->io, aio->regs.bf_sourcech_ctrl, mask);

		mask = BIT(BF_SRC_CFGX_SFIFO_ENA);
		sspreg_clr_bits(aio->io, aio->regs.bf_sourcech_cfg, mask);

		/* set group_sync_dis = 0 */
		mask = BIT(aio->portnum);
		sspreg_clr_bits(aio->io, BF_SRC_GRP_SYNC_DIS_OFFSET, mask);

		break;
	case PORT_SPDIF:
		mask = 0x3;
		sspreg_clr_bits(aio->io, SPDIF_FORMAT_CFG_OFFSET, mask);

		mask = BIT(BF_SOURCECH_CTRL_PLAY_RUN);
		sspreg_clr_bits(aio->io, aio->regs.bf_sourcech_ctrl, mask);

		mask = BIT(BF_SRC_CFGX_SFIFO_ENA);
		sspreg_clr_bits(aio->io, aio->regs.bf_sourcech_cfg, mask);
		break;
	default:
		dev_err(aio->dev, "Port not supported %d\n", aio->portnum);
		status = -EINVAL;
	}

	return status;
}

static int aio_ssp_set_clocks(struct aio_port *aio)
{
	u32 value;
	u32 mask;
	u32 sclk;
	u32 mclk_rate;
	unsigned int bits_per_frame;
	unsigned int bit_rate;
	unsigned int ratio;

	bits_per_frame = aio->slots_per_frame * aio->slot_width;
	bit_rate = bits_per_frame * aio->lrclk;

	/*
	 * Check if the bit clock can be generated from the given MCLK.
	 * MCLK must be a perfect multiple of bit clock and must be one of the
	 * following values... (2,4,6,8,10,12,14)
	 */
	if ((aio->mclk % bit_rate) != 0)
		return -EINVAL;

	ratio = aio->mclk / bit_rate;

	if ((ratio == 1) || ((ratio < 512) && (ratio % 2) == 0)) {
		mclk_rate = ratio / 2;
	} else {
		dev_err(aio->dev, "Invalid combination of MCLK and BCLK\n");
		dev_err(aio->dev, "lrclk = %u, bits/frame = %u, mclk = %u\n",
			aio->lrclk, bits_per_frame, aio->mclk);
		return -EINVAL;
	}

	/* Set sclk rate */
	switch (aio->port_type) {
	case PORT_TDM:
		sclk = bits_per_frame;
		if (sclk == 512)
			sclk = 0;

		/* sclks_per_1fs_div = sclk cycles/32 */
		sclk /= 32;

		/* Set number of bitclks per frame */
		mask = I2S_OUT_CFGX_SCLKS_PER_1FS_DIV32_MASK;
		value = sclk << I2S_OUT_CFGX_SCLKS_PER_1FS_DIV32;
		sspreg_update(aio->io, aio->regs.i2s_cfg, mask, value);
		dev_dbg(aio->dev, "SCLKS_PER_1FS_DIV32 = 0x%x\n", value);
		break;
	case PORT_SPDIF:
		break;
	default:
		dev_err(aio->dev, "Unknown port type\n");
		return -EINVAL;
	}

	mask = BIT(I2S_OUT_MCLK_DIV1);
	if (ratio == 1)
		sspreg_set_bits(aio->io, aio->regs.i2s_mclk_cfg, mask);
	else
		sspreg_clr_bits(aio->io, aio->regs.i2s_mclk_cfg, mask);

	/* Set MCLK_RATE ssp port (spdif and ssp are the same) */
	mask = I2S_OUT_MCLKRATE_MASK;
	value = (mclk_rate << I2S_OUT_MCLKRATE_SHIFT);
	sspreg_update(aio->io, aio->regs.i2s_mclk_cfg, mask, value);

	dev_dbg(aio->dev, "mclk cfg reg = 0x%x\n", value);
	dev_dbg(aio->dev, "bits per frame = %u, mclk = %u Hz, lrclk = %u Hz\n",
			bits_per_frame, aio->mclk, aio->lrclk);
	return 0;
}

static int aio_ssp_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct aio_port *aio = aio_dai_get_portinfo(dai);
	int rate, bitres, bits_per_sample;
	u32 value;
	u32 mask;
	int ret = 0;

	dev_dbg(aio->dev, "%s port = %d\n", __func__, aio->portnum);
	dev_dbg(aio->dev, "params_channels %d\n", params_channels(params));
	dev_dbg(aio->dev, "rate %d\n", params_rate(params));
	dev_dbg(aio->dev, "format %d\n", params_format(params));

	rate = params_rate(params);

	switch (aio->mode) {
	case SSPMODE_TDM:
		/* It's expected that set_dai_tdm_slot has been called */
		if ((rate == 192000) && (params_channels(params) > 4)) {
			dev_err(aio->dev, "Cannot run %d channels at %dHz\n",
				params_channels(params), rate);
			return -EINVAL;
		}
		break;
	case SSPMODE_I2S:
		if (params_channels(params) != 2) {
			dev_err(aio->dev, "i2s mode must use 2 channels\n");
			return -EINVAL;
		}

		aio->active_slots = 2;
		aio->slots_per_frame = 2;
		aio->slot_width = 32; /* Use 64Fs */

		break;
	default:
		dev_err(aio->dev, "%s unknown mode\n", __func__);
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mask = BIT(BF_SRC_CFGX_SAMPLE_CH_MODE);
		sspreg_clr_bits(aio->io, aio->regs.bf_sourcech_cfg, mask);

		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			bitres = 16;
			bits_per_sample = 16;
			break;

		case SNDRV_PCM_FORMAT_S24_LE:
		case SNDRV_PCM_FORMAT_S32_LE:
			bitres = 0; /* 32 bit mode is coded as 0 */
			bits_per_sample = 0; /* 32 bits coded as 0 */
			break;

		default:
			return -EINVAL;
		}

		mask = BF_SRC_CFGX_BITRES_MASK;
		value = (bitres << BF_SRC_CFGX_BIT_RES);
		sspreg_update(aio->io, aio->regs.bf_sourcech_cfg, mask, value);

		/* Only needed for LSB mode, ignored for MSB */
		mask = I2S_OUT_CFGX_BIT_PER_SAMPLE_MASK;
		value = (bits_per_sample << I2S_OUT_CFGX_BITS_PER_SAMPLE);
		sspreg_update(aio->io, aio->regs.i2s_cfg, mask, value);
	} else {

		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			bits_per_sample = 16;

			mask = BIT(BF_DST_CFGX_CAP_MODE);
			sspreg_set_bits(aio->io, aio->regs.bf_destch_cfg, mask);
			break;

		case SNDRV_PCM_FORMAT_S24_LE:
		case SNDRV_PCM_FORMAT_S32_LE:
			bits_per_sample = 0; /* 32 bits coded as 0 */

			mask = BIT(BF_DST_CFGX_CAP_MODE);
			sspreg_clr_bits(aio->io, aio->regs.bf_destch_cfg, mask);
			break;

		default:
			return -EINVAL;
		}

		/* Used for both LSB and MSB modes */
		mask = I2S_IN_CFGX_BIT_PER_SAMPLE_MASK;
		value = (bits_per_sample << I2S_IN_CFGX_BITS_PER_SAMPLE);
		sspreg_update(aio->io, aio->regs.i2s_cap_cfg, mask, value);
	}

	if (aio->streams_on == 0) {
		if (aio->port_type != PORT_SPDIF)
			update_ssp_cfg(aio);

		aio->lrclk = rate;

		if (!aio->is_slave)
			ret = aio_ssp_set_clocks(aio);
	}

	return ret;
}

/*
 * Check that the actual mclk is within about 1% of the requested rate.
 * The check is rather loose and is intended to catch any big mistakes.
 * It is expected that the actual mclk rate may be a little different
 * than the requested rate because the clock from which the mclk is
 * derived (PLL) may not be an exact multiple of the mclk.
 */
static bool mclk_in_range(unsigned int target, unsigned int actual)
{
	unsigned int delta;

	/* Mclk is at least several MHz, so simple div by 100 will suffice */
	delta = target / 100;
	return (actual > (target - delta)) && (actual < (target + delta));
}

/*
 * This function sets the mclk frequency for pll clock
 */
static int aio_ssp_set_sysclk(struct snd_soc_dai *dai,
			int clk_id, unsigned int freq, int dir)
{
	int ret;
	u32 mask;
	long rate;
	struct aio_port *aio = aio_dai_get_portinfo(dai);

	dev_dbg(aio->dev, "%s Enter port = %d\n", __func__, aio->portnum);

	/*
	 * This should not happen, but the machine file may inadvertently
	 * call set_sysclk without configuring a clock via the devicetree.
	 */
	if (!aio->clk_info.audio_clk) {
		dev_err(aio->dev, "%s Error. No clock assigned.\n", __func__);
		return -ENODEV;
	}

	rate = clk_round_rate(aio->clk_info.audio_clk, freq);
	if (rate < 0) {
		dev_err(aio->dev, "%s Error with with clock %ld.\n",
			__func__, rate);
		return rate;
	}

	if (!mclk_in_range(freq, rate)) {
		dev_err(aio->dev, "%s Can not set rate to %u  actual %ld.\n",
			__func__, freq, rate);
		return -EINVAL;
	}

	ret = clk_set_rate(aio->clk_info.audio_clk, freq);
	if (ret) {
		dev_err(aio->dev, "%s Set MCLK rate fail %d\n", __func__, ret);
		return ret;
	}

	aio->mclk = freq;

	/* Set bit for active */
	mask = BIT(AUD_MISC_SEROUT_MCLK_OE + (aio->portnum * 4));
	sspreg_set_bits(aio->io, AUD_MISC_SEROUT_OE_REG_BASE, mask);

	return 0;
}

static int aio_ssp_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct omega_audio *aiotop = snd_soc_dai_get_drvdata(dai);
	struct aio_port *aio = aio_dai_get_portinfo(dai);
	struct iproc_pcm_dma_info *dma_info;

	dev_dbg(aio->dev, "Enter %s\n", __func__);

	audio_ssp_init_portregs(aio);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_info = &aiotop->dma_info_play[aio->portnum];
	else
		dma_info = &aiotop->dma_info_cap[aio->portnum];

	dma_info->portnum = aio->portnum;
	dma_info->audio = aio->io->audio;
	dma_info->substream = substream;

	snd_soc_dai_set_dma_data(dai, substream, dma_info);

	substream->runtime->hw.rate_min = SSP_RATE_MIN;
	substream->runtime->hw.rate_max = SSP_RATE_MAX;

	snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &ssp_rate_constraint);

	return clk_prepare_enable(aio->clk_info.audio_clk);
}

static void aio_ssp_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct aio_port *aio = aio_dai_get_portinfo(dai);

	dev_dbg(aio->dev, "Enter %s\n", __func__);

	clk_disable_unprepare(aio->clk_info.audio_clk);
}

int omega_ssp_set_custom_fsync_width(struct snd_soc_dai *cpu_dai, int len)
{
	struct aio_port *aio = aio_dai_get_portinfo(cpu_dai);

	if ((len > 0) && (len < 256)) {
		aio->fsync_width = len;
		return 0;
	} else {
		return -EINVAL;
	}
}
EXPORT_SYMBOL_GPL(omega_ssp_set_custom_fsync_width);

static int aio_ssp_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct aio_port *aio = aio_dai_get_portinfo(cpu_dai);
	u32 val;
	u32 mask;

	dev_dbg(aio->dev, "%s Enter  fmt: %x\n", __func__, fmt);

	if (aio->port_type == PORT_SPDIF)
		return -EINVAL;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aio->is_slave = 1;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aio->is_slave = 0;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		aio->fs_delay = 1;
		aio->mode = SSPMODE_I2S;
		break;

	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		/* DSP_A = data after FS, DSP_B = data during FS */
		if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_DSP_A)
			aio->fs_delay = 1;
		else {
			if (aio->is_slave) {
				dev_err(aio->dev,
				"%s DSP_B mode not supported while slave.\n",
					__func__);
				return -EINVAL;
			}
			aio->fs_delay = 0;
		}
		aio->mode = SSPMODE_TDM;
		break;

	default:
		return -EINVAL;
	}

	/* We must be i2s master to invert any clock */
//LH	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF) {
//LH		if (aio->is_slave || (aio->mode == SSPMODE_TDM)) {
//LH			dev_err(aio->dev,
//LH			"%s Can only invert clocks in i2s master mode\n",
//LH				__func__);
//LH			return -EINVAL;
//LH		}
//LH	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_NF:
		aio->invert_bclk = true;
		aio->invert_fs = false;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		aio->invert_bclk = false;
		aio->invert_fs = true;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		aio->invert_bclk = true;
		aio->invert_fs = true;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		aio->invert_bclk = false;
		aio->invert_fs = false;
		break;
	default:
		return -EINVAL;
	}

	/*
	 * Configure the word clk and bit clk as output or tristate
	 * Each port has 4 bits for controlling its pins.
	 * Shift the mask based upon port number.
	 */
	mask = BIT(AUD_MISC_SEROUT_LRCK_OE) | BIT(AUD_MISC_SEROUT_SCLK_OE);
	mask = (mask << (aio->portnum * 4));
	if (aio->is_slave)
		val = 0;  /* Clear bit for tri-state */
	else
		val = mask;   /* Set bit for drive */

	dev_dbg(aio->dev, "%s  Set OE bits 0x%x\n", __func__, val);
	sspreg_update(aio->io, AUD_MISC_SEROUT_OE_REG_BASE, mask, val);

	return 0;
}

static int aio_ssp_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct aio_port *aio = aio_dai_get_portinfo(dai);

	dev_dbg(aio->dev,
		"%s cmd %d at port = %d\n", __func__, cmd, aio->portnum);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			audio_ssp_out_enable(aio);
		else
			audio_ssp_in_enable(aio);

		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			audio_ssp_out_disable(aio);
		else
			audio_ssp_in_disable(aio);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int aio_ssp_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	struct aio_port *aio = aio_dai_get_portinfo(cpu_dai);
	unsigned int active_slots;
	unsigned int bits_per_frame;
	bool found = false;
	unsigned int i;

	if (tx_mask != rx_mask) {
		dev_err(aio->dev, "%s tx_mask must equal rx_mask\n", __func__);
		return -EINVAL;
	}

	active_slots = hweight32(tx_mask);

	if ((active_slots == 0) || (active_slots > 16))
		return -EINVAL;

	/* Slot value must be even */
	if (active_slots % 2)
		return -EINVAL;

	if ((slot_width != 16) && (slot_width != 32))
		return -EINVAL;

	bits_per_frame = slots * slot_width;

	for (i = 0; i < ARRAY_SIZE(ssp_valid_tdm_framesize); i++) {
		if (ssp_valid_tdm_framesize[i] == bits_per_frame) {
			found = true;
			break;
		}
	}

	if (!found) {
		dev_err(aio->dev, "%s In TDM mode, frame bits INVALID (%d)\n",
			__func__, bits_per_frame);
		return -EINVAL;
	}

	aio->active_slots = active_slots;
	aio->slot_width = slot_width;
	aio->slots_per_frame = slots;

	dev_dbg(aio->dev, "%s active_slots %u, bits per frame %d\n",
			__func__, aio->active_slots, bits_per_frame);
	return 0;
}

static int aio_ssp_set_pll(struct snd_soc_dai *cpu_dai, int pll_id,
				 int source, unsigned int freq_in,
				 unsigned int freq_out)
{
	struct aio_port *aio = aio_dai_get_portinfo(cpu_dai);
	struct clk *clk_pll, *clk_post_div;
	int ret = 0;

	if (!aio->clk_info.audio_clk) {
		dev_err(aio->dev,
			"%s: port %d does not have an assigned clock.\n",
			__func__, aio->portnum);
		return -ENODEV;
	}

	clk_post_div = clk_get_parent(aio->clk_info.audio_clk);
	if (IS_ERR(clk_post_div)) {
		dev_err(aio->dev,
			"%s: could not get post div clock.\n", __func__);
		return -ENODEV;
	}

	clk_pll = clk_get_parent(clk_post_div);
	if (IS_ERR(clk_pll)) {
		dev_err(aio->dev,
			"%s: could not get audiopll clock.\n", __func__);
		return -ENODEV;
	}

	ret = clk_set_rate(clk_pll, freq_out);

	return ret;
}

#define I2S_OUT_CFG_REG_UPDATE_MASK  \
	(BIT(I2S_OUT_CFGX_TDM_MODE)       | \
	BIT(I2S_OUT_CFGX_SLAVE_MODE)      | \
	I2S_OUT_CFGX_FSYNC_WIDTH_MASK     | \
	I2S_OUT_CFGX_VALID_SLOT_MASK      | \
	BIT(I2S_OUT_CFGX_BITS_PER_SLOT)   | \
	I2S_OUT_CFGX_DATA_ALIGNMENT_MASK  | \
	BIT(I2S_OUT_CFGX_SCLK_POLARITY)   | \
	BIT(I2S_OUT_CFGX_LRCK_POLARITY))

/* Input cfg is same as output, but the FS width is not a valid field */
#define I2S_IN_CFG_REG_UPDATE_MASK  (I2S_OUT_CFG_REG_UPDATE_MASK & \
				     ~I2S_OUT_CFGX_FSYNC_WIDTH_MASK)

static void update_ssp_cfg(struct aio_port *aio)
{
	u32 valid_slots;       /* reg val to program */
	int bits_per_slot_cmn;

	u32 ssp_newcfg;
	u32 ssp_outcfg;
	u32 ssp_incfg;
	u32 fsync_width;

	if (aio->port_type == PORT_SPDIF)
		return;

	ssp_newcfg = 0;

	/* We encode 32 slots as 0 in the reg */
	valid_slots = aio->active_slots / 2;
	if (aio->active_slots == 32)
		valid_slots = 0;

	ssp_newcfg |= (valid_slots << I2S_OUT_CFGX_VALID_SLOT);

	/* Slot Width is either 16 or 32 */
	bits_per_slot_cmn = 0;     /* Default to 32 bits */
	if (aio->slot_width <= 16)
		bits_per_slot_cmn = 1;

	ssp_newcfg |= (bits_per_slot_cmn << I2S_OUT_CFGX_BITS_PER_SLOT);

	if (aio->mode == SSPMODE_TDM) {
		ssp_newcfg |= BIT(I2S_OUT_CFGX_TDM_MODE);
		if (aio->fsync_width == -1)
			fsync_width = 1;
		else
			fsync_width = aio->fsync_width;

		ssp_newcfg |= (fsync_width << I2S_OUT_CFGX_FSYNC_WIDTH);
	}

	if (aio->is_slave)
		ssp_newcfg |= BIT(I2S_OUT_CFGX_SLAVE_MODE);
	else
		ssp_newcfg &= ~BIT(I2S_OUT_CFGX_SLAVE_MODE);

	if (aio->mode == SSPMODE_I2S) {
		ssp_newcfg |= (1 << I2S_OUT_CFGX_DATA_ALIGNMENT);
		ssp_newcfg |= BIT(I2S_OUT_CFGX_FSYNC_WIDTH);
	} else {
		ssp_newcfg |= (aio->fs_delay << I2S_OUT_CFGX_DATA_ALIGNMENT);
	}

	/* The framesync logic is inverted on Omega */
	if (aio->invert_bclk)
		ssp_newcfg |= BIT(I2S_OUT_CFGX_SCLK_POLARITY);

	if (aio->invert_fs)
		ssp_newcfg &= ~BIT(I2S_OUT_CFGX_LRCK_POLARITY);
	else
		ssp_newcfg |= BIT(I2S_OUT_CFGX_LRCK_POLARITY);

	/*
	 * SSP in cfg.
	 * Always set slave mode for Rx formatter.
	 * The Rx formatter's Slave Mode bit controls if it uses its own
	 * internal clock or the clock signal that comes from the Slave Mode
	 * bit set in the Tx formatter (which would be the Tx Formatters
	 * internal clock or signal from external pin).
	 */
	ssp_incfg = ssp_newcfg | BIT(I2S_OUT_CFGX_SLAVE_MODE);
	ssp_incfg &= I2S_IN_CFG_REG_UPDATE_MASK;
	sspreg_update(aio->io, aio->regs.i2s_cap_cfg,
			I2S_IN_CFG_REG_UPDATE_MASK, ssp_incfg);

	ssp_outcfg = ssp_newcfg;
	sspreg_update(aio->io, aio->regs.i2s_cfg,
			I2S_OUT_CFG_REG_UPDATE_MASK, ssp_outcfg);
}

static const struct snd_soc_dai_ops aio_ssp_dai_ops = {
	.startup	= aio_ssp_startup,
	.shutdown	= aio_ssp_shutdown,
	.trigger	= aio_ssp_trigger,
	.hw_params	= aio_ssp_hw_params,
	.set_fmt	= aio_ssp_set_fmt,
	.set_sysclk	= aio_ssp_set_sysclk,
	.set_tdm_slot	= aio_ssp_set_dai_tdm_slot,
	.set_pll	= aio_ssp_set_pll,
};

static const struct snd_soc_dai_ops aio_spdif_dai_ops = {
	.startup	= aio_ssp_startup,
	.shutdown	= aio_ssp_shutdown,
	.trigger	= aio_ssp_trigger,
	.hw_params	= aio_ssp_hw_params,
	.set_sysclk	= aio_ssp_set_sysclk,
};

#define INIT_CPU_DAI(num) { \
	.name = "omega-ssp" #num, \
	.playback = { \
		.channels_min = 2, \
		.channels_max = 16, \
		.rates = SNDRV_PCM_RATE_KNOT, \
		.formats = SNDRV_PCM_FMTBIT_S16_LE | \
				SNDRV_PCM_FMTBIT_S24_LE | \
				SNDRV_PCM_FMTBIT_S32_LE, \
	}, \
	.capture = { \
		.channels_min = 2, \
		.channels_max = 16, \
		.rates = SNDRV_PCM_RATE_KNOT, \
		.formats = SNDRV_PCM_FMTBIT_S16_LE | \
				SNDRV_PCM_FMTBIT_S24_LE | \
				SNDRV_PCM_FMTBIT_S32_LE, \
	}, \
	.ops = &aio_ssp_dai_ops, \
}

static const struct snd_soc_dai_driver aio_ssp_dai_info[] = {
	INIT_CPU_DAI(0),
	INIT_CPU_DAI(1),
	INIT_CPU_DAI(2),
};

static const struct snd_soc_dai_driver aio_spdif_dai_info = {
	.name = "omega-spdif",
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &aio_spdif_dai_ops,
};

static struct snd_soc_dai_driver aio_ssp_dai[AIO_MAX_PORTS];

static const struct snd_soc_component_driver aio_ssp_component = {
	.name		= "omega-audio",
};

/*
 * Return < 0 if error
 * Return 0 if disabled
 * Return 1 if enabled and node is parsed successfully
 */
static int parse_ssp_child_node(struct platform_device *pdev,
				struct device_node *dn,
				struct omega_audio *aiotop,
				struct snd_soc_dai_driver *p_dai,
				struct aio_port *aio)
{
	struct aio_ssp_regs ssp_regs[3];
	u32 rawval;
	int portnum = -1;
	enum audio_port_type port_type;
	struct clk *clk;

	if (of_property_read_u32(dn, "reg", &rawval)) {
		dev_err(&pdev->dev, "Missing reg property\n");
		return -EINVAL;
	}

	portnum = rawval;
	switch (rawval) {
	case 0:
		ssp_regs[0] = INIT_SSP_REGS(0);
		port_type = PORT_TDM;
		break;
	case 1:
		ssp_regs[1] = INIT_SSP_REGS(1);
		port_type = PORT_TDM;
		break;
	case 2:
		ssp_regs[2] = INIT_SSP_REGS(2);
		port_type = PORT_TDM;
		break;
	case 3:
		port_type = PORT_SPDIF;
		break;
	default:
		dev_err(&pdev->dev, "Bad value for reg %u\n", rawval);
		return -EINVAL;
	}

	aio->io = &aiotop->io;
	aio->portnum = portnum;
	aio->port_type = port_type;
	aio->fsync_width = -1;

	switch (port_type) {
	case PORT_TDM:
		aio->regs = ssp_regs[portnum];
		*p_dai = aio_ssp_dai_info[portnum];
		aio->mode = SSPMODE_UNKNOWN;
		break;

	case PORT_SPDIF:
		aio->regs.bf_sourcech_cfg = BF_SRC_CFG3_OFFSET;
		aio->regs.bf_sourcech_ctrl = BF_SRC_CTRL3_OFFSET;
		aio->regs.i2s_mclk_cfg = SPDIF_MCLK_CFG_OFFSET;
		aio->regs.i2s_stream_cfg = SPDIF_STREAM_CFG_OFFSET;
		*p_dai = aio_spdif_dai_info;

		/* For the purposes of this code SPDIF can be I2S mode */
		aio->mode = SSPMODE_I2S;
		break;
	default:
		dev_err(&pdev->dev, "Bad value for port_type %d\n", port_type);
		return -EINVAL;
	}

	dev_dbg(&pdev->dev, "%s portnum = %d\n", __func__, aio->portnum);
	aio->streams_on = 0;
	aio->dev = &pdev->dev;

	aio->clk_info.audio_clk = NULL;

	/*
	 * The default in the DT is to assign a clock. It is possible
	 * the user may not want a clock if the port is only used in slave
	 * mode.  In this case, they could override the default using this
	 * mechanism:    /delete-property/ clocks;
	 */
	if (of_property_read_bool(dn, "clocks")) {
		clk = devm_get_clk_from_child(&pdev->dev, dn, "ssp_clk");
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev,
				"Port %d: devm_clk_get ssp-clk err %ld\n",
				portnum, PTR_ERR(clk));
			return PTR_ERR(clk);
		}

		aio->clk_info.audio_clk = clk;
	} else {
		dev_dbg(&pdev->dev, "No clock provided for port %d\n", portnum);
	}

	return audio_ssp_init_portregs(aio);
}

static struct regmap_config audio_primary_regmap_cfg = {
	.name = "ssp_main",

	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,

	.cache_type = REGCACHE_NONE,
};

static struct regmap_config i2s_in_regmap_cfg = {
	.name = "ssp_secondary",

	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,

	.cache_type = REGCACHE_NONE,
};

static int omega_ssp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *child_node;
	struct resource *res = pdev->resource;
	struct omega_audio *aiotop;
	void __iomem *ioregs;
	int err = -EINVAL;
	int node_count;
	int active_port_count;
	int irq_num;
	u32 mask;

	aiotop = devm_kzalloc(dev, sizeof(struct omega_audio), GFP_KERNEL);
	if (!aiotop)
		return -ENOMEM;

	dev_set_drvdata(dev, aiotop);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "aud");
	ioregs = devm_ioremap_resource(dev, res);
	if (IS_ERR(ioregs))
		return PTR_ERR(ioregs);

	audio_primary_regmap_cfg.max_register = resource_size(res);
	aiotop->io.audio = devm_regmap_init_mmio(&pdev->dev, ioregs,
					    &audio_primary_regmap_cfg);
	if (IS_ERR(aiotop->io.audio)) {
		dev_err(&pdev->dev, "regmap failed\n");
		return PTR_ERR(aiotop->io.audio);
	}

	aiotop->io.cmn_io = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							"brcm,clk-mux-syscon");
	if (IS_ERR(aiotop->io.cmn_io)) {
		dev_err(&pdev->dev, "regmap failed\n");
		return PTR_ERR(aiotop->io.cmn_io);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "i2s_in");
	ioregs = devm_ioremap_resource(dev, res);
	if (IS_ERR(ioregs))
		return PTR_ERR(ioregs);

	i2s_in_regmap_cfg.max_register = resource_size(res);
	aiotop->io.i2s_in = devm_regmap_init_mmio(&pdev->dev, ioregs,
					    &i2s_in_regmap_cfg);
	if (IS_ERR(aiotop->io.i2s_in)) {
		dev_err(&pdev->dev, "regmap failed\n");
		return PTR_ERR(aiotop->io.i2s_in);
	}

	/* Tri-state all controlable pins until we know that we need them */
	mask = SSP_TRISTATE_MASK;
	sspreg_clr_bits(&aiotop->io, AUD_MISC_SEROUT_OE_REG_BASE, mask);

	node_count = of_get_child_count(pdev->dev.of_node);
	if ((node_count < 1) || (node_count > AIO_MAX_PORTS)) {
		dev_err(dev, "child nodes is %d.  Must be between 1 and %d\n",
			node_count, AIO_MAX_PORTS);
		return -EINVAL;
	}

	active_port_count = 0;
	for_each_available_child_of_node(pdev->dev.of_node, child_node) {
		err = parse_ssp_child_node(pdev, child_node, aiotop,
					&aio_ssp_dai[active_port_count],
					&aiotop->portinfo[active_port_count]);

		/* negative is err, 0 is active and good, 1 is disabled */
		if (err < 0)
			return err;
		else if (!err) {
			dev_dbg(dev, "Activating DAI: %s\n",
				aio_ssp_dai[active_port_count].name);
			active_port_count++;
		}
	}

	aiotop->dev = dev;

	dev_dbg(dev, "Registering %d DAIs\n", active_port_count);
	err = snd_soc_register_component(dev, &aio_ssp_component,
				aio_ssp_dai, active_port_count);
	if (err) {
		dev_err(dev, "snd_soc_register_dai failed\n");
		return err;
	}

	irq_num = platform_get_irq(pdev, 0);
	if (irq_num <= 0) {
		dev_err(dev, "platform_get_irq failed\n");
		err = irq_num;
		goto err_irq;
	}

	aiotop->rb_info.dev = dev;
	aiotop->rb_info.audio = aiotop->io.audio;
	aiotop->rb_info.irq_num = irq_num;

	aiotop->rb_info.num_playback = AIO_MAX_PLAYBACK_PORTS;
	aiotop->rb_info.rb_state_play = &aiotop->dma_info_play[0];

	aiotop->rb_info.num_capture = AIO_MAX_CAPTURE_PORTS;
	aiotop->rb_info.rb_state_cap = &aiotop->dma_info_cap[0];

	err = iproc_pcm_platform_register(dev, &aiotop->rb_info);
	if (err) {
		dev_err(dev, "platform reg error %d\n", err);
		goto err_irq;
	}

	return 0;

err_irq:
	snd_soc_unregister_component(dev);
	return err;
}

static int omega_ssp_remove(struct platform_device *pdev)
{
	iproc_pcm_platform_unregister(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int omega_ssp_suspend(struct device *dev)
{
	struct omega_audio *aiotop = dev_get_drvdata(dev);

	sspreg_read(&aiotop->io, AUD_MISC_SEROUT_OE_REG_BASE,
		    &aiotop->oe_reg_context);
	return 0;
}

static int omega_ssp_resume(struct device *dev)
{
	struct omega_audio *aiotop = dev_get_drvdata(dev);

	sspreg_write(&aiotop->io, AUD_MISC_SEROUT_OE_REG_BASE,
		     aiotop->oe_reg_context);
	return 0;
}

static SIMPLE_DEV_PM_OPS(omega_ssp_pm_ops,
			 omega_ssp_suspend, omega_ssp_resume);
#endif

static const struct of_device_id omega_ssp_of_match[] = {
	{ .compatible = "brcm,omega-audio" },
	{},
};
MODULE_DEVICE_TABLE(of, omega_ssp_of_match);

static struct platform_driver omega_ssp_driver = {
	.probe		= omega_ssp_probe,
	.remove		= omega_ssp_remove,
	.driver		= {
		.name		= "omega-ssp",
		.of_match_table	= omega_ssp_of_match,
		.pm		= &omega_ssp_pm_ops,
	},
};

module_platform_driver(omega_ssp_driver);

MODULE_ALIAS("platform:omega-ssp");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Omega ASoC SSP Interface");
