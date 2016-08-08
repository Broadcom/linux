/*
 * OmniVision OV5640 sensor driver
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
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
 * This code is derived and modified (only sensor driver part) from
 * the original code/patch -> https://lwn.net/Articles/470643/
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <media/v4l2-subdev.h>
#include <media/soc_camera.h>

/* As of now only one std control is registered */
#define NUM_STD_CONTROLS	1
/**
 *struct ov5640_reg - ov5640 register format
 *@reg: 16-bit offset to register
 *@val: 8/16/32-bit register value
 *@length: length of the register
 *
 * Define a structure for OV5640 register initialization values
 */
struct ov5640_reg {
	u16 reg;
	u8 val;
};

/* OV5640 has only one fixed colorspace per pixelcode */
struct ov5640_datafmt {
	u32 code;
	enum v4l2_colorspace colorspace;
};

struct ov5640_priv {
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler hdl;
	int i_size;
	int i_fmt;
};

static const struct ov5640_datafmt ov5640_fmts[] = {
	/*
	 * Order important: first natively supported,
	 * second supported with a GPIO extender
	 */
	{MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
	{MEDIA_BUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG},
	{MEDIA_BUS_FMT_RGB565_2X8_LE, V4L2_COLORSPACE_JPEG},
};

enum ov5640_size {
	OV5640_SIZE_VGA,	/*  640 x 480 */
	OV5640_SIZE_LAST,
	OV5640_SIZE_MAX
};

static const struct v4l2_frmsize_discrete ov5640_frmsizes[OV5640_SIZE_LAST] = {
	{640, 480},
};

static const struct ov5640_reg jpeg_init_common[] = {
	/* System Clock Div */
	{0x3035, 0x12},		/*SystemClkDiv 7:4, /1=676Mhz
				* MIPI Sclk Div 3:0, /1=676Mhz
				*/
	/*System/IO pad Control */
	{0x3000, 0x00},		/*Resets */
	{0x3002, 0x00},
	{0x3004, 0xff},		/*Clocks */
	{0x3006, 0xff},
	/*Format control */
	{0x4300, 0x30},		/*Output Format[7:4] Sequence[3:0] (UVYV) */
	/*MIPI Control */
	{0x4837, 0x16},
	/*PCLK Divider */
	{0x3824, 0x04},		/*Scale Divider [4:0] */
	{0x3008, 0x42},		/*stop sensor streaming */
	{0xFFFF, 0x00}
};

static const struct ov5640_reg ov5640_stream[] = {
	/* System Control */
	{0x3008, 0x02},		/* enable streaming */
	{0xFFFF, 0x00}
};

static const struct ov5640_reg ov5640_power_down[] = {
	/* System Control */
	{0x3008, 0x42},		/* disable streaming */
	{0xFFFF, 0x01},		/* Sleep 1ms */
	{0xFFFF, 0x00}
};

static const struct ov5640_reg ov5640_settings_omnivision_3[] = {
	{0x3103, 0x11},
	{0x3008, 0x82},
	{0x3008, 0x42},
	{0x3103, 0x03},
	{0x3017, 0xff},
	{0x3018, 0xff},
	{0x3034, 0x1a},   /* 8 bit mode */
	{0x4745, 0x02},   /* data order */
	{0x4740, 0x22},
	{0x3035, 0x11},
	{0x3036, 0x46},
	{0x3037, 0x13},
	{0x3108, 0x01},
	{0x3630, 0x36},
	{0x3631, 0x0e},
	{0x3632, 0xe2},
	{0x3633, 0x12},
	{0x3621, 0xe0},
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3905, 0x02},
	{0x3906, 0x10},
	{0x3901, 0x0a},
	{0x3731, 0x12},
	{0x3600, 0x08},
	{0x3601, 0x33},
	{0x302d, 0x60},
	{0x3620, 0x52},
	{0x371b, 0x20},
	{0x471c, 0x50},
	{0x3a13, 0x43},
	{0x3a18, 0x00},
	{0x3a19, 0xf8},
	{0x3635, 0x13},
	{0x3636, 0x03},
	{0x3634, 0x40},
	{0x3622, 0x01},
	{0x3c01, 0x34},
	{0x3c04, 0x28},
	{0x3c05, 0x98},
	{0x3c06, 0x00},
	{0x3c07, 0x08},
	{0x3c08, 0x00},
	{0x3c09, 0x1c},
	{0x3c0a, 0x9c},
	{0x3c0b, 0x40},
	{0x3820, 0x41},
	{0x3821, 0x07},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0x9b},
	{0x3808, 0x02},
	{0x3809, 0x80},
	{0x380a, 0x01},
	{0x380b, 0xe0},
	{0x380c, 0x07},
	{0x380d, 0x68},
	{0x380e, 0x03},
	{0x380f, 0xd8},
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x06},
	{0x3618, 0x00},
	{0x3612, 0x29},
	{0x3708, 0x64},
	{0x3709, 0x52},
	{0x370c, 0x03},
	{0x3a02, 0x0b},
	{0x3a03, 0x88},
	{0x3a08, 0x01},
	{0x3a09, 0x27},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0e, 0x03},
	{0x3a0d, 0x04},
	{0x3a14, 0x0b},
	{0x3a15, 0x88},
	{0x4001, 0x02},
	{0x4004, 0x02},
	{0x3000, 0x00},
	{0x3002, 0x1c},
	{0x3004, 0xff},
	{0x3006, 0xc3},
	{0x300e, 0x58},
	{0x302e, 0x00},
	{0x4300, 0x32},
	{0x501f, 0x00},
	{0x4713, 0x03},
	{0x4407, 0x04},
	{0x440e, 0x00},
	{0x460b, 0x35},
	{0x460c, 0x22},
	{0x4837, 0x22},
	{0x3824, 0x02},
	{0x5000, 0xa7},
	{0x5001, 0xa3},
	{0x5180, 0xff},
	{0x5181, 0xf2},
	{0x5182, 0x00},
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x09},
	{0x5187, 0x09},
	{0x5188, 0x09},
	{0x5189, 0x75},
	{0x518a, 0x54},
	{0x518b, 0xe0},
	{0x518c, 0xb2},
	{0x518d, 0x42},
	{0x518e, 0x3d},
	{0x518f, 0x56},
	{0x5190, 0x46},
	{0x5191, 0xf8},
	{0x5192, 0x04},
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x03},
	{0x5197, 0x01},
	{0x5198, 0x04},
	{0x5199, 0x12},
	{0x519a, 0x04},
	{0x519b, 0x00},
	{0x519c, 0x06},
	{0x519d, 0x82},
	{0x519e, 0x38},
	{0x5381, 0x1e},
	{0x5382, 0x5b},
	{0x5383, 0x08},
	{0x5384, 0x0a},
	{0x5385, 0x7e},
	{0x5386, 0x88},
	{0x5387, 0x7c},
	{0x5388, 0x6c},
	{0x5389, 0x10},
	{0x538a, 0x01},
	{0x538b, 0x98},
	{0x5300, 0x08},
	{0x5301, 0x30},
	{0x5302, 0x10},
	{0x5303, 0x00},
	{0x5304, 0x08},
	{0x5305, 0x30},
	{0x5306, 0x08},
	{0x5307, 0x16},
	{0x5309, 0x08},
	{0x530a, 0x30},
	{0x530b, 0x04},
	{0x530c, 0x06},
	{0x5480, 0x01},
	{0x5481, 0x08},
	{0x5482, 0x14},
	{0x5483, 0x28},
	{0x5484, 0x51},
	{0x5485, 0x65},
	{0x5486, 0x71},
	{0x5487, 0x7d},
	{0x5488, 0x87},
	{0x5489, 0x91},
	{0x548a, 0x9a},
	{0x548b, 0xaa},
	{0x548c, 0xb8},
	{0x548d, 0xcd},
	{0x548e, 0xdd},
	{0x548f, 0xea},
	{0x5490, 0x1d},
	{0x5580, 0x02},
	{0x5583, 0x40},
	{0x5584, 0x10},
	{0x5589, 0x10},
	{0x558a, 0x00},
	{0x558b, 0xf8},
	{0x5800, 0x23},
	{0x5801, 0x14},
	{0x5802, 0x0f},
	{0x5803, 0x0f},
	{0x5804, 0x12},
	{0x5805, 0x26},
	{0x5806, 0x0c},
	{0x5807, 0x08},
	{0x5808, 0x05},
	{0x5809, 0x05},
	{0x580a, 0x08},
	{0x580b, 0x0d},
	{0x580c, 0x08},
	{0x580d, 0x03},
	{0x580e, 0x00},
	{0x580f, 0x00},
	{0x5810, 0x03},
	{0x5811, 0x09},
	{0x5812, 0x07},
	{0x5813, 0x03},
	{0x5814, 0x00},
	{0x5815, 0x01},
	{0x5816, 0x03},
	{0x5817, 0x08},
	{0x5818, 0x0d},
	{0x5819, 0x08},
	{0x581a, 0x05},
	{0x581b, 0x06},
	{0x581c, 0x08},
	{0x581d, 0x0e},
	{0x581e, 0x29},
	{0x581f, 0x17},
	{0x5820, 0x11},
	{0x5821, 0x11},
	{0x5822, 0x15},
	{0x5823, 0x28},
	{0x5824, 0x46},
	{0x5825, 0x26},
	{0x5826, 0x08},
	{0x5827, 0x26},
	{0x5828, 0x64},
	{0x5829, 0x26},
	{0x582a, 0x24},
	{0x582b, 0x22},
	{0x582c, 0x24},
	{0x582d, 0x24},
	{0x582e, 0x06},
	{0x582f, 0x22},
	{0x5830, 0x40},
	{0x5831, 0x42},
	{0x5832, 0x24},
	{0x5833, 0x26},
	{0x5834, 0x24},
	{0x5835, 0x22},
	{0x5836, 0x22},
	{0x5837, 0x26},
	{0x5838, 0x44},
	{0x5839, 0x24},
	{0x583a, 0x26},
	{0x583b, 0x28},
	{0x583c, 0x42},
	{0x583d, 0xce},
	{0x5025, 0x00},
	{0x3a0f, 0x30},
	{0x3a10, 0x28},
	{0x3a1b, 0x30},
	{0x3a1e, 0x26},
	{0x3a11, 0x60},
	{0x3a1f, 0x14},
	{0x3035, 0x11},
	{0x3503, 0x00},
	{0xFFFF, 0x00}
};

/* Find a data format by a pixel code in an array */
static int ov5640_find_datafmt(int code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov5640_fmts); i++)
		if (ov5640_fmts[i].code == code)
			break;
	/* If not found, select latest */
	if (i >= ARRAY_SIZE(ov5640_fmts))
		i = ARRAY_SIZE(ov5640_fmts) - 1;

	return i;
}

/* Find a frame size in an array */
static int ov5640_find_framesize(u32 width, u32 height)
{
	int i;

	for (i = 0; i < OV5640_SIZE_LAST; i++) {
		if ((ov5640_frmsizes[i].width >= width) &&
			(ov5640_frmsizes[i].height >= height))
			break;
	}
	/* If not found, select biggest */
	if (i >= OV5640_SIZE_LAST)
		i = OV5640_SIZE_LAST - 1;

	return i;
}

static struct ov5640_priv *to_ov5640_priv(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client),
			struct ov5640_priv, subdev);
}

static int ov5640_reg_read(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	unsigned char data[2] = { (u8) reg >> 8, (u8) reg & 0xff };

	ret = i2c_master_send(client, data, 2);
	if (ret < 2)
		goto err;

	ret = i2c_master_recv(client, val, 1);
	if (ret < 1)
		goto err;
	return 0;
err:
	dev_err(&client->dev, "%s: i2c read error, reg: 0x%x\n", __func__, reg);
	return ret < 0 ? ret : -EIO;
}

static int ov5640_reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { (u8) (reg >> 8), (u8) (reg & 0xff), val };

	ret = i2c_master_send(client, data, 3);
	if (ret < 3) {
		dev_err(&client->dev, "%s: i2c write error, 0xreg: %x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static int ov5640_reg_writes(struct i2c_client *client,
		const struct ov5640_reg reglist[])
{
	int err = 0, index;

	for (index = 0; ((reglist[index].reg != 0xFFFF) && (err == 0));
			index++) {
		err |=
			ov5640_reg_write(client, reglist[index].reg,
					reglist[index].val);
		/*  Check for Pause condition */
		if ((reglist[index + 1].reg == 0xFFFF)
				&& (reglist[index + 1].val != 0)) {
			msleep(reglist[index + 1].val);
			index += 1;
		}
	}
	return 0;
}

static int ov5640_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640_priv *ov5640_priv = to_ov5640_priv(client);
	int ret = 0;

	if (enable) {
		dev_dbg(&client->dev, "%s\n", __func__);
		/* Initialize and global setting */
		ret = ov5640_reg_writes(client, ov5640_settings_omnivision_3);
		if (ret)
			return ret;
		/* set format */
		switch ((u32) ov5640_fmts[ov5640_priv->i_fmt].code) {
		case MEDIA_BUS_FMT_UYVY8_2X8:
			dev_dbg(&client->dev, "%s FMT_UYVY8_2X8\n", __func__);
			/* Format Control: 0x32 YUV422 , UYVY order */
			ret = ov5640_reg_write(client, 0x4300, 0x32);
			if (ret)
				return ret;
			break;
		case MEDIA_BUS_FMT_YUYV8_2X8:
			dev_dbg(&client->dev, "%s FMT_YUYV8_2X8\n", __func__);
			/* Format Control: 0x32 YUV422 , YUYV order */
			ret = ov5640_reg_write(client, 0x4300, 0x33);
			if (ret)
				return ret;
			break;
		case MEDIA_BUS_FMT_RGB565_2X8_LE:
			dev_dbg(&client->dev, "%s FMT_RGB565_2X8_LE\n",
								__func__);
			/* Format Control: 0x60 RGB565 , bgr order */
			ret = ov5640_reg_write(client, 0x4300, 0x60);
			/* Format MUX: ISP RGB */
			ret = ov5640_reg_write(client, 0x501F, 0x01);
			if (ret)
				return ret;
			break;
		case MEDIA_BUS_FMT_JPEG_1X8:
			ret = ov5640_reg_writes(client,	jpeg_init_common);
			if (ret)
				return ret;
			break;
		default:
			/* This shouldn't happen */
			ret = -EINVAL;
			return ret;
		}
		/* stream */
		ret = ov5640_reg_writes(client, ov5640_stream);
		if (ret)
			return ret;
	} else {
		ret = ov5640_reg_writes(client, ov5640_power_down);
	}
	return ret;
}

static int ov5640_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(ov5640_fmts))
		return -EINVAL;

	code->code = ov5640_fmts[code->index].code;
	return 0;
}

static int ov5640_g_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640_priv *ov5640_priv = to_ov5640_priv(client);

	mf->width = ov5640_frmsizes[ov5640_priv->i_size].width;
	mf->height = ov5640_frmsizes[ov5640_priv->i_size].height;
	mf->code = ov5640_fmts[ov5640_priv->i_fmt].code;
	mf->colorspace = ov5640_fmts[ov5640_priv->i_fmt].colorspace;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int ov5640_s_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640_priv *ov5640_priv = to_ov5640_priv(client);
	int ret = 0;
	int i_fmt, i_size;

	i_fmt = ov5640_find_datafmt(mf->code);

	mf->code = ov5640_fmts[i_fmt].code;
	mf->colorspace = ov5640_fmts[i_fmt].colorspace;
	mf->field = V4L2_FIELD_NONE;

	i_size = ov5640_find_framesize(mf->width, mf->height);

	mf->width = ov5640_frmsizes[i_size].width;
	mf->height = ov5640_frmsizes[i_size].height;

	ov5640_priv->i_size = ov5640_find_framesize(mf->width, mf->height);
	ov5640_priv->i_fmt = ov5640_find_datafmt(mf->code);

	switch ((u32) ov5640_fmts[ov5640_priv->i_fmt].code) {
	case MEDIA_BUS_FMT_UYVY8_2X8:
		dev_dbg(&client->dev, "%s FMT_UYVY8_2X8\n", __func__);
		break;
	case MEDIA_BUS_FMT_YUYV8_2X8:
		dev_dbg(&client->dev, "%s FMT_YUYV8_2X8\n", __func__);
		break;
	case MEDIA_BUS_FMT_RGB565_2X8_LE:
		dev_dbg(&client->dev, "%s FMT_RGB565_2X8_LE\n", __func__);
		break;
	case MEDIA_BUS_FMT_JPEG_1X8:
		break;
	default:
		/* This shouldn't happen */
		ret = -EINVAL;
		return ret;
	}
	return ret;
}

static int ov5640_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct v4l2_captureparm *cparm;

	if (param->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	cparm = &param->parm.capture;
	memset(param, 0, sizeof(*param));

	param->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	/* Configured only for VGA resolution */
	cparm->timeperframe.numerator = 1;
	cparm->timeperframe.denominator = 24;

	return 0;
}

static int ov5640_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	return ov5640_g_parm(sd, param);
}

static const struct v4l2_subdev_video_ops ov5640_subdev_video_ops = {
	.s_stream = ov5640_s_stream,
	.g_parm = ov5640_g_parm,
	.s_parm = ov5640_s_parm,
};

static const struct v4l2_subdev_pad_ops ov5640_subdev_pad_ops = {
	.enum_mbus_code = ov5640_enum_mbus_code,
	.get_fmt        = ov5640_g_fmt,
	.set_fmt        = ov5640_s_fmt,
};

static int ov5640_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	/* Quantity of initial bad frames to skip. Revisit. */
	*frames = 0;
	return 0;
}

static const struct v4l2_subdev_sensor_ops ov5640_subdev_sensor_ops = {
	.g_skip_frames = ov5640_g_skip_frames,
};

static const struct v4l2_subdev_ops ov5640_subdev_ops = {
	.video = &ov5640_subdev_video_ops,
	.sensor = &ov5640_subdev_sensor_ops,
	.pad    = &ov5640_subdev_pad_ops,
};

/*
 * Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one
 */
static int ov5640_video_probe(struct i2c_client *client)
{
	unsigned long flags;
	int ret = 0;
	u8 revision = 0;
	struct ov5640_priv *priv = to_ov5640_priv(client);

	ret = ov5640_reg_read(client, 0x302A, &revision);
	if (ret) {
		dev_err(&client->dev, "I2C read error [%d]\n", ret);
		goto out;
	}
	revision &= 0xF;
	dev_dbg(&client->dev, "revision  is 0x%x\n", revision);

	ret = ov5640_reg_read(client, 0x300A, &revision);
	if (ret) {
		dev_err(&client->dev, "I2C read error\n");
		goto out;
	}

	dev_dbg(&client->dev, "chip id high is 0x%x\n", revision);
	ret = ov5640_reg_read(client, 0x300B, &revision);
	if (ret) {
		dev_err(&client->dev, "I2C read error\n");
		goto out;
	}

	dev_dbg(&client->dev, "chip id low is 0x%x\n", revision);
	ret = ov5640_reg_read(client, 0x3100, &revision);
	if (ret) {
		dev_err(&client->dev, "I2C read error\n");
		goto out;
	}
	dev_dbg(&client->dev, "I2C slave id is 0x%x\n", revision);

	flags = SOCAM_DATAWIDTH_8;
	ret = v4l2_ctrl_handler_setup(&priv->hdl);
out:
	return ret;
}

static int ov5640_probe(struct i2c_client *client,
		const struct i2c_device_id *did)
{
	int ret;
	u8 sample = 0;
	struct ov5640_priv *priv;
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);

	if (!ssdd && !client->dev.of_node) {
		dev_err(&client->dev, "OV5640: missing platform data!\n");
		return -EPROBE_DEFER;
	}
	/*
	* Sample i2c read to check if the sensor module has been powered or not.
	* The power to sensor is not being provided with a regulator, but by
	* following proper power reset from the host IP.
	*/
	ret = ov5640_reg_read(client, 0x302A, &sample);
	if (ret == -ENXIO) {
		dev_err(&client->dev, "NAK Addr/Data\n");
		return -EPROBE_DEFER;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct ov5640_priv),
		GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&priv->subdev, client, &ov5640_subdev_ops);
	v4l2_ctrl_handler_init(&priv->hdl, NUM_STD_CONTROLS);

	priv->subdev.ctrl_handler = &priv->hdl;
	priv->subdev.dev = &client->dev;

	ret = ov5640_video_probe(client);
	if (ret)
		v4l2_ctrl_handler_free(&priv->hdl);

	ret = v4l2_async_register_subdev(&priv->subdev);
	if (ret < 0)
		v4l2_ctrl_handler_free(&priv->hdl);

	dev_info(&client->dev, "OV5640 Probed\n");

	return ret;
}

static int ov5640_remove(struct i2c_client *client)
{
	struct ov5640_priv *priv = to_ov5640_priv(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_device_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
	return 0;
}

static const struct i2c_device_id ov5640_id[] = {
	{"ov5640", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ov5640_id);

static const struct of_device_id ov5640_of_match[] = {
	{.compatible = "ovti,ov5640", },
	{},
};
MODULE_DEVICE_TABLE(of, ov5640_of_match);

static struct i2c_driver ov5640_i2c_driver = {
	.driver = {
		.name = "ov5640",
		.of_match_table = of_match_ptr(ov5640_of_match),
	},
	.probe = ov5640_probe,
	.remove = ov5640_remove,
	.id_table = ov5640_id,
};

module_i2c_driver(ov5640_i2c_driver);

MODULE_DESCRIPTION("OmniVision OV5640 Camera driver");
MODULE_AUTHOR("Sergio Aguirre <saaguirre@ti.com>");
MODULE_LICENSE("GPL v2");
