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
 * V4L2 Driver for unicam cpi(parallel) camera host
 */

#include <linux/mfd/syscon.h>
#include <media/videobuf2-core.h>
#include <media/v4l2-mediabus.h>

#include "bcm_unicam_cpi.h"
/*
 * Unicam camera host exports
 */
#define UNICAM_BUF_MAGIC		0xBABEFACE
#define UNICAM_CAM_DRV_NAME		"unicam-camera"

#define CPI_ISR_FE              0x2
#define CPI_ISR_EMSK1           1
#define CPI_ISR_EBITS1          7
#define CPI_ISR_EMSK2           0xf
#define CPI_ISR_EBITS2          2

#define BUFCUR_FLAG_BOTH        0x55aa
#define BUFCUR_FLAG_1           0x55
#define BUFCUR_FLAG_2           0xaa

#define MODULE_NAME_MAX 20

char const unicam_camera_name[] = "iproc-camera";

struct unicam_camera_buffer {
	struct vb2_v4l2_buffer vb; /* v4l buffer must be first */
	struct list_head queue;
	unsigned int magic;
};

static struct unicam_camera_buffer *to_unicam_camera_vb(
	struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct unicam_camera_buffer, vb);
}

static int unicam_videobuf_setup(struct vb2_queue *vq,
		unsigned int *count, unsigned int *numplanes,
		unsigned int sizes[], struct device *alloc_devs[])
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
			icd->current_fmt->host_fmt);

	if (bytes_per_line < 0) {
		dev_err(icd->parent, "[%s] bytes_per_line %d\n",
						__func__, bytes_per_line);
		return bytes_per_line;
	}

	*numplanes = 1;

	sizes[0] = bytes_per_line * icd->user_height;

	if (!*count)
		*count = 2;
	return 0;
}

static int unicam_videobuf_prepare(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
			icd->current_fmt->host_fmt);
	unsigned long size;

	if (bytes_per_line < 0) {
		dev_err(icd->parent, "[%s]: bytes_per_line %d\n",
						__func__, bytes_per_line);
		return bytes_per_line;
	}

	size = icd->user_height * bytes_per_line;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(icd->parent, "[%s]: not fit to plane (%lu < %lu)\n",
					__func__, vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

static void unicam_camera_update_buf(struct unicam_camera_dev *unicam_dev)
{
	struct buffer_desc im0, im1;
	dma_addr_t dma_addr;
	unsigned int line_stride;

	dma_addr = vb2_dma_contig_plane_dma_addr(
		&unicam_dev->active->vb2_buf, 0);
	if (!dma_addr) {
		dev_err(unicam_dev->icd->parent, "[%s] NOMEM\n", __func__);
		unicam_dev->active = NULL;
		return;
	}

	if (unicam_dev->icd->current_fmt->code != MEDIA_BUS_FMT_JPEG_1X8) {
		if ((unicam_dev->sel.r.top == 0) ||
				(unicam_dev->sel.r.left == 0)) {
			line_stride = soc_mbus_bytes_per_line(
					unicam_dev->icd->user_width,
					unicam_dev->icd->current_fmt->host_fmt);
		} else {
			line_stride = soc_mbus_bytes_per_line(
					unicam_dev->sel.r.width,
					unicam_dev->icd->current_fmt->host_fmt);
		}

		im0.start = (unsigned long) dma_addr;
		im0.ls = (unsigned long) line_stride;
		im0.size = line_stride * unicam_dev->icd->user_height;

		if (unicam_dev->curr == BUFCUR_FLAG_BOTH) {
			/* image 1 */
			im1.start = dma_addr;
			im1.ls = im0.ls;
			im1.size = im0.size;
			unicam_cpi_update_addr(unicam_dev, &im0, &im1,
								NULL, NULL);
		} else if (unicam_dev->curr == BUFCUR_FLAG_1) {
			unicam_cpi_update_addr(unicam_dev, &im0, NULL,
								NULL, NULL);
		} else if (unicam_dev->curr == BUFCUR_FLAG_2) {
			unicam_cpi_update_addr(unicam_dev, NULL, &im0,
								NULL, NULL);
		}
	}
}

static void unicam_camera_capture(struct unicam_camera_dev *unicam_dev)
{
	if (!unicam_dev->active || unicam_dev->b_mode != BUFFER_TRIGGER)
		return;

	unicam_cpi_trigger_cap(unicam_dev);
}

static void unicam_videobuf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;
	struct unicam_camera_buffer *buf = to_unicam_camera_vb(vbuf);
	unsigned long flags;

	dev_dbg(icd->parent, "vb=0x%p pbuf=0x%p size=%lu\n", vb,
				(void *) vb2_dma_contig_plane_dma_addr(vb, 0),
				vb2_get_plane_payload(vb, 0));

	spin_lock_irqsave(&unicam_dev->lock, flags);
	list_add_tail(&buf->queue, &unicam_dev->video_buffer_list);

	if ((!unicam_dev->active)) {
		unicam_dev->active = vbuf;
		unicam_camera_update_buf(unicam_dev);
		unicam_camera_capture(unicam_dev);
	}
	spin_unlock_irqrestore(&unicam_dev->lock, flags);
}

static void unicam_videobuf_release(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;
	struct unicam_camera_buffer *buf = to_unicam_camera_vb(vbuf);
	unsigned long flags;

	 dev_dbg(icd->parent, "vb=0x%p pbuf=0x%p size=%lu\n", vb,
				(void *) vb2_dma_contig_plane_dma_addr(vb, 0),
				vb2_get_plane_payload(vb, 0));

	spin_lock_irqsave(&unicam_dev->lock, flags);

	if (buf->magic == UNICAM_BUF_MAGIC)
		list_del_init(&buf->queue);
	spin_unlock_irqrestore(&unicam_dev->lock, flags);

}

static int unicam_videobuf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct unicam_camera_buffer *buf = to_unicam_camera_vb(vbuf);

	INIT_LIST_HEAD(&buf->queue);
	buf->magic = UNICAM_BUF_MAGIC;
	return 0;
}

static void unicam_videobuf_start_streaming_int(
			struct unicam_camera_dev *unicam_dev,
					unsigned int count)
{
	struct soc_camera_device *icd = unicam_dev->icd;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	int ret;

	unicam_dev->panic_count = 0;

	unicam_dev->b_mode = BUFFER_DOUBLE;

	if ((unicam_dev->cam_state.devbusy == true)
			|| (unicam_dev->cam_state.mode != INVALID)) {
		dev_err(icd->parent, "Error : Already Instantiated\n");
		return;
	}

	unicam_dev->cam_state.devbusy = true;
	unicam_dev->cam_state.mode = DVP;
	unicam_dev->cam_state.trigger = false;
	unicam_dev->cam_state.db_en = true;

	/* Configure CPI IP for capture */
	unicam_camera_cpi_config(unicam_dev);
	/* Camera Module RESET & STANDBY */
	unicam_camera_reset_standby(unicam_dev);

	if (unicam_dev->active)
		atomic_set(&unicam_dev->streaming, 1);

	ret = v4l2_subdev_call(sd, video, s_stream, 1);

	if (ret < 0 && ret != -ENOIOCTLCMD)
		dev_err(icd->parent, "Error subdev s_stream\n");
}

int unicam_videobuf_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(q);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev =  ici->priv;

	if (!atomic_read(&unicam_dev->streaming))
		unicam_videobuf_start_streaming_int(unicam_dev, count);
	else
		dev_err(icd->parent, "[%s]: Already started\n", __func__);

	unicam_camera_capture(unicam_dev);
	return 0;
}

static void unicam_videobuf_stop_streaming_int(
		struct unicam_camera_dev *unicam_dev)
{
	struct soc_camera_device *icd = unicam_dev->icd;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	int ret;
	unsigned long flags;

	if (unicam_dev->active) {
		spin_lock_irqsave(&unicam_dev->lock, flags);
		if (unicam_dev->camera_mode_continuous) {
			/* Release the last buffer */
			vb2_buffer_done(&unicam_dev->active->vb2_buf,
				VB2_BUF_STATE_DONE);
		}
		spin_unlock_irqrestore(&unicam_dev->lock, flags);
		atomic_set(&unicam_dev->stopping, 1);
		ret = wait_for_completion_interruptible_timeout(
				&unicam_dev->stop,
				msecs_to_jiffies(500));
		atomic_set(&unicam_dev->stopping, 0);
		if (ret <= 0) {
			dev_err(icd->parent,
				"[%s]: Stop stream timed out\n", __func__);
			return;
		}
	}

	spin_lock_irqsave(&unicam_dev->lock, flags);
	disable_cpi_interface(unicam_dev);
	unicam_cpi_get_rx_stat(unicam_dev, 1);

	memset(&unicam_dev->cam_state, 0x0, sizeof(struct unicam_cpi_generic));
	unicam_dev->cam_state.devbusy = false;
	unicam_dev->cam_state.mode = INVALID;

	unicam_dev->active = NULL;

	memset(&unicam_dev->sel, 0x00, sizeof(struct v4l2_selection));
	unicam_dev->last_buffer_in_queue = false;
	spin_unlock_irqrestore(&unicam_dev->lock, flags);

	atomic_set(&unicam_dev->streaming, 0);

	ret = v4l2_subdev_call(sd, video, s_stream, 0);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		dev_err(icd->parent,
			"[%s]: Failed to stop sensor streaming\n", __func__);
	}
}

static void unicam_videobuf_stop_streaming(struct vb2_queue *q)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(q);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;

	if (atomic_read(&unicam_dev->streaming))
		unicam_videobuf_stop_streaming_int(unicam_dev);
	else
		dev_err(icd->parent, "[%s]: Already stopped\n", __func__);
}

static struct vb2_ops unicam_videobuf_ops = {
	.queue_setup = unicam_videobuf_setup,
	.buf_prepare = unicam_videobuf_prepare,
	.buf_queue = unicam_videobuf_queue,
	.buf_cleanup = unicam_videobuf_release,
	.buf_init = unicam_videobuf_init,
	.start_streaming = unicam_videobuf_start_streaming,
	.stop_streaming = unicam_videobuf_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish
};

static int unicam_camera_init_videobuf(struct vb2_queue *q,
		struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_READ;
	q->drv_priv = icd;
	q->ops = &unicam_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct unicam_camera_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->dev = ici->v4l2_dev.dev;
	q->lock = &ici->host_lock;

	return vb2_queue_init(q);
}

static int unicam_camera_set_bus_param(struct soc_camera_device *icd)
{
	dev_dbg(icd->parent, "unicam_camera_set_bus_param\n");
	return 0;
}

static int unicam_camera_querycap(struct soc_camera_host *ici,
			struct v4l2_capability *cap)
{
	strlcpy(cap->driver, "bcm_unicam_cpi", sizeof(cap->driver));
	strlcpy(cap->card, "Unicam Camera", sizeof(cap->card));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	snprintf(cap->bus_info, sizeof(cap->bus_info),
			"platform:%s%d", UNICAM_CAM_DRV_NAME, ici->nr);
	return 0;
}

static unsigned int unicam_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;

	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static int unicam_camera_try_fmt(struct soc_camera_device *icd,
						struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev_pad_config pad_cfg;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};
	struct v4l2_mbus_framefmt *mf = &format.format;
	u32 pixfmt = pix->pixelformat;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_warn(icd->parent, "Format %x not found\n", pixfmt);
		return -EINVAL;
	}
	pix->sizeimage = pix->height * pix->bytesperline;
	/* limit to sensor capabilities */
	mf->width = pix->width;
	mf->height = pix->height;
	mf->field = pix->field;
	mf->colorspace = pix->colorspace;
	mf->code = xlate->code;

	ret = v4l2_subdev_call(sd, pad, set_fmt, &pad_cfg, &format);
	if (ret < 0)
		return ret;

	pix->width = mf->width;
	pix->height = mf->height;
	pix->colorspace = mf->colorspace;

	switch (mf->field) {
	case V4L2_FIELD_ANY:
	case V4L2_FIELD_NONE:
		pix->field = V4L2_FIELD_NONE;
		break;
	default:
		dev_err(icd->parent, "Field type %d unsupported.\n", mf->field);
		return -EINVAL;
	}
	switch (mf->code) {
	case MEDIA_BUS_FMT_JPEG_1X8:
	case MEDIA_BUS_FMT_YUYV8_2X8:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	case MEDIA_BUS_FMT_RGB565_2X8_LE:
		/* Above formats are supported */
		break;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		pix->bytesperline =  ALIGN(((pix->width * 10) >> 3), 32);
#ifdef UNPACK_RAW10_TO_16BITS
		pix->bytesperline = ((pix->width * 2), 32);
#endif
		break;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		pix->bytesperline = ALIGN(pix->width, 32);
		break;
	default:
		dev_err(icd->parent, "Sensor format code %d unsupported.\n",
			mf->code);
		return -EINVAL;
	}
	return ret;
}

static int unicam_camera_get_selection(struct soc_camera_device *icd,
				       struct v4l2_selection *sel)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;

	if (sel != NULL)
		*sel = unicam_dev->sel;

	return 0;
}

static int unicam_camera_set_selection(struct soc_camera_device *icd,
				       struct v4l2_selection *sel)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;

	if (sel == NULL)
		return -EINVAL;

	unicam_dev->sel = *sel;
	return 0;
}

static int unicam_camera_set_fmt_int(struct unicam_camera_dev *unicam_dev)
{
	struct soc_camera_device *icd = unicam_dev->icd;
	struct device *dev = icd->parent;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate = NULL;
	struct v4l2_format *f = &(unicam_dev->active_fmt);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct v4l2_mbus_framefmt *mf = &format.format;
	int ret;
	unsigned int skip_frames;

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_err(dev, "Format %x not found\n", pix->pixelformat);
		return -EINVAL;
	}
	mf->width = pix->width;
	mf->height = pix->height;
	mf->field = pix->field;
	mf->colorspace = pix->colorspace;
	mf->code = xlate->code;

	ret = v4l2_subdev_call(sd, pad, set_fmt, NULL, &format);
	if (ret < 0) {
		dev_err(icd->parent, "Error s_mbus_fmt error\n");
		return ret;
	}
	ret = v4l2_subdev_call(sd, sensor, g_skip_frames, &skip_frames);
	if (ret < 0) {
		dev_err(icd->parent, "Error subdev g_skip_frames\n");
		skip_frames = 0;
		ret = 0;
	}
	unicam_dev->skip_frames = skip_frames;

	pix->width = mf->width;
	pix->height = mf->height;
	pix->field = mf->field;
	pix->colorspace = mf->colorspace;
	icd->current_fmt = xlate;
	/* Initialize selection window for now */
	unicam_dev->sel.r.width = pix->width;
	unicam_dev->sel.r.height = pix->height;
	unicam_dev->sel.r.top = unicam_dev->sel.r.left = 0;

	return ret;
}

static int unicam_camera_set_fmt(struct soc_camera_device *icd,
		struct v4l2_format *f)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;
	int ret;

	unicam_dev->active_fmt = *f;
	ret = unicam_camera_set_fmt_int(unicam_dev);
	return ret;
}

static int unicam_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	int err;

	if (unicam_dev->icd) {
		dev_warn(icd->parent,
		"Unicam camera driver already attached to another client\n");
		return -EBUSY;
	}
	err = v4l2_subdev_call(sd, core, s_power, 1);
	if (err < 0 && err != -ENOIOCTLCMD) {
		dev_err(icd->parent, "Could not power up subdevice\n");
		return err;
	}
	err = 0;
	unicam_dev->icd = icd;

	return err;
}

static void unicam_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);

	WARN_ON(icd != unicam_dev->icd);

	if (atomic_read(&unicam_dev->streaming))
		unicam_videobuf_stop_streaming(&icd->vb2_vidq);

	v4l2_subdev_call(sd, core, s_power, 0);
	unicam_dev->icd = NULL;

	dev_info(icd->parent,
			"Unicam CPI driver detached from camera %d\n",
			icd->devnum);
}

static struct soc_camera_host_ops unicam_soc_camera_host_ops = {
	.owner = THIS_MODULE,
	.add = unicam_camera_add_device,
	.remove = unicam_camera_remove_device,
	.set_fmt = unicam_camera_set_fmt,
	.try_fmt = unicam_camera_try_fmt,
	.init_videobuf2 = unicam_camera_init_videobuf,
	.poll = unicam_camera_poll,
	.querycap = unicam_camera_querycap,
	.set_bus_param = unicam_camera_set_bus_param,
	.set_selection = unicam_camera_set_selection,
	.get_selection = unicam_camera_get_selection,
};

static irqreturn_t unicam_camera_isr(int irq, void *arg)
{
	struct unicam_camera_dev *unicam_dev =
		(struct unicam_camera_dev *) arg;
	unsigned int isr_status, raw_stat;
	unsigned long flags;
	struct vb2_v4l2_buffer *vbuf = unicam_dev->active;

	if (atomic_read(&unicam_dev->streaming) == 0 || !vbuf)
		return IRQ_NONE;

	isr_status = unicam_cpi_get_int_stat(unicam_dev, 0, 0);
	raw_stat = unicam_cpi_get_rx_stat(unicam_dev, 1);

	dev_dbg(unicam_dev->icd->parent,
			"[%s] isr: %x\n", __func__, isr_status);

	if (isr_status == 0) {
		/* ignore the status 0 interrupt */
		return IRQ_NONE;
	} else if (isr_status & CPI_ISR_FE) {/* FE interrupt */

		isr_status = unicam_cpi_get_int_stat(unicam_dev, raw_stat, 0);
		/* FS and FE handling */
		if (raw_stat & ((CPI_ISR_EMSK1 << CPI_ISR_EBITS1)|
					(CPI_ISR_EMSK2 << CPI_ISR_EBITS2))) {
			dev_err(unicam_dev->icd->parent, "ISR: data error");
			goto out;
		}
		if (likely(unicam_dev->skip_frames <= 0)) {

			spin_lock_irqsave(&unicam_dev->lock, flags);
			list_del_init(&to_unicam_camera_vb(vbuf)->queue);

			if (atomic_read(&unicam_dev->stopping) == 1) {
				complete(&unicam_dev->stop);
				unicam_dev->active = NULL;
			} else if (!list_empty
					(&unicam_dev->video_buffer_list)) {
				unicam_dev->active =
				&list_entry(unicam_dev->video_buffer_list.next,
						struct unicam_camera_buffer,
						queue)->vb;
			} else {
				if (!unicam_dev->camera_mode_continuous)
					unicam_dev->active = NULL;
				else
					/* Do not release last buffer in
					* Continuous mode as camera HW
					* needs one buffer to write
					*/
					unicam_dev->last_buffer_in_queue = true;
			}
			spin_unlock_irqrestore(&unicam_dev->lock, flags);

			if (!unicam_dev->last_buffer_in_queue)
				vb2_buffer_done(&vbuf->vb2_buf,
					VB2_BUF_STATE_DONE);

		} else {
			unicam_dev->skip_frames--;
		}
		if (unicam_dev->active && !unicam_dev->last_buffer_in_queue) {
			if (unicam_dev->icd->current_fmt->code !=
					MEDIA_BUS_FMT_JPEG_1X8) {
				unicam_camera_update_buf(unicam_dev);
			}
			unicam_camera_capture(unicam_dev);
		}
		unicam_dev->last_buffer_in_queue = false;
	}
out:
	return IRQ_HANDLED;
}

/* Camera uses audio pll channel 2 to generate a fixed reference clock
*	of 24.576Mhz. Reconfiguring this pll channel's frequency is not
*	advisable for the smooth operation of Camera
*/
static int unicam_clk_notify(struct notifier_block *nb,
					unsigned long action, void *mclk)
{
	if (action == PRE_RATE_CHANGE)
		return NOTIFY_BAD;

	return NOTIFY_DONE;
}

static struct notifier_block unicam_clk_nb = {
	.notifier_call = unicam_clk_notify,
};

static void * __init get_of_camera_data(struct platform_device *pdev,
		struct unicam_camera_dev *pcdev)
{
	struct soc_camera_desc *sdesc_camera;
	unsigned int val, clk_rate;
	struct device_node *np =  pdev->dev.of_node;
	int ret = -ENXIO;

	sdesc_camera = devm_kzalloc(&pdev->dev, sizeof(*sdesc_camera),
			GFP_KERNEL);

	if (of_property_read_u32(np, "brcm,unicam-data-width", &val))
		goto fail_out;
	pcdev->data_width = val;

	/* Defines Capture Mode of Camera
	*  Single-Shot or Continuous
	*/
	if (of_get_property(np, "cam-mode-cont", NULL))
		pcdev->camera_mode_continuous = true;
	else
		pcdev->camera_mode_continuous = false;

	/* Only applicable in Continuous Mode */
	pcdev->last_buffer_in_queue = false;

	pcdev->clk = devm_clk_get(&pdev->dev, "cam-clock");
	if (IS_ERR(pcdev->clk)) {
		dev_err(&pdev->dev, "Failed to get AUDIO_CH2 pll clock\n");
		ret = -ENODEV;
		goto fail_out;
	}

	ret = of_property_read_u32(np, "clock-frequency", &clk_rate);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to parse ref_clock\n");
		ret = -EINVAL;
		goto fail_out;
	}
	pcdev->clk_freq = clk_rate;

	ret = clk_set_rate(pcdev->clk, pcdev->clk_freq);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set to %u\n", pcdev->clk_freq);
		ret = -EINVAL;
		goto fail_out;
	}

	ret = clk_prepare_enable(pcdev->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clock\n");
		ret = -EINVAL;
		goto fail_out;
	}

	ret = clk_notifier_register(pcdev->clk, &unicam_clk_nb);
	if (ret) {
		dev_err(&pdev->dev, "Failed clk_notifier_register [%u]\n", ret);
		ret = -EINVAL;
		goto fail_cam_notifier;
	}

	pcdev->asiu_clk = devm_clk_get(&pdev->dev, "cam-gate-clk");
	if (IS_ERR(pcdev->asiu_clk)) {
		dev_err(&pdev->dev, "Failed to get cam gate clock\n");
		ret = PTR_ERR(pcdev->asiu_clk);
		goto fail_cam_gate_clk;
	}

	ret = clk_prepare_enable(pcdev->asiu_clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable cam gate clock\n");
		ret = -EINVAL;
		goto fail_cam_gate_clk;
	}

	return sdesc_camera;

fail_cam_gate_clk:
	clk_notifier_unregister(pcdev->clk, &unicam_clk_nb);
fail_cam_notifier:
	clk_disable_unprepare(pcdev->clk);
fail_out:
	dev_err(&pdev->dev, "get_of_camera_data failed, err [0x%x]\n", ret);
	return NULL;
}

static int unicam_camera_probe(struct platform_device *pdev)
{
	struct unicam_camera_dev *pcdev;
	struct resource *res;
	void __iomem *base;
	void __iomem *asiu_pad_ctrl;
	unsigned int irq;
	int err = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	asiu_pad_ctrl = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							"brcm,asiu-padctrl");
	if (IS_ERR(asiu_pad_ctrl))
		return PTR_ERR(asiu_pad_ctrl);

	irq = platform_get_irq(pdev, 0);
	if ((int)irq <= 0) {
		dev_err(&pdev->dev, "Failed to get IRQ for the device\n");
		return -ENODEV;
	}

	pcdev = devm_kzalloc(&pdev->dev, sizeof(*pcdev), GFP_KERNEL);
	pcdev->irq = irq;
	pcdev->base = base;
	pcdev->asiu_pad_ctrl = asiu_pad_ctrl;

	INIT_LIST_HEAD(&pcdev->video_buffer_list);
	spin_lock_init(&pcdev->lock);

	pcdev->pdata = pdev->dev.platform_data;
	if (!pcdev->pdata) {
		pdev->dev.platform_data = get_of_camera_data(pdev, pcdev);
		pcdev->pdata = pdev->dev.platform_data;
		if (!pcdev->pdata) {
			dev_err(&pdev->dev,
				"Unicam CPI platform data not available.\n");
			return -EINVAL;
		}
		pdev->id = 0;
	}

	err = devm_request_irq(&pdev->dev, pcdev->irq, unicam_camera_isr,
			0, dev_name(&pdev->dev), pcdev);
	if (err) {
		dev_err(&pdev->dev, "Unable to register CPI interrupt.\n");
		goto exit_release;
	}

	pcdev->soc_host.priv = pcdev;
	pcdev->soc_host.v4l2_dev.dev = &pdev->dev;
	pcdev->soc_host.nr = pdev->id;
	pcdev->soc_host.drv_name = dev_name(&pdev->dev);
	pcdev->soc_host.ops = &unicam_soc_camera_host_ops;

	pcdev->cam_state.devbusy = false;
	pcdev->cam_state.mode = INVALID;

	pcdev->b_mode = BUFFER_DOUBLE;
	pcdev->curr = BUFCUR_FLAG_1;
	atomic_set(&pcdev->stopping, 0);
	init_completion(&pcdev->stop);

	/* Camera Module RESET & STANDBY */
	unicam_camera_reset_standby(pcdev);

	err = soc_camera_host_register(&pcdev->soc_host);
	if (err) {
		dev_err(&pdev->dev,
			"soc_camera_host_register failed err =0x%x", err);
		goto exit_release;
	}

	return 0;

exit_release:
	return err;
}

static int unicam_camera_remove(struct platform_device *pdev)
{
	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct unicam_camera_dev *pcdev =  soc_host->priv;

	soc_camera_host_unregister(soc_host);
	clk_disable_unprepare(pcdev->asiu_clk);
	clk_notifier_unregister(pcdev->clk, &unicam_clk_nb);
	clk_disable_unprepare(pcdev->clk);
	return 0;
}

static const struct of_device_id unicam_cpi_of_match[] = {
	{ .compatible = "brcm,unicamcpi", },
	{},
};
static struct platform_driver unicam_camera_driver = {
	.driver = {
		.name = UNICAM_CAM_DRV_NAME,
		.of_match_table = unicam_cpi_of_match,
	},
	.probe = unicam_camera_probe,
	.remove = unicam_camera_remove,
};

module_platform_driver(unicam_camera_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Unicam Camera CPI (parallel) Host driver");
MODULE_AUTHOR("Broadcom");
