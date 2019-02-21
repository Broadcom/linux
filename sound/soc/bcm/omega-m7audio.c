// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Broadcom
 */
#include <linux/bcm_iproc_mailbox.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define IPC_CMD_ID_SHIFT	0
#define IPC_CMD_ID_WIDTH	8
#define IPC_CMD_ID_MASK		(BIT(IPC_CMD_ID_WIDTH) - 1)

#define IPC_CMD_PROTO_ID_SHIFT	10
#define IPC_CMD_PROTO_ID_WIDTH	8
#define IPC_CMD_PROTO_ID_MASK	(BIT(IPC_CMD_PROTO_ID_WIDTH) - 1)

#define IPC_CMD_ID_CREATE(proto, msg_id) \
	((((proto) & IPC_CMD_PROTO_ID_MASK) << IPC_CMD_PROTO_ID_SHIFT) | \
	(((msg_id) & IPC_CMD_ID_MASK) << IPC_CMD_ID_SHIFT))

enum ipc_proto_id {
	IPC_PROTO_ID_BCM_LEGACY = 0x0,
	IPC_PROTO_ID_PWR_DMN = 0x11,
	IPC_PROTO_ID_SYS_PWR = 0x12,

	IPC_PROTO_ID_BCM_AUDIO = 0x81,

	IPC_PROTO_ID_BCM_DEBUG = 0x82,
	IPC_PROTO_ID_BCM_TEST = 0x83,
};

enum ipc_cmd_bcm_audio {
	IPC_AUDIO_STOP_CMD = 0x2,
	IPC_AUDIO_CAPTURE_DMA_START_CMD = 0x3,
	IPC_AUDIO_CAPTURE_DMA_STOP_CMD = 0x4,
	IPC_AUDIO_PLAYBACK_DMA_START_CMD = 0x5,
	IPC_AUDIO_PLAYBACK_DMA_STOP_CMD = 0x6,
};

struct ipc_audio_capture_dma {
	/* Physical address of control struct*/
	uint32_t dram_dma_ctrl;
	/* Physical address of DRAM area */
	uint32_t dram_dma_addr;
};

struct ipc_audio_playback_dma {
	/* Input */
	/* Output device
	 * OUT_SP   (1 << 0)
	 * OUT_EP   (1 << 1)
	 * OUT_HS   (1 << 2)
	 * OUT_MONO (1 << 3)
	 */
	uint32_t out_device;
	/* Physical address of control struct*/
	uint32_t dram_dma_ctrl;
	/* Physical address of DRAM area */
	uint32_t dram_dma_addr;
};

/* This structure has to be the same as the one used on the M7 */
struct m7_xfer_info {
	u32 write_index;
	u32 read_index;
	u32 capacity;
	u32 debug[5];
};

struct ipc_audio_msg {
	union {
		struct ipc_audio_playback_dma ipc_tx_msg;
		struct ipc_audio_capture_dma ipc_rx_msg;
	};
};

/* Allow for up to 128K samples, 512KB */
#define SAMPLE_BUF_MAX_U32 (128 * 1024)

/* This struture shows how m7 audio shared memory is organized  */
struct m7_audio_shmem {
	u32 sample_buf_mem[SAMPLE_BUF_MAX_U32];
	struct m7_xfer_info xfer_ctrl_mem;
	struct ipc_audio_msg ipc_msg_mem;
};

struct m7_dma_xfer_info {
	size_t  alloc_size;

	dma_addr_t sample_buf_phys;
	unsigned char *sample_buf;

	dma_addr_t xfer_ctrl_phys;
	struct m7_xfer_info *xfer_ctrl;

	dma_addr_t ipc_msg_phys;
	struct ipc_audio_msg *ipc_msg;
};

struct m7audio_subdata {
	struct hrtimer timer;
	int period_ns;
	struct snd_pcm_substream *substream;

	atomic_t active;

	struct m7_dma_xfer_info xfer_info;
};

struct m7audio_rtd {
	struct m7audio_subdata play;
	struct m7audio_subdata cap;

	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;
};

static const struct snd_pcm_hardware omega_m7_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED,

	.period_bytes_min = 256,
	.period_bytes_max = 256 * 1024,

	.periods_min = 2,
	.periods_max = 255,

	.buffer_bytes_max = SAMPLE_BUF_MAX_U32 * sizeof(u32),
};

static struct m7audio_rtd *get_m7_priv(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	return dev_get_drvdata(rtd->cpu_dai->dev);
}

static struct m7audio_subdata *get_m7_subdata(struct snd_pcm_substream *substr)
{
	struct m7audio_rtd *m7_priv;

	m7_priv = get_m7_priv(substr);

	if (substr->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return &m7_priv->play;
	else
		return &m7_priv->cap;
}

static int ipc_dma_mbox_send_msg(struct mbox_chan *mbox,
				 struct iproc_mbox_msg *msg)
{
	int err;

	err = mbox_send_message(mbox, msg);
	if (err < 0)
		return err;

	mbox_client_txdone(mbox, 0);
	return 0;
}

static int ipc_dma_capture_start(struct mbox_chan *mbox_chan,
				  struct m7_dma_xfer_info *rx_info)
{
	struct iproc_mbox_msg msg;
	struct ipc_audio_capture_dma *ipc_rx_msg;

	ipc_rx_msg = (struct ipc_audio_capture_dma *)rx_info->ipc_msg;

	/* Initialize the IPC parameter */
	ipc_rx_msg->dram_dma_addr = rx_info->sample_buf_phys;
	ipc_rx_msg->dram_dma_ctrl = rx_info->xfer_ctrl_phys;

	msg.cmd = IPC_CMD_ID_CREATE(IPC_PROTO_ID_BCM_AUDIO,
				    IPC_AUDIO_CAPTURE_DMA_START_CMD);
	msg.param = rx_info->ipc_msg_phys;
	msg.wait_ack = true;

	return ipc_dma_mbox_send_msg(mbox_chan, &msg);
}

static int ipc_dma_capture_stop(struct mbox_chan *mbox_chan,
				 struct m7_dma_xfer_info *rx_info)
{
	struct iproc_mbox_msg msg;

	msg.cmd = IPC_CMD_ID_CREATE(IPC_PROTO_ID_BCM_AUDIO,
				    IPC_AUDIO_CAPTURE_DMA_STOP_CMD);
	msg.param = rx_info->ipc_msg_phys;
	msg.wait_ack = true;

	return ipc_dma_mbox_send_msg(mbox_chan, &msg);
}

static int ipc_dma_playback_start(struct mbox_chan *mbox_chan,
				   struct m7_dma_xfer_info *tx_info)

{
	struct iproc_mbox_msg msg;
	struct ipc_audio_playback_dma *ipc_tx_msg;

	ipc_tx_msg = (struct ipc_audio_playback_dma *)tx_info->ipc_msg;

	/* Initialize the IPC parameter */
	ipc_tx_msg->dram_dma_addr = tx_info->sample_buf_phys;
	ipc_tx_msg->dram_dma_ctrl = tx_info->xfer_ctrl_phys;
	ipc_tx_msg->out_device = 0x2;

	msg.cmd = IPC_CMD_ID_CREATE(IPC_PROTO_ID_BCM_AUDIO,
				    IPC_AUDIO_PLAYBACK_DMA_START_CMD);
	msg.param = tx_info->ipc_msg_phys;
	msg.wait_ack = true;

	return ipc_dma_mbox_send_msg(mbox_chan, &msg);
}

static int ipc_dma_playback_stop(struct mbox_chan *mbox_chan,
				  struct m7_dma_xfer_info *tx_info)
{
	struct iproc_mbox_msg msg;

	msg.cmd = IPC_CMD_ID_CREATE(IPC_PROTO_ID_BCM_AUDIO,
				    IPC_AUDIO_PLAYBACK_DMA_STOP_CMD);
	msg.param = tx_info->ipc_msg_phys;
	msg.wait_ack = true;

	return ipc_dma_mbox_send_msg(mbox_chan, &msg);
}

static enum hrtimer_restart m7_hrtimer_play(struct hrtimer *hrt)
{
	struct m7audio_subdata *m7_subdata;
	unsigned int idx;

	m7_subdata = container_of(hrt, struct m7audio_subdata, timer);

	if (!atomic_read(&m7_subdata->active))
		return HRTIMER_NORESTART;

	/* Set to full */
	if (m7_subdata->xfer_info.xfer_ctrl->read_index == 0)
		idx = m7_subdata->xfer_info.xfer_ctrl->capacity - 1;
	else
		idx = m7_subdata->xfer_info.xfer_ctrl->read_index - 1;

	m7_subdata->xfer_info.xfer_ctrl->write_index = idx;

	snd_pcm_period_elapsed(m7_subdata->substream);
	hrtimer_forward_now(hrt, ns_to_ktime(m7_subdata->period_ns));

	return HRTIMER_RESTART;
}

static enum hrtimer_restart m7_hrtimer_capture(struct hrtimer *hrt)
{
	struct m7audio_subdata *m7_subdata;

	m7_subdata = container_of(hrt, struct m7audio_subdata, timer);

	if (!atomic_read(&m7_subdata->active))
		return HRTIMER_NORESTART;

	/* Set to empty */
	m7_subdata->xfer_info.xfer_ctrl->read_index =
			m7_subdata->xfer_info.xfer_ctrl->write_index;

	snd_pcm_period_elapsed(m7_subdata->substream);
	hrtimer_forward_now(hrt, ns_to_ktime(m7_subdata->period_ns));

	return HRTIMER_RESTART;
}

static int m7_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct m7audio_rtd *m7_priv;
	struct m7audio_subdata *m7_subdata;
	int ret;

	dev_dbg(rtd->cpu_dai->dev, "Enter  cmd %d\n", cmd);

	m7_priv = get_m7_priv(substream);
	m7_subdata = get_m7_subdata(substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			ret = ipc_dma_playback_start(m7_priv->mbox_chan,
						     &m7_subdata->xfer_info);
		} else {
			ret = ipc_dma_capture_start(m7_priv->mbox_chan,
						    &m7_subdata->xfer_info);
		}

		if (ret)
			return ret;

		atomic_set(&m7_subdata->active, 1);
		hrtimer_start(&m7_subdata->timer,
			      ns_to_ktime(m7_subdata->period_ns),
			      HRTIMER_MODE_REL);
		dev_dbg(rtd->cpu_dai->dev,
			"%s. Starting timer: period_ns %d\n",
			__func__, m7_subdata->period_ns);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			ret = ipc_dma_playback_stop(m7_priv->mbox_chan,
						    &m7_subdata->xfer_info);
		} else {
			ret = ipc_dma_capture_stop(m7_priv->mbox_chan,
						   &m7_subdata->xfer_info);
		}

		atomic_set(&m7_subdata->active, 0);

		if (ret)
			return ret;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int m7_pcm_open(struct snd_pcm_substream *substream)
{
	struct m7audio_subdata *m7_subdata;

	m7_subdata = get_m7_subdata(substream);

	m7_subdata->substream = substream;
	atomic_set(&m7_subdata->active, 0);
	hrtimer_init(&m7_subdata->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		m7_subdata->timer.function = m7_hrtimer_play;
	else
		m7_subdata->timer.function = m7_hrtimer_capture;

	snd_soc_set_runtime_hwparams(substream, &omega_m7_hardware);

	return 0;
}

static int m7_pcm_close(struct snd_pcm_substream *substream)
{
	struct m7audio_subdata *m7_subdata;

	m7_subdata = get_m7_subdata(substream);

	hrtimer_cancel(&m7_subdata->timer);

	return 0;
}

static int m7_pcm_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params)
{
	struct m7audio_subdata *m7_subdata;

	m7_subdata = get_m7_subdata(substream);

	m7_subdata->period_ns = 1000000000 / params_rate(params) *
				params_period_size(params);

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	return 0;
}

static int m7_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int m7_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct m7audio_subdata *m7_subdata;
	unsigned int bufsize, periodsize, start, bufsize_u32;

	m7_subdata = get_m7_subdata(substream);

	bufsize = snd_pcm_lib_buffer_bytes(substream);
	bufsize_u32 = bufsize / 4;

	periodsize = snd_pcm_lib_period_bytes(substream);
	start = substream->runtime->dma_addr;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Set the pointers to indicate full */
		m7_subdata->xfer_info.xfer_ctrl->read_index = 0;
		m7_subdata->xfer_info.xfer_ctrl->write_index = bufsize_u32 - 1;
		m7_subdata->xfer_info.xfer_ctrl->capacity = bufsize_u32;
	} else {
		/* Set the pointers to indicate empty */
		m7_subdata->xfer_info.xfer_ctrl->read_index = 0;
		m7_subdata->xfer_info.xfer_ctrl->write_index = 0;
		m7_subdata->xfer_info.xfer_ctrl->capacity = bufsize_u32;
	}

	dev_dbg(rtd->cpu_dai->dev,
		"%s (buf_size %u) (period_size %u) (dma_addr 0x%x)\n",
		__func__, bufsize, periodsize, start);

	return 0;
}

static snd_pcm_uframes_t m7_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct m7audio_subdata *m7_subdata;
	unsigned int idx;

	m7_subdata = get_m7_subdata(substream);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		idx = m7_subdata->xfer_info.xfer_ctrl->read_index;
		dev_dbg(rtd->cpu_dai->dev, "playback read idx %u\n", idx);
	} else {
		idx = m7_subdata->xfer_info.xfer_ctrl->write_index;
		dev_dbg(rtd->cpu_dai->dev, "capture write idx %u\n", idx);
	}

	return bytes_to_frames(substream->runtime, (idx * 4));
}

static const struct snd_pcm_ops m7_pcm_ops = {
	.open		= m7_pcm_open,
	.close		= m7_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= m7_pcm_hw_params,
	.hw_free	= m7_pcm_hw_free,
	.prepare	= m7_pcm_prepare,
	.trigger	= m7_pcm_trigger,
	.pointer	= m7_pcm_pointer,
};

static void m7_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct m7audio_subdata *m7_subdata;
	struct snd_dma_buffer *buf;
	int i;

	for (i = 0; i < 2; i++) {
		substream = pcm->streams[i].substream;
		if (!substream)
			continue;

		m7_subdata = get_m7_subdata(substream);
		buf = &substream->dma_buffer;
		if (buf->area) {
			dma_free_coherent(pcm->card->dev,
					m7_subdata->xfer_info.alloc_size,
					m7_subdata->xfer_info.sample_buf,
					m7_subdata->xfer_info.sample_buf_phys);
			buf->area = NULL;
		}
	}
}

static int m7_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream,
					struct m7audio_subdata *m7_subdata)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size;
	struct device *dev;
	struct m7_audio_shmem *m7_shmem;
	dma_addr_t m7_shmem_phys;
	struct m7_dma_xfer_info *xfer_info;

	dev = pcm->card->dev;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = dev;
	buf->private_data = NULL;

	xfer_info = &m7_subdata->xfer_info;

	size = sizeof(struct m7_audio_shmem);
	xfer_info->alloc_size = size;
	m7_shmem = dma_alloc_coherent(dev, size, &m7_shmem_phys, GFP_KERNEL);
	if (!m7_shmem) {
		dev_err(dev, "dma_alloc_coherent failed\n");
		return -ENOMEM;
	}

	/* memory for audio samples */
	xfer_info->sample_buf_phys = m7_shmem_phys;
	xfer_info->sample_buf = (unsigned char *)&m7_shmem->sample_buf_mem[0];

	/* memory for buffer control */
	xfer_info->xfer_ctrl_phys = m7_shmem_phys +
				offsetof(struct m7_audio_shmem, xfer_ctrl_mem);
	xfer_info->xfer_ctrl = (struct m7_xfer_info *)&m7_shmem->xfer_ctrl_mem;

	/* memory for ipc message */
	xfer_info->ipc_msg_phys = m7_shmem_phys +
				offsetof(struct m7_audio_shmem, ipc_msg_mem);
	xfer_info->ipc_msg = (struct ipc_audio_msg *)&m7_shmem->ipc_msg_mem;

	buf->area = (unsigned char *)xfer_info->sample_buf;
	buf->addr = xfer_info->sample_buf_phys;
	buf->bytes = omega_m7_hardware.buffer_bytes_max;

	dev_dbg(dev, "%s: size 0x%zx @ %pK  0x%llx\n",
		__func__, size, m7_shmem, m7_shmem_phys);
	return 0;
}

static int m7_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	struct m7audio_rtd *m7_priv;
	int ret;

	m7_priv = (struct m7audio_rtd *)dev_get_drvdata(rtd->cpu_dai->dev);

	ret = dma_set_mask_and_coherent(card->dev, DMA_BIT_MASK(32));

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = m7_pcm_preallocate_dma_buffer(pcm,
						    SNDRV_PCM_STREAM_PLAYBACK,
						    &m7_priv->play);
		if (ret)
			return ret;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		ret = m7_pcm_preallocate_dma_buffer(pcm,
						    SNDRV_PCM_STREAM_CAPTURE,
						    &m7_priv->cap);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct snd_soc_component_driver m7_soc_platform = {
	.ops		= &m7_pcm_ops,
	.pcm_new	= m7_pcm_new,
	.pcm_free	= m7_pcm_free,
};

static int omega_m7audio_dma_create(struct platform_device *pdev)
{
	struct m7audio_rtd *m7_priv;
	int ret;

	m7_priv = devm_kzalloc(&pdev->dev,
			       sizeof(*m7_priv),
			       GFP_KERNEL);
	if (!m7_priv)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, m7_priv);

	/* Request mailbox channel. */
	m7_priv->mbox_client.dev          = &pdev->dev;
	m7_priv->mbox_client.tx_block     = false;
	m7_priv->mbox_client.tx_tout      = 1;
	m7_priv->mbox_client.knows_txdone = true;

	m7_priv->mbox_chan = mbox_request_channel(&m7_priv->mbox_client, 0);
	if (IS_ERR(m7_priv->mbox_chan)) {
		dev_err(&pdev->dev, "unable to get mbox channel\n");
		ret = PTR_ERR(m7_priv->mbox_chan);
		return ret;
	}

	ret = devm_snd_soc_register_component(&pdev->dev, &m7_soc_platform,
					      NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "%s failed\n", __func__);
		goto err_free_mbox;
	}

	return 0;

err_free_mbox:
	mbox_free_channel(m7_priv->mbox_chan);
	return ret;
}

static struct snd_soc_dai_driver omega_m7audio_port_dai = {
	.name = "omega-m7-audio-port",
	.playback = {
		.channels_min = 4,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.channels_min = 4,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE,
	},
};

static const struct snd_soc_component_driver omega_m7audio_port_component = {
	.name = "omega-m7-audio-port",
};

static int omega_m7audio_port_probe(struct platform_device *pdev)
{
	int ret;

	ret = omega_m7audio_dma_create(pdev);
	if (ret)
		return ret;

	ret = devm_snd_soc_register_component(&pdev->dev,
			&omega_m7audio_port_component,
			&omega_m7audio_port_dai, 1);
	return ret;
}

static int omega_m7audio_port_remove(struct platform_device *pdev)
{
	struct m7audio_rtd *m7_priv;

	m7_priv = (struct m7audio_rtd *)dev_get_drvdata(&pdev->dev);
	mbox_free_channel(m7_priv->mbox_chan);
	return 0;
}

static const struct of_device_id omega_m7audio_port_of_match[] = {
	{ .compatible = "brcm,omega-m7audio" },
	{}
};
MODULE_DEVICE_TABLE(of, omega_m7audio_port_of_match);

static struct platform_driver omega_m7audio_port_driver = {
	.probe = omega_m7audio_port_probe,
	.remove = omega_m7audio_port_remove,
	.driver = {
		.name = "omega-m7-audio-port",
		.of_match_table = omega_m7audio_port_of_match,
	},
};

module_platform_driver(omega_m7audio_port_driver);

MODULE_DESCRIPTION("Omega M7 Audio driver");
MODULE_AUTHOR("Lori Hikichi <lori.hikichi@broadcom.com>");
MODULE_LICENSE("GPL v2");

