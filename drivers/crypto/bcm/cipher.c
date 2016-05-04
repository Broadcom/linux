/*
 * Copyright 2016 Broadcom
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/crypto.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/rtnetlink.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/of_address.h>

#include <crypto/algapi.h>
#include <crypto/aead.h>
#include <crypto/internal/aead.h>
#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/sha.h>
#include <crypto/md5.h>
#include <crypto/authenc.h>
#include <crypto/skcipher.h>
#include <crypto/hash.h>
#include <crypto/aes.h>

#include "util.h"
#include "cipher.h"
#include "spu.h"
#include "spum.h"
#include "spu2.h"

/* ================= Device Structure ================== */

struct device_private iproc_priv;

/* ==================== Parameters ===================== */

int flow_debug_logging;
int packet_debug_logging;
int debug_logging_sleep;

module_param(flow_debug_logging, int, 0644);
MODULE_PARM_DESC(flow_debug_logging, "Enable Flow Debug Logging");

module_param(packet_debug_logging, int, 0644);
MODULE_PARM_DESC(packet_debug_logging, "Enable Packet Debug Logging");

module_param(debug_logging_sleep, int, 0644);
MODULE_PARM_DESC(debug_logging_sleep, "Packet Debug Logging Sleep");

int givencrypt_test_mode;

module_param(givencrypt_test_mode, int, 0644);
MODULE_PARM_DESC(givencrypt_test_mode, "Turn off givencrypt rnd IV generation");

#define MAX_SPUS 16

/* A type 3 BCM header, expected to precede the SPU header for SPU-M.
 * Bits 3 and 4 in the first byte encode the channel number (the dma ringset).
 * 0x60 - ring 0
 * 0x68 - ring 1
 * 0x70 - ring 2
 * 0x78 - ring 3
 */
char BCMHEADER[] = { 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28 };
/*
 * Some SPU hw does not use BCM header on SPU messages. So BCM_HDR_LEN
 * is set dynamically after reading SPU type from device tree.
 */
#define BCM_HDR_LEN  iproc_priv.bcm_hdr_len

/* ==================== Queue Tasks and Helpers ==================== */

static int handle_ablkcipher_req(struct iproc_reqctx_s *rctx);
static void handle_ablkcipher_resp(struct iproc_reqctx_s *rctx);
static int handle_ahash_req(struct iproc_reqctx_s *rctx);
static void handle_ahash_resp(struct iproc_reqctx_s *rctx);
static int ahash_req_done(struct iproc_reqctx_s *rctx);
static int handle_aead_req(struct iproc_reqctx_s *rctx);
static void handle_aead_resp(struct iproc_reqctx_s *rctx);

/* finish_req() is used to notify that the current request has been completed */
static void finish_req(struct iproc_reqctx_s *rctx, int err);

/*
 * Select a SPU channel to handle a crypto request. Selects channel in round
 * robin order.
 *
 * Returns:
 *   channel index if successful
 *   < 0 if an error occurs
 */
static u8 select_channel(void)
{
	u8 chan_idx = atomic_inc_return(&iproc_priv.next_chan);

	return (chan_idx % iproc_priv.spu.num_chan);
}

/* Build up the scatterlist of buffers used to receive a SPU response message
 * for an ablkcipher request. Includes buffers to catch SPU message headers
 * and the response data.
 * Inputs:
 *   mssg - mailbox message containing the receive sg
 *   rctx - crypto request context
 *   rx_frag_num - number of scatterlist elements required to hold the
 *                 SPU response message
 *   chunksize - Number of bytes of response data expected
 *   stat_pad_len - Number of bytes required to pad the STAT field to
 *		    a 4-byte boundary
 * Returns:
 *   0 if successful
 *   < 0 if an error
 */
static int
spu_ablkcipher_rx_sg_create(struct brcm_message *mssg,
			    struct iproc_reqctx_s *rctx,
			    u8 rx_frag_num,
			    unsigned chunksize, u32 stat_pad_len)
{
	struct device *dev = &iproc_priv.pdev->dev;
	struct spu_hw *spu = &iproc_priv.spu;
	struct scatterlist *sg;	/* used to build sgs in mbox message */
	struct iproc_ctx_s *ctx = rctx->ctx;
	u32 datalen;		/* Number of bytes of response data expected */

	mssg->spu.dst = kcalloc(rx_frag_num, sizeof(struct scatterlist),
				GFP_KERNEL);
	if (mssg->spu.dst == NULL)
		return -ENOMEM;

	sg = mssg->spu.dst;
	sg_init_table(sg, rx_frag_num);
	/* Space for SPU message header */
	sg_set_buf(sg++, rctx->msg_buf->spu_resp_hdr, ctx->spu_resp_hdr_len);

	/* Copy in each dst sg entry from request, up to chunksize */
	datalen = spu_msg_sg_add(&sg, &rctx->dst_sg, &rctx->dst_skip,
				 rctx->dst_nents, chunksize);
	if (datalen < chunksize) {
		dev_err(dev,
			"%s(): failed to copy dst sg to mbox msg. chunksize %u, datalen %u",
			__func__, chunksize, datalen);
		return -EFAULT;
	}

	if (ctx->cipher.alg == CIPHER_ALG_RC4)
		/* Add buffer to catch 260-byte SUPDT field for RC4 */
		sg_set_buf(sg++, rctx->msg_buf->c.supdt, SPU_SUPDT_LEN);

	if (stat_pad_len)
		sg_set_buf(sg++, rctx->msg_buf->rx_stat_pad, stat_pad_len);

	sg_set_buf(sg, rctx->msg_buf->rx_stat, spu->spu_rx_status_len());

	return 0;
}

/* Build up the scatterlist of buffers used to send a SPU request message
 * for an ablkcipher request. Includes SPU message headers and the request
 * data.
 *
 * Inputs:
 *   mssg - mailbox message containing the transmit sg
 *   rctx - crypto request context
 *   tx_frag_num - number of scatterlist elements required to construct the
 *                 SPU request message
 *   chunksize - Number of bytes of request data
 *   pad_len - Number of pad bytes
 * Returns:
 *   0 if successful
 *   < 0 if an error
 */
static int
spu_ablkcipher_tx_sg_create(struct brcm_message *mssg,
			    struct iproc_reqctx_s *rctx,
			    u8 tx_frag_num, unsigned chunksize, u32 pad_len)
{
	struct device *dev = &iproc_priv.pdev->dev;
	struct spu_hw *spu = &iproc_priv.spu;
	struct scatterlist *sg;	/* used to build sgs in mbox message */
	struct iproc_ctx_s *ctx = rctx->ctx;
	u32 datalen;		/* Number of bytes of response data expected */
	u32 msg_len = 0;	/* length of SPU request in bytes  */

	mssg->spu.src = kcalloc(tx_frag_num, sizeof(struct scatterlist),
				GFP_KERNEL);
	if (unlikely(mssg->spu.src == NULL))
		return -ENOMEM;

	sg = mssg->spu.src;
	sg_init_table(sg, tx_frag_num);

	sg_set_buf(sg++, rctx->msg_buf->bcm_spu_req_hdr,
		   BCM_HDR_LEN + ctx->spu_req_hdr_len);
	msg_len += ctx->spu_req_hdr_len;

	/* Copy in each src sg entry from request, up to chunksize */
	datalen = spu_msg_sg_add(&sg, &rctx->src_sg, &rctx->src_skip,
				 rctx->src_nents, chunksize);
	if (unlikely(datalen < chunksize)) {
		dev_err(dev, "%s(): failed to copy src sg to mbox msg",
			__func__);
		return -EFAULT;
	}
	msg_len += datalen;

	if (pad_len) {
		sg_set_buf(sg++, rctx->msg_buf->spu_req_pad, pad_len);
		msg_len += pad_len;
	}

	if (spu->spu_tx_status_len()) {
		sg_set_buf(sg, rctx->msg_buf->tx_stat,
			   spu->spu_tx_status_len());
		msg_len += spu->spu_tx_status_len();
	}

	if (unlikely(msg_len >= spu->max_pkt_size)) {
		dev_err(dev,
			"SPU message for block cipher too big. Length %u. Max %u.",
			msg_len, spu->max_pkt_size);
		return -EFAULT;
	}
	return 0;
}

/*
 * Submit as much of a block cipher request as fits in a single SPU request
 * message, starting at the current position in the request data. This may
 * be called on the crypto API thread, or, when a request is so large it
 * must be broken into multiple SPU messages, on the thread used to invoke
 * the response callback. When requests are broken into multiple SPU
 * messages, we assume subsequent messages depend on previous results, and
 * thus always wait for previous results before submitting the next message.
 * Because requests are submitted in lock step like this, there is no need
 * to synchronize access to request data structures.
 */
static int handle_ablkcipher_req(struct iproc_reqctx_s *rctx)
{
	struct device *dev = &iproc_priv.pdev->dev;
	struct spu_hw *spu = &iproc_priv.spu;
	struct crypto_async_request *areq = rctx->parent;
	struct ablkcipher_request *req =
	    container_of(areq, struct ablkcipher_request, base);
	struct iproc_ctx_s *ctx = rctx->ctx;
	struct spu_cipher_parms cipher_parms;
	int err = 0;
	unsigned chunksize = 0;	/* Number of bytes of request to submit */
	int remaining = 0;	/* Bytes of request still to process */
	int chunk_start;	/* Beginning of data for current SPU msg */

	/* IV or ctr value to use in this SPU msg */
	u8 local_iv_ctr[MAX_IV_SIZE];
	u32 gcm_pad_len;
	u32 db_size;
	u32 stat_pad_len;	/* num bytes to align status field */
	u32 pad_len;		/* total length of all padding */
	bool update_key = false;
	struct brcm_message *mssg;	/* mailbox message */

	/* number of entries in src and dst sg in mailbox message. */
	u8 rx_frag_num = 2;	/* response header and STATUS */
	u8 tx_frag_num = 1;	/* request header */

	flow_log("%s\n", __func__);

	cipher_parms.alg = ctx->cipher.alg;
	cipher_parms.mode = ctx->cipher.mode;
	cipher_parms.type = ctx->cipher_type;
	cipher_parms.key_len = ctx->enckeylen;
	cipher_parms.key_buf = ctx->enckey;
	cipher_parms.iv_buf = local_iv_ctr;
	cipher_parms.iv_len = rctx->iv_ctr_len;

	mssg = &rctx->mb_mssg;
	chunk_start = rctx->src_sent;
	remaining = rctx->total_todo - chunk_start;

	/* determine the chunk we are breaking off and update the indexes */
	chunksize = (remaining > ctx->max_payload) ? ctx->max_payload :
	    remaining;
	rctx->src_sent += chunksize;
	rctx->total_sent = rctx->src_sent;

	/* Count number of sg entries to be included in this request */
	rctx->src_nents = spu_sg_count(rctx->src_sg, rctx->src_skip, chunksize);
	rctx->dst_nents = spu_sg_count(rctx->dst_sg, rctx->dst_skip, chunksize);

	if ((ctx->cipher.mode == CIPHER_MODE_CBC) &&
	    rctx->is_encrypt && chunk_start) {
		/* Encrypting non-first first chunk. Copy last block of
		 * previous result to IV for this chunk.
		 */
		sg_copy_part_to_buf(req->dst, rctx->iv_ctr, rctx->iv_ctr_len,
				    chunk_start - rctx->iv_ctr_len);
	}

	if (rctx->iv_ctr)
		/* get our local copy of the iv */
		__builtin_memcpy(local_iv_ctr, rctx->iv_ctr, rctx->iv_ctr_len);

	/* generate the next IV if possible */
	if ((ctx->cipher.mode == CIPHER_MODE_CBC) && !rctx->is_encrypt) {
		/* CBC Decrypt: next IV is the last ciphertext block in
		 * this chunk
		 */
		sg_copy_part_to_buf(req->src, rctx->iv_ctr,
				    rctx->iv_ctr_len,
				    rctx->src_sent - rctx->iv_ctr_len);
	} else if (ctx->cipher.mode == CIPHER_MODE_CTR) {
		/* CTR mode, increment counter for next block. Assumes 16-byte
		 * block (AES).  SPU does not support CTR mode for DES/3DES.
		 */
		add_to_ctr(rctx->iv_ctr, chunksize);
	}

	if (ctx->cipher.alg == CIPHER_ALG_RC4) {
		rx_frag_num++;
		if (chunk_start) {
			/* for non-first RC4 chunks, use SUPDT from previous
			 * response as key for this chunk.
			 */
			cipher_parms.key_buf = rctx->msg_buf->c.supdt;
			update_key = true;
			cipher_parms.type = CIPHER_TYPE_UPDT;
		} else if (!rctx->is_encrypt) {
			/* First RC4 chunk. For decrypt, key in pre-built msg
			 * header may have been changed if encrypt required
			 * multiple chunks. So revert the key to the
			 * ctx->enckey value.
			 */
			update_key = true;
			cipher_parms.type = CIPHER_TYPE_INIT;
		}
	}

	flow_log("%s()-send req:%p rctx:%p ctx:%p\n", __func__, req, rctx, ctx);
	flow_log("max_payload:%u sent:%u start:%u remains:%u size:%u\n",
		 ctx->max_payload, rctx->src_sent, chunk_start, remaining,
		 chunksize);

	/* Copy SPU header template created at setkey time */
	memcpy(rctx->msg_buf->bcm_spu_req_hdr, ctx->bcm_spu_req_hdr,
	       sizeof(rctx->msg_buf->bcm_spu_req_hdr));

	/*
	 * Pass SUPDT field as key. Key field in finish() call is only used
	 * when update_key has been set above for RC4. Will be ignored in
	 * all other cases.
	 */
	spu->spu_cipher_req_finish(rctx->msg_buf->bcm_spu_req_hdr + BCM_HDR_LEN,
				   ctx->spu_req_hdr_len, !(rctx->is_encrypt),
				   &cipher_parms, update_key, chunksize);

	atomic64_add(chunksize, &iproc_priv.bytes_out);

	gcm_pad_len = spu->spu_gcm_pad_len(ctx->cipher.mode, chunksize);
	db_size = spu_real_db_size(0, 0, 0, chunksize, 0, 0, 0);
	stat_pad_len = spu_status_padlen(db_size);
	if (stat_pad_len)
		rx_frag_num++;
	pad_len = gcm_pad_len + stat_pad_len;
	if (pad_len) {
		tx_frag_num++;
		spu->spu_request_pad(rctx->msg_buf->spu_req_pad, gcm_pad_len, 0,
				     ctx->auth.alg, rctx->total_sent,
				     stat_pad_len);
	}

	spu->spu_dump_msg_hdr(rctx->msg_buf->bcm_spu_req_hdr + BCM_HDR_LEN,
			      ctx->spu_req_hdr_len);
	packet_log("payload:\n");
	dump_sg(rctx->src_sg, rctx->src_skip, chunksize);
	packet_dump("   pad: ", rctx->msg_buf->spu_req_pad, pad_len);

	/* Build mailbox message containing SPU request msg and rx buffers
	 * to catch response message
	 */
	memset(mssg, 0, sizeof(*mssg));
	mssg->type = BRCM_MESSAGE_SPU;
	mssg->ctx = rctx;	/* Will be returned in response */

	/* Create rx scatterlist to catch result */
	rx_frag_num += rctx->dst_nents;
	err = spu_ablkcipher_rx_sg_create(mssg, rctx, rx_frag_num, chunksize,
					  stat_pad_len);
	if (err)
		return err;

	/* Create tx scatterlist containing SPU request message */
	tx_frag_num += rctx->src_nents;
	if (spu->spu_tx_status_len())
		tx_frag_num++;
	err = spu_ablkcipher_tx_sg_create(mssg, rctx, tx_frag_num, chunksize,
					  pad_len);
	if (err)
		return err;

	err = mbox_send_message(iproc_priv.mbox[rctx->chan_idx], mssg);
	if (err < 0) {
		atomic_inc(&iproc_priv.mb_send_fail);
		dev_err(dev, "%s(): Failed to send mailbox message. err %d.",
			__func__, err);
		return err;
	}

	return -EINPROGRESS;
}

/*
 * Process a block cipher SPU response. Updates the total received count for
 * the request and updates global stats.
 */
static void handle_ablkcipher_resp(struct iproc_reqctx_s *rctx)
{
	struct spu_hw *spu = &iproc_priv.spu;
#ifdef DEBUG
	struct crypto_async_request *areq = rctx->parent;
	struct ablkcipher_request *req = ablkcipher_request_cast(areq);
#endif
	struct iproc_ctx_s *ctx = rctx->ctx;
	u32 payload_len;

	/* See how much data was returned */
	payload_len = spu->spu_payload_length(rctx->msg_buf->spu_resp_hdr);

	atomic64_add(payload_len, &iproc_priv.bytes_in);

	flow_log("%s() rctx:%p  offset: %u, bd_len: %u BD:\n",
		 __func__, rctx, rctx->total_received, payload_len);

	dump_sg(req->dst, rctx->total_received, payload_len);
	if (ctx->cipher.alg == CIPHER_ALG_RC4)
		packet_dump("  supdt ", rctx->msg_buf->c.supdt, SPU_SUPDT_LEN);

	rctx->total_received += payload_len;
	if (rctx->total_received == rctx->total_todo)
		atomic_inc(&iproc_priv.op_counts[SPU_OP_CIPHER]);
}

/* Build up the scatterlist of buffers used to receive a SPU response message
 * for an ahash request.
 * Inputs:
 *   mssg - mailbox message containing the receive sg
 *   rctx - crypto request context
 *   rx_frag_num - number of scatterlist elements required to hold the
 *                 SPU response message
 *   digestsize - length of hash digest, in bytes
 *   stat_pad_len - Number of bytes required to pad the STAT field to
 *		    a 4-byte boundary
 * Returns:
 *   0 if successful
 *   < 0 if an error
 */
static int
spu_ahash_rx_sg_create(struct brcm_message *mssg,
		       struct iproc_reqctx_s *rctx,
		       u8 rx_frag_num, unsigned int digestsize,
		       u32 stat_pad_len)
{
	struct spu_hw *spu = &iproc_priv.spu;
	struct scatterlist *sg;	/* used to build sgs in mbox message */
	struct iproc_ctx_s *ctx = rctx->ctx;

	mssg->spu.dst = kcalloc(rx_frag_num, sizeof(struct scatterlist),
				GFP_KERNEL);
	if (mssg->spu.dst == NULL)
		return -ENOMEM;

	sg = mssg->spu.dst;
	sg_init_table(sg, rx_frag_num);
	/* Space for SPU message header */
	sg_set_buf(sg++, rctx->msg_buf->spu_resp_hdr, ctx->spu_resp_hdr_len);

	/* Space for digest */
	sg_set_buf(sg++, rctx->msg_buf->digest, digestsize);

	if (stat_pad_len)
		sg_set_buf(sg++, rctx->msg_buf->rx_stat_pad, stat_pad_len);

	sg_set_buf(sg, rctx->msg_buf->rx_stat, spu->spu_rx_status_len());
	return 0;
}

/* Build up the scatterlist of buffers used to send a SPU request message
 * for an ahash request. Includes SPU message headers and the request
 * data.
 *
 * Inputs:
 *   mssg - mailbox message containing the transmit sg
 *   rctx - crypto request context
 *   tx_frag_num - number of scatterlist elements required to construct the
 *		   SPU request message
 *   spu_hdr_len - length in bytes of SPU message header
 *   hash_carry_len - Number of bytes of data carried over from previous req
 *   new_data_len - Number of bytes of new request data
 *   pad_len - Number of pad bytes
 * Returns:
 *   0 if successful
 *   < 0 if an error
 */
static int
spu_ahash_tx_sg_create(struct brcm_message *mssg,
		       struct iproc_reqctx_s *rctx,
		       u8 tx_frag_num,
		       u32 spu_hdr_len,
		       unsigned hash_carry_len,
		       unsigned new_data_len, u32 pad_len)
{
	struct device *dev = &iproc_priv.pdev->dev;
	struct spu_hw *spu = &iproc_priv.spu;
	struct scatterlist *sg;	/* used to build sgs in mbox message */
	u32 datalen;		/* Number of bytes of response data expected */
	u32 msg_len = 0;	/* length of SPU request in bytes  */

	mssg->spu.src = kcalloc(tx_frag_num, sizeof(struct scatterlist),
				GFP_KERNEL);
	if (mssg->spu.src == NULL)
		return -ENOMEM;

	sg = mssg->spu.src;
	sg_init_table(sg, tx_frag_num);

	sg_set_buf(sg++, rctx->msg_buf->bcm_spu_req_hdr,
		   BCM_HDR_LEN + spu_hdr_len);
	msg_len += spu_hdr_len;

	if (hash_carry_len) {
		sg_set_buf(sg++, rctx->hash_carry, hash_carry_len);
		msg_len += hash_carry_len;
	}

	if (new_data_len) {
		/* Copy in each src sg entry from request, up to chunksize */
		datalen = spu_msg_sg_add(&sg, &rctx->src_sg, &rctx->src_skip,
					 rctx->src_nents, new_data_len);
		if (datalen < new_data_len) {
			dev_err(dev,
				"%s(): failed to copy src sg to mbox msg",
				__func__);
			return -EFAULT;
		}
		msg_len += datalen;
	}

	if (pad_len) {
		sg_set_buf(sg++, rctx->msg_buf->spu_req_pad, pad_len);
		msg_len += pad_len;
	}

	if (spu->spu_tx_status_len()) {
		sg_set_buf(sg, rctx->msg_buf->tx_stat,
			   spu->spu_tx_status_len());
		msg_len += spu->spu_tx_status_len();
	}

	if (unlikely(msg_len >= spu->max_pkt_size)) {
		dev_err(dev,
			"SPU message for ahash too big. Length %u. Max %u.\n",
			msg_len, spu->max_pkt_size);
		return -EFAULT;
	}
	return 0;
}

/* Process an asynchronous hash request from the crypto API. Builds a SPU
 * request message embedded in a mailbox message and submits the mailbox
 * message on a selected mailbox channel. The SPU request message is
 * constructed as a scatterlist, including entries from the crypto API's
 * src scatterlist to avoid copying the data to be hashed. This function is
 * called either on the thread from the crypto API, or, in the case that the
 * crypto API request is too large to fit in a single SPU request message,
 * on the thread that invokes the receive callback with a response message.
 * Because some operations require the response from one chunk before the next
 * chunk can be submitted, we always wait for the response for the previous
 * chunk before submitting the next chunk. Because requests are submitted in
 * lock step like this, there is no need to synchronize access to request data
 * structures.
 * Returns:
 *   -EINPROGRESS: request has been submitted to SPU and response will be
 *		   returned asynchronously
 *   -EAGAIN:      non-final request included a small amount of data, which for
 *		   efficiency we did not submit to the SPU, but instead stored
 *		   to be submitted to the SPU with the next part of the request
 *   other:        an error code
 */
static int handle_ahash_req(struct iproc_reqctx_s *rctx)
{
	struct device *dev = &iproc_priv.pdev->dev;
	struct spu_hw *spu = &iproc_priv.spu;
	struct crypto_async_request *areq = rctx->parent;
	struct ahash_request *req = ahash_request_cast(areq);
	struct iproc_ctx_s *ctx = rctx->ctx;

	/* number of bytes still to be hashed in this req */
	unsigned nbytes_to_hash = 0;
	int err = 0;
	unsigned chunksize = 0;	/* length of hash carry + new data */
	unsigned chunk_start = 0;
	u32 db_size;	 /* Length of data field, incl gcm and hash padding */
	int pad_len = 0; /* total pad len, including gcm, hash, stat padding */
	u32 gcm_pad_len = 0;	/* length of GCM padding */
	u32 stat_pad_len = 0;	/* length of padding to align STATUS word */
	struct brcm_message *mssg;	/* mailbox message */
	struct spu_request_opts req_opts;
	struct spu_cipher_parms cipher_parms;
	struct spu_hash_parms hash_parms;
	struct spu_aead_parms aead_parms;
	unsigned local_nbuf;
	u32 spu_hdr_len;
	unsigned int digestsize;

	/* number of entries in src and dst sg. Always includes SPU msg header.
	 * rx always includes a buffer to catch digest and STATUS.
	 */
	u8 rx_frag_num = 3;
	u8 tx_frag_num = 1;

	flow_log("total_todo %u, total_sent %u\n",
		 rctx->total_todo, rctx->total_sent);

	memset(&req_opts, 0, sizeof(req_opts));
	memset(&cipher_parms, 0, sizeof(cipher_parms));
	memset(&hash_parms, 0, sizeof(hash_parms));
	memset(&aead_parms, 0, sizeof(aead_parms));

	req_opts.bd_suppress = true;
	hash_parms.alg = ctx->auth.alg;
	hash_parms.mode = ctx->auth.mode;
	hash_parms.type = HASH_TYPE_NONE;
	hash_parms.key_buf = (u8 *) ctx->authkey;
	hash_parms.key_len = ctx->authkeylen;

	mssg = &rctx->mb_mssg;
	chunk_start = rctx->src_sent;

	/* compute the amount remaining to hash. This may include data
	 * carried over from previous requests.
	 */
	nbytes_to_hash = rctx->total_todo - rctx->total_sent;
	chunksize = nbytes_to_hash;
	if (unlikely(chunksize > ctx->max_payload))
		chunksize = ctx->max_payload;

	/* If this is not a final request and the request data is less than
	 * the amount we want to submit in a SPU message for efficiency
	 * reasons, then simply park the data and prefix it to the data
	 * for the next request.
	 */
	if ((!rctx->is_final) && (chunksize < HASH_CARRY_MAX)) {
		sg_copy_part_to_buf(req->src,
				    rctx->hash_carry + rctx->hash_carry_len,
				    nbytes_to_hash - rctx->hash_carry_len,
				    rctx->src_sent);

		rctx->hash_carry_len = nbytes_to_hash;
		flow_log("  Exiting with stored remnant. hash_carry_len: %u\n",
			 rctx->hash_carry_len);
		packet_dump("  buf: ", rctx->hash_carry, rctx->hash_carry_len);
		return -EAGAIN;
	}

	/* if we have hash carry, then prefix it to the data in this request */
	local_nbuf = rctx->hash_carry_len;
	rctx->hash_carry_len = 0;
	if (local_nbuf)
		tx_frag_num++;

	/* Count number of sg entries to be used in this request */
	rctx->src_nents = spu_sg_count(rctx->src_sg, rctx->src_skip,
				       chunksize - local_nbuf);


	hash_parms.type = spu->spu_hash_type(rctx->src_sent);
	digestsize = spu->spu_digest_size(ctx->digestsize, ctx->auth.alg,
				     hash_parms.type);
	hash_parms.digestsize =	digestsize;

	/* update the indexes */
	rctx->total_sent += chunksize;
	/* if you sent a prebuf then that wasn't from this req->src */
	rctx->src_sent += chunksize - local_nbuf;

	if ((rctx->total_sent == rctx->total_todo) && rctx->is_final)
		hash_parms.pad_len = spu->spu_hash_pad_len(chunksize,
							   HASH_BLOCK_SIZE);

	/* If a non-first chunk, then include the digest returned from the
	 * previous chunk so that hw can add to it.
	 */
	if (hash_parms.type == HASH_TYPE_UPDT) {
		hash_parms.key_buf = rctx->msg_buf->digest;
		hash_parms.key_len = digestsize;
	}

	atomic64_add(chunksize, &iproc_priv.bytes_out);

	flow_log("%s() final: %u max_payload: %u nbuf: %u ",
		 __func__, rctx->is_final, ctx->max_payload, local_nbuf);
	flow_log("chunk_start: %u chunk_size: %u\n", chunk_start, chunksize);

	/* Prepend SPU header with type 3 BCM header */
	memcpy(rctx->msg_buf->bcm_spu_req_hdr, BCMHEADER, BCM_HDR_LEN);

	hash_parms.prebuf_len = local_nbuf;
	spu_hdr_len = spu->spu_create_request(rctx->msg_buf->bcm_spu_req_hdr +
					      BCM_HDR_LEN,
					      &req_opts, &cipher_parms,
					      &hash_parms, &aead_parms,
					      chunksize - local_nbuf);

	if (spu_hdr_len == 0) {
		pr_err("Failed to create SPU request header\n");
		return -EFAULT;
	}

	/* Determine total length of padding required. Put all padding in one
	 * buffer.
	 */
	gcm_pad_len = spu->spu_gcm_pad_len(ctx->cipher.mode, chunksize);
	db_size = spu_real_db_size(0, 0, local_nbuf, chunksize - local_nbuf,
				   0, 0, hash_parms.pad_len);
	if (spu->spu_tx_status_len())
		stat_pad_len = spu_status_padlen(db_size);
	if (stat_pad_len)
		rx_frag_num++;
	pad_len = hash_parms.pad_len + gcm_pad_len + stat_pad_len;
	if (pad_len) {
		tx_frag_num++;
		spu->spu_request_pad(rctx->msg_buf->spu_req_pad, gcm_pad_len,
				     hash_parms.pad_len, ctx->auth.alg,
				     rctx->total_sent, stat_pad_len);
	}

	spu->spu_dump_msg_hdr(rctx->msg_buf->bcm_spu_req_hdr + BCM_HDR_LEN,
			      spu_hdr_len);
	packet_dump("    prebuf: ", rctx->hash_carry, local_nbuf);
	flow_log("Data:\n");
	dump_sg(rctx->src_sg, rctx->src_skip, chunksize - local_nbuf);
	packet_dump("   pad: ", rctx->msg_buf->spu_req_pad, pad_len);

	/* Build mailbox message containing SPU request msg and rx buffers
	 * to catch response message
	 */
	memset(mssg, 0, sizeof(*mssg));
	mssg->type = BRCM_MESSAGE_SPU;
	mssg->ctx = rctx;	/* Will be returned in response */

	/* Create rx scatterlist to catch result */
	err = spu_ahash_rx_sg_create(mssg, rctx, rx_frag_num, digestsize,
				     stat_pad_len);
	if (err)
		return err;

	/* Create tx scatterlist containing SPU request message */
	tx_frag_num += rctx->src_nents;
	if (spu->spu_tx_status_len())
		tx_frag_num++;
	err = spu_ahash_tx_sg_create(mssg, rctx, tx_frag_num, spu_hdr_len,
				     local_nbuf, chunksize - local_nbuf,
				     pad_len);
	if (err)
		return err;

	err = mbox_send_message(iproc_priv.mbox[rctx->chan_idx], mssg);
	if (err < 0) {
		atomic_inc(&iproc_priv.mb_send_fail);
		dev_err(dev, "%s(): Failed to send mailbox message. err %d.",
			__func__, err);
		return err;
	}

	return -EINPROGRESS;
}

/* Process a SPU response message for a hash request. Unmaps the scatterlists
 * in the mailbox message used to submit the request. Checks if the entire
 * crypto API request has been processed, and if so, invokes post processing
 * on the result.
 * Returns:
 *   0 if successful
 *   < 0 otherwise
 */
static void handle_ahash_resp(struct iproc_reqctx_s *rctx)
{
	struct iproc_ctx_s *ctx = rctx->ctx;
#ifdef DEBUG
	struct crypto_async_request *areq = rctx->parent;
	struct ahash_request *req = ahash_request_cast(areq);
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	unsigned blocksize = crypto_tfm_alg_blocksize(crypto_ahash_tfm(ahash));
#endif

	flow_log("%s() req:%p blocksize:%u digestsize:%u\n",
		 __func__, req, blocksize, ctx->digestsize);

	atomic64_add(ctx->digestsize, &iproc_priv.bytes_in);

	if (rctx->total_sent == rctx->total_todo)
		ahash_req_done(rctx);
}

static int spu_hmac_outer_hash(struct ahash_request *req,
				struct iproc_ctx_s *ctx)
{
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	unsigned blocksize = crypto_tfm_alg_blocksize(crypto_ahash_tfm(ahash));

	switch (ctx->auth.alg) {
	case HASH_ALG_MD5:
		do_shash("md5", req->result, ctx->opad, blocksize,
			 req->result, ctx->digestsize);
		break;
	case HASH_ALG_SHA1:
		do_shash("sha1", req->result, ctx->opad, blocksize,
			 req->result, ctx->digestsize);
		break;
	case HASH_ALG_SHA224:
		do_shash("sha224", req->result, ctx->opad, blocksize,
			 req->result, ctx->digestsize);
		break;
	case HASH_ALG_SHA256:
		do_shash("sha256", req->result, ctx->opad, blocksize,
			 req->result, ctx->digestsize);
		break;
	default:
		pr_err("%s() Error : unknown hmac type\n", __func__);
		return -EINVAL;
	}
	return 0;
}

/* Do whatever processing is required after the entire hash request
 * has been processed through the SPU hardware.
 */
static int ahash_req_done(struct iproc_reqctx_s *rctx)
{
	struct spu_hw *spu = &iproc_priv.spu;
	struct crypto_async_request *areq = rctx->parent;
	struct ahash_request *req = ahash_request_cast(areq);
	struct iproc_ctx_s *ctx = rctx->ctx;
	int err;

	/* tcrypt ahash speed tests provide a stack-allocated result buffer
	 * that causes the DMA mapping to crash. So we have to copy the
	 * result here.
	 */
	memcpy(req->result, rctx->msg_buf->digest, ctx->digestsize);

	if (spu->spu_type == SPU_TYPE_SPUM) {
		/* byte swap the output from the UPDT function to network byte
		 * order
		 */
		if (ctx->auth.alg == HASH_ALG_MD5) {
			__swab32s((u32 *) req->result);
			__swab32s(((u32 *) req->result) + 1);
			__swab32s(((u32 *) req->result) + 2);
			__swab32s(((u32 *) req->result) + 3);
			__swab32s(((u32 *) req->result) + 4);
		}
	}

	flow_dump("  digest ", req->result, ctx->digestsize);

	/* if this an HMAC then do the outer hash */
	if (rctx->is_sw_hmac) {
		err = spu_hmac_outer_hash(req, ctx);
		if (err < 0)
			return err;
		flow_dump("  hmac: ", req->result, ctx->digestsize);
	}

	if (rctx->is_sw_hmac)
		atomic_inc(&iproc_priv.op_counts[SPU_OP_HMAC]);
	else
		atomic_inc(&iproc_priv.op_counts[SPU_OP_HASH]);

	return 0;
}

/* A helper function for AEAD requests */
static unsigned spu_dtls_hmac_offset(struct aead_request *req,
				     struct iproc_reqctx_s *rctx,
				     unsigned chunksize)
{
	struct iproc_ctx_s *ctx = rctx->ctx;
	unsigned block_size =
	    crypto_tfm_alg_blocksize(crypto_aead_tfm(crypto_aead_reqtfm(req)));

	unsigned hmac_offset;
	u16 swap_hmac_offset = 0;

	if (!rctx->is_encrypt) {
		char iv_buf[MAX_IV_SIZE];
		char src_buf[MAX_IV_SIZE];
		char dest_buf[MAX_IV_SIZE];
		unsigned i;
		char *alg_name = spu_alg_name(ctx->cipher.alg,
					      ctx->cipher.mode);

		switch (ctx->cipher.mode) {
		case CIPHER_MODE_CBC:
			sg_copy_part_to_buf(req->src, iv_buf, block_size,
					    chunksize - (block_size * 2));
			break;

		case CIPHER_MODE_ECB:
			break;

		case CIPHER_MODE_CTR:
			memcpy(iv_buf, req->iv, block_size);

			for (i = ((chunksize / block_size) - 1); i > 0; i--)
				crypto_inc(iv_buf, block_size);
			break;

		case CIPHER_MODE_GCM:
			memcpy(iv_buf, req->iv, block_size);

			for (i = (chunksize / block_size); i > 0; i--)
				crypto_inc(iv_buf, block_size);
			break;

		default:
			break;
		}

		sg_copy_part_to_buf(req->src, src_buf, block_size,
				    chunksize - (block_size));
		do_decrypt(alg_name, ctx->enckey, ctx->enckeylen,
			   iv_buf, src_buf, dest_buf, block_size);

		hmac_offset = chunksize - 1 - dest_buf[block_size - 1] -
		    ctx->digestsize;
	} else {
		char src_buf[8];

		sg_copy_part_to_buf(req->src, src_buf, 8, chunksize - 8);
		hmac_offset = chunksize - 1 - src_buf[7] - ctx->digestsize;
	}

	/* Update length field in the DTLS Authen header (assoc data) */
	swap_hmac_offset = ntohs(hmac_offset & 0xffff);
	sg_copy_part_from_buf(rctx->assoc, (u8 *) &swap_hmac_offset, 2,
			      req->assoclen - 2);

	return hmac_offset;
}

/* Build up the scatterlist of buffers used to receive a SPU response message
 * for an AEAD request. Includes buffers to catch SPU message headers
 * and the response data.
 * Inputs:
 *   mssg - mailbox message containing the receive sg
 *   rctx - crypto request context
 *   rx_frag_num - number of scatterlist elements required to hold the
 *		   SPU response message
 *   assoc_len - Length of associated data included in the crypto request
 *   resp_len - Number of bytes of response data expected to be written to
 *              dst buffer from crypto API
 *   digestsize - length of hash digest, in bytes
 *   stat_pad_len - Number of bytes required to pad the STAT field to
 *		    a 4-byte boundary
 * Returns:
 *   0 if successful
 *   < 0 if an error
 */
static int
spu_aead_rx_sg_create(struct brcm_message *mssg,
		      struct aead_request *req,
		      struct iproc_reqctx_s *rctx,
		      u8 rx_frag_num,
		      unsigned assoc_len, unsigned resp_len,
		      unsigned digestsize, u32 stat_pad_len)
{
	struct device *dev = &iproc_priv.pdev->dev;
	struct spu_hw *spu = &iproc_priv.spu;
	struct scatterlist *sg;	/* used to build sgs in mbox message */
	struct iproc_ctx_s *ctx = rctx->ctx;
	u32 datalen;		/* Number of bytes of response data expected */
	u32 assoc_buf_len;
	u8 gcm_padlen = 0;

	if (ctx->cipher.mode == CIPHER_MODE_GCM) {
		gcm_padlen = spu->spu_gcm_pad_len(ctx->cipher.mode, resp_len);
		if (gcm_padlen)
			/* have to catch gcm pad in separate buffer */
			rx_frag_num++;
	}

	mssg->spu.dst = kcalloc(rx_frag_num, sizeof(struct scatterlist),
				GFP_KERNEL);
	if (mssg->spu.dst == NULL)
		return -ENOMEM;

	sg = mssg->spu.dst;
	sg_init_table(sg, rx_frag_num);

	/* Space for SPU message header */
	sg_set_buf(sg++, rctx->msg_buf->spu_resp_hdr, ctx->spu_resp_hdr_len);

	assoc_buf_len = spu->spu_assoc_resp_len(ctx->cipher.mode,
						ctx->alg->dtls_hmac, assoc_len,
						rctx->iv_ctr_len);
	if (assoc_buf_len)
		/* Don't write directly to req->dst, because SPU may pad the
		 * assoc data in the response
		 */
		sg_set_buf(sg++, rctx->msg_buf->a.resp_aad, assoc_buf_len);

	/* Copy in each dst sg entry from request, up to chunksize.
	 * dst sg catches just the data. digest caught in separate buf.
	 */
	datalen = spu_msg_sg_add(&sg, &rctx->dst_sg, &rctx->dst_skip,
				 rctx->dst_nents, resp_len);
	if (datalen < (resp_len)) {
		dev_err(dev,
			"%s(): failed to copy dst sg to mbox msg. expected len %u, datalen %u",
			__func__, resp_len, datalen);
		return -EFAULT;
	}

	/* If GCM data is padded, catch padding in separate buffer */
	if (gcm_padlen)
		sg_set_buf(sg++, rctx->msg_buf->a.gcmpad, gcm_padlen);

	/* Always catch ICV in separate buffer */
	sg_set_buf(sg++, rctx->msg_buf->digest, digestsize);

	flow_log("stat_pad_len %u\n", stat_pad_len);
	if (stat_pad_len)
		sg_set_buf(sg++, rctx->msg_buf->rx_stat_pad, stat_pad_len);

	sg_set_buf(sg, rctx->msg_buf->rx_stat, spu->spu_rx_status_len());

	return 0;
}

/* Build up the scatterlist of buffers used to send a SPU request message
 * for an AEAD request. Includes SPU message headers and the request
 * data.
 *
 * Inputs:
 *   mssg - mailbox message containing the transmit sg
 *   rctx - crypto request context
 *   tx_frag_num - number of scatterlist elements required to construct the
 *		   SPU request message
 *   spu_hdr_len - length of SPU message header in bytes
 *   assoc - crypto API associated data scatterlist
 *   assoc_len - length of associated data
 *   assoc_nents - number of scatterlist entries containing assoc data
 *   aead_iv_len - length of AEAD IV, if included
 *   chunksize - Number of bytes of request data
 *   aad_pad_len - Number of bytes of padding at end of AAD. For GCM.
 *   pad_len - Number of pad bytes
 *   incl_icv - If true, write separate ICV buffer after data and
 *              any padding
 * Returns:
 *   0 if successful
 *   < 0 if an error
 */
static int
spu_aead_tx_sg_create(struct brcm_message *mssg,
		      struct iproc_reqctx_s *rctx,
		      u8 tx_frag_num,
		      u32 spu_hdr_len,
		      struct scatterlist *assoc,
		      unsigned assoc_len,
		      int assoc_nents,
		      unsigned aead_iv_len,
		      unsigned chunksize,
		      u32 aad_pad_len, u32 pad_len, bool incl_icv)
{
	struct device *dev = &iproc_priv.pdev->dev;
	struct spu_hw *spu = &iproc_priv.spu;
	struct scatterlist *sg;	/* used to build sgs in mbox message */
	struct scatterlist *assoc_sg = assoc;
	struct iproc_ctx_s *ctx = rctx->ctx;
	u32 datalen;		/* Number of bytes of data to write */
	u32 written;		/* Number of bytes of data written */
	u32 msg_len = 0;	/* length of SPU request in bytes  */
	u32 assoc_offset = 0;

	mssg->spu.src = kcalloc(tx_frag_num, sizeof(struct scatterlist),
				GFP_KERNEL);
	if (mssg->spu.src == NULL)
		return -ENOMEM;

	sg = mssg->spu.src;
	sg_init_table(sg, tx_frag_num);

	sg_set_buf(sg++, rctx->msg_buf->bcm_spu_req_hdr,
		   BCM_HDR_LEN + spu_hdr_len);
	msg_len += spu_hdr_len;

	if (assoc_len) {
		/* Copy in each associated data sg entry from request */
		written = spu_msg_sg_add(&sg, &assoc_sg, &assoc_offset,
					 assoc_nents, assoc_len);
		if (written < assoc_len) {
			dev_err(dev,
				"%s(): failed to copy assoc sg to mbox msg",
				__func__);
			return -EFAULT;
		}
	}

	if (aead_iv_len) {
		sg_set_buf(sg++, rctx->iv_ctr, aead_iv_len);
		msg_len += aead_iv_len;
	}

	if (aad_pad_len) {
		sg_set_buf(sg++, rctx->msg_buf->a.req_aad_pad, aad_pad_len);
		msg_len += pad_len;
	}

	datalen = chunksize;
	if ((chunksize > ctx->digestsize) && incl_icv)
		datalen -= ctx->digestsize;
	if (datalen) {
		/* For aead, a single msg should consume the entire src sg */
		written = spu_msg_sg_add(&sg, &rctx->src_sg, &rctx->src_skip,
					 rctx->src_nents, datalen);
		if (written < datalen) {
			dev_err(dev, "%s(): failed to copy src sg to mbox msg",
				__func__);
			return -EFAULT;
		}
		msg_len += written;
	}

	if (pad_len) {
		sg_set_buf(sg++, rctx->msg_buf->spu_req_pad, pad_len);
		msg_len += pad_len;
	}

	if (incl_icv) {
		sg_set_buf(sg++, rctx->msg_buf->digest, ctx->digestsize);
		msg_len += ctx->digestsize;
	}

	if (spu->spu_tx_status_len()) {
		sg_set_buf(sg, rctx->msg_buf->tx_stat,
			   spu->spu_tx_status_len());
		msg_len += spu->spu_tx_status_len();
	}

	if (unlikely(msg_len >= spu->max_pkt_size)) {
		pr_err("SPU message for AEAD request too big. Length %u. Max %u.\n",
		     msg_len, spu->max_pkt_size);
		return -EFAULT;
	}
	return 0;
}

/* Submit a SPU request message for the next chunk of the current AEAD request.
 * Unlike other operation types, we assume the length of the request fits in
 * a single SPU request message. aead_enqueue() makes sure this is true.
 * Comments for other op types regarding threads applies here as well.
 *
 * Unlike incremental hash ops, where the spu returns the entire hash for
 * truncated algs like sha-224, the spu returns just the truncated hash in
 * response to aead requests. So digestsize is always ctx->digestsize here.
 */
static int handle_aead_req(struct iproc_reqctx_s *rctx)
{
	struct device *dev = &iproc_priv.pdev->dev;
	struct spu_hw *spu = &iproc_priv.spu;
	struct crypto_async_request *areq = rctx->parent;
	struct aead_request *req = container_of(areq,
						struct aead_request, base);
	struct iproc_ctx_s *ctx = rctx->ctx;
	int err;
	unsigned chunksize;
	unsigned resp_len;
	u32 spu_hdr_len;
	u32 db_size;
	u32 stat_pad_len;
	u32 pad_len;
	struct brcm_message *mssg;	/* mailbox message */
	struct spu_request_opts req_opts;
	struct spu_cipher_parms cipher_parms;
	struct spu_hash_parms hash_parms;
	struct spu_aead_parms aead_parms;
	int assoc_nents = 0;
	bool incl_icv = false;
	unsigned int digestsize = ctx->digestsize;

	/* number of entries in src and dst sg. Always includes SPU msg header.
	 */
	u8 rx_frag_num = 2;	/* and STATUS */
	u8 tx_frag_num = 1;

	/* doing the whole thing at once */
	chunksize = rctx->total_todo;

	flow_log("%s: rctx %p, chunksize %u\n", __func__, rctx, chunksize);

	memset(&req_opts, 0, sizeof(req_opts));
	memset(&hash_parms, 0, sizeof(hash_parms));
	memset(&aead_parms, 0, sizeof(aead_parms));

	req_opts.is_inbound = !(rctx->is_encrypt);
	req_opts.auth_first = ctx->auth_first;
	req_opts.is_aead = true;
	req_opts.dtls_aead = ctx->alg->dtls_hmac;

	cipher_parms.alg = ctx->cipher.alg;
	cipher_parms.mode = ctx->cipher.mode;
	cipher_parms.type = ctx->cipher_type;
	cipher_parms.key_buf = ctx->enckey;
	cipher_parms.key_len = ctx->enckeylen;
	cipher_parms.iv_buf = rctx->iv_ctr;
	cipher_parms.iv_len = rctx->iv_ctr_len;

	hash_parms.alg = ctx->auth.alg;
	hash_parms.mode = ctx->auth.mode;
	hash_parms.type = HASH_TYPE_NONE;
	hash_parms.key_buf = (u8 *) ctx->authkey;
	hash_parms.key_len = ctx->authkeylen;
	hash_parms.digestsize = digestsize;

	if ((ctx->auth.alg == HASH_ALG_SHA224) &&
	    (ctx->authkeylen < SHA224_DIGEST_SIZE))
		hash_parms.key_len = SHA224_DIGEST_SIZE;

	aead_parms.assoc_size = req->assoclen;

	/* Count number of sg entries from the crypto API request that are to
	 * be included in this mailbox message. For dst sg, don't count space
	 * for digest. Digest gets caught in a separate buffer and copied back
	 * to dst sg when processing response.
	 */
	rctx->src_nents = spu_sg_count(rctx->src_sg, rctx->src_skip, chunksize);
	rctx->dst_nents = spu_sg_count(rctx->dst_sg, rctx->dst_skip, chunksize);
	if (req->assoclen)
		assoc_nents = spu_sg_count(rctx->assoc, 0, req->assoclen);

	mssg = &rctx->mb_mssg;

	if (ctx->alg->dtls_hmac)
		hash_parms.hmac_offset = spu_dtls_hmac_offset(req, rctx,
							      chunksize);

	rctx->total_sent = chunksize;
	rctx->src_sent = chunksize;
	if (spu->spu_assoc_resp_len(ctx->cipher.mode, ctx->alg->dtls_hmac,
				    req->assoclen, rctx->iv_ctr_len))
		rx_frag_num++;

	aead_parms.iv_len = spu->spu_aead_ivlen(ctx->cipher.mode,
						ctx->alg->dtls_hmac,
						rctx->iv_ctr_len);

	if (ctx->auth.alg == HASH_ALG_AES)
		hash_parms.type = ctx->cipher_type;

	aead_parms.aad_pad_len = spu->spu_gcm_pad_len(ctx->cipher.mode,
						      req->assoclen);
	flow_log("gcm AAD padding: %u bytes\n", aead_parms.aad_pad_len);
	aead_parms.gcm_pad_len = spu->spu_gcm_pad_len(ctx->cipher.mode,
						      chunksize);
	flow_log("gcm pad: %u bytes\n", aead_parms.gcm_pad_len);

	if (spu_req_incl_icv(ctx->cipher.mode, rctx->is_encrypt)) {
		incl_icv = true;
		tx_frag_num++;
		/* Copy ICV from end of src scatterlist to digest buf */
		sg_copy_part_to_buf(req->src, rctx->msg_buf->digest, digestsize,
				    req->assoclen + rctx->total_sent -
				    digestsize);
	}

	atomic64_add(chunksize, &iproc_priv.bytes_out);

	flow_log("%s()-sent req:%p chunksize:%u hmac_offset:%u\n",
		 __func__, req, chunksize, hash_parms.hmac_offset);

	/* Prepend SPU header with type 3 BCM header */
	memcpy(rctx->msg_buf->bcm_spu_req_hdr, BCMHEADER, BCM_HDR_LEN);

	spu_hdr_len = spu->spu_create_request(rctx->msg_buf->bcm_spu_req_hdr +
					      BCM_HDR_LEN, &req_opts,
					      &cipher_parms, &hash_parms,
					      &aead_parms, chunksize);

	/* Determine total length of padding. Put all padding in one buffer. */
	db_size = spu_real_db_size(req->assoclen, aead_parms.iv_len, 0,
				   chunksize, aead_parms.aad_pad_len,
				   aead_parms.gcm_pad_len, 0);
	stat_pad_len = spu_status_padlen(db_size + aead_parms.gcm_pad_len);
	if (stat_pad_len)
		rx_frag_num++;
	pad_len = aead_parms.gcm_pad_len + stat_pad_len;
	if (pad_len) {
		tx_frag_num++;
		spu->spu_request_pad(rctx->msg_buf->spu_req_pad,
				     aead_parms.gcm_pad_len, 0,
				     ctx->auth.alg, rctx->total_sent,
				     stat_pad_len);
	}

	spu->spu_dump_msg_hdr(rctx->msg_buf->bcm_spu_req_hdr + BCM_HDR_LEN,
			      spu_hdr_len);
	dump_sg(rctx->assoc, 0, req->assoclen);
	packet_dump("    aead iv: ", rctx->iv_ctr, aead_parms.iv_len);
	packet_log("BD:\n");
	dump_sg(rctx->src_sg, rctx->src_skip, chunksize);
	packet_dump("   pad: ", rctx->msg_buf->spu_req_pad, pad_len);

	/* Build mailbox message containing SPU request msg and rx buffers
	 * to catch response message
	 */
	memset(mssg, 0, sizeof(*mssg));
	mssg->type = BRCM_MESSAGE_SPU;
	mssg->ctx = rctx;	/* Will be returned in response */

	/* Create rx scatterlist to catch result */
	rx_frag_num += rctx->dst_nents;
	resp_len = chunksize;

	/* Always catch ICV in separate buffer. Have to for GCM because of GCM
	 * padding. Have to for SHA-224 and other truncated SHAs because SPU
	 * sends entire digest back.
	 */
	rx_frag_num++;

	if ((ctx->cipher.mode == CIPHER_MODE_GCM) && !rctx->is_encrypt)
		/* Input is ciphertxt plus ICV, but ICV not incl
		 * in output.
		 */
		resp_len -= ctx->digestsize;

	err = spu_aead_rx_sg_create(mssg, req, rctx, rx_frag_num, req->assoclen,
				    resp_len, digestsize, stat_pad_len);
	if (err)
		return err;

	/* Create tx scatterlist containing SPU request message */
	tx_frag_num += rctx->src_nents;
	tx_frag_num += assoc_nents;
	if (aead_parms.aad_pad_len)
		tx_frag_num++;
	if (aead_parms.iv_len)
		tx_frag_num++;
	if (spu->spu_tx_status_len())
		tx_frag_num++;
	err = spu_aead_tx_sg_create(mssg, rctx, tx_frag_num, spu_hdr_len,
				    rctx->assoc, req->assoclen, assoc_nents,
				    aead_parms.iv_len, chunksize,
				    aead_parms.aad_pad_len, pad_len, incl_icv);
	if (err)
		return err;

	err = mbox_send_message(iproc_priv.mbox[rctx->chan_idx], mssg);
	if (err < 0) {
		atomic_inc(&iproc_priv.mb_send_fail);
		dev_err(dev, "%s(): Failed to send mailbox message. err %d.",
			__func__, err);
		return err;
	}

	return -EINPROGRESS;
}

/* Process a SPU response message for an AEAD request. Unmaps the scatterlists
 * in the mailbox message and updates stats.
 */
static void handle_aead_resp(struct iproc_reqctx_s *rctx)
{
	struct spu_hw *spu = &iproc_priv.spu;
	struct crypto_async_request *areq = rctx->parent;
	struct aead_request *req = container_of(areq,
						struct aead_request, base);
	struct iproc_ctx_s *ctx = rctx->ctx;
	u32 payload_len;
	unsigned icv_offset;
	u32 result_len;

	/* See how much data was returned */
	payload_len = spu->spu_payload_length(rctx->msg_buf->spu_resp_hdr);
	flow_log("payload_len %u\n", payload_len);

	/* only count payload */
	atomic64_add(payload_len, &iproc_priv.bytes_in);

	if (rctx->msg_buf->a.resp_aad && req->assoclen)
		packet_dump("  assoc_data ", rctx->msg_buf->a.resp_aad,
			    req->assoclen);

	/* Copy the ICV back to the destination
	 * buffer. In decrypt case, SPU gives us back the digest, but crypto
	 * API doesn't expect ICV in dst buffer.
	 */
	result_len = req->cryptlen;
	if (rctx->is_encrypt) {
		icv_offset = req->assoclen + rctx->total_sent;
		packet_dump("  ICV: ", rctx->msg_buf->digest, ctx->digestsize);
		flow_log("copying ICV to dst sg at offset %u\n", icv_offset);
		sg_copy_part_from_buf(req->dst, rctx->msg_buf->digest,
				      ctx->digestsize, icv_offset);
		result_len += ctx->digestsize;
	}

	packet_log("response data:  ");
	dump_sg(req->dst, req->assoclen, result_len);

	if (rctx->is_encrypt) {
		packet_log("digest:  ");
		dump_sg(req->dst, req->assoclen + result_len, ctx->digestsize);
	}

	atomic_inc(&iproc_priv.op_counts[SPU_OP_AEAD]);
}

/* finish_req() is used to notify that the current request has been completed */
static void finish_req(struct iproc_reqctx_s *rctx, int err)
{
	struct crypto_async_request *areq = rctx->parent;
	/* mailbox message used to tx request */
	struct brcm_message *mssg = &rctx->mb_mssg;

	flow_log("%s() rctx:%p err:%d\n\n", __func__, rctx, err);

	kfree(mssg->spu.src);
	kfree(mssg->spu.dst);
	memset(mssg, 0, sizeof(struct brcm_message));

	kfree(rctx->msg_buf);
	rctx->msg_buf = NULL;
	kfree(rctx->iv_ctr);
	rctx->iv_ctr = NULL;

	if (areq)
		areq->complete(areq, err);
}

/* Callback from mailbox framework presenting a SPU response */
static void spu_rx_callback(struct mbox_client *cl, void *msg)
{
	struct spu_hw *spu = &iproc_priv.spu;
	struct device *dev = &iproc_priv.pdev->dev;
	struct brcm_message *mssg = msg;
	struct iproc_reqctx_s *rctx;
	struct iproc_ctx_s *ctx;
	struct crypto_async_request *areq;
	int err = 0;

	rctx = mssg->ctx;
	flow_log("%s() rctx: %p\n", __func__, rctx);
	if (unlikely(rctx == NULL)) {
		/* This is fatal */
		dev_err(dev, "%s(): no request context", __func__);
		err = -EFAULT;
		goto cb_finish;
	}
	areq = rctx->parent;
	ctx = rctx->ctx;

	/* process the SPU status */
	err = spu->spu_status_process(rctx->msg_buf->rx_stat);
	if (err != 0) {
		if (err == SPU_INVALID_ICV)
			atomic_inc(&iproc_priv.bad_icv);
		err = -EBADMSG;
		goto cb_finish;
	}

	/* Process the SPU response message */
	switch (rctx->ctx->alg->type) {
	case CRYPTO_ALG_TYPE_ABLKCIPHER:
		handle_ablkcipher_resp(rctx);
		break;
	case CRYPTO_ALG_TYPE_AHASH:
		handle_ahash_resp(rctx);
		break;
	case CRYPTO_ALG_TYPE_AEAD:
		handle_aead_resp(rctx);
		break;
	default:
		err = -EINVAL;
		goto cb_finish;
	}

	/* If this response does not complete the request, then send the next
	 * request chunk.
	 */
	if (rctx->total_sent < rctx->total_todo) {
		switch (rctx->ctx->alg->type) {
		case CRYPTO_ALG_TYPE_ABLKCIPHER:
			err = handle_ablkcipher_req(rctx);
			break;
		case CRYPTO_ALG_TYPE_AHASH:
			err = handle_ahash_req(rctx);
			break;
		case CRYPTO_ALG_TYPE_AEAD:
			err = handle_aead_req(rctx);
			break;
		default:
			err = -EINVAL;
		}

		if (err == -EINPROGRESS)
			/* Successfully submitted request for next chunk */
			return;
	}

cb_finish:
	finish_req(rctx, err);
}

/* ==================== Kernel Cryptographic API ==================== */

/* ablkcipher helpers */

static int ablkcipher_enqueue(struct ablkcipher_request *req, bool encrypt)
{
	struct iproc_reqctx_s *rctx = ablkcipher_request_ctx(req);
	struct iproc_ctx_s *ctx =
	    crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	int err;

	flow_log("%s() req:%p enc:%u rctx:%p ctx:%p\n", __func__, req,
		 encrypt, rctx, ctx);

	rctx->parent = &req->base;
	rctx->is_encrypt = encrypt;
	rctx->bd_suppress = false;
	rctx->total_todo = req->nbytes;
	rctx->src_sent = rctx->total_sent = 0;
	rctx->total_received = 0;
	rctx->ctx = ctx;
	memset(&rctx->mb_mssg, 0, sizeof(struct brcm_message));

	/* Initialize current position in src and dst scatterlists */
	rctx->src_sg = req->src;
	rctx->src_nents = 0;
	rctx->src_skip = 0;
	rctx->dst_sg = req->dst;
	rctx->dst_nents = 0;
	rctx->dst_skip = 0;

	/* Allocate a set of buffers to be used as SPU message fragments */
	rctx->msg_buf = kzalloc(sizeof(struct spu_msg_buf), GFP_KERNEL);
	if (rctx->msg_buf == NULL)
		return -ENOMEM;

	if (ctx->cipher.mode == CIPHER_MODE_CBC ||
	    ctx->cipher.mode == CIPHER_MODE_CTR ||
	    ctx->cipher.mode == CIPHER_MODE_OFB ||
	    ctx->cipher.mode == CIPHER_MODE_GCM) {
		rctx->iv_ctr_len =
		    crypto_ablkcipher_ivsize(crypto_ablkcipher_reqtfm(req));
		rctx->iv_ctr = kmalloc(rctx->iv_ctr_len, GFP_KERNEL);
		if (rctx->iv_ctr == NULL)
			return -ENOMEM;
		memcpy(rctx->iv_ctr, req->info, rctx->iv_ctr_len);
	} else {
		rctx->iv_ctr = NULL;
		rctx->iv_ctr_len = 0;
	}

	/* Choose a SPU to process this request */
	rctx->chan_idx = select_channel();
	err = handle_ablkcipher_req(rctx);
	if (err != -EINPROGRESS)
		finish_req(rctx, err);

	return -EINPROGRESS;
}

static int des_setkey(struct crypto_ablkcipher *cipher, const u8 *key,
		      unsigned keylen)
{
	struct iproc_ctx_s *ctx = crypto_ablkcipher_ctx(cipher);
	u32 tmp[DES_EXPKEY_WORDS];

	if (keylen == DES_KEY_SIZE) {
		if (des_ekey(tmp, key) == 0) {
			if (crypto_ablkcipher_get_flags(cipher) &
			    CRYPTO_TFM_REQ_WEAK_KEY) {
				crypto_ablkcipher_set_flags(cipher,
						CRYPTO_TFM_RES_WEAK_KEY);
				return -EINVAL;
			}
		}

		ctx->cipher_type = CIPHER_TYPE_DES;
	} else {
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}
	return 0;
}

static int threedes_setkey(struct crypto_ablkcipher *cipher, const u8 *key,
			   unsigned keylen)
{
	struct iproc_ctx_s *ctx = crypto_ablkcipher_ctx(cipher);

	if (keylen == (DES_KEY_SIZE * 3)) {
		const u32 *K = (const u32 *)key;

		if (!((K[0] ^ K[2]) | (K[1] ^ K[3])) ||
		    !((K[2] ^ K[4]) | (K[3] ^ K[5]))) {
			crypto_ablkcipher_set_flags(cipher,
						CRYPTO_TFM_RES_BAD_KEY_SCHED);
			return -EINVAL;
		}

		ctx->cipher_type = CIPHER_TYPE_3DES;
	} else {
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}
	return 0;
}

static int aes_setkey(struct crypto_ablkcipher *cipher, const u8 *key,
		      unsigned keylen)
{
	struct iproc_ctx_s *ctx = crypto_ablkcipher_ctx(cipher);

	switch (keylen) {
	case AES_KEYSIZE_128:
		ctx->cipher_type = CIPHER_TYPE_AES128;
		ctx->max_payload -= AES_KEYSIZE_128;
		break;
	case AES_KEYSIZE_192:
		ctx->cipher_type = CIPHER_TYPE_AES192;
		/* Subtract a multiple of the block size rather than
		 * exact key len
		 */
		ctx->max_payload -= AES_KEYSIZE_256;	/* yes, 256 */
		break;
	case AES_KEYSIZE_256:
		ctx->cipher_type = CIPHER_TYPE_AES256;
		ctx->max_payload -= AES_KEYSIZE_256;
		break;
	default:
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}
	WARN_ON((ctx->max_payload % AES_BLOCK_SIZE) != 0);
	return 0;
}

static int rc4_setkey(struct crypto_ablkcipher *cipher, const u8 *key,
		      unsigned keylen)
{
	struct iproc_ctx_s *ctx = crypto_ablkcipher_ctx(cipher);
	int i;

	ctx->enckeylen = ARC4_MAX_KEY_SIZE + ARC4_STATE_SIZE;

	ctx->enckey[0] = 0x00;	/* 0x00 */
	ctx->enckey[1] = 0x00;	/* i    */
	ctx->enckey[2] = 0x00;	/* 0x00 */
	ctx->enckey[3] = 0x00;	/* j    */
	for (i = 0; i < ARC4_MAX_KEY_SIZE; i++)
		ctx->enckey[i + ARC4_STATE_SIZE] = key[i % keylen];

	ctx->cipher_type = CIPHER_TYPE_INIT;

	return 0;
}

static int ablkcipher_setkey(struct crypto_ablkcipher *cipher, const u8 *key,
			     unsigned keylen)
{
	struct spu_hw *spu = &iproc_priv.spu;
	struct iproc_ctx_s *ctx = crypto_ablkcipher_ctx(cipher);
	struct spu_cipher_parms cipher_parms;
	u32 alloc_len = 0;
	int err;

	flow_log("ablkcipher_setkey() keylen: %d\n", keylen);
	flow_dump("  key: ", key, keylen);

	switch (ctx->cipher.alg) {
	case CIPHER_ALG_DES:
		err = des_setkey(cipher, key, keylen);
		break;
	case CIPHER_ALG_3DES:
		err = threedes_setkey(cipher, key, keylen);
		break;
	case CIPHER_ALG_AES:
		err = aes_setkey(cipher, key, keylen);
		break;
	case CIPHER_ALG_RC4:
		err = rc4_setkey(cipher, key, keylen);
		break;
	default:
		pr_err("%s() Error: unknown cipher alg\n", __func__);
		err = -EINVAL;
	}
	if (err)
		return err;

	/* RC4 already populated ctx->enkey */
	if (ctx->cipher.alg != CIPHER_ALG_RC4) {
		memcpy(ctx->enckey, key, keylen);
		ctx->enckeylen = keylen;
	}

	if (spu->spu_type == SPU_TYPE_SPUM)
		alloc_len = BCM_HDR_LEN + SPU_HEADER_ALLOC_LEN;
	else if (spu->spu_type == SPU_TYPE_SPU2)
		alloc_len = BCM_HDR_LEN + SPU2_HEADER_ALLOC_LEN;
	memset(ctx->bcm_spu_req_hdr, 0, alloc_len);
	cipher_parms.iv_buf = NULL;
	cipher_parms.iv_len = crypto_ablkcipher_ivsize(cipher);
	flow_log("%s: iv_len %u\n", __func__, cipher_parms.iv_len);

	cipher_parms.alg = ctx->cipher.alg;
	cipher_parms.mode = ctx->cipher.mode;
	cipher_parms.type = ctx->cipher_type;
	cipher_parms.key_buf = ctx->enckey;
	cipher_parms.key_len = ctx->enckeylen;

	/* Prepend SPU request message with BCM header */
	memcpy(ctx->bcm_spu_req_hdr, BCMHEADER, BCM_HDR_LEN);
	ctx->spu_req_hdr_len =
	    spu->spu_cipher_req_init(ctx->bcm_spu_req_hdr + BCM_HDR_LEN,
				     &cipher_parms);

	ctx->spu_resp_hdr_len = spu->spu_response_hdr_len(ctx->authkeylen,
							  ctx->enckeylen,
							  false);

	atomic_inc(&iproc_priv.setkey_cnt[SPU_OP_CIPHER]);

	return 0;
}

static int ablkcipher_encrypt(struct ablkcipher_request *req)
{
	flow_log("ablkcipher_encrypt() alkb_req:%p nbytes:%u\n", req,
		 req->nbytes);
	flow_log("sg src %p, sg dst %p\n", req->src, req->dst);
	flow_log("Number of sg entries in src: %u\n", sg_nents(req->src));
	flow_log("Number of sg entries in dst: %u\n", sg_nents(req->dst));

	return ablkcipher_enqueue(req, true);
}

static int ablkcipher_decrypt(struct ablkcipher_request *req)
{
	flow_log("ablkcipher_decrypt() alkb_req:%p nbytes:%u\n", req,
		 req->nbytes);
	return ablkcipher_enqueue(req, false);
}

/* ahash helpers */

static int ahash_enqueue(struct ahash_request *req)
{
	struct iproc_reqctx_s *rctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct iproc_ctx_s *ctx = crypto_ahash_ctx(tfm);
	int err = 0;

	flow_log("ahash_enqueue() req:%p base:%p nbytes:%u\n", req, &req->base,
		 req->nbytes);

	rctx->parent = &req->base;
	rctx->msg_buf = NULL;
	rctx->ctx = ctx;
	rctx->iv_ctr = NULL;
	rctx->bd_suppress = true;
	memset(&rctx->mb_mssg, 0, sizeof(struct brcm_message));

	/* Initialize position in src scatterlist */
	rctx->src_sg = req->src;
	rctx->src_skip = 0;
	rctx->src_nents = 0;
	rctx->dst_sg = NULL;
	rctx->dst_skip = 0;
	rctx->dst_nents = 0;

	/* SPU2 hardware does not compute hash of zero length data */
	if ((rctx->is_final == 1) && (rctx->total_todo == 0) &&
	    (iproc_priv.spu.spu_type == SPU_TYPE_SPU2)) {
		pr_err("%s() Error: hash of zero length data\n",
		     __func__);
		return -EINVAL;
	}

	/* Allocate a set of buffers to be used as SPU message fragments */
	rctx->msg_buf = kzalloc(sizeof(struct spu_msg_buf), GFP_KERNEL);
	if (rctx->msg_buf == NULL)
		return -ENOMEM;

	/* tcrypt points result to stack allocated buffer, which dma mapping
	 * code doesn't like. So work with local buffer, and copy to result
	 * when we're all done. Not sure if this copy is required, but
	 * likely doesn't hurt.
	 */
	memcpy(rctx->msg_buf->digest, req->result, ctx->digestsize);

	/* Choose a SPU to process this request */
	rctx->chan_idx = select_channel();

	err = handle_ahash_req(rctx);
	if (err != -EINPROGRESS) {
		if (err == -EAGAIN)
			/* we saved data in hash carry, but tell crypto API
			 * we successfully completed request.
			 */
			err = 0;
		finish_req(rctx, err);
	}

	return -EINPROGRESS;
}

static int ahash_init(struct ahash_request *req)
{
	struct spu_hw *spu = &iproc_priv.spu;
	struct iproc_reqctx_s *rctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct iproc_ctx_s *ctx = crypto_ahash_ctx(tfm);

	flow_log("ahash_init() req:%p\n", req);

	/* Initialize the context */
	rctx->hash_carry_len = 0;
	rctx->is_final = 0;

	rctx->total_todo = 0;
	rctx->src_sent = rctx->total_sent = 0;
	rctx->total_received = 0;

	ctx->digestsize = crypto_ahash_digestsize(tfm);
	/* If we add a hash whose digest is larger, catch it here. */
	WARN_ON(ctx->digestsize > MAX_DIGEST_SIZE);

	rctx->is_sw_hmac = false;

	ctx->spu_resp_hdr_len = spu->spu_response_hdr_len(ctx->authkeylen, 0,
							  true);

	return 0;
}

static int ahash_update(struct ahash_request *req)
{
	struct iproc_reqctx_s *rctx = ahash_request_ctx(req);

	flow_log("ahash_update() req:%p nbytes:%u\n", req, req->nbytes);
	/* dump_sg(req->src, req->nbytes); */

	if (!req->nbytes)
		return 0;
	rctx->total_todo += req->nbytes;

	return ahash_enqueue(req);
}

static int ahash_final(struct ahash_request *req)
{
	struct iproc_reqctx_s *rctx = ahash_request_ctx(req);

	flow_log("ahash_final() req:%p nbytes:%u\n", req, req->nbytes);
	/* dump_sg(req->src, req->nbytes); */

	rctx->is_final = 1;

	return ahash_enqueue(req);
}

static int ahash_finup(struct ahash_request *req)
{
	struct iproc_reqctx_s *rctx = ahash_request_ctx(req);

	flow_log("ahash_finup() req:%p nbytes:%u\n", req, req->nbytes);
	/* dump_sg(req->src, req->nbytes); */

	rctx->total_todo += req->nbytes;
	rctx->is_final = 1;

	return ahash_enqueue(req);
}

static int ahash_digest(struct ahash_request *req)
{
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	int err = 0;

	flow_log("ahash_digest() req:%p nbytes:%u\n", req, req->nbytes);

	/* whole thing at once */
	err = ahash->init(req);
	if (!err)
		err = ahash->finup(req);

	return err;
}

/*  HMAC ahash functions */

static int ahash_hmac_setkey(struct crypto_ahash *ahash, const u8 *key,
			     unsigned keylen)
{
	struct iproc_ctx_s *ctx = crypto_ahash_ctx(ahash);
	unsigned blocksize = crypto_tfm_alg_blocksize(crypto_ahash_tfm(ahash));
	unsigned digestsize = crypto_ahash_digestsize(ahash);
	unsigned index;

	flow_log("%s() ahash:%p key:%p keylen:%u blksz:%u digestsz:%u\n",
		 __func__, ahash, key, keylen, blocksize, digestsize);
	flow_dump("  key: ", key, keylen);

	if (keylen > blocksize) {
		switch (ctx->auth.alg) {
		case HASH_ALG_MD5:
			do_shash("md5", ctx->ipad, key, keylen, NULL, 0);
			break;
		case HASH_ALG_SHA1:
			do_shash("sha1", ctx->ipad, key, keylen, NULL, 0);
			break;
		case HASH_ALG_SHA224:
			do_shash("sha224", ctx->ipad, key, keylen, NULL, 0);
			break;
		case HASH_ALG_SHA256:
			do_shash("sha256", ctx->ipad, key, keylen, NULL, 0);
			break;
		default:
			pr_err("%s() Error: unknown hash alg\n", __func__);
			return -EINVAL;
		}

		keylen = digestsize;

		flow_log("  keylen > digestsize... hashed\n");
		flow_dump("  newkey: ", ctx->ipad, keylen);
	} else {
		memcpy(ctx->ipad, key, keylen);
	}

	ctx->digestsize = digestsize;
	/* not "keylen" since we are using hash only operation */
	ctx->authkeylen = 0;

	memset(ctx->ipad + keylen, 0, blocksize - keylen);
	memcpy(ctx->opad, ctx->ipad, blocksize);

	for (index = 0; index < blocksize; index++) {
		ctx->ipad[index] ^= 0x36;
		ctx->opad[index] ^= 0x5c;
	}

	atomic_inc(&iproc_priv.setkey_cnt[SPU_OP_HMAC]);

	flow_dump("  ipad: ", ctx->ipad, HASH_BLOCK_SIZE);
	flow_dump("  opad: ", ctx->opad, HASH_BLOCK_SIZE);

	return 0;
}

static int ahash_hmac_init(struct ahash_request *req)
{
	struct iproc_reqctx_s *rctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct iproc_ctx_s *ctx = crypto_ahash_ctx(tfm);

	flow_log("ahash_hmac_init() req:%p\n", req);

	/* init the context as a hash, but */
	ahash_init(req);

	/* start with a prepended ipad */
	memcpy(rctx->hash_carry, ctx->ipad, HASH_BLOCK_SIZE);
	rctx->hash_carry_len = HASH_BLOCK_SIZE;
	rctx->total_todo += HASH_BLOCK_SIZE;

	ctx->auth.mode = HASH_MODE_HASH;

	rctx->is_sw_hmac = true;

	return 0;
}

static int ahash_hmac_update(struct ahash_request *req)
{
	flow_log("ahash_hmac_update() req:%p nbytes:%u\n", req, req->nbytes);
	/* dump_sg(req->src, req->nbytes); */

	if (!req->nbytes)
		return 0;

	return ahash_update(req);
}

static int ahash_hmac_final(struct ahash_request *req)
{
	flow_log("ahash_hmac_final() req:%p nbytes:%u\n", req, req->nbytes);
	/* dump_sg(req->src, req->nbytes); */

	return ahash_final(req);
}

static int ahash_hmac_finup(struct ahash_request *req)
{
	flow_log("ahash_hmac_finupl() req:%p nbytes:%u\n", req, req->nbytes);
	/* dump_sg(req->src, req->nbytes); */

	return ahash_finup(req);
}

static int ahash_hmac_digest(struct ahash_request *req)
{
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	int err = 0;

	flow_log("ahash_hmac_digest() req:%p nbytes:%u\n", req, req->nbytes);
	/* dump_sg(req->src, req->nbytes); */

	/* whole thing at once */
	err = ahash->init(req);
	if (!err)
		err = ahash->finup(req);

	return err;
}

/* aead helpers */

static int aead_need_fallback(struct aead_request *req)
{
	struct iproc_reqctx_s *rctx = aead_request_ctx(req);
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct iproc_ctx_s *ctx = crypto_aead_ctx(aead);

	unsigned packetlen =
	    (ctx->authkeylen + ctx->enckeylen + rctx->iv_ctr_len +
	     req->assoclen + (req->cryptlen & 0xffff) + 40);

	flow_log("%s() packetlen:%u\n", __func__, packetlen);

	return packetlen > ctx->max_payload;
}

static void aead_complete(struct crypto_async_request *areq, int err)
{
	struct aead_request *req =
	    container_of(areq, struct aead_request, base);
	struct iproc_reqctx_s *rctx = aead_request_ctx(req);
	struct crypto_aead *aead = crypto_aead_reqtfm(req);

	flow_log("%s() req:%p err:%d\n", __func__, areq, err);

	areq->tfm = crypto_aead_tfm(aead);

	areq->complete = rctx->old_complete;
	areq->data = rctx->old_data;

	areq->complete(areq, err);
}

static int aead_do_fallback(struct aead_request *req, bool is_encrypt)
{
	struct crypto_tfm *tfm = crypto_aead_tfm(crypto_aead_reqtfm(req));
	struct iproc_reqctx_s *rctx = aead_request_ctx(req);
	struct iproc_ctx_s *ctx = crypto_tfm_ctx(tfm);
	int err;

	flow_log("%s() req:%p enc:%u\n", __func__, req, is_encrypt);

	if (ctx->fallback_cipher) {
		/* Store the cipher tfm and then use the fallback tfm */
		rctx->old_tfm = tfm;
		aead_request_set_tfm(req, ctx->fallback_cipher);
		/* Save the callback and chain ourselves in, so we can restore
		 * the tfm
		 */
		rctx->old_complete = req->base.complete;
		rctx->old_data = req->base.data;
		aead_request_set_callback(req, aead_request_flags(req),
					  aead_complete, req);

		err =
		    is_encrypt ? crypto_aead_encrypt(req) :
		    crypto_aead_decrypt(req);
	} else
		err = -EINVAL;

	return err;
}

/* req->src now includes associated data. */
static int aead_enqueue(struct aead_request *req, bool is_encrypt)
{
	struct iproc_reqctx_s *rctx = aead_request_ctx(req);
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct iproc_ctx_s *ctx = crypto_aead_ctx(aead);
	int err;

	flow_log("%s() req:%p enc:%u\n", __func__, req, is_encrypt);

	if (req->assoclen > MAX_ASSOC_SIZE) {
		pr_err
		    ("%s() Error: associated data too long. (%u > %u bytes)\n",
		     __func__, req->assoclen, MAX_ASSOC_SIZE);
		return -EINVAL;
	}

	/* SPU cannot handle the trivial AES-GCM case where Plaintext and AAD
	 * are both 0 bytes long
	 */
	if ((ctx->cipher.mode == CIPHER_MODE_GCM) &&
	    (req->cryptlen + req->assoclen) == 0) {
		pr_err("%s() Error: For GCM mode, SPU requires either associated data or text\n",
		     __func__);
		return -EINVAL;
	}

	rctx->parent = &req->base;
	rctx->is_encrypt = is_encrypt;
	rctx->bd_suppress = false;
	rctx->total_todo = req->cryptlen;
	rctx->src_sent = rctx->total_sent = 0;
	rctx->total_received = 0;
	rctx->is_sw_hmac = false;
	rctx->ctx = ctx;
	memset(&rctx->mb_mssg, 0, sizeof(struct brcm_message));

	/* assoc data is at start of src sg */
	rctx->assoc = req->src;

	/* Allocate a set of buffers to be used as SPU message fragments */
	rctx->msg_buf = kzalloc(sizeof(struct spu_msg_buf), GFP_KERNEL);
	if (rctx->msg_buf == NULL)
		return -ENOMEM;

	/*
	 * Init current position in src scatterlist to be after assoc data.
	 * src_skip set to buffer offset where data begins. (Assoc data could
	 * end in the middle of a buffer.)
	 */
	if (spu_sg_at_offset(req->src, req->assoclen, &rctx->src_sg,
			     &rctx->src_skip) < 0) {
		pr_err("%s() Error: Unable to find start of src data\n",
		     __func__);
		return -EINVAL;
	}

	rctx->src_nents = 0;
	rctx->dst_nents = 0;
	if (req->dst == req->src) {
		rctx->dst_sg = rctx->src_sg;
		rctx->dst_skip = rctx->src_skip;
	} else {
		/* Expect req->dst to have room for assoc data followed by
		 * output data and ICV, if encrypt. So initialize dst_sg
		 * to point beyond assoc len offset.
		 */
		if (spu_sg_at_offset(req->dst, req->assoclen, &rctx->dst_sg,
				     &rctx->dst_skip) < 0) {
			pr_err("%s() Error: Unable to find start of dst data\n",
			     __func__);
			return -EINVAL;
		}
	}

	if (ctx->cipher.mode == CIPHER_MODE_CBC ||
	    ctx->cipher.mode == CIPHER_MODE_CTR ||
	    ctx->cipher.mode == CIPHER_MODE_OFB ||
	    ctx->cipher.mode == CIPHER_MODE_GCM) {
		rctx->iv_ctr_len = crypto_aead_ivsize(crypto_aead_reqtfm(req));
	} else {
		rctx->iv_ctr_len = 0;
	}

	rctx->hash_carry_len = 0;

	flow_log("  src sg: %p\n", req->src);
	flow_log("  rctx->src_sg: %p, src_skip %u\n",
		 rctx->src_sg, rctx->src_skip);
	flow_log("  assoc:  %p, assoclen %u\n", rctx->assoc, req->assoclen);
	flow_log("  dst sg: %p\n", req->dst);
	flow_log("  rctx->dst_sg: %p, dst_skip %u\n",
		 rctx->dst_sg, rctx->dst_skip);
	flow_log("  iv_ctr_len:%u\n", rctx->iv_ctr_len);
	flow_dump("  iv: ", req->iv, rctx->iv_ctr_len);
	flow_log("  authkeylen:%u\n", ctx->authkeylen);
	flow_log("  ctx:%p\n", ctx);
	flow_log("  max_payload: %u\n", ctx->max_payload);

	if (rctx->iv_ctr_len) {
		rctx->iv_ctr = kmalloc(rctx->iv_ctr_len, GFP_KERNEL);
		if (rctx->iv_ctr == NULL)
			return -ENOMEM;
		memcpy(rctx->iv_ctr, req->iv, rctx->iv_ctr_len);
	}

	/* If we need authenc.c to handle the request then do it... */
	if (unlikely(aead_need_fallback(req)))
		return aead_do_fallback(req, is_encrypt);

	rctx->chan_idx = select_channel();
	err = handle_aead_req(rctx);
	if (err != -EINPROGRESS)
		finish_req(rctx, err);

	return -EINPROGRESS;
}

static int aead_authenc_setkey(struct crypto_aead *cipher,
			       const u8 *key, unsigned keylen)
{
	struct spu_hw *spu = &iproc_priv.spu;
	struct iproc_ctx_s *ctx = crypto_aead_ctx(cipher);
	struct crypto_tfm *tfm = crypto_aead_tfm(cipher);
	struct rtattr *rta = (void *)key;
	struct crypto_authenc_key_param *param;
	const u8 *origkey = key;
	const unsigned origkeylen = keylen;

	int ret = 0;

	flow_log("%s() aead:%p key:%p keylen:%u\n", __func__, cipher, key,
		 keylen);
	flow_dump("  key: ", key, keylen);

	if (!RTA_OK(rta, keylen))
		goto badkey;
	if (rta->rta_type != CRYPTO_AUTHENC_KEYA_PARAM)
		goto badkey;
	if (RTA_PAYLOAD(rta) < sizeof(*param))
		goto badkey;

	param = RTA_DATA(rta);
	ctx->enckeylen = be32_to_cpu(param->enckeylen);

	key += RTA_ALIGN(rta->rta_len);
	keylen -= RTA_ALIGN(rta->rta_len);

	if (keylen < ctx->enckeylen)
		goto badkey;
	if (ctx->enckeylen > MAX_KEY_SIZE)
		goto badkey;

	ctx->authkeylen = keylen - ctx->enckeylen;

	if (ctx->authkeylen > MAX_KEY_SIZE)
		goto badkey;

	memcpy(ctx->enckey, key + ctx->authkeylen, ctx->enckeylen);
	/* May end up padding auth key. So make sure it's zeroed. */
	memset(ctx->authkey, 0, sizeof(ctx->authkey));
	memcpy(ctx->authkey, key, ctx->authkeylen);

	switch (ctx->alg->cipher_info.alg) {
	case CIPHER_ALG_DES:
		if (ctx->enckeylen == DES_KEY_SIZE) {
			u32 tmp[DES_EXPKEY_WORDS];

			if (des_ekey(tmp, key) == 0) {
				if (crypto_aead_get_flags(cipher) &
				    CRYPTO_TFM_REQ_WEAK_KEY) {
					crypto_aead_set_flags(cipher,
						CRYPTO_TFM_RES_WEAK_KEY);
					return -EINVAL;
				}
			}

			ctx->cipher_type = CIPHER_TYPE_DES;
		} else {
			goto badkey;
		}
		break;
	case CIPHER_ALG_3DES:
		if (ctx->enckeylen == (DES_KEY_SIZE * 3)) {
			const u32 *K = (const u32 *)key;

			if (!((K[0] ^ K[2]) | (K[1] ^ K[3])) ||
			    !((K[2] ^ K[4]) | (K[3] ^ K[5]))) {
				crypto_aead_set_flags(cipher,
					      CRYPTO_TFM_RES_BAD_KEY_SCHED);
				return -EINVAL;
			}

			ctx->cipher_type = CIPHER_TYPE_3DES;
		} else {
			crypto_aead_set_flags(cipher,
					      CRYPTO_TFM_RES_BAD_KEY_LEN);
			return -EINVAL;
		}
		break;
	case CIPHER_ALG_AES:
		switch (ctx->enckeylen) {
		case AES_KEYSIZE_128:
			ctx->cipher_type = CIPHER_TYPE_AES128;
			break;
		case AES_KEYSIZE_192:
			ctx->cipher_type = CIPHER_TYPE_AES192;
			break;
		case AES_KEYSIZE_256:
			ctx->cipher_type = CIPHER_TYPE_AES256;
			break;
		default:
			goto badkey;
		}
		break;
	case CIPHER_ALG_RC4:
		ctx->cipher_type = CIPHER_TYPE_INIT;
		break;
	default:
		pr_err("%s() Error: Unknown cipher alg\n", __func__);
		return -EINVAL;
	}

	flow_log("  enckeylen:%u authkeylen:%u\n", ctx->enckeylen,
		 ctx->authkeylen);
	flow_dump("  enc: ", ctx->enckey, ctx->enckeylen);
	flow_dump("  auth: ", ctx->authkey, ctx->authkeylen);

	/* setkey the fallback just in case we needto use it */
	if (ctx->fallback_cipher) {
		flow_log("  running fallback setkey()\n");

		ctx->fallback_cipher->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
		ctx->fallback_cipher->base.crt_flags |=
		    tfm->crt_flags & CRYPTO_TFM_REQ_MASK;
		ret =
		    crypto_aead_setkey(ctx->fallback_cipher, origkey,
				       origkeylen);
		if (ret) {
			flow_log("  fallback setkey() returned:%d\n", ret);
			tfm->crt_flags &= ~CRYPTO_TFM_RES_MASK;
			tfm->crt_flags |=
			    (ctx->fallback_cipher->
			     base.crt_flags & CRYPTO_TFM_RES_MASK);
		}
	}

	ctx->spu_resp_hdr_len = spu->spu_response_hdr_len(ctx->authkeylen,
							  ctx->enckeylen,
							  false);

	atomic_inc(&iproc_priv.setkey_cnt[SPU_OP_AEAD]);

	return ret;

badkey:
	ctx->enckeylen = 0;
	ctx->authkeylen = 0;
	ctx->digestsize = 0;

	crypto_aead_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
	return -EINVAL;
}

static int aead_gcm_setkey(struct crypto_aead *cipher,
			   const u8 *key, unsigned keylen)
{
	struct spu_hw *spu = &iproc_priv.spu;
	struct iproc_ctx_s *ctx = crypto_aead_ctx(cipher);
	struct crypto_tfm *tfm = crypto_aead_tfm(cipher);
	const u8 *origkey = key;
	const unsigned origkeylen = keylen;

	int ret = 0;

	flow_log("%s() aead:%p key:%p keylen:%u\n", __func__, cipher, key,
		 keylen);
	flow_dump("  key: ", key, keylen);

	ctx->enckeylen = keylen;
	ctx->digestsize = keylen;
	ctx->authkeylen = 0;

	memcpy(ctx->enckey, key, ctx->enckeylen);

	switch (ctx->enckeylen) {
	case AES_KEYSIZE_128:
		ctx->cipher_type = CIPHER_TYPE_AES128;
		break;
	case AES_KEYSIZE_192:
		ctx->cipher_type = CIPHER_TYPE_AES192;
		break;
	case AES_KEYSIZE_256:
		ctx->cipher_type = CIPHER_TYPE_AES256;
		break;
	default:
		goto badkey;
	}

	flow_log("  enckeylen:%u authkeylen:%u\n", ctx->enckeylen,
		 ctx->authkeylen);
	flow_dump("  enc: ", ctx->enckey, ctx->enckeylen);
	flow_dump("  auth: ", ctx->authkey, ctx->authkeylen);

	/* setkey the fallback just in case we need to use it */
	if (ctx->fallback_cipher) {
		flow_log("  running fallback setkey()\n");

		ctx->fallback_cipher->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
		ctx->fallback_cipher->base.crt_flags |=
		    tfm->crt_flags & CRYPTO_TFM_REQ_MASK;
		ret =
		    crypto_aead_setkey(ctx->fallback_cipher, origkey,
				       origkeylen);
		if (ret) {
			flow_log("  fallback setkey() returned:%d\n", ret);
			tfm->crt_flags &= ~CRYPTO_TFM_RES_MASK;
			tfm->crt_flags |=
			    (ctx->fallback_cipher->
			     base.crt_flags & CRYPTO_TFM_RES_MASK);
		}
	}

	ctx->spu_resp_hdr_len = spu->spu_response_hdr_len(ctx->authkeylen,
							  ctx->enckeylen,
							  false);

	atomic_inc(&iproc_priv.setkey_cnt[SPU_OP_AEAD]);

	flow_log("  enckeylen:%u authkeylen:%u\n", ctx->enckeylen,
		 ctx->authkeylen);
	flow_log("  ctx:%p\n", ctx);

	return ret;

badkey:
	ctx->enckeylen = 0;
	ctx->authkeylen = 0;
	ctx->digestsize = 0;

	crypto_aead_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
	return -EINVAL;
}

static int aead_setauthsize(struct crypto_aead *cipher, unsigned authsize)
{
	struct iproc_ctx_s *ctx = crypto_aead_ctx(cipher);
	int ret = 0;

	flow_log("%s() aead:%p authkeylen:%u authsize:%u\n",
		 __func__, cipher, ctx->authkeylen, authsize);

	ctx->digestsize = authsize;

	/* setkey the fallback just in case we needto use it */
	if (ctx->fallback_cipher) {
		flow_log("  running fallback setauth()\n");

		ret = crypto_aead_setauthsize(ctx->fallback_cipher, authsize);
		if (ret)
			flow_log("  fallback setauth() returned:%d\n", ret);
	}

	return ret;
}

static int aead_encrypt(struct aead_request *req)
{
	flow_log("%s() aead_req:%p cryptlen:%u %08x\n", __func__, req,
		 req->cryptlen, req->cryptlen);
	dump_sg(req->src, 0, req->cryptlen + req->assoclen);
	flow_log("  assoc_len:%u\n", req->assoclen);

	return aead_enqueue(req, true);
}

static int aead_decrypt(struct aead_request *req)
{
	flow_log("%s() aead_req:%p cryptlen:%u %08x\n", __func__, req,
		 req->cryptlen, req->cryptlen);
	dump_sg(req->src, 0, req->cryptlen + req->assoclen);
	flow_log("  assoc_len:%u\n", req->assoclen);

	return aead_enqueue(req, false);
}

/* ==================== Supported Cipher Algorithms ==================== */

static struct iproc_alg_s driver_algs[] = {
/* AEAD algorithms. */
	/* AES-GCM */
	{
	 .type = CRYPTO_ALG_TYPE_AEAD,
	 .alg.aead = {
		 .base = {
			.cra_name = "gcm(aes)",
			.cra_driver_name = "gcm-aes-iproc",
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_NEED_FALLBACK
		 },
		.ivsize = 12,
		.maxauthsize = AES_BLOCK_SIZE,
	 },
	 .cipher_info = {
			 .alg = CIPHER_ALG_AES,
			 .mode = CIPHER_MODE_GCM,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_AES,
		       .mode = HASH_MODE_GCM,
		       },
	 .auth_first = 0,
	 .max_payload = -32,
	 .dtls_hmac = 0,
	 },

/* IPSEC AEAD algorithms. */
	/* enc -> hash - aes */
	{
	 .type = CRYPTO_ALG_TYPE_AEAD,
	 .alg.aead = {
		 .base = {
			.cra_name = "authenc(hmac(md5),cbc(aes))",
			.cra_driver_name = "authenc-hmac-md5-cbc-aes-iproc",
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_NEED_FALLBACK | CRYPTO_ALG_ASYNC
		 },
		.ivsize = AES_BLOCK_SIZE,
		.maxauthsize = MD5_DIGEST_SIZE,
	 },
	 .cipher_info = {
			 .alg = CIPHER_ALG_AES,
			 .mode = CIPHER_MODE_CBC,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_MD5,
		       .mode = HASH_MODE_HMAC,
		       },
	 .auth_first = 0,
	 .max_payload = -32,
	 .dtls_hmac = 0,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_AEAD,
	 .alg.aead = {
		 .base = {
			.cra_name = "authenc(hmac(sha1),cbc(aes))",
			.cra_driver_name = "authenc-hmac-sha1-cbc-aes-iproc",
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_NEED_FALLBACK | CRYPTO_ALG_ASYNC
		 },
		 .ivsize = AES_BLOCK_SIZE,
		 .maxauthsize = SHA1_DIGEST_SIZE,
	 },
	 .cipher_info = {
			 .alg = CIPHER_ALG_AES,
			 .mode = CIPHER_MODE_CBC,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_SHA1,
		       .mode = HASH_MODE_HMAC,
		       },
	 .auth_first = 0,
	 .max_payload = -32,
	 .dtls_hmac = 0,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_AEAD,
	 .alg.aead = {
		 .base = {
			.cra_name = "authenc(hmac(sha256),cbc(aes))",
			.cra_driver_name = "authenc-hmac-sha256-cbc-aes-iproc",
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_NEED_FALLBACK | CRYPTO_ALG_ASYNC
		 },
		 .ivsize = AES_BLOCK_SIZE,
		 .maxauthsize = SHA256_DIGEST_SIZE,
	 },
	 .cipher_info = {
			 .alg = CIPHER_ALG_AES,
			 .mode = CIPHER_MODE_CBC,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_SHA256,
		       .mode = HASH_MODE_HMAC,
		       },
	 .auth_first = 0,
	 .max_payload = -32,
	 .dtls_hmac = 0,
	 },
	/* enc -> hash - des */
	{
	 .type = CRYPTO_ALG_TYPE_AEAD,
	 .alg.aead = {
		 .base = {
			.cra_name = "authenc(hmac(md5),cbc(des))",
			.cra_driver_name = "authenc-hmac-md5-cbc-des-iproc",
			.cra_blocksize = DES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_NEED_FALLBACK | CRYPTO_ALG_ASYNC
		 },
		 .ivsize = DES_BLOCK_SIZE,
		 .maxauthsize = MD5_DIGEST_SIZE,
	 },
	 .cipher_info = {
			 .alg = CIPHER_ALG_DES,
			 .mode = CIPHER_MODE_CBC,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_MD5,
		       .mode = HASH_MODE_HMAC,
		       },
	 .auth_first = 0,
	 .max_payload = -32,
	 .dtls_hmac = 0,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_AEAD,
	 .alg.aead = {
		 .base = {
			.cra_name = "authenc(hmac(sha1),cbc(des))",
			.cra_driver_name = "authenc-hmac-sha1-cbc-des-iproc",
			.cra_blocksize = DES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_NEED_FALLBACK | CRYPTO_ALG_ASYNC
		 },
		 .ivsize = DES_BLOCK_SIZE,
		 .maxauthsize = SHA1_DIGEST_SIZE,
	 },
	 .cipher_info = {
			 .alg = CIPHER_ALG_DES,
			 .mode = CIPHER_MODE_CBC,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_SHA1,
		       .mode = HASH_MODE_HMAC,
		       },
	 .auth_first = 0,
	 .max_payload = -32,
	 .dtls_hmac = 0,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_AEAD,
	 .alg.aead = {
		 .base = {
			.cra_name = "authenc(hmac(sha224),cbc(des))",
			.cra_driver_name = "authenc-hmac-sha224-cbc-des-iproc",
			.cra_blocksize = DES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_NEED_FALLBACK | CRYPTO_ALG_ASYNC
		 },
		 .ivsize = DES_BLOCK_SIZE,
		 .maxauthsize = SHA224_DIGEST_SIZE,
	 },
	 .cipher_info = {
			 .alg = CIPHER_ALG_DES,
			 .mode = CIPHER_MODE_CBC,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_SHA224,
		       .mode = HASH_MODE_HMAC,
		       },
	 .auth_first = 0,
	 .max_payload = -32,
	 .dtls_hmac = 0,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_AEAD,
	 .alg.aead = {
		 .base = {
			.cra_name = "authenc(hmac(sha256),cbc(des))",
			.cra_driver_name = "authenc-hmac-sha256-cbc-des-iproc",
			.cra_blocksize = DES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_NEED_FALLBACK | CRYPTO_ALG_ASYNC
		 },
		 .ivsize = DES_BLOCK_SIZE,
		 .maxauthsize = SHA256_DIGEST_SIZE,
	 },
	 .cipher_info = {
			 .alg = CIPHER_ALG_DES,
			 .mode = CIPHER_MODE_CBC,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_SHA256,
		       .mode = HASH_MODE_HMAC,
		       },
	 .auth_first = 0,
	 .max_payload = -32,
	 .dtls_hmac = 0,
	 },
	/* enc -> hash - 3des */
	{
	 .type = CRYPTO_ALG_TYPE_AEAD,
	 .alg.aead = {
		 .base = {
			.cra_name = "authenc(hmac(md5),cbc(des3_ede))",
			.cra_driver_name = "authenc-hmac-md5-cbc-des3-iproc",
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_NEED_FALLBACK | CRYPTO_ALG_ASYNC
		 },
		 .ivsize = DES3_EDE_BLOCK_SIZE,
		 .maxauthsize = MD5_DIGEST_SIZE,
	 },
	 .cipher_info = {
			 .alg = CIPHER_ALG_3DES,
			 .mode = CIPHER_MODE_CBC,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_MD5,
		       .mode = HASH_MODE_HMAC,
		       },
	 .auth_first = 0,
	 .max_payload = -32,
	 .dtls_hmac = 0,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_AEAD,
	 .alg.aead = {
		 .base = {
			.cra_name = "authenc(hmac(sha1),cbc(des3_ede))",
			.cra_driver_name = "authenc-hmac-sha1-cbc-des3-iproc",
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_NEED_FALLBACK | CRYPTO_ALG_ASYNC
		 },
		 .ivsize = DES3_EDE_BLOCK_SIZE,
		 .maxauthsize = SHA1_DIGEST_SIZE,
	 },
	 .cipher_info = {
			 .alg = CIPHER_ALG_3DES,
			 .mode = CIPHER_MODE_CBC,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_SHA1,
		       .mode = HASH_MODE_HMAC,
		       },
	 .auth_first = 0,
	 .max_payload = -32,
	 .dtls_hmac = 0,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_AEAD,
	 .alg.aead = {
		 .base = {
			.cra_name = "authenc(hmac(sha224),cbc(des3_ede))",
			.cra_driver_name = "authenc-hmac-sha224-cbc-des3-iproc",
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_NEED_FALLBACK | CRYPTO_ALG_ASYNC
		 },
		 .ivsize = DES3_EDE_BLOCK_SIZE,
		 .maxauthsize = SHA224_DIGEST_SIZE,
	 },
	 .cipher_info = {
			 .alg = CIPHER_ALG_3DES,
			 .mode = CIPHER_MODE_CBC,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_SHA224,
		       .mode = HASH_MODE_HMAC,
		       },
	 .auth_first = 0,
	 .max_payload = -32,
	 .dtls_hmac = 0,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_AEAD,
	 .alg.aead = {
		 .base = {
			.cra_name = "authenc(hmac(sha256),cbc(des3_ede))",
			.cra_driver_name = "authenc-hmac-sha256-cbc-des3-iproc",
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_NEED_FALLBACK | CRYPTO_ALG_ASYNC
		 },
		 .ivsize = DES3_EDE_BLOCK_SIZE,
		 .maxauthsize = SHA256_DIGEST_SIZE,
	 },
	 .cipher_info = {
			 .alg = CIPHER_ALG_3DES,
			 .mode = CIPHER_MODE_CBC,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_SHA256,
		       .mode = HASH_MODE_HMAC,
		       },
	 .auth_first = 0,
	 .max_payload = -32,
	 .dtls_hmac = 0,
	 },

/* ABLKCIPHER algorithms. */
	{
	 .type = CRYPTO_ALG_TYPE_ABLKCIPHER,
	 .alg.crypto = {
			.cra_name = "ecb(arc4)",
			.cra_driver_name = "ecb-arc4-iproc",
			.cra_blocksize = ARC4_BLOCK_SIZE,
			.cra_ablkcipher = {
					   .min_keysize = ARC4_MIN_KEY_SIZE,
					   .max_keysize = ARC4_MAX_KEY_SIZE,
					   .ivsize = 0,
					   }
			},
	 .cipher_info = {
			 .alg = CIPHER_ALG_RC4,
			 .mode = CIPHER_MODE_NONE,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_NONE,
		       .mode = HASH_MODE_NONE,
		       },
	 .max_payload = -312,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_ABLKCIPHER,
	 .alg.crypto = {
			.cra_name = "ofb(des)",
			.cra_driver_name = "ofb-des-iproc",
			.cra_blocksize = DES_BLOCK_SIZE,
			.cra_ablkcipher = {
					   .min_keysize = DES_KEY_SIZE,
					   .max_keysize = DES_KEY_SIZE,
					   .ivsize = DES_BLOCK_SIZE,
					   }
			},
	 .cipher_info = {
			 .alg = CIPHER_ALG_DES,
			 .mode = CIPHER_MODE_OFB,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_NONE,
		       .mode = HASH_MODE_NONE,
		       },
	 .max_payload = -64,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_ABLKCIPHER,
	 .alg.crypto = {
			.cra_name = "cbc(des)",
			.cra_driver_name = "cbc-des-iproc",
			.cra_blocksize = DES_BLOCK_SIZE,
			.cra_ablkcipher = {
					   .min_keysize = DES_KEY_SIZE,
					   .max_keysize = DES_KEY_SIZE,
					   .ivsize = DES_BLOCK_SIZE,
					   }
			},
	 .cipher_info = {
			 .alg = CIPHER_ALG_DES,
			 .mode = CIPHER_MODE_CBC,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_NONE,
		       .mode = HASH_MODE_NONE,
		       },
	 .max_payload = -64,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_ABLKCIPHER,
	 .alg.crypto = {
			.cra_name = "ecb(des)",
			.cra_driver_name = "ecb-des-iproc",
			.cra_blocksize = DES_BLOCK_SIZE,
			.cra_ablkcipher = {
					   .min_keysize = DES_KEY_SIZE,
					   .max_keysize = DES_KEY_SIZE,
					   .ivsize = 0,
					   }
			},
	 .cipher_info = {
			 .alg = CIPHER_ALG_DES,
			 .mode = CIPHER_MODE_ECB,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_NONE,
		       .mode = HASH_MODE_NONE,
		       },
	 .max_payload = -64,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_ABLKCIPHER,
	 .alg.crypto = {
			.cra_name = "ofb(des3_ede)",
			.cra_driver_name = "ofb-des3-iproc",
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_ablkcipher = {
					   .min_keysize = DES3_EDE_KEY_SIZE,
					   .max_keysize = DES3_EDE_KEY_SIZE,
					   .ivsize = DES3_EDE_BLOCK_SIZE,
					   }
			},
	 .cipher_info = {
			 .alg = CIPHER_ALG_3DES,
			 .mode = CIPHER_MODE_OFB,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_NONE,
		       .mode = HASH_MODE_NONE,
		       },
	 .max_payload = -80,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_ABLKCIPHER,
	 .alg.crypto = {
			.cra_name = "cbc(des3_ede)",
			.cra_driver_name = "cbc-des3-iproc",
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_ablkcipher = {
					   .min_keysize = DES3_EDE_KEY_SIZE,
					   .max_keysize = DES3_EDE_KEY_SIZE,
					   .ivsize = DES3_EDE_BLOCK_SIZE,
					   }
			},
	 .cipher_info = {
			 .alg = CIPHER_ALG_3DES,
			 .mode = CIPHER_MODE_CBC,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_NONE,
		       .mode = HASH_MODE_NONE,
		       },
	 .max_payload = -80,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_ABLKCIPHER,
	 .alg.crypto = {
			.cra_name = "ecb(des3_ede)",
			.cra_driver_name = "ecb-des3-iproc",
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_ablkcipher = {
					   .min_keysize = DES3_EDE_KEY_SIZE,
					   .max_keysize = DES3_EDE_KEY_SIZE,
					   .ivsize = 0,
					   }
			},
	 .cipher_info = {
			 .alg = CIPHER_ALG_3DES,
			 .mode = CIPHER_MODE_ECB,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_NONE,
		       .mode = HASH_MODE_NONE,
		       },
	 .max_payload = -72,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_ABLKCIPHER,
	 .alg.crypto = {
			.cra_name = "ofb(aes)",
			.cra_driver_name = "ofb-aes-iproc",
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ablkcipher = {
					   .min_keysize = AES_MIN_KEY_SIZE,
					   .max_keysize = AES_MAX_KEY_SIZE,
					   .ivsize = AES_BLOCK_SIZE,
					   }
			},
	 .cipher_info = {
			 .alg = CIPHER_ALG_AES,
			 .mode = CIPHER_MODE_OFB,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_NONE,
		       .mode = HASH_MODE_NONE,
		       },
	 .max_payload = -64,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_ABLKCIPHER,
	 .alg.crypto = {
			.cra_name = "cbc(aes)",
			.cra_driver_name = "cbc-aes-iproc",
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ablkcipher = {
					   .min_keysize = AES_MIN_KEY_SIZE,
					   .max_keysize = AES_MAX_KEY_SIZE,
					   .ivsize = AES_BLOCK_SIZE,
					   }
			},
	 .cipher_info = {
			 .alg = CIPHER_ALG_AES,
			 .mode = CIPHER_MODE_CBC,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_NONE,
		       .mode = HASH_MODE_NONE,
		       },
	 .max_payload = -64,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_ABLKCIPHER,
	 .alg.crypto = {
			.cra_name = "ecb(aes)",
			.cra_driver_name = "ecb-aes-iproc",
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ablkcipher = {
					   .min_keysize = AES_MIN_KEY_SIZE,
					   .max_keysize = AES_MAX_KEY_SIZE,
					   .ivsize = 0,
					   }
			},
	 .cipher_info = {
			 .alg = CIPHER_ALG_AES,
			 .mode = CIPHER_MODE_ECB,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_NONE,
		       .mode = HASH_MODE_NONE,
		       },
	 .max_payload = -48,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_ABLKCIPHER,
	 .alg.crypto = {
			.cra_name = "ctr(aes)",
			.cra_driver_name = "ctr-aes-iproc",
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ablkcipher = {
					   /* .geniv = "chainiv", */
					   .min_keysize = AES_MIN_KEY_SIZE,
					   .max_keysize = AES_MAX_KEY_SIZE,
					   .ivsize = AES_BLOCK_SIZE,
					   }
			},
	 .cipher_info = {
			 .alg = CIPHER_ALG_AES,
			 .mode = CIPHER_MODE_CTR,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_NONE,
		       .mode = HASH_MODE_NONE,
		       },
	 .max_payload = -96,
	 },

/* AHASH algorithms. */
	{
	 .type = CRYPTO_ALG_TYPE_AHASH,
	 .alg.hash = {
		      .halg.digestsize = MD5_DIGEST_SIZE,
		      .halg.base = {
				    .cra_name = "md5",
				    .cra_driver_name = "md5-iproc",
				    .cra_blocksize = MD5_BLOCK_WORDS * 4,
				    .cra_flags = CRYPTO_ALG_TYPE_AHASH |
					     CRYPTO_ALG_ASYNC,
				    }
		      },
	 .cipher_info = {
			 .alg = CIPHER_ALG_NONE,
			 .mode = CIPHER_MODE_NONE,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_MD5,
		       .mode = HASH_MODE_HASH,
		       },
	 .max_payload = -128,
	 },
	{
	 .type = CRYPTO_ALG_TYPE_AHASH,
	 .alg.hash = {
		      .halg.digestsize = MD5_DIGEST_SIZE,
		      .halg.base = {
				    .cra_name = "hmac(md5)",
				    .cra_driver_name = "hmac-md5-iproc",
				    .cra_blocksize = MD5_BLOCK_WORDS * 4,
				    }
		      },
	 .cipher_info = {
			 .alg = CIPHER_ALG_NONE,
			 .mode = CIPHER_MODE_NONE,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_MD5,
		       .mode = HASH_MODE_HMAC,
		       },
	 .max_payload = -128,
	 },
	{.type = CRYPTO_ALG_TYPE_AHASH,
	 .alg.hash = {
		      .halg.digestsize = SHA1_DIGEST_SIZE,
		      .halg.base = {
				    .cra_name = "sha1",
				    .cra_driver_name = "sha1-iproc",
				    .cra_blocksize = SHA1_BLOCK_SIZE,
				    }
		      },
	 .cipher_info = {
			 .alg = CIPHER_ALG_NONE,
			 .mode = CIPHER_MODE_NONE,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_SHA1,
		       .mode = HASH_MODE_HASH,
		       },
	 .max_payload = -128,
	 },
	{.type = CRYPTO_ALG_TYPE_AHASH,
	 .alg.hash = {
		      .halg.digestsize = SHA1_DIGEST_SIZE,
		      .halg.base = {
				    .cra_name = "hmac(sha1)",
				    .cra_driver_name = "hmac-sha1-iproc",
				    .cra_blocksize = SHA1_BLOCK_SIZE,
				    }
		      },
	 .cipher_info = {
			 .alg = CIPHER_ALG_NONE,
			 .mode = CIPHER_MODE_NONE,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_SHA1,
		       .mode = HASH_MODE_HMAC,
		       },
	 .max_payload = -128,
	 },
	{.type = CRYPTO_ALG_TYPE_AHASH,
	 .alg.hash = {
		      .halg.digestsize = SHA224_DIGEST_SIZE,
		      .halg.base = {
				    .cra_name = "sha224",
				    .cra_driver_name = "sha224-iproc",
				    .cra_blocksize = SHA224_BLOCK_SIZE,
				    }
		      },
	 .cipher_info = {
			 .alg = CIPHER_ALG_NONE,
			 .mode = CIPHER_MODE_NONE,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_SHA224,
		       .mode = HASH_MODE_HASH,
		       },
	 .max_payload = -128,
	 },
	{.type = CRYPTO_ALG_TYPE_AHASH,
	 .alg.hash = {
		      .halg.digestsize = SHA224_DIGEST_SIZE,
		      .halg.base = {
				    .cra_name = "hmac(sha224)",
				    .cra_driver_name = "hmac-sha224-iproc",
				    .cra_blocksize = SHA224_BLOCK_SIZE,
				    }
		      },
	 .cipher_info = {
			 .alg = CIPHER_ALG_NONE,
			 .mode = CIPHER_MODE_NONE,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_SHA224,
		       .mode = HASH_MODE_HMAC,
		       },
	 .max_payload = -128,
	 },
	{.type = CRYPTO_ALG_TYPE_AHASH,
	 .alg.hash = {
		      .halg.digestsize = SHA256_DIGEST_SIZE,
		      .halg.base = {
				    .cra_name = "sha256",
				    .cra_driver_name = "sha256-iproc",
				    .cra_blocksize = SHA256_BLOCK_SIZE,
				    }
		      },
	 .cipher_info = {
			 .alg = CIPHER_ALG_NONE,
			 .mode = CIPHER_MODE_NONE,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_SHA256,
		       .mode = HASH_MODE_HASH,
		       },
	 .max_payload = -128,
	 },
	{.type = CRYPTO_ALG_TYPE_AHASH,
	 .alg.hash = {
		      .halg.digestsize = SHA256_DIGEST_SIZE,
		      .halg.base = {
				    .cra_name = "hmac(sha256)",
				    .cra_driver_name = "hmac-sha256-iproc",
				    .cra_blocksize = SHA256_BLOCK_SIZE,
				    }
		      },
	 .cipher_info = {
			 .alg = CIPHER_ALG_NONE,
			 .mode = CIPHER_MODE_NONE,
			 },
	 .auth_info = {
		       .alg = HASH_ALG_SHA256,
		       .mode = HASH_MODE_HMAC,
		       },
	 .max_payload = -128,
	 },
};

static int generic_cra_init(struct crypto_tfm *tfm,
			    struct iproc_alg_s *cipher_alg)
{
	struct iproc_ctx_s *ctx = crypto_tfm_ctx(tfm);

	flow_log("%s() tfm:%p ctx:%p\n", __func__, tfm, ctx);

	ctx->alg = cipher_alg;
	ctx->cipher = cipher_alg->cipher_info;
	ctx->auth = cipher_alg->auth_info;
	ctx->auth_first = cipher_alg->auth_first;
	ctx->max_payload = (unsigned) cipher_alg->max_payload;
	ctx->fallback_cipher = NULL;

	ctx->enckeylen = 0;
	ctx->authkeylen = 0;

	atomic_inc(&iproc_priv.stream_count);
	atomic_inc(&iproc_priv.session_count);

	return 0;
}

static int ablkcipher_cra_init(struct crypto_tfm *tfm)
{
	struct crypto_alg *alg = tfm->__crt_alg;
	struct iproc_alg_s *cipher_alg;

	flow_log("%s() tfm:%p\n", __func__, tfm);

	tfm->crt_ablkcipher.reqsize = sizeof(struct iproc_reqctx_s);

	cipher_alg = container_of(alg, struct iproc_alg_s, alg.crypto);
	return generic_cra_init(tfm, cipher_alg);
}

static int ahash_cra_init(struct crypto_tfm *tfm)
{
	int err;
	struct crypto_alg *alg = tfm->__crt_alg;
	struct iproc_alg_s *cipher_alg;

	cipher_alg = container_of(__crypto_ahash_alg(alg), struct iproc_alg_s,
				  alg.hash);

	err = generic_cra_init(tfm, cipher_alg);
	flow_log("%s() tfm:%p\n", __func__, tfm);

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct iproc_reqctx_s));

	return err;
}

static int aead_cra_init(struct crypto_aead *aead)
{
	struct crypto_tfm *tfm = crypto_aead_tfm(aead);
	struct iproc_ctx_s *ctx = crypto_tfm_ctx(tfm);
	struct crypto_alg *alg = tfm->__crt_alg;
	struct aead_alg *aalg = container_of(alg, struct aead_alg, base);
	struct iproc_alg_s *cipher_alg = container_of(aalg, struct iproc_alg_s,
						      alg.aead);

	int err = generic_cra_init(tfm, cipher_alg);

	flow_log("%s() tfm:%p\n", __func__, tfm);

	crypto_aead_set_reqsize(aead, sizeof(struct iproc_reqctx_s));

	/* random first IV */
	get_random_bytes(ctx->iv, MAX_IV_SIZE);
	flow_dump("  iv: ", ctx->iv, MAX_IV_SIZE);

	if (!err) {
		if (alg->cra_flags & CRYPTO_ALG_NEED_FALLBACK) {
			flow_log("%s() creating fallback cipher\n", __func__);

			ctx->fallback_cipher =
			    crypto_alloc_aead(alg->cra_name, 0,
					      CRYPTO_ALG_ASYNC |
					      CRYPTO_ALG_NEED_FALLBACK);
			if (IS_ERR(ctx->fallback_cipher)) {
				pr_err("%s() Error: failed to allocate fallback for %s\n",
				     __func__, alg->cra_name);
				return PTR_ERR(ctx->fallback_cipher);
			}
		}
	}

	return err;
}

static void generic_cra_exit(struct crypto_tfm *tfm)
{
	atomic_dec(&iproc_priv.session_count);
}

static void aead_cra_exit(struct crypto_aead *aead)
{
	struct crypto_tfm *tfm = crypto_aead_tfm(aead);
	struct iproc_ctx_s *ctx = crypto_tfm_ctx(tfm);

	generic_cra_exit(tfm);

	if (ctx->fallback_cipher) {
		crypto_free_aead(ctx->fallback_cipher);
		ctx->fallback_cipher = NULL;
	}
}

/* ==================== Kernel Platform API ==================== */

static int spu_dt_validate(struct device *dev, struct spu_hw *spu)
{
	if (spu->max_pkt_size % HASH_BLOCK_SIZE) {
		dev_err(dev,
			"SPU max pkt size %u must be multiple of hash block size %u",
			spu->max_pkt_size, HASH_BLOCK_SIZE);
		return -EINVAL;
	}

	if (spu->max_pkt_size % AES_BLOCK_SIZE) {
		dev_err(dev,
			"SPU max pkt size %u must be multiple of cipher block size %u",
			spu->max_pkt_size, AES_BLOCK_SIZE);
		return -EINVAL;
	}

	return 0;
}

static int spu_dt_read(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spu_hw *spu = &iproc_priv.spu;
	struct device_node *dn = pdev->dev.of_node;
	struct resource *spu_ctrl_regs;
	void __iomem *spu_reg_vbase[MAX_SPUS];
	int i;
	u32 max_pkt_size;
	int err;

	if (!of_device_is_available(dn)) {
		dev_crit(dev, "SPU device not available");
		return -ENODEV;
	}

	/* Count number of mailbox channels */
	spu->num_chan = of_count_phandle_with_args(dn, "mboxes", "#mbox-cells");
	dev_dbg(dev, "Device has %d SPU channels", spu->num_chan);

	if (of_device_is_compatible(dn, "brcm,spum-crypto"))
		spu->spu_type = SPU_TYPE_SPUM;
	else if (of_device_is_compatible(dn, "brcm,spu2-crypto"))
		spu->spu_type = SPU_TYPE_SPU2;
	else {
		dev_err(dev, "Unknown SPU type");
		return -EINVAL;
	}

	/* Get max SPU message packet size */
	err = of_property_read_u32(dn, "brcm,max-pkt-size", &max_pkt_size);
	if (err < 0) {
		dev_err(dev,
			"%s failed to get max-pkt-size from device tree",
			__func__);
		return -EINVAL;
	}
	spu->max_pkt_size = max_pkt_size;
	dev_dbg(dev, "Maximum SPU pkt size %u bytes", spu->max_pkt_size);

	/* Read registers and count number of SPUs */
	i = 0;
	while ((i < MAX_SPUS) && ((spu_ctrl_regs =
		platform_get_resource(pdev, IORESOURCE_MEM, i)) != NULL)) {

		dev_dbg(dev,
			 "SPU %d control register region res.start = %#x, res.end = %#x",
			 i,
			 (unsigned int)spu_ctrl_regs->start,
			 (unsigned int)spu_ctrl_regs->end);

		spu_reg_vbase[i] = devm_ioremap_resource(dev, spu_ctrl_regs);
		if (IS_ERR(spu_reg_vbase[i])) {
			err = PTR_ERR(spu_reg_vbase[i]);
			dev_err(&pdev->dev, "Failed to map registers: %d\n",
				err);
			spu_reg_vbase[i] = NULL;
			return err;
		}
		i++;
	}
	spu->num_spu = i;
	dev_dbg(dev, "Device has %d SPUs", spu->num_spu);

	spu->reg_vbase = devm_kcalloc(dev, spu->num_spu,
				      sizeof(*(spu->reg_vbase)), GFP_KERNEL);
	if (spu->reg_vbase == NULL)
		return -ENOMEM;
	memcpy(spu->reg_vbase, spu_reg_vbase,
	       spu->num_spu * sizeof(*(spu->reg_vbase)));

	return spu_dt_validate(dev, spu);
}

/*
 * Specify hardware-specific SPU functions based on SPU type read from device
 * tree.
 */
static void spu_functions_register(struct device *dev,
				   enum spu_spu_type spu_type)
{
	struct spu_hw *spu = &iproc_priv.spu;

	if (spu_type == SPU_TYPE_SPUM) {
		dev_dbg(dev, "Registering SPUM functions");
		spu->spu_dump_msg_hdr = spum_dump_msg_hdr;
		spu->spu_payload_length = spum_payload_length;
		spu->spu_response_hdr_len = spum_response_hdr_len;
		spu->spu_hash_pad_len = spum_hash_pad_len;
		spu->spu_gcm_pad_len = spum_gcm_pad_len;
		spu->spu_assoc_resp_len = spum_assoc_resp_len;
		spu->spu_aead_ivlen = spum_aead_ivlen;
		spu->spu_hash_type = spum_hash_type;
		spu->spu_digest_size = spum_digest_size;
		spu->spu_create_request = spum_create_request;
		spu->spu_cipher_req_init = spum_cipher_req_init;
		spu->spu_cipher_req_finish = spum_cipher_req_finish;
		spu->spu_request_pad = spum_request_pad;
		spu->spu_tx_status_len = spum_tx_status_len;
		spu->spu_rx_status_len = spum_rx_status_len;
		spu->spu_status_process = spum_status_process;

	} else if (spu_type == SPU_TYPE_SPU2) {
		dev_dbg(dev, "Registering SPU2 functions");
		spu->spu_dump_msg_hdr = spu2_dump_msg_hdr;
		spu->spu_payload_length = spu2_payload_length;
		spu->spu_response_hdr_len = spu2_response_hdr_len;
		spu->spu_hash_pad_len = spu2_hash_pad_len;
		spu->spu_gcm_pad_len = spu2_gcm_pad_len;
		spu->spu_assoc_resp_len = spu2_assoc_resp_len;
		spu->spu_aead_ivlen = spu2_aead_ivlen;
		spu->spu_hash_type = spu2_hash_type;
		spu->spu_digest_size = spu2_digest_size;
		spu->spu_create_request = spu2_create_request;
		spu->spu_cipher_req_init = spu2_cipher_req_init;
		spu->spu_cipher_req_finish = spu2_cipher_req_finish;
		spu->spu_request_pad = spu2_request_pad;
		spu->spu_tx_status_len = spu2_tx_status_len;
		spu->spu_rx_status_len = spu2_rx_status_len;
		spu->spu_status_process = spu2_status_process;
	}
}

/*
 * Initialize mailbox client. This client waits for the framework to
 * confirm the outcome of each tx. Request ownership of each mailbox
 * channel in the device tree.
 */
static int spu_mb_init(struct device *dev)
{
	int i;
	struct mbox_client *mcl = &iproc_priv.mcl;
	int err;

	iproc_priv.mbox = kcalloc(iproc_priv.spu.num_chan,
				  sizeof(struct mbox_chan *), GFP_KERNEL);
	if (iproc_priv.mbox == NULL)
		return -ENOMEM;

	mcl->dev = dev;
	mcl->tx_block = true;
	mcl->tx_tout = 10000;	/* ms */
	mcl->knows_txdone = false;
	mcl->rx_callback = spu_rx_callback;
	mcl->tx_done = NULL;

	for (i = 0; i < iproc_priv.spu.num_chan; i++) {
		iproc_priv.mbox[i] = mbox_request_channel(mcl, i);
		if (IS_ERR(iproc_priv.mbox[i])) {
			err = (int)PTR_ERR(iproc_priv.mbox[i]);
			dev_err(dev,
				"Mbox channel %d request failed with err %d",
				i, err);
			iproc_priv.mbox[i] = NULL;
			return err;
		}
	}

	return 0;
}

static void spu_mb_release(struct platform_device *pdev)
{
	int i;

	if (!iproc_priv.mbox)
		return;

	for (i = 0; i < iproc_priv.spu.num_chan; i++)
		mbox_free_channel(iproc_priv.mbox[i]);

	kfree(iproc_priv.mbox);
	iproc_priv.mbox = NULL;
}

static int spu_register_ablkcipher(struct iproc_alg_s *driver_alg)
{
	struct spu_hw *spu = &iproc_priv.spu;
	struct crypto_alg *crypto = &driver_alg->alg.crypto;
	int err;

	/* SPU2 does not support RC4 */
	if ((driver_alg->cipher_info.alg == CIPHER_ALG_RC4) &&
	    (spu->spu_type == SPU_TYPE_SPU2))
		return 0;

	crypto->cra_module = THIS_MODULE;
	/* Higher value is higher priority. The newer software
	 * implementation for CONFIG_ARM64_CRYPTO registers
	 * with priority 300. So use priority 400 to prefer hw
	 * to sw.
	 */
	crypto->cra_priority = 400;
	crypto->cra_alignmask = 0;
	crypto->cra_ctxsize = sizeof(struct iproc_ctx_s);
	INIT_LIST_HEAD(&crypto->cra_list);

	crypto->cra_init = ablkcipher_cra_init;
	crypto->cra_exit = generic_cra_exit;
	crypto->cra_type = &crypto_ablkcipher_type;
	crypto->cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_KERN_DRIVER_ONLY;

	crypto->cra_ablkcipher.setkey = ablkcipher_setkey;
	crypto->cra_ablkcipher.encrypt = ablkcipher_encrypt;
	crypto->cra_ablkcipher.decrypt = ablkcipher_decrypt;

	err = crypto_register_alg(crypto);
	pr_info("  registered ablkcipher %s\n", crypto->cra_driver_name);
	return err;
}

static int spu_register_ahash(struct iproc_alg_s *driver_alg)
{
	struct ahash_alg *hash = &driver_alg->alg.hash;
	int err;

	hash->halg.base.cra_module = THIS_MODULE;
	hash->halg.base.cra_priority = 400;
	hash->halg.base.cra_alignmask = 0;
	hash->halg.base.cra_ctxsize = sizeof(struct iproc_ctx_s);
	hash->halg.base.cra_init = ahash_cra_init;
	hash->halg.base.cra_exit = generic_cra_exit;
	hash->halg.base.cra_type = &crypto_ahash_type;
	hash->halg.base.cra_flags = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC;
	/* Must be non-zero even though we don't use export()/import() */
	hash->halg.statesize = 1;

	if (driver_alg->auth_info.mode != HASH_MODE_HMAC) {
		hash->init = ahash_init;
		hash->update = ahash_update;
		hash->final = ahash_final;
		hash->finup = ahash_finup;
		hash->digest = ahash_digest;
	} else {
		hash->setkey = ahash_hmac_setkey;
		hash->init = ahash_hmac_init;
		hash->update = ahash_hmac_update;
		hash->final = ahash_hmac_final;
		hash->finup = ahash_hmac_finup;
		hash->digest = ahash_hmac_digest;
	}

	err = crypto_register_ahash(hash);
	pr_info("  registered ahash %s\n", hash->halg.base.cra_driver_name);
	return err;
}

static int spu_register_aead(struct iproc_alg_s *driver_alg)
{
	struct aead_alg *aead = &driver_alg->alg.aead;
	int err;

	aead->base.cra_module = THIS_MODULE;
	aead->base.cra_priority = 1500;
	aead->base.cra_alignmask = 0;
	aead->base.cra_ctxsize = sizeof(struct iproc_ctx_s);
	INIT_LIST_HEAD(&aead->base.cra_list);

	aead->base.cra_flags |= CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC;
	if (driver_alg->cipher_info.mode == CIPHER_MODE_GCM)
		aead->setkey = aead_gcm_setkey;
	else
		aead->setkey = aead_authenc_setkey;
	aead->setauthsize = aead_setauthsize;
	aead->encrypt = aead_encrypt;
	aead->decrypt = aead_decrypt;
	aead->init = aead_cra_init;
	aead->exit = aead_cra_exit;

	err = crypto_register_aead(aead);
	pr_info("  registered aead %s\n", aead->base.cra_driver_name);
	return err;
}

static int spu_algs_register(struct device *dev)
{
	struct spu_hw *spu = &iproc_priv.spu;
	int i, j;
	int err;

		/* register crypto algorithms the device supports */
	for (i = 0; i < ARRAY_SIZE(driver_algs); i++) {

		driver_algs[i].max_payload += spu->max_pkt_size;

		switch (driver_algs[i].type) {
		case CRYPTO_ALG_TYPE_ABLKCIPHER:
			err = spu_register_ablkcipher(&driver_algs[i]);
			break;
		case CRYPTO_ALG_TYPE_AHASH:
			err = spu_register_ahash(&driver_algs[i]);
			break;
		case CRYPTO_ALG_TYPE_AEAD:
			err = spu_register_aead(&driver_algs[i]);
			break;
		default:
			dev_err(dev,
				"iproc-crypto: unknown alg type: %d",
				driver_algs[i].type);
			err = -EINVAL;
		}

		if (err) {
			dev_err(dev, "alg registration failed with error %d\n",
				err);
			goto err_algs;
		}
	}

	return 0;

err_algs:
	for (j = 0; j < i; j++) {
		switch (driver_algs[j].type) {
		case CRYPTO_ALG_TYPE_ABLKCIPHER:
			crypto_unregister_alg(&driver_algs[j].alg.crypto);
			break;
		case CRYPTO_ALG_TYPE_AHASH:
			crypto_unregister_ahash(&driver_algs[j].alg.hash);
			break;
		case CRYPTO_ALG_TYPE_AEAD:
			crypto_unregister_aead(&driver_algs[j].alg.aead);
			break;
		}
	}
	return err;
}

int bcm_spu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spu_hw *spu = &iproc_priv.spu;
	int err = 0;
	int i;

	iproc_priv.pdev = pdev;
	platform_set_drvdata(iproc_priv.pdev, &iproc_priv);

	err = spu_dt_read(pdev);
	if (err < 0)
		goto failure;

	if (spu->spu_type == SPU_TYPE_SPUM)
		iproc_priv.bcm_hdr_len = 8;
	else if (spu->spu_type == SPU_TYPE_SPU2)
		iproc_priv.bcm_hdr_len = 0;

	spu_functions_register(&pdev->dev, spu->spu_type);

	err = spu_mb_init(&pdev->dev);
	if (err < 0)
		goto failure;

	atomic_set(&iproc_priv.session_count, 0);
	atomic_set(&iproc_priv.stream_count, 0);
	atomic_set(&iproc_priv.next_chan, (int)spu->num_chan);
	atomic64_set(&iproc_priv.bytes_in, 0);
	atomic64_set(&iproc_priv.bytes_out, 0);
	for (i = 0; i < SPU_OP_NUM; i++) {
		atomic_set(&iproc_priv.op_counts[i], 0);
		atomic_set(&iproc_priv.setkey_cnt[i], 0);
	}
	atomic_set(&iproc_priv.mb_send_fail, 0);
	atomic_set(&iproc_priv.bad_icv, 0);

	spu_setup_debugfs();

	err = spu_algs_register(dev);
	if (err < 0)
		goto fail_reg;

	return 0;

fail_reg:
	spu_free_debugfs_stats();
	spu_free_debugfs();
failure:
	spu_mb_release(pdev);
	dev_err(dev, "%s failed with error %d.\n", __func__, err);

	return err;
}

int bcm_spu_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(driver_algs); i++) {
		switch (driver_algs[i].type) {
		case CRYPTO_ALG_TYPE_ABLKCIPHER:
			crypto_unregister_alg(&driver_algs[i].alg.crypto);
			pr_info("  unregistered cipher %s\n",
				driver_algs[i].alg.crypto.cra_driver_name);
			break;
		case CRYPTO_ALG_TYPE_AHASH:
			crypto_unregister_ahash(&driver_algs[i].alg.hash);
			pr_info("  unregistered hash %s\n",
				driver_algs[i].alg.hash.halg.
				base.cra_driver_name);
			break;
		case CRYPTO_ALG_TYPE_AEAD:
			crypto_unregister_aead(&driver_algs[i].alg.aead);
			pr_info("  unregistered aead %s\n",
				driver_algs[i].alg.aead.base.cra_driver_name);
			break;
		}
	}
	spu_free_debugfs_stats();
	spu_free_debugfs();
	spu_mb_release(pdev);
	return 0;
}

/* ===== Kernel Module API ===== */

static const struct of_device_id bcm_spu_dt_ids[] = {
	{.compatible = "brcm,spum-crypto"},
	{.compatible = "brcm,spu2-crypto"},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, bcm_spu_dt_ids);

static struct platform_driver bcm_spu_pdriver = {
	.driver = {
		   .name = "brcm-spu-crypto",
		   .of_match_table = of_match_ptr(bcm_spu_dt_ids),
		   },
	.probe = bcm_spu_probe,
	.remove = bcm_spu_remove,
};
module_platform_driver(bcm_spu_pdriver);

MODULE_AUTHOR("Rob Rice <rob.rice@broadcom.com>");
MODULE_DESCRIPTION("Broadcom symmetric crypto offload driver");
MODULE_LICENSE("GPL v2");
