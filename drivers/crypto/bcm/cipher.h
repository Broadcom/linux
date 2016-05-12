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

#ifndef _CIPHER_H
#define _CIPHER_H

#include <linux/atomic.h>
#include <linux/mailbox/brcm-message.h>
#include <linux/mailbox_client.h>
#include <crypto/aes.h>
#include <crypto/internal/hash.h>
#include <crypto/aead.h>
#include <crypto/sha.h>
#include <crypto/rabin.h>

#include "spu.h"
#include "spum.h"
#include "spu2.h"

#define ARC4_MIN_KEY_SIZE   1
#define ARC4_MAX_KEY_SIZE   256
#define ARC4_BLOCK_SIZE     1
#define ARC4_STATE_SIZE     4

#define MAX_KEY_SIZE	ARC4_MAX_KEY_SIZE
#define MAX_IV_SIZE	AES_BLOCK_SIZE
#define MAX_DIGEST_SIZE	SHA256_DIGEST_SIZE
#define MAX_ASSOC_SIZE	512

/* MD5, SHA1, SHA224, SHA256 all have the same block size of 64 bytes */
#define HASH_BLOCK_SIZE 64

/* Maximum number of bytes from a non-final hash request that can
 * be deferred until more data is available.
 */
#define HASH_CARRY_MAX  2048

/* Force at least 4-byte alignment of all SPU message fields */
#define SPU_MSG_ALIGN  4

/* op_counts[] indexes */
enum op_type {
	SPU_OP_CIPHER,
	SPU_OP_HASH,
	SPU_OP_HMAC,
	SPU_OP_AEAD,
	SPU_OP_NUM
};

enum spu_spu_type {
	SPU_TYPE_SPUM,
	SPU_TYPE_SPU2
};

struct cipher_op {
	enum spu_cipher_alg alg;
	enum spu_cipher_mode mode;
};

struct auth_op {
	enum hash_alg alg;
	enum hash_mode mode;
};

struct iproc_alg_s {
	u32 type;
	union {
		struct crypto_alg crypto;
		struct ahash_alg hash;
		struct aead_alg aead;
		struct rabin_alg rabin;
	} alg;
	struct cipher_op cipher_info;
	struct auth_op auth_info;
	bool auth_first;
	bool dtls_hmac;

	/* Has to be signed, because alg initialization sets to negative value
	 * the amount to reduce the device max packet size by.
	 */
	s32 max_payload;
};

/* Buffers for a SPU request/reply message pair. All part of one structure to
 * allow a single alloc per request.
 */
struct spu_msg_buf {
	/* Request message fragments */

	/* SPU request message header. For SPU-M, holds MH, EMH, SCTX, BDESC,
	 * and BD header. For SPU2, holds FMD, OMD.
	 */
	u8 bcm_spu_req_hdr[ALIGN(SPU2_HEADER_ALLOC_LEN, SPU_MSG_ALIGN)];

	/* Hash digest. request and response. */
	u8 digest[ALIGN(MAX_DIGEST_SIZE, SPU_MSG_ALIGN)];

	/* SPU request message padding */
	u8 spu_req_pad[ALIGN(SPU_PAD_LEN_MAX, SPU_MSG_ALIGN)];

	/* SPU-M request message STATUS field */
	u8 tx_stat[ALIGN(SPU_TX_STATUS_LEN, SPU_MSG_ALIGN)];

	/* Response message fragments */

	/* SPU response message header */
	u8 spu_resp_hdr[ALIGN(SPU2_HEADER_ALLOC_LEN, SPU_MSG_ALIGN)];

	/* SPU response message STATUS field padding */
	u8 rx_stat_pad[ALIGN(SPU_STAT_PAD_MAX, SPU_MSG_ALIGN)];

	/* SPU response message STATUS field */
	u8 rx_stat[ALIGN(SPU_RX_STATUS_LEN, SPU_MSG_ALIGN)];

	union {
		/* Buffers only used for ablkcipher */
		struct {
			/* SPU RC4 SUPDT field */
			u8 supdt[ALIGN(SPU_SUPDT_LEN, SPU_MSG_ALIGN)];
		} c;

		/* Buffers only used for aead */
		struct {
			/* SPU response pad for GCM data */
			u8 gcmpad[ALIGN(AES_BLOCK_SIZE, SPU_MSG_ALIGN)];

			/* SPU request msg padding for GCM AAD */
			u8 req_aad_pad[ALIGN(SPU_PAD_LEN_MAX, SPU_MSG_ALIGN)];

			/* SPU response data to be discarded */
			u8 resp_aad[ALIGN(MAX_ASSOC_SIZE + MAX_IV_SIZE,
					  SPU_MSG_ALIGN)];
		} a;
	};
};

struct iproc_ctx_s {
	u8 enckey[MAX_KEY_SIZE + ARC4_STATE_SIZE];
	unsigned int enckeylen;

	u8 authkey[MAX_KEY_SIZE + ARC4_STATE_SIZE];
	unsigned int authkeylen;

	u8 iv[MAX_IV_SIZE];

	unsigned int digestsize;

	struct iproc_alg_s *alg;

	struct cipher_op cipher;
	enum spu_cipher_type cipher_type;

	struct auth_op auth;
	bool auth_first;
	unsigned max_payload;

	struct crypto_aead *fallback_cipher;

	/* auth_type is determined during processing of request */

	u8 ipad[HASH_BLOCK_SIZE];
	u8 opad[HASH_BLOCK_SIZE];

	/* Buffer to hold SPU message header template. Template is created at
	 * setkey time for ablkcipher requests, since most of the fields in the
	 * header are known at that time. At request time, just fill in a few
	 * missing pieces related to length of data in the request and IVs, etc.
	 */
	u8 bcm_spu_req_hdr[ALIGN(SPU2_HEADER_ALLOC_LEN, SPU_MSG_ALIGN)];

	/* Length of SPU request header */
	u16 spu_req_hdr_len;

	/* Expected length of SPU response header */
	u16 spu_resp_hdr_len;
};

struct iproc_reqctx_s {
	/* general context */
	struct crypto_async_request *parent;

	/* only valid after enqueue() */
	struct iproc_ctx_s *ctx;

	/* Set of buffers used as SPU message fragments */
	struct spu_msg_buf *msg_buf;

	u8 chan_idx;   /* Mailbox channel to be used to submit this request */

	/* total todo, rx'd, and sent for this request */
	unsigned total_todo;
	unsigned total_received;	/* only valid for ablkcipher */
	unsigned total_sent;

	/* this can differ from total_sent for hashes due to nbuf carried */
	/* from the previous req if the src wasn't % BLOCK_SIZE */
	unsigned src_sent;
	unsigned hmac_offset;

	/* For AEAD requests, start of associated data. This will typically
	 * point to the beginning of the src scatterlist from the request,
	 * since assoc data is at the beginning of the src scatterlist rather
	 * than in its own sg.
	 */
	struct scatterlist *assoc;

	/* scatterlist entry and offset to start of data for next chunk. Crypto
	 * API src scatterlist for AEAD starts with AAD, if present. For first
	 * chunk, src_sg is sg entry at beginning of input data (after AAD).
	 * src_skip begins at the offset in that sg entry where data begins.
	 */
	struct scatterlist *src_sg;
	int src_nents;		/* Number of src entries with data */
	u32 src_skip;		/* bytes of current sg entry already used */

	/* Same for destination. For AEAD, if there is AAD, output data must
	 * be written at offset following AAD.
	 */
	struct scatterlist *dst_sg;
	int dst_nents;		/* Number of dst entries with data */
	u32 dst_skip;		/* bytes of current sg entry already written */

	/* Mailbox message used to send this request to PDC driver */
	struct brcm_message mb_mssg;

	bool bd_suppress;	/* suppress BD field in SPU response? */

	/* cipher context */
	bool is_encrypt;

	/* CBC mode: IV.  CTR mode: counter.  Else empty. Used as a DMA
	 * buffer for AEAD requests. So allocate as DMAable memory.
	 */
	u8 *iv_ctr;
	/* = block_size if either an IV or CTR is present, else 0 */
	unsigned iv_ctr_len;

	/* Hash requests can be of any size, whether initial, update, or final.
	 * A non-final request must be submitted to the SPU as an integral
	 * number of blocks. This may leave data at the end of the request
	 * that is not a full block. We could submit this remainder as its
	 * own small SPU request message, but doing so would be inefficient.
	 * So, we write the remainder to this hash_carry buffer and hold it
	 * until the next request arrives. The carry data is then submitted
	 * at the beginning of the data in the next SPU msg. hash_carry_len
	 * is the number of bytes currently in hash_carry. These fields are
	 * only used for ahash requests.
	 */
	u8 hash_carry[HASH_CARRY_MAX];
	unsigned hash_carry_len;
	unsigned is_final;	/* is this the final for the hash op? */

	/* hmac context */
	bool is_sw_hmac;

	/* aead context */
	struct crypto_tfm *old_tfm;
	crypto_completion_t old_complete;
	void *old_data;
	u32 rabin_tag_idx;
	char rabin_tag[CRYPTO_MAX_ALG_NAME];
};

/*
 * Structure encapsulates a set of function pointers specific to the type of
 * SPU hardware running. These functions handling creation and parsing of
 * SPU request messages and SPU response messages. Includes hardware-specific
 * values read from device tree.
 */
struct spu_hw {
	void (*spu_dump_msg_hdr)(u8 *buf, unsigned buf_len);
	u32 (*spu_payload_length)(u8 *spu_hdr);
	u16 (*spu_response_hdr_len)(u16 auth_key_len, u16 enc_key_len,
				    bool is_hash);
	u16 (*spu_hash_pad_len)(u32 chunksize, u16 hash_block_size);
	u32 (*spu_gcm_pad_len)(enum spu_cipher_mode cipher_mode,
			       unsigned data_size);
	u32 (*spu_assoc_resp_len)(enum spu_cipher_mode cipher_mode,
				  bool dtls_hmac, unsigned assoc_len,
				  unsigned iv_len);
	u8 (*spu_aead_ivlen)(enum spu_cipher_mode cipher_mode, bool dtls_hmac,
			     u16 iv_len);
	enum hash_type (*spu_hash_type)(u32 src_sent);
	u32 (*spu_digest_size)(u32 digest_size, enum hash_alg alg,
			       enum hash_type);
	u32 (*spu_create_request)(u8 *spu_hdr,
				  struct spu_request_opts *req_opts,
				  struct spu_cipher_parms *cipher_parms,
				  struct spu_hash_parms *hash_parms,
				  struct spu_aead_parms *aead_parms,
				  unsigned data_size);
	u16 (*spu_cipher_req_init)(u8 *spu_hdr,
				   struct spu_cipher_parms *cipher_parms);
	void (*spu_cipher_req_finish)(u8 *spu_hdr,
				      u16 spu_req_hdr_len,
				      unsigned is_inbound,
				      struct spu_cipher_parms *cipher_parms,
				      bool update_key,
				      unsigned data_size);
	void (*spu_request_pad)(u8 *pad_start, u32 gcm_padding,
				u32 hash_pad_len,
				enum hash_alg auth_alg, unsigned total_sent,
				u32 status_padding);
	u8 (*spu_tx_status_len)(void);
	u8 (*spu_rx_status_len)(void);
	int (*spu_status_process)(u8 *statp);

	/* The base virtual address of the SPU hw registers */
	void __iomem **reg_vbase;

	/* Version of the SPU hardware */
	enum spu_spu_type spu_type;

	/* The number of SPUs on this platform */
	u32 num_spu;

	/* The number of SPU channels on this platform */
	u32 num_chan;

	/*
	 * The max pkt size must be a multiple of block size, so that when
	 * we break a request into chunks, the chunks are a multiple
	 * of block size. We always check that message sizes are strictly less
	 * than the max pkt size.
	 */
	u32 max_pkt_size;
};

struct device_private {
	struct platform_device *pdev;

	struct spu_hw spu;

	atomic_t session_count;	/* number of streams active */
	atomic_t stream_count;	/* monotonic counter for streamID's */

	/* Length of BCM header. Set to 0 when hw does not expect BCM HEADER. */
	u8 bcm_hdr_len;

	/* The index of the channel to use for the next crypto request */
	atomic_t next_chan;

	struct dentry *debugfs_dir;
	struct dentry *debugfs_stats;

	/* Number of request bytes processed and result bytes returned */
	atomic64_t bytes_in;
	atomic64_t bytes_out;

	/* Number of operations of each type */
	atomic_t op_counts[SPU_OP_NUM];

	/* Number of calls to setkey() for each operation type */
	atomic_t setkey_cnt[SPU_OP_NUM];

	/* Number of mailbox send failures */
	atomic_t mb_send_fail;

	/* Number of ICV check failures for AEAD messages */
	atomic_t bad_icv;

	struct mbox_client mcl;
	/* Array of mailbox channel pointers, one for each channel */
	struct mbox_chan **mbox;
	struct rfp_desc rfp_desc;
};

extern struct device_private iproc_priv;

#endif
