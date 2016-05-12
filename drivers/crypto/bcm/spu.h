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

/*
 * This file contains the definition of SPU messages. There are currently two
 * SPU message formats: SPU-M and SPU2. The hardware uses different values to
 * identify the same things in SPU-M vs SPU2. So this file defines values that
 * are hardware independent. Software can use these values for any version of
 * SPU hardware. These values are used in APIs in spu.c. Functions internal to
 * spu.c and spu2.c convert these to hardware-specific values.
 */

#ifndef _SPU_H
#define _SPU_H

#include <linux/types.h>
#include <linux/scatterlist.h>
#include <crypto/sha.h>

enum spu_cipher_alg {
	CIPHER_ALG_NONE = 0x0,
	CIPHER_ALG_RC4 = 0x1,
	CIPHER_ALG_DES = 0x2,
	CIPHER_ALG_3DES = 0x3,
	CIPHER_ALG_AES = 0x4,
};

enum spu_cipher_mode {
	CIPHER_MODE_NONE = 0x0,
	CIPHER_MODE_ECB = 0x0,
	CIPHER_MODE_CBC = 0x1,
	CIPHER_MODE_OFB = 0x2,
	CIPHER_MODE_CFB = 0x3,
	CIPHER_MODE_CTR = 0x4,
	CIPHER_MODE_CCM = 0x5,
	CIPHER_MODE_GCM = 0x6,
	CIPHER_MODE_XTS = 0x7,
};

enum spu_cipher_type {
	CIPHER_TYPE_NONE = 0x0,
	CIPHER_TYPE_DES = 0x0,
	CIPHER_TYPE_3DES = 0x0,
	CIPHER_TYPE_INIT = 0x0,	/* used for ARC4 */
	CIPHER_TYPE_AES128 = 0x0,
	CIPHER_TYPE_AES192 = 0x1,
	CIPHER_TYPE_UPDT = 0x1,	/* used for ARC4 */
	CIPHER_TYPE_AES256 = 0x2,
};

enum hash_alg {
	HASH_ALG_NONE = 0x0,
	HASH_ALG_MD5 = 0x1,
	HASH_ALG_SHA1 = 0x2,
	HASH_ALG_SHA224 = 0x3,
	HASH_ALG_SHA256 = 0x4,
	HASH_ALG_AES = 0x5,
};

enum hash_mode {
	HASH_MODE_NONE = 0x0,
	HASH_MODE_HASH = 0x0,
	HASH_MODE_XCBC = 0x0,
	HASH_MODE_CMAC = 0x1,
	HASH_MODE_CTXT = 0x1,
	HASH_MODE_HMAC = 0x2,
	HASH_MODE_RABIN = 0x4,
	HASH_MODE_FHMAC = 0x6,
	HASH_MODE_CCM = 0x5,
	HASH_MODE_GCM = 0x6,
};

enum hash_type {
	HASH_TYPE_NONE = 0x0,
	HASH_TYPE_FULL = 0x0,
	HASH_TYPE_INIT = 0x1,
	HASH_TYPE_UPDT = 0x2,
	HASH_TYPE_FIN = 0x3,
	HASH_TYPE_AES128 = 0x0,
	HASH_TYPE_AES192 = 0x1,
	HASH_TYPE_AES256 = 0x2
};

struct spu_request_opts {
	bool is_inbound;
	bool auth_first;
	bool is_aead;
	bool dtls_aead;
	bool bd_suppress;
};

struct spu_cipher_parms {
	enum spu_cipher_alg  alg;
	enum spu_cipher_mode mode;
	enum spu_cipher_type type;
	u8                  *key_buf;
	u16                  key_len;
	u8                  *iv_buf;
	u16                  iv_len;
};

struct spu_hash_parms {
	enum hash_alg  alg;
	enum hash_mode mode;
	enum hash_type type;
	u8             digestsize;
	u8            *key_buf;
	u16            key_len;
	u16            prebuf_len;
	u16            hmac_offset;
	/* length of hash pad. signed, needs to handle roll-overs */
	int            pad_len;
};

struct spu_aead_parms {
	u32 assoc_size;
	u16 iv_len;      /* length of IV field between assoc data and data */
	u8  aad_pad_len; /* For AES GCM, length of padding after AAD */
	u8  gcm_pad_len; /* For AES GCM, length of padding after data */
};

/************** SPU sizes ***************/

#define SPU_RX_STATUS_LEN  4

/* Max length of padding for 4-byte alignment of STATUS field */
#define SPU_STAT_PAD_MAX  4

/* Max length of pad fragment. 4 is for 4-byte alignment of STATUS field */
#define SPU_PAD_LEN_MAX (SPU_GCM_ALIGN + HASH_BLOCK_SIZE + SPU_STAT_PAD_MAX)

/* GCM requires 16-byte alignment */
#define SPU_GCM_ALIGN 16

/* Length up SUPDT field in SPU response message for RC4 */
#define SPU_SUPDT_LEN 260

/* SPU status error codes. These used as common error codes across all
 * SPU variants.
 */
#define SPU_INVALID_ICV  1

/**
 * spu_req_incl_icv() - Return true if SPU request message should include the
 * ICV as a separate buffer.
 * @cipher_mode:  the cipher mode being requested
 * @is_encrypt:   true if encrypting. false if decrypting.
 *
 * Return:  true if ICV to be included as separate buffer
 */
static __always_inline  bool spu_req_incl_icv(enum spu_cipher_mode cipher_mode,
					      bool is_encrypt)
{
	if ((cipher_mode == CIPHER_MODE_GCM) && !is_encrypt)
		return true;
	else
		return false;
}

static __always_inline u32 spu_real_db_size(u32 assoc_size,
					    u32 aead_iv_buf_len,
					    u32 prebuf_len,
					    u32 data_size,
					    u32 aad_pad_len,
					    u32 gcm_pad_len,
					    u32 hash_pad_len)
{
	return assoc_size + aead_iv_buf_len + prebuf_len + data_size +
	    aad_pad_len + gcm_pad_len + hash_pad_len;
}

/**
 * spu_status_padlen() - Given the length of the DB field and padding in a SPU
 * request message, determine the padding required to align the STATUS word on a
 * 4-byte boundary.
 * @db_size: length of DB field in bytes
 *
 * Return: length of status field padding, in bytes
 */
static __always_inline u32 spu_status_padlen(u32 db_size)
{
	return ((db_size + 3) & ~3) - db_size;
}

/************** SPU Functions Prototypes **************/

void spum_dump_msg_hdr(u8 *buf, unsigned buf_len);

u32 spum_payload_length(u8 *spu_hdr);
u16 spum_response_hdr_len(u16 auth_key_len, u16 enc_key_len, bool is_hash);
u16 spum_hash_pad_len(u32 chunksize, u16 hash_block_size);
u32 spum_gcm_pad_len(enum spu_cipher_mode cipher_mode, unsigned data_size);
u32 spum_assoc_resp_len(enum spu_cipher_mode cipher_mode, bool dtls_hmac,
			unsigned assoc_len, unsigned iv_len);
u8 spum_aead_ivlen(enum spu_cipher_mode cipher_mode, bool dtls_hmac,
		   u16 iv_len);
bool spu_req_incl_icv(enum spu_cipher_mode cipher_mode, bool is_encrypt);
enum hash_type spum_hash_type(u32 src_sent);
u32 spum_digest_size(u32 alg_digest_size, enum hash_alg alg,
		     enum hash_type htype);

u32 spum_create_request(u8 *spu_hdr,
			struct spu_request_opts *req_opts,
			struct spu_cipher_parms *cipher_parms,
			struct spu_hash_parms *hash_parms,
			struct spu_aead_parms *aead_parms,
			unsigned data_size);

u16 spum_cipher_req_init(u8 *spu_hdr, struct spu_cipher_parms *cipher_parms);

void spum_cipher_req_finish(u8 *spu_hdr,
			    u16 spu_req_hdr_len,
			    unsigned isInbound,
			    struct spu_cipher_parms *cipher_parms,
			    bool update_key,
			    unsigned data_size);

void spum_request_pad(u8 *pad_start,
		      u32 gcm_padding,
		      u32 hash_pad_len,
		      enum hash_alg auth_alg,
		      unsigned total_sent, u32 status_padding);

u8 spum_tx_status_len(void);
u8 spum_rx_status_len(void);
int spum_status_process(u8 *statp);

int rabintag_to_hash_index(unsigned char *tag);
#endif
