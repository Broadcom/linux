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
 * This file contains SPU message definitions specific to SPU-M.
 */

#ifndef _SPUM_H_
#define _SPUM_H_

#define SPU_CRYPTO_OPERATION_GENERIC	0x1

/* Length of STATUS field in tx and rx packets */
#define SPU_TX_STATUS_LEN  4

/* SPU-M error codes */
#define SPU_STATUS_MASK                 0x0000FF00
#define SPU_STATUS_SUCCESS              0x00000000
#define SPU_STATUS_INVALID_ICV          0x00000100

#define SPU_STATUS_ERROR_FLAG           0x00020000

/* Request message. MH + EMH + BDESC + BD header */
#define SPU_REQ_FIXED_LEN 24

/* Max length of a SPU message header. Used to allocate a buffer where
 * the SPU message header is constructed. Can be used for either a SPU-M
 * header or a SPU2 header.
 * For SPU-M, sum of the following:
 *    MH - 4 bytes
 *    EMH - 4
 *    SCTX - 3 +
 *      max auth key len - 64
 *      max cipher key len - 264 (RC4)
 *      max IV len - 16
 *    BDESC - 12
 *    BD header - 4
 * Total:  371
 *
 * For SPU2, FMD_SIZE (32) plus lengths of hash and cipher keys,
 * hash and cipher IVs. If SPU2 does not support RC4, then
 */
#define SPU_HEADER_ALLOC_LEN  (SPU_REQ_FIXED_LEN + MAX_KEY_SIZE + \
				MAX_KEY_SIZE + MAX_IV_SIZE)

/* Response message header length. Normally MH, EMH, BD header, but when
 * BD_SUPPRESS is used for hash requests, there is no BD header.
 */
#define SPU_RESP_HDR_LEN 12
#define SPU_HASH_RESP_HDR_LEN 8

/* Buffer Descriptor Header [BDESC]. SPU in big-endian mode. */
struct BDESC_HEADER {
	u16 offsetMAC;		/* word 0 [31-16] */
	u16 lengthMAC;		/* word 0 [15-0]  */
	u16 offsetCrypto;	/* word 1 [31-16] */
	u16 lengthCrypto;	/* word 1 [15-0]  */
	u16 offsetICV;		/* word 2 [31-16] */
	u16 offsetIV;		/* word 2 [15-0]  */
};

/* Buffer Data Header [BD]. SPU in big-endian mode. */
struct BD_HEADER {
	u16 size;
	u16 PrevLength;
};

/* Command Context Header. SPU-M in big endian mode. */
struct MHEADER {
	u8 flags;	/* [31:24] */
	u8 op_code;	/* [23:16] */
	u16 reserved;	/* [15:0] */
};

/* MH header flags bits */
#define MH_SUPDT_PRES   (1 << 0)
#define MH_HASH_PRES    (1 << 2)
#define MH_BD_PRES      (1 << 3)
#define MH_MFM_PRES     (1 << 4)
#define MH_BDESC_PRES   (1 << 5)
#define MH_SCTX_PRES	(1 << 7)

/* SCTX word 0 bit offsets and fields masks */
#define SCTX_SIZE               0x000000FF

/* SCTX word 1 bit shifts and field masks */
#define  UPDT_OFST              0x000000FF   /* offset of SCTX updateable fld */
#define  HASH_TYPE              0x00000300   /* hash alg operation type */
#define  HASH_TYPE_SHIFT                 8
#define  HASH_MODE              0x00001C00   /* one of spu2_hash_mode */
#define  HASH_MODE_SHIFT                10
#define  HASH_ALG               0x0000E000   /* hash algorithm */
#define  HASH_ALG_SHIFT                 13
#define  CIPHER_TYPE            0x00030000   /* encryption operation type */
#define  CIPHER_TYPE_SHIFT              16
#define  CIPHER_MODE            0x001C0000   /* encryption mode */
#define  CIPHER_MODE_SHIFT              18
#define  CIPHER_ALG             0x00E00000   /* encryption algo */
#define  CIPHER_ALG_SHIFT               21
#define  CIPHER_ORDER             (1 << 30)
#define  CIPHER_ORDER_SHIFT             30
#define  CIPHER_INBOUND           (1 << 31)
#define  CIPHER_INBOUND_SHIFT           31

/* SCTX word 2 bit shifts and field masks */
#define  EXP_IV_SIZE                   0x7
#define  IV_OFFSET                 (1 << 3)
#define  IV_OFFSET_SHIFT                 3
#define  GEN_IV                    (1 << 5)
#define  GEN_IV_SHIFT                    5
#define  EXPLICIT_IV               (1 << 6)
#define  EXPLICIT_IV_SHIFT               6
#define  SCTX_IV                   (1 << 7)
#define  SCTX_IV_SHIFT                   7
#define  ICV_SIZE                   0x0F00
#define  ICV_SIZE_SHIFT                  8
#define  CHECK_ICV                (1 << 12)
#define  CHECK_ICV_SHIFT                12
#define  INSERT_ICV               (1 << 13)
#define  INSERT_ICV_SHIFT               13
#define  BD_SUPPRESS              (1 << 19)
#define  BD_SUPPRESS_SHIFT              19

/* Generic Mode Security Context Structure [SCTX] */
struct SCTX {
/* word 0: protocol flags */
	u32 proto_flags;

/* word 1: cipher flags */
	u32 cipher_flags;

/* word 2: Extended cipher flags */
	u32 ecf;

};

struct SPUHEADER {
	struct MHEADER mh;
	u32 emh;
	struct SCTX sa;
};

/* Determine the length of GCM padding required. */
static __always_inline u32 spu_gcm_padlen(enum spu_cipher_mode cipher_mode,
					  unsigned data_size)
{
	u32 gcm_pad_len = 0;
	u32 m1 = SPU_GCM_ALIGN - 1;

	if (cipher_mode == CIPHER_MODE_GCM)
		gcm_pad_len = ((data_size + m1) & ~m1) - data_size;

	return gcm_pad_len;
}

#endif /* _SPUM_H_ */
