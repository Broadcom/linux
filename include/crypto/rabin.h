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

#ifndef _CRYPTO_RABIN_H
#define _CRYPTO_RABIN_H

#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <crypto/sha.h>
#include <crypto/md5.h>

struct fp_desc {
	unsigned int size; /* size of chunk */
	unsigned int offset; /* starting offset of the chunk in input stream */
	unsigned int fp; /* finger print for this chunk */
	unsigned short status; /* identify the validity status of this chunk */
} __attribute__((__packed__));

struct rfp_desc {
	unsigned int rabin_lut[256]; /* LUT table*/
	unsigned int win_sz; /* window size which moves over input data */
	unsigned int polynomial; /* polynomial used */
	unsigned int min_chunk_sz;
	unsigned int max_chunk_sz;
	unsigned int tfp[4]; /* finger print termination value */
	unsigned int tfp_mask[4]; /* finger print termination mask */
	unsigned int tfp_bitmask; /* indicating valid termination cond*/
#define FLAG_RABIN_WIN_SZ	BIT(0)
#define FLAG_RABIN_POLY		BIT(1)
#define FLAG_RABIN_CHUNK_SZ	BIT(2)
#define FLAG_RABIN_LUT_DONE	BIT(3)
	unsigned int rabin_flag; /* tracking status of RFP cycle */
};

struct rabin_request {
	struct crypto_async_request base;

	/* sg list holding the input data stream */
	struct scatterlist *src_sg;
	/* sg list to hold output of fp descs along with tag*/
	struct scatterlist *dst_sg;

	/* tag type that need to be used along with finger print */
	unsigned char *tag;

	void *__ctx[] CRYPTO_MINALIGN_ATTR;
};

static inline struct rabin_request *rabin_request_cast(
	struct crypto_async_request *req)
{
	return container_of(req, struct rabin_request, base);
}

static inline void *rabin_request_ctx(struct rabin_request *req)
{
	return req->__ctx;
}

struct crypto_rabin {

	int (*set_chunk_size)(struct crypto_rabin *tfm, u32 minchunk,
								u32 maxchunk);
	int (*set_polynomial)(struct crypto_rabin *tfm, u32 polynomial);
	int (*set_win_size)(struct crypto_rabin *tfm, u32 winsize);
	int (*set_fp_termination)(struct crypto_rabin *tfm, u32 tfp,
								u32 tfp_mask);
	int (*get_finger_print)(struct rabin_request *req);

	int reqsize;
	struct crypto_tfm base;

};
static inline struct crypto_rabin *__crypto_rabin_cast(struct crypto_tfm *tfm)
{
	return container_of(tfm, struct crypto_rabin, base);
}

/**
 * crypto_rabin_reqtfm() - obtain cipher handle from request
 * @req: asynchronous request handle that contains the reference to the rabin
 *	 cipher handle
 *
 * Return the rabin cipher handle that is registered with the asynchronous
 * request handle rabin_request.
 *
 * Return: rabin cipher handle
 */
static inline struct crypto_rabin *crypto_rabin_reqtfm(
	struct rabin_request *req)
{
	return __crypto_rabin_cast(req->base.tfm);
}


struct rabin_alg {
	int (*set_chunk_size)(struct crypto_rabin *tfm, u32 minchunk,
								u32 maxchunk);
	int (*set_polynomial)(struct crypto_rabin *tfm, u32 polynomial);
	int (*set_win_size)(struct crypto_rabin *tfm, u32 winsize);
	int (*set_fp_termination)(struct crypto_rabin *tfm, u32 tfprint,
								u32 mask);
	int (*get_finger_print)(struct rabin_request *req);
	void (*exit)(struct crypto_rabin *tfm);
	int (*init)(struct crypto_rabin *tfm);

	struct crypto_alg base;

};

struct crypto_rabin *crypto_alloc_rabin(const char *alg_name,
						u32 type, u32 mask);

static inline struct crypto_tfm *crypto_rabin_tfm(struct crypto_rabin *tfm)
{
	return &tfm->base;
}

static inline void crypto_free_rabin(struct crypto_rabin *tfm)
{
	crypto_destroy_tfm(tfm, crypto_rabin_tfm(tfm));
}

/**
 * rabin_request_set_tfm() - update cipher handle reference in request
 * @req: request handle to be modified
 * @tfm: cipher handle that shall be added to the request handle
 *
 * Allow the caller to replace the existing rabin handle in the request
 * data structure with a different one.
 */
static inline void rabin_request_set_tfm(struct rabin_request *req,
					 struct crypto_rabin *tfm)
{
	req->base.tfm = crypto_rabin_tfm(tfm);
}

/**
 * crypto_rabin_reqsize() - obtain size of the request data structure
 * @tfm: cipher handle
 *
 * Return: number of bytes
 */
static inline unsigned int crypto_rabin_reqsize(struct crypto_rabin *tfm)
{
	return tfm->reqsize;
}


/**
 * rabin_request_alloc() - allocate request data structure
 * @tfm: cipher handle to be registered with the request
 * @gfp: memory allocation flag that is handed to kmalloc by the API call.
 *
 * Allocate the request data structure that must be used with the rabin
 * message digest API calls. During
 * the allocation, the provided rabin handle
 * is registered in the request data structure.
 *
 * Return: allocated request handle in case of success; IS_ERR() is true in case
 *	   of an error, PTR_ERR() returns the error code.
 */
static inline struct rabin_request *rabin_request_alloc(
	struct crypto_rabin *tfm, gfp_t gfp)
{
	struct rabin_request *req;

	req = kmalloc(sizeof(*req) + crypto_rabin_reqsize(tfm), gfp);

	if (likely(req))
		rabin_request_set_tfm(req, tfm);

	return req;
}

/**
 * rabin_request_free() - zeroize and free the request data structure
 * @req: request data structure cipher handle to be freed
 */
static inline void rabin_request_free(struct rabin_request *req)
{
	kzfree(req);
}


/**
 * rabin_request_set_callback() - set asynchronous callback function
 * @req: request handle
 * @flags: specify zero or an ORing of the flags
 *	   CRYPTO_TFM_REQ_MAY_BACKLOG the request queue may back log and
 *	   increase the wait queue beyond the initial maximum size;
 *	   CRYPTO_TFM_REQ_MAY_SLEEP the request processing may sleep
 * @compl: callback function pointer to be registered with the request handle
 * @data: The data pointer refers to memory that is not used by the kernel
 *	  crypto API, but provided to the callback function for it to use. Here,
 *	  the caller can provide a reference to memory the callback function can
 *	  operate on. As the callback function is invoked asynchronously to the
 *	  related functionality, it may need to access data structures of the
 *	  related functionality which can be referenced using this pointer. The
 *	  callback function can access the memory via the "data" field in the
 *	  &crypto_async_request data structure provided to the callback func.
 *
 * This function allows setting the callback function that is triggered once
 * the cipher operation completes.
 *
 * The callback function is registered with the &rabin_request handle and
 * must comply with the following template
 *
 *	void callback_function(struct crypto_async_request *req, int error)
 */
static inline void rabin_request_set_callback(struct rabin_request *req,
					      u32 flags,
					      crypto_completion_t compl,
					      void *data)
{
	req->base.complete = compl;
	req->base.data = data;
	req->base.flags = flags;
}

/* Setting the polynomial for RFP core */
int crypto_rabin_set_polynomial(struct crypto_rabin *tfm, u32 polynomial);

/* Sets window size for RFP core */
int crypto_rabin_set_win_size(struct crypto_rabin *tfm, u32 win_size);

/* Sets minimum and maximum chunk size for RFP core */
int crypto_rabin_set_chunk_size(struct crypto_rabin *tfm, u32 min_chunk_size,
							u32 max_chunk_size);

/*
 * used for setting termination value and mask for RFP core. RFP core
 * applies @tfp_mask on fp and compare it with this @tfp to figure out
 * termination condition.
 */
int crypto_rabin_set_termination(struct crypto_rabin *tfm, u32 tfp,
							u32 tfp_mask);

/* API to trigger the RFP core to generate the Finger Print descriptors list */
int crypto_rabin_get_finger_print(struct rabin_request *req);

/* used to get tag size. this depends on the hash used for tag */
int crypto_rabin_get_tag_size(u8 *tag);

/* to initialize LUT table */
void initialize_lut(u32 win_sz, u32 poly, u32 *lut);

#endif
