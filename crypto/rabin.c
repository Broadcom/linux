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

#include <linux/crypto.h>
#include <crypto/rabin.h>
#include <crypto/algapi.h>
#include <crypto/hash.h>
#include "internal.h"

static inline struct rabin_alg *crypto_rabin_alg(struct crypto_rabin *tfm)
{
	return container_of(crypto_rabin_tfm(tfm)->__crt_alg, struct rabin_alg,
			    base);
}

static int crypto_rabin_init_tfm(struct crypto_tfm *tfm)
{
	struct crypto_rabin *rabin = __crypto_rabin_cast(tfm);
	struct rabin_alg *alg = crypto_rabin_alg(rabin);

	rabin->set_chunk_size = alg->set_chunk_size;
	rabin->set_polynomial = alg->set_polynomial;
	rabin->set_win_size = alg->set_win_size;
	rabin->set_fp_termination = alg->set_fp_termination;
	rabin->get_finger_print = alg->get_finger_print;

	if (alg->init)
		return alg->init(rabin);

	return 0;
}

const struct crypto_type crypto_rabin_type = {
	.extsize = crypto_alg_extsize,
	.init_tfm = crypto_rabin_init_tfm,
	.maskclear = ~CRYPTO_ALG_TYPE_MASK,
	.maskset = CRYPTO_ALG_TYPE_MASK,
	.type = CRYPTO_ALG_TYPE_RABIN,
	.tfmsize = offsetof(struct crypto_rabin, base),
};
EXPORT_SYMBOL_GPL(crypto_rabin_type);

struct crypto_rabin *crypto_alloc_rabin(const char *alg_name,
						  u32 type, u32 mask)
{
	return crypto_alloc_tfm(alg_name, &crypto_rabin_type, type, mask);
}
EXPORT_SYMBOL_GPL(crypto_alloc_rabin);

int crypto_register_rabin(struct rabin_alg *alg)
{
	struct crypto_alg *base = &alg->base;

	pr_info("%s: Register Rabin Finger print algorithm\n", __func__);
	return crypto_register_alg(base);
}
EXPORT_SYMBOL_GPL(crypto_register_rabin);

void crypto_unregister_rabin(struct rabin_alg *alg)
{
	crypto_unregister_alg(&alg->base);
}
EXPORT_SYMBOL_GPL(crypto_unregister_rabin);

int crypto_register_rabins(struct rabin_alg *algs, int count)
{
	int i, ret;

	for (i = 0; i < count; i++) {
		ret = crypto_register_rabin(&algs[i]);
		if (ret)
			goto err;
	}

	return 0;

err:
	for (--i; i >= 0; --i)
		crypto_unregister_rabin(&algs[i]);

	return ret;
}
EXPORT_SYMBOL_GPL(crypto_register_rabins);

void crypto_unregister_rabins(struct rabin_alg *algs, int count)
{
	int i;

	for (i = count - 1; i >= 0; --i)
		crypto_unregister_rabin(&algs[i]);
}
EXPORT_SYMBOL_GPL(crypto_unregister_rabins);

int crypto_rabin_set_polynomial(struct crypto_rabin *tfm, u32 polynomial)
{
	return tfm->set_polynomial(tfm, polynomial);
}
EXPORT_SYMBOL_GPL(crypto_rabin_set_polynomial);

int crypto_rabin_set_win_size(struct crypto_rabin *tfm, u32 win_size)
{
	return tfm->set_win_size(tfm, win_size);
}
EXPORT_SYMBOL_GPL(crypto_rabin_set_win_size);

int crypto_rabin_set_chunk_size(struct crypto_rabin *tfm, u32 min_chunk_size,
							u32 max_chunk_size)
{
	return tfm->set_chunk_size(tfm, min_chunk_size, max_chunk_size);
}
EXPORT_SYMBOL_GPL(crypto_rabin_set_chunk_size);

int crypto_rabin_set_termination(struct crypto_rabin *tfm, u32 tfp,
								u32 tfp_mask)
{
	return tfm->set_fp_termination(tfm, tfp, tfp_mask);
}
EXPORT_SYMBOL_GPL(crypto_rabin_set_termination);

int crypto_rabin_get_finger_print(struct rabin_request *req)
{
	struct crypto_rabin *rabin = crypto_rabin_reqtfm(req);

	return crypto_rabin_alg(rabin)->get_finger_print(req);
}
EXPORT_SYMBOL_GPL(crypto_rabin_get_finger_print);

int crypto_rabin_get_tag_size(unsigned char *tag)
{
	if (!memcmp(tag, "md5", strlen("md5")))
		return MD5_DIGEST_SIZE;
	else if (!memcmp(tag, "sha1", strlen("sha1")))
		return SHA1_DIGEST_SIZE;
	else if (!memcmp(tag, "sha224", strlen("sha224")))
		return SHA224_DIGEST_SIZE;
	else if (!memcmp(tag, "sha256", strlen("sha256")))
		return SHA256_DIGEST_SIZE;
	else if (!memcmp(tag, "sha384", strlen("sha384")))
		return SHA384_DIGEST_SIZE;
	else if (!memcmp(tag, "sha512", strlen("sha512")))
		return SHA512_DIGEST_SIZE;
	else if (!memcmp(tag, "sha512/224", strlen("sha512/224")))
		return SHA512_DIGEST_SIZE;
	else if (!memcmp(tag, "sha512/256", strlen("sha512/256")))
		return SHA512_DIGEST_SIZE;
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(crypto_rabin_get_tag_size);

static u32 append_byte(u32 fp, u8 byte, u32 polynomial)
{
	unsigned int i = 0;

	for (i = 0; i < 8; i++) {
		if (fp & BIT(31))
			fp = ((fp << 1) | (byte >> 7)) ^ polynomial;
		else
			fp = (fp << 1) | (byte >> 7);
		byte = byte << 1;
	}
	return fp;
}

void initialize_lut(u32 win_sz, u32 poly, u32 *lut)
{
	unsigned int i, j;
	unsigned int fp;

	for (i = 0; i < 256; i++) {
		fp = i;
		for (j = 1; j < win_sz; j++)
			fp = append_byte(fp, 0x0, poly);

		lut[i] = fp;
	}
}
EXPORT_SYMBOL_GPL(initialize_lut);
