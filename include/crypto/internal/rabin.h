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

#ifndef _CRYPTO_INTERNAL_RABIN_H
#define _CRYPTO_INTERNAL_RABIN_H

#include <crypto/algapi.h>
#include <crypto/rabin.h>

int crypto_register_rabin(struct rabin_alg *alg);
int crypto_unregister_rabin(struct rabin_alg *alg);

extern const struct crypto_type crypto_rabin_type;

static inline void *crypto_rabin_ctx(struct crypto_rabin *tfm)
{
	return crypto_tfm_ctx(crypto_rabin_tfm(tfm));
}

static inline struct rabin_alg *__crypto_rabin_alg(struct crypto_alg *alg)
{
	return container_of(alg, struct rabin_alg, base);
}

static inline void crypto_rabin_set_reqsize(struct crypto_rabin *rabin,
					   unsigned int reqsize)
{
	rabin->reqsize = reqsize;
}
#endif
