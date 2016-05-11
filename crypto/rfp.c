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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/scatterlist.h>
#include <linux/bitops.h>
#include <crypto/internal/rabin.h>
#include <crypto/hash.h>

struct sdesc {
	struct shash_desc shash;
	char ctx[];
};

/* produce a message digest from data of length n bytes */
static int do_shash(unsigned char *name, unsigned char *result,
	     const u8 *data1, unsigned data1_len,
	     const u8 *data2, unsigned data2_len)
{
	int rc;
	unsigned size;
	struct crypto_shash *hash;
	struct sdesc *sdesc;

	hash = crypto_alloc_shash(name, 0, 0);
	if (IS_ERR(hash)) {
		rc = PTR_ERR(hash);
		pr_err("%s: Crypto %s allocation error %d", __func__, name, rc);
		return rc;
	}

	size = sizeof(struct shash_desc) + crypto_shash_descsize(hash);
	sdesc = kmalloc(size, GFP_KERNEL);
	if (!sdesc) {
		rc = -ENOMEM;
		pr_err("%s: Memory allocation failure", __func__);
		goto do_shash_err;
	}
	sdesc->shash.tfm = hash;
	sdesc->shash.flags = 0x0;

	rc = crypto_shash_init(&sdesc->shash);
	if (rc) {
		pr_err("%s: Could not init %s shash", __func__, name);
		goto do_shash_err;
	}
	rc = crypto_shash_update(&sdesc->shash, data1, data1_len);
	if (rc) {
		pr_err("%s: Could not update1", __func__);
		goto do_shash_err;
	}
	if (data2 && data2_len) {
		rc = crypto_shash_update(&sdesc->shash, data2, data2_len);
		if (rc) {
			pr_err("%s: Could not update2", __func__);
			goto do_shash_err;
		}
	}
	rc = crypto_shash_final(&sdesc->shash, result);
	if (rc)
		pr_err("%s: Could not genereate %s hash", __func__, name);

do_shash_err:
	crypto_free_shash(hash);
	kfree(sdesc);

	return rc;
}

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

int rabin_get_fps(struct scatterlist *src_sg, struct scatterlist *dst_sg,
						u8 *tag, struct rfp_desc *ctx)
{
	unsigned int *lut;
	struct scatterlist *ssg;
	struct fp_desc *fp_desc;
	u8 *chunkp, *rfcdp;
	u32 min_chunk_sz, chunk_sz, rfcd_sz, tag_sz;
	u32 i, tsrc_len, src_done, fp, widx, chunk_start_offset;
	u8 out, in;
	bool fp_termination;
	int ret = 0;
	unsigned char win[128] = {0,};

	tsrc_len = 0;
	for_each_sg(src_sg, ssg, sg_nents(src_sg), i)
		tsrc_len += ssg->length;

	tag_sz = crypto_rabin_get_tag_size(tag);
	pr_info("tsrc_len :%u, tag_size:%u\n", tsrc_len, tag_sz);

	lut = ctx->rabin_lut;
	chunk_start_offset = (ctx->win_sz < ctx->min_chunk_sz) ?
					(ctx->min_chunk_sz - ctx->win_sz) : 0;

	rfcd_sz = tag_sz + sizeof(*fp_desc);
	rfcdp = kzalloc(rfcd_sz, GFP_KERNEL);
	if (!rfcdp)
		return -ENOMEM;

	chunkp = kzalloc(ctx->max_chunk_sz, GFP_KERNEL);
	if (!chunkp) {
		ret = -ENOMEM;
		goto chk_sz_fail;
	}

	min_chunk_sz = min(ctx->max_chunk_sz, tsrc_len);
	ret = sg_pcopy_to_buffer(src_sg, sg_nents(src_sg),
						chunkp, min_chunk_sz, 0);
	if (ret != min_chunk_sz) {
		pr_info("failed to copy to buffer from 0th offset\n");
		ret = -EINVAL;
		goto out;
	}

	fp = 0;
	widx = 0;
	src_done = 0;
	chunk_sz = chunk_start_offset;
	for (i = chunk_start_offset; i < tsrc_len; i++) {
		/* window management */
		out = win[widx];
		in  = chunkp[chunk_sz];
		chunk_sz++;
		win[widx] = in;
		widx = (widx + 1) % ctx->win_sz;

		fp = append_byte(fp, in, ctx->polynomial);

		if (chunk_sz >= ctx->win_sz)
			fp = lut[out] ^ fp;

		fp_termination = ((ctx->tfp_bitmask & (1 << 0)) ?
				((fp & ctx->tfp_mask[0]) == ctx->tfp[0]) : 0) ||
				((ctx->tfp_bitmask & (1 << 1)) ?
				((fp & ctx->tfp_mask[1]) == ctx->tfp[1]) : 0) ||
				((ctx->tfp_bitmask & (1 << 2)) ?
				((fp & ctx->tfp_mask[2]) == ctx->tfp[2]) : 0) ||
				((ctx->tfp_bitmask & (1 << 3)) ?
				((fp & ctx->tfp_mask[3]) == ctx->tfp[3]) : 0);

		if ((fp_termination && (chunk_sz >= ctx->min_chunk_sz))
		  || (chunk_sz >= ctx->min_chunk_sz) || (i == (tsrc_len - 1))) {

			fp_desc = (struct fp_desc *)(rfcdp + tag_sz);
			fp_desc->size = chunk_sz;
			fp_desc->offset = i - (chunk_sz - 1);
			fp_desc->fp = fp;
			fp_desc->status = 0x0;
			/* calculate tag */
			do_shash(tag, rfcdp, chunkp, fp_desc->size, NULL, 0);

			if ((i == (tsrc_len - 1))) {
				fp_desc->status = 0x40;
				fp_desc->fp = 0;
			}

			ret = sg_pcopy_from_buffer(dst_sg, sg_nents(dst_sg),
						     rfcdp, rfcd_sz, src_done);
			if (ret != rfcd_sz) {
				pr_info("%s: dst sg copy failed at off:0x%x\n",
						     __func__, fp_desc->offset);
				break;
			}

			src_done += rfcd_sz;
			if (i == (tsrc_len - 1)) {
				pr_info("reached end\n");
				break;
			}

			/* copy buffer for next chunk */
			min_chunk_sz = min(ctx->max_chunk_sz,
							tsrc_len - (i + 1));
			ret = sg_pcopy_to_buffer(src_sg, sg_nents(src_sg),
						chunkp, min_chunk_sz, i + 1);
			if (ret != min_chunk_sz) {
				pr_err("copy to buf failed at\n"
					"off:%x, ret:%x, sz:%x, rem:%x\n",
				    i + 1, ret, min_chunk_sz, tsrc_len - i + 1);
				ret = -EINVAL;
				goto out;
			}
			/* reset fp parameter */
			i += chunk_start_offset;
			chunk_sz = chunk_start_offset;
			widx = 0;
			fp = 0;
			memset(win, 0x0, ctx->win_sz);
		}
	}
	ret = 0;
	pr_info("finger print calculated\n");
out:
	kfree(chunkp);
chk_sz_fail:
	kfree(rfcdp);
	return ret;
}
EXPORT_SYMBOL_GPL(rabin_get_fps);

static int rabin_get_finger_print(struct rabin_request *req)
{
	struct rfp_desc *ctx = crypto_rabin_ctx(crypto_rabin_reqtfm(req));
	int rc;

	if (!(ctx->rabin_flag & FLAG_RABIN_LUT_DONE)) {
		if ((ctx->rabin_flag & FLAG_RABIN_WIN_SZ) &&
			(ctx->rabin_flag & FLAG_RABIN_POLY)) {
			initialize_lut(ctx->win_sz, ctx->polynomial,
							ctx->rabin_lut);
			ctx->rabin_flag |= FLAG_RABIN_LUT_DONE;
		} else {
			pr_err("%s: LUT initialization error flag:%x\n",
						__func__, ctx->rabin_flag);
			return -EINVAL;
		}
	}
	rc = rabin_get_fps(req->src_sg, req->dst_sg, req->tag, ctx);

	return rc;
}

static int rabin_set_chunk_size(struct crypto_rabin *tfm,
						u32 minchunk, u32 maxchunk)
{
	struct rfp_desc *ctx = crypto_rabin_ctx(tfm);

	ctx->min_chunk_sz = minchunk;
	ctx->max_chunk_sz = maxchunk;

	ctx->rabin_flag |= FLAG_RABIN_CHUNK_SZ;
	pr_info("%s:chunk_size min=0x%x, max:0x%x\n",
			__func__, ctx->min_chunk_sz, ctx->max_chunk_sz);

	return 0;
}

static int rabin_set_polynomial(struct crypto_rabin *tfm, unsigned int poly)
{
	struct rfp_desc *ctx = crypto_rabin_ctx(tfm);

	ctx->polynomial = poly;
	ctx->rabin_flag &= ~FLAG_RABIN_LUT_DONE;
	ctx->rabin_flag |= FLAG_RABIN_POLY;

	pr_info("polynomial :0x%x\n", poly);

	return 0;
}

static int rabin_set_win_size(struct crypto_rabin *tfm, unsigned int winsize)
{
	struct rfp_desc *ctx = crypto_rabin_ctx(tfm);

	ctx->win_sz = winsize;
	ctx->rabin_flag &= ~FLAG_RABIN_LUT_DONE;
	ctx->rabin_flag |= FLAG_RABIN_WIN_SZ;

	pr_info("window size:0x%x\n", winsize);

	return 0;
}

static int rabin_set_termination(struct crypto_rabin *tfm,
						u32 tfprint, u32 mask)
{
	struct rfp_desc *ctx = crypto_rabin_ctx(tfm);
	u32 idx = 0;

	idx = ffz(ctx->tfp_bitmask);
	ctx->tfp[idx] = tfprint;
	ctx->tfp_mask[idx] = mask;
	ctx->tfp_bitmask |= (1 << idx);

	pr_info("%s:tfprint:%u, mask:%u, bitmsak:0x%x\n",
			__func__, tfprint, mask, ctx->tfp_bitmask);

	return 0;
}

static int rabin_cra_init(struct crypto_rabin *rabin)
{
	struct crypto_tfm *tfm = crypto_rabin_tfm(rabin);
	struct rfp_desc *ctx = crypto_tfm_ctx(tfm);

	pr_info("%s() tfm:%p\n", __func__, tfm);

	memset(ctx, 0x0, sizeof(*ctx));

	return 0;
}

static struct rabin_alg alg = {
	.base = {
		.cra_name = "rabin-fp",
		.cra_driver_name = "rabin-fp-iproc",
		.cra_module = THIS_MODULE,
		.cra_priority = 400,
		.cra_alignmask = 0,
		.cra_ctxsize = sizeof(struct rfp_desc),
		.cra_type = &crypto_rabin_type,
		.cra_flags = CRYPTO_ALG_TYPE_RABIN,
	},
	.set_chunk_size = rabin_set_chunk_size,
	.set_polynomial = rabin_set_polynomial,
	.set_win_size = rabin_set_win_size,
	.get_finger_print = rabin_get_finger_print,
	.set_fp_termination = rabin_set_termination,
	.init = rabin_cra_init,
};

static int __init rfp_mod_init(void)
{
	pr_info("Registering Rabin algo\n");
	return crypto_register_rabin(&alg);
}

static void __exit rfp_mod_fini(void)
{
	crypto_unregister_rabin(&alg);
}

module_init(rfp_mod_init);
module_exit(rfp_mod_fini);
