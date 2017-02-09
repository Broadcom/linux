/* Copyright (C) 2016 Broadcom
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
 * Broadcom FlexSparx4 Mailbox Test Client
 *
 * This driver is a mailbox client which directly injects
 * "struct brcm_message" instances into mailbox channels
 * pointing to fixed input/output data patterns. The driver
 * also provides a sysfs interface to configure and start
 * the test.
 *
 * Following are the possible uses of the driver:
 * 1. Debug a particular feature of FS4 offload engine or
 * the SoC specific ring manager hardware
 * 2. Stress a particular feature of FS4 offload engine
 * 3. Benchmark raw performance of a FS4 offload engine feature
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox/brcm-message.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/raid/xor.h>
#include <linux/raid/pq.h>

#define FS4_ENGINE_REG_SIZE		0x1000
#define FS4_MAX_BATCH_COUNT		512
#define FS4_MAX_CHANNELS		16

struct fs4_test_msg {
	void *src[FS4_MAX_BATCH_COUNT];
	dma_addr_t src_dma[FS4_MAX_BATCH_COUNT];
	void *dst[FS4_MAX_BATCH_COUNT];
	dma_addr_t dst_dma[FS4_MAX_BATCH_COUNT];
	void *dst1[FS4_MAX_BATCH_COUNT];
	dma_addr_t dst1_dma[FS4_MAX_BATCH_COUNT];
	void *dst_resp[FS4_MAX_BATCH_COUNT];
	dma_addr_t dst_resp_dma[FS4_MAX_BATCH_COUNT];
	struct brcm_sba_command *cmds[FS4_MAX_BATCH_COUNT];
	struct brcm_message msg[FS4_MAX_CHANNELS * FS4_MAX_BATCH_COUNT];
	unsigned int msg_count;
	struct brcm_message bmsg[FS4_MAX_CHANNELS];
	unsigned int bmsg_count;
	ktime_t start_ktime;
	s64 runtime_usecs;
	atomic_t done_count;
	struct completion done;
};

struct fs4_test {
	struct device *dev;
	struct device *mbox_dev;
	int (*exec_func)(struct fs4_test *test);
	void __iomem *regs;
	unsigned int engine_count;
	struct mbox_client client;
	unsigned int mchans_count;
	struct mbox_chan *mchans[FS4_MAX_CHANNELS];
	struct mutex lock;
	unsigned int chan;
	unsigned int batch_count;
	unsigned int min_split_size;
	unsigned int src_count;
	unsigned int src_size;
	unsigned int iterations;
	unsigned int timeout;
	unsigned int update;
	unsigned int verify;
	unsigned int verbose;
	unsigned int poll;
	unsigned int software;
	unsigned int start;
};

#define fs4_debug(__test, __msg...) \
do { \
	if ((__test)->verbose) \
		dev_info((__test)->dev, __msg); \
} while (0)

#define fs4_info(__test, __msg...) dev_info((__test)->dev, __msg)

static unsigned long long fs4_test_persec(s64 runtime, unsigned int val)
{
	unsigned long long per_sec = 1000000;

	if (runtime <= 0)
		return 0;

	/* drop precision until runtime is 32-bits */
	while (runtime > UINT_MAX) {
		runtime >>= 1;
		per_sec <<= 1;
	}

	per_sec *= val;
	do_div(per_sec, runtime);
	return per_sec;
}

static unsigned long long fs4_test_KBs(s64 runtime, unsigned long long len)
{
	return fs4_test_persec(runtime, len >> 10);
}

static void fs4_test_receive_message(struct mbox_client *cl, void *m)
{
	struct brcm_message *msg = m;
	struct fs4_test_msg *cmsg = msg->ctx;

	if (atomic_dec_return(&cmsg->done_count))
		return;

	if (!cmsg->runtime_usecs)
		cmsg->runtime_usecs =
			ktime_us_delta(ktime_get(), cmsg->start_ktime);

	complete(&cmsg->done);
}

static const u8 spu2_ref_input[] = {
0x11, 0x90, 0x00, 0x04, 0x0e, 0x20, 0x00, 0xb7,
0x80, 0x00, 0x05, 0x81, 0x00, 0xe0, 0x80, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x40, 0x01, 0x00, 0x00, 0x22, 0x6d, 0xdd, 0xf7,
0xf6, 0xea, 0xd3, 0xb2, 0x0e, 0xbb, 0x93, 0x6c,
0xcd, 0x31, 0x8d, 0xe1, 0x87, 0x58, 0xaf, 0x6d,
0x62, 0x8a, 0x71, 0xa2, 0x24, 0x5a, 0x20, 0xf9,
0x58, 0x46, 0x66, 0xad, 0x67, 0xda, 0xde, 0x11,
0x53, 0x0d, 0x98, 0x7c, 0x17, 0xf2, 0xf2, 0x8a,
0xbb, 0x75, 0x67, 0x50, 0x2b, 0x0c, 0x99, 0xcd,
0x78, 0x9f, 0xe3, 0xb1, 0xee, 0x8c, 0x89, 0x14,
0x4a, 0x43, 0x97, 0x9e, 0xda, 0xb2, 0xa0, 0x7f,
0x60, 0x59, 0x23, 0x43, 0xd7, 0xc3, 0x59, 0x9c,
0x10, 0xe8, 0x44, 0xb7, 0x93, 0x97, 0xf8, 0xb5,
0x09, 0x8d, 0x88, 0x93, 0x33, 0xde, 0x89, 0x45,
0xb4, 0x45, 0x81, 0x46, 0x23, 0x00, 0xcc, 0x17,
0xde, 0x95, 0xce, 0xd1, 0xfd, 0xae, 0xfb, 0x82,
0x9f, 0x20, 0x01, 0x75, 0xc1, 0xe7, 0xa9, 0xd2,
0xd0, 0xc7, 0x25, 0x67, 0x3d, 0xa7, 0xf0, 0x73,
0x28, 0x64, 0x02, 0x9f, 0xe4, 0xef, 0x4e, 0x20,
0x00, 0x3e, 0x56, 0xc6, 0x63, 0x62, 0xe5, 0xe5,
0x7e, 0x76, 0xfa, 0x37, 0x6e, 0x11, 0xe8, 0x76,
0xa1, 0x8f, 0xef, 0x94, 0x82, 0x16, 0x31, 0x3c,
0xa1, 0xf9, 0x10, 0x9c, 0xd5, 0xb6, 0xac, 0xeb,
0x9e, 0x77, 0x43, 0xe3, 0xd7, 0xf1, 0x30, 0x03,
0x64, 0x2a, 0x62, 0xf4, 0x4b, 0x02, 0x1d, 0x0a,
0x28, 0x74, 0x9b, 0xc3, 0xe3, 0xe1, 0xf2, 0x6b,
0x49, 0xf7, 0xc7, 0xcc, 0x19, 0xa8, 0x5a, 0x77,
0x05, 0x61, 0x11, 0x57, 0x4b, 0x7f, 0xff, 0x2c,
0xcf, 0x19, 0xa6, 0xb0, 0x9f, 0xfe, 0xc1, 0x91,
0xf3, 0x30, 0xf9, 0x0b, 0x2b, 0x3c, 0xda, 0x65,
0x6d, 0xaf, 0xd2, 0x27, 0xb5, 0xfa, 0x62, 0x66,
0x54, 0x3d, 0x36, 0xd3, 0x22, 0xcd, 0x86, 0x78,
0x5a, 0x11, 0x46, 0x44, 0xd6, 0x3f, 0x19, 0x23,
0x6c, 0xde, 0xd2, 0x7d, 0xab, 0x1d, 0x1b, 0x5e,
0xd7, 0xf4, 0xff, 0xab, 0x38, 0xfd, 0x40, 0x3e,
0x8c, 0xd4, 0x0d, 0xbd, 0xe2, 0xd7, 0x7c, 0x2a,
0x32, 0xde, 0xb1, 0x2e, 0x30, 0x5d, 0x1b, 0x2d,
0xd6, 0x79, 0x79, 0xa7, 0x7e, 0xae, 0xc1, 0x8a,
0xb9, 0xc7, 0x3f, 0x59, 0xf3, 0x73, 0x08, 0x8d,
0xb0, 0xfd, 0x14, 0x56, 0x47, 0xbb, 0x8c, 0xc6,
0xd7, 0x11, 0xd1, 0x74, 0x94, 0x26, 0x99, 0x68,
0x8e, 0x8a, 0x32, 0x82, 0x97, 0xca, 0x55, 0xb0,
0x3d, 0xd5, 0xd5, 0xac, 0x9c, 0x1f, 0x88, 0x4c,
0x05, 0xc8, 0x00, 0x69, 0xc9, 0x95, 0xb6, 0x6b,
0xab, 0x07, 0x70, 0x8a, 0x7c, 0x34, 0xa6, 0x2e,
};

static const u8 spu2_ref_output[] = {
0xbf, 0x95, 0xe7, 0xc9, 0x43, 0xcd, 0x44, 0xe5,
0xc3, 0xb2, 0xf8, 0x97, 0x67, 0xd1, 0x6f, 0x59,
0x64, 0xb3, 0x46, 0xba, 0x5d, 0xe4, 0xfe, 0x24,
0xa2, 0x1e, 0xa6, 0x0d, 0xbc, 0x12, 0x91, 0xaa,
0xbc, 0x6d, 0xab, 0x4d, 0x47, 0xf9, 0xaa, 0x44,
0x17, 0xf4, 0x4d, 0xb0, 0xcf, 0x15, 0xf1, 0x8a,
0x24, 0xc8, 0xa3, 0x58, 0x71, 0x08, 0x51, 0x59,
0x34, 0x30, 0x7d, 0xb6, 0xff, 0x1f, 0x29, 0xc8,
0x83, 0x16, 0xa5, 0x7f, 0x61, 0xa6, 0x9c, 0x39,
0x80, 0xdc, 0x91, 0x50, 0xda, 0x27, 0x6a, 0xa0,
0x45, 0xec, 0x13, 0xf5, 0x1c, 0xde, 0x4f, 0x28,
0x5c, 0xdc, 0x58, 0xd4, 0x39, 0x5d, 0xce, 0x76,
0xa6, 0xe1, 0xf7, 0x4f, 0x54, 0xc4, 0xc6, 0xe5,
0x50, 0xc5, 0xb1, 0xab, 0xd2, 0x58, 0xa4, 0x38,
0xfb, 0x28, 0xfc, 0xad, 0xe2, 0x35, 0x63, 0xde,
0xaf, 0x2a, 0x91, 0x65, 0x73, 0x44, 0x2c, 0x1d,
0x80, 0x03, 0xe4, 0x52, 0x94, 0x17, 0xc5, 0x98,
0xda, 0x83, 0x51, 0xd6, 0xc8, 0xc1, 0x6e, 0xb0,
0x08, 0x13, 0xcc, 0x1f, 0x1a, 0x50, 0x66, 0x6e,
0xa2, 0x8c, 0xfd, 0x99, 0xdf, 0x61, 0x19, 0x05,
0x4d, 0xe5, 0x73, 0x0a, 0x86, 0x6c, 0xdb, 0x1a,
0xf3, 0xec, 0x47, 0xa4, 0xa7, 0xb0, 0xec, 0x39,
0xf2, 0x3a, 0x99, 0x39, 0x8a, 0x50, 0xf9, 0x59,
0xa6, 0xc6, 0x55, 0xa0, 0x67, 0x79, 0xd2, 0xb9,
0x4e, 0xd5, 0x48, 0xc7, 0x6d, 0xdf, 0x57, 0x1c,
0xe9, 0xd6, 0xc5, 0xbc, 0x2b, 0xfb, 0x3c, 0xae,
0xbd, 0xc1, 0xb9, 0x35, 0xc5, 0x68, 0xbf, 0x78,
0xb8, 0xba, 0x77, 0x9d, 0xf1, 0x8d, 0xce, 0xbe,
0x39, 0x00, 0x24, 0x31, 0x09, 0xab, 0xd2, 0x1f,
0x60, 0x5f, 0x80, 0x28, 0x51, 0x62, 0x7d, 0xcd,
0xc3, 0xe1, 0x26, 0x40, 0x5e, 0x4b, 0xf1, 0xb5,
0x99, 0xd4, 0x33, 0x4a, 0x3b, 0x8b, 0x2d, 0x03,
0x93, 0xdb, 0xd3, 0x62, 0x52, 0xf5, 0x82, 0x20,
0xdb, 0x1b, 0x54, 0x11, 0xef, 0x44, 0xa2, 0x01,
0xb0, 0xc4, 0x14, 0x5d, 0x86, 0x5a, 0xe7, 0x7a,
0x11, 0x7e, 0x3b, 0xef, 0x3f, 0x39, 0xdc, 0x34,
0x79, 0x96, 0x66, 0x7e, 0x93, 0x8f, 0x34, 0x46,
0x77, 0x72, 0xbb, 0xd5, 0x17, 0x85, 0x4b, 0x5d,
0x10, 0xec, 0x19, 0x29, 0xe0, 0xbc, 0x62, 0x97,
0x43, 0x94, 0x57, 0xbd, 0xd5, 0x8f, 0x29, 0xd2,
};

/* Note: Must be called with test->lock held */
static int __spu2_exec(struct fs4_test *test)
{
	int rc = 0, i;
	unsigned int iter = 0;
	unsigned long tout;
	struct scatterlist src;
	struct scatterlist dst;
	struct mbox_chan *chan;
	struct fs4_test_msg *cmsg;
	unsigned long long input_bytes_count;
	const u8 *ref_input = spu2_ref_input;
	size_t ref_input_size = sizeof(spu2_ref_input);
	const u8 *ref_output = spu2_ref_output;
	size_t ref_output_size = sizeof(spu2_ref_output);
	s64 iter_usecs, min_usecs = 0, max_usecs = 0, avg_usecs = 0;
	unsigned long long iter_KBs, min_KBs = 0, max_KBs = 0, avg_KBs = 0;

	if (ref_input_size > PAGE_SIZE) {
		fs4_info(test, "unsupported input size\n");
		return -ENOTSUPP;
	}

	if (ref_output_size > PAGE_SIZE) {
		fs4_info(test, "unsupported output size\n");
		return -ENOTSUPP;
	}

	cmsg = devm_kzalloc(test->dev, sizeof(*cmsg), GFP_KERNEL);
	if (!cmsg)
		return -ENOMEM;

	cmsg->src[0] = devm_kzalloc(test->dev, ref_input_size,
				    GFP_KERNEL);
	if (!cmsg->src[0]) {
		devm_kfree(test->dev, cmsg);
		return -ENOMEM;
	}

	fs4_debug(test, "src=0x%p src_size=0x%x\n",
		  cmsg->src[0], (u32)ref_input_size);

	memcpy(cmsg->src[0], ref_input, ref_input_size);

	cmsg->dst[0] = devm_kzalloc(test->dev, ref_output_size,
				 GFP_KERNEL);
	if (!cmsg->dst[0]) {
		devm_kfree(test->dev, cmsg->src[0]);
		devm_kfree(test->dev, cmsg);
		return -ENOMEM;
	}

	fs4_debug(test, "dst=0x%p dst_size=0x%x\n",
		  cmsg->dst[0], (u32)ref_output_size);

	sg_init_one(&src, cmsg->src[0], ref_input_size);
	sg_init_one(&dst, cmsg->dst[0], ref_output_size);

	while (iter < test->iterations) {
		chan = test->mchans[test->chan];
		fs4_info(test, "iter=%u channel=%u",
			 iter, test->chan);
		test->chan++;
		if (test->chan >= test->mchans_count)
			test->chan = 0;

		memset(cmsg->dst[0], 0, ref_output_size);
		input_bytes_count = ref_input_size;

		cmsg->start_ktime = ktime_get();
		cmsg->runtime_usecs = 0;

		cmsg->msg[0].type = BRCM_MESSAGE_SPU;
		cmsg->msg[0].spu.src = &src;
		cmsg->msg[0].spu.dst = &dst;
		cmsg->msg[0].ctx = cmsg;
		cmsg->msg[0].error = 0;
		cmsg->msg_count = 1;
		atomic_set(&cmsg->done_count, 1);
		init_completion(&cmsg->done);

		rc = mbox_send_message(chan, &cmsg->msg[0]);
		if (rc < 0) {
			fs4_info(test, "iter=%u send error\n", iter);
			break;
		}
		rc = 0;

		if (cmsg->msg[0].error < 0) {
			rc = cmsg->msg[0].error;
			break;
		}

		if (test->poll) {
			while (atomic_read(&cmsg->done_count) > 0)
				for (i = 0; i < test->mchans_count; i++)
					mbox_client_peek_data(test->mchans[i]);
		} else {
			tout = (unsigned long)test->timeout * 1000;
			tout = msecs_to_jiffies(tout);
			tout = wait_for_completion_timeout(&cmsg->done, tout);
			if (!tout) {
				fs4_info(test, "iter=%u wait timeout\n", iter);
				rc = -ETIMEDOUT;
				break;
			}
		}

		rc = 0;
		for (i = 0; i < cmsg->msg_count; i++)
			if (cmsg->msg[0].error < 0) {
				fs4_info(test, "iter=%u msg=%d rx error\n",
					 iter, i);
				rc = cmsg->msg[0].error;
			}
		if (rc < 0)
			break;

		if (test->verify) {
			for (i = 0; i < ref_output_size; i++)
				if (((u8 *)cmsg->dst[0])[i] !=
						((u8 *)ref_output)[i])
					break;
			if (i != ref_output_size) {
				fs4_info(test, "iter=%u mismatch at %d\n",
					 iter, i);
				print_hex_dump(KERN_INFO, "ref: ",
						DUMP_PREFIX_ADDRESS,
						16, 1, ref_output,
						ref_output_size, true);
				print_hex_dump(KERN_INFO, "dst: ",
						DUMP_PREFIX_ADDRESS,
						16, 1, cmsg->dst[0],
						ref_output_size, true);
				rc = -EIO;
				break;
			}
		}

		iter_usecs = cmsg->runtime_usecs;
		iter_KBs =
		fs4_test_KBs(cmsg->runtime_usecs, input_bytes_count);
		min_usecs = (iter == 0) ?
			    iter_usecs : min(min_usecs, iter_usecs);
		max_usecs = (iter == 0) ?
			    iter_usecs : max(max_usecs, iter_usecs);
		avg_usecs += iter_usecs;
		min_KBs = (iter == 0) ?
			  iter_KBs : min(min_KBs, iter_KBs);
		max_KBs = (iter == 0) ?
			  iter_KBs : max(max_KBs, iter_KBs);
		avg_KBs += iter_KBs;
		fs4_info(test, "iter=%u usecs=%ld KBs=%llu",
			 iter, (long)iter_usecs, iter_KBs);

		iter++;
	}

	devm_kfree(test->dev, cmsg->dst[0]);
	devm_kfree(test->dev, cmsg->src[0]);
	devm_kfree(test->dev, cmsg);

	if (iter) {
		avg_usecs = avg_usecs / iter;
		avg_KBs = avg_KBs / iter;
	}

	fs4_info(test, "completed %u/%u iterations\n",
		 iter, test->iterations);
	fs4_info(test, "min_usecs=%ld min_KBs=%llu",
		 (long)min_usecs, min_KBs);
	fs4_info(test, "max_usecs=%ld max_KBs=%llu",
		 (long)max_usecs, max_KBs);
	fs4_info(test, "avg_usecs=%ld avg_KBs=%llu",
		 (long)avg_usecs, avg_KBs);

	return rc;
}

/* SBA command helper macros */
#define SBA_DEC(_d, _s, _m)		(((_d) >> (_s)) & (_m))
#define SBA_ENC(_d, _v, _s, _m)		\
			do { \
				(_d) &= ~((u64)(_m) << (_s)); \
				(_d) |= (((u64)(_v) & (_m)) << (_s)); \
			} while (0)

/* SBA command encoding */
#define SBA_TYPE_SHIFT				48
#define SBA_TYPE_MASK				0x3
#define SBA_TYPE_A				0x0
#define SBA_TYPE_B				0x2
#define SBA_TYPE_C				0x3
#define SBA_USER_DEF_SHIFT			32
#define SBA_USER_DEF_MASK			0xffff
#define SBA_R_MDATA_SHIFT			24
#define SBA_R_MDATA_MASK			0xff
#define SBA_C_MDATA_MS_SHIFT			18
#define SBA_C_MDATA_MS_MASK			0x3
#define SBA_INT_SHIFT				17
#define SBA_INT_MASK				0x1
#define SBA_RESP_SHIFT				16
#define SBA_RESP_MASK				0x1
#define SBA_C_MDATA_SHIFT			8
#define SBA_C_MDATA_MASK			0xff
#define SBA_CMD_SHIFT				0
#define SBA_CMD_MASK				0xf
#define SBA_CMD_ZERO_ALL_BUFFERS		0x8
#define SBA_CMD_LOAD_BUFFER			0x9
#define SBA_CMD_XOR				0xa
#define SBA_CMD_GALOIS_XOR			0xb
#define SBA_CMD_GALOIS				0xe
#define SBA_CMD_ZERO_BUFFER			0x4
#define SBA_CMD_WRITE_BUFFER			0xc

/* SBA C_MDATA helper macros */
#define SBA_C_MDATA_LOAD_VAL(__bnum0)		((__bnum0) & 0x3)
#define SBA_C_MDATA_WRITE_VAL(__bnum0)		((__bnum0) & 0x3)
#define SBA_C_MDATA_XOR_VAL(__bnum1, __bnum0)			\
			({	u32 __v = ((__bnum0) & 0x3);	\
				__v |= ((__bnum1) & 0x3) << 2;	\
				__v;				\
			})
#define SBA_C_MDATA_PQ_VAL(__dnum, __bnum1, __bnum0)		\
			({	u32 __v = ((__bnum0) & 0x3);	\
				__v |= ((__bnum1) & 0x3) << 2;	\
				__v |= ((__dnum) & 0x1f) << 5;	\
				__v;				\
			})
#define SBA_C_MDATA_LS(__c_mdata_val)	((__c_mdata_val) & 0xff)
#define SBA_C_MDATA_MS(__c_mdata_val)	(((__c_mdata_val) >> 8) & 0x3)

/* SBA response encoding */
#define SBA_RESP_SIZE				0x8

/* SBA limits */
#define SBA_HW_BUF_SIZE				(4*1024)

static const u8 sba_memcpy_ref[] = {
0xd0, 0x0d, 0xfe, 0xed, 0xaa, 0xaa, 0x55, 0x55,
0xde, 0xad, 0xc0, 0x01, 0xa5, 0xa5, 0xa5, 0xa5,
0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
};

#define SBA_MEMCPY_SPLIT_CMD_COUNT(test, split_buf_count)	\
		(((test)->src_count * 2) * (split_buf_count))
#define SBA_MEMCPY_REF_SIZE		sizeof(sba_memcpy_ref)
#define SBA_MEMCPY_CMD_COUNT(test, split_count, split_buf_count)	\
	((split_count) * SBA_MEMCPY_SPLIT_CMD_COUNT(test, split_buf_count))

/* Note: Must be called with test->lock held */
static unsigned int __sba_memcpy_split_cmds(struct fs4_test *test,
					    struct brcm_sba_command *cmds,
					    unsigned int split,
					    unsigned int cur_split_size,
					    unsigned int split_size,
					    dma_addr_t src_dma_base,
					    dma_addr_t dst_dma_base,
					    dma_addr_t dst_resp_dma_base)
{
	int s;
	u64 cmd;
	unsigned int cpos = 0, csize;
	unsigned int cmds_count = 0;

	while (cur_split_size) {
		csize = (cur_split_size < SBA_HW_BUF_SIZE) ?
					cur_split_size : SBA_HW_BUF_SIZE;

		for (s = 0; s < test->src_count; s++) {
			/* Type-B command to load data into buf0 */
			cmd = 0;
			SBA_ENC(cmd, SBA_TYPE_B,
				SBA_TYPE_SHIFT, SBA_TYPE_MASK);
			SBA_ENC(cmd, csize,
				SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
			SBA_ENC(cmd, 0x0, SBA_RESP_SHIFT, SBA_RESP_MASK);
			SBA_ENC(cmd, 0x0,
				SBA_C_MDATA_SHIFT, SBA_C_MDATA_MASK);
			SBA_ENC(cmd, SBA_CMD_LOAD_BUFFER,
				SBA_CMD_SHIFT, SBA_CMD_MASK);
			cmds[cmds_count].cmd = cmd;
			cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_B;
			cmds[cmds_count].data =
					src_dma_base + s * test->src_size +
					split * split_size + cpos;
			cmds[cmds_count].data_len = csize;
			cmds_count++;

			/* Type-A command to write buf0 */
			cmd = 0;
			SBA_ENC(cmd, SBA_TYPE_A,
				SBA_TYPE_SHIFT, SBA_TYPE_MASK);
			SBA_ENC(cmd, csize,
				SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
			SBA_ENC(cmd, 0x1, SBA_RESP_SHIFT, SBA_RESP_MASK);
			SBA_ENC(cmd, 0x0,
			SBA_C_MDATA_SHIFT, SBA_C_MDATA_MASK);
			SBA_ENC(cmd, SBA_CMD_WRITE_BUFFER,
				SBA_CMD_SHIFT, SBA_CMD_MASK);
			cmds[cmds_count].cmd = cmd;
			cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_A;
			cmds[cmds_count].flags |= BRCM_SBA_CMD_HAS_RESP;
			cmds[cmds_count].flags |= BRCM_SBA_CMD_HAS_OUTPUT;
			cmds[cmds_count].resp =
				dst_resp_dma_base + split * SBA_RESP_SIZE;
			cmds[cmds_count].resp_len = SBA_RESP_SIZE;
			cmds[cmds_count].data =
				dst_dma_base + s * test->src_size +
				split * split_size + cpos;
			cmds[cmds_count].data_len = csize;
			cmds_count++;
		}

		cpos += csize;
		cur_split_size -= csize;
	}

	return cmds_count;
}

/* Note: Must be called with test->lock held */
static void __sba_memcpy_free(struct fs4_test *test,
			      struct fs4_test_msg *cmsg,
			      unsigned int split_count)
{
	int b;

	if (!test || !cmsg)
		return;

	for (b = 0; b < test->batch_count; b++) {
		if (cmsg->dst_resp[b]) {
			dma_free_coherent(
				  test->mbox_dev,
				  SBA_RESP_SIZE * split_count,
				  cmsg->dst_resp[b], cmsg->dst_resp_dma[b]);
			cmsg->dst_resp[b] = NULL;
		}
		if (cmsg->dst[b]) {
			dma_free_coherent(
					test->mbox_dev,
					test->src_count * test->src_size,
					cmsg->dst[b], cmsg->dst_dma[b]);
			cmsg->dst[b] = NULL;
		}
		if (cmsg->src[b]) {
			dma_free_coherent(
					test->mbox_dev,
					test->src_count * test->src_size,
					cmsg->src[b], cmsg->src_dma[b]);
			cmsg->src[b] = NULL;
		}
		if (cmsg->cmds[b]) {
			devm_kfree(test->dev, cmsg->cmds[b]);
			cmsg->cmds[b] = NULL;
		}
	}

	devm_kfree(test->dev, cmsg);
}

/* Note: Must be called with test->lock held */
static struct fs4_test_msg *__sba_memcpy_alloc(struct fs4_test *test,
					       unsigned int split_count,
					       unsigned int split_size,
					       unsigned int split_buf_count)
{
	unsigned int b, s, i;
	struct fs4_test_msg *cmsg = NULL;
	unsigned int cmds_idx, cmds_count;
	unsigned int cur_split_size, src_size;

	if (!test)
		return NULL;

	cmsg = devm_kzalloc(test->dev, sizeof(*cmsg), GFP_KERNEL);
	if (!cmsg)
		return NULL;

	for (b = 0; b < test->batch_count; b++) {
		cmsg->cmds[b] = devm_kcalloc(test->dev,
		SBA_MEMCPY_CMD_COUNT(test, split_count, split_buf_count),
		sizeof(*cmsg->cmds[b]), GFP_KERNEL);
		if (!cmsg->cmds[b]) {
			fs4_info(test, "failed to alloc sba command array\n");
			__sba_memcpy_free(test, cmsg, split_count);
			return NULL;
		}

		cmsg->src[b] = dma_alloc_coherent(
				test->mbox_dev,
				test->src_count * test->src_size,
				&cmsg->src_dma[b], GFP_KERNEL);
		if (!cmsg->src[b]) {
			fs4_info(test, "failed to alloc src buffer\n");
			__sba_memcpy_free(test, cmsg, split_count);
			return NULL;
		}

		fs4_debug(test, "src[%d]=0x%p src_dma[%d]=0x%lx\n",
		b, cmsg->src[b], b, (unsigned long)cmsg->src_dma[b]);

		for (s = 0; s < test->src_count; s++) {
			for (i = 0;
			     i < (test->src_size / SBA_MEMCPY_REF_SIZE);
			     i++)
				memcpy(cmsg->src[b] +
				s * test->src_size + i * SBA_MEMCPY_REF_SIZE,
				sba_memcpy_ref, SBA_MEMCPY_REF_SIZE);
		}

		cmsg->dst[b] = dma_alloc_coherent(
					test->mbox_dev,
					test->src_count * test->src_size,
					&cmsg->dst_dma[b], GFP_KERNEL);
		if (!cmsg->dst[b]) {
			fs4_info(test, "failed to alloc dst buffer\n");
			__sba_memcpy_free(test, cmsg, split_count);
			return NULL;
		}

		fs4_debug(test, "dst[%d]=0x%p dst_dma[%d]=0x%lx\n",
		b, cmsg->dst[b], b, (unsigned long)cmsg->dst_dma[b]);

		cmsg->dst_resp[b] = dma_alloc_coherent(
					test->mbox_dev,
					SBA_RESP_SIZE * split_count,
					&cmsg->dst_resp_dma[b], GFP_KERNEL);
		if (!cmsg->dst_resp[b]) {
			fs4_info(test, "failed to alloc dst_resp buffer\n");
			__sba_memcpy_free(test, cmsg, split_count);
			return NULL;
		}

		fs4_debug(test, "dst_resp[%d]=0x%p dst_resp_dma[%d]=0x%lx\n",
		b, cmsg->dst_resp[b], b, (unsigned long)cmsg->dst_resp_dma[b]);

		src_size = test->src_size;
		for (s = 0; (s < split_count) && src_size; s++) {
			cur_split_size = min(src_size, split_size);

			cmds_idx =
			s * SBA_MEMCPY_SPLIT_CMD_COUNT(test, split_buf_count);
			cmds_count = __sba_memcpy_split_cmds(test,
					   &cmsg->cmds[b][cmds_idx],
					   s, cur_split_size, split_size,
					   cmsg->src_dma[b], cmsg->dst_dma[b],
					   cmsg->dst_resp_dma[b]);

			cmsg->msg[cmsg->msg_count + s].type =
						BRCM_MESSAGE_SBA;
			cmsg->msg[cmsg->msg_count + s].sba.cmds =
						&cmsg->cmds[b][cmds_idx];
			cmsg->msg[cmsg->msg_count + s].sba.cmds_count =
						cmds_count;
			cmsg->msg[cmsg->msg_count + s].ctx = cmsg;
			cmsg->msg[cmsg->msg_count + s].error = 0;

			src_size -= cur_split_size;
		}

		cmsg->msg_count += split_count;
	}

	b = cmsg->msg_count / test->mchans_count;
	if ((b * test->mchans_count) < cmsg->msg_count)
		b++;
	s = 0;
	cmsg->bmsg_count = 0;
	while (s < cmsg->msg_count) {
		i = min((cmsg->msg_count - s), b);
		cmsg->bmsg[cmsg->bmsg_count].type = BRCM_MESSAGE_BATCH;
		cmsg->bmsg[cmsg->bmsg_count].batch.msgs = &cmsg->msg[s];
		cmsg->bmsg[cmsg->bmsg_count].batch.msgs_queued = 0;
		cmsg->bmsg[cmsg->bmsg_count].batch.msgs_count = i;
		fs4_debug(test, "batch%d msg_idx=%d msg_count=%d\n",
			  cmsg->bmsg_count, s, i);
		cmsg->bmsg_count++;
		s += i;
	}

	fs4_debug(test, "msgs_count=%d msgs_per_chan=%d batch_msg_count=%d\n",
		  cmsg->msg_count, b, cmsg->bmsg_count);

	return cmsg;
}

/* Note: Must be called with test->lock held */
static bool __sba_memcpy_verify(struct fs4_test *test,
				struct fs4_test_msg *cmsg,
				unsigned int iter)
{
	int b, i, t;
	bool ret = true;

	for (b = 0; b < test->batch_count; b++) {
		for (i = 0; i < (test->src_size * test->src_count) / 8; i++)
			if (((u64 *)cmsg->dst[b])[i] !=
						((u64 *)cmsg->src[b])[i])
				break;
		if (i != ((test->src_size * test->src_count) / 8)) {
			i *= 8;
			fs4_info(test, "iter=%u batch=%u mismatch at %d\n",
				 iter, b, i);
			t = test->src_size - i;
			if (t >= SBA_MEMCPY_REF_SIZE)
				t = SBA_MEMCPY_REF_SIZE;
			print_hex_dump(KERN_INFO, "src: ",
					DUMP_PREFIX_ADDRESS,
					8, 1, cmsg->src[b] + i,
					t, true);
			print_hex_dump(KERN_INFO, "dst_resp: ",
					DUMP_PREFIX_ADDRESS,
					8, 1, cmsg->dst_resp[b],
					SBA_RESP_SIZE, true);
			print_hex_dump(KERN_INFO, "dst: ",
					DUMP_PREFIX_ADDRESS,
					8, 1, cmsg->dst[b] + i,
					t, true);
			ret = false;
			break;
		}
	}

	return ret;
}

/* Note: Must be called with test->lock held */
static int __sba_memcpy_exec(struct fs4_test *test)
{
	int rc = 0, i;
	unsigned long tout;
	struct mbox_chan *chan;
	struct fs4_test_msg *cmsg;
	unsigned long long input_bytes_count;
	unsigned int cur_split_size, src_size, iter = 0;
	unsigned int split_count, split_size, split_buf_count;
	s64 iter_usecs, min_usecs = 0, max_usecs = 0, avg_usecs = 0;
	unsigned long long iter_KBs, min_KBs = 0, max_KBs = 0, avg_KBs = 0;

	if (test->batch_count > FS4_MAX_BATCH_COUNT) {
		fs4_info(test, "batch_count should be less than %d\n",
			 (int)FS4_MAX_BATCH_COUNT);
		return -EINVAL;
	}
	if ((SBA_HW_BUF_SIZE/4) > test->min_split_size) {
		fs4_info(test, "min_split_size should be greater than %d\n",
			 (int)(SBA_HW_BUF_SIZE/4 - 1));
		return -EINVAL;
	}
	if (test->min_split_size > 0x10000) {
		fs4_info(test, "min_split_size can be upto 1MB or less\n");
		return -EINVAL;
	}
	if ((SBA_HW_BUF_SIZE/4 > test->src_size)) {
		fs4_info(test, "src_size should be greater than %d\n",
			 (int)(SBA_HW_BUF_SIZE/4 - 1));
		return -EINVAL;
	}
	if (test->src_size & (SBA_MEMCPY_REF_SIZE - 1)) {
		fs4_info(test, "src_size has to be multiple of %d\n",
			 (int)SBA_MEMCPY_REF_SIZE);
		return -EINVAL;
	}
	if ((test->src_size * test->src_count) > 0x200000) {
		fs4_info(test, "memory requirement greater than 2M\n");
		return -EINVAL;
	}

	if (test->src_size <= test->min_split_size) {
		split_size = test->src_size;
	} else {
		split_size = test->src_size / test->mchans_count;
		if (split_size * test->mchans_count < test->src_size)
			split_size++;
		if (split_size <= test->min_split_size)
			split_size = test->min_split_size;
		if (split_size > test->src_size)
			split_size = test->src_size;
	}
	split_buf_count = split_size / SBA_HW_BUF_SIZE;
	if ((split_buf_count * SBA_HW_BUF_SIZE) < split_size)
		split_buf_count++;
	split_count = 0;
	src_size = test->src_size;
	while (src_size) {
		cur_split_size = min(src_size, split_size);
		split_count++;
		src_size -= cur_split_size;
	}

	fs4_info(test, "split_count=%u split_size=%u split_buf_count=%u\n",
		 split_count, split_size, split_buf_count);

	cmsg = __sba_memcpy_alloc(test, split_count,
				  split_size, split_buf_count);
	if (!cmsg)
		return -ENOMEM;

	while (iter < test->iterations) {
		fs4_info(test, "iter=%u started", iter);

		input_bytes_count = test->src_count * test->src_size;
		input_bytes_count *= test->batch_count;
		for (i = 0; i < test->batch_count; i++) {
			memset(cmsg->dst[i], 0,
				test->src_count * test->src_size);
			memset(cmsg->dst_resp[i], 0,
				SBA_RESP_SIZE * split_count);
		}

		cmsg->start_ktime = ktime_get();
		cmsg->runtime_usecs = 0;

		if (test->software) {
			for (i = 0; i < test->batch_count; i++)
				memcpy(cmsg->dst[i], cmsg->src[i],
					test->src_count * test->src_size);
			cmsg->runtime_usecs =
			ktime_us_delta(ktime_get(), cmsg->start_ktime);
			goto skip_mailbox;
		}

		atomic_set(&cmsg->done_count, cmsg->msg_count);
		init_completion(&cmsg->done);

		rc = 0;
		for (i = 0; i < cmsg->bmsg_count; i++) {
			chan = test->mchans[test->chan];
			test->chan++;
			if (test->chan >= test->mchans_count)
				test->chan = 0;

			cmsg->bmsg[i].batch.msgs_queued = 0;
			rc = mbox_send_message(chan, &cmsg->bmsg[i]);
			if (rc < 0) {
				fs4_info(test, "iter=%u msg=%d send error %d\n",
					 iter, i, rc);
				break;
			}
			rc = 0;

			if (cmsg->bmsg[i].error < 0) {
				rc = cmsg->bmsg[i].error;
				break;
			}
		}
		if (rc < 0)
			break;

		if (test->poll) {
			while (atomic_read(&cmsg->done_count) > 0)
				for (i = 0; i < test->mchans_count; i++)
					mbox_client_peek_data(test->mchans[i]);
		} else {
			tout = (unsigned long)test->timeout * 1000;
			tout = msecs_to_jiffies(tout);
			tout = wait_for_completion_timeout(&cmsg->done, tout);
			if (!tout) {
				fs4_info(test, "iter=%u wait timeout\n", iter);
				rc = -ETIMEDOUT;
				break;
			}
		}

		rc = 0;
		for (i = 0; i < cmsg->msg_count; i++)
			if (cmsg->msg[i].error < 0) {
				fs4_info(test, "iter=%u msg=%d rx error\n",
					 iter, i);
				rc = cmsg->msg[i].error;
			}
		if (rc < 0)
			break;

skip_mailbox:
		if (test->verify) {
			if (!__sba_memcpy_verify(test, cmsg, iter))
				break;
		}

		iter_usecs = cmsg->runtime_usecs;
		iter_KBs =
		fs4_test_KBs(cmsg->runtime_usecs, input_bytes_count);
		min_usecs = (iter == 0) ?
			    iter_usecs : min(min_usecs, iter_usecs);
		max_usecs = (iter == 0) ?
			    iter_usecs : max(max_usecs, iter_usecs);
		avg_usecs += iter_usecs;
		min_KBs = (iter == 0) ?
			  iter_KBs : min(min_KBs, iter_KBs);
		max_KBs = (iter == 0) ?
			  iter_KBs : max(max_KBs, iter_KBs);
		avg_KBs += iter_KBs;

		fs4_info(test, "iter=%u usecs=%ld KBs=%llu",
			 iter, (long)iter_usecs, iter_KBs);

		iter++;
	}

	__sba_memcpy_free(test, cmsg, split_count);

	if (iter) {
		avg_usecs = avg_usecs / iter;
		avg_KBs = avg_KBs / iter;
	}

	fs4_info(test, "completed %u/%u iterations\n",
		 iter, test->iterations);
	fs4_info(test, "min_usecs=%ld min_KBs=%llu",
		 (long)min_usecs, min_KBs);
	fs4_info(test, "max_usecs=%ld max_KBs=%llu",
		 (long)max_usecs, max_KBs);
	fs4_info(test, "avg_usecs=%ld avg_KBs=%llu",
		 (long)avg_usecs, avg_KBs);

	return rc;
}

static const u8 sba_xor_ref1[] = {
0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
};

static const u8 sba_xor_ref2[] = {
0x5a, 0x5a, 0x5a, 0x5a, 0x5a, 0x5a, 0x5a, 0x5a,
};

static const u8 sba_xor_ref3[] = {
0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
};

static const u8 sba_xor_ref4[] = {
0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
};

#define SBA_XOR_MAX_SRC_COUNT		14
#define SBA_XOR_SPLIT_CMD_COUNT(test, split_buf_count)	\
		(((test)->src_count + 1) * (split_buf_count))
#define SBA_XOR_REF_SIZE		sizeof(sba_xor_ref1)
#define SBA_XOR_CMD_COUNT(test, split_count, split_buf_count)	\
	((split_count) * SBA_XOR_SPLIT_CMD_COUNT(test, split_buf_count))

/* Note: Must be called with test->lock held */
static unsigned int __sba_xor_split_cmds(struct fs4_test *test,
					 struct brcm_sba_command *cmds,
					 unsigned int split,
					 unsigned int cur_split_size,
					 unsigned int split_size,
					 dma_addr_t src_dma_base,
					 dma_addr_t dst_dma_base,
					 dma_addr_t dst_resp_dma_base)
{
	int s;
	u64 cmd;
	unsigned int cpos = 0, csize;
	unsigned int cmds_count = 0;

	while (cur_split_size) {
		csize = (cur_split_size < SBA_HW_BUF_SIZE) ?
					cur_split_size : SBA_HW_BUF_SIZE;

		/* Type-B command to load data into buf0 */
		cmd = 0;
		SBA_ENC(cmd, SBA_TYPE_B,
			SBA_TYPE_SHIFT, SBA_TYPE_MASK);
		SBA_ENC(cmd, csize,
			SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
		SBA_ENC(cmd, 0x0, SBA_RESP_SHIFT, SBA_RESP_MASK);
		SBA_ENC(cmd, 0x0,
			SBA_C_MDATA_SHIFT, SBA_C_MDATA_MASK);
		SBA_ENC(cmd, SBA_CMD_LOAD_BUFFER,
			SBA_CMD_SHIFT, SBA_CMD_MASK);
		cmds[cmds_count].cmd = cmd;
		cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_B;
		cmds[cmds_count].data =
			src_dma_base + split * split_size + cpos;
		cmds[cmds_count].data_len = csize;
		cmds_count++;

		/* Type-B command to xor data onto buf0 */
		for (s = 1; s < test->src_count; s++) {
			cmd = 0;
			SBA_ENC(cmd, SBA_TYPE_B,
				SBA_TYPE_SHIFT, SBA_TYPE_MASK);
			SBA_ENC(cmd, csize,
				SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
			SBA_ENC(cmd, 0x0, SBA_RESP_SHIFT, SBA_RESP_MASK);
			SBA_ENC(cmd, 0x0,
				SBA_C_MDATA_SHIFT, SBA_C_MDATA_MASK);
			SBA_ENC(cmd, SBA_CMD_XOR,
				SBA_CMD_SHIFT, SBA_CMD_MASK);
			cmds[cmds_count].cmd = cmd;
			cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_B;
			cmds[cmds_count].data =
				src_dma_base + s * test->src_size +
				split * split_size + cpos;
			cmds[cmds_count].data_len = csize;
			cmds_count++;
		}

		/* Type-A command to write buf0 */
		cmd = 0;
		SBA_ENC(cmd, SBA_TYPE_A,
			SBA_TYPE_SHIFT, SBA_TYPE_MASK);
		SBA_ENC(cmd, csize,
			SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
		SBA_ENC(cmd, 0x1, SBA_RESP_SHIFT, SBA_RESP_MASK);
		SBA_ENC(cmd, 0x0,
			SBA_C_MDATA_SHIFT, SBA_C_MDATA_MASK);
		SBA_ENC(cmd, SBA_CMD_WRITE_BUFFER,
			SBA_CMD_SHIFT, SBA_CMD_MASK);
		cmds[cmds_count].cmd = cmd;
		cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_A;
		cmds[cmds_count].flags |= BRCM_SBA_CMD_HAS_RESP;
		cmds[cmds_count].flags |= BRCM_SBA_CMD_HAS_OUTPUT;
		cmds[cmds_count].resp =
				dst_resp_dma_base + split * SBA_RESP_SIZE;
		cmds[cmds_count].resp_len = SBA_RESP_SIZE;
		cmds[cmds_count].data =
				dst_dma_base + split * split_size + cpos;
		cmds[cmds_count].data_len = csize;
		cmds_count++;

		cpos += csize;
		cur_split_size -= csize;
	}

	return cmds_count;
}

/* Note: Must be called with test->lock held */
static void __sba_software_xor(struct fs4_test *test, void *dst, void *src)
{
	int s, src_off, src_cnt, xor_src_cnt;
	void *srcs[SBA_XOR_MAX_SRC_COUNT];

	preempt_disable();

	for (s = 0; s < test->src_count; s++)
		srcs[s] = src + s * test->src_size;

	src_off = 0;
	src_cnt = test->src_count;
	while (src_cnt > 0) {
		/* process up to 'MAX_XOR_BLOCKS' sources */
		xor_src_cnt = min(src_cnt, MAX_XOR_BLOCKS);
		xor_blocks(xor_src_cnt, test->src_size, dst, &srcs[src_off]);

		/* drop completed sources */
		src_cnt -= xor_src_cnt;
		src_off += xor_src_cnt;
	}

	preempt_enable();
}

/* Note: Must be called with test->lock held */
static void __sba_xor_free(struct fs4_test *test,
			   struct fs4_test_msg *cmsg,
			   unsigned int split_count)
{
	int b;

	if (!test || !cmsg)
		return;

	for (b = 0; b < test->batch_count; b++) {
		if (cmsg->dst_resp[b]) {
			dma_free_coherent(
				  test->mbox_dev,
				  SBA_RESP_SIZE * split_count,
				  cmsg->dst_resp[b], cmsg->dst_resp_dma[b]);
			cmsg->dst_resp[b] = NULL;
		}
		if (cmsg->dst[b]) {
			dma_free_coherent(
					test->mbox_dev,
					test->src_size,
					cmsg->dst[b], cmsg->dst_dma[b]);
			cmsg->dst[b] = NULL;
		}
		if (cmsg->src[b]) {
			dma_free_coherent(
					test->mbox_dev,
					test->src_count * test->src_size,
					cmsg->src[b], cmsg->src_dma[b]);
			cmsg->src[b] = NULL;
		}
		if (cmsg->cmds[b]) {
			devm_kfree(test->dev, cmsg->cmds[b]);
			cmsg->cmds[b] = NULL;
		}
	}

	devm_kfree(test->dev, cmsg);
}

/* Note: Must be called with test->lock held */
static struct fs4_test_msg *__sba_xor_alloc(struct fs4_test *test,
					    unsigned int split_count,
					    unsigned int split_size,
					    unsigned int split_buf_count)
{
	unsigned int b, s, i;
	const u8 *ref_split;
	struct fs4_test_msg *cmsg = NULL;
	unsigned int cmds_idx, cmds_count;
	unsigned int cur_split_size, src_size;

	if (!test)
		return NULL;

	cmsg = devm_kzalloc(test->dev, sizeof(*cmsg), GFP_KERNEL);
	if (!cmsg)
		return NULL;

	for (b = 0; b < test->batch_count; b++) {
		cmsg->cmds[b] = devm_kcalloc(test->dev,
			SBA_XOR_CMD_COUNT(test, split_count, split_buf_count),
			sizeof(*cmsg->cmds[b]), GFP_KERNEL);
		if (!cmsg->cmds[b]) {
			fs4_info(test, "failed to alloc sba command array\n");
			__sba_xor_free(test, cmsg, split_count);
			return NULL;
		}

		cmsg->src[b] = dma_alloc_coherent(
				test->mbox_dev,
				test->src_count * test->src_size,
				&cmsg->src_dma[b], GFP_KERNEL);
		if (!cmsg->src[b]) {
			fs4_info(test, "failed to alloc src buffer\n");
			__sba_xor_free(test, cmsg, split_count);
			return NULL;
		}

		fs4_debug(test, "src[%d]=0x%p src_dma[%d]=0x%lx\n",
		b, cmsg->src[b], b, (unsigned long)cmsg->src_dma[b]);

		for (s = 0; s < test->src_count; s++) {
			switch (s & 0x3) {
			case 0:
				ref_split = sba_xor_ref1;
				break;
			case 1:
				ref_split = sba_xor_ref2;
				break;
			case 2:
				ref_split = sba_xor_ref3;
				break;
			case 3:
				ref_split = sba_xor_ref4;
				break;
			}
			for (i = 0; i < test->src_size / SBA_XOR_REF_SIZE; i++)
				memcpy(cmsg->src[b] +
				s * test->src_size + i * SBA_XOR_REF_SIZE,
				ref_split, SBA_XOR_REF_SIZE);
		}

		cmsg->dst[b] = dma_alloc_coherent(
					test->mbox_dev,
					test->src_size, &cmsg->dst_dma[b],
					GFP_KERNEL);
		if (!cmsg->dst[b]) {
			fs4_info(test, "failed to alloc dst buffer\n");
			__sba_xor_free(test, cmsg, split_count);
			return NULL;
		}

		fs4_debug(test, "dst[%d]=0x%p dst_dma[%d]=0x%lx\n",
		b, cmsg->dst[b], b, (unsigned long)cmsg->dst_dma[b]);

		cmsg->dst_resp[b] = dma_alloc_coherent(
					test->mbox_dev,
					SBA_RESP_SIZE * split_count,
					&cmsg->dst_resp_dma[b], GFP_KERNEL);
		if (!cmsg->dst_resp[b]) {
			fs4_info(test, "failed to alloc dst_resp buffer\n");
			__sba_xor_free(test, cmsg, split_count);
			return NULL;
		}

		fs4_debug(test, "dst_resp[%d]=0x%p dst_resp_dma[%d]=0x%lx\n",
		b, cmsg->dst_resp[b], b, (unsigned long)cmsg->dst_resp_dma[b]);

		src_size = test->src_size;
		for (s = 0; (s < split_count) && src_size; s++) {
			cur_split_size = min(src_size, split_size);

			cmds_idx =
			s * SBA_XOR_SPLIT_CMD_COUNT(test, split_buf_count);
			cmds_count = __sba_xor_split_cmds(test,
					   &cmsg->cmds[b][cmds_idx],
					   s, cur_split_size, split_size,
					   cmsg->src_dma[b], cmsg->dst_dma[b],
					   cmsg->dst_resp_dma[b]);

			cmsg->msg[cmsg->msg_count + s].type =
						BRCM_MESSAGE_SBA;
			cmsg->msg[cmsg->msg_count + s].sba.cmds =
						&cmsg->cmds[b][cmds_idx];
			cmsg->msg[cmsg->msg_count + s].sba.cmds_count =
						cmds_count;
			cmsg->msg[cmsg->msg_count + s].ctx = cmsg;
			cmsg->msg[cmsg->msg_count + s].error = 0;

			src_size -= cur_split_size;
		}

		cmsg->msg_count += split_count;
	}

	b = cmsg->msg_count / test->mchans_count;
	if ((b * test->mchans_count) < cmsg->msg_count)
		b++;
	s = 0;
	cmsg->bmsg_count = 0;
	while (s < cmsg->msg_count) {
		i = min((cmsg->msg_count - s), b);
		cmsg->bmsg[cmsg->bmsg_count].type = BRCM_MESSAGE_BATCH;
		cmsg->bmsg[cmsg->bmsg_count].batch.msgs = &cmsg->msg[s];
		cmsg->bmsg[cmsg->bmsg_count].batch.msgs_queued = 0;
		cmsg->bmsg[cmsg->bmsg_count].batch.msgs_count = i;
		fs4_debug(test, "batch%d msg_idx=%d msg_count=%d\n",
			  cmsg->bmsg_count, s, i);
		cmsg->bmsg_count++;
		s += i;
	}

	fs4_debug(test, "msgs_count=%d msgs_per_chan=%d batch_msg_count=%d\n",
		  cmsg->msg_count, b, cmsg->bmsg_count);

	return cmsg;
}

/* Note: Must be called with test->lock held */
static bool __sba_xor_verify(struct fs4_test *test,
			     struct fs4_test_msg *cmsg,
			     unsigned int iter, u64 ref_out_magic)
{
	int b, i, t;
	bool ret = true;

	for (b = 0; b < test->batch_count; b++) {
		for (i = 0; i < (test->src_size / 8); i++)
			if (((u64 *)cmsg->dst[b])[i] != ref_out_magic) {
				fs4_info(test, "got=0x%lx exp=0x%lx\n",
				(unsigned long)((u64 *)cmsg->dst[b])[i],
				(unsigned long)ref_out_magic);
				ret = false;
				break;
			}
		if (i != (test->src_size / 8)) {
			i *= 8;
			fs4_info(test, "iter=%u batch=%u mismatch at %d\n",
				 iter, b, i);
			t = test->src_size - i;
			if (t >= SBA_XOR_REF_SIZE)
				t = SBA_XOR_REF_SIZE;
			print_hex_dump(KERN_INFO, "dst_resp: ",
					DUMP_PREFIX_ADDRESS,
					8, 1, cmsg->dst_resp[b],
					SBA_RESP_SIZE, true);
			print_hex_dump(KERN_INFO, "dst: ",
					DUMP_PREFIX_ADDRESS,
					8, 1, cmsg->dst[b] + i,
					t, true);
			break;
		}
	}

	return ret;
}

/* Note: Must be called with test->lock held */
static int __sba_xor_exec(struct fs4_test *test)
{
	int rc = 0, i;
	u64 ref_out_magic;
	unsigned long tout;
	struct mbox_chan *chan;
	struct fs4_test_msg *cmsg;
	unsigned long long input_bytes_count;
	unsigned int cur_split_size, src_size, iter = 0;
	unsigned int split_count, split_size, split_buf_count;
	s64 iter_usecs, min_usecs = 0, max_usecs = 0, avg_usecs = 0;
	unsigned long long iter_KBs, min_KBs = 0, max_KBs = 0, avg_KBs = 0;

	if (test->batch_count > FS4_MAX_BATCH_COUNT) {
		fs4_info(test, "batch_count should be less than %d\n",
			 (int)FS4_MAX_BATCH_COUNT);
		return -EINVAL;
	}
	if ((SBA_HW_BUF_SIZE/4) > test->min_split_size) {
		fs4_info(test, "min_split_size should be greater than %d\n",
			 (int)(SBA_HW_BUF_SIZE/4 - 1));
		return -EINVAL;
	}
	if (test->min_split_size > 0x10000) {
		fs4_info(test, "min_split_size can be upto 1MB or less\n");
		return -EINVAL;
	}
	if ((SBA_HW_BUF_SIZE/4) > test->src_size) {
		fs4_info(test, "src_size should be greater than %d\n",
			 (int)(SBA_HW_BUF_SIZE/4 - 1));
		return -EINVAL;
	}
	if (test->src_size & (SBA_XOR_REF_SIZE - 1)) {
		fs4_info(test, "src_size has to be multiple of %d\n",
			 (int)SBA_XOR_REF_SIZE);
		return -EINVAL;
	}
	if (test->src_count < 2) {
		fs4_info(test, "src_count cannot be less than 2\n");
		return -EINVAL;
	}
	if (test->update && (test->src_count != 3)) {
		fs4_info(test, "src_count has to be 3 for update mode\n");
		return -EINVAL;
	}
	if (test->src_count > SBA_XOR_MAX_SRC_COUNT) {
		fs4_info(test, "src_count cannot be greater than %d\n",
			 (int)SBA_XOR_MAX_SRC_COUNT);
		return -EINVAL;
	}
	if ((test->src_size * test->src_count) > 0x200000) {
		fs4_info(test, "memory requirement greater than 2M\n");
		return -EINVAL;
	}

	if (test->src_size <= test->min_split_size) {
		split_size = test->src_size;
	} else {
		split_size = test->src_size / test->mchans_count;
		if (split_size * test->mchans_count < test->src_size)
			split_size++;
		if (split_size <= test->min_split_size)
			split_size = test->min_split_size;
		if (split_size > test->src_size)
			split_size = test->src_size;
	}
	split_buf_count = split_size / SBA_HW_BUF_SIZE;
	if ((split_buf_count * SBA_HW_BUF_SIZE) < split_size)
		split_buf_count++;
	split_count = 0;
	src_size = test->src_size;
	while (src_size) {
		cur_split_size = min(src_size, split_size);
		split_count++;
		src_size -= cur_split_size;
	}

	fs4_info(test, "split_count=%u split_size=%u split_buf_count=%u\n",
		 split_count, split_size, split_buf_count);

	cmsg = __sba_xor_alloc(test,
			split_count, split_size, split_buf_count);
	if (!cmsg)
		return -ENOMEM;

	ref_out_magic = 0x0;
	for (i = 0; i < test->src_count; i++)
		ref_out_magic = ref_out_magic ^
				(*(u64 *)(cmsg->src[0] + i * test->src_size));

	while (iter < test->iterations) {
		fs4_info(test, "iter=%u started", iter);

		input_bytes_count = test->src_size * test->src_count;
		input_bytes_count *= test->batch_count;
		for (i = 0; i < test->batch_count; i++) {
			memset(cmsg->dst[i], 0, test->src_size);
			memset(cmsg->dst_resp[i], 0,
				SBA_RESP_SIZE * split_count);
		}

		cmsg->start_ktime = ktime_get();
		cmsg->runtime_usecs = 0;

		if (test->software) {
			for (i = 0; i < test->batch_count; i++)
				__sba_software_xor(test,
						cmsg->dst[i], cmsg->src[i]);
			cmsg->runtime_usecs =
			ktime_us_delta(ktime_get(), cmsg->start_ktime);
			goto skip_mailbox;
		}

		atomic_set(&cmsg->done_count, cmsg->msg_count);
		init_completion(&cmsg->done);

		rc = 0;
		for (i = 0; i < cmsg->bmsg_count; i++) {
			chan = test->mchans[test->chan];
			test->chan++;
			if (test->chan >= test->mchans_count)
				test->chan = 0;

			cmsg->bmsg[i].batch.msgs_queued = 0;
			rc = mbox_send_message(chan, &cmsg->bmsg[i]);
			if (rc < 0) {
				fs4_info(test, "iter=%u msg=%d send error %d\n",
					 iter, i, rc);
				break;
			}
			rc = 0;

			if (cmsg->bmsg[i].error < 0) {
				rc = cmsg->bmsg[i].error;
				break;
			}
		}
		if (rc < 0)
			break;

		if (test->poll) {
			while (atomic_read(&cmsg->done_count) > 0)
				for (i = 0; i < test->mchans_count; i++)
					mbox_client_peek_data(test->mchans[i]);
		} else {
			tout = (unsigned long)test->timeout * 1000;
			tout = msecs_to_jiffies(tout);
			tout = wait_for_completion_timeout(&cmsg->done, tout);
			if (!tout) {
				fs4_info(test, "iter=%u wait timeout\n", iter);
				rc = -ETIMEDOUT;
				break;
			}
		}

		rc = 0;
		for (i = 0; i < cmsg->msg_count; i++)
			if (cmsg->msg[i].error < 0) {
				fs4_info(test, "iter=%u msg=%d rx error\n",
					 iter, i);
				rc = cmsg->msg[i].error;
			}
		if (rc < 0)
			break;

skip_mailbox:
		if (test->verify) {
			if (!__sba_xor_verify(test, cmsg,
					      iter, ref_out_magic))
				break;
		}

		iter_usecs = cmsg->runtime_usecs;
		iter_KBs =
		fs4_test_KBs(cmsg->runtime_usecs, input_bytes_count);
		min_usecs = (iter == 0) ?
			    iter_usecs : min(min_usecs, iter_usecs);
		max_usecs = (iter == 0) ?
			    iter_usecs : max(max_usecs, iter_usecs);
		avg_usecs += iter_usecs;
		min_KBs = (iter == 0) ?
			  iter_KBs : min(min_KBs, iter_KBs);
		max_KBs = (iter == 0) ?
			  iter_KBs : max(max_KBs, iter_KBs);
		avg_KBs += iter_KBs;

		fs4_info(test, "iter=%u usecs=%ld KBs=%llu",
			 iter, (long)iter_usecs, iter_KBs);

		iter++;
	}

	__sba_xor_free(test, cmsg, split_count);

	if (iter) {
		avg_usecs = avg_usecs / iter;
		avg_KBs = avg_KBs / iter;
	}

	fs4_info(test, "completed %u/%u iterations\n",
		 iter, test->iterations);
	fs4_info(test, "min_usecs=%ld min_KBs=%llu",
		 (long)min_usecs, min_KBs);
	fs4_info(test, "max_usecs=%ld max_KBs=%llu",
		 (long)max_usecs, max_KBs);
	fs4_info(test, "avg_usecs=%ld avg_KBs=%llu",
		 (long)avg_usecs, avg_KBs);

	return rc;
}

static const u8 sba_pq_ref1[] = {
0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
};

static const u8 sba_pq_ref2[] = {
0x5a, 0x5a, 0x5a, 0x5a, 0x5a, 0x5a, 0x5a, 0x5a,
};

static const u8 sba_pq_ref3[] = {
0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
};

static const u8 sba_pq_ref4[] = {
0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
};

#define SBA_PQ_MAX_SRC_COUNT		12
#define SBA_UPD_PQ_NEW_D		0
#define SBA_UPD_PQ_OLD_D		1
#define SBA_UPD_PQ_OLD_P		2
#define SBA_UPD_PQ_OLD_Q		3
#define SBA_UPD_PQ_POS			(SBA_PQ_MAX_SRC_COUNT / 2)
#define SBA_PQ_SPLIT_CMD_COUNT(test, split_buf_count)	\
		(((test)->src_count + 3) * (split_buf_count))
#define SBA_PQ_REF_SIZE		sizeof(sba_pq_ref1)
#define SBA_PQ_CMD_COUNT(test, split_count, split_buf_count)	\
	((split_count) * SBA_PQ_SPLIT_CMD_COUNT(test, split_buf_count))

static unsigned int __sba_upd_pq_split_cmds(struct fs4_test *test,
					    struct brcm_sba_command *cmds,
					    unsigned int split,
					    unsigned int cur_split_size,
					    unsigned int split_size,
					    dma_addr_t src_dma_base,
					    dma_addr_t dst_dma_base,
					    dma_addr_t dst1_dma_base,
					    dma_addr_t dst_resp_dma_base)
{
	u64 cmd;
	unsigned int cmds_count = 0;
	unsigned int cpos = 0, c_mdata, csize;

	while (cur_split_size) {
		csize = (cur_split_size < SBA_HW_BUF_SIZE) ?
					cur_split_size : SBA_HW_BUF_SIZE;

		/* Type-B command to load old P into buf0 */
		cmd = 0;
		SBA_ENC(cmd, SBA_TYPE_B, SBA_TYPE_SHIFT, SBA_TYPE_MASK);
		SBA_ENC(cmd, csize, SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
		SBA_ENC(cmd, 0x0, SBA_RESP_SHIFT, SBA_RESP_MASK);
		c_mdata = SBA_C_MDATA_LOAD_VAL(0);
		SBA_ENC(cmd, SBA_C_MDATA_LS(c_mdata),
			SBA_C_MDATA_SHIFT, SBA_C_MDATA_MASK);
		SBA_ENC(cmd, SBA_CMD_LOAD_BUFFER,
			SBA_CMD_SHIFT, SBA_CMD_MASK);
		cmds[cmds_count].cmd = cmd;
		cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_B;
		cmds[cmds_count].data = src_dma_base +
					SBA_UPD_PQ_OLD_P * test->src_size +
					split * split_size + cpos;
		cmds[cmds_count].data_len = csize;
		cmds_count++;

		/* Type-B command to load old Q into buf1 */
		cmd = 0;
		SBA_ENC(cmd, SBA_TYPE_B, SBA_TYPE_SHIFT, SBA_TYPE_MASK);
		SBA_ENC(cmd, csize, SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
		SBA_ENC(cmd, 0x0, SBA_RESP_SHIFT, SBA_RESP_MASK);
		c_mdata = SBA_C_MDATA_LOAD_VAL(1);
		SBA_ENC(cmd, SBA_C_MDATA_LS(c_mdata),
			SBA_C_MDATA_SHIFT, SBA_C_MDATA_MASK);
		SBA_ENC(cmd, SBA_CMD_LOAD_BUFFER,
			SBA_CMD_SHIFT, SBA_CMD_MASK);
		cmds[cmds_count].cmd = cmd;
		cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_B;
		cmds[cmds_count].data = src_dma_base +
					SBA_UPD_PQ_OLD_Q * test->src_size +
					split * split_size + cpos;
		cmds[cmds_count].data_len = csize;
		cmds_count++;

		/* Type-B commands for generate P onto buf0 and Q onto buf1 */
		cmd = 0;
		SBA_ENC(cmd, SBA_TYPE_B,
			SBA_TYPE_SHIFT, SBA_TYPE_MASK);
		SBA_ENC(cmd, csize,
			SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
		SBA_ENC(cmd, 0x0, SBA_RESP_SHIFT, SBA_RESP_MASK);
		c_mdata = SBA_C_MDATA_PQ_VAL(SBA_UPD_PQ_POS, 1, 0);
		SBA_ENC(cmd, SBA_C_MDATA_LS(c_mdata),
			SBA_C_MDATA_SHIFT, SBA_C_MDATA_MASK);
		SBA_ENC(cmd, SBA_C_MDATA_MS(c_mdata),
			SBA_C_MDATA_MS_SHIFT, SBA_C_MDATA_MS_MASK);
		SBA_ENC(cmd, SBA_CMD_GALOIS_XOR,
			SBA_CMD_SHIFT, SBA_CMD_MASK);
		cmds[cmds_count].cmd = cmd;
		cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_B;
		cmds[cmds_count].data = src_dma_base +
					SBA_UPD_PQ_OLD_D * test->src_size +
					split * split_size + cpos;
		cmds[cmds_count].data_len = csize;
		cmds_count++;

		/* Type-B commands for generate P onto buf0 and Q onto buf1 */
		cmd = 0;
		SBA_ENC(cmd, SBA_TYPE_B,
			SBA_TYPE_SHIFT, SBA_TYPE_MASK);
		SBA_ENC(cmd, csize,
			SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
		SBA_ENC(cmd, 0x0, SBA_RESP_SHIFT, SBA_RESP_MASK);
		c_mdata = SBA_C_MDATA_PQ_VAL(SBA_UPD_PQ_POS, 1, 0);
		SBA_ENC(cmd, SBA_C_MDATA_LS(c_mdata),
			SBA_C_MDATA_SHIFT, SBA_C_MDATA_MASK);
		SBA_ENC(cmd, SBA_C_MDATA_MS(c_mdata),
			SBA_C_MDATA_MS_SHIFT, SBA_C_MDATA_MS_MASK);
		SBA_ENC(cmd, SBA_CMD_GALOIS_XOR,
			SBA_CMD_SHIFT, SBA_CMD_MASK);
		cmds[cmds_count].cmd = cmd;
		cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_B;
		cmds[cmds_count].data = src_dma_base +
					SBA_UPD_PQ_NEW_D * test->src_size +
					split * split_size + cpos;
		cmds[cmds_count].data_len = csize;
		cmds_count++;

		/* Type-A command to write buf0 */
		cmd = 0;
		SBA_ENC(cmd, SBA_TYPE_A,
			SBA_TYPE_SHIFT, SBA_TYPE_MASK);
		SBA_ENC(cmd, csize,
			SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
		SBA_ENC(cmd, 0x1, SBA_RESP_SHIFT, SBA_RESP_MASK);
		c_mdata = SBA_C_MDATA_WRITE_VAL(0);
		SBA_ENC(cmd, SBA_C_MDATA_LS(c_mdata),
			SBA_C_MDATA_SHIFT, SBA_C_MDATA_MASK);
		SBA_ENC(cmd, SBA_CMD_WRITE_BUFFER,
			SBA_CMD_SHIFT, SBA_CMD_MASK);
		cmds[cmds_count].cmd = cmd;
		cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_A;
		cmds[cmds_count].flags |= BRCM_SBA_CMD_HAS_RESP;
		cmds[cmds_count].flags |= BRCM_SBA_CMD_HAS_OUTPUT;
		cmds[cmds_count].resp =
				dst_resp_dma_base + split * SBA_RESP_SIZE;
		cmds[cmds_count].resp_len = SBA_RESP_SIZE;
		cmds[cmds_count].data =
				dst_dma_base + split * split_size + cpos;
		cmds[cmds_count].data_len = csize;
		cmds_count++;

		/* Type-A command to write buf1 */
		cmd = 0;
		SBA_ENC(cmd, SBA_TYPE_A,
			SBA_TYPE_SHIFT, SBA_TYPE_MASK);
		SBA_ENC(cmd, csize,
			SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
		SBA_ENC(cmd, 0x1, SBA_RESP_SHIFT, SBA_RESP_MASK);
		c_mdata = SBA_C_MDATA_WRITE_VAL(1);
		SBA_ENC(cmd, SBA_C_MDATA_LS(c_mdata),
			SBA_C_MDATA_SHIFT, SBA_C_MDATA_MASK);
		SBA_ENC(cmd, SBA_CMD_WRITE_BUFFER,
			SBA_CMD_SHIFT, SBA_CMD_MASK);
		cmds[cmds_count].cmd = cmd;
		cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_A;
		cmds[cmds_count].flags |= BRCM_SBA_CMD_HAS_RESP;
		cmds[cmds_count].flags |= BRCM_SBA_CMD_HAS_OUTPUT;
		cmds[cmds_count].resp =
				dst_resp_dma_base + split * SBA_RESP_SIZE;
		cmds[cmds_count].resp_len = SBA_RESP_SIZE;
		cmds[cmds_count].data =
				dst1_dma_base + split * split_size + cpos;
		cmds[cmds_count].data_len = csize;
		cmds_count++;

		cpos += csize;
		cur_split_size -= csize;
	}

	return cmds_count;
}

static unsigned int __sba_gen_pq_split_cmds(struct fs4_test *test,
					    struct brcm_sba_command *cmds,
					    unsigned int split,
					    unsigned int cur_split_size,
					    unsigned int split_size,
					    dma_addr_t src_dma_base,
					    dma_addr_t dst_dma_base,
					    dma_addr_t dst1_dma_base,
					    dma_addr_t dst_resp_dma_base)
{
	u64 cmd;
	unsigned int s, cmds_count = 0;
	unsigned int cpos = 0, c_mdata, csize;

	while (cur_split_size) {
		csize = (cur_split_size < SBA_HW_BUF_SIZE) ?
					cur_split_size : SBA_HW_BUF_SIZE;

		/* Type-A command to zero-out buffers */
		cmd = 0;
		SBA_ENC(cmd, SBA_TYPE_A, SBA_TYPE_SHIFT, SBA_TYPE_MASK);
		SBA_ENC(cmd, csize,
			SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
		SBA_ENC(cmd, SBA_CMD_ZERO_ALL_BUFFERS,
			SBA_CMD_SHIFT, SBA_CMD_MASK);
		cmds[cmds_count].cmd = cmd;
		cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_A;
		cmds_count++;

		/* Type-B commands for generate P onto buf0 and Q onto buf1 */
		for (s = 0; s < test->src_count; s++) {
			cmd = 0;
			SBA_ENC(cmd, SBA_TYPE_B,
				SBA_TYPE_SHIFT, SBA_TYPE_MASK);
			SBA_ENC(cmd, csize,
				SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
			SBA_ENC(cmd, 0x0, SBA_RESP_SHIFT, SBA_RESP_MASK);
			c_mdata = SBA_C_MDATA_PQ_VAL(s, 1, 0);
			SBA_ENC(cmd, SBA_C_MDATA_LS(c_mdata),
				SBA_C_MDATA_SHIFT, SBA_C_MDATA_MASK);
			SBA_ENC(cmd, SBA_C_MDATA_MS(c_mdata),
				SBA_C_MDATA_MS_SHIFT, SBA_C_MDATA_MS_MASK);
			SBA_ENC(cmd, SBA_CMD_GALOIS_XOR,
				SBA_CMD_SHIFT, SBA_CMD_MASK);
			cmds[cmds_count].cmd = cmd;
			cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_B;
			cmds[cmds_count].data =
				src_dma_base + s * test->src_size +
				split * split_size + cpos;
			cmds[cmds_count].data_len = csize;
			cmds_count++;
		}

		/* Type-A command to write buf0 */
		cmd = 0;
		SBA_ENC(cmd, SBA_TYPE_A,
			SBA_TYPE_SHIFT, SBA_TYPE_MASK);
		SBA_ENC(cmd, csize,
			SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
		SBA_ENC(cmd, 0x1, SBA_RESP_SHIFT, SBA_RESP_MASK);
		c_mdata = SBA_C_MDATA_WRITE_VAL(0);
		SBA_ENC(cmd, SBA_C_MDATA_LS(c_mdata),
			SBA_C_MDATA_SHIFT, SBA_C_MDATA_MASK);
		SBA_ENC(cmd, SBA_CMD_WRITE_BUFFER,
			SBA_CMD_SHIFT, SBA_CMD_MASK);
		cmds[cmds_count].cmd = cmd;
		cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_A;
		cmds[cmds_count].flags |= BRCM_SBA_CMD_HAS_RESP;
		cmds[cmds_count].flags |= BRCM_SBA_CMD_HAS_OUTPUT;
		cmds[cmds_count].resp =
				dst_resp_dma_base + split * SBA_RESP_SIZE;
		cmds[cmds_count].resp_len = SBA_RESP_SIZE;
		cmds[cmds_count].data =
				dst_dma_base + split * split_size + cpos;
		cmds[cmds_count].data_len = csize;
		cmds_count++;

		/* Type-A command to write buf1 */
		cmd = 0;
		SBA_ENC(cmd, SBA_TYPE_A,
			SBA_TYPE_SHIFT, SBA_TYPE_MASK);
		SBA_ENC(cmd, csize,
			SBA_USER_DEF_SHIFT, SBA_USER_DEF_MASK);
		SBA_ENC(cmd, 0x1, SBA_RESP_SHIFT, SBA_RESP_MASK);
		c_mdata = SBA_C_MDATA_WRITE_VAL(1);
		SBA_ENC(cmd, SBA_C_MDATA_LS(c_mdata),
			SBA_C_MDATA_SHIFT, SBA_C_MDATA_MASK);
		SBA_ENC(cmd, SBA_CMD_WRITE_BUFFER,
			SBA_CMD_SHIFT, SBA_CMD_MASK);
		cmds[cmds_count].cmd = cmd;
		cmds[cmds_count].flags = BRCM_SBA_CMD_TYPE_A;
		cmds[cmds_count].flags |= BRCM_SBA_CMD_HAS_RESP;
		cmds[cmds_count].flags |= BRCM_SBA_CMD_HAS_OUTPUT;
		cmds[cmds_count].resp =
				dst_resp_dma_base + split * SBA_RESP_SIZE;
		cmds[cmds_count].resp_len = SBA_RESP_SIZE;
		cmds[cmds_count].data =
				dst1_dma_base + split * split_size + cpos;
		cmds[cmds_count].data_len = csize;
		cmds_count++;

		cpos += csize;
		cur_split_size -= csize;
	}

	return cmds_count;
}

/* Note: Must be called with test->lock held */
static unsigned int __sba_pq_split_cmds(struct fs4_test *test,
					struct brcm_sba_command *cmds,
					unsigned int split,
					unsigned int cur_split_size,
					unsigned int split_size,
					dma_addr_t src_dma_base,
					dma_addr_t dst_dma_base,
					dma_addr_t dst1_dma_base,
					dma_addr_t dst_resp_dma_base)
{
	if (test->update)
		return __sba_upd_pq_split_cmds(test,
					       cmds,
					       split,
					       cur_split_size,
					       split_size,
					       src_dma_base,
					       dst_dma_base,
					       dst1_dma_base,
					       dst_resp_dma_base);
	else
		return __sba_gen_pq_split_cmds(test,
					       cmds,
					       split,
					       cur_split_size,
					       split_size,
					       src_dma_base,
					       dst_dma_base,
					       dst1_dma_base,
					       dst_resp_dma_base);
}

/* Note: Must be called with test->lock held */
static void __sba_software_pq(struct fs4_test *test,
			      void *dst, void *dst1, void *src)
{
	int s;
	unsigned int pos = 0, size;
	void *old, *new, *srcs[SBA_PQ_MAX_SRC_COUNT + 2];

	preempt_disable();

	if (test->update) {
		for (s = 0; s < SBA_PQ_MAX_SRC_COUNT; s++)
			srcs[s] = (void *)raid6_empty_zero_page;
		srcs[SBA_PQ_MAX_SRC_COUNT] = dst;
		srcs[SBA_PQ_MAX_SRC_COUNT + 1] = dst1;
		old = src + SBA_UPD_PQ_OLD_D * test->src_size;
		new = src + SBA_UPD_PQ_NEW_D * test->src_size;
	} else {
		for (s = 0; s < test->src_count; s++)
			srcs[s] = src + s * test->src_size;
		srcs[test->src_count] = dst;
		srcs[test->src_count + 1] = dst1;
		old = new = NULL;
	}

	while (pos < test->src_size) {
		size = ((test->src_size - pos) < PAGE_SIZE) ?
			(test->src_size - pos) : PAGE_SIZE;

		if (test->update) {
			old += pos;
			new += pos;
			srcs[SBA_PQ_MAX_SRC_COUNT] += pos;
			srcs[SBA_PQ_MAX_SRC_COUNT + 1] += pos;
			srcs[SBA_UPD_PQ_POS] = old;
			raid6_call.xor_syndrome(SBA_PQ_MAX_SRC_COUNT + 2,
						SBA_UPD_PQ_POS,
						SBA_UPD_PQ_POS + 1,
						size, srcs);
			srcs[SBA_UPD_PQ_POS] = new;
			raid6_call.xor_syndrome(SBA_PQ_MAX_SRC_COUNT + 2,
						SBA_UPD_PQ_POS,
						SBA_UPD_PQ_POS + 1,
						size, srcs);
		} else {
			for (s = 0; s < test->src_count + 2; s++)
				srcs[s] += pos;

			raid6_call.gen_syndrome(test->src_count + 2,
						size, srcs);
		}

		pos += size;
	}

	preempt_enable();
}

/* Note: Must be called with test->lock held */
static void __sba_pq_free(struct fs4_test *test,
			   struct fs4_test_msg *cmsg,
			   unsigned int split_count)
{
	int b;

	if (!test || !cmsg)
		return;

	for (b = 0; b < test->batch_count; b++) {
		if (cmsg->dst_resp[b]) {
			dma_free_coherent(
				  test->mbox_dev,
				  SBA_RESP_SIZE * split_count,
				  cmsg->dst_resp[b], cmsg->dst_resp_dma[b]);
			cmsg->dst_resp[b] = NULL;
		}
		if (cmsg->dst1[b]) {
			dma_free_coherent(
					test->mbox_dev,
					test->src_size,
					cmsg->dst1[b], cmsg->dst1_dma[b]);
			cmsg->dst1[b] = NULL;
		}
		if (cmsg->dst[b]) {
			dma_free_coherent(
					test->mbox_dev,
					test->src_size,
					cmsg->dst[b], cmsg->dst_dma[b]);
			cmsg->dst[b] = NULL;
		}
		if (cmsg->src[b]) {
			dma_free_coherent(
					test->mbox_dev,
					test->src_count * test->src_size,
					cmsg->src[b], cmsg->src_dma[b]);
			cmsg->src[b] = NULL;
		}
		if (cmsg->cmds[b]) {
			devm_kfree(test->dev, cmsg->cmds[b]);
			cmsg->cmds[b] = NULL;
		}
	}

	devm_kfree(test->dev, cmsg);
}

/* Note: Must be called with test->lock held */
static struct fs4_test_msg *__sba_pq_alloc(struct fs4_test *test,
					   unsigned int split_count,
					   unsigned int split_size,
					   unsigned int split_buf_count)
{
	unsigned int b, s, i;
	const u8 *ref_split;
	struct fs4_test_msg *cmsg = NULL;
	unsigned int cmds_idx, cmds_count;
	unsigned int cur_split_size, src_size;

	if (!test)
		return NULL;

	cmsg = devm_kzalloc(test->dev, sizeof(*cmsg), GFP_KERNEL);
	if (!cmsg)
		return NULL;

	for (b = 0; b < test->batch_count; b++) {
		cmsg->cmds[b] = devm_kcalloc(test->dev,
			SBA_PQ_CMD_COUNT(test, split_count, split_buf_count),
			sizeof(*cmsg->cmds[b]), GFP_KERNEL);
		if (!cmsg->cmds[b]) {
			fs4_info(test, "failed to alloc sba command array\n");
			__sba_pq_free(test, cmsg, split_count);
			return NULL;
		}

		cmsg->src[b] = dma_alloc_coherent(
				test->mbox_dev,
				test->src_count * test->src_size,
				&cmsg->src_dma[b], GFP_KERNEL);
		if (!cmsg->src[b]) {
			fs4_info(test, "failed to alloc src buffer\n");
			__sba_pq_free(test, cmsg, split_count);
			return NULL;
		}

		fs4_debug(test, "src[%d]=0x%p src_dma[%d]=0x%lx\n",
		b, cmsg->src[b], b, (unsigned long)cmsg->src_dma[b]);

		for (s = 0; s < test->src_count; s++) {
			switch (s & 0x3) {
			case 0:
				ref_split = sba_pq_ref1;
				break;
			case 1:
				ref_split = sba_pq_ref2;
				break;
			case 2:
				ref_split = sba_pq_ref3;
				break;
			case 3:
				ref_split = sba_pq_ref4;
				break;
			}
			for (i = 0; i < test->src_size / SBA_PQ_REF_SIZE; i++)
				memcpy(cmsg->src[b] +
				s * test->src_size + i * SBA_PQ_REF_SIZE,
				ref_split, SBA_PQ_REF_SIZE);
		}

		cmsg->dst[b] = dma_alloc_coherent(
					test->mbox_dev,
					test->src_size, &cmsg->dst_dma[b],
					GFP_KERNEL);
		if (!cmsg->dst[b]) {
			fs4_info(test, "failed to alloc dst buffer\n");
			__sba_pq_free(test, cmsg, split_count);
			return NULL;
		}

		fs4_debug(test, "dst[%d]=0x%p dst_dma[%d]=0x%lx\n",
		b, cmsg->dst[b], b, (unsigned long)cmsg->dst_dma[b]);

		cmsg->dst1[b] = dma_alloc_coherent(
					test->mbox_dev,
					test->src_size, &cmsg->dst1_dma[b],
					GFP_KERNEL);
		if (!cmsg->dst1[b]) {
			fs4_info(test, "failed to alloc dst1 buffer\n");
			__sba_pq_free(test, cmsg, split_count);
			return NULL;
		}

		fs4_debug(test, "dst1[%d]=0x%p dst1_dma[%d]=0x%lx\n",
		b, cmsg->dst1[b], b, (unsigned long)cmsg->dst1_dma[b]);

		cmsg->dst_resp[b] = dma_alloc_coherent(
					test->mbox_dev,
					SBA_RESP_SIZE * split_count,
					&cmsg->dst_resp_dma[b], GFP_KERNEL);
		if (!cmsg->dst_resp[b]) {
			fs4_info(test, "failed to alloc dst_resp buffer\n");
			__sba_pq_free(test, cmsg, split_count);
			return NULL;
		}

		fs4_debug(test, "dst_resp[%d]=0x%p dst_resp_dma[%d]=0x%lx\n",
		b, cmsg->dst_resp[b], b, (unsigned long)cmsg->dst_resp_dma[b]);

		src_size = test->src_size;
		for (s = 0; (s < split_count) && src_size; s++) {
			cur_split_size = min(src_size, split_size);

			cmds_idx =
			s * SBA_PQ_SPLIT_CMD_COUNT(test, split_buf_count);
			cmds_count = __sba_pq_split_cmds(test,
					   &cmsg->cmds[b][cmds_idx],
					   s, cur_split_size, split_size,
					   cmsg->src_dma[b],
					   cmsg->dst_dma[b],
					   cmsg->dst1_dma[b],
					   cmsg->dst_resp_dma[b]);

			cmsg->msg[cmsg->msg_count + s].type =
						BRCM_MESSAGE_SBA;
			cmsg->msg[cmsg->msg_count + s].sba.cmds =
						&cmsg->cmds[b][cmds_idx];
			cmsg->msg[cmsg->msg_count + s].sba.cmds_count =
						cmds_count;
			cmsg->msg[cmsg->msg_count + s].ctx = cmsg;
			cmsg->msg[cmsg->msg_count + s].error = 0;

			src_size -= cur_split_size;
		}

		cmsg->msg_count += split_count;
	}

	b = cmsg->msg_count / test->mchans_count;
	if ((b * test->mchans_count) < cmsg->msg_count)
		b++;
	s = 0;
	cmsg->bmsg_count = 0;
	while (s < cmsg->msg_count) {
		i = min((cmsg->msg_count - s), b);
		cmsg->bmsg[cmsg->bmsg_count].type = BRCM_MESSAGE_BATCH;
		cmsg->bmsg[cmsg->bmsg_count].batch.msgs = &cmsg->msg[s];
		cmsg->bmsg[cmsg->bmsg_count].batch.msgs_queued = 0;
		cmsg->bmsg[cmsg->bmsg_count].batch.msgs_count = i;
		fs4_debug(test, "batch%d msg_idx=%d msg_count=%d\n",
			  cmsg->bmsg_count, s, i);
		cmsg->bmsg_count++;
		s += i;
	}

	fs4_debug(test, "msgs_count=%d msgs_per_chan=%d batch_msg_count=%d\n",
		  cmsg->msg_count, b, cmsg->bmsg_count);

	return cmsg;
}

/* Note: Must be called with test->lock held */
static bool __sba_pq_verify(struct fs4_test *test,
			    struct fs4_test_msg *cmsg, unsigned int iter,
			    u64 ref_out_magic, u64 ref_out1_magic)
{
	int b, i, t;
	bool ret = true;

	for (b = 0; b < test->batch_count; b++) {
		for (i = 0; i < (test->src_size / 8); i++)
			if (((u64 *)cmsg->dst[b])[i] != ref_out_magic) {
				fs4_info(test, "got=0x%lx exp=0x%lx\n",
				(unsigned long)((u64 *)cmsg->dst[b])[i],
				(unsigned long)ref_out_magic);
				ret = false;
				break;
			}
		if (i != (test->src_size / 8)) {
			i *= 8;
			fs4_info(test, "iter=%u batch=%u P mismatch at %d\n",
				 iter, b, i);
			t = test->src_size - i;
			if (t >= SBA_PQ_REF_SIZE)
				t = SBA_PQ_REF_SIZE;
			print_hex_dump(KERN_INFO, "dst_resp: ",
					DUMP_PREFIX_ADDRESS,
					8, 1, cmsg->dst_resp[b],
					SBA_RESP_SIZE, true);
			print_hex_dump(KERN_INFO, "dst: ",
					DUMP_PREFIX_ADDRESS,
					8, 1, cmsg->dst[b] + i,
					t, true);
			break;
		}
	}
	for (b = 0; b < test->batch_count; b++) {
		for (i = 0; i < (test->src_size / 8); i++)
			if (((u64 *)cmsg->dst1[b])[i] != ref_out1_magic) {
				fs4_info(test, "got=0x%lx exp=0x%lx\n",
				(unsigned long)((u64 *)cmsg->dst1[b])[i],
				(unsigned long)ref_out1_magic);
				ret = false;
				break;
			}
		if (i != (test->src_size / 8)) {
			i *= 8;
			fs4_info(test, "iter=%u batch=%u Q mismatch at %d\n",
				 iter, b, i);
			t = test->src_size - i;
			if (t >= SBA_PQ_REF_SIZE)
				t = SBA_PQ_REF_SIZE;
			print_hex_dump(KERN_INFO, "dst_resp: ",
					DUMP_PREFIX_ADDRESS,
					8, 1, cmsg->dst_resp[b],
					SBA_RESP_SIZE, true);
			print_hex_dump(KERN_INFO, "dst1: ",
					DUMP_PREFIX_ADDRESS,
					8, 1, cmsg->dst1[b] + i,
					t, true);
			break;
		}
	}

	return ret;
}

u64 raid6_gf_mul_uint64(int pos, u64 val)
{
	u64 ret = 0x0;
	u8 coef = raid6_gfexp[pos];
	u8 *valp = (u8 *)&val;
	u8 *retp = (u8 *)&ret;
	const u8 *mul_table = raid6_gfmul[coef];

	retp[0] = mul_table[valp[0]];
	retp[1] = mul_table[valp[1]];
	retp[2] = mul_table[valp[2]];
	retp[3] = mul_table[valp[3]];
	retp[4] = mul_table[valp[4]];
	retp[5] = mul_table[valp[5]];
	retp[6] = mul_table[valp[6]];
	retp[7] = mul_table[valp[7]];

	return ret;
}

/* Note: Must be called with test->lock held */
static int __sba_pq_exec(struct fs4_test *test)
{
	int rc = 0, i;
	unsigned long tout;
	struct mbox_chan *chan;
	struct fs4_test_msg *cmsg;
	u64 tmp, ref_out_magic, ref_out1_magic;
	unsigned long long input_bytes_count;
	unsigned int cur_split_size, src_size, iter = 0;
	unsigned int split_count, split_size, split_buf_count;
	s64 iter_usecs, min_usecs = 0, max_usecs = 0, avg_usecs = 0;
	unsigned long long iter_KBs, min_KBs = 0, max_KBs = 0, avg_KBs = 0;

	if (test->batch_count > FS4_MAX_BATCH_COUNT) {
		fs4_info(test, "batch_count should be less than %d\n",
			 (int)FS4_MAX_BATCH_COUNT);
		return -EINVAL;
	}
	if ((SBA_HW_BUF_SIZE/4) > test->min_split_size) {
		fs4_info(test, "min_split_size should be greater than %d\n",
			 (int)(SBA_HW_BUF_SIZE/4 - 1));
		return -EINVAL;
	}
	if (test->min_split_size > 0x10000) {
		fs4_info(test, "min_split_size can be upto 1MB or less\n");
		return -EINVAL;
	}
	if ((SBA_HW_BUF_SIZE/4) > test->src_size) {
		fs4_info(test, "src_size should be greater than %d\n",
			 (int)(SBA_HW_BUF_SIZE/4 - 1));
		return -EINVAL;
	}
	if (test->src_size & (SBA_PQ_REF_SIZE - 1)) {
		fs4_info(test, "src_size has to be multiple of %d\n",
			 (int)SBA_PQ_REF_SIZE);
		return -EINVAL;
	}
	if (test->src_count < 2) {
		fs4_info(test, "src_count cannot be less than 2\n");
		return -EINVAL;
	}
	if (test->update && (test->src_count != 4)) {
		fs4_info(test, "src_count has to be 4 for update mode\n");
		return -EINVAL;
	}
	if (test->src_count > SBA_PQ_MAX_SRC_COUNT) {
		fs4_info(test, "src_count cannot be greater than %d\n",
			 (int)SBA_PQ_MAX_SRC_COUNT);
		return -EINVAL;
	}
	if ((test->src_size * test->src_count) > 0x200000) {
		fs4_info(test, "memory requirement greater than 2M\n");
		return -EINVAL;
	}

	if (test->src_size <= test->min_split_size) {
		split_size = test->src_size;
	} else {
		split_size = test->src_size / test->mchans_count;
		if (split_size * test->mchans_count < test->src_size)
			split_size++;
		if (split_size <= test->min_split_size)
			split_size = test->min_split_size;
		if (split_size > test->src_size)
			split_size = test->src_size;
	}
	split_buf_count = split_size / SBA_HW_BUF_SIZE;
	if ((split_buf_count * SBA_HW_BUF_SIZE) < split_size)
		split_buf_count++;
	split_count = 0;
	src_size = test->src_size;
	while (src_size) {
		cur_split_size = min(src_size, split_size);
		split_count++;
		src_size -= cur_split_size;
	}

	fs4_info(test, "split_count=%u split_size=%u split_buf_count=%u\n",
		 split_count, split_size, split_buf_count);

	cmsg = __sba_pq_alloc(test,
			split_count, split_size, split_buf_count);
	if (!cmsg)
		return -ENOMEM;

	if (test->update) {
		tmp = *(u64 *)(cmsg->src[0] +
			       SBA_UPD_PQ_OLD_D * test->src_size);
		tmp = tmp ^ *(u64 *)(cmsg->src[0] +
				     SBA_UPD_PQ_NEW_D * test->src_size);
		ref_out_magic =	*(u64 *)(cmsg->src[0] +
					 SBA_UPD_PQ_OLD_P * test->src_size);
		ref_out_magic = ref_out_magic ^ tmp;
		ref_out1_magic = *(u64 *)(cmsg->src[0] +
					  SBA_UPD_PQ_OLD_Q * test->src_size);
		ref_out1_magic = ref_out1_magic ^
				 raid6_gf_mul_uint64(SBA_UPD_PQ_POS, tmp);
	} else {
		ref_out_magic = 0x0;
		ref_out1_magic = 0x0;
		for (i = 0; i < test->src_count; i++) {
			tmp = *(u64 *)(cmsg->src[0] + i * test->src_size);
			ref_out_magic = ref_out_magic ^ tmp;
			ref_out1_magic = ref_out1_magic ^
					 raid6_gf_mul_uint64(i, tmp);
		}
	}

	while (iter < test->iterations) {
		fs4_info(test, "iter=%u started", iter);

		input_bytes_count = test->src_size * test->src_count;
		input_bytes_count *= test->batch_count;
		for (i = 0; i < test->batch_count; i++) {
			if (test->update) {
				memcpy(cmsg->dst[i],
				       (cmsg->src[0] +
					SBA_UPD_PQ_OLD_P * test->src_size),
				       test->src_size);
				memcpy(cmsg->dst1[i],
				       (cmsg->src[0] +
					SBA_UPD_PQ_OLD_Q * test->src_size),
				       test->src_size);
			} else {
				memset(cmsg->dst[i], 0, test->src_size);
				memset(cmsg->dst1[i], 0, test->src_size);
			}
			memset(cmsg->dst_resp[i], 0,
				SBA_RESP_SIZE * split_count);
		}

		cmsg->start_ktime = ktime_get();
		cmsg->runtime_usecs = 0;

		if (test->software) {
			for (i = 0; i < test->batch_count; i++)
				__sba_software_pq(test,
				cmsg->dst[i], cmsg->dst1[i], cmsg->src[i]);
			cmsg->runtime_usecs =
			ktime_us_delta(ktime_get(), cmsg->start_ktime);
			goto skip_mailbox;
		}

		atomic_set(&cmsg->done_count, cmsg->msg_count);
		init_completion(&cmsg->done);

		rc = 0;
		for (i = 0; i < cmsg->bmsg_count; i++) {
			chan = test->mchans[test->chan];
			test->chan++;
			if (test->chan >= test->mchans_count)
				test->chan = 0;

			cmsg->bmsg[i].batch.msgs_queued = 0;
			rc = mbox_send_message(chan, &cmsg->bmsg[i]);
			if (rc < 0) {
				fs4_info(test, "iter=%u msg=%d send error %d\n",
					 iter, i, rc);
				break;
			}
			rc = 0;

			if (cmsg->bmsg[i].error < 0) {
				rc = cmsg->bmsg[i].error;
				break;
			}
		}
		if (rc < 0)
			break;

		if (test->poll) {
			while (atomic_read(&cmsg->done_count) > 0)
				for (i = 0; i < test->mchans_count; i++)
					mbox_client_peek_data(test->mchans[i]);
		} else {
			tout = (unsigned long)test->timeout * 1000;
			tout = msecs_to_jiffies(tout);
			tout = wait_for_completion_timeout(&cmsg->done, tout);
			if (!tout) {
				fs4_info(test, "iter=%u wait timeout\n", iter);
				rc = -ETIMEDOUT;
				break;
			}
		}

		rc = 0;
		for (i = 0; i < cmsg->msg_count; i++)
			if (cmsg->msg[i].error < 0) {
				fs4_info(test, "iter=%u msg=%d rx error\n",
					 iter, i);
				rc = cmsg->msg[i].error;
			}
		if (rc < 0)
			break;

skip_mailbox:
		if (test->verify) {
			if (!__sba_pq_verify(test, cmsg,
					iter, ref_out_magic, ref_out1_magic))
				break;
		}

		iter_usecs = cmsg->runtime_usecs;
		iter_KBs =
		fs4_test_KBs(cmsg->runtime_usecs, input_bytes_count);
		min_usecs = (iter == 0) ?
			    iter_usecs : min(min_usecs, iter_usecs);
		max_usecs = (iter == 0) ?
			    iter_usecs : max(max_usecs, iter_usecs);
		avg_usecs += iter_usecs;
		min_KBs = (iter == 0) ?
			  iter_KBs : min(min_KBs, iter_KBs);
		max_KBs = (iter == 0) ?
			  iter_KBs : max(max_KBs, iter_KBs);
		avg_KBs += iter_KBs;

		fs4_info(test, "iter=%u usecs=%ld KBs=%llu",
			 iter, (long)iter_usecs, iter_KBs);

		iter++;
	}

	__sba_pq_free(test, cmsg, split_count);

	if (iter) {
		avg_usecs = avg_usecs / iter;
		avg_KBs = avg_KBs / iter;
	}

	fs4_info(test, "completed %u/%u iterations\n",
		 iter, test->iterations);
	fs4_info(test, "min_usecs=%ld min_KBs=%llu",
		 (long)min_usecs, min_KBs);
	fs4_info(test, "max_usecs=%ld max_KBs=%llu",
		 (long)max_usecs, max_KBs);
	fs4_info(test, "avg_usecs=%ld avg_KBs=%llu",
		 (long)avg_usecs, avg_KBs);

	return rc;
}

#define FS4_TEST_DECLARE_DEV_ATTR_UINT(__name) \
static ssize_t __name##_show(struct device *dev, \
			     struct device_attribute *attr, \
			     char *buf) \
{ \
	ssize_t ret; \
	struct platform_device *pdev = to_platform_device(dev); \
	struct fs4_test *test = platform_get_drvdata(pdev); \
\
	mutex_lock(&test->lock); \
	ret = sprintf(buf, "%u\n", test->__name); \
	mutex_unlock(&test->lock); \
\
	return ret; \
} \
\
static ssize_t __name##_store(struct device *dev, \
			      struct device_attribute *attr, \
			      const char *buf, size_t count) \
{ \
	int rc; \
	unsigned int state; \
	struct platform_device *pdev = to_platform_device(dev); \
	struct fs4_test *test = platform_get_drvdata(pdev); \
\
	rc = kstrtouint(buf, 0, &state); \
	if (rc) \
		return rc; \
\
	if (state < 1) \
		return -EINVAL; \
\
	mutex_lock(&test->lock); \
	test->__name = state; \
	mutex_unlock(&test->lock); \
\
	return strnlen(buf, count); \
} \
\
static DEVICE_ATTR(__name, S_IRUGO | S_IWUSR, \
		   __name##_show, __name##_store)

FS4_TEST_DECLARE_DEV_ATTR_UINT(batch_count);
FS4_TEST_DECLARE_DEV_ATTR_UINT(min_split_size);
FS4_TEST_DECLARE_DEV_ATTR_UINT(src_size);
FS4_TEST_DECLARE_DEV_ATTR_UINT(src_count);
FS4_TEST_DECLARE_DEV_ATTR_UINT(iterations);
FS4_TEST_DECLARE_DEV_ATTR_UINT(timeout);

static void __fs4_do_test(struct fs4_test *test)
{
	int rc;

	if (test->start) {
		fs4_info(test, "test started\n");
		fs4_info(test, "iterations=%u timeout=%u verify=%u ",
			 test->iterations, test->timeout, test->verify);
		fs4_info(test, "verbose=%u poll=%u software=%u\n",
			 test->verbose, test->poll, test->software);
		fs4_info(test, "batch_count=%u min_split_size=%u\n",
			 test->batch_count, test->min_split_size);
		fs4_info(test, "update=%u src_size=%u src_count=%u\n",
			 test->update, test->src_size, test->src_count);
		rc = test->exec_func(test);
		test->start = 0;
		fs4_info(test, "test finished (error %d)\n", rc);
	}
}

#define FS4_TEST_DECLARE_DEV_ATTR_BOOL(__name, __change_func) \
static ssize_t __name##_show(struct device *dev, \
			     struct device_attribute *attr, \
			     char *buf) \
{ \
	ssize_t ret; \
	struct platform_device *pdev = to_platform_device(dev); \
	struct fs4_test *test = platform_get_drvdata(pdev); \
\
	mutex_lock(&test->lock); \
	ret = sprintf(buf, "%u\n", test->__name); \
	mutex_unlock(&test->lock); \
\
	return ret; \
} \
\
static ssize_t __name##_store(struct device *dev, \
			      struct device_attribute *attr, \
			      const char *buf, size_t count) \
{ \
	int rc; \
	unsigned int state; \
	struct platform_device *pdev = to_platform_device(dev); \
	struct fs4_test *test = platform_get_drvdata(pdev); \
	void (*cfunc)(struct fs4_test *) = __change_func; \
\
	rc = kstrtouint(buf, 0, &state); \
	if (rc) \
		return rc; \
\
	if ((state != 0) && (state != 1)) \
		return -EINVAL; \
\
	mutex_lock(&test->lock); \
	test->__name = state; \
	if (cfunc) \
		cfunc(test); \
	mutex_unlock(&test->lock); \
\
	return strnlen(buf, count); \
} \
\
static DEVICE_ATTR(__name, S_IRUGO | S_IWUSR, \
		   __name##_show, __name##_store)

FS4_TEST_DECLARE_DEV_ATTR_BOOL(update, NULL);
FS4_TEST_DECLARE_DEV_ATTR_BOOL(verify, NULL);
FS4_TEST_DECLARE_DEV_ATTR_BOOL(verbose, NULL);
FS4_TEST_DECLARE_DEV_ATTR_BOOL(poll, NULL);
FS4_TEST_DECLARE_DEV_ATTR_BOOL(software, NULL);
FS4_TEST_DECLARE_DEV_ATTR_BOOL(start, __fs4_do_test);

static const struct of_device_id fs4_test_of_match[] = {
{ .compatible = "brcm,fs4-test-spu2", .data = __spu2_exec, },
{ .compatible = "brcm,fs4-test-sba-memcpy", .data = __sba_memcpy_exec, },
{ .compatible = "brcm,fs4-test-sba-xor", .data = __sba_xor_exec, },
{ .compatible = "brcm,fs4-test-sba-pq", .data = __sba_pq_exec, },
{},};
MODULE_DEVICE_TABLE(of, fs4_test_of_match);

static int fs4_test_probe(struct platform_device *pdev)
{
	int i, ret = 0;
	struct of_phandle_args args;
	const struct of_device_id *of_id;
	struct platform_device *mbox_pdev;
	struct resource *iomem;
	struct fs4_test *test;

	of_id = of_match_node(fs4_test_of_match, pdev->dev.of_node);
	if (!of_id)
		return -ENODEV;

	test = devm_kzalloc(&pdev->dev, sizeof(*test), GFP_KERNEL);
	if (!test)
		return -ENOMEM;

	test->dev = &pdev->dev;
	test->exec_func = of_id->data;
	platform_set_drvdata(pdev, test);

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (iomem) {
		test->regs = devm_ioremap_resource(&pdev->dev, iomem);
		if (IS_ERR(test->regs)) {
			ret = PTR_ERR(test->regs);
			dev_err(&pdev->dev, "Failed to remap regs: %d\n", ret);
			return ret;
		}
		test->engine_count =
			resource_size(iomem) / FS4_ENGINE_REG_SIZE;
	} else {
		test->regs = NULL;
		test->engine_count = 0;
	}

	test->client.dev		= &pdev->dev;
	test->client.rx_callback	= fs4_test_receive_message;
	test->client.tx_block		= true;
	test->client.knows_txdone	= false;
	test->client.tx_tout		= 0;

	ret = device_create_file(&pdev->dev, &dev_attr_batch_count);
	if (ret < 0)
		goto fail;
	ret = device_create_file(&pdev->dev, &dev_attr_min_split_size);
	if (ret < 0)
		goto fail_remove_attr_batch_count;
	ret = device_create_file(&pdev->dev, &dev_attr_src_size);
	if (ret < 0)
		goto fail_remove_attr_min_split_size;
	ret = device_create_file(&pdev->dev, &dev_attr_src_count);
	if (ret < 0)
		goto fail_remove_attr_src_size;
	ret = device_create_file(&pdev->dev, &dev_attr_iterations);
	if (ret < 0)
		goto fail_remove_attr_src_count;
	ret = device_create_file(&pdev->dev, &dev_attr_timeout);
	if (ret < 0)
		goto fail_remove_attr_iterations;
	ret = device_create_file(&pdev->dev, &dev_attr_update);
	if (ret < 0)
		goto fail_remove_attr_timeout;
	ret = device_create_file(&pdev->dev, &dev_attr_verify);
	if (ret < 0)
		goto fail_remove_attr_update;
	ret = device_create_file(&pdev->dev, &dev_attr_verbose);
	if (ret < 0)
		goto fail_remove_attr_verify;
	ret = device_create_file(&pdev->dev, &dev_attr_poll);
	if (ret < 0)
		goto fail_remove_attr_verbose;
	ret = device_create_file(&pdev->dev, &dev_attr_software);
	if (ret < 0)
		goto fail_remove_attr_poll;
	ret = device_create_file(&pdev->dev, &dev_attr_start);
	if (ret < 0)
		goto fail_remove_attr_software;

	ret = of_count_phandle_with_args(pdev->dev.of_node,
					 "mboxes", "#mbox-cells");
	if ((ret <= 0) || (ret > FS4_MAX_CHANNELS)) {
		ret = -ENODEV;
		goto fail_remove_attr_start;
	}

	ret = of_parse_phandle_with_args(pdev->dev.of_node,
					 "mboxes", "#mbox-cells", 0, &args);
	if (ret)
		goto fail_remove_attr_start;
	mbox_pdev = of_find_device_by_node(args.np);
	of_node_put(args.np);
	if (!mbox_pdev) {
		ret = -ENODEV;
		goto fail_remove_attr_start;
	}
	test->mbox_dev = &mbox_pdev->dev;

	test->mchans_count = ret;
	for (i = 0; i < test->mchans_count; i++) {
		test->mchans[i] = mbox_request_channel(&test->client, i);
		if (!IS_ERR(test->mchans[i]))
			continue;
		ret = PTR_ERR(test->mchans[i]);
		test->mchans[i] = NULL;
		goto fail_free_channels;
	}

	mutex_init(&test->lock);

	/* Default values of test parameters */
	test->chan = 0;
	test->batch_count = 32;
	test->min_split_size = 4096;
	test->src_size = 8192;
	test->src_count = 3;
	test->iterations = 50;
	test->timeout = 300;
	test->update = 0;
	test->verify = 0;
	test->software = 0;
	test->verbose = 1;
	test->poll = 0;
	test->start = 0;

	fs4_info(test, "fs4 test ready with %d channels",
		 test->mchans_count);

	return 0;

fail_free_channels:
	for (i = 0; i < test->mchans_count; i++)
		if (test->mchans[i]) {
			mbox_free_channel(test->mchans[i]);
			test->mchans[i] = NULL;
		}
	test->mchans_count = 0;
fail_remove_attr_start:
	device_remove_file(&pdev->dev, &dev_attr_start);
fail_remove_attr_software:
	device_remove_file(&pdev->dev, &dev_attr_software);
fail_remove_attr_poll:
	device_remove_file(&pdev->dev, &dev_attr_poll);
fail_remove_attr_verbose:
	device_remove_file(&pdev->dev, &dev_attr_verbose);
fail_remove_attr_verify:
	device_remove_file(&pdev->dev, &dev_attr_verify);
fail_remove_attr_update:
	device_remove_file(&pdev->dev, &dev_attr_update);
fail_remove_attr_timeout:
	device_remove_file(&pdev->dev, &dev_attr_timeout);
fail_remove_attr_iterations:
	device_remove_file(&pdev->dev, &dev_attr_iterations);
fail_remove_attr_src_count:
	device_remove_file(&pdev->dev, &dev_attr_src_count);
fail_remove_attr_src_size:
	device_remove_file(&pdev->dev, &dev_attr_src_size);
fail_remove_attr_min_split_size:
	device_remove_file(&pdev->dev, &dev_attr_min_split_size);
fail_remove_attr_batch_count:
	device_remove_file(&pdev->dev, &dev_attr_batch_count);
fail:
	return ret;
}

static int fs4_test_remove(struct platform_device *pdev)
{
	int i;
	struct fs4_test *test = platform_get_drvdata(pdev);

	for (i = 0; i < test->mchans_count; i++)
		if (test->mchans[i]) {
			mbox_free_channel(test->mchans[i]);
			test->mchans[i] = NULL;
		}
	test->mchans_count = 0;

	device_remove_file(&pdev->dev, &dev_attr_start);
	device_remove_file(&pdev->dev, &dev_attr_software);
	device_remove_file(&pdev->dev, &dev_attr_poll);
	device_remove_file(&pdev->dev, &dev_attr_verbose);
	device_remove_file(&pdev->dev, &dev_attr_verify);
	device_remove_file(&pdev->dev, &dev_attr_update);
	device_remove_file(&pdev->dev, &dev_attr_timeout);
	device_remove_file(&pdev->dev, &dev_attr_iterations);
	device_remove_file(&pdev->dev, &dev_attr_src_count);
	device_remove_file(&pdev->dev, &dev_attr_src_size);
	device_remove_file(&pdev->dev, &dev_attr_min_split_size);
	device_remove_file(&pdev->dev, &dev_attr_batch_count);

	return 0;
}

static struct platform_driver fs4_test_driver = {
	.driver = {
		.name = "brcm-fs4-test",
		.of_match_table = fs4_test_of_match,
	},
	.probe		= fs4_test_probe,
	.remove		= fs4_test_remove,
};
module_platform_driver(fs4_test_driver);

MODULE_AUTHOR("Anup Patel <anup.patel@broadcom.com>");
MODULE_DESCRIPTION("Broadcom FlexSparx4 Test Client");
MODULE_LICENSE("GPL v2");
