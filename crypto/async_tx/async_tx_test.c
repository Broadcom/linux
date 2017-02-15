/*
 * Copyright (C) 2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/async_tx.h>
#include <linux/completion.h>
#include <linux/gfp.h>
#include <linux/kthread.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/raid/pq.h>
#include <linux/slab.h>

#undef pr
#define pr(fmt, args...) pr_info("async_tx_test: " fmt, ##args)

#undef pr_err
#define pr_err(fmt, args...) pr_info("async_tx_test: error: " fmt, ##args)

#define MIN_BLOCK_SIZE	8
#define MAX_BLOCK_SIZE	PAGE_SIZE
#define MIN_DISKS	2
#define MAX_DISKS	253

static unsigned int thread_count = 1;
module_param(thread_count, uint, 0644);
MODULE_PARM_DESC(thread_count,
"Number of threads, Should be atleast 1");

static unsigned int disk_block_size = PAGE_SIZE;
module_param(disk_block_size, uint, 0644);
MODULE_PARM_DESC(disk_block_size,
"Size of disk block, Should be between 8 to PAGE_SIZE and multiple of 8");

static unsigned int disk_count = 4;
module_param(disk_count, uint, 0644);
MODULE_PARM_DESC(disk_count,
"Number of test disks, Should be between 2 to 256");

static unsigned int request_count = 1;
module_param(request_count, uint, 0644);
MODULE_PARM_DESC(request_count,
"Number of request submitted in one iteration, Should be atleast 1");

static unsigned int iteration_count = 1;
module_param(iteration_count, uint, 0644);
MODULE_PARM_DESC(iteration_count,
"Number of iterations, Should be atleast 1");

static int async_tx_test_type_set(const char *val,
				 const struct kernel_param *kp);
static int async_tx_test_type_get(char *val,
				 const struct kernel_param *kp);
static const struct kernel_param_ops type_ops = {
	.set = async_tx_test_type_set,
	.get = async_tx_test_type_get,
};
static char test_type[20] = "pq";
module_param_cb(type, &type_ops, test_type, 0644);
MODULE_PARM_DESC(type, "Type of test (default: pq)");

static bool verbose;
module_param(verbose, bool, 0644);
MODULE_PARM_DESC(verbose, "Print more info (default: off)");

static int async_tx_test_run_set(const char *val,
				 const struct kernel_param *kp);
static int async_tx_test_run_get(char *val,
				 const struct kernel_param *kp);
static const struct kernel_param_ops run_ops = {
	.set = async_tx_test_run_set,
	.get = async_tx_test_run_get,
};
static bool test_run;
module_param_cb(run, &run_ops, &test_run, 0644);
MODULE_PARM_DESC(run, "Run the test (default: false)");

static DEFINE_MUTEX(test_lock);

struct async_tx_test;

struct async_tx_test_request {
	unsigned int num;
	struct async_tx_test *test;
	struct page *disk[MAX_DISKS+2];
	struct page *p;
	struct page *q;
	addr_conv_t addr_conv[MAX_DISKS+2];
};

struct async_tx_test_ops {
	char name[20];
	unsigned int min_disk_count;
	unsigned int max_disk_count;
	unsigned int (*io_size)(struct async_tx_test *test);
	int (*prep_input)(struct async_tx_test_request *req);
	int (*prep_output)(struct async_tx_test_request *req);
	int (*submit)(struct async_tx_test_request *req);
	int (*verify_output)(struct async_tx_test_request *req);
	void (*cleanup)(struct async_tx_test_request *req);
};

struct async_tx_test {
	/* Test parameters */
	bool verbose;
	unsigned int thread_count;
	unsigned int block_size;
	unsigned int disk_count;
	unsigned int request_count;
	unsigned int iteration_count;
	struct async_tx_test_ops *ops;
	/* Test iteration data */
	struct completion *done;
	struct async_tx_test_request *reqs;
	ktime_t iter_start_ktime;
	s64 iter_runtime_usecs;
	atomic_t iter_done_count;
	struct completion iter_done;
	/* Test thread data */
	unsigned int num;
	struct task_struct *task;
	unsigned long long min_KBs;
	unsigned long long max_KBs;
	unsigned long long avg_KBs;
	unsigned long long min_IOPs;
	unsigned long long max_IOPs;
	unsigned long long avg_IOPs;
	atomic_t *done_count;
};

static unsigned long long async_tx_test_persec(s64 runtime, unsigned int val)
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

static void async_tx_test_callback(void *param)
{
	struct async_tx_test_request *req = param;
	struct async_tx_test *test;

	if (!req || !req->test)
		return;
	test = req->test;

	if (atomic_dec_return(&test->iter_done_count))
		return;

	if (!test->iter_runtime_usecs)
		test->iter_runtime_usecs =
			ktime_us_delta(ktime_get(), test->iter_start_ktime);

	complete(&test->iter_done);
}

#define ASYNC_TX_TEST_REF0	0xa5a5a5a5a5a5a5a5ULL
#define ASYNC_TX_TEST_REF1	0x5a5a5a5a5a5a5a5aULL
#define ASYNC_TX_TEST_REF2	0x1020304050607080ULL
#define ASYNC_TX_TEST_REF3	0x1122334455667788ULL

static u64 async_tx_test_ref64(unsigned int disk_num)
{
	switch (disk_num & 0x3) {
	case 0:
		return ASYNC_TX_TEST_REF0;
	case 1:
		return ASYNC_TX_TEST_REF1;
	case 2:
		return ASYNC_TX_TEST_REF2;
	case 3:
		return ASYNC_TX_TEST_REF3;
	}

	return 0;
}

u64 async_tx_test_gfmul64(int pos, u64 val)
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

static void async_tx_test_dummy_cleanup(struct async_tx_test_request *req)
{
	/* Nothing to do here. */
}

static unsigned int memcpy_test_io_size(struct async_tx_test *test)
{
	return test->block_size;
}

static int memcpy_test_prep_input(struct async_tx_test_request *req)
{
	u64 *data;
	unsigned int i;
	struct async_tx_test *test = req->test;
	u64 ref = async_tx_test_ref64(req->num);

	data = (u64 *)page_address(req->disk[0]);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		data[i] = ref;

	return 0;
}

static int memcpy_test_prep_output(struct async_tx_test_request *req)
{
	struct async_tx_test *test = req->test;

	memset(page_address(req->p), 0, test->block_size);

	return 0;
}

static int memcpy_test_submit(struct async_tx_test_request *req)
{
	struct async_submit_ctl submit;
	struct dma_async_tx_descriptor *tx;
	struct async_tx_test *test = req->test;

	init_async_submit(&submit, ASYNC_TX_ACK, NULL,
			  async_tx_test_callback, req, req->addr_conv);
	tx = async_memcpy(req->p, req->disk[0], 0, 0,
			  test->block_size, &submit);
	async_tx_issue_pending(tx);

	return 0;
}

static int memcpy_test_verify_output(struct async_tx_test_request *req)
{
	u64 *data;
	unsigned int i;
	struct async_tx_test *test = req->test;
	u64 out_ref = async_tx_test_ref64(req->num);

	data = (u64 *)page_address(req->p);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		if (data[i] != out_ref)
			return 0;

	return 1;
}

static unsigned int xor_test_io_size(struct async_tx_test *test)
{
	return test->block_size * test->disk_count;
}

static int xor_test_prep_input(struct async_tx_test_request *req)
{
	u64 *data;
	unsigned int i, j;
	struct async_tx_test *test = req->test;

	for (i = 0; i < test->disk_count; i++) {
		data = (u64 *)page_address(req->disk[i]);
		for (j = 0; j < test->block_size / sizeof(u64); j++)
			data[j] = async_tx_test_ref64(i);
	}

	return 0;
}

static int xor_test_prep_output(struct async_tx_test_request *req)
{
	struct async_tx_test *test = req->test;

	memset(page_address(req->p), 0, test->block_size);

	return 0;
}

static int xor_test_submit(struct async_tx_test_request *req)
{
	struct async_submit_ctl submit;
	struct dma_async_tx_descriptor *tx;
	struct async_tx_test *test = req->test;

	init_async_submit(&submit, ASYNC_TX_ACK, NULL,
			  async_tx_test_callback, req, req->addr_conv);
	tx = async_xor(req->p, req->disk, 0, test->disk_count,
		       test->block_size, &submit);
	async_tx_issue_pending(tx);

	return 0;
}

static int xor_test_verify_output(struct async_tx_test_request *req)
{
	u64 *data;
	u64 out_ref;
	unsigned int i;
	struct async_tx_test *test = req->test;

	out_ref = 0x0;
	for (i = 0; i < test->disk_count; i++)
		out_ref = out_ref ^ async_tx_test_ref64(i);

	data = (u64 *)page_address(req->p);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		if (data[i] != out_ref)
			return 0;

	return 1;
}

static unsigned int pq_test_io_size(struct async_tx_test *test)
{
	return test->block_size * test->disk_count;
}

static int pq_test_prep_input(struct async_tx_test_request *req)
{
	u64 *data;
	unsigned int i, j;
	struct async_tx_test *test = req->test;

	for (i = 0; i < test->disk_count; i++) {
		data = (u64 *)page_address(req->disk[i]);
		for (j = 0; j < test->block_size / sizeof(u64); j++)
			data[j] = async_tx_test_ref64(i);
	}

	return 0;
}

static int pq_test_prep_output(struct async_tx_test_request *req)
{
	struct async_tx_test *test = req->test;

	memset(page_address(req->p), 0, test->block_size);
	memset(page_address(req->q), 0, test->block_size);

	return 0;
}

static int pq_test_submit(struct async_tx_test_request *req)
{
	struct async_submit_ctl submit;
	struct dma_async_tx_descriptor *tx;
	struct async_tx_test *test = req->test;

	init_async_submit(&submit, ASYNC_TX_ACK, NULL,
			  async_tx_test_callback, req, req->addr_conv);
	tx = async_gen_syndrome(req->disk, 0, test->disk_count + 2,
				test->block_size, &submit);
	async_tx_issue_pending(tx);

	return 0;
}

static int pq_test_verify_output(struct async_tx_test_request *req)
{
	u64 *data;
	unsigned int i;
	u64 ref, out_ref, out_gf_ref;
	struct async_tx_test *test = req->test;

	out_ref = out_gf_ref = 0x0;
	for (i = 0; i < test->disk_count; i++) {
		ref = async_tx_test_ref64(i);
		out_ref = out_ref ^ ref;
		out_gf_ref = out_gf_ref ^ async_tx_test_gfmul64(i, ref);
	}

	data = (u64 *)page_address(req->p);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		if (data[i] != out_ref)
			return 0;

	data = (u64 *)page_address(req->q);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		if (data[i] != out_gf_ref)
			return 0;

	return 1;
}

#define UPDATE_PQ_DISKS		16
#define UPDATE_PQ_OLD_DATA	0
#define UPDATE_PQ_NEW_DATA	1
#define UPDATE_PQ_P		2
#define UPDATE_PQ_Q		3

static unsigned int update_pq_test_io_size(struct async_tx_test *test)
{
	return test->block_size * 4;
}

static int update_pq_test_prep_input(struct async_tx_test_request *req)
{
	u64 *data;
	unsigned int i;
	struct async_tx_test *test = req->test;

	data = (u64 *)page_address(req->disk[UPDATE_PQ_OLD_DATA]);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		data[i] = async_tx_test_ref64(UPDATE_PQ_OLD_DATA);

	data = (u64 *)page_address(req->disk[UPDATE_PQ_NEW_DATA]);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		data[i] = async_tx_test_ref64(UPDATE_PQ_NEW_DATA);

	return 0;
}

static int update_pq_test_prep_output(struct async_tx_test_request *req)
{
	u64 *data;
	unsigned int i;
	struct async_tx_test *test = req->test;

	data = (u64 *)page_address(req->p);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		data[i] = async_tx_test_ref64(UPDATE_PQ_P);

	data = (u64 *)page_address(req->q);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		data[i] = async_tx_test_ref64(UPDATE_PQ_Q);

	return 0;
}

static int update_pq_test_submit(struct async_tx_test_request *req)
{
	int i, pos = req->num % UPDATE_PQ_DISKS;
	struct async_submit_ctl submit;
	struct dma_async_tx_descriptor *tx;
	struct async_tx_test *test = req->test;
	struct page *srcs[UPDATE_PQ_DISKS + 2];

	for (i = 0; i < UPDATE_PQ_DISKS; i++)
		srcs[i] = NULL;

	srcs[pos] = req->disk[UPDATE_PQ_OLD_DATA];
	srcs[UPDATE_PQ_DISKS] = req->p;
	srcs[UPDATE_PQ_DISKS + 1] = req->q;

	init_async_submit(&submit, ASYNC_TX_PQ_XOR_DST, NULL,
			  NULL, NULL, req->addr_conv);
	tx = async_gen_syndrome(srcs, 0, UPDATE_PQ_DISKS + 2,
				test->block_size, &submit);

	srcs[pos] = req->disk[UPDATE_PQ_NEW_DATA];

	init_async_submit(&submit, ASYNC_TX_PQ_XOR_DST|ASYNC_TX_ACK, tx,
			  async_tx_test_callback, req, req->addr_conv);
	tx = async_gen_syndrome(srcs, 0, UPDATE_PQ_DISKS + 2,
				test->block_size, &submit);

	async_tx_issue_pending(tx);

	return 0;
}

static int update_pq_test_verify_output(struct async_tx_test_request *req)
{
	u64 *data;
	u64 ref, out_ref, out_gf_ref;
	int i, pos = req->num % UPDATE_PQ_DISKS;
	struct async_tx_test *test = req->test;

	out_ref = async_tx_test_ref64(UPDATE_PQ_P);
	out_gf_ref = async_tx_test_ref64(UPDATE_PQ_Q);

	ref = async_tx_test_ref64(UPDATE_PQ_OLD_DATA);
	out_ref = out_ref ^ ref;
	out_gf_ref = out_gf_ref ^ async_tx_test_gfmul64(pos, ref);

	ref = async_tx_test_ref64(UPDATE_PQ_NEW_DATA);
	out_ref = out_ref ^ ref;
	out_gf_ref = out_gf_ref ^ async_tx_test_gfmul64(pos, ref);

	data = (u64 *)page_address(req->p);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		if (data[i] != out_ref)
			return 0;

	data = (u64 *)page_address(req->q);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		if (data[i] != out_gf_ref)
			return 0;

	return 1;
}

static unsigned int recov_datap_test_io_size(struct async_tx_test *test)
{
	return test->block_size * test->disk_count;
}

static int recov_datap_test_prep_input(struct async_tx_test_request *req)
{
	u64 *data;
	unsigned int i, j;
	u64 ref, out_ref, out_gf_ref;
	struct async_tx_test *test = req->test;

	out_ref = out_gf_ref = 0x0;
	for (i = 0; i < test->disk_count; i++) {
		ref = async_tx_test_ref64(i);
		out_ref = out_ref ^ ref;
		out_gf_ref = out_gf_ref ^ async_tx_test_gfmul64(i, ref);
		data = (u64 *)page_address(req->disk[i]);
		for (j = 0; j < test->block_size / sizeof(u64); j++)
			data[j] = ref;
	}

	data = (u64 *)page_address(req->p);
	for (j = 0; j < test->block_size / sizeof(u64); j++)
		data[j] = out_ref;

	data = (u64 *)page_address(req->q);
	for (j = 0; j < test->block_size / sizeof(u64); j++)
		data[j] = out_gf_ref;

	return 0;
}

static int recov_datap_test_prep_output(struct async_tx_test_request *req)
{
	struct async_tx_test *test = req->test;
	unsigned int faila = req->num % test->disk_count;

	memset(page_address(req->p), 0, test->block_size);
	memset(page_address(req->disk[faila]), 0, test->block_size);

	return 0;
}

static int recov_datap_test_submit(struct async_tx_test_request *req)
{
	struct async_submit_ctl submit;
	struct dma_async_tx_descriptor *tx;
	struct async_tx_test *test = req->test;
	unsigned int faila = req->num % test->disk_count;

	init_async_submit(&submit, ASYNC_TX_ACK, NULL,
			  async_tx_test_callback, req, req->addr_conv);
	tx = async_raid6_datap_recov(test->disk_count + 2, test->block_size,
				     faila, req->disk, &submit);
	async_tx_issue_pending(tx);

	return 0;
}

static int recov_datap_test_verify_output(struct async_tx_test_request *req)
{
	u64 *data;
	unsigned int i;
	u64 ref, out_ref, out_data_ref;
	struct async_tx_test *test = req->test;
	unsigned int faila = req->num % test->disk_count;

	out_ref = 0x0;
	for (i = 0; i < test->disk_count; i++) {
		ref = async_tx_test_ref64(i);
		out_ref = out_ref ^ ref;
	}
	out_data_ref = async_tx_test_ref64(faila);

	data = (u64 *)page_address(req->p);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		if (data[i] != out_ref)
			return 0;

	data = (u64 *)page_address(req->disk[faila]);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		if (data[i] != out_data_ref)
			return 0;

	return 1;
}

static unsigned int recov_2data_test_io_size(struct async_tx_test *test)
{
	return test->block_size * test->disk_count;
}

static int recov_2data_test_prep_input(struct async_tx_test_request *req)
{
	u64 *data;
	unsigned int i, j;
	u64 ref, out_ref, out_gf_ref;
	struct async_tx_test *test = req->test;

	out_ref = out_gf_ref = 0x0;
	for (i = 0; i < test->disk_count; i++) {
		ref = async_tx_test_ref64(i);
		out_ref = out_ref ^ ref;
		out_gf_ref = out_gf_ref ^ async_tx_test_gfmul64(i, ref);
		data = (u64 *)page_address(req->disk[i]);
		for (j = 0; j < test->block_size / sizeof(u64); j++)
			data[j] = ref;
	}

	data = (u64 *)page_address(req->p);
	for (j = 0; j < test->block_size / sizeof(u64); j++)
		data[j] = out_ref;

	data = (u64 *)page_address(req->q);
	for (j = 0; j < test->block_size / sizeof(u64); j++)
		data[j] = out_gf_ref;

	return 0;
}

static int recov_2data_test_prep_output(struct async_tx_test_request *req)
{
	struct async_tx_test *test = req->test;
	unsigned int faila = req->num % test->disk_count;
	unsigned int failb = (req->num + 1) % test->disk_count;

	memset(page_address(req->disk[faila]), 0, test->block_size);
	memset(page_address(req->disk[failb]), 0, test->block_size);

	return 0;
}

static int recov_2data_test_submit(struct async_tx_test_request *req)
{
	struct async_submit_ctl submit;
	struct dma_async_tx_descriptor *tx;
	struct async_tx_test *test = req->test;
	unsigned int faila = req->num % test->disk_count;
	unsigned int failb = (req->num + 1) % test->disk_count;

	init_async_submit(&submit, ASYNC_TX_ACK, NULL,
			  async_tx_test_callback, req, req->addr_conv);
	tx = async_raid6_2data_recov(test->disk_count + 2, test->block_size,
				     faila, failb, req->disk, &submit);
	async_tx_issue_pending(tx);

	return 0;
}

static int recov_2data_test_verify_output(struct async_tx_test_request *req)
{
	u64 *data;
	unsigned int i;
	u64 out_data_ref, out_data1_ref;
	struct async_tx_test *test = req->test;
	unsigned int faila = req->num % test->disk_count;
	unsigned int failb = (req->num + 1) % test->disk_count;

	out_data_ref = async_tx_test_ref64(faila);
	out_data1_ref = async_tx_test_ref64(failb);

	data = (u64 *)page_address(req->disk[faila]);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		if (data[i] != out_data_ref)
			return 0;

	data = (u64 *)page_address(req->disk[failb]);
	for (i = 0; i < test->block_size / sizeof(u64); i++)
		if (data[i] != out_data1_ref)
			return 0;

	return 1;
}

static struct async_tx_test_ops ops_table[] = {
	{
		.name = "memcpy",
		.min_disk_count = 1,
		.max_disk_count = 1,
		.io_size = memcpy_test_io_size,
		.prep_input = memcpy_test_prep_input,
		.prep_output = memcpy_test_prep_output,
		.submit = memcpy_test_submit,
		.verify_output = memcpy_test_verify_output,
		.cleanup = async_tx_test_dummy_cleanup,
	},
	{
		.name = "xor",
		.min_disk_count = 1,
		.max_disk_count = MAX_DISKS,
		.io_size = xor_test_io_size,
		.prep_input = xor_test_prep_input,
		.prep_output = xor_test_prep_output,
		.submit = xor_test_submit,
		.verify_output = xor_test_verify_output,
		.cleanup = async_tx_test_dummy_cleanup,
	},
	{
		.name = "pq",
		.min_disk_count = 1,
		.max_disk_count = MAX_DISKS,
		.io_size = pq_test_io_size,
		.prep_input = pq_test_prep_input,
		.prep_output = pq_test_prep_output,
		.submit = pq_test_submit,
		.verify_output = pq_test_verify_output,
		.cleanup = async_tx_test_dummy_cleanup,
	},
	{
		.name = "update_pq",
		.min_disk_count = 2,
		.max_disk_count = 2,
		.io_size = update_pq_test_io_size,
		.prep_input = update_pq_test_prep_input,
		.prep_output = update_pq_test_prep_output,
		.submit = update_pq_test_submit,
		.verify_output = update_pq_test_verify_output,
		.cleanup = async_tx_test_dummy_cleanup,
	},
	{
		.name = "recov_datap",
		.min_disk_count = 2,
		.max_disk_count = MAX_DISKS,
		.io_size = recov_datap_test_io_size,
		.prep_input = recov_datap_test_prep_input,
		.prep_output = recov_datap_test_prep_output,
		.submit = recov_datap_test_submit,
		.verify_output = recov_datap_test_verify_output,
		.cleanup = async_tx_test_dummy_cleanup,
	},
	{
		.name = "recov_2data",
		.min_disk_count = 2,
		.max_disk_count = MAX_DISKS,
		.io_size = recov_2data_test_io_size,
		.prep_input = recov_2data_test_prep_input,
		.prep_output = recov_2data_test_prep_output,
		.submit = recov_2data_test_submit,
		.verify_output = recov_2data_test_verify_output,
		.cleanup = async_tx_test_dummy_cleanup,
	},
};

static struct async_tx_test_ops *async_tx_test_find(const char *name)
{
	int i;
	struct async_tx_test_ops *ops;

	for (i = 0; i < ARRAY_SIZE(ops_table); i++) {
		ops = &ops_table[i];
		if (!strncmp(ops->name, name, sizeof(ops->name)))
			return ops;
	}

	return NULL;
}

static int async_tx_test_thread_main(void *data)
{
	int ret = 0;
	unsigned int i, r, io_size;
	struct async_tx_test_request *req;
	unsigned long long input_bytes_count;
	s64 iter_usecs, min_usecs = 0, max_usecs = 0, avg_usecs = 0;
	unsigned long long iter_KBs, min_KBs = 0, max_KBs = 0, avg_KBs = 0;
	struct async_tx_test *test = data;

	/* Print thread start bannner */
	pr("thread=%u started\n", test->num);

	/* Allocate request array */
	test->reqs = kcalloc(test->request_count,
			     sizeof(*test->reqs), GFP_KERNEL);
	if (!test->reqs) {
		ret = -ENOMEM;
		pr_err("failed to alloc requests for thread=%u\n",
			test->num);
		goto fail;
	}

	/* Allocate all request disk data */
	for (r = 0; r < test->request_count; r++) {
		req = &test->reqs[r];

		req->num = r;
		req->test = test;
		for (i = 0; i < (test->disk_count + 2); i++) {
			req->disk[i] = alloc_page(GFP_KERNEL);
			if (req->disk[i])
				continue;

			ret = -ENOMEM;
			pr_err("alloc page failed thread=%u req=%u disk=%u\n",
				test->num, r, i);
			goto free_reqs_data;
		}
		req->p = req->disk[test->disk_count];
		req->q = req->disk[test->disk_count + 1];
	}

	/* Calculate IO size and total input bytes */
	io_size = test->ops->io_size(test);
	input_bytes_count = io_size;
	input_bytes_count *= test->request_count;

	/* Prepare input data */
	for (r = 0; r < test->request_count; r++) {
		ret = test->ops->prep_input(&test->reqs[r]);
		if (!ret)
			continue;

		pr_err("prepare input failed for thread=%u req=%u\n",
			test->num, r);
		goto cleanup_reqs;
	}

	/* Execute each iteration */
	for (i = 0; i < test->iteration_count; i++) {
		/* Prepare output data for iteration */
		for (r = 0; r < test->request_count; r++) {
			ret = test->ops->prep_output(&test->reqs[r]);
			if (!ret)
				continue;

			pr_err("prepare output fail thread=%u iter=%u req=%u\n",
			   test->num, i, r);
			goto cleanup_reqs;
		}

		/* Start of iteration */
		test->iter_start_ktime = ktime_get();
		test->iter_runtime_usecs = 0;
		atomic_set(&test->iter_done_count, test->request_count);
		init_completion(&test->iter_done);

		/* Submit all request */
		for (r = 0; r < test->request_count; r++) {
			ret = test->ops->submit(&test->reqs[r]);
			if (!ret)
				continue;

			pr_err("submit failed for thread=%u iter=%u req=%u\n",
				test->num, i, r);
			goto cleanup_reqs;
		}

		/* Wait for all request to complete */
		wait_for_completion(&test->iter_done);

		/* Verify output data for iteration */
		for (r = 0; r < test->request_count; r++) {
			if (test->ops->verify_output(&test->reqs[r]))
				continue;

			ret = -EIO;
			pr_err("verify failed thread=%u iter=%u req=%u\n",
				test->num, i, r);
			goto cleanup_reqs;
		}

		/* Update stats */
		iter_usecs = test->iter_runtime_usecs;
		iter_KBs = async_tx_test_persec(test->iter_runtime_usecs,
						input_bytes_count >> 10);
		min_usecs = (i == 0) ?
			    iter_usecs : min(min_usecs, iter_usecs);
		max_usecs = (i == 0) ?
			    iter_usecs : max(max_usecs, iter_usecs);
		avg_usecs += iter_usecs;
		min_KBs = (i == 0) ?
			  iter_KBs : min(min_KBs, iter_KBs);
		max_KBs = (i == 0) ?
			  iter_KBs : max(max_KBs, iter_KBs);
		avg_KBs += iter_KBs;

		/* Print iteration summary */
		if (test->verbose)
			pr("thread=%u cpu=%d iter=%u usecs=%ld KBs=%llu\n",
			   test->num, smp_processor_id(),
			   i, (long)iter_usecs, iter_KBs);
	}

	/* Compute average usecs and average KBs */
	if (i) {
		avg_usecs = div_s64(avg_usecs, i);
		avg_KBs = div_u64(avg_KBs, i);
	}

	/* Save stats */
	test->min_KBs = min_KBs;
	test->max_KBs = max_KBs;
	test->avg_KBs = avg_KBs;
	test->min_IOPs = div_u64(min_KBs * 1024, io_size);
	test->max_IOPs = div_u64(max_KBs * 1024, io_size);
	test->avg_IOPs = div_u64(avg_KBs * 1024, io_size);

cleanup_reqs:
	/* Cleanup all requests */
	for (r = 0; r < test->request_count; r++)
		test->ops->cleanup(&test->reqs[r]);

	/* Free all requests disk data  */
free_reqs_data:
	for (r = 0; r < test->request_count; r++) {
		req = &test->reqs[r];

		for (i = 0; i < (test->disk_count + 2); i++) {
			if (req->disk[i]) {
				__free_page(req->disk[i]);
				req->disk[i] = NULL;
			}
		}
	}

	/* Free all requests */
	kfree(test->reqs);

fail:
	/* Signal completion to parent thread */
	if (!atomic_dec_return(test->done_count))
		complete(test->done);

	return ret;
}

static int async_tx_test_run(void)
{
	int ret = 0;
	unsigned int i, j, cpu;
	atomic_t done_count;
	struct completion done;
	struct async_tx_test_ops *ops;
	struct async_tx_test *thread;
	unsigned long long min_KBs, max_KBs, avg_KBs;
	unsigned long long min_IOPs, max_IOPs, avg_IOPs;

	ops = async_tx_test_find(strim(test_type));
	if (!ops) {
		pr_err("invalid type %s\n", test_type);
		return -EINVAL;
	}

	if (!thread_count) {
		pr_err("invalid thread_count %u\n", thread_count);
		return -EINVAL;
	}

	if ((disk_block_size < MIN_BLOCK_SIZE) ||
	    (disk_block_size > MAX_BLOCK_SIZE)) {
		pr_err("invalid disk_block_size %u\n", disk_block_size);
		return -EINVAL;
	}
	if (disk_block_size & (MIN_BLOCK_SIZE - 1)) {
		pr_err("disk_block_size %u not multiple of %u\n",
			disk_block_size, MIN_BLOCK_SIZE);
		return -EINVAL;
	}

	if ((disk_count < ops->min_disk_count) ||
	    (disk_count > ops->max_disk_count)) {
		pr_err("invalid disk_count %u\n", disk_count);
		return -EINVAL;
	}

	if (!request_count) {
		pr_err("invalid request_count %u\n", request_count);
		return -EINVAL;
	}

	if (!iteration_count) {
		pr_err("invalid iteration_count %u\n", iteration_count);
		return -EINVAL;
	}

	thread = kcalloc(thread_count, sizeof(*thread), GFP_KERNEL);
	if (!thread)
		return -ENOMEM;

	/* Print test configuration */
	pr("type=%s block_size=%u disk_count=%u\n",
	   ops->name, disk_block_size, disk_count);
	pr("request_count=%u iteration_count=%u\n",
	   request_count, iteration_count);
	pr("threads=%u\n", thread_count);

	atomic_set(&done_count, thread_count);
	init_completion(&done);

	for (i = 0; i < thread_count; i++) {
		thread[i].ops = ops;
		thread[i].verbose = verbose;
		thread[i].thread_count = thread_count;
		thread[i].block_size = disk_block_size;
		thread[i].disk_count = disk_count;
		thread[i].request_count = request_count;
		thread[i].iteration_count = iteration_count;
		thread[i].num = i;
		thread[i].task = NULL;
		thread[i].min_KBs = 0;
		thread[i].max_KBs = 0;
		thread[i].avg_KBs = 0;
		thread[i].min_IOPs = 0;
		thread[i].max_IOPs = 0;
		thread[i].avg_IOPs = 0;
		thread[i].done_count = &done_count;
		thread[i].done = &done;
		thread[i].task = kthread_create(async_tx_test_thread_main,
						&thread[i],
						"async-tx-test%u", i);
		if (IS_ERR(thread[i].task)) {
			pr_err("failed to create thread=%u\n", i);
			if (!atomic_dec_return(&done_count))
				complete(&done);
			continue;
		}

		get_task_struct(thread[i].task);

		j = i;
		do {
			for_each_online_cpu(cpu) {
				if (!j) {
					kthread_bind(thread[i].task, cpu);
					break;
				}
				j--;
			}
		} while (j);

		wake_up_process(thread[i].task);
	}

	/* Wait for threads to finish */
	wait_for_completion(&done);

	/* Print stats of each thread */
	min_KBs = max_KBs = avg_KBs = 0;
	min_IOPs = max_IOPs = avg_IOPs = 0;
	for (i = 0; i < thread_count; i++) {
		if (IS_ERR(thread[i].task))
			continue;

		ret = kthread_stop(thread[i].task);
		if (ret)
			pr_err("thread=%u failed with error=%d\n", i, ret);
		put_task_struct(thread[i].task);
		thread[i].task = NULL;

		pr("thread=%u min_KBs=%llu max_KBs=%llu avg_KBs=%llu\n",
		   i, thread[i].min_KBs, thread[i].max_KBs, thread[i].avg_KBs);
		pr("thread=%u min_IOPS=%llu max_IOPS=%llu avg_IOPS=%llu\n",
		   i, thread[i].min_IOPs, thread[i].max_IOPs,
		   thread[i].avg_IOPs);

		min_KBs += thread[i].min_KBs;
		max_KBs += thread[i].max_KBs;
		avg_KBs += thread[i].avg_KBs;
		min_IOPs += thread[i].min_IOPs;
		max_IOPs += thread[i].max_IOPs;
		avg_IOPs += thread[i].avg_IOPs;
	}

	/* Print overall stats */
	pr("overall min_KBs=%llu max_KBs=%llu avg_KBs=%llu\n",
	   min_KBs, max_KBs, avg_KBs);
	pr("overall min_IOPS=%llu max_IOPS=%llu avg_IOPS=%llu\n",
	   min_IOPs, max_IOPs, avg_IOPs);

	return ret;
}

static int async_tx_test_type_set(const char *val,
				  const struct kernel_param *kp)
{
	char str[20];
	struct async_tx_test_ops *ops;

	mutex_lock(&test_lock);

	strncpy(str, val, sizeof(str));
	str[sizeof(str) - 1] = '\0';
	strim(str);

	ops = async_tx_test_find(str);
	if (!ops) {
		pr("invalid test type %s\n", str);
		mutex_unlock(&test_lock);
		return -EINVAL;
	}

	strncpy(test_type, ops->name, sizeof(ops->name));

	mutex_unlock(&test_lock);

	return 0;
}

static int async_tx_test_type_get(char *val,
				  const struct kernel_param *kp)
{
	int i, r = 0;
	struct async_tx_test_ops *ops;

	mutex_lock(&test_lock);

	for (i = 0; i < ARRAY_SIZE(ops_table); i++) {
		ops = &ops_table[i];
		if (!strncmp(ops->name, test_type, sizeof(ops->name))) {
			if (!i)
				r += sprintf(val + r, "[%s]", ops->name);
			else
				r += sprintf(val + r, " [%s]", ops->name);
		} else {
			if (!i)
				r += sprintf(val + r, "%s", ops->name);
			else
				r += sprintf(val + r, " %s", ops->name);
		}
	}

	mutex_unlock(&test_lock);

	return r;
}

static int async_tx_test_run_set(const char *val,
				 const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&test_lock);

	test_run = true;

	ret = param_set_bool(val, kp);
	if (ret) {
		test_run = false;
		mutex_unlock(&test_lock);
		return ret;
	}

	ret = async_tx_test_run();
	if (ret)
		pr("failed (error %d)\n", ret);
	else
		pr("passed\n");

	test_run = false;

	mutex_unlock(&test_lock);

	return ret;
}

static int async_tx_test_run_get(char *val,
				 const struct kernel_param *kp)
{
	return param_get_bool(val, kp);
}

MODULE_AUTHOR("Anup Patel <anup.patel@broadcom.com>");
MODULE_DESCRIPTION("Async Tx Test Module");
MODULE_LICENSE("GPL");
