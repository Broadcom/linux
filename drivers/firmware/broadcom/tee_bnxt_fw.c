// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 Broadcom.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/tee_drv.h>
#include <linux/uuid.h>

#include <linux/firmware/broadcom/tee_bnxt_fw.h>

#define DRIVER_NAME	"tee-bnxt-fw"

#define MAX_SHM_MEM_SZ	SZ_4M

enum ta_cmd {
/*
 * TA_CMD_BNXT_FASTBOOT - boot bnxt device by copying f/w into sram
 *
 * param[0] unused
 * param[1] unused
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_ITEM_NOT_FOUND - Corrupt f/w image found on memory
 */
	TA_CMD_BNXT_FASTBOOT = 0,

/*
 * TA_CMD_BNXT_HEALTH_STATUS - to check health of bnxt device
 *
 * param[0] (out value) - value.a: health status
 * param[1] unused
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_BAD_PARAMETERS - Incorrect input param
 */
	TA_CMD_BNXT_HEALTH_STATUS,

/*
 * TA_CMD_BNXT_HANDSHAKE - to check bnxt device is booted
 *
 * param[0] (in value)  - value.a: max timeout value
 * param[0] (out value) - value.a: boot status
 * param[1] unused
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_BAD_PARAMETERS - Incorrect input param
 */
	TA_CMD_BNXT_HANDSHAKE,

/*
 * TA_CMD_BNXT_COPY_COREDUMP - copy the core dump into shm
 *
 * param[0] (in value) - value.a: offset at which data to be copied from
 *			 value.b: size of the data
 * param[1] unused
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_BAD_PARAMETERS - Incorrect input param
 * TEE_ERROR_ITEM_NOT_FOUND - Corrupt core dump
 */
	TA_CMD_BNXT_COPY_COREDUMP,

/*
 * TA_CMD_BNXT_FW_UPGRADE - upgrade the bnxt firmware
 *
 * param[0] (in value) - value.a: size of the f/w image
 * param[1] unused
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_BAD_PARAMETERS - Incorrect input param
 */
	TA_CMD_BNXT_FW_UPGRADE,
};

/**
 * struct tee_bnxt_fw_private - OP-TEE bnxt private data
 * @dev:		OP-TEE based bnxt device.
 * @ctx:		OP-TEE context handler.
 * @session_id:		TA session identifier.
 */
struct tee_bnxt_fw_private {
	struct device *dev;
	struct tee_context *ctx;
	u32 session_id;
	struct tee_shm *fw_shm_pool;
};

static struct tee_bnxt_fw_private pvt_data;

/**
 * tee_bnxt_fw_load() - Load the bnxt firmware
 *		    Uses an OP-TEE call to start a secure
 *		    boot process.
 * Returns 0 on success, negative errno otherwise.
 */
int tee_bnxt_fw_load(void)
{
	int ret = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	if (!pvt_data.ctx)
		return -ENODEV;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke TA_CMD_BNXT_FASTBOOT function of Trusted App */
	inv_arg.func = TA_CMD_BNXT_FASTBOOT;
	inv_arg.session = pvt_data.session_id;
	inv_arg.num_params = 4;

	ret = tee_client_invoke_func(pvt_data.ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(pvt_data.dev, "TA_CMD_BNXT_LOAD invoke err: %x\n",
			inv_arg.ret);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(tee_bnxt_fw_load);

/**
 * tee_bnxt_health_status() - Get the health status
 *		    Uses an OP-TEE call to get health
 *		    status of bnxt device.
 * @status:	    status is returned on this pointer
 * Returns 0 on success, negative errno otherwise.
 */
int tee_bnxt_health_status(u32 *status)
{
	int ret = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	if (!pvt_data.ctx)
		return -ENODEV;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke TA_CMD_BNXT_HEALTH_STATUS function of Trusted App */
	inv_arg.func = TA_CMD_BNXT_HEALTH_STATUS;
	inv_arg.session = pvt_data.session_id;
	inv_arg.num_params = 4;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;

	ret = tee_client_invoke_func(pvt_data.ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(pvt_data.dev, "TA_CMD_BNXT_HEALTH_STATUS invoke err: %x\n",
			inv_arg.ret);
		return -EINVAL;
	}

	*status = param[0].u.value.a;

	return 0;
}
EXPORT_SYMBOL(tee_bnxt_health_status);

/**
 * tee_bnxt_handshake() - Get the handshake status
 *		    Uses an OP-TEE call to get handshake
 *		    status after bnxt device`s boot process.
 * @timeout:	    max timeout to wait for handshake
 * @status:	    status is populated
 * Returns 0 on success, negative errno otherwise.
 */
int tee_bnxt_handshake(u32 timeout, u32 *status)
{
	int ret = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	if (!pvt_data.ctx)
		return -ENODEV;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke TA_CMD_BNXT_HANDSHAKE function of Trusted App */
	inv_arg.func = TA_CMD_BNXT_HANDSHAKE;
	inv_arg.session = pvt_data.session_id;
	inv_arg.num_params = 4;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	param[0].u.value.a = timeout;

	ret = tee_client_invoke_func(pvt_data.ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(pvt_data.dev, "TA_CMD_BNXT_HANDSHAKE invoke err: %x\n",
			inv_arg.ret);
		return -EINVAL;
	}

	*status = param[0].u.value.a;

	return 0;
}
EXPORT_SYMBOL(tee_bnxt_handshake);

/**
 * tee_bnxt_copy_coredump() - Copy coredump from the allocated memory
 *			    Uses an OP-TEE call to copy coredump
 * @buf:	desintation buffer where core dump is copied into
 * @offset:	offset from the base address of core dump area
 * @size:	size of the dump
 *
 * Returns 0 on success, negative errno otherwise.
 */
int tee_bnxt_copy_coredump(void *buf, u32 offset, u32 size)
{
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	void *core_data;
	u32 rbytes = size;
	u32 nbytes = 0;
	int ret = 0;

	if (!pvt_data.ctx)
		return -ENODEV;

	if (!buf)
		return -EINVAL;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke TA_CMD_BNXT_COPY_COREDUMP function of Trusted App */
	inv_arg.func = TA_CMD_BNXT_COPY_COREDUMP;
	inv_arg.session = pvt_data.session_id;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	param[0].u.memref.shm = pvt_data.fw_shm_pool;
	param[0].u.memref.size = MAX_SHM_MEM_SZ;
	param[0].u.memref.shm_offs = 0;

	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;

	while (rbytes)  {
		nbytes = rbytes;

		nbytes = min_t(u32, rbytes, param[0].u.memref.size);

		param[1].u.value.a = offset;
		param[1].u.value.b = nbytes;

		ret = tee_client_invoke_func(pvt_data.ctx, &inv_arg, param);
		if ((ret < 0) || (inv_arg.ret != 0)) {
			dev_err(pvt_data.dev,
				"TA_CMD_BNXT_COPY_COREDUMP invoke err: %x\n",
				inv_arg.ret);
			return -EINVAL;
		}

		core_data = tee_shm_get_va(pvt_data.fw_shm_pool, 0);
		if (IS_ERR(core_data)) {
			dev_err(pvt_data.dev, "tee_shm_get_va failed\n");
			return PTR_ERR(core_data);
		}

		memcpy(buf, core_data, nbytes);

		rbytes -= nbytes;
		buf += nbytes;
		offset += nbytes;
	}

	return 0;
}
EXPORT_SYMBOL(tee_bnxt_copy_coredump);

/**
 * bnxt_fw_upgrade() - Upgrade the bnxt firmware and configuration of
 *		       bnxt device into the flash device.
 *		       Uses an OP-TEE call to upgrade firmware.
 * @buf:	source buffer of firmware image
 * @size:	size of the image
 *
 * Returns 0 on success, negative errno otherwise.
 */
int bnxt_fw_upgrade(void *buf, u32 size)
{
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	void *fw_image_dst;
	int ret = 0;

	if (!pvt_data.ctx)
		return -ENODEV;

	/* we do not expect firmware size more than allocated tee shm size */
	if (!buf || size > MAX_SHM_MEM_SZ)
		return -EINVAL;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke TA_CMD_BNXT_FW_UPGRADE function of Trusted App */
	inv_arg.func = TA_CMD_BNXT_FW_UPGRADE;
	inv_arg.session = pvt_data.session_id;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	param[0].u.memref.shm = pvt_data.fw_shm_pool;
	param[0].u.memref.size = MAX_SHM_MEM_SZ;
	param[0].u.memref.shm_offs = 0;

	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[1].u.value.a = size;

	fw_image_dst = tee_shm_get_va(pvt_data.fw_shm_pool, 0);
	if (IS_ERR(fw_image_dst)) {
		dev_err(pvt_data.dev, "tee_shm_get_va failed\n");
		return PTR_ERR(fw_image_dst);
	}

	memcpy(fw_image_dst, buf, size);

	ret = tee_client_invoke_func(pvt_data.ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(pvt_data.dev, "TA_CMD_BNXT_FW_UPGRADE invoke err: %x\n",
			inv_arg.ret);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(bnxt_fw_upgrade);

static int optee_ctx_match(struct tee_ioctl_version_data *ver, const void *data)
{
	if (ver->impl_id == TEE_IMPL_ID_OPTEE)
		return 1;
	else
		return 0;
}

static int tee_bnxt_fw_probe(struct device *dev)
{
	struct tee_client_device *bnxt_device = to_tee_client_device(dev);
	int ret = 0, err = -ENODEV;
	struct tee_ioctl_open_session_arg sess_arg;
	struct tee_shm *fw_shm_pool;

	memset(&sess_arg, 0, sizeof(sess_arg));

	/* Open context with TEE driver */
	pvt_data.ctx = tee_client_open_context(NULL, optee_ctx_match, NULL,
					       NULL);
	if (IS_ERR(pvt_data.ctx))
		return -ENODEV;

	/* Open session with Bnxt load Trusted App */
	memcpy(sess_arg.uuid, bnxt_device->id.uuid.b, TEE_IOCTL_UUID_LEN);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	sess_arg.num_params = 0;

	ret = tee_client_open_session(pvt_data.ctx, &sess_arg, NULL);
	if ((ret < 0) || (sess_arg.ret != 0)) {
		dev_err(dev, "tee_client_open_session failed, err: %x\n",
			sess_arg.ret);
		err = -EINVAL;
		goto out_ctx;
	}
	pvt_data.session_id = sess_arg.session;

	pvt_data.dev = dev;

	fw_shm_pool = tee_shm_alloc(pvt_data.ctx, MAX_SHM_MEM_SZ,
				    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(fw_shm_pool)) {
		dev_err(pvt_data.dev, "tee_shm_alloc failed\n");
		err = PTR_ERR(fw_shm_pool);
		goto out_sess;
	}

	pvt_data.fw_shm_pool = fw_shm_pool;

	return 0;

out_sess:
	tee_client_close_session(pvt_data.ctx, pvt_data.session_id);
out_ctx:
	tee_client_close_context(pvt_data.ctx);

	return err;
}

static int tee_bnxt_fw_remove(struct device *dev)
{
	tee_client_close_session(pvt_data.ctx, pvt_data.session_id);
	tee_client_close_context(pvt_data.ctx);
	pvt_data.ctx = NULL;

	return 0;
}

static const struct tee_client_device_id tee_bnxt_fw_id_table[] = {
	{UUID_INIT(0x6272636D, 0x2019, 0x0716,
		    0x42, 0x43, 0x4D, 0x5F, 0x53, 0x43, 0x48, 0x49)},
	{}
};

MODULE_DEVICE_TABLE(tee, tee_bnxt_fw_id_table);

static struct tee_client_driver tee_bnxt_fw_driver = {
	.id_table	= tee_bnxt_fw_id_table,
	.driver		= {
		.name		= DRIVER_NAME,
		.bus		= &tee_bus_type,
		.probe		= tee_bnxt_fw_probe,
		.remove		= tee_bnxt_fw_remove,
	},
};

static int __init tee_bnxt_fw_mod_init(void)
{
	return driver_register(&tee_bnxt_fw_driver.driver);
}

static void __exit tee_bnxt_fw_mod_exit(void)
{
	driver_unregister(&tee_bnxt_fw_driver.driver);
}

module_init(tee_bnxt_fw_mod_init);
module_exit(tee_bnxt_fw_mod_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom bnxt firmware manager");
MODULE_LICENSE("GPL v2");
