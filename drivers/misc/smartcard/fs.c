/*
 * Copyright (C) 2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "bscd_datatypes.h"
#include "bscd_priv.h"
#include "ioctl.h"
#include "sci.h"

#define  MAX_SCI_DEVS 1

static int sci_open(struct inode *inode, struct file *filp)
{
	int ret;
	struct p_chnl_hdl     *handle;
	struct sci_ctx *pctx = container_of(inode->i_cdev,
					struct sci_ctx, sci_cdev);
	if (pctx == NULL) {
		pr_err("%s:failed to get sci ctxt\n", __func__);
		return -ENODEV;
	}
	mutex_lock(&pctx->sc_mutex);
	ret = chnl_open(pctx->mod_hdl, &handle,
			pctx->sci_id, NULL, pctx->sci_regs);
	if (ret != BERR_SUCCESS) {
		dev_err(pctx->dev, "OPEN channel failed\n");
		ret = -EPERM;
		goto out;
	}

	ret = chnl_reset_ifd(handle, rst_cold);
	if (ret != BERR_SUCCESS) {
		dev_err(pctx->dev, "RESET IFD failed\n");
		ret = -EPERM;
		goto chnl_close;
	}

	handle->priv = pctx;
	filp->private_data = handle;
	dev_dbg(pctx->dev, "Open channel success\n");
	mutex_unlock(&pctx->sc_mutex);
	return ret;

chnl_close:
	chnl_close(pctx->mod_hdl, 0);
out:
	mutex_unlock(&pctx->sc_mutex);
	return ret;
}

static int sci_release(struct inode *inode, struct file *filp)
{
	int ret;
	struct sci_ctx *pctx = container_of(inode->i_cdev,
					    struct sci_ctx, sci_cdev);
	if (pctx == NULL) {
		pr_err("%s: failed to get iproc sci ctxt\n", __func__);
		return -ENODEV;
	}
	mutex_lock(&pctx->sc_mutex);
	ret = chnl_close(pctx->mod_hdl, pctx->sci_id);
	if (ret != BERR_SUCCESS) {
		dev_err(pctx->dev, "CLOSE failed\n");
		ret = -1;
		goto out;
	}
	dev_dbg(pctx->dev, "Close channel success\n");
	ret = 0;

out:
	mutex_unlock(&pctx->sc_mutex);
	return ret;
}

int sc_reset_get_atr(struct p_chnl_hdl     *handle, struct atr_parm __user *up)
{
	int ret;
	unsigned char *rbuf;
	unsigned long rsize;
	struct atr_parm atr_info;
	struct sci_ctx *pctx;

	pctx = handle->priv;

	ret = chnl_reset_ifd(handle, rst_cold);
	if (ret != BERR_SUCCESS) {
		dev_err(pctx->dev, "RESET IFD failed\n");
		return -EFAULT;
	}
	dev_dbg(pctx->dev, "RESET IFD OK\n");

	ret = chnl_detect_card_non_blk(handle,
				       card_inserted);
	if (ret != BERR_SUCCESS) {
		dev_err(pctx->dev, "DETECT CARD failed\n");
		return -EFAULT;
	}
	dev_dbg(pctx->dev, "DETECT CARD OK\n");

	ret = chnl_rst_card(handle,
				rst_card_act_rcv_decode);
	if (ret != BERR_SUCCESS) {
		dev_err(pctx->dev, "RESET CARD failed\n");
		return -EFAULT;
	}
	dev_dbg(pctx->dev, "RESET CARD OK\n");

	rbuf = kzalloc(BSCD_MAX_ATR_SIZE, GFP_KERNEL);
	if (!rbuf)
		return -ENOMEM;

	ret = bscd_chnl_get_atr(handle, rbuf, &rsize);
	if (ret != BERR_SUCCESS) {
		dev_err(pctx->dev, "RECEIVE failed, err:%d\n", ret);
		kfree(rbuf);
		return -EFAULT;
	}

	memcpy(atr_info.atr, rbuf, rsize);
	atr_info.atr_len = rsize;
	kfree(rbuf);

	if (copy_to_user(up, &atr_info, sizeof(struct atr_parm))) {
		dev_err(pctx->dev, "copy_to_user failed for GET_ATR\n");
		return -EFAULT;
	}

	return ret;
}
int iproc_sc_transmit(struct p_chnl_hdl *handle, struct apdu_args __user *up)
{
	struct apdu_args ap;
	int ret;
	struct sci_ctx *pctx;

	pctx = handle->priv;

	if (copy_from_user(&ap, up, sizeof(*up))) {
		dev_err(pctx->dev, "copy_from_user failed for TRANSMIT\n");
		return -EFAULT;
	}
	ret = bscd_chnl_apdu_transceive(handle,
					ap.txbuf, ap.txlen,
					ap.rxbuf, (unsigned long *)ap.rxlen,
					BSCD_MAX_APDU_SIZE);
	if (ret != BERR_SUCCESS) {
		dev_err(pctx->dev, "Transceive failed\n");
		return -EFAULT;
	}
	if (copy_to_user(up, &ap, sizeof(ap))) {
		dev_err(pctx->dev, "copy_to_user failed for TRANSMIT\n");
		return -EFAULT;
	}
	return ret;
}

int iproc_sc_do_pps(struct p_chnl_hdl *handle)
{
	int ret;
	struct sci_ctx *pctx;

	pctx = handle->priv;
/* DO PPS */
	if (handle->is_pps_needed == false)
		return 0;
	ret = bscd_chnl_pps(handle);
	if (ret != BERR_SUCCESS) {
		dev_err(pctx->dev, "do pps failed\n");
		return -EFAULT;
	}
/* Set the negotiated channel parameters */
	ret = chnl_set_params(handle, &(handle->negotiated_chs));
	if (ret != BERR_SUCCESS) {
		dev_err(pctx->dev, "set channel parameters\n");
		return -EFAULT;
	}
	return ret;
}

static long sci_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int ret = BERR_SUCCESS;
	struct p_chnl_hdl     *chnl_hdl;
	struct sci_ctx *pctx;
	void __user *p = (void __user *)arg;

	chnl_hdl = (struct p_chnl_hdl     *)filep->private_data;
	pctx = chnl_hdl->priv;

	mutex_lock(&pctx->sc_mutex);
	pctx->busy = 1;

	/* Before return from switch statement clear pctx->busy */
	switch (cmd) {
	case SC_IOCTL_INIT:
		dev_dbg(pctx->dev, "Get IOCTL INIT\n");
		break;
	case SC_IOCTL_POWERUP:
		dev_dbg(pctx->dev, "Get IOCTL POWERUP\n");
		ret = sc_reset_get_atr(chnl_hdl, p);
		if (ret != BERR_SUCCESS) {
			dev_err(pctx->dev, "POWERUP failed\n");
			ret = -EFAULT;
			goto out;
		}
		break;
	case SC_IOCTL_POWERDOWN:
		dev_dbg(pctx->dev, "Get IOCTL POWERDOWN\n");
		ret = chnl_reset_power_icc(chnl_hdl,
					pwricc_pwrdwn);
		if (ret != BERR_SUCCESS) {
			dev_err(pctx->dev, "POWERICC DOWN failed\n");
			ret = -EFAULT;
			goto out;
		}
		break;
	case SC_IOCTL_TRANSMIT:
		dev_dbg(pctx->dev, "Get IOCTL TRANSMIT\n");
		ret = iproc_sc_transmit(chnl_hdl, p);
		break;
	case SC_IOCTL_RESET:
		dev_dbg(pctx->dev, "Get IOCTL RESET\n");
		ret = sc_reset_get_atr(chnl_hdl, p);
		if (ret != BERR_SUCCESS) {
			dev_err(pctx->dev, "RESET failed\n");
			ret = -EFAULT;
			goto out;
		}
		break;
	case SC_IOCTL_ICCSTATUS:
		dev_dbg(pctx->dev, "Get IOCTL ICCSTATUS\n");
		ret = chnl_detect_card_non_blk(chnl_hdl,
						card_inserted);
		if (ret == BERR_UNKNOWN) {
			/* no card or non responsive card */
			put_user(0, (int __user *)arg);
			chnl_reset_power_icc(chnl_hdl,
						pwricc_pwrdwn);
		} else if (ret == BERR_SUCCESS)
			put_user(1, (int __user *)arg);
		else {
			dev_err(pctx->dev, "GETSTATUS failed\n");
			put_user(-1, (int __user *)arg);
			ret = -EFAULT;
			goto out;
		}
		ret = BERR_SUCCESS;
		break;
	case SC_IOCTL_DOPPS:
		dev_dbg(pctx->dev, "DO PPS IOCTL\n");
		ret = iproc_sc_do_pps(chnl_hdl);
	break;
	default:
		dev_err(pctx->dev, "Unrecognized ioctl:0x%x\n", cmd);
		ret = -ENOTTY;
		break;
	}

out:
	pctx->busy = 0;
	mutex_unlock(&pctx->sc_mutex);

	return ret;
}

static const struct file_operations sci_dev_fops = {
	.open    = sci_open,
	.release = sci_release,
	.unlocked_ioctl = sci_ioctl,
};

int init_fs_intfs(struct sci_ctx *pctx)
{
	int ret;

	ret = alloc_chrdev_region(&pctx->devt, 0, MAX_SCI_DEVS,
				  pctx->sci_id ? "sciB" : "sciA");
	if (ret) {
		dev_err(pctx->dev, "Failed to alloc chrdev region:%d\n", ret);
		return ret;
	}
	cdev_init(&pctx->sci_cdev, &sci_dev_fops);
	ret = cdev_add(&pctx->sci_cdev, pctx->devt, MAX_SCI_DEVS);
	if (ret) {
		dev_err(pctx->dev, "cannot get add chrdev, err:%d\n", ret);
		goto out_region;
	}
	pctx->dev = device_create(pctx->sci_class, pctx->dev,
				  pctx->devt, pctx,
				  pctx->sci_id ? "sciB" : "sciA");
	if (IS_ERR(pctx->dev)) {
		dev_err(pctx->dev, "failed to create device\n");
		ret = PTR_ERR(pctx->dev);
		goto out_cdev;
	}
	dev_info(pctx->dev, "%d devices added.\n", MAX_SCI_DEVS);

	return ret;
out_cdev:
	cdev_del(&pctx->sci_cdev);
out_region:
	unregister_chrdev_region(pctx->devt, MAX_SCI_DEVS);
	return ret;

}
void rm_fs_intfs(struct sci_ctx *pctx)
{
	device_destroy(pctx->sci_class, pctx->devt);
	cdev_del(&pctx->sci_cdev);
	unregister_chrdev_region(pctx->devt, MAX_SCI_DEVS);
}
