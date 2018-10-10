// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018 Broadcom.
 * The term "Broadcom" refers to Broadcom Inc. and/or its subsidiaries.
 */

#include <linux/bcm_iproc_mailbox.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

enum pm_state {
	IPC_PM_STATE_ACTIVE = 0x0,
	IPC_PM_STATE_DEEPSLEEP = 0x2,
	IPC_PM_STATE_OFF = 0x3,
};

/* Regulator type */
enum IPC_REGULATOR_TYPE {
	IPC_REGULATOR_TYPE_GPIO = 1,
};

/* Regulator type field within 32-bit regulator id */
#define IPC_REGULATOR_TYPE_SHIFT 24
#define IPC_REGULATOR_TYPE_WIDTH 8
#define IPC_REGULATOR_TYPE_MASK (BIT(IPC_REGULATOR_TYPE_WIDTH) - 1)

/* AON GPIO id field within 32-bit regulator id */
#define IPC_REGULATOR_GPIO_ID_SHIFT 0
#define IPC_REGULATOR_GPIO_ID_WIDTH 8
#define IPC_REGULATOR_GPIO_ID_MASK (BIT(IPC_REGULATOR_GPIO_ID_WIDTH) - 1)

/*
 * Get SCP version
 *
 * Param - physical address of ipc_version_string_get struct,
 *         where scp will store version string
 */
#define IPC_VERSION_STRING_GET 0x11

/*
 * Configuration of AON GPIO that controls power rails voltage regulator
 * Param - ipc_regulator_cfg
 * Response - return code
 */
#define IPC_REGULATOR_CFG  0x13

struct ipc_regulator_cfg {
	/*
	 * regulator id:
	 * bit0-7 is used for GPIO; bit 24-31 is used for regulator type
	 */
	uint32_t reg_id;
	/*
	 * 1 if voltage regulator controlled by this GPIO is active high; 0 if
	 *  it is active low.
	 */
	uint32_t enable_active_high;
	/* Enable ramp up time in usec .*/
	uint32_t ramp_delay_usec;
	/*
	 * Mask of PM states in which this regulator should be disabled.
	 * Bit IPC_PM_STATE_xyz is 1 if this regulator should be in disabled
	 *  state in PM state xyz
	 */
	uint32_t disabled_in_pm_states_mask;
};

#define IPC_VERSION_STRING_LEN 256
struct ipc_version_string_get {
	char version_string[IPC_VERSION_STRING_LEN];
};

struct omega_scp_data {
	struct device      *dev;
	struct mbox_client mbox_client;
	struct mbox_chan   *mbox_chan;
};

static inline int omega_mbox_send_msg(struct mbox_chan *chan, u32 cmd,
				      u32 param, bool wait_ack)
{
	int ret;
	struct iproc_mbox_msg msg;

	if (!chan)
		return -ENODEV;

	msg.cmd = cmd;
	msg.param = param;
	msg.wait_ack = wait_ack;
	ret = mbox_send_message(chan, &msg);
	mbox_client_txdone(chan, 0);

	return ret < 0 ? ret : msg.reply_code;
}

/*
 * For IPC commands with struct as a parameter, sends message to SCP using
 *  mailbox framework, while taking care of cache-coherency of the struct
TODO pdev
 * @cmd The command to send.
 * @param Virtual pointer to struct to be sent as command parameter
 * @size Size of the struct to be sent
 * @wait_ack true to wait for SCP to send a reply to this command, false
 *   to return immediately and not wait for a reply.
 *
 * @return A negative value if sending the message failed, otherwise the reply
 *   code from the SCP indicating success or failure of executing the command: 0
 *   indicates success.
 */
static int omega_mbox_send_msg_with_struct(struct platform_device *pdev,
					   u32 cmd, void *param, size_t size,
					   bool wait_ack)
{
	int err;
	dma_addr_t dma_handle;
	struct omega_scp_data *omega_scp = platform_get_drvdata(pdev);

	dma_handle = dma_map_single(omega_scp->dev, param, size,
				    DMA_BIDIRECTIONAL);
	if (dma_mapping_error(omega_scp->dev, dma_handle)) {
		dev_err(omega_scp->dev,
			"Failed to dma map param of command 0x%x\n", cmd);
		return -ENOMEM;
	}

	err = omega_mbox_send_msg(omega_scp->mbox_chan, cmd, dma_handle,
				  wait_ack);

	dma_unmap_single(omega_scp->dev, dma_handle, size, DMA_BIDIRECTIONAL);

	return err;
}

static int omega_scp_gpio_config(struct platform_device *pdev,
				 struct device_node *regulator_node,
				 u32 disabled_in_pm_states_mask)
{
	struct ipc_regulator_cfg *cmd;
	struct of_phandle_args args;
	int err = 0;
	int gpio;
	bool enable_active_high = false;
	u32 startup_delay = 0;

	cmd = kmalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	err = of_parse_phandle_with_args(regulator_node, "gpio", "#gpio-cells",
					 0, &args);
	if (err) {
		dev_err(&pdev->dev, "Failed to parse AON gpio node");
		goto err_gpio;
	}
	gpio = args.args[0];

	/* Get is-active-high property */
	if (of_property_read_bool(regulator_node, "enable-active-high"))
		enable_active_high = true;

	/* Get ramp delay property */
	err = of_property_read_u32(regulator_node, "startup-delay-us",
				   &startup_delay);
	if (err) {
		dev_err(&pdev->dev, "startup delay is not defined");
		goto err_gpio;
	}

	/* Send the configuration IPC message to scp */
	cmd->reg_id = ((gpio &  IPC_REGULATOR_GPIO_ID_MASK) <<
			IPC_REGULATOR_GPIO_ID_SHIFT) |
		      ((IPC_REGULATOR_TYPE_GPIO & IPC_REGULATOR_TYPE_MASK) <<
			IPC_REGULATOR_TYPE_SHIFT);
	cmd->enable_active_high = enable_active_high;
	cmd->ramp_delay_usec = startup_delay;
	cmd->disabled_in_pm_states_mask = disabled_in_pm_states_mask;

	dev_info(&pdev->dev,
		"Configuring AON regulator: aon gpio %u, active %s, startup delay usec %u, disabled PM states mask 0x%x\n",
		 gpio, (enable_active_high ? "high" : " low"), startup_delay,
		 disabled_in_pm_states_mask);

	err = omega_mbox_send_msg_with_struct(pdev, IPC_REGULATOR_CFG, cmd,
					      sizeof(cmd), true);

err_gpio:
	kfree(cmd);

	return err;
}

/*
 * Parse aon regulator device tree node.
 */
static int omega_scp_regulator_config(struct platform_device *pdev,
				      struct device_node *np)
{
	int err = 0;
	u32 disabled_in_pm_states_mask = 0;
	struct device_node *regulator_node;

	/* Get regulator - mandatory property */
	regulator_node = of_parse_phandle(np, "brcm,pm-supply", 0);
	if (!regulator_node) {
		err = -EINVAL;
		goto out_scp;
	}

	/*
	 * Find out if regulator should be enabled/disabled in different
	 * PM states.
	 */
	if (of_get_property(np, "brcm,pm-regulator-off-in-off-state", NULL))
		disabled_in_pm_states_mask |= (BIT(IPC_PM_STATE_OFF));

	if (of_get_property(np, "brcm,pm-regulator-off-in-mem-state", NULL))
		disabled_in_pm_states_mask |= (BIT(IPC_PM_STATE_DEEPSLEEP));

	/* Get GPIO - mandatory property */
	if (of_find_property(regulator_node, "gpio", NULL)) {
		err = omega_scp_gpio_config(pdev, regulator_node,
					    disabled_in_pm_states_mask);
		if (err) {
			dev_err(&pdev->dev,
				"Error configuring regulator (node %s): %d\n",
				of_node_full_name(np), err);
			err = err < 0 ? err : -EINVAL;
		}
	} else {
		dev_err(&pdev->dev, "AON gpio undefined");
		err = -EINVAL;
	}

out_scp:
	return err;
}

static int omega_scp_config(struct platform_device *pdev)
{
	int err;
	const char *pm_function;
	struct device_node *child;
	struct device_node *np = pdev->dev.of_node;

	for_each_available_child_of_node(np, child) {
		err = of_property_read_string(child, "brcm,pm-function",
					      &pm_function);
		if (err < 0) {
			dev_err(&pdev->dev, "brcm,pm-function undefined");
			return err;
		}

		if (!strcasecmp(pm_function, "regulator")) {
			err = omega_scp_regulator_config(pdev, child);
			if (err)
				return err;
		} else {
			dev_err(&pdev->dev, "unsupported function %s",
				pm_function);
			return -ENODEV;
		}
	}

	return 0;
}

static int omega_scp_get_version(struct platform_device *pdev)
{
	int ret;
	struct ipc_version_string_get *get_version;

	get_version = kmalloc(sizeof(*get_version), GFP_KERNEL);
	if (!get_version)
		return -ENOMEM;

	ret = omega_mbox_send_msg_with_struct(pdev, IPC_VERSION_STRING_GET,
					      get_version, sizeof(get_version),
					      true);
	if (ret) {
		dev_err(&pdev->dev, "Failed to communicate with SCP\n");
		kfree(get_version);
		return ret < 0 ? ret : -EINVAL;
	}

	dev_info(&pdev->dev, "SCP detected: %s\n", get_version->version_string);

	kfree(get_version);

	return 0;
}

static int omega_scp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct omega_scp_data *omega_scp;
	int err = 0;

	omega_scp = devm_kzalloc(dev, sizeof(*omega_scp), GFP_KERNEL);
	if (!omega_scp)
		return -ENOMEM;

	omega_scp->dev = &pdev->dev;
	platform_set_drvdata(pdev, omega_scp);

	/* Get mailbox channel. */
	omega_scp->mbox_client.dev          = &pdev->dev;
	omega_scp->mbox_client.tx_block     = false;
	omega_scp->mbox_client.tx_tout      = 2;
	omega_scp->mbox_client.knows_txdone = true;
	omega_scp->mbox_chan = mbox_request_channel(&omega_scp->mbox_client, 0);
	if (IS_ERR(omega_scp->mbox_chan)) {
		dev_err(&pdev->dev, "unable to get mbox channel\n");
		err = PTR_ERR(omega_scp->mbox_chan);
		return err;
	}

	err = omega_scp_get_version(pdev);
	if (err) {
		dev_err(&pdev->dev, "Fail to get SCP version!");
		goto err_free_mbox;
	}

	err = omega_scp_config(pdev);
	if (err) {
		dev_err(&pdev->dev,
			"Failed to configure aon regulators in device tree\n");
		goto err_free_mbox;
	}

	dev_info(&pdev->dev, "Omega SCP is supported\n");

err_free_mbox:
	mbox_free_channel(omega_scp->mbox_chan);

	return err;
}

static const struct of_device_id omega_scp_of_match[] = {
	{ .compatible = "brcm,omega-scp", },
	{},
};
MODULE_DEVICE_TABLE(of, omega_scp_of_match);

static struct platform_driver omega_scp_driver = {
	.driver = {
		.name = "omega-scp",
		.of_match_table = omega_scp_of_match,
	},
	.probe = omega_scp_probe,
};
builtin_platform_driver(omega_scp_driver);

MODULE_DESCRIPTION("Omega power domain driver");
MODULE_LICENSE("GPL v2");
