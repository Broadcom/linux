/*
 * Copyright (C) 2016 Broadcom.
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
 * This driver initializes power management on the M0 co-processor using the
 * mailbox controller. The driver does nothing after probe other than manage
 * resources (IO and the mailbox channel) used by the platform_suspend_ops
 * functions.
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include "cygnus_pm.h"

enum m0_pm_state {
	M0_IPC_PM_STATE_ACTIVE,
	M0_IPC_PM_STATE_SLEEP,
	M0_IPC_PM_STATE_DEEPSLEEP,
	M0_IPC_PM_STATE_OFF,
};

/*
 * Get M0 version
 *
 * Param - physical address of m0_ipc_m0_cmd_get_version struct,
 *         where m0 will store version string
 */
#define M0_IPC_M0_CMD_GET_VERSION  0x11

/*
 * Configuration of AON GPIO that controls power rails voltage regulator
 * Param - m0_ipc_m0_cmd_power_rail_regulator_cfg
 * Response - return code
 */
#define M0_IPC_M0_CMD_POWER_RAIL_REGULATOR_CFG  0x13

struct m0_ipc_m0_cmd_power_rail_regulator_cfg {
	/* AON GPIO number */
	uint32_t gpio;
	/*
	 * 1 if voltage regulator controlled by this GPIO is active high; 0 if
	 *  it is active low.
	 */
	uint32_t enable_active_high;
	/* Enable ramp up time in usec .*/
	uint32_t startup_delay_usec;
	/*
	 * Mask of PM states in which this regulator should be disabled.
	 * Bit M0_IPC_PM_STATE_xyz is 1 if this regulator should be in disabled
	 *  state in PM state xyz
	 */
	uint32_t disabled_in_pm_states_mask;
};

struct m0_ipc_m0_cmd_get_version {
	char version_string[256];
};

static struct cygnus_pm *cygnus_pm;

/*
 * For M0 IPC commands with struct as a parameter, sends message to M0 using
 *  mailbox framework, while taking care of cache-coherency of the struct
 * @cmd The command to send.
 * @param Virtual pointer to struct to be sent as command parameter
 * @size Size of the struct to be sent
 * @wait_ack true to wait for M0 to send a reply to this command, false
 *   to return immediately and not wait for a reply.
 *
 * @return A negative value if sending the message failed, otherwise the reply
 *   code from the M0 indicating success or failure of executing the command: 0
 *   indicates success.
 */
static int cygnus_mbox_send_msg_with_struct(u32 cmd, void *param, size_t size,
	bool wait_ack)
{
	int err;
	dma_addr_t dma_handle;

	dma_handle = dma_map_single(cygnus_pm->dev, param, size,
		DMA_BIDIRECTIONAL);
	if (dma_mapping_error(cygnus_pm->dev, dma_handle)) {
		dev_err(cygnus_pm->dev,
			"Failed to dma map param of command 0x%x\n", cmd);
		return -ENOMEM;
	}

	err = cygnus_mbox_send_msg(cygnus_pm->mbox_chan, cmd, dma_handle,
		wait_ack);

	dma_unmap_single(cygnus_pm->dev, dma_handle, size, DMA_BIDIRECTIONAL);

	return err;
}

/*
 * Find out if regulator is disabled in the given PM state.
 * AON regulator node in the device tree is matched against state_name parameter
 *
 * @return true if regulator should be externally disabled in the given PM
 *   state, otherwise return false
 */
static bool cygnus_pm_parse_aon_regulator_is_disabled_in_state(
	struct device_node *regulator_node,
	const char *state_name)
{
	struct device_node *state_node = of_find_node_by_name(regulator_node,
		state_name);

	of_node_get(regulator_node);
	of_node_put(state_node);

	if (state_node && of_property_read_bool(state_node,
		"regulator-off-in-suspend"))
		return true;

	return false;
}

/*
 * Parse aon regulator device tree node.
 */
static int cygnus_pm_parse_aon_regulator_cfg(struct platform_device *pdev,
	struct device_node *regulator_node)
{
	int err;
	bool enable_active_high = false;
	u32 startup_delay = 0;
	u32 disabled_in_pm_states_mask = 0;
	int gpio;
	struct m0_ipc_m0_cmd_power_rail_regulator_cfg cmd;
	struct of_phandle_args args;

	/* Get AON GPIO - mandatory property */
	err = of_parse_phandle_with_args(regulator_node, "gpio", "#gpio-cells",
		0, &args);
	if (err) {
		dev_err(&pdev->dev,
			"Error parsing PM aon regulators config: gpio property of PM regulator is required (node %s)\n",
			of_node_full_name(regulator_node));
		return err;
	}
	gpio = args.args[0];
	of_node_put(args.np);

	/* Get is-active-high property */
	if (of_property_read_bool(regulator_node, "enable-active-high"))
		enable_active_high = true;

	/* Get ramp delay property */
	of_property_read_u32(regulator_node, "startup-delay-us",
		&startup_delay);

	/*
	 * Find out if regulator should be enabled/disabled in different
	 * PM states.
	 */
	if (cygnus_pm_parse_aon_regulator_is_disabled_in_state(regulator_node,
		"regulator-state-off"))
		disabled_in_pm_states_mask |= (1 << M0_IPC_PM_STATE_OFF);

	if (cygnus_pm_parse_aon_regulator_is_disabled_in_state(regulator_node,
		"regulator-state-standby"))
		disabled_in_pm_states_mask |= (1 << M0_IPC_PM_STATE_SLEEP);

	if (cygnus_pm_parse_aon_regulator_is_disabled_in_state(regulator_node,
		"regulator-state-mem"))
		disabled_in_pm_states_mask |= (1 << M0_IPC_PM_STATE_DEEPSLEEP);

	/* Send the configuration IPC message to M0 */
	cmd.gpio = gpio;
	cmd.enable_active_high = enable_active_high;
	cmd.startup_delay_usec = startup_delay;
	cmd.disabled_in_pm_states_mask = disabled_in_pm_states_mask;

	dev_info(&pdev->dev,
		"Configuring AON regulator: aon gpio %u, active %s, startup delay usec %u, disabled PM states mask 0x%x\n",
		gpio, (enable_active_high ? "high" : " low"), startup_delay,
		disabled_in_pm_states_mask);

	err = cygnus_mbox_send_msg_with_struct(
		M0_IPC_M0_CMD_POWER_RAIL_REGULATOR_CFG, &cmd, sizeof(cmd),
		true);
	if (err) {
		dev_err(&pdev->dev,
			"Error configuring PM aon regulator (node %s): %d\n",
			of_node_full_name(regulator_node), err);
		err = err < 0 ? err : -EINVAL;
	}

	return err;
}

static int cygnus_pm_parse_aon_regulators_cfg(struct platform_device *pdev)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *regulators_node;
	struct device_node *regulator_node;

	regulators_node = of_find_node_by_name(np, "aon_regulators");
	if (!regulators_node)
		return 0;

	for_each_available_child_of_node(regulators_node, regulator_node) {
		ret = cygnus_pm_parse_aon_regulator_cfg(pdev, regulator_node);
		if (ret)
			return ret;
	}

	of_node_put(regulators_node);

	return 0;
}

static int cygnus_pm_m0_init(void)
{
	int ret;
	struct m0_ipc_m0_cmd_get_version get_version;

	ret = cygnus_mbox_send_msg_with_struct(M0_IPC_M0_CMD_GET_VERSION,
		&get_version, sizeof(get_version), true);
	if (ret) {
		dev_err(cygnus_pm->dev, "Failed to communicate with M0\n");
		return ret < 0 ? ret : -EINVAL;
	}

	dev_info(cygnus_pm->dev, "M0 detected: %s\n",
		get_version.version_string);

	return 0;
}

static int cygnus_pm_probe(struct platform_device *pdev)
{
	int err;
	struct resource *res;

	cygnus_pm = devm_kzalloc(&pdev->dev, sizeof(*cygnus_pm),
		GFP_KERNEL);
	if (!cygnus_pm)
		return -ENOMEM;

	/*
	 * These registers are used only by the platform_suspend_ops when
	 * entering/exiting suspend/resume.
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "scu");
	cygnus_pm->scu = ioremap(res->start, resource_size(res));
	if (!cygnus_pm->scu) {
		dev_err(&pdev->dev, "unable to map I/O space\n");
		err = -ENXIO;
		goto err_iomap;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "crmu");
	cygnus_pm->crmu = ioremap(res->start, resource_size(res));
	if (!cygnus_pm->crmu) {
		dev_err(&pdev->dev, "unable to map I/O space\n");
		err = -ENXIO;
		goto err_iomap;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cru");
	cygnus_pm->cru = ioremap(res->start, resource_size(res));
	if (!cygnus_pm->cru) {
		dev_err(&pdev->dev, "unable to map I/O space\n");
		err = -ENXIO;
		goto err_iomap;
	}

	cygnus_pm->dev = &pdev->dev;
	platform_set_drvdata(pdev, cygnus_pm);

	/* Get mailbox channel. */
	cygnus_pm->mbox_client.dev          = &pdev->dev;
	cygnus_pm->mbox_client.tx_block     = false;
	cygnus_pm->mbox_client.tx_tout      = 2;
	cygnus_pm->mbox_client.knows_txdone = true;
	cygnus_pm->mbox_chan = mbox_request_channel(&cygnus_pm->mbox_client, 0);
	if (IS_ERR(cygnus_pm->mbox_chan)) {
		dev_err(&pdev->dev, "unable to get mbox channel\n");
		err = PTR_ERR(cygnus_pm->mbox_chan);
		goto err_iomap;
	}

	err = cygnus_pm_m0_init();
	if (err) {
		dev_err(&pdev->dev, "Fail to initailize M0!");
		goto err_free_mbox;
	}

	err = cygnus_pm_parse_aon_regulators_cfg(pdev);
	if (err) {
		dev_err(&pdev->dev,
			"Failed to configure aon regulators in device tree\n");
		goto err_free_mbox;
	}

	dev_info(&pdev->dev, "Cygnus Power Management supported\n");

	return 0;

err_free_mbox:
	mbox_free_channel(cygnus_pm->mbox_chan);

err_iomap:
	if (cygnus_pm->scu)
		iounmap(cygnus_pm->scu);
	if (cygnus_pm->crmu)
		iounmap(cygnus_pm->crmu);
	if (cygnus_pm->cru)
		iounmap(cygnus_pm->cru);
	/* Free immediately to ensure pointer isn't passed to suspend ops. */
	devm_kfree(&pdev->dev, cygnus_pm);
	cygnus_pm = NULL;

	return err;
}

static const struct of_device_id cygnus_pm_of_match[] = {
	{.compatible = "brcm,cygnus-pm", },
	{},
};
MODULE_DEVICE_TABLE(of, cygnus_pm_of_match);

static struct platform_driver cygnus_pm_platform_driver = {
	.driver = {
		.name = "cygnus-pm",
		.of_match_table = cygnus_pm_of_match,
	},
	.probe = cygnus_pm_probe,
};

/*
 * Register the cygnus power management driver.
 */
struct cygnus_pm * __init cygnus_pm_device_init(void)
{
	int err;

	err = platform_driver_register(&cygnus_pm_platform_driver);
	if (err)
		return NULL;

	return cygnus_pm;
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Cygnus Power Management Driver");
