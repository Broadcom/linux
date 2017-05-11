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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>

#define CDRU_USBPHY_CLK_RST_SEL_OFFSET			0x0
#define CDRU_USBPHY2_HOST_DEV_SEL_OFFSET		0x4
#define CDRU_USB_DEV_SUSPEND_RESUME_CTRL_OFFSET		0x5C
#define CDRU_USBPHY_P0_STATUS_OFFSET			0x1C
#define CDRU_USBPHY_P1_STATUS_OFFSET			0x34
#define CDRU_USBPHY_P2_STATUS_OFFSET			0x4C

#define CRMU_USB_PHY_AON_CTRL_OFFSET			0x0

#define CDRU_USBPHY_USBPHY_ILDO_ON_FLAG			1
#define CDRU_USBPHY_USBPHY_PLL_LOCK			0
#define CDRU_USB_DEV_SUSPEND_RESUME_CTRL_DISABLE	0

#define PHY2_DEV_HOST_CTRL_SEL_DEVICE			0
#define PHY2_DEV_HOST_CTRL_SEL_HOST			1
#define PHY2_DEV_HOST_CTRL_SEL_IDLE			2
#define CRMU_USBPHY_P0_AFE_CORERDY_VDDC			1
#define CRMU_USBPHY_P0_RESETB				2
#define CRMU_USBPHY_P1_AFE_CORERDY_VDDC			9
#define CRMU_USBPHY_P1_RESETB				10
#define CRMU_USBPHY_P2_AFE_CORERDY_VDDC			17
#define CRMU_USBPHY_P2_RESETB				18

#define USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET		0x0408
#define USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable	0
#define SUSPEND_OVERRIDE_0				13
#define SUSPEND_OVERRIDE_1				14
#define SUSPEND_OVERRIDE_2				15
#define USB2_IDM_IDM_RESET_CONTROL_OFFSET		0x0800
#define USB2_IDM_IDM_RESET_CONTROL__RESET		0
#define USB2D_IDM_IDM_IO_SS_CLEAR_NAK_NEMPTY_EN_I (1 << 24)

#define PLL_LOCK_RETRY_COUNT				1000
#define MAX_REGULATOR_NAME_LEN				25
#define DUAL_ROLE_PHY					2

#define USBPHY_WQ_DELAY_MS		msecs_to_jiffies(500)
#define USB2_SEL_DEVICE			0
#define USB2_SEL_HOST			1
#define USB2_SEL_IDLE			2
#define USB_CONNECTED			1
#define USB_DISCONNECTED		0
#define MAX_NUM_PHYS			3

static const int status_reg[] = {CDRU_USBPHY_P0_STATUS_OFFSET,
				CDRU_USBPHY_P1_STATUS_OFFSET,
				CDRU_USBPHY_P2_STATUS_OFFSET};

struct bcm_usb_extcon_info {
	struct extcon_dev *edev;
	struct gpio_desc *id_gpiod;
	int id_irq;
	struct gpio_desc *vbus_gpiod;
	int vbus_irq;
	spinlock_t lock;
	unsigned int connect_mode;
};

struct bcm_phy_instance;

struct bcm_phy_driver {
	void __iomem *cdru_usbphy_regs;
	void __iomem *crmu_usbphy_aon_ctrl_regs;
	void __iomem *usb2h_idm_regs;
	void __iomem *usb2d_idm_regs;
	int num_phys;
	bool idm_host_enabled;
	struct bcm_phy_instance *instances;
	int phyto_src_clk;
	struct platform_device *pdev;
};

struct bcm_phy_instance {
	struct bcm_phy_driver *driver;
	struct phy *generic_phy;
	int port;
	int new_state;		/* 1 - Host , 0 - device, 2 - idle*/
	int current_state;	/* 1 - Host , 0 - device, 2 - idle*/
	bool power;		/* 1 -powered_on 0 -powered off */
	struct regulator *vbus_supply;
	spinlock_t lock;
	struct bcm_usb_extcon_info extcon;
	struct extcon_specific_cable_nb extcon_dev;
	struct extcon_specific_cable_nb extcon_host;
	struct notifier_block host_nb;
	struct notifier_block dev_nb;
	struct delayed_work conn_work;
};

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static inline int bcm_phy_cdru_usbphy_status_wait(u32 usb_reg, int reg_bit,
					  struct bcm_phy_driver *phy_driver)
{
	/* Wait for the PLL lock status */
	int retry = PLL_LOCK_RETRY_COUNT;
	u32 reg_val;

	do {
		udelay(1);
		reg_val = readl(phy_driver->cdru_usbphy_regs +
				usb_reg);
		if (reg_val & (1 << reg_bit))
			return 0;
	} while (--retry > 0);

	return -EBUSY;

}

static struct phy *bcm_usb_phy_xlate(struct device *dev,
				     struct of_phandle_args *args)
{
	struct bcm_phy_driver *phy_driver = dev_get_drvdata(dev);
	struct bcm_phy_instance *instance_ptr;

	if (!phy_driver)
		return ERR_PTR(-EINVAL);

	if (WARN_ON(args->args[0] >= phy_driver->num_phys))
		return ERR_PTR(-ENODEV);

	if (WARN_ON(args->args_count < 1))
		return ERR_PTR(-EINVAL);

	instance_ptr = &phy_driver->instances[args->args[0]];
	if (instance_ptr->port > phy_driver->phyto_src_clk)
		phy_driver->phyto_src_clk = instance_ptr->port;

	if (instance_ptr->port == DUAL_ROLE_PHY)
		goto ret_p2;
	phy_driver->instances[instance_ptr->port].new_state =
						PHY2_DEV_HOST_CTRL_SEL_HOST;

ret_p2:
	return phy_driver->instances[instance_ptr->port].generic_phy;
}

static void bcm_usbp2_dev_clock_init(struct phy *generic_phy)
{
	u32 reg_val;
	struct bcm_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct bcm_phy_driver *phy_driver = instance_ptr->driver;

	/* Enable clock to USB device and take the USB device out of reset */
	reg_val = readl(phy_driver->usb2d_idm_regs +
			USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
	reg_val |= BIT(USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
	writel(reg_val, phy_driver->usb2d_idm_regs +
			USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

	reg_val = readl(phy_driver->usb2d_idm_regs +
			USB2_IDM_IDM_RESET_CONTROL_OFFSET);
	reg_val &= ~BIT(USB2_IDM_IDM_RESET_CONTROL__RESET);
	writel(reg_val, phy_driver->usb2d_idm_regs +
			USB2_IDM_IDM_RESET_CONTROL_OFFSET);
}

static void bcm_usbp2_dev_clock_deinit(struct phy *generic_phy)
{
	u32 reg_val;
	struct bcm_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct bcm_phy_driver *phy_driver = instance_ptr->driver;

	/* Disable clock to USB device and reset the USB device */
	reg_val = readl(phy_driver->usb2d_idm_regs +
			USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
	reg_val &= ~BIT(USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
	writel(reg_val, phy_driver->usb2d_idm_regs +
			USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

	reg_val = readl(phy_driver->usb2d_idm_regs +
			USB2_IDM_IDM_RESET_CONTROL_OFFSET);
	reg_val |= BIT(USB2_IDM_IDM_RESET_CONTROL__RESET);
	writel(reg_val, phy_driver->usb2d_idm_regs +
			USB2_IDM_IDM_RESET_CONTROL_OFFSET);
}

static void bcm_phy_clk_reset_src_switch(struct phy *generic_phy, u32 src_phy)
{
	u32 reg_val;
	struct bcm_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct bcm_phy_driver *phy_driver = instance_ptr->driver;

	writel(src_phy, phy_driver->cdru_usbphy_regs +
		CDRU_USBPHY_CLK_RST_SEL_OFFSET);

	/* Force the clock/reset source phy to not suspend */
	reg_val = readl(phy_driver->usb2h_idm_regs +
			USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
	reg_val &= ~(BIT(SUSPEND_OVERRIDE_0) |
			BIT(SUSPEND_OVERRIDE_1) |
			BIT(SUSPEND_OVERRIDE_2));
	reg_val |= BIT(SUSPEND_OVERRIDE_0 + src_phy);

	writel(reg_val, phy_driver->usb2h_idm_regs +
		USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
}

static int bcm_phy_init(struct phy *generic_phy)
{
	u32 reg_val;
	unsigned long flags;
	struct bcm_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct bcm_phy_driver *phy_driver = instance_ptr->driver;

	spin_lock_irqsave(&instance_ptr->lock, flags);

	/*
	 * Only PORT 2 is capabale of being device and host
	 * Default setting is device, check if it is set to host
	 */
	if (instance_ptr->port == DUAL_ROLE_PHY) {
		if (instance_ptr->new_state == PHY2_DEV_HOST_CTRL_SEL_HOST) {
			writel(PHY2_DEV_HOST_CTRL_SEL_HOST,
				phy_driver->cdru_usbphy_regs +
				CDRU_USBPHY2_HOST_DEV_SEL_OFFSET);
		} else {
			/*
			 * Disable suspend/resume signals to device controller
			 * when a port is in device mode
			 */
			writel(PHY2_DEV_HOST_CTRL_SEL_DEVICE,
				phy_driver->cdru_usbphy_regs +
				CDRU_USBPHY2_HOST_DEV_SEL_OFFSET);
			reg_val = readl(phy_driver->cdru_usbphy_regs +
				      CDRU_USB_DEV_SUSPEND_RESUME_CTRL_OFFSET);
			reg_val |=
				BIT(CDRU_USB_DEV_SUSPEND_RESUME_CTRL_DISABLE);
			writel(reg_val, phy_driver->cdru_usbphy_regs +
				CDRU_USB_DEV_SUSPEND_RESUME_CTRL_OFFSET);
		}
	}
	spin_unlock_irqrestore(&instance_ptr->lock, flags);

	return 0;
}

static int bcm_phy_shutdown(struct phy *generic_phy)
{
	u32 reg_val;
	int i;
	unsigned long flags;
	bool power_off_flag = true;
	struct bcm_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct bcm_phy_driver *phy_driver = instance_ptr->driver;
	u32 extcon_event = instance_ptr->new_state;

	spin_lock_irqsave(&instance_ptr->lock, flags);

	/* power down the phy */
	reg_val = readl(phy_driver->crmu_usbphy_aon_ctrl_regs +
			CRMU_USB_PHY_AON_CTRL_OFFSET);
	if (instance_ptr->port == 0) {
		reg_val &= ~BIT(CRMU_USBPHY_P0_AFE_CORERDY_VDDC);
		reg_val &= ~BIT(CRMU_USBPHY_P0_RESETB);
	} else if (instance_ptr->port == 1) {
		reg_val &= ~BIT(CRMU_USBPHY_P1_AFE_CORERDY_VDDC);
		reg_val &= ~BIT(CRMU_USBPHY_P1_RESETB);
	} else if (instance_ptr->port == 2) {
		reg_val &= ~BIT(CRMU_USBPHY_P2_AFE_CORERDY_VDDC);
		reg_val &= ~BIT(CRMU_USBPHY_P2_RESETB);
	}
	writel(reg_val, phy_driver->crmu_usbphy_aon_ctrl_regs +
		CRMU_USB_PHY_AON_CTRL_OFFSET);

	instance_ptr->power = false;

	/* Determine whether all the phy's are powered off */
	for (i = 0; i < phy_driver->num_phys; i++) {
		if (phy_driver->instances[i].power == true) {
			power_off_flag = false;
			break;
		}
	}

	/*
	 * Put the host controller into reset state and
	 * disable clock if all the phy's are powered off
	 */
	if (power_off_flag) {
		reg_val = readl(phy_driver->usb2h_idm_regs +
			USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
		reg_val &=
		  ~BIT(USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
		writel(reg_val, phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

		reg_val = readl(phy_driver->usb2h_idm_regs +
				 USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		reg_val |= BIT(USB2_IDM_IDM_RESET_CONTROL__RESET);
		writel(reg_val, phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		phy_driver->idm_host_enabled = false;
	}
	instance_ptr->current_state = extcon_event;
	spin_unlock_irqrestore(&instance_ptr->lock, flags);

	return 0;
}

static int bcm_phy_poweron(struct phy *generic_phy)
{
	int ret;
	unsigned long flags;
	u32 reg_val;
	struct bcm_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct bcm_phy_driver *phy_driver = instance_ptr->driver;
	u32 extcon_event = instance_ptr->new_state;

	/*
	 * Switch on the regulator only if in HOST mode
	 */
	if (instance_ptr->vbus_supply &&
			extcon_event == PHY2_DEV_HOST_CTRL_SEL_HOST) {
		ret = regulator_enable(instance_ptr->vbus_supply);
		if (ret) {
			dev_err(&generic_phy->dev,
				"Failed to enable regulator\n");
			return ret;
		}
	}

	spin_lock_irqsave(&instance_ptr->lock, flags);

	/* Bring the AFE block out of reset to start powering up the PHY */
	reg_val = readl(phy_driver->crmu_usbphy_aon_ctrl_regs +
			CRMU_USB_PHY_AON_CTRL_OFFSET);
	if (instance_ptr->port == 0)
		reg_val |= BIT(CRMU_USBPHY_P0_AFE_CORERDY_VDDC);
	else if (instance_ptr->port == 1)
		reg_val |= BIT(CRMU_USBPHY_P1_AFE_CORERDY_VDDC);
	else if (instance_ptr->port == 2)
		reg_val |= BIT(CRMU_USBPHY_P2_AFE_CORERDY_VDDC);
	writel(reg_val, phy_driver->crmu_usbphy_aon_ctrl_regs +
		CRMU_USB_PHY_AON_CTRL_OFFSET);

	/* Check for power on and PLL lock */
	ret = bcm_phy_cdru_usbphy_status_wait(status_reg[instance_ptr->port],
		   CDRU_USBPHY_USBPHY_ILDO_ON_FLAG, phy_driver);
	if (ret < 0) {
		dev_err(&generic_phy->dev,
			"Timed out waiting for USBPHY_ILDO_ON_FLAG on port %d",
			instance_ptr->port);
		goto err_shutdown;
	}
	ret = bcm_phy_cdru_usbphy_status_wait(status_reg[instance_ptr->port],
		CDRU_USBPHY_USBPHY_PLL_LOCK, phy_driver);
	if (ret < 0) {
		dev_err(&generic_phy->dev,
			"Timed out waiting for USBPHY_PLL_LOCK on port %d",
			instance_ptr->port);
		goto err_shutdown;
	}

	instance_ptr->power = true;

	/* Check if the port 2 is configured for device */
	if (instance_ptr->port == 2 &&
		extcon_event == PHY2_DEV_HOST_CTRL_SEL_DEVICE) {

		/*
		 * Enable clock to USB device and take
		 * the USB device out of reset
		 */
		bcm_usbp2_dev_clock_init(instance_ptr->generic_phy);
	}

	/* Set clock source provider to be the last powered on phy */
	if (instance_ptr->port == phy_driver->phyto_src_clk)
		bcm_phy_clk_reset_src_switch(generic_phy,
				instance_ptr->port);

	if (phy_driver->idm_host_enabled != true &&
		extcon_event == PHY2_DEV_HOST_CTRL_SEL_HOST) {
		/* Enable clock to USB host and take the host out of reset */
		reg_val = readl(phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
		reg_val |= BIT(USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
		writel(reg_val, phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

		reg_val = readl(phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		reg_val &= ~BIT(USB2_IDM_IDM_RESET_CONTROL__RESET);
		writel(reg_val, phy_driver->usb2h_idm_regs +
				 USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		phy_driver->idm_host_enabled = true;
	}
	instance_ptr->current_state = extcon_event;
	spin_unlock_irqrestore(&instance_ptr->lock, flags);

	return 0;

err_shutdown:
	spin_unlock_irqrestore(&instance_ptr->lock, flags);
	bcm_phy_shutdown(generic_phy);
	return ret;
}

static void connect_work(struct work_struct *work)
{
	struct bcm_phy_instance *instance_ptr =
				container_of(to_delayed_work(work),
				struct bcm_phy_instance, conn_work);
	u32 extcon_event = instance_ptr->new_state;
	struct phy *generic_phy = instance_ptr->generic_phy;
	unsigned long flags;
	int ret;

	if (extcon_event == PHY2_DEV_HOST_CTRL_SEL_DEVICE ||
		extcon_event == PHY2_DEV_HOST_CTRL_SEL_HOST) {
		bcm_phy_init(generic_phy);

		spin_lock_irqsave(&instance_ptr->lock, flags);
		if (instance_ptr->port == 2 &&
			instance_ptr->new_state ==
			PHY2_DEV_HOST_CTRL_SEL_DEVICE) {
			/*
			 * Enable clock to USB device and take
			 * the USB device out of reset
			 */
			bcm_usbp2_dev_clock_init(generic_phy);
		}
		spin_unlock_irqrestore(&instance_ptr->lock, flags);

		/* Power on regulator only if transition from IDLE->HOST */
		if (instance_ptr->vbus_supply &&
		    instance_ptr->new_state == PHY2_DEV_HOST_CTRL_SEL_HOST) {
			ret = regulator_enable(instance_ptr->vbus_supply);
			if (ret) {
				dev_err(&generic_phy->dev,
					"Failed to enable regulator\n");
				return;
			}
		}
	} else if (extcon_event == PHY2_DEV_HOST_CTRL_SEL_IDLE) {

		/* Shutdown regulator only if transition from HOST->IDLE */
		if (instance_ptr->vbus_supply &&
		    instance_ptr->current_state ==
		    PHY2_DEV_HOST_CTRL_SEL_HOST) {
			ret = regulator_disable(instance_ptr->vbus_supply);
			if (ret) {
				dev_err(&generic_phy->dev,
					"Failed to disable regulator\n");
				return;
			}
		}
		spin_lock_irqsave(&instance_ptr->lock, flags);
		if (instance_ptr->port == 2 &&
			instance_ptr->current_state ==
					PHY2_DEV_HOST_CTRL_SEL_DEVICE) {
			/*
			 * Disable clock to USB device and take
			 * the USB device out of reset
			 */
			bcm_usbp2_dev_clock_deinit(generic_phy);
		}
		spin_unlock_irqrestore(&instance_ptr->lock, flags);
	}

	spin_lock_irqsave(&instance_ptr->lock, flags);
	instance_ptr->current_state = extcon_event;
	spin_unlock_irqrestore(&instance_ptr->lock, flags);
}

static int bcm_drd_device_notify(struct notifier_block *self,
			unsigned long event, void *ptr)
{
	unsigned int conn_type = event;
	struct bcm_phy_instance *instance_ptr =
			container_of(self, struct bcm_phy_instance, dev_nb);

	if (conn_type) {
		instance_ptr->new_state = PHY2_DEV_HOST_CTRL_SEL_DEVICE;
		schedule_delayed_work(&instance_ptr->conn_work, 0);
	} else {
		instance_ptr->new_state = PHY2_DEV_HOST_CTRL_SEL_IDLE;
		schedule_delayed_work(&instance_ptr->conn_work,
					USBPHY_WQ_DELAY_MS);
	}

	return NOTIFY_DONE;
}

static int bcm_drd_host_notify(struct notifier_block *self,
			unsigned long event, void *ptr)
{
	unsigned int conn_type;
	struct bcm_phy_instance *instance_ptr =
			container_of(self, struct bcm_phy_instance, host_nb);
	conn_type = event;

	if (conn_type) {
		instance_ptr->new_state = PHY2_DEV_HOST_CTRL_SEL_HOST;
		schedule_delayed_work(&instance_ptr->conn_work, 0);
	} else {
		instance_ptr->new_state = PHY2_DEV_HOST_CTRL_SEL_IDLE;
		schedule_delayed_work(&instance_ptr->conn_work,
					USBPHY_WQ_DELAY_MS);
	}

	return NOTIFY_DONE;
}

static void bcm_phy_drd_cable_init_state(struct bcm_phy_instance *instance_ptr)
{
	unsigned int vbus_presence, usb_id;
	struct bcm_usb_extcon_info *extcon = &instance_ptr->extcon;

	/*
	 * Check the initial state of vbus and id
	 * This is required if cable is kept connected
	 * when booting
	 */
	vbus_presence = gpiod_get_value(extcon->vbus_gpiod);
	usb_id = gpiod_get_value(extcon->id_gpiod);

	if (usb_id == 0) {
		extcon->connect_mode = USB2_SEL_HOST;
		extcon_set_cable_state_(extcon->edev, EXTCON_USB_HOST,
					USB_CONNECTED);
	} else if (vbus_presence == 1) {
		extcon->connect_mode = USB2_SEL_DEVICE;
		extcon_set_cable_state_(extcon->edev, EXTCON_USB,
					USB_CONNECTED);
	} else
		extcon->connect_mode = USB2_SEL_IDLE;
}

static void bcm_phy_drd_cable_state(struct bcm_phy_instance *instance_ptr,
					bool resume)
{
	unsigned int vbus_presence, usb_id, conn_mode;
	struct bcm_usb_extcon_info *extcon = &instance_ptr->extcon;
	unsigned long flags;
	unsigned int prev_mode;

	/*
	 * Get the vbus and id gpio values
	 * ID = 0 ==> Host
	 * ID = 1 and VBUS = 1 ==> Device
	 * ID = 1 and VBUS = 0 ==> No connection(idle)
	 */
	vbus_presence = gpiod_get_value(extcon->vbus_gpiod);
	usb_id = gpiod_get_value(extcon->id_gpiod);

	spin_lock_irqsave(&extcon->lock, flags);
	conn_mode = extcon->connect_mode;
	prev_mode = extcon->connect_mode;
	spin_unlock_irqrestore(&extcon->lock, flags);

	switch (prev_mode) {
	case USB2_SEL_IDLE:
	if (usb_id == 1) {
		if (vbus_presence == 1) {
			conn_mode = USB2_SEL_DEVICE;
			extcon_set_cable_state_(extcon->edev, EXTCON_USB,
						USB_CONNECTED);
		} else
			conn_mode = USB2_SEL_IDLE;
	} else {
		conn_mode = USB2_SEL_HOST;
		extcon_set_cable_state_(extcon->edev, EXTCON_USB_HOST,
					USB_CONNECTED);
	}
	break;

	case USB2_SEL_DEVICE:
	if (usb_id == 1) {
		if (vbus_presence == 0) {
			conn_mode = USB2_SEL_IDLE;
			extcon_set_cable_state_(extcon->edev, EXTCON_USB,
						USB_DISCONNECTED);
		} else
			conn_mode = USB2_SEL_DEVICE;
	} else {
		extcon_set_cable_state_(extcon->edev, EXTCON_USB,
					USB_DISCONNECTED);
		conn_mode = USB2_SEL_HOST;
		extcon_set_cable_state_(extcon->edev, EXTCON_USB_HOST,
					USB_CONNECTED);
	}
	break;

	case USB2_SEL_HOST:
	if (usb_id == 1) {
		conn_mode = USB2_SEL_IDLE;
		extcon_set_cable_state_(extcon->edev, EXTCON_USB_HOST,
					USB_DISCONNECTED);
		if (vbus_presence == 1 && resume) {
			conn_mode = USB2_SEL_DEVICE;
			extcon_set_cable_state_(extcon->edev, EXTCON_USB,
						USB_CONNECTED);
		}
	} else
		conn_mode = USB2_SEL_HOST;
	break;

	default:
		dev_err(&instance_ptr->generic_phy->dev, "Invalid case\n");
	}

	spin_lock_irqsave(&extcon->lock, flags);
	extcon->connect_mode = conn_mode;
	spin_unlock_irqrestore(&extcon->lock, flags);
}

static irqreturn_t bcm_phy_drd_vbus_id_isr(int irq, void *data)
{
	struct bcm_phy_instance *instance_ptr = (struct bcm_phy_instance *)data;

	bcm_phy_drd_cable_state(instance_ptr, false);

	return IRQ_HANDLED;
}

static int bcm_phy_drd_extcon_init(struct bcm_phy_instance *instance_ptr)
{
	struct device *dev = &instance_ptr->generic_phy->dev;
	struct bcm_usb_extcon_info *extcon = &instance_ptr->extcon;
	int ret;

	extcon->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR(extcon->edev)) {
		dev_err(dev, "Failed to allocate extcon device\n");
		return -ENOMEM;
	}

	extcon->edev->name = dev->of_node->name;
	extcon->edev->supported_cable = usb_extcon_cable;
	ret = devm_extcon_dev_register(dev, extcon->edev);
	if (ret < 0) {
		dev_err(dev, "Failed to register extcon device\n");
		return ret;
	}

	spin_lock_init(&extcon->lock);

	extcon->id_gpiod = devm_gpiod_get(dev, "id", GPIOD_IN);
	if (IS_ERR(extcon->id_gpiod)) {
		dev_err(dev, "Failed to get ID GPIO\n");
		return PTR_ERR(extcon->id_gpiod);
	}

	extcon->vbus_gpiod = devm_gpiod_get(dev, "vbus", GPIOD_IN);
	if (IS_ERR(extcon->vbus_gpiod)) {
		dev_err(dev, "Failed to get VBUS GPIO\n");
		return PTR_ERR(extcon->vbus_gpiod);
	}

	extcon->id_irq = gpiod_to_irq(extcon->id_gpiod);
	if (extcon->id_irq < 0) {
		dev_err(dev, "Failed to get ID IRQ\n");
		return extcon->id_irq;
	}

	ret = devm_request_threaded_irq(dev, extcon->id_irq, NULL,
						bcm_phy_drd_vbus_id_isr,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						"id", instance_ptr);
	if (ret < 0) {
		dev_err(dev, "Failed to request handler for ID IRQ\n");
		return ret;
	}

	extcon->vbus_irq = gpiod_to_irq(extcon->vbus_gpiod);
	if (extcon->vbus_irq < 0) {
		dev_err(dev, "Failed to get VBUS IRQ\n");
		return extcon->vbus_irq;
	}

	ret = devm_request_threaded_irq(dev, extcon->vbus_irq, NULL,
							bcm_phy_drd_vbus_id_isr,
							IRQF_TRIGGER_RISING |
							IRQF_TRIGGER_FALLING |
							IRQF_ONESHOT,
							"vbus", instance_ptr);
	if (ret < 0) {
		dev_err(dev, "Failed to request handler for VBUS IRQ\n");
		return ret;
	}

	instance_ptr->host_nb.notifier_call = bcm_drd_host_notify;
	instance_ptr->dev_nb.notifier_call = bcm_drd_device_notify;

	/*
	 * Register for the USB device mode connection
	 * state change notification
	 */
	ret = extcon_register_notifier(extcon->edev,
			EXTCON_USB,
			&instance_ptr->dev_nb);
	if (ret < 0) {
		pr_info("Cannot register extcon_dev for %s.\n",
				extcon->edev->name);
		return -EINVAL;
	}

	/*
	 * Register for the USB host mode connection
	 * state change notification
	 */
	ret = extcon_register_notifier(extcon->edev,
				EXTCON_USB_HOST,
				&instance_ptr->host_nb);
	if (ret < 0) {
		pr_info("Cannot register extcon_dev for %s.\n",
				extcon->edev->name);
		ret = -EINVAL;
		goto err_host;
	}

	/* set the initial vbus and id values */
	bcm_phy_drd_cable_init_state(instance_ptr);

	return 0;

err_host:
	extcon_unregister_notifier(extcon->edev,
				EXTCON_USB,
				&instance_ptr->dev_nb);
	return ret;
}

static const struct phy_ops ops = {
	.init		= bcm_phy_init,
	.power_on	= bcm_phy_poweron,
	.power_off	= bcm_phy_shutdown,
};

static int bcm_phy_instance_create(struct bcm_phy_driver *phy_driver)
{
	struct device_node *child;
	char *vbus_name;
	struct platform_device *pdev = phy_driver->pdev;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct bcm_phy_instance *instance_ptr;
	unsigned int id, ret;

	for_each_available_child_of_node(node, child) {
		if (of_property_read_u32(child, "reg", &id)) {
			dev_err(dev, "missing reg property for %s\n",
				child->name);
			ret = -EINVAL;
			goto put_child;
		}

		if (id >= MAX_NUM_PHYS) {
			dev_err(dev, "invalid PHY id: %u\n", id);
			ret = -EINVAL;
			goto put_child;
		}

		instance_ptr = &phy_driver->instances[id];
		instance_ptr->driver = phy_driver;
		instance_ptr->port = id;
		spin_lock_init(&instance_ptr->lock);

		if (instance_ptr->generic_phy) {
			dev_err(dev, "duplicated PHY id: %u\n", id);
			ret = -EINVAL;
			goto put_child;
		}

		instance_ptr->generic_phy =
				devm_phy_create(dev, child, &ops);
		if (IS_ERR(instance_ptr->generic_phy)) {
			dev_err(dev, "Failed to create usb phy %d", id);
			ret = PTR_ERR(instance_ptr->generic_phy);
			goto put_child;
		}

		vbus_name = devm_kzalloc(&instance_ptr->generic_phy->dev,
					MAX_REGULATOR_NAME_LEN,
					GFP_KERNEL);
		if (!vbus_name) {
			ret = -ENOMEM;
			goto put_child;
		}

		/* regulator use is optional */
		sprintf(vbus_name, "vbus-p%d", id);
		instance_ptr->vbus_supply =
			devm_regulator_get(&instance_ptr->generic_phy->dev,
					vbus_name);
		if (IS_ERR(instance_ptr->vbus_supply))
			instance_ptr->vbus_supply = NULL;
		devm_kfree(&instance_ptr->generic_phy->dev, vbus_name);
		phy_set_drvdata(instance_ptr->generic_phy, instance_ptr);
	}

	return 0;

put_child:
	of_node_put(child);
	return ret;
}

static int bcm_phy_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct bcm_phy_driver *phy_driver;
	struct phy_provider *phy_provider;
	int ret;
	u32 reg_val;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;

	/* allocate memory for each phy instance */
	phy_driver = devm_kzalloc(dev, sizeof(struct bcm_phy_driver),
				  GFP_KERNEL);
	if (!phy_driver)
		return -ENOMEM;

	phy_driver->num_phys = of_get_child_count(node);

	if (phy_driver->num_phys == 0) {
		dev_err(dev, "PHY no child node\n");
		return -ENODEV;
	}

	phy_driver->instances = devm_kcalloc(dev, phy_driver->num_phys,
						sizeof(struct bcm_phy_instance),
						GFP_KERNEL);

	phy_driver->pdev = pdev;
	platform_set_drvdata(pdev, phy_driver);

	ret = bcm_phy_instance_create(phy_driver);
	if (ret)
		return ret;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"crmu-usbphy-aon-ctrl");
	phy_driver->crmu_usbphy_aon_ctrl_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy_driver->crmu_usbphy_aon_ctrl_regs))
		return PTR_ERR(phy_driver->crmu_usbphy_aon_ctrl_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"cdru-usbphy");
	phy_driver->cdru_usbphy_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy_driver->cdru_usbphy_regs))
		return PTR_ERR(phy_driver->cdru_usbphy_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "usb2h-idm");
	phy_driver->usb2h_idm_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy_driver->usb2h_idm_regs))
		return PTR_ERR(phy_driver->usb2h_idm_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "usb2d-idm");
	phy_driver->usb2d_idm_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy_driver->usb2d_idm_regs))
		return PTR_ERR(phy_driver->usb2d_idm_regs);

	reg_val = readl(phy_driver->usb2d_idm_regs);
	writel((reg_val | USB2D_IDM_IDM_IO_SS_CLEAR_NAK_NEMPTY_EN_I),
		   phy_driver->usb2d_idm_regs);
	phy_driver->idm_host_enabled = false;

	/*
	 * Shutdown all ports. They can be powered up as
	 * required
	 */
	reg_val = readl(phy_driver->crmu_usbphy_aon_ctrl_regs +
			CRMU_USB_PHY_AON_CTRL_OFFSET);
	reg_val &= ~BIT(CRMU_USBPHY_P0_AFE_CORERDY_VDDC);
	reg_val &= ~BIT(CRMU_USBPHY_P0_RESETB);
	reg_val &= ~BIT(CRMU_USBPHY_P1_AFE_CORERDY_VDDC);
	reg_val &= ~BIT(CRMU_USBPHY_P1_RESETB);
	reg_val &= ~BIT(CRMU_USBPHY_P2_AFE_CORERDY_VDDC);
	reg_val &= ~BIT(CRMU_USBPHY_P2_RESETB);
	writel(reg_val, phy_driver->crmu_usbphy_aon_ctrl_regs +
		CRMU_USB_PHY_AON_CTRL_OFFSET);

	phy_provider = devm_of_phy_provider_register(dev,
					bcm_usb_phy_xlate);

	if (IS_ERR(phy_provider)) {
		dev_err(dev, "Failed to register as phy provider\n");
		ret = PTR_ERR(phy_provider);
		return ret;
	}

	INIT_DELAYED_WORK(&phy_driver->instances[DUAL_ROLE_PHY].conn_work,
			connect_work);

	ret = bcm_phy_drd_extcon_init(&phy_driver->instances[DUAL_ROLE_PHY]);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id bcm_phy_dt_ids[] = {
	{ .compatible = "brcm,cygnus-usb-phy", },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm_phy_dt_ids);

static struct platform_driver bcm_phy_driver = {
	.probe = bcm_phy_probe,
	.driver = {
		.name = "bcm-cygnus-usbphy",
		.of_match_table = of_match_ptr(bcm_phy_dt_ids),
	},
};
module_platform_driver(bcm_phy_driver);

MODULE_ALIAS("platform:bcm-cygnus-usbphy");
MODULE_AUTHOR("Raveendra Padasalagi <Raveendra.padasalagi@broadcom.com");
MODULE_DESCRIPTION("Broadcom Cygnus USB PHY driver");
MODULE_LICENSE("GPL v2");
