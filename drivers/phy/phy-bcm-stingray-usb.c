/*
 * Copyright (C) 2016 Broadcom
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

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#define DRDU2_U2PLL_NDIV_FRAC		0x0

#define DRDU2_U2PLL_NDIV_INT		0x4

#define DRDU2_U2PLL_CTRL		0x8
#define DRDU2_U2PLL_LOCK		BIT(6)
#define DRDU2_U2PLL_RESETB		BIT(5)
#define DRDU2_U2PLL_PDIV_MASK		0xF
#define DRDU2_U2PLL_PDIV_OFFSET		1
#define DRDU2_U2PLL_SUSPEND_EN		BIT(0)

#define DRDU2_PHY_CTRL			0x0C
#define DRDU2_U2IDDQ			BIT(30)
#define DRDU2_U2SOFT_RST_N		BIT(29)
#define DRDU2_U2PHY_ON_FLAG		BIT(22)
#define DRDU2_U2PHY_PCTL_MASK		0xFFFF
#define DRDU2_U2PHY_PCTL_OFFSET		6
#define DRDU2_U2PHY_RESETB		BIT(5)
#define DRDU2_U2PHY_ISO			BIT(4)
#define DRDU2_U2AFE_BG_PWRDWNB		BIT(3)
#define DRDU2_U2AFE_PLL_PWRDWNB		BIT(2)
#define DRDU2_U2AFE_LDO_PWRDWNB		BIT(1)
#define DRDU2_U2CTRL_CORERDY		BIT(0)

#define DRDU2_STRAP_CTRL		0x18
#define DRDU2_FORCE_HOST_MODE		BIT(5)
#define DRDU2_FORCE_DEVICE_MODE		BIT(4)
#define BDC_USB_STP_SPD_MASK		0x7
#define BDC_USB_STP_SPD_OFFSET		0

#define DRDU2_PWR_CTRL				0x1C
#define DRDU2_U2PHY_DFE_SWITCH_PWROKIN_I	BIT(2)
#define DRDU2_U2PHY_DFE_SWITCH_PWRONIN_I	BIT(1)

#define DRDU2_SOFT_RESET_CTRL		0x20
#define DRDU2_BDC_AXI_SOFT_RST_N	BIT(0)

#define USB3H_U2PLL_NDIV_FRAC		0x4

#define USB3H_U2PLL_NDIV_INT		0x8

#define USB3H_U2PLL_CTRL		0xC
#define USB3H_U2PLL_LOCK		BIT(6)
#define USB3H_U2PLL_RESETB		BIT(5)
#define USB3H_U2PLL_PDIV_MASK		0xF
#define USB3H_U2PLL_PDIV_OFFSET		1

#define USB3H_U2PHY_CTRL		0x10
#define USB3H_U2PHY_ON_FLAG		22
#define USB3H_U2PHY_PCTL_MASK		0xFFFF
#define USB3H_U2PHY_PCTL_OFFSET		6
#define USB3H_U2PHY_RESETB		BIT(5)
#define USB3H_U2PHY_ISO			BIT(4)
#define USB3H_U2AFE_BG_PWRDWNB		BIT(3)
#define USB3H_U2AFE_PLL_PWRDWNB		BIT(2)
#define USB3H_U2AFE_LDO_PWRDWNB		BIT(1)
#define USB3H_U2CTRL_CORERDY		BIT(0)

#define USB3H_U3PHY_CTRL		0x14
#define USB3H_U3SOFT_RST_N		BIT(30)
#define USB3H_U3MDIO_RESETB_I		BIT(29)
#define USB3H_U3POR_RESET_I		BIT(28)
#define USB3H_U3PHY_PCTL_MASK		0xFFFF
#define USB3H_U3PHY_PCTL_OFFSET		2
#define USB3H_U3PHY_RESETB		BIT(1)

#define USB3H_U3PHY_PLL_CTRL		0x18
#define USB3H_U3PLL_REFCLK_MASK		0x7
#define USB3H_U3PLL_REFCLK_OFFSET	4
#define USB3H_U3PLL_SS_LOCK		BIT(3)
#define USB3H_U3PLL_SEQ_START		BIT(2)
#define USB3H_U3SSPLL_SUSPEND_EN	BIT(1)
#define USB3H_U3PLL_RESETB		BIT(0)

#define USB3H_PWR_CTRL			0x28
#define USB3H_PWR_CTRL_OVERRIDE_I_R	4

#define USB3H_SOFT_RESET_CTRL		0x2C
#define USB3H_XHC_AXI_SOFT_RST_N	BIT(1)

#define USB3H_PHY_PWR_CTRL		0x38
#define USB3H_DISABLE_USB30_P0		BIT(2)
#define USB3H_DISABLE_EUSB_P1		BIT(1)
#define USB3H_DISABLE_EUSB_P0		BIT(0)


#define DRDU3_U2PLL_NDIV_FRAC		0x4

#define DRDU3_U2PLL_NDIV_INT		0x8

#define DRDU3_U2PLL_CTRL		0xC
#define DRDU3_U2PLL_LOCK		BIT(6)
#define DRDU3_U2PLL_RESETB		BIT(5)
#define DRDU3_U2PLL_PDIV_MASK		0xF
#define DRDU3_U2PLL_PDIV_OFFSET		1

#define DRDU3_U2PHY_CTRL		0x10
#define DRDU3_U2PHY_ON_FLAG		BIT(22)
#define DRDU3_U2PHY_PCTL_MASK		0xFFFF
#define DRDU3_U2PHY_PCTL_OFFSET		6
#define DRDU3_U2PHY_RESETB		BIT(5)
#define DRDU3_U2PHY_ISO			BIT(4)
#define DRDU3_U2AFE_BG_PWRDWNB		BIT(3)
#define DRDU3_U2AFE_PLL_PWRDWNB		BIT(2)
#define DRDU3_U2AFE_LDO_PWRDWNB		BIT(1)
#define DRDU3_U2CTRL_CORERDY		BIT(0)

#define DRDU3_U3PHY_CTRL		0x14
#define DRDU3_U3XHC_SOFT_RST_N		BIT(31)
#define DRDU3_U3BDC_SOFT_RST_N		BIT(30)
#define DRDU3_U3MDIO_RESETB_I		BIT(29)
#define DRDU3_U3POR_RESET_I		BIT(28)
#define DRDU3_U3PHY_PCTL_MASK		0xFFFF
#define DRDU3_U3PHY_PCTL_OFFSET		2
#define DRDU3_U3PHY_RESETB		BIT(1)

#define DRDU3_U3PHY_PLL_CTRL		0x18
#define DRDU3_U3PLL_REFCLK_MASK		0x7
#define DRDU3_U3PLL_REFCLK_OFFSET	4
#define DRDU3_U3PLL_SS_LOCK		BIT(3)
#define DRDU3_U3PLL_SEQ_START		BIT(2)
#define DRDU3_U3SSPLL_SUSPEND_EN	BIT(1)
#define DRDU3_U3PLL_RESETB		BIT(0)

#define DRDU3_STRAP_CTRL		0x28
#define BDC_USB_STP_SPD_MASK		0x7
#define BDC_USB_STP_SPD_OFFSET		0
#define BDC_USB_STP_SPD_SS		0x0
#define BDC_USB_STP_SPD_HS		0x2

#define DRDU3_PWR_CTRL			0x2c
#define DRDU3_PWR_CTRL_OVERRIDE_I_R	4

#define DRDU3_SOFT_RESET_CTRL		0x30
#define DRDU3_XHC_AXI_SOFT_RST_N	BIT(1)
#define DRDU3_BDC_AXI_SOFT_RST_N	BIT(0)

#define DRDU3_PHY_PWR_CTRL		0x3c
#define DRDU3_DISABLE_USB30_P0		BIT(2)
#define DRDU3_DISABLE_EUSB_P1		BIT(1)
#define DRDU3_DISABLE_EUSB_P0		BIT(0)

#define PLL_REFCLK_PAD		0x0
#define PLL_REFCLK_25MHZ	0x1
#define PLL_REFCLK_96MHZ	0x2
#define PLL_REFCLK_INTERNAL	0x3
#define PLL_LOCK_RETRY_COUNT	1000


#define U2PLL_NDIV_INT_VAL	0x13
#define U2PLL_NDIV_FRAC_VAL	0x1005
#define U2PLL_PDIV_VAL		0x1
/*
 * 0x0806 of PCTL_VAL has below bits set
 * BIT-8 : refclk divider 1
 * BIT-3:2: device mode; mode is not effect
 * BIT-1: soft reset active low
 */
#define U2PHY_PCTL_VAL	0x0806
#define U3PHY_PCTL_VAL	0x0006

#define MAX_NR_PORTS	3

#define USB3H_DRDU2_PHY	1
#define DRDU3_PHY	2

#define USB_HOST_MODE	1
#define USB_DEV_MODE	2

#define USB3SS_PORT	0
#define DRDU2_PORT	1
#define USB3HS_PORT	2

#define DRD3SS_PORT	0
#define DRD3HS_PORT	1

struct sr_usb_phy {
	void __iomem *drdu2reg;
	void __iomem *usb3hreg;
	void __iomem *drdu3reg;
	uint32_t phy_id;
	struct sr_usb_phy_port *phy_port;
};

struct sr_usb_phy_port {
	uint32_t port_id;
	uint32_t mode;
	struct phy *gphy;
	struct sr_usb_phy *p;
};

static inline void reg32_clrbits(void __iomem *addr, uint32_t clear)
{
	writel(readl(addr) & ~clear, addr);
}

static inline void reg32_setbits(void __iomem *addr, uint32_t set)
{
	writel(readl(addr) | set, addr);
}

static int pll_lock_check(void __iomem *addr, u32 bit)
{
	int retry;
	u32 rd_data;

	retry = PLL_LOCK_RETRY_COUNT;
	do {
		rd_data = readl(addr);
		if (rd_data & bit)
			return 0;
		udelay(1);
	} while (--retry > 0);

	pr_err("%s: FAIL\n", __func__);
	return -ETIMEDOUT;
}

static struct phy *bcm_usb_phy_xlate(struct device *dev,
		struct of_phandle_args *args)
{
	struct sr_usb_phy_port *phy_port = dev_get_drvdata(dev);
	struct sr_usb_phy *p;

	if (!phy_port)
		return ERR_PTR(-EINVAL);

	p = phy_port->p;

	if (WARN_ON(args->args[0] < 1 || args->args[0] > 2))
		return ERR_PTR(-EINVAL);

	if (WARN_ON(args->args_count < 1))
		return ERR_PTR(-EINVAL);

	if ((p->phy_id == USB3H_DRDU2_PHY) && (phy_port->port_id == 0) &&
		(args->args[0] == USB_DEV_MODE))
		return ERR_PTR(-EINVAL);

	phy_port->mode |= args->args[0];

	return phy_port->gphy;
}

static int usb3h_u2_phy_power_on(void __iomem *regs)
{
	u32 rd_data;
	int ret = 0;

	/* Set Core Ready high */
	reg32_setbits(regs + USB3H_U2PHY_CTRL, USB3H_U2CTRL_CORERDY);
	msleep(100);

	rd_data = readl(regs + USB3H_U2PHY_CTRL);
	rd_data &= ~(USB3H_U2PHY_PCTL_MASK << USB3H_U2PHY_PCTL_OFFSET);
	rd_data |= (U2PHY_PCTL_VAL << USB3H_U2PHY_PCTL_OFFSET);
	writel(rd_data, regs + USB3H_U2PHY_CTRL);

	msleep(300);
	reg32_setbits(regs + USB3H_U2PLL_CTRL,
			USB3H_U2PLL_RESETB);

	reg32_setbits(regs + USB3H_U2PHY_CTRL,
			USB3H_U2PHY_RESETB);

	ret = pll_lock_check(regs + USB3H_U2PLL_CTRL, USB3H_U2PLL_LOCK);

	return ret;
}


static int usb3h_u3_phy_power_on(void __iomem *regs)
{
	u32 rd_data;
	int ret = 0;

	/* Set pctl with mode and soft reset */
	rd_data = readl(regs + USB3H_U3PHY_CTRL);
	rd_data &= ~(USB3H_U3PHY_PCTL_MASK << USB3H_U3PHY_PCTL_OFFSET);
	rd_data |= (U3PHY_PCTL_VAL << USB3H_U3PHY_PCTL_OFFSET);
	writel(rd_data, regs + USB3H_U3PHY_CTRL);

	reg32_clrbits(regs + USB3H_U3PHY_PLL_CTRL,
			USB3H_U3SSPLL_SUSPEND_EN);
	reg32_setbits(regs + USB3H_U3PHY_PLL_CTRL,
			USB3H_U3PLL_SEQ_START);
	reg32_setbits(regs + USB3H_U3PHY_PLL_CTRL,
			USB3H_U3PLL_RESETB);

	msleep(30);

	ret = pll_lock_check(regs + USB3H_U3PHY_PLL_CTRL, USB3H_U3PLL_SS_LOCK);

	return ret;
}


static int drdu3_u2_phy_power_on(void __iomem *regs)
{
	u32 rd_data;
	int ret = 0;

	/* Set Core Ready high */
	reg32_setbits(regs + DRDU3_U2PHY_CTRL, DRDU3_U2CTRL_CORERDY);

	msleep(100);

	rd_data = readl(regs + DRDU3_STRAP_CTRL);
	rd_data &= (~(BDC_USB_STP_SPD_MASK << BDC_USB_STP_SPD_OFFSET));
	rd_data |= (BDC_USB_STP_SPD_SS << BDC_USB_STP_SPD_OFFSET);
	writel(rd_data, regs + DRDU3_STRAP_CTRL);

	rd_data = readl(regs + DRDU3_U2PHY_CTRL);
	rd_data &= ~(DRDU3_U2PHY_PCTL_MASK << DRDU3_U2PHY_PCTL_OFFSET);
	rd_data |= (U2PHY_PCTL_VAL << DRDU3_U2PHY_PCTL_OFFSET);
	writel(rd_data, regs + DRDU3_U2PHY_CTRL);

	msleep(300);

	reg32_setbits(regs + DRDU3_U2PLL_CTRL,
			DRDU3_U2PLL_RESETB);

	reg32_setbits(regs + DRDU3_U2PHY_CTRL,
			DRDU3_U2PHY_RESETB);

	ret = pll_lock_check(regs + DRDU3_U2PLL_CTRL, DRDU3_U2PLL_LOCK);

	return ret;
}

static int drdu3_u3_phy_power_on(void __iomem *regs)
{
	u32 rd_data;
	int ret = 0;

	/* Set pctl with mode and soft reset */
	rd_data = readl(regs + DRDU3_U3PHY_CTRL);
	rd_data &= ~(DRDU3_U3PHY_PCTL_MASK << DRDU3_U3PHY_PCTL_OFFSET);
	rd_data |= (U3PHY_PCTL_VAL << DRDU3_U3PHY_PCTL_OFFSET);
	writel(rd_data, regs + DRDU3_U3PHY_CTRL);

	reg32_clrbits(regs + DRDU3_U3PHY_PLL_CTRL,
			DRDU3_U3SSPLL_SUSPEND_EN);
	reg32_setbits(regs + DRDU3_U3PHY_PLL_CTRL,
			DRDU3_U3PLL_SEQ_START);
	reg32_setbits(regs + DRDU3_U3PHY_PLL_CTRL,
			DRDU3_U3PLL_RESETB);

	msleep(30);

	ret = pll_lock_check(regs + DRDU3_U3PHY_PLL_CTRL, DRDU3_U3PLL_SS_LOCK);

	return ret;
}


static int drdu2_u2_phy_power_on(void __iomem *regs)
{
	u32 rd_data;
	int ret = 0;

	writel(U2PLL_NDIV_INT_VAL, regs + DRDU2_U2PLL_NDIV_INT);
	writel(U2PLL_NDIV_FRAC_VAL, regs + DRDU2_U2PLL_NDIV_FRAC);

	rd_data = readl(regs + DRDU2_U2PLL_CTRL);
	rd_data &= ~(DRDU2_U2PLL_PDIV_MASK << DRDU2_U2PLL_PDIV_OFFSET);
	rd_data |= (U2PLL_PDIV_VAL << DRDU2_U2PLL_PDIV_OFFSET);
	writel(rd_data, regs + DRDU2_U2PLL_CTRL);

	reg32_setbits(regs + DRDU2_PHY_CTRL, DRDU2_U2CTRL_CORERDY);

	msleep(100);
	reg32_setbits(regs + DRDU2_U2PLL_CTRL,
			DRDU2_U2PLL_RESETB);

	reg32_setbits(regs + DRDU2_PHY_CTRL,
			DRDU2_U2PHY_RESETB);

	rd_data = readl(regs + DRDU2_PHY_CTRL);
	rd_data &= ~(DRDU2_U2PHY_PCTL_MASK << DRDU2_U2PHY_PCTL_OFFSET);
	rd_data |= (U2PHY_PCTL_VAL << DRDU2_U2PHY_PCTL_OFFSET);
	writel(rd_data, regs + DRDU2_PHY_CTRL);

	ret = pll_lock_check(regs + DRDU2_U2PLL_CTRL, DRDU2_U2PLL_LOCK);

	return ret;
}
static int sr_u3h_u2drd_phy_reset(struct phy *gphy)
{
	struct sr_usb_phy_port *phy_port = phy_get_drvdata(gphy);
	struct sr_usb_phy *p = phy_port->p;

	switch (phy_port->port_id) {
	case USB3HS_PORT:
		reg32_clrbits(p->usb3hreg + USB3H_U2PHY_CTRL,
				USB3H_U2CTRL_CORERDY);
		reg32_setbits(p->usb3hreg + USB3H_U2PHY_CTRL,
				USB3H_U2CTRL_CORERDY);
		break;
	case DRDU2_PORT:
		reg32_clrbits(p->drdu2reg + DRDU2_PHY_CTRL,
				DRDU2_U2CTRL_CORERDY);
		reg32_setbits(p->drdu2reg + DRDU2_PHY_CTRL,
				DRDU2_U2CTRL_CORERDY);
		break;
	}
	return 0;
}

static int sr_u3drd_phy_reset(struct phy *gphy)
{
	struct sr_usb_phy_port *phy_port = phy_get_drvdata(gphy);
	struct sr_usb_phy *p = phy_port->p;

	switch (phy_port->port_id) {
	case DRD3HS_PORT:
		reg32_clrbits(p->drdu3reg + DRDU3_U2PHY_CTRL,
				DRDU3_U2CTRL_CORERDY);
		reg32_setbits(p->drdu3reg + DRDU3_U2PHY_CTRL,
				DRDU3_U2CTRL_CORERDY);
		break;
	}
	return 0;
}

static int sr_u3h_u2drd_phy_power_on(struct phy *gphy)
{
	struct sr_usb_phy_port *phy_port = phy_get_drvdata(gphy);
	struct sr_usb_phy *p = phy_port->p;
	int ret = 0;

	switch (phy_port->port_id) {
	case USB3SS_PORT:
		reg32_clrbits(p->usb3hreg + USB3H_PHY_PWR_CTRL,
				 USB3H_DISABLE_USB30_P0);
		ret = usb3h_u3_phy_power_on(p->usb3hreg);
		if (ret)
			goto err_usb3h_phy_on;
		break;
	case USB3HS_PORT:
		reg32_clrbits(p->usb3hreg + USB3H_PHY_PWR_CTRL,
				USB3H_DISABLE_EUSB_P1);
		ret = usb3h_u2_phy_power_on(p->usb3hreg);
		if (ret)
			goto err_usb3h_phy_on;

		break;
	case DRDU2_PORT:
		reg32_clrbits(p->usb3hreg + USB3H_PHY_PWR_CTRL,
				USB3H_DISABLE_EUSB_P0);

		ret = drdu2_u2_phy_power_on(p->drdu2reg);
		if (ret) {
			reg32_setbits(p->usb3hreg + USB3H_PHY_PWR_CTRL,
					USB3H_DISABLE_EUSB_P0);
			goto err_drdu2_phy_on;
		}
		break;
	}

	/* Device Mode */
	if (phy_port->port_id == DRDU2_PORT) {
		writel(DRDU2_BDC_AXI_SOFT_RST_N,
				p->drdu2reg + DRDU2_SOFT_RESET_CTRL);
		reg32_setbits(p->drdu2reg + DRDU2_PHY_CTRL,
				DRDU2_U2SOFT_RST_N);
	}
	/* Host Mode */
	writel(USB3H_XHC_AXI_SOFT_RST_N,
			p->usb3hreg + USB3H_SOFT_RESET_CTRL);
	reg32_setbits(p->usb3hreg + USB3H_U3PHY_CTRL,
			USB3H_U3SOFT_RST_N);

	return 0;
err_usb3h_phy_on:
	reg32_setbits(p->usb3hreg + USB3H_PHY_PWR_CTRL,
			(USB3H_DISABLE_EUSB_P1 |
			 USB3H_DISABLE_USB30_P0));
err_drdu2_phy_on:

	return ret;
}

static int sr_u3drd_phy_power_on(struct phy *gphy)
{
	struct sr_usb_phy_port *phy_port = phy_get_drvdata(gphy);
	struct sr_usb_phy *p = phy_port->p;
	int ret = 0;

	switch (phy_port->port_id) {
	case DRD3SS_PORT:
		reg32_clrbits(p->drdu3reg + USB3H_PHY_PWR_CTRL,
				DRDU3_DISABLE_USB30_P0);

		ret = drdu3_u3_phy_power_on(p->drdu3reg);
		if (ret)
			goto err_drdu3_phy_on;
		break;
	case DRD3HS_PORT:
		reg32_clrbits(p->drdu3reg + USB3H_PHY_PWR_CTRL,
				DRDU3_DISABLE_EUSB_P0);
			ret = drdu3_u2_phy_power_on(p->drdu3reg);
		if (ret)
			goto err_drdu3_phy_on;
		break;
	}
	/* Host Mode */
	reg32_setbits(p->drdu3reg + DRDU3_SOFT_RESET_CTRL,
			DRDU3_XHC_AXI_SOFT_RST_N);
	reg32_setbits(p->drdu3reg + DRDU3_U3PHY_CTRL,
			DRDU3_U3XHC_SOFT_RST_N);
	/* Device Mode */
	reg32_setbits(p->drdu3reg + DRDU3_SOFT_RESET_CTRL,
			DRDU3_BDC_AXI_SOFT_RST_N);
	reg32_setbits(p->drdu3reg + DRDU3_U3PHY_CTRL,
			DRDU3_U3BDC_SOFT_RST_N);

	return 0;
err_drdu3_phy_on:
	reg32_setbits(p->drdu3reg + DRDU3_PHY_PWR_CTRL,
			(DRDU3_DISABLE_EUSB_P0 |
			 DRDU3_DISABLE_USB30_P0));

	return ret;
}

static int sr_u3h_u2drd_phy_power_off(struct phy *gphy)
{
	struct sr_usb_phy_port *phy_port = phy_get_drvdata(gphy);
	struct sr_usb_phy *p = phy_port->p;

	switch (phy_port->port_id) {
	case USB3SS_PORT:
		reg32_setbits(p->usb3hreg + USB3H_PHY_PWR_CTRL,
				 USB3H_DISABLE_USB30_P0);
		break;
	case USB3HS_PORT:
		reg32_setbits(p->usb3hreg + USB3H_PHY_PWR_CTRL,
				USB3H_DISABLE_EUSB_P1);
		break;
	case DRDU2_PORT:
		reg32_setbits(p->usb3hreg + USB3H_PHY_PWR_CTRL,
				USB3H_DISABLE_EUSB_P0);
		break;
	}

	return 0;
}

static int sr_u3drd_phy_power_off(struct phy *gphy)
{
	struct sr_usb_phy_port *phy_port = phy_get_drvdata(gphy);
	struct sr_usb_phy *p = phy_port->p;

	switch (phy_port->port_id) {
	case DRD3SS_PORT:
		reg32_setbits(p->drdu3reg + DRDU3_PHY_PWR_CTRL,
				 DRDU3_DISABLE_USB30_P0);
		break;
	case DRD3HS_PORT:
		reg32_setbits(p->drdu3reg + DRDU3_PHY_PWR_CTRL,
				DRDU3_DISABLE_EUSB_P0);
		break;
	}
	return 0;
}

static struct phy_ops sr_u3h_u2drd_phy_ops = {
	.power_on	= sr_u3h_u2drd_phy_power_on,
	.power_off	= sr_u3h_u2drd_phy_power_off,
	.reset		= sr_u3h_u2drd_phy_reset,
	.owner		= THIS_MODULE,
};

static struct phy_ops sr_u3drd_phy_ops = {
	.power_on	= sr_u3drd_phy_power_on,
	.power_off	= sr_u3drd_phy_power_off,
	.reset		= sr_u3drd_phy_reset,
	.owner		= THIS_MODULE,
};

static const struct of_device_id sr_usb_phy_of_match[] = {
	{
		.compatible = "brcm,sr-u3h-u2drd-phy",
		.data = &sr_u3h_u2drd_phy_ops,
	},
	{
		.compatible = "brcm,sr-u3drd-phy",
		.data = &sr_u3drd_phy_ops,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, sr_usb_phy_of_match);

static int sr_usb_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sr_usb_phy *p;
	struct device_node *dn = dev->of_node, *child;
	struct resource *res;
	struct phy *gphy;
	const struct phy_ops *ops;
	struct phy_provider *phy_provider;
	const struct of_device_id *match;
	struct sr_usb_phy_port *phy_port;
	uint32_t child_cnt, cnt = 0;

	match = of_match_node(sr_usb_phy_of_match, dn);
	if (!match) {
		dev_err(dev, "of_match_node() failed\n");
		return -EINVAL;
	}
	ops = match->data;

	child_cnt = of_get_child_count(dn);
	if (!child_cnt || child_cnt > MAX_NR_PORTS)
		return -ENODEV;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->phy_port = devm_kcalloc(dev, child_cnt,
			sizeof(struct sr_usb_phy_port),
			GFP_KERNEL);
	if (!p->phy_port)
		return -ENOMEM;

	if (of_device_is_compatible(dn, "brcm,sr-u3h-u2drd-phy")) {
		void __iomem *drdu2reg, *usb3hreg;

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"drdu2");
		drdu2reg = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(drdu2reg))
			return PTR_ERR(drdu2reg);

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"usb3h");
		usb3hreg = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(usb3hreg))
			return PTR_ERR(usb3hreg);

		p->drdu2reg = drdu2reg;
		p->usb3hreg = usb3hreg;
		p->phy_id = USB3H_DRDU2_PHY;
	}
	if (of_device_is_compatible(dn, "brcm,sr-u3drd-phy")) {
		void __iomem *drdu3reg;

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"drdu3");
		drdu3reg = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(drdu3reg))
			return PTR_ERR(drdu3reg);

		p->drdu3reg = drdu3reg;
		p->phy_id = DRDU3_PHY;
	}

	for_each_available_child_of_node(dn, child) {
		phy_port = &p->phy_port[cnt];
		if (of_property_read_u32(child, "reg", &phy_port->port_id)) {
			dev_err(dev, "missing reg property in node %s\n",
					child->name);
			return -EINVAL;
		}
		if (phy_port->port_id >= MAX_NR_PORTS)
			return -EINVAL;

		gphy = devm_phy_create(dev, child, ops);
		if (IS_ERR(gphy))
			return PTR_ERR(gphy);

		phy_set_drvdata(gphy, phy_port);
		dev_set_drvdata(&gphy->dev, phy_port);
		phy_provider = devm_of_phy_provider_register(&gphy->dev,
				bcm_usb_phy_xlate);
		if (IS_ERR(phy_provider)) {
			dev_err(dev, "Failed to register phy provider\n");
			return PTR_ERR(phy_provider);
		}
		phy_port->gphy = gphy;
		phy_port->p = p;
		cnt++;
	}

	return 0;
}

static struct platform_driver sr_usb_phy_driver = {
	.driver = {
		.name = "phy-bcm-sr-usb",
		.of_match_table = sr_usb_phy_of_match,
	},
	.probe = sr_usb_phy_probe,
};
module_platform_driver(sr_usb_phy_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom stingray USB Phy driver");
MODULE_LICENSE("GPL v2");
