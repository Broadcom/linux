/*
 * Copyright (C) 2015 Broadcom Corporation
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

#include <linux/types.h>
#include <linux/mdio.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/kfifo.h>

#include "bcm-amac-regs.h"
#include "bcm-amac-enet.h"
#include "bcm-amac-core.h"

#define ADVERTISE_100M (ADVERTISE_100BASE4 | ADVERTISE_100FULL | \
				ADVERTISE_100HALF)

static int amac_gphy_advertise_100M(struct phy_device *phydev, bool enable)
{
	int adv;
	int rc = 0;

	adv = phy_read(phydev, MII_ADVERTISE);
	if (adv < 0)
		return adv;

	if (enable) {
		/* Enable 100M advertisement */
		if (!(adv & ADVERTISE_100M)) {
			/* 100BaseT4 is not supported in Cygnus,
			 * so don't enable it
			 */
			adv |= ADVERTISE_100FULL;
			adv |= ADVERTISE_100HALF;
			rc = phy_write(phydev, MII_ADVERTISE, adv);
		}
		phydev->supported |= (SUPPORTED_100baseT_Half);
		phydev->supported |= (SUPPORTED_100baseT_Full);
	} else {
		/* Disable 100M advertisement */
		if (adv & ADVERTISE_100M) {
			adv &= ~ADVERTISE_100M;
			rc = phy_write(phydev, MII_ADVERTISE, adv);
		}
		phydev->supported &= ~(SUPPORTED_100baseT_Half);
		phydev->supported &= ~(SUPPORTED_100baseT_Full);
	}
	return rc;
}

static int amac_gphy_advertise_1G(struct phy_device *phydev, bool enable)
{
	int adv;
	int rc = 0;

	adv = phy_read(phydev, MII_CTRL1000);
	if (adv < 0)
		return adv;

	if (enable) {
		/* Enable 1000M (1G) advertisement */
		if (!(adv & (ADVERTISE_1000FULL | ADVERTISE_1000HALF))) {
			adv |= ADVERTISE_1000FULL;
			adv |= ADVERTISE_1000HALF;
			rc = phy_write(phydev, MII_CTRL1000, adv);
		}
		phydev->supported |= (SUPPORTED_1000baseT_Half);
		phydev->supported |= (SUPPORTED_1000baseT_Full);
	} else {
		/* Disable 1000M (1G) advertisement */
		if (adv & (ADVERTISE_1000FULL | ADVERTISE_1000HALF)) {
			adv &= ~(ADVERTISE_1000FULL | ADVERTISE_1000HALF);
			rc = phy_write(phydev, MII_CTRL1000, adv);
		}
		phydev->supported &= ~(SUPPORTED_1000baseT_Half);
		phydev->supported &= ~(SUPPORTED_1000baseT_Full);
	}
	return rc;
}

static void amac_gphy_handle_link_change(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	struct phy_device *phydev = privp->port.info[AMAC_PORT_TYPE_EXT].phydev;
	struct port_status *port_stat =
		&privp->port.info[AMAC_PORT_TYPE_EXT].stat;

	/* Act on link, speed, duplex status changes */
	if ((phydev->link !=  port_stat->link) ||
	    (phydev->speed !=  port_stat->speed) ||
	    (phydev->duplex !=  port_stat->duplex)) {
		/* Update the new status */
		port_stat->link = phydev->link;
		port_stat->speed = phydev->speed;
		port_stat->duplex = phydev->duplex;

		/* send netlink update */
		bcm_amac_enet_netlink_send(privp,
					   phydev->addr,
					   phydev,
					   phydev->link);
	}
}

static void amac_gphy_enable(struct phy_device *phy_dev, int enable)
{
	int val;

	/* Read current value */
	val = phy_read(phy_dev, GPHY_MII_CTRL_REG);

	/* Set or Clear the Power and Reset bits */
	if (enable)
		val &= (~(GPHY_MII_CTRL_REG_RST_MASK |
			GPHY_MII_CTRL_REG_PWR_MASK));
	else
		val |= (GPHY_MII_CTRL_REG_PWR_MASK);

	phy_write(phy_dev, GPHY_MII_CTRL_REG, val);
}

int bcm_amac_gphy_init(struct bcm_amac_priv *privp)
{
	struct port_info *port = &privp->port.info[AMAC_PORT_TYPE_EXT];
	int rc;

	if (privp->port.count < 2)
		return 0;

	/* Connect and configure the PHY */
	port->phydev = of_phy_connect(privp->ndev,
					   port->phy_node,
					   amac_gphy_handle_link_change,
					   0,
					   (phy_interface_t)port->phy_mode);
	if (IS_ERR(port->phydev)) {
		dev_err(&privp->pdev->dev, "Failed to connect phy\n");
		return -ENODEV;
	}

	/* Apply settings */
	port->phydev->autoneg = port->cfg.aneg;
	port->phydev->speed = port->cfg.speed;
	port->phydev->duplex = port->cfg.duplex;
	port->phydev->pause = port->cfg.pause;

	/* Disable 1G advertisement for 10/100M ports */
	if ((port->phydev->speed == AMAC_PORT_SPEED_100M) ||
	    (port->phydev->speed == AMAC_PORT_SPEED_10M))
		amac_gphy_advertise_1G(port->phydev, false);

	/* Disable 100M advertisement for 10M ports */
	if (port->phydev->speed == AMAC_PORT_SPEED_10M)
		amac_gphy_advertise_100M(port->phydev,
					 false);

	rc = phy_start_aneg(port->phydev);
	if (rc < 0) {
		phy_disconnect(port->phydev);

		dev_err(&privp->pdev->dev,
			"Cannot start PHY: %d\n",
			port->phydev->addr);

		return -ENODEV;
	}

	dev_info(&privp->pdev->dev,
		 "Initialized PHY: %s\n",
		 dev_name(&port->phydev->dev));

	return 0;
}

/* bcm_amac_gphy_powerup() - Power up the PHY's
 * @privp: driver local data structure pointer
 */
void bcm_amac_gphy_powerup(struct bcm_amac_priv *privp)
{
	if (privp->port.count > 1)
		if (privp->port.info[AMAC_PORT_TYPE_EXT].phydev)
			/* Power up the PHY(s) */
			amac_gphy_enable(
				privp->port.info[AMAC_PORT_TYPE_EXT].phydev,
				1);
}

/* bcm_amac_gphy_shutdown() - Reset and power down the PHY's
 * @privp: driver local data structure pointer
 */
void bcm_amac_gphy_shutdown(struct bcm_amac_priv *privp)
{
	if (privp->port.count > 1)
		if (privp->port.info[AMAC_PORT_TYPE_EXT].phydev)
			/* Reset and power down all the PHY */
			amac_gphy_enable(
				privp->port.info[AMAC_PORT_TYPE_EXT].phydev,
				0);
}

/* bcm_amac_gphy_stop_phy() - stop the PHY's
 * @privp: driver local data structure pointer
 */
void bcm_amac_gphy_stop_phy(struct bcm_amac_priv *privp)
{
	if (privp->port.count > 1)
		if (privp->port.info[AMAC_PORT_TYPE_EXT].phydev)
			phy_stop(privp->port.info[AMAC_PORT_TYPE_EXT].phydev);
}

/* bcm_amac_gphy_start_phy() - start the PHY's
 * @privp: driver local data structure pointer
 */
void bcm_amac_gphy_start_phy(struct bcm_amac_priv *privp)
{
	if (privp->port.count > 1)
		if (privp->port.info[AMAC_PORT_TYPE_EXT].phydev)
			phy_start(privp->port.info[AMAC_PORT_TYPE_EXT].phydev);
}

