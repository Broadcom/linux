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

#include <linux/ethtool.h>
#include <linux/phy.h>

#include "bcm-amac-enet.h"

#define MOD_VERSION "0.9.0"

struct ethtool_keys {
	const char string[32];
};

/* Derrived from 'struct amac_ethtool_stats'
 * The ordering needs to be in sync with
 * 'struct amac_ethtool_stats'
 */
static const struct ethtool_keys amac_ethtool_keys[] = {
	{"rx_packets"},
	{"tx_packets"},
	{"rx_bytes"},
	{"tx_bytes"},
	{"rx_dropped"},
	{"tx_dropped"},
	{"multicast"},
	{"rx_length_errors"},
	{"rx_fifo_errors"},
	{"tx_fifo_errors"},
};

/* Do not change the names of the following members
 * as they are used in macro expansion to formulate
 * corresponding data routines.
 */
struct amac_ethtool_stats {
	u64 rx_packets;
	u64 tx_packets;
	u64 rx_bytes;
	u64 tx_bytes;
	u64 rx_dropped;
	u64 tx_dropped;
	u64 multicast;
	u64 rx_length_errors;
	u64 rx_fifo_errors;
	u64 tx_fifo_errors;
};

static int amac_ethtool_get_settings(struct net_device *ndev,
				     struct ethtool_cmd *cmd)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (!privp->port.ext_port.phydev)
		return -ENODEV;

	return phy_ethtool_gset(privp->port.ext_port.phydev, cmd);
}

int amac_ethtool_set_settings(struct net_device *ndev,
			      struct ethtool_cmd *cmd)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (!privp->port.ext_port.phydev)
		return -ENODEV;

	return phy_ethtool_sset(privp->port.ext_port.phydev, cmd);
}

static void amac_ethtool_get_drvinfo(struct net_device *ndev,
				     struct ethtool_drvinfo *info)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	strcpy(info->driver, "amac-enet");
	strcpy(info->version, MOD_VERSION);
	strcpy(info->fw_version, "N/A");

	if (privp->port.ext_port.phydev) {
		if (privp->port.ext_port.phydev->mdio.bus)
			strcpy(info->bus_info,
			       privp->port.ext_port.phydev->mdio.bus->name);
	} else {
		strcpy(info->bus_info, "N/A");
	}
}

static int amac_ethtool_nway_reset(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (!privp->port.ext_port.phydev)
		return -ENODEV;

	if (!netif_running(ndev))
		return -EAGAIN;

	return phy_start_aneg(privp->port.ext_port.phydev);
}

static void amac_ethtool_get_strings(struct net_device *ndev,
				     u32 stringset, u8 *buf)
{
	if (stringset == ETH_SS_STATS)
		memcpy(buf, &amac_ethtool_keys,
		       sizeof(amac_ethtool_keys));
}

static int amac_ethtool_get_sset_count(struct net_device *ndev, int sset)
{
	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;

	return ARRAY_SIZE(amac_ethtool_keys);
}

static void amac_ethtool_get_stats(struct net_device *ndev,
				   struct ethtool_stats *estats, u64 *tmp_stats)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	struct amac_ethtool_stats *stats =
		(struct amac_ethtool_stats *)tmp_stats;

	stats->rx_packets = privp->ndev->stats.rx_packets;
	stats->tx_packets = privp->ndev->stats.tx_packets;
	stats->rx_bytes = privp->ndev->stats.rx_bytes;
	stats->tx_bytes = privp->ndev->stats.tx_bytes;
	stats->rx_dropped = privp->ndev->stats.rx_dropped;
	stats->tx_dropped = privp->ndev->stats.tx_dropped;
	stats->multicast = privp->ndev->stats.multicast;
	stats->rx_length_errors = privp->ndev->stats.rx_length_errors;
	stats->rx_fifo_errors = privp->ndev->stats.rx_fifo_errors;
	stats->tx_fifo_errors = privp->ndev->stats.tx_fifo_errors;
}

static void amac_ethtool_get_pauseparam(struct net_device *ndev,
					struct ethtool_pauseparam *epause)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	epause->autoneg = 0;
	epause->rx_pause = 0;
	epause->tx_pause = 0;

	if (privp->port.ext_port.phydev) {
		epause->autoneg = privp->port.ext_port.phydev->autoneg;

		if (privp->port.ext_port.phydev->pause & FLOW_CTRL_RX)
			epause->rx_pause = 1;

		if (privp->port.ext_port.phydev->pause & FLOW_CTRL_TX)
			epause->tx_pause = 1;
	}
}

static const struct ethtool_ops amac_ethtool_ops = {
	.get_settings = amac_ethtool_get_settings,
	.set_settings = amac_ethtool_set_settings,
	.get_drvinfo = amac_ethtool_get_drvinfo,
	.nway_reset = amac_ethtool_nway_reset,
	.get_link = ethtool_op_get_link,
	.get_pauseparam = amac_ethtool_get_pauseparam,
	.get_strings = amac_ethtool_get_strings,
	.get_ethtool_stats = amac_ethtool_get_stats,
	.get_sset_count = amac_ethtool_get_sset_count,
};

void bcm_amac_set_ethtool_ops(struct net_device *ndev)
{
	ndev->ethtool_ops = &amac_ethtool_ops;
}
